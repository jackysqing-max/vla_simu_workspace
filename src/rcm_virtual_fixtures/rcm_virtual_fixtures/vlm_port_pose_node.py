#!/usr/bin/env python3
"""Estimate and lock an RCM port pose from a SAM3 mask and RGB-D data."""

from __future__ import annotations

from collections import deque
import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener


def image_to_mask(msg: Image) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height,
        msg.width,
    )


def image_to_bgr(msg: Image) -> np.ndarray:
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height,
        msg.width,
        3,
    )
    if str(msg.encoding).lower() == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def bgr_to_image(image: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg


def image_to_depth(msg: Image) -> np.ndarray:
    encoding = str(msg.encoding).lower()
    if encoding in {"32fc1", "32fc"}:
        return np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height,
            msg.width,
        )
    if encoding in {"16uc1", "mono16"}:
        depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height,
            msg.width,
        )
        return depth.astype(np.float32) * 0.001
    raise ValueError(f"unsupported depth encoding {msg.encoding!r}")


def transform_to_matrix(transform_msg) -> np.ndarray:
    translation = transform_msg.transform.translation
    rotation = transform_msg.transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def normalize(vector, fallback=(0.0, 0.0, -1.0)) -> np.ndarray:
    vector = np.asarray(vector, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return np.asarray(fallback, dtype=np.float64)
    return vector / norm


def fit_plane_ransac(
    points_world: np.ndarray,
    normal_reference: np.ndarray,
    *,
    distance_threshold_m: float,
    max_tilt_deg: float,
    iterations: int,
):
    if points_world.shape[0] < 12:
        return None

    reference = normalize(normal_reference, fallback=(0.0, 0.0, 1.0))
    minimum_alignment = math.cos(math.radians(max_tilt_deg))
    rng = np.random.default_rng(17)
    best_inliers = None

    for _ in range(max(iterations, 1)):
        indices = rng.choice(points_world.shape[0], 3, replace=False)
        p0, p1, p2 = points_world[indices]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-8:
            continue
        normal /= norm
        if float(np.dot(normal, reference)) < 0.0:
            normal = -normal
        if float(np.dot(normal, reference)) < minimum_alignment:
            continue
        distances = np.abs((points_world - p0) @ normal)
        inliers = distances <= distance_threshold_m
        if best_inliers is None or int(inliers.sum()) > int(best_inliers.sum()):
            best_inliers = inliers

    if best_inliers is None or int(best_inliers.sum()) < 12:
        return None

    inlier_points = points_world[best_inliers]
    centroid = np.mean(inlier_points, axis=0)
    centered = inlier_points - centroid
    _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
    normal = normalize(vh[-1], fallback=reference)
    if float(np.dot(normal, reference)) < 0.0:
        normal = -normal
    distances = np.abs(centered @ normal)
    return {
        "centroid": centroid,
        "normal": normal,
        "count": int(inlier_points.shape[0]),
        "rms": float(np.sqrt(np.mean(distances * distances))),
    }


class VlmPortPoseNode(Node):
    """Extract dynamic RCM port candidates and lock externally verified choices."""

    def __init__(self):
        super().__init__("vlm_port_pose_node")

        self.declare_parameter("mask_topic", "/sam3/mask")
        self.declare_parameter(
            "color_topic",
            "/sim/camera/color/image_raw",
        )
        self.declare_parameter(
            "depth_topic",
            "/sim/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/sim/camera/color/camera_info",
        )
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("prompt_topic", "/sam3/active_prompt")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("min_score", 0.05)
        self.declare_parameter("min_mask_area_px", 40)
        self.declare_parameter("max_mask_area_fraction", 0.35)
        self.declare_parameter("max_mask_components", 48)
        self.declare_parameter("annulus_radius_px", 18)
        self.declare_parameter("hole_search_radius_px", 40)
        self.declare_parameter("hole_min_depth_m", 0.025)
        self.declare_parameter("hole_min_area_px", 80)
        self.declare_parameter("hole_min_depth_area_fraction", 0.08)
        self.declare_parameter("hole_min_component_overlap_fraction", 0.20)
        self.declare_parameter("hole_max_center_offset_px", 24.0)
        self.declare_parameter("hole_min_circularity", 0.28)
        self.declare_parameter("hole_axis_slice_count", 5)
        self.declare_parameter("hole_axis_min_slices", 3)
        self.declare_parameter("hole_axis_slice_min_area_px", 12)
        self.declare_parameter("hole_axis_min_depth_span_m", 0.008)
        self.declare_parameter("hole_axis_max_rms_m", 0.012)
        self.declare_parameter("max_plane_samples", 2500)
        self.declare_parameter("plane_distance_threshold_m", 0.0035)
        self.declare_parameter("plane_max_tilt_deg", 35.0)
        self.declare_parameter("plane_ransac_iterations", 160)
        self.declare_parameter("normal_reference", [0.0, 0.0, 1.0])
        self.declare_parameter("axis_mode", "surface_normal")
        self.declare_parameter(
            "calibrated_inward_axis",
            [0.335067, 0.0, -0.942194],
        )
        self.declare_parameter(
            "expected_port_world",
            [0.701726, 0.0, 0.404701],
        )
        self.declare_parameter("expected_port_max_distance_m", 0.0)
        self.declare_parameter("stable_frames", 3)
        self.declare_parameter("stability_window", 5)
        self.declare_parameter("max_center_spread_m", 0.006)
        self.declare_parameter("max_axis_spread_deg", 5.0)
        self.declare_parameter(
            "raw_port_point_topic",
            "/vlm_rcm/port_point_raw",
        )
        self.declare_parameter(
            "raw_port_axis_topic",
            "/vlm_rcm/port_axis_raw",
        )
        self.declare_parameter(
            "selected_surface_axis_topic",
            "/vlm_rcm/selected_surface_axis",
        )
        self.declare_parameter(
            "locked_port_point_topic",
            "/vlm_rcm/locked_port_point",
        )
        self.declare_parameter(
            "locked_port_axis_topic",
            "/vlm_rcm/locked_port_axis",
        )
        self.declare_parameter(
            "locked_surface_axis_topic",
            "/vlm_rcm/locked_surface_axis",
        )
        self.declare_parameter("axis_latency_topic", "/vlm_rcm/axis_latency")
        self.declare_parameter("valid_topic", "/vlm_rcm/port_valid")
        self.declare_parameter("ready_topic", "/vlm_rcm/port_ready")
        self.declare_parameter("status_topic", "/vlm_rcm/status")
        self.declare_parameter("candidate_topic", "/vlm_rcm/hole_candidates")
        self.declare_parameter("candidate_alias_topic", "/vlm_rcm/candidates")
        self.declare_parameter("language_instruction_topic", "/vlm_rcm/language_command")
        self.declare_parameter("require_language_instruction", True)
        self.declare_parameter(
            "verified_selection_topic",
            "/vlm_rcm/verified_selected_port",
        )
        self.declare_parameter("phantom_surface_min_gap_m", 0.035)
        self.declare_parameter("reset_topic", "/vlm_rcm/reset_lock")
        self.declare_parameter("overlay_topic", "/vlm_rcm/overlay")
        self.declare_parameter("candidate_overlay_topic", "/vlm_rcm/candidate_overlay")
        self.declare_parameter("display_overlay", True)
        self.declare_parameter("display_scale", 1.25)
        self.declare_parameter(
            "display_window_name",
            "VLM RCM Port Detection",
        )
        self.declare_parameter("axis_outside_m", 0.045)
        self.declare_parameter("axis_inside_m", 0.085)

        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.score_topic = str(self.get_parameter("score_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.language_instruction_topic = str(
            self.get_parameter("language_instruction_topic").value
        )
        self.require_language_instruction = bool(
            self.get_parameter("require_language_instruction").value
        )
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.min_mask_area_px = max(
            int(self.get_parameter("min_mask_area_px").value),
            4,
        )
        self.max_mask_area_fraction = float(
            self.get_parameter("max_mask_area_fraction").value
        )
        self.max_mask_components = max(
            int(self.get_parameter("max_mask_components").value),
            1,
        )
        self.annulus_radius_px = max(
            int(self.get_parameter("annulus_radius_px").value),
            2,
        )
        self.hole_search_radius_px = max(
            int(self.get_parameter("hole_search_radius_px").value),
            self.annulus_radius_px + 4,
        )
        self.hole_min_depth_m = max(
            float(self.get_parameter("hole_min_depth_m").value),
            0.005,
        )
        self.hole_min_area_px = max(
            int(self.get_parameter("hole_min_area_px").value),
            8,
        )
        self.hole_min_depth_area_fraction = max(
            float(self.get_parameter("hole_min_depth_area_fraction").value),
            0.0,
        )
        self.hole_min_component_overlap_fraction = max(
            float(
                self.get_parameter(
                    "hole_min_component_overlap_fraction"
                ).value
            ),
            0.0,
        )
        self.hole_max_center_offset_px = max(
            float(self.get_parameter("hole_max_center_offset_px").value),
            0.0,
        )
        self.hole_min_circularity = max(
            float(self.get_parameter("hole_min_circularity").value),
            0.0,
        )
        self.hole_axis_slice_count = max(
            int(self.get_parameter("hole_axis_slice_count").value),
            2,
        )
        self.hole_axis_min_slices = max(
            int(self.get_parameter("hole_axis_min_slices").value),
            2,
        )
        self.hole_axis_slice_min_area_px = max(
            int(self.get_parameter("hole_axis_slice_min_area_px").value),
            4,
        )
        self.hole_axis_min_depth_span_m = max(
            float(self.get_parameter("hole_axis_min_depth_span_m").value),
            0.0,
        )
        self.hole_axis_max_rms_m = max(
            float(self.get_parameter("hole_axis_max_rms_m").value),
            0.001,
        )
        self.max_plane_samples = max(
            int(self.get_parameter("max_plane_samples").value),
            32,
        )
        self.plane_distance_threshold_m = max(
            float(
                self.get_parameter("plane_distance_threshold_m").value
            ),
            0.0005,
        )
        self.plane_max_tilt_deg = float(
            self.get_parameter("plane_max_tilt_deg").value
        )
        self.plane_ransac_iterations = max(
            int(self.get_parameter("plane_ransac_iterations").value),
            20,
        )
        self.normal_reference = normalize(
            self.get_parameter("normal_reference").value,
            fallback=(0.0, 0.0, 1.0),
        )
        self.axis_mode = str(
            self.get_parameter("axis_mode").value
        ).strip().lower()
        if self.axis_mode not in {"surface_normal", "calibrated", "section_centers"}:
            self.get_logger().warning(
                f"unsupported axis_mode={self.axis_mode!r}; "
                "using surface_normal"
            )
            self.axis_mode = "surface_normal"
        self.calibrated_inward_axis = normalize(
            self.get_parameter("calibrated_inward_axis").value,
            fallback=(0.335067, 0.0, -0.942194),
        )
        self.expected_port_world = np.asarray(
            self.get_parameter("expected_port_world").value,
            dtype=np.float64,
        )
        self.expected_port_max_distance_m = max(
            float(
                self.get_parameter("expected_port_max_distance_m").value
            ),
            0.0,
        )
        self.stable_frames = max(
            int(self.get_parameter("stable_frames").value),
            1,
        )
        self.stability_window = max(
            int(self.get_parameter("stability_window").value),
            self.stable_frames,
        )
        self.max_center_spread_m = max(
            float(self.get_parameter("max_center_spread_m").value),
            0.0005,
        )
        self.max_axis_spread_deg = max(
            float(self.get_parameter("max_axis_spread_deg").value),
            0.5,
        )
        self.display_overlay = bool(
            self.get_parameter("display_overlay").value
        )
        self.display_scale = max(
            float(self.get_parameter("display_scale").value),
            0.25,
        )
        self.display_window_name = str(
            self.get_parameter("display_window_name").value
        )
        self.axis_outside_m = max(
            float(self.get_parameter("axis_outside_m").value),
            0.005,
        )
        self.axis_inside_m = max(
            float(self.get_parameter("axis_inside_m").value),
            0.005,
        )
        self.phantom_surface_min_gap_m = max(
            float(self.get_parameter("phantom_surface_min_gap_m").value),
            0.001,
        )
        self.verified_selection_topic = str(
            self.get_parameter("verified_selection_topic").value
        )

        qos_image = QoSProfile(depth=1)
        qos_image.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_image.durability = DurabilityPolicy.VOLATILE
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(Image, self.mask_topic, self.on_mask, qos_image)
        self.create_subscription(
            Image,
            self.color_topic,
            self.on_color,
            qos_image,
        )
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_image)
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_camera_info,
            qos_image,
        )
        self.create_subscription(Float32, self.score_topic, self.on_score, 10)
        self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
        self.create_subscription(
            String,
            self.language_instruction_topic,
            self.on_language_instruction,
            10,
        )
        self.create_subscription(
            String,
            self.verified_selection_topic,
            self.on_verified_selection,
            qos_latched,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("reset_topic").value),
            self.on_reset,
            10,
        )

        self.pub_raw_point = self.create_publisher(
            PointStamped,
            str(self.get_parameter("raw_port_point_topic").value),
            10,
        )
        self.pub_raw_axis = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("raw_port_axis_topic").value),
            10,
        )
        self.pub_selected_surface_axis = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("selected_surface_axis_topic").value),
            10,
        )
        self.pub_locked_point = self.create_publisher(
            PointStamped,
            str(self.get_parameter("locked_port_point_topic").value),
            qos_latched,
        )
        self.pub_locked_axis = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("locked_port_axis_topic").value),
            qos_latched,
        )
        self.pub_locked_surface_axis = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("locked_surface_axis_topic").value),
            qos_latched,
        )
        self.pub_axis_latency = self.create_publisher(
            String,
            str(self.get_parameter("axis_latency_topic").value),
            qos_latched,
        )
        self.pub_valid = self.create_publisher(
            Bool,
            str(self.get_parameter("valid_topic").value),
            10,
        )
        self.pub_ready = self.create_publisher(
            Bool,
            str(self.get_parameter("ready_topic").value),
            qos_latched,
        )
        self.pub_status = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.pub_candidates = self.create_publisher(
            String,
            str(self.get_parameter("candidate_topic").value),
            qos_latched,
        )
        self.pub_candidates_alias = self.create_publisher(
            String,
            str(self.get_parameter("candidate_alias_topic").value),
            qos_latched,
        )
        self.pub_overlay = self.create_publisher(
            Image,
            str(self.get_parameter("overlay_topic").value),
            qos_image,
        )
        self.pub_candidate_overlay = self.create_publisher(
            Image,
            str(self.get_parameter("candidate_overlay_topic").value),
            qos_image,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.latest_depth_header = None
        self.latest_info = None
        self.latest_score = 0.0
        self.current_prompt = ""
        self.pose_samples = deque(maxlen=self.stability_window)
        self.surface_axis_samples = deque(maxlen=self.stability_window)
        self.locked_point = None
        self.locked_axis = None
        self.locked_surface_axis = None
        self.locked_axis_source = ""
        self.last_candidates = []
        self.last_selected_candidate_id = None
        self.last_filter_note = ""
        self.language_detection_armed = not self.require_language_instruction
        self.language_instruction = ""
        self.language_instruction_wall_time_sec = 0.0
        self.verified_candidate_id = None
        self.verified_instruction = ""
        self.verified_reason = ""
        self.verified_decision = "NONE"
        self.verified_request_id = None
        self.verified_instruction_received_wall_time_sec = 0.0
        self.verified_instruction_received_ros_stamp_sec = 0.0
        self.verified_semantic_completed_wall_time_sec = 0.0
        self.verified_semantic_latency_sec = 0.0
        self.axis_latency_reported_request_id = None
        self.verified_candidate_scores = {}
        self.verified_candidate_ranks = {}
        self.candidate_stability = {}
        self.last_status_time = 0.0
        self.create_timer(0.5, self.publish_locked_pose)
        self.publish_ready(False)

        self.get_logger().info(
            "VLM port pose estimator started: "
            f"mask={self.mask_topic}, depth={self.depth_topic}, "
            f"color={self.color_topic}"
        )

    def on_color(self, msg: Image):
        try:
            color = image_to_bgr(msg)
        except (ValueError, cv2.error) as exc:
            self.publish_status(f"invalid color image: {exc}", valid=False)
            return
        with self.lock:
            self.latest_color = color
            self.latest_color_header = msg.header

    def on_depth(self, msg: Image):
        try:
            depth = image_to_depth(msg)
        except ValueError as exc:
            self.publish_status(f"invalid depth: {exc}", valid=False)
            return
        with self.lock:
            self.latest_depth = depth
            self.latest_depth_header = msg.header

    def on_camera_info(self, msg: CameraInfo):
        with self.lock:
            self.latest_info = msg

    def clear_lock_state(
        self,
        *,
        clear_verified_selection: bool = False,
        clear_language_instruction: bool = False,
    ):
        self.pose_samples.clear()
        self.surface_axis_samples.clear()
        self.locked_point = None
        self.locked_axis = None
        self.locked_surface_axis = None
        self.locked_axis_source = ""
        self.last_candidates = []
        self.last_selected_candidate_id = None
        self.last_filter_note = ""
        if clear_language_instruction:
            self.language_detection_armed = not self.require_language_instruction
            self.language_instruction = ""
            self.language_instruction_wall_time_sec = 0.0
        if clear_verified_selection:
            self.verified_candidate_id = None
            self.verified_instruction = ""
            self.verified_reason = ""
            self.verified_decision = "NONE"
            self.verified_request_id = None
            self.verified_instruction_received_wall_time_sec = 0.0
            self.verified_instruction_received_ros_stamp_sec = 0.0
            self.verified_semantic_completed_wall_time_sec = 0.0
            self.verified_semantic_latency_sec = 0.0
            self.axis_latency_reported_request_id = None
            self.verified_candidate_scores = {}
            self.verified_candidate_ranks = {}

    def on_score(self, msg: Float32):
        with self.lock:
            self.latest_score = float(msg.data)

    def on_prompt(self, msg: String):
        with self.lock:
            prompt = str(msg.data or "").strip()
            if prompt != self.current_prompt:
                self.clear_lock_state(clear_verified_selection=True)
                self.publish_ready(False)
            self.current_prompt = prompt

    def on_language_instruction(self, msg: String):
        instruction = str(msg.data or "").strip()
        if not instruction:
            return
        with self.lock:
            self.clear_lock_state(
                clear_verified_selection=True,
                clear_language_instruction=False,
            )
            self.language_detection_armed = True
            self.language_instruction = instruction
            self.language_instruction_wall_time_sec = time.time()
        self.publish_ready(False)
        self.publish_status(
            "language_instruction_armed_detection: "
            f"{instruction}",
            valid=False,
        )

    @staticmethod
    def parse_candidate_id(value):
        text = str(value if value is not None else "").strip()
        if not text:
            return None
        if text[:1].lower() == "h":
            text = text[1:]
        try:
            candidate_id = int(text)
        except Exception:
            return None
        return candidate_id if candidate_id >= 0 else None

    def on_verified_selection(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("verified selection must be a JSON object")
        except Exception as exc:
            self.publish_status(f"verified selection rejected: invalid JSON {exc}", valid=False)
            return

        decision = str(payload.get("decision", "")).strip().upper()
        candidate_id = self.parse_candidate_id(
            payload.get("selected_candidate_id", payload.get("selected_id"))
        )
        score_by_id = {}
        rank_by_id = {}
        for rank, item in enumerate(payload.get("candidate_scores", []), start=1):
            if not isinstance(item, dict):
                continue
            item_id = self.parse_candidate_id(
                item.get("candidate_id", item.get("id"))
            )
            if item_id is None:
                continue
            try:
                score_by_id[item_id] = float(
                    item.get("score", item.get("semantic_score", 0.0))
                )
            except Exception:
                score_by_id[item_id] = 0.0
            try:
                rank_by_id[item_id] = int(item.get("rank", rank))
            except Exception:
                rank_by_id[item_id] = rank

        with self.lock:
            self.clear_lock_state(clear_verified_selection=False)
            self.verified_decision = decision or "UNKNOWN"
            self.verified_reason = str(payload.get("reason", ""))
            self.verified_instruction = str(payload.get("instruction", ""))
            self.verified_request_id = payload.get("request_id", None)
            try:
                self.verified_instruction_received_wall_time_sec = float(
                    payload.get("instruction_received_wall_time_sec", 0.0)
                )
            except Exception:
                self.verified_instruction_received_wall_time_sec = 0.0
            try:
                self.verified_instruction_received_ros_stamp_sec = float(
                    payload.get("instruction_received_ros_stamp_sec", 0.0)
                )
            except Exception:
                self.verified_instruction_received_ros_stamp_sec = 0.0
            try:
                self.verified_semantic_completed_wall_time_sec = float(
                    payload.get("semantic_completed_wall_time_sec", 0.0)
                )
            except Exception:
                self.verified_semantic_completed_wall_time_sec = 0.0
            try:
                self.verified_semantic_latency_sec = float(
                    payload.get("semantic_latency_sec", 0.0)
                )
            except Exception:
                self.verified_semantic_latency_sec = 0.0
            self.axis_latency_reported_request_id = None
            self.verified_candidate_scores = score_by_id
            self.verified_candidate_ranks = rank_by_id
            if decision == "ACCEPT" and candidate_id is not None:
                self.verified_candidate_id = candidate_id
            else:
                self.verified_candidate_id = None
        self.publish_ready(False)
        if decision == "ACCEPT" and candidate_id is not None:
            self.publish_status(
                f"verified selection accepted: H{candidate_id} "
                f"reason={self.verified_reason} "
                f"semantic_latency={self.verified_semantic_latency_sec:.3f}s",
                valid=False,
            )
        else:
            self.publish_status(
                f"verified selection not accepted: decision={decision or 'UNKNOWN'} "
                f"reason={self.verified_reason}",
                valid=False,
            )

    def on_reset(self, msg: Bool):
        if not msg.data:
            return
        with self.lock:
            self.clear_lock_state(
                clear_verified_selection=True,
                clear_language_instruction=True,
            )
        self.publish_ready(False)
        self.publish_status(
            "lock reset; waiting for language instruction"
        )

    def publish_ready(self, ready: bool):
        msg = Bool()
        msg.data = bool(ready)
        self.pub_ready.publish(msg)

    def publish_status(self, text: str, *, valid: bool | None = None):
        now = time.monotonic()
        if valid is not None:
            msg = Bool()
            msg.data = bool(valid)
            self.pub_valid.publish(msg)
        if now - self.last_status_time < 0.25:
            return
        self.last_status_time = now
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def publish_point(self, publisher, point):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = float(point[2])
        publisher.publish(msg)

    def publish_axis(self, publisher, axis):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.vector.x = float(axis[0])
        msg.vector.y = float(axis[1])
        msg.vector.z = float(axis[2])
        publisher.publish(msg)

    def publish_locked_pose(self):
        with self.lock:
            point = (
                None
                if self.locked_point is None
                else self.locked_point.copy()
            )
            axis = (
                None
                if self.locked_axis is None
                else self.locked_axis.copy()
            )
            surface_axis = (
                None
                if self.locked_surface_axis is None
                else self.locked_surface_axis.copy()
            )
            axis_source = self.locked_axis_source
            candidates = list(self.last_candidates)
            selected_candidate_id = self.last_selected_candidate_id
            filter_note = self.last_filter_note
        if point is None or axis is None:
            return
        self.publish_point(self.pub_locked_point, point)
        self.publish_axis(self.pub_locked_axis, axis)
        if surface_axis is not None:
            self.publish_axis(self.pub_locked_surface_axis, surface_axis)
        if candidates:
            self.publish_candidates(
                candidates,
                selected_id=selected_candidate_id,
                filter_note=filter_note,
            )
        self.publish_ready(True)
        surface_text = ""
        if surface_axis is not None:
            alignment = float(np.clip(np.dot(axis, surface_axis), -1.0, 1.0))
            angle_deg = math.degrees(math.acos(alignment))
            surface_text = (
                f" surface_axis=({surface_axis[0]:.3f},"
                f"{surface_axis[1]:.3f},{surface_axis[2]:.3f})"
                f" axis_surface_angle={angle_deg:.1f}deg"
            )
        self.publish_status(
            "locked=true "
            f"verified=H{selected_candidate_id} "
            f"center=({point[0]:.4f},{point[1]:.4f},{point[2]:.4f}) "
            f"axis=({axis[0]:.3f},{axis[1]:.3f},{axis[2]:.3f}) "
            f"axis_source={axis_source}"
            f"{surface_text}",
            valid=True,
        )

    def mask_components(self, mask: np.ndarray, *, sort_by_area: bool = False):
        binary = (mask > 127).astype(np.uint8)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary,
            connectivity=8,
        )
        if count <= 1:
            return []

        image_area = int(mask.shape[0] * mask.shape[1])
        components = []
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < self.min_mask_area_px:
                continue
            if area > self.max_mask_area_fraction * image_area:
                continue
            centroid = np.asarray(centroids[index], dtype=np.float64)
            components.append(
                {
                    "id": len(components),
                    "label": int(index),
                    "component": (labels == index).astype(np.uint8),
                    "centroid": centroid,
                    "area": area,
                    "bbox": (
                        int(stats[index, cv2.CC_STAT_LEFT]),
                        int(stats[index, cv2.CC_STAT_TOP]),
                        int(stats[index, cv2.CC_STAT_WIDTH]),
                        int(stats[index, cv2.CC_STAT_HEIGHT]),
                    ),
                }
            )

        if sort_by_area:
            components.sort(key=lambda item: item["area"], reverse=True)
        else:
            components.sort(
                key=lambda item: (
                    float(item["centroid"][1]),
                    float(item["centroid"][0]),
                )
            )
        for new_id, item in enumerate(components):
            item["id"] = new_id
        return components[: self.max_mask_components]

    def largest_mask_component(self, mask: np.ndarray):
        components = self.mask_components(mask, sort_by_area=True)
        if not components:
            return None
        component = components[0]
        return component["component"], component["centroid"], component["area"]

    def camera_to_world(self, frame_id: str):
        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            frame_id,
            Time(),
        )
        return transform_to_matrix(transform)

    def project_world_point(
        self,
        point_world: np.ndarray,
        info: CameraInfo,
    ):
        camera_to_world = self.camera_to_world(info.header.frame_id)
        world_to_camera = np.linalg.inv(camera_to_world)
        homogeneous = np.append(
            np.asarray(point_world, dtype=np.float64),
            1.0,
        )
        point_camera = world_to_camera @ homogeneous
        if point_camera[2] <= 1e-5:
            return None
        u = float(info.k[0]) * point_camera[0] / point_camera[2] + float(
            info.k[2]
        )
        v = float(info.k[4]) * point_camera[1] / point_camera[2] + float(
            info.k[5]
        )
        return int(round(u)), int(round(v))

    def component_circularity(self, component_mask: np.ndarray) -> float:
        contours, _hierarchy = cv2.findContours(
            component_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return 0.0
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        if area <= 1e-6 or perimeter <= 1e-6:
            return 0.0
        return float(4.0 * math.pi * area / (perimeter * perimeter))

    def component_aperture_center_uv(
        self,
        component_mask: np.ndarray,
        fallback_uv: np.ndarray,
    ):
        contours, _hierarchy = cv2.findContours(
            component_mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return np.asarray(fallback_uv, dtype=np.float64), "component_centroid"

        contour = max(contours, key=cv2.contourArea)
        if len(contour) >= 5:
            try:
                (cx, cy), (_major, _minor), _angle = cv2.fitEllipse(contour)
                center = np.asarray([float(cx), float(cy)], dtype=np.float64)
                if np.all(np.isfinite(center)):
                    return center, "sam_aperture_ellipse"
            except cv2.error:
                pass

        moments = cv2.moments(contour)
        if abs(float(moments["m00"])) > 1e-6:
            center = np.asarray(
                [
                    float(moments["m10"] / moments["m00"]),
                    float(moments["m01"] / moments["m00"]),
                ],
                dtype=np.float64,
            )
            if np.all(np.isfinite(center)):
                return center, "sam_aperture_moments"

        (cx, cy), _radius = cv2.minEnclosingCircle(contour)
        center = np.asarray([float(cx), float(cy)], dtype=np.float64)
        if np.all(np.isfinite(center)):
            return center, "sam_aperture_enclosing_circle"
        return np.asarray(fallback_uv, dtype=np.float64), "component_centroid"

    def fit_section_center_axis(
        self,
        selected_pixels: np.ndarray,
        search_depth_m: np.ndarray,
        search_points_world: np.ndarray,
        inward_surface_normal: np.ndarray,
    ):
        selected_depth = search_depth_m[selected_pixels]
        selected_points = search_points_world[selected_pixels]
        if selected_depth.size < self.hole_axis_slice_min_area_px:
            return None

        depth_low = float(np.percentile(selected_depth, 20.0))
        depth_high = float(np.percentile(selected_depth, 90.0))
        if depth_high - depth_low < self.hole_axis_min_depth_span_m:
            return None

        levels = np.linspace(
            depth_low,
            depth_high,
            self.hole_axis_slice_count,
            dtype=np.float64,
        )
        band = max(
            (depth_high - depth_low) / max(self.hole_axis_slice_count - 1, 1),
            0.002,
        )
        centers = []
        for level in levels:
            in_band = selected_pixels & (np.abs(search_depth_m - level) <= band)
            if int(np.count_nonzero(in_band)) < self.hole_axis_slice_min_area_px:
                in_band = selected_pixels & (search_depth_m >= level)
            if int(np.count_nonzero(in_band)) < self.hole_axis_slice_min_area_px:
                continue
            centers.append(np.mean(search_points_world[in_band], axis=0))

        if len(centers) < self.hole_axis_min_slices:
            return None
        centers = np.asarray(centers, dtype=np.float64)
        center_mean = np.mean(centers, axis=0)
        centered = centers - center_mean
        _u, _s, vh = np.linalg.svd(centered, full_matrices=False)
        axis = normalize(vh[0], fallback=inward_surface_normal)
        if float(np.dot(axis, inward_surface_normal)) < 0.0:
            axis = -axis

        projections = centered @ axis
        closest = center_mean + projections[:, None] * axis[None, :]
        rms = float(
            np.sqrt(np.mean(np.sum((centers - closest) ** 2, axis=1)))
        )
        depth_span = float(np.ptp(projections))
        if (
            depth_span < self.hole_axis_min_depth_span_m
            or rms > self.hole_axis_max_rms_m
        ):
            return None
        alignment = float(
            np.clip(np.dot(axis, inward_surface_normal), -1.0, 1.0)
        )
        return {
            "axis": axis,
            "centers": centers,
            "count": int(centers.shape[0]),
            "rms": rms,
            "depth_span": depth_span,
            "surface_angle_deg": float(math.degrees(math.acos(alignment))),
        }

    def select_recessed_hole_component(
        self,
        *,
        hole_labels: np.ndarray,
        hole_stats: np.ndarray,
        hole_centroids: np.ndarray,
        component: np.ndarray,
        centroid_uv: np.ndarray,
        search_rows: np.ndarray,
        search_cols: np.ndarray,
        search_depth_m: np.ndarray,
        search_points_world: np.ndarray,
        inward_surface_normal: np.ndarray,
    ):
        component_area = max(int(np.count_nonzero(component)), 1)
        best = None
        rejection_notes = []
        for index in range(1, int(hole_stats.shape[0])):
            hole_area = int(hole_stats[index, cv2.CC_STAT_AREA])
            if hole_area < self.hole_min_area_px:
                rejection_notes.append(f"Hdepth{index}:area={hole_area}")
                continue
            hole_mask = hole_labels == index
            centroid = np.asarray(hole_centroids[index], dtype=np.float64)
            center_offset_px = float(np.linalg.norm(centroid - centroid_uv))
            if (
                self.hole_max_center_offset_px > 0.0
                and center_offset_px > self.hole_max_center_offset_px
            ):
                rejection_notes.append(
                    f"Hdepth{index}:offset={center_offset_px:.1f}px"
                )
                continue
            overlap_px = int(np.count_nonzero(hole_mask & (component > 0)))
            overlap_fraction = overlap_px / max(hole_area, 1)
            if overlap_fraction < self.hole_min_component_overlap_fraction:
                rejection_notes.append(
                    f"Hdepth{index}:overlap={overlap_fraction:.2f}"
                )
                continue
            area_fraction = hole_area / component_area
            if area_fraction < self.hole_min_depth_area_fraction:
                rejection_notes.append(
                    f"Hdepth{index}:area_frac={area_fraction:.2f}"
                )
                continue
            circularity = self.component_circularity(hole_mask.astype(np.uint8))
            if circularity < self.hole_min_circularity:
                rejection_notes.append(
                    f"Hdepth{index}:circ={circularity:.2f}"
                )
                continue

            selected_pixels = hole_mask[search_rows, search_cols]
            selected_depth = search_depth_m[selected_pixels]
            if selected_depth.size < self.hole_min_area_px:
                rejection_notes.append(
                    f"Hdepth{index}:valid_depth={selected_depth.size}"
                )
                continue
            median_depth = float(np.median(selected_depth))
            p90_depth = float(np.percentile(selected_depth, 90.0))
            section_fit = self.fit_section_center_axis(
                selected_pixels,
                search_depth_m,
                search_points_world,
                inward_surface_normal,
            )
            score = (
                float(hole_area)
                + 120.0 * overlap_fraction
                + 80.0 * circularity
                - 2.0 * center_offset_px
            )
            if best is None or score > best["score"]:
                best = {
                    "index": int(index),
                    "mask": hole_mask,
                    "area": hole_area,
                    "centroid_uv": centroid,
                    "offset_px": center_offset_px,
                    "overlap_fraction": overlap_fraction,
                    "area_fraction": area_fraction,
                    "circularity": circularity,
                    "median_depth_m": median_depth,
                    "p90_depth_m": p90_depth,
                    "section_fit": section_fit,
                    "score": score,
                }

        if best is None:
            raise ValueError(
                "no recessed aperture passed geometry checks"
                + (
                    ": " + "; ".join(rejection_notes[:4])
                    if rejection_notes
                    else ""
                )
            )
        return best

    def publish_overlay(
        self,
        mask: np.ndarray,
        info: CameraInfo | None,
        *,
        point=None,
        axis=None,
        surface_axis=None,
        candidates=None,
        selected_candidate_id=None,
        status: str,
        locked: bool,
    ):
        with self.lock:
            color = (
                None
                if self.latest_color is None
                else self.latest_color.copy()
            )
            color_header = self.latest_color_header
            prompt = self.current_prompt
            score = self.latest_score
        if color is None:
            color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            color_header = None
        if color.shape[:2] != mask.shape:
            color = cv2.resize(
                color,
                (mask.shape[1], mask.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )

        candidate_by_id = {}
        if candidates is not None:
            candidate_by_id = {int(candidate["id"]): candidate for candidate in candidates}
        for component_info in self.mask_components(mask):
            component = component_info["component"]
            component_id = int(component_info["id"])
            centroid_uv = component_info["centroid"]
            tint = np.zeros_like(color)
            tint[:, :, 1] = 255
            selection = component.astype(bool)
            color[selection] = cv2.addWeighted(
                color[selection],
                0.45,
                tint[selection],
                0.55,
                0.0,
            )
            contours, _hierarchy = cv2.findContours(
                component,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            outline_color = (
                (255, 255, 0)
                if component_id == selected_candidate_id
                else (0, 255, 0)
            )
            cv2.drawContours(color, contours, -1, outline_color, 1)
            center_px = (
                int(round(float(centroid_uv[0]))),
                int(round(float(centroid_uv[1]))),
            )
            cv2.drawMarker(
                color,
                center_px,
                (0, 255, 0),
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=1,
            )
            cv2.circle(color, center_px, 7, outline_color, 1)
            label = f"{component_id}"
            if component_id in candidate_by_id:
                candidate_score = candidate_by_id[component_id]["diagnostics"].get(
                    "selection_score",
                    None,
                )
                if candidate_score is None:
                    label = f"H{component_id}"
                else:
                    label = f"H{component_id} {float(candidate_score):.2f}"
            cv2.putText(
                color,
                label,
                (center_px[0] + 8, center_px[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                outline_color,
                1,
                cv2.LINE_AA,
            )

        if point is not None and axis is not None and info is not None:
            outside = point - self.axis_outside_m * axis
            inside = point + self.axis_inside_m * axis
            outside_px = self.project_world_point(outside, info)
            inside_px = self.project_world_point(inside, info)
            point_px = self.project_world_point(point, info)
            if outside_px is not None and inside_px is not None:
                cv2.arrowedLine(
                    color,
                    outside_px,
                    inside_px,
                    (255, 255, 0),
                    3,
                    tipLength=0.18,
                )
            if point_px is not None:
                cv2.drawMarker(
                    color,
                    point_px,
                    (255, 255, 0),
                    markerType=cv2.MARKER_CROSS,
                    markerSize=24,
                    thickness=2,
                )
                cv2.circle(color, point_px, 10, (255, 255, 0), 2)

        if point is not None and surface_axis is not None and info is not None:
            surface_axis = normalize(surface_axis, fallback=(0.0, 0.0, -1.0))
            outside = point - 0.75 * self.axis_outside_m * surface_axis
            inside = point + 0.75 * self.axis_inside_m * surface_axis
            outside_px = self.project_world_point(outside, info)
            inside_px = self.project_world_point(inside, info)
            point_px = self.project_world_point(point, info)
            if outside_px is not None and inside_px is not None:
                cv2.arrowedLine(
                    color,
                    outside_px,
                    inside_px,
                    (255, 0, 255),
                    2,
                    tipLength=0.20,
                )
            if point_px is not None:
                cv2.circle(color, point_px, 14, (255, 0, 255), 1)

        panel_height = 78
        cv2.rectangle(
            color,
            (0, 0),
            (color.shape[1], panel_height),
            (18, 22, 26),
            thickness=-1,
        )
        state_text = "LOCKED" if locked else "SEARCHING"
        state_color = (70, 220, 90) if locked else (0, 190, 255)
        cv2.putText(
            color,
            f"VLM RCM PORT  |  {state_text}",
            (14, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            state_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            color,
            "cyan=final axis  magenta=surface equivalent axis",
            (300, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            color,
            f'prompt: "{prompt}"  score: {score:.3f}',
            (14, 49),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            color,
            status[:90],
            (14, 69),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (210, 215, 220),
            1,
            cv2.LINE_AA,
        )

        header = color_header
        if header is None:
            header = info.header if info is not None else None
        if header is not None:
            overlay_msg = bgr_to_image(color, header)
            self.pub_overlay.publish(overlay_msg)
            self.pub_candidate_overlay.publish(overlay_msg)

        if self.display_overlay:
            display = color
            if abs(self.display_scale - 1.0) > 1e-6:
                display = cv2.resize(
                    color,
                    None,
                    fx=self.display_scale,
                    fy=self.display_scale,
                    interpolation=cv2.INTER_LINEAR,
                )
            cv2.imshow(self.display_window_name, display)
            cv2.waitKey(1)

    def estimate_component_pose(
        self,
        component: np.ndarray,
        centroid_uv: np.ndarray,
        area: int,
        mask_shape,
        depth: np.ndarray,
        info: CameraInfo,
    ):
        kernel_size = 2 * self.annulus_radius_px + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        dilated = cv2.dilate(component, kernel, iterations=1)
        annulus = (dilated > 0) & (component == 0)
        valid = annulus & np.isfinite(depth) & (depth > 0.03)
        rows, cols = np.nonzero(valid)
        if rows.size < 24:
            raise ValueError(f"not enough valid rim depth: {rows.size}")
        if rows.size > self.max_plane_samples:
            select = np.linspace(
                0,
                rows.size - 1,
                self.max_plane_samples,
                dtype=np.int64,
            )
            rows = rows[select]
            cols = cols[select]

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        z = depth[rows, cols].astype(np.float64)
        points_camera = np.column_stack(
            (
                (cols.astype(np.float64) - cx) * z / fx,
                (rows.astype(np.float64) - cy) * z / fy,
                z,
            )
        )
        camera_to_world = self.camera_to_world(info.header.frame_id)
        points_world = (
            camera_to_world[:3, :3] @ points_camera.T
        ).T + camera_to_world[:3, 3]

        plane = fit_plane_ransac(
            points_world,
            self.normal_reference,
            distance_threshold_m=self.plane_distance_threshold_m,
            max_tilt_deg=self.plane_max_tilt_deg,
            iterations=self.plane_ransac_iterations,
        )
        if plane is None:
            raise ValueError("rim plane RANSAC failed")

        inward_surface_normal = -normalize(
            plane["normal"],
            fallback=(0.0, 0.0, 1.0),
        )
        center_col = float(centroid_uv[0])
        center_row = float(centroid_uv[1])
        grid_rows, grid_cols = np.ogrid[: mask_shape[0], : mask_shape[1]]
        search_roi = (
            (grid_cols - center_col) ** 2
            + (grid_rows - center_row) ** 2
            <= self.hole_search_radius_px**2
        )
        search_valid = (
            search_roi
            & np.isfinite(depth)
            & (depth > 0.03)
        )
        hole_rows, hole_cols = np.nonzero(search_valid)
        hole_depth = depth[hole_rows, hole_cols].astype(np.float64)
        hole_points_camera = np.column_stack(
            (
                (hole_cols.astype(np.float64) - cx) * hole_depth / fx,
                (hole_rows.astype(np.float64) - cy) * hole_depth / fy,
                hole_depth,
            )
        )
        hole_points_world = (
            camera_to_world[:3, :3] @ hole_points_camera.T
        ).T + camera_to_world[:3, 3]
        search_depth_m = (
            (hole_points_world - plane["centroid"])
            @ inward_surface_normal
        )
        behind_surface = search_depth_m >= self.hole_min_depth_m
        depth_hole_mask = np.zeros(mask_shape, dtype=np.uint8)
        depth_hole_mask[
            hole_rows[behind_surface],
            hole_cols[behind_surface],
        ] = 1
        (
            hole_count,
            hole_labels,
            hole_stats,
            hole_centroids,
        ) = cv2.connectedComponentsWithStats(
            depth_hole_mask,
            connectivity=8,
        )
        if hole_count <= 1:
            raise ValueError("no recessed aperture found inside SAM3 ROI")
        recessed_hole = self.select_recessed_hole_component(
            hole_labels=hole_labels,
            hole_stats=hole_stats,
            hole_centroids=hole_centroids,
            component=component,
            centroid_uv=centroid_uv,
            search_rows=hole_rows,
            search_cols=hole_cols,
            search_depth_m=search_depth_m,
            search_points_world=hole_points_world,
            inward_surface_normal=inward_surface_normal,
        )
        hole_area = int(recessed_hole["area"])
        geometric_centroid_uv = np.asarray(recessed_hole["centroid_uv"])
        section_fit = recessed_hole["section_fit"]
        aperture_center_uv, aperture_center_source = self.component_aperture_center_uv(
            component,
            centroid_uv,
        )

        u = float(aperture_center_uv[0])
        v = float(aperture_center_uv[1])
        ray_camera = normalize(
            [(u - cx) / fx, (v - cy) / fy, 1.0],
            fallback=(0.0, 0.0, 1.0),
        )
        ray_world = normalize(
            camera_to_world[:3, :3] @ ray_camera,
            fallback=(0.0, 0.0, -1.0),
        )
        camera_origin = camera_to_world[:3, 3]
        denominator = float(np.dot(plane["normal"], ray_world))
        if abs(denominator) < 1e-5:
            raise ValueError("port center ray is parallel to fitted plane")
        distance = float(
            np.dot(
                plane["normal"],
                plane["centroid"] - camera_origin,
            )
            / denominator
        )
        if distance <= 0.0:
            raise ValueError("port plane intersection is behind camera")
        aperture_center_world = camera_origin + distance * ray_world
        center_world = aperture_center_world

        section_surface_center_world = None
        if section_fit is not None:
            section_axis = section_fit["axis"]
            section_origin = np.mean(section_fit["centers"], axis=0)
            denominator_line = float(np.dot(plane["normal"], section_axis))
            if abs(denominator_line) > 1e-5:
                section_distance = float(
                    np.dot(
                        plane["normal"],
                        plane["centroid"] - section_origin,
                    )
                    / denominator_line
                )
                section_surface_center_world = (
                    section_origin + section_distance * section_axis
                )

        expected_distance = float(
            np.linalg.norm(center_world - self.expected_port_world)
        )
        section_axis = None if section_fit is None else section_fit["axis"]
        if self.axis_mode == "section_centers" and section_axis is not None:
            inward_axis = section_axis.copy()
            axis_source = "rgbd_section_centers"
        elif self.axis_mode == "calibrated":
            inward_axis = self.calibrated_inward_axis.copy()
            axis_source = "calibrated_port_geometry"
        else:
            inward_axis = inward_surface_normal
            axis_source = "rgbd_surface_normal"
        axis_surface_alignment = float(
            np.clip(np.dot(inward_axis, inward_surface_normal), -1.0, 1.0)
        )
        axis_surface_angle_deg = float(
            math.degrees(math.acos(axis_surface_alignment))
        )
        return center_world, inward_axis, {
            "area": area,
            "hole_area": hole_area,
            "plane_count": plane["count"],
            "plane_rms": plane["rms"],
            "plane_centroid": plane["centroid"],
            "plane_normal": plane["normal"],
            "surface_equivalent_axis": inward_surface_normal,
            "surface_axis_source": "rgbd_fitted_surface_plane",
            "section_center_axis": section_axis,
            "section_center_axis_source": (
                "rgbd_depth_section_centers"
                if section_axis is not None
                else "unavailable"
            ),
            "section_center_count": (
                0 if section_fit is None else int(section_fit["count"])
            ),
            "section_center_rms_m": (
                float("nan") if section_fit is None else float(section_fit["rms"])
            ),
            "section_center_depth_span_m": (
                0.0 if section_fit is None else float(section_fit["depth_span"])
            ),
            "section_surface_angle_deg": (
                float("nan")
                if section_fit is None
                else float(section_fit["surface_angle_deg"])
            ),
            "depth_hole_center_offset_px": recessed_hole["offset_px"],
            "depth_hole_overlap_fraction": recessed_hole["overlap_fraction"],
            "depth_hole_area_fraction": recessed_hole["area_fraction"],
            "depth_hole_circularity": recessed_hole["circularity"],
            "depth_hole_median_depth_m": recessed_hole["median_depth_m"],
            "depth_hole_p90_depth_m": recessed_hole["p90_depth_m"],
            "axis_surface_angle_deg": axis_surface_angle_deg,
            "expected_distance": expected_distance,
            "sam_centroid_uv": centroid_uv,
            "geometric_centroid_uv": geometric_centroid_uv,
            "aperture_center_uv": aperture_center_uv,
            "aperture_center_source": aperture_center_source,
            "aperture_center_world": aperture_center_world,
            "section_surface_center_world": section_surface_center_world,
            "depth_centroid_to_aperture_offset_px": float(
                np.linalg.norm(geometric_centroid_uv - aperture_center_uv)
            ),
            "axis_source": axis_source,
        }

    def estimate_pose_candidates(
        self,
        mask: np.ndarray,
        depth: np.ndarray,
        info: CameraInfo,
    ):
        components = self.mask_components(mask)
        if not components:
            raise ValueError("SAM3 mask has no connected port components")

        candidates = []
        rejected = 0
        for component_info in components:
            try:
                point, axis, diagnostics = self.estimate_component_pose(
                    component_info["component"],
                    component_info["centroid"],
                    component_info["area"],
                    mask.shape,
                    depth,
                    info,
                )
            except Exception:
                rejected += 1
                continue
            diagnostics["component_id"] = int(component_info["id"])
            diagnostics["bbox"] = component_info["bbox"]
            candidates.append(
                {
                    "id": int(component_info["id"]),
                    "point": point,
                    "axis": axis,
                    "centroid_uv": component_info["centroid"],
                    "area": int(component_info["area"]),
                    "diagnostics": diagnostics,
                }
            )

        if not candidates:
            raise ValueError(
                "no valid recessed aperture candidates found "
                f"from {len(components)} mask components"
            )
        return candidates, rejected

    def filter_candidates_to_phantom_surface(self, candidates):
        if len(candidates) < 2:
            return candidates, "phantom_surface_filter=single_candidate"

        surface_z = np.asarray(
            [
                float(candidate["diagnostics"]["plane_centroid"][2])
                for candidate in candidates
            ],
            dtype=np.float64,
        )
        order = np.argsort(surface_z)
        sorted_z = surface_z[order]
        gaps = np.diff(sorted_z)
        if gaps.size == 0:
            return candidates, "phantom_surface_filter=single_surface"

        split_gap_index = int(np.argmax(gaps))
        max_gap = float(gaps[split_gap_index])
        if max_gap < self.phantom_surface_min_gap_m:
            return (
                candidates,
                "phantom_surface_filter=no_separate_surface "
                f"gap={max_gap:.3f}m",
            )

        split = split_gap_index + 1
        keep_indices = set(int(index) for index in order[split:])
        filtered = [
            candidate
            for index, candidate in enumerate(candidates)
            if index in keep_indices
        ]
        if not filtered:
            return candidates, "phantom_surface_filter=empty_after_split"

        return (
            filtered,
            "phantom_surface_filter=highest_surface "
            f"raw={len(candidates)} kept={len(filtered)} "
            f"z_low={float(sorted_z[0]):.3f}m "
            f"z_high={float(sorted_z[-1]):.3f}m "
            f"gap={max_gap:.3f}m",
        )

    def annotate_candidate_runtime_fields(self, candidates):
        with self.lock:
            latest_score = float(self.latest_score)
            previous_stability = dict(self.candidate_stability)
        next_stability = {}
        current_ids = set()

        for candidate in candidates:
            candidate_id = int(candidate["id"])
            current_ids.add(candidate_id)
            diagnostics = candidate["diagnostics"]
            circularity = float(diagnostics.get("depth_hole_circularity", 0.0))
            overlap = float(diagnostics.get("depth_hole_overlap_fraction", 0.0))
            plane_rms = float(diagnostics.get("plane_rms", 0.0))
            plane_quality = max(0.0, min(1.0, 1.0 - plane_rms / 0.012))
            section_quality = (
                1.0
                if int(diagnostics.get("section_center_count", 0))
                >= self.hole_axis_min_slices
                else 0.35
            )
            geometry_confidence = max(
                0.0,
                min(
                    1.0,
                    0.35 * circularity
                    + 0.30 * overlap
                    + 0.20 * plane_quality
                    + 0.15 * section_quality,
                ),
            )

            previous = previous_stability.get(candidate_id)
            count = 1
            if previous is not None:
                previous_point = np.asarray(previous.get("point"), dtype=np.float64)
                jump = float(np.linalg.norm(candidate["point"] - previous_point))
                if jump <= max(3.0 * self.max_center_spread_m, 0.015):
                    count = int(previous.get("count", 0)) + 1
            temporal_stability = min(1.0, count / max(float(self.stable_frames), 1.0))
            next_stability[candidate_id] = {
                "point": candidate["point"].copy(),
                "count": count,
            }

            diagnostics["candidate_id"] = f"H{candidate_id}"
            diagnostics["geometric_validity"] = True
            diagnostics["geometry_confidence"] = geometry_confidence
            diagnostics["perception_quality"] = max(
                0.0,
                min(1.0, 0.45 * latest_score + 0.55 * geometry_confidence),
            )
            diagnostics["temporal_stability"] = temporal_stability
            diagnostics["visibility"] = 1.0
            diagnostics["depth_validity"] = 1.0
            diagnostics["rcm_feasibility"] = 1.0
            diagnostics["rcm_feasible"] = True
            diagnostics["rcm_feasibility_source"] = "not_evaluated"
            diagnostics["ik_margin"] = 0.0
            diagnostics["ik_feasibility_source"] = "not_evaluated"
            diagnostics["collision_feasibility"] = 1.0
            diagnostics["collision_feasible"] = True
            diagnostics["collision_feasibility_source"] = "not_evaluated"
            diagnostics["semantic_risk"] = 0.0
            diagnostics["unknown_space_exposure"] = 0.0
            diagnostics["overall_acceptance_status"] = "candidate_only"

        with self.lock:
            self.candidate_stability = {
                candidate_id: state
                for candidate_id, state in next_stability.items()
                if candidate_id in current_ids
            }

    def annotate_verified_selection_scores(self, candidates):
        with self.lock:
            selected_id = self.verified_candidate_id
            score_by_id = dict(self.verified_candidate_scores)
            rank_by_id = dict(self.verified_candidate_ranks)
            decision = self.verified_decision
        for candidate in candidates:
            candidate_id = int(candidate["id"])
            diagnostics = candidate["diagnostics"]
            score = score_by_id.get(candidate_id)
            diagnostics["selection_score"] = 0.0 if score is None else float(score)
            diagnostics["selection_score_raw"] = None if score is None else float(score)
            diagnostics["selection_rank"] = int(rank_by_id.get(candidate_id, 0))
            diagnostics["selection_score_source"] = (
                "semantic_grounder"
                if score is not None
                else "unscored_candidate"
            )
            if decision == "ACCEPT" and candidate_id == selected_id:
                diagnostics["overall_acceptance_status"] = "verified_selected"

    def select_verified_candidate(self, candidates):
        with self.lock:
            selected_id = self.verified_candidate_id
        if selected_id is None:
            return None
        for candidate in candidates:
            if int(candidate["id"]) == int(selected_id):
                return candidate
        return None

    def publish_candidates(
        self,
        candidates,
        *,
        raw_count=None,
        filter_note="",
        selected_id=None,
        image_shape=None,
    ):
        with self.lock:
            verified_candidate_id = self.verified_candidate_id
            verified_decision = self.verified_decision
            verified_reason = self.verified_reason
            verified_instruction = self.verified_instruction
            language_detection_armed = self.language_detection_armed
            language_instruction = self.language_instruction
        image_size = None
        if image_shape is not None and len(image_shape) >= 2:
            image_size = [int(image_shape[1]), int(image_shape[0])]
        payload = {
            "schema_version": "dynamic_rcm_port_candidates.v2",
            "frame_id": self.target_frame,
            "stamp_sec": self.get_clock().now().nanoseconds * 1e-9,
            "raw_count": len(candidates) if raw_count is None else int(raw_count),
            "count": len(candidates),
            "filter": filter_note,
            "language_triggered": bool(language_detection_armed),
            "language_instruction": language_instruction,
            "selected_id": None if selected_id is None else int(selected_id),
            "image_size": image_size,
            "verified_selection": {
                "decision": verified_decision,
                "candidate_id": (
                    None
                    if verified_candidate_id is None
                    else f"H{int(verified_candidate_id)}"
                ),
                "selected_id": verified_candidate_id,
                "reason": verified_reason,
                "instruction": verified_instruction,
            },
            "holes": [],
        }
        for candidate in candidates:
            diagnostics = candidate["diagnostics"]
            plane_centroid = diagnostics["plane_centroid"]
            plane_normal = diagnostics["plane_normal"]
            surface_axis = diagnostics.get(
                "surface_equivalent_axis",
                -plane_normal,
            )
            section_axis = diagnostics.get("section_center_axis", None)
            section_surface_center = diagnostics.get(
                "section_surface_center_world",
                None,
            )
            selection_score_raw = diagnostics.get("selection_score_raw", None)
            if selection_score_raw is not None:
                selection_score_raw = float(selection_score_raw)
                if not math.isfinite(selection_score_raw):
                    selection_score_raw = None
            def finite_or_none(value):
                try:
                    value = float(value)
                except Exception:
                    return None
                return value if math.isfinite(value) else None

            payload["holes"].append(
                {
                    "id": int(candidate["id"]),
                    "candidate_id": f"H{int(candidate['id'])}",
                    "center_px": [
                        float(candidate["centroid_uv"][0]),
                        float(candidate["centroid_uv"][1]),
                    ],
                    "geometric_center_px": [
                        float(diagnostics["geometric_centroid_uv"][0]),
                        float(diagnostics["geometric_centroid_uv"][1]),
                    ],
                    "aperture_center_px": [
                        float(diagnostics["aperture_center_uv"][0]),
                        float(diagnostics["aperture_center_uv"][1]),
                    ],
                    "aperture_center_source": str(
                        diagnostics.get(
                            "aperture_center_source",
                            "unknown",
                        )
                    ),
                    "center_world": [
                        float(candidate["point"][0]),
                        float(candidate["point"][1]),
                        float(candidate["point"][2]),
                    ],
                    "aperture_center_world": [
                        float(diagnostics["aperture_center_world"][0]),
                        float(diagnostics["aperture_center_world"][1]),
                        float(diagnostics["aperture_center_world"][2]),
                    ],
                    "section_surface_center_world": (
                        None
                        if section_surface_center is None
                        else [
                            float(section_surface_center[0]),
                            float(section_surface_center[1]),
                            float(section_surface_center[2]),
                        ]
                    ),
                    "axis": [
                        float(candidate["axis"][0]),
                        float(candidate["axis"][1]),
                        float(candidate["axis"][2]),
                    ],
                    "support_center_world": [
                        float(plane_centroid[0]),
                        float(plane_centroid[1]),
                        float(plane_centroid[2]),
                    ],
                    "support_normal": [
                        float(plane_normal[0]),
                        float(plane_normal[1]),
                        float(plane_normal[2]),
                    ],
                    "surface_equivalent_axis": [
                        float(surface_axis[0]),
                        float(surface_axis[1]),
                        float(surface_axis[2]),
                    ],
                    "section_center_axis": (
                        None
                        if section_axis is None
                        else [
                            float(section_axis[0]),
                            float(section_axis[1]),
                            float(section_axis[2]),
                        ]
                    ),
                    "section_center_axis_source": str(
                        diagnostics.get(
                            "section_center_axis_source",
                            "unavailable",
                        )
                    ),
                    "section_center_count": int(
                        diagnostics.get("section_center_count", 0)
                    ),
                    "section_center_rms_m": finite_or_none(
                        diagnostics.get("section_center_rms_m", None)
                    ),
                    "section_center_depth_span_m": finite_or_none(
                        diagnostics.get("section_center_depth_span_m", None)
                    ),
                    "section_surface_angle_deg": finite_or_none(
                        diagnostics.get("section_surface_angle_deg", None)
                    ),
                    "surface_axis_source": str(
                        diagnostics.get(
                            "surface_axis_source",
                            "rgbd_fitted_surface_plane",
                        )
                    ),
                    "axis_surface_angle_deg": float(
                        diagnostics.get("axis_surface_angle_deg", 0.0)
                    ),
                    "support_z_m": float(plane_centroid[2]),
                    "area_px": int(candidate["area"]),
                    "hole_area_px": int(diagnostics["hole_area"]),
                    "depth_hole_center_offset_px": float(
                        diagnostics.get("depth_hole_center_offset_px", 0.0)
                    ),
                    "depth_hole_overlap_fraction": float(
                        diagnostics.get("depth_hole_overlap_fraction", 0.0)
                    ),
                    "depth_hole_area_fraction": float(
                        diagnostics.get("depth_hole_area_fraction", 0.0)
                    ),
                    "depth_hole_circularity": float(
                        diagnostics.get("depth_hole_circularity", 0.0)
                    ),
                    "depth_hole_median_depth_m": float(
                        diagnostics.get("depth_hole_median_depth_m", 0.0)
                    ),
                    "depth_hole_p90_depth_m": float(
                        diagnostics.get("depth_hole_p90_depth_m", 0.0)
                    ),
                    "depth_centroid_to_aperture_offset_px": float(
                        diagnostics.get(
                            "depth_centroid_to_aperture_offset_px",
                            0.0,
                        )
                    ),
                    "expected_distance_m": float(
                        diagnostics["expected_distance"]
                    ),
                    "axis_source": str(diagnostics["axis_source"]),
                    "geometry_confidence": float(
                        diagnostics.get("geometry_confidence", 0.0)
                    ),
                    "perception_quality": float(
                        diagnostics.get("perception_quality", 0.0)
                    ),
                    "geometric_validity": bool(
                        diagnostics.get("geometric_validity", True)
                    ),
                    "temporal_stability": float(
                        diagnostics.get("temporal_stability", 0.0)
                    ),
                    "visibility": float(diagnostics.get("visibility", 1.0)),
                    "depth_validity": float(
                        diagnostics.get("depth_validity", 1.0)
                    ),
                    "rcm_feasibility": float(
                        diagnostics.get("rcm_feasibility", 1.0)
                    ),
                    "rcm_feasible": bool(
                        diagnostics.get("rcm_feasible", True)
                    ),
                    "rcm_feasibility_source": str(
                        diagnostics.get("rcm_feasibility_source", "not_evaluated")
                    ),
                    "ik_margin": float(diagnostics.get("ik_margin", 0.0)),
                    "ik_feasibility_source": str(
                        diagnostics.get("ik_feasibility_source", "not_evaluated")
                    ),
                    "collision_feasibility": float(
                        diagnostics.get("collision_feasibility", 1.0)
                    ),
                    "collision_feasible": bool(
                        diagnostics.get("collision_feasible", True)
                    ),
                    "collision_feasibility_source": str(
                        diagnostics.get(
                            "collision_feasibility_source",
                            "not_evaluated",
                        )
                    ),
                    "semantic_risk": float(
                        diagnostics.get("semantic_risk", 0.0)
                    ),
                    "unknown_space_exposure": float(
                        diagnostics.get("unknown_space_exposure", 0.0)
                    ),
                    "overall_acceptance_status": str(
                        diagnostics.get(
                            "overall_acceptance_status",
                            "candidate_only",
                        )
                    ),
                    "selection_score": float(
                        diagnostics.get("selection_score", 0.0)
                    ),
                    "selection_score_raw": selection_score_raw,
                    "selection_rank": int(
                        diagnostics.get("selection_rank", 0)
                    ),
                    "selection_score_source": str(
                        diagnostics.get("selection_score_source", "none")
                    ),
                }
            )

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.pub_candidates.publish(msg)
        self.pub_candidates_alias.publish(msg)

    def publish_axis_latency_record(self, point, axis, surface_axis, diagnostics):
        axis_acquired_wall_time_sec = time.time()
        axis_acquired_ros_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        request_id = self.verified_request_id
        already_reported = (
            request_id is not None
            and self.axis_latency_reported_request_id == request_id
        )
        if already_reported:
            return

        instruction_received_wall_time_sec = (
            self.verified_instruction_received_wall_time_sec
        )
        semantic_completed_wall_time_sec = (
            self.verified_semantic_completed_wall_time_sec
        )
        axis_latency_sec = None
        verification_to_axis_latency_sec = None
        if instruction_received_wall_time_sec > 0.0:
            axis_latency_sec = max(
                0.0,
                axis_acquired_wall_time_sec
                - instruction_received_wall_time_sec,
            )
        if semantic_completed_wall_time_sec > 0.0:
            verification_to_axis_latency_sec = max(
                0.0,
                axis_acquired_wall_time_sec
                - semantic_completed_wall_time_sec,
            )

        payload = {
            "schema_version": "vlm_rcm_axis_latency.v1",
            "request_id": request_id,
            "instruction": self.verified_instruction,
            "selected_candidate_id": (
                None
                if self.verified_candidate_id is None
                else f"H{int(self.verified_candidate_id)}"
            ),
            "selected_id": self.verified_candidate_id,
            "verification_decision": self.verified_decision,
            "verification_reason": self.verified_reason,
            "instruction_received_wall_time_sec": (
                instruction_received_wall_time_sec
            ),
            "instruction_received_ros_stamp_sec": (
                self.verified_instruction_received_ros_stamp_sec
            ),
            "semantic_completed_wall_time_sec": semantic_completed_wall_time_sec,
            "semantic_latency_sec": self.verified_semantic_latency_sec,
            "axis_acquired_wall_time_sec": axis_acquired_wall_time_sec,
            "axis_acquired_ros_stamp_sec": axis_acquired_ros_stamp_sec,
            "axis_latency_sec": axis_latency_sec,
            "verification_to_axis_latency_sec": verification_to_axis_latency_sec,
            "stable_samples": len(self.pose_samples),
            "stable_frames_required": self.stable_frames,
            "axis_source": str(diagnostics.get("axis_source", "")),
            "surface_axis_source": str(
                diagnostics.get("surface_axis_source", "")
            ),
            "point_world": [
                float(point[0]),
                float(point[1]),
                float(point[2]),
            ],
            "axis_world": [
                float(axis[0]),
                float(axis[1]),
                float(axis[2]),
            ],
            "surface_axis_world": [
                float(surface_axis[0]),
                float(surface_axis[1]),
                float(surface_axis[2]),
            ],
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.pub_axis_latency.publish(msg)
        self.axis_latency_reported_request_id = request_id

        latency_text = (
            "unknown"
            if axis_latency_sec is None
            else f"{axis_latency_sec:.3f}s"
        )
        verification_to_axis_text = (
            "unknown"
            if verification_to_axis_latency_sec is None
            else f"{verification_to_axis_latency_sec:.3f}s"
        )
        self.get_logger().info(
            "[AXIS_LATENCY] "
            f"request_id={request_id} "
            f"candidate=H{diagnostics.get('selected_id', '?')} "
            f"instruction_to_axis={latency_text} "
            f"semantic={self.verified_semantic_latency_sec:.3f}s "
            f"verification_to_axis={verification_to_axis_text}"
        )

    def update_lock(self, point, axis, diagnostics):
        surface_axis = normalize(
            diagnostics.get("surface_equivalent_axis", axis),
            fallback=axis,
        )
        self.pose_samples.append((point.copy(), axis.copy()))
        self.surface_axis_samples.append(surface_axis.copy())
        points = np.asarray([sample[0] for sample in self.pose_samples])
        axes = np.asarray([sample[1] for sample in self.pose_samples])
        surface_axes = np.asarray(
            [sample for sample in self.surface_axis_samples],
            dtype=np.float64,
        )
        center = np.median(points, axis=0)
        mean_axis = normalize(np.mean(axes, axis=0))
        mean_surface_axis = normalize(
            np.mean(surface_axes, axis=0),
            fallback=surface_axis,
        )
        center_spread = float(
            np.max(np.linalg.norm(points - center[None, :], axis=1))
        )
        dots = np.clip(axes @ mean_axis, -1.0, 1.0)
        axis_spread_deg = float(np.degrees(np.max(np.arccos(dots))))

        self.publish_point(self.pub_raw_point, point)
        self.publish_axis(self.pub_raw_axis, axis)
        self.publish_axis(self.pub_selected_surface_axis, surface_axis)
        stable = (
            len(self.pose_samples) >= self.stable_frames
            and center_spread <= self.max_center_spread_m
            and axis_spread_deg <= self.max_axis_spread_deg
        )
        if stable and self.locked_point is None:
            self.locked_point = center.copy()
            self.locked_axis = mean_axis.copy()
            self.locked_surface_axis = mean_surface_axis.copy()
            self.locked_axis_source = diagnostics["axis_source"]
            self.publish_locked_pose()
            self.publish_axis_latency_record(
                center,
                mean_axis,
                mean_surface_axis,
                diagnostics,
            )
            self.get_logger().info(
                "[PORT_LOCKED] "
                f"point=({center[0]:.4f},{center[1]:.4f},{center[2]:.4f}) "
                f"axis=({mean_axis[0]:.3f},{mean_axis[1]:.3f},"
                f"{mean_axis[2]:.3f})"
            )

        axis_latency_sec = None
        if self.verified_instruction_received_wall_time_sec > 0.0:
            axis_latency_sec = max(
                0.0,
                time.time() - self.verified_instruction_received_wall_time_sec,
            )
        axis_latency_text = (
            "unknown"
            if axis_latency_sec is None
            else f"{axis_latency_sec:.3f}s"
        )
        self.publish_status(
            "source=sam3+rgbd "
            f"prompt={self.current_prompt!r} score={self.latest_score:.3f} "
            f"candidate=H{diagnostics.get('selected_id', '?')} "
            f"raw_candidates={diagnostics.get('raw_candidate_count', '?')} "
            f"candidates={diagnostics.get('candidate_count', '?')} "
            f"rejected={diagnostics.get('rejected_count', '?')} "
            f"{diagnostics.get('filter_note', '')} "
            f"verified_decision={self.verified_decision} "
            f"verified_reason={self.verified_reason} "
            f"semantic_latency={self.verified_semantic_latency_sec:.3f}s "
            f"axis_latency={axis_latency_text} "
            f"score={diagnostics.get('selection_score', 0.0):.3f} "
            f"rank={diagnostics.get('selection_rank', 0)} "
            f"geom_offset={diagnostics.get('depth_hole_center_offset_px', 0.0):.1f}px "
            f"ap_offset={diagnostics.get('depth_centroid_to_aperture_offset_px', 0.0):.1f}px "
            f"overlap={diagnostics.get('depth_hole_overlap_fraction', 0.0):.2f} "
            f"circ={diagnostics.get('depth_hole_circularity', 0.0):.2f} "
            f"depth={diagnostics.get('depth_hole_median_depth_m', 0.0):.3f}m "
            f"sections={diagnostics.get('section_center_count', 0)} "
            f"surface_axis=({surface_axis[0]:.3f},{surface_axis[1]:.3f},"
            f"{surface_axis[2]:.3f}) "
            f"axis_surface_angle="
            f"{diagnostics.get('axis_surface_angle_deg', 0.0):.1f}deg "
            f"mask={diagnostics['area']}px hole={diagnostics['hole_area']}px "
            f"plane={diagnostics['plane_count']} "
            f"rms={diagnostics['plane_rms']:.4f}m "
            f"axis_source={diagnostics['axis_source']} "
            f"samples={len(self.pose_samples)}/{self.stable_frames} "
            f"spread={center_spread:.4f}m axis_spread={axis_spread_deg:.2f}deg "
            f"locked={str(self.locked_point is not None).lower()}",
            valid=True,
        )

    def on_mask(self, msg: Image):
        mask = image_to_mask(msg)
        with self.lock:
            locked_point = (
                None
                if self.locked_point is None
                else self.locked_point.copy()
            )
            locked_axis = (
                None
                if self.locked_axis is None
                else self.locked_axis.copy()
            )
            locked_surface_axis = (
                None
                if self.locked_surface_axis is None
                else self.locked_surface_axis.copy()
            )
            last_candidates = list(self.last_candidates)
            last_selected_candidate_id = self.last_selected_candidate_id
            locked_axis_source = self.locked_axis_source
            language_detection_armed = self.language_detection_armed
            language_instruction = self.language_instruction
            depth = (
                None
                if self.latest_depth is None
                else self.latest_depth.copy()
            )
            info = self.latest_info
            score = self.latest_score
        if locked_point is not None and locked_axis is not None:
            self.publish_overlay(
                mask,
                info,
                point=locked_point,
                axis=locked_axis,
                surface_axis=locked_surface_axis,
                candidates=last_candidates,
                selected_candidate_id=last_selected_candidate_id,
                status=(
                    "center: SAM3-guided RGB-D  |  "
                    f"axis: {locked_axis_source}"
                ),
                locked=True,
            )
            return
        if self.require_language_instruction and not language_detection_armed:
            self.pose_samples.clear()
            self.surface_axis_samples.clear()
            self.publish_ready(False)
            self.publish_status(
                "idle_waiting_for_language_instruction; "
                "candidate estimation is disabled before language command",
                valid=False,
            )
            self.publish_overlay(
                mask,
                info,
                status=(
                    "idle: send language instruction to start target "
                    "port detection"
                ),
                locked=False,
            )
            return
        if depth is None or info is None:
            self.publish_status("waiting for synchronized RGB-D", valid=False)
            self.publish_overlay(
                mask,
                info,
                status="waiting for aligned depth and camera intrinsics",
                locked=False,
            )
            return
        if score < self.min_score:
            self.publish_status(
                f"SAM3 score {score:.3f} below {self.min_score:.3f}",
                valid=False,
            )
            self.publish_overlay(
                mask,
                info,
                status=(
                    f"SAM3 score {score:.3f} below "
                    f"{self.min_score:.3f}"
                ),
                locked=False,
            )
            return

        if mask.shape != depth.shape:
            self.publish_status(
                f"mask/depth shape mismatch {mask.shape} != {depth.shape}",
                valid=False,
            )
            self.publish_overlay(
                mask,
                info,
                status="mask/depth resolution mismatch",
                locked=False,
            )
            return
        candidates = []
        selected_candidate_id = None
        try:
            candidates, rejected = self.estimate_pose_candidates(mask, depth, info)
            raw_candidate_count = len(candidates)
            filter_note = ""
            if self.expected_port_max_distance_m > 0.0:
                before_count = len(candidates)
                candidates = [
                    candidate
                    for candidate in candidates
                    if candidate["diagnostics"]["expected_distance"]
                    <= self.expected_port_max_distance_m
                ]
                filter_note = (
                    "workspace_gate "
                    f"raw={before_count} kept={len(candidates)} "
                    f"radius={self.expected_port_max_distance_m:.3f}m"
                )
                if not candidates:
                    raise ValueError("no candidate inside configured workspace gate")
            self.annotate_candidate_runtime_fields(candidates)
            self.annotate_verified_selection_scores(candidates)
            selected = self.select_verified_candidate(candidates)
            if selected is not None:
                selected_candidate_id = int(selected["id"])
            with self.lock:
                self.last_candidates = list(candidates)
                self.last_selected_candidate_id = selected_candidate_id
                self.last_filter_note = filter_note
            self.publish_candidates(
                candidates,
                raw_count=raw_candidate_count,
                filter_note=filter_note,
                selected_id=selected_candidate_id,
                image_shape=mask.shape,
            )
            if selected is None:
                with self.lock:
                    requested_id = self.verified_candidate_id
                    decision = self.verified_decision
                    reason = self.verified_reason
                if requested_id is None:
                    status = (
                        f"instruction={language_instruction!r} "
                        f"candidates={len(candidates)} "
                        "waiting_for_verified_selection"
                    )
                else:
                    status = (
                        f"verified H{requested_id} not visible; "
                        f"candidates={len(candidates)} decision={decision} "
                        f"reason={reason}"
                    )
                self.pose_samples.clear()
                self.surface_axis_samples.clear()
                self.publish_ready(False)
                self.publish_status(status, valid=False)
                self.publish_overlay(
                    mask,
                    info,
                    candidates=candidates,
                    selected_candidate_id=selected_candidate_id,
                    status=status,
                    locked=False,
                )
                return
            point = selected["point"]
            axis = selected["axis"]
            diagnostics = dict(selected["diagnostics"])
            surface_axis = normalize(
                diagnostics.get("surface_equivalent_axis", axis),
                fallback=axis,
            )
            diagnostics["candidate_count"] = len(candidates)
            diagnostics["raw_candidate_count"] = raw_candidate_count
            diagnostics["rejected_count"] = rejected
            diagnostics["selected_id"] = selected_candidate_id
            diagnostics["filter_note"] = filter_note
        except Exception as exc:
            self.pose_samples.clear()
            self.publish_status(f"port estimate rejected: {exc}", valid=False)
            self.publish_overlay(
                mask,
                info,
                candidates=candidates,
                selected_candidate_id=selected_candidate_id,
                status=f"estimate rejected: {exc}",
                locked=False,
            )
            return
        self.update_lock(point, axis, diagnostics)
        with self.lock:
            is_locked = self.locked_point is not None
            overlay_point = (
                self.locked_point.copy()
                if is_locked
                else point.copy()
            )
            overlay_axis = (
                self.locked_axis.copy()
                if is_locked
                else axis.copy()
            )
            overlay_surface_axis = (
                self.locked_surface_axis.copy()
                if is_locked and self.locked_surface_axis is not None
                else surface_axis.copy()
            )
        self.publish_overlay(
            mask,
            info,
            point=overlay_point,
            axis=overlay_axis,
            surface_axis=overlay_surface_axis,
            candidates=candidates,
            selected_candidate_id=selected_candidate_id,
            status=(
                f"center=({overlay_point[0]:.3f},"
                f"{overlay_point[1]:.3f},{overlay_point[2]:.3f}) m  "
                f"H{selected_candidate_id} "
                f"candidates={len(candidates)} "
                f"{diagnostics.get('filter_note', '')} "
                f"axis={diagnostics['axis_source']}"
            ),
            locked=is_locked,
        )

    def destroy_node(self):
        if self.display_overlay:
            try:
                cv2.destroyWindow(self.display_window_name)
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VlmPortPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
