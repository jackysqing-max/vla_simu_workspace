#!/usr/bin/env python3
"""Estimate and lock an RCM port pose from a SAM3 mask and RGB-D data."""

from __future__ import annotations

from collections import deque
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
    """Convert a text-grounded port mask into a stable world-frame port pose."""

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
        self.declare_parameter("annulus_radius_px", 18)
        self.declare_parameter("hole_search_radius_px", 40)
        self.declare_parameter("hole_min_depth_m", 0.025)
        self.declare_parameter("hole_min_area_px", 80)
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
        self.declare_parameter("expected_port_max_distance_m", 0.080)
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
            "locked_port_point_topic",
            "/vlm_rcm/locked_port_point",
        )
        self.declare_parameter(
            "locked_port_axis_topic",
            "/vlm_rcm/locked_port_axis",
        )
        self.declare_parameter("valid_topic", "/vlm_rcm/port_valid")
        self.declare_parameter("ready_topic", "/vlm_rcm/port_ready")
        self.declare_parameter("status_topic", "/vlm_rcm/status")
        self.declare_parameter("reset_topic", "/vlm_rcm/reset_lock")
        self.declare_parameter("overlay_topic", "/vlm_rcm/overlay")
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
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.min_mask_area_px = max(
            int(self.get_parameter("min_mask_area_px").value),
            4,
        )
        self.max_mask_area_fraction = float(
            self.get_parameter("max_mask_area_fraction").value
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
        if self.axis_mode not in {"surface_normal", "calibrated"}:
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
        self.pub_overlay = self.create_publisher(
            Image,
            str(self.get_parameter("overlay_topic").value),
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
        self.locked_point = None
        self.locked_axis = None
        self.locked_axis_source = ""
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

    def on_score(self, msg: Float32):
        with self.lock:
            self.latest_score = float(msg.data)

    def on_prompt(self, msg: String):
        with self.lock:
            prompt = str(msg.data or "").strip()
            if prompt != self.current_prompt and self.locked_point is None:
                self.pose_samples.clear()
            self.current_prompt = prompt

    def on_reset(self, msg: Bool):
        if not msg.data:
            return
        with self.lock:
            self.pose_samples.clear()
            self.locked_point = None
            self.locked_axis = None
            self.locked_axis_source = ""
        self.publish_ready(False)
        self.publish_status("lock reset; waiting for SAM3 port observations")

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
            axis_source = self.locked_axis_source
        if point is None or axis is None:
            return
        self.publish_point(self.pub_locked_point, point)
        self.publish_axis(self.pub_locked_axis, axis)
        self.publish_ready(True)
        self.publish_status(
            "locked=true "
            f"center=({point[0]:.4f},{point[1]:.4f},{point[2]:.4f}) "
            f"axis=({axis[0]:.3f},{axis[1]:.3f},{axis[2]:.3f}) "
            f"axis_source={axis_source}",
            valid=True,
        )

    def largest_mask_component(self, mask: np.ndarray):
        binary = (mask > 127).astype(np.uint8)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary,
            connectivity=8,
        )
        if count <= 1:
            return None
        index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = int(stats[index, cv2.CC_STAT_AREA])
        component = (labels == index).astype(np.uint8)
        centroid = np.asarray(centroids[index], dtype=np.float64)
        return component, centroid, area

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

    def publish_overlay(
        self,
        mask: np.ndarray,
        info: CameraInfo | None,
        *,
        point=None,
        axis=None,
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

        component_result = self.largest_mask_component(mask)
        if component_result is not None:
            component, centroid_uv, _area = component_result
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
            cv2.circle(color, center_px, 7, (0, 255, 0), 1)

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
            self.pub_overlay.publish(bgr_to_image(color, header))

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

    def estimate_pose(self, mask: np.ndarray, depth: np.ndarray, info: CameraInfo):
        component_result = self.largest_mask_component(mask)
        if component_result is None:
            raise ValueError("SAM3 mask has no connected port component")
        component, centroid_uv, area = component_result
        image_area = int(mask.shape[0] * mask.shape[1])
        if area < self.min_mask_area_px:
            raise ValueError(f"port mask too small: {area}px")
        if area > self.max_mask_area_fraction * image_area:
            raise ValueError(f"port mask too large: {area}px")

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
        grid_rows, grid_cols = np.ogrid[: mask.shape[0], : mask.shape[1]]
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
        behind_surface = (
            (hole_points_world - plane["centroid"])
            @ inward_surface_normal
        ) >= self.hole_min_depth_m
        depth_hole_mask = np.zeros(mask.shape, dtype=np.uint8)
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
        hole_index = 1 + int(
            np.argmax(hole_stats[1:, cv2.CC_STAT_AREA])
        )
        hole_area = int(hole_stats[hole_index, cv2.CC_STAT_AREA])
        if hole_area < self.hole_min_area_px:
            raise ValueError(
                f"recessed aperture too small: {hole_area}px"
            )
        geometric_centroid_uv = np.asarray(
            hole_centroids[hole_index],
            dtype=np.float64,
        )

        u = float(geometric_centroid_uv[0])
        v = float(geometric_centroid_uv[1])
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
        center_world = camera_origin + distance * ray_world

        expected_distance = float(
            np.linalg.norm(center_world - self.expected_port_world)
        )
        if (
            self.expected_port_max_distance_m > 0.0
            and expected_distance > self.expected_port_max_distance_m
        ):
            raise ValueError(
                "port estimate outside safety gate: "
                f"{expected_distance:.3f}m"
            )
        if self.axis_mode == "calibrated":
            inward_axis = self.calibrated_inward_axis.copy()
            axis_source = "calibrated_port_geometry"
        else:
            inward_axis = inward_surface_normal
            axis_source = "rgbd_surface_normal"
        return center_world, inward_axis, {
            "area": area,
            "hole_area": hole_area,
            "plane_count": plane["count"],
            "plane_rms": plane["rms"],
            "expected_distance": expected_distance,
            "sam_centroid_uv": centroid_uv,
            "geometric_centroid_uv": geometric_centroid_uv,
            "axis_source": axis_source,
        }

    def update_lock(self, point, axis, diagnostics):
        self.pose_samples.append((point.copy(), axis.copy()))
        points = np.asarray([sample[0] for sample in self.pose_samples])
        axes = np.asarray([sample[1] for sample in self.pose_samples])
        center = np.median(points, axis=0)
        mean_axis = normalize(np.mean(axes, axis=0))
        center_spread = float(
            np.max(np.linalg.norm(points - center[None, :], axis=1))
        )
        dots = np.clip(axes @ mean_axis, -1.0, 1.0)
        axis_spread_deg = float(np.degrees(np.max(np.arccos(dots))))

        self.publish_point(self.pub_raw_point, point)
        self.publish_axis(self.pub_raw_axis, axis)
        stable = (
            len(self.pose_samples) >= self.stable_frames
            and center_spread <= self.max_center_spread_m
            and axis_spread_deg <= self.max_axis_spread_deg
        )
        if stable and self.locked_point is None:
            self.locked_point = center.copy()
            self.locked_axis = mean_axis.copy()
            self.locked_axis_source = diagnostics["axis_source"]
            self.publish_locked_pose()
            self.get_logger().info(
                "[PORT_LOCKED] "
                f"point=({center[0]:.4f},{center[1]:.4f},{center[2]:.4f}) "
                f"axis=({mean_axis[0]:.3f},{mean_axis[1]:.3f},"
                f"{mean_axis[2]:.3f})"
            )

        self.publish_status(
            "source=sam3+rgbd "
            f"prompt={self.current_prompt!r} score={self.latest_score:.3f} "
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
            locked_axis_source = self.locked_axis_source
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
                status=(
                    "center: SAM3-guided RGB-D  |  "
                    f"axis: {locked_axis_source}"
                ),
                locked=True,
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
        try:
            point, axis, diagnostics = self.estimate_pose(mask, depth, info)
        except Exception as exc:
            self.pose_samples.clear()
            self.publish_status(f"port estimate rejected: {exc}", valid=False)
            self.publish_overlay(
                mask,
                info,
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
        self.publish_overlay(
            mask,
            info,
            point=overlay_point,
            axis=overlay_axis,
            status=(
                f"center=({overlay_point[0]:.3f},"
                f"{overlay_point[1]:.3f},{overlay_point[2]:.3f}) m  "
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
