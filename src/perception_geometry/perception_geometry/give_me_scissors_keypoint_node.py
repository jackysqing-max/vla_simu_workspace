"""Generate indexed ReKep/Give-me-scissors-style keypoint candidates."""

from __future__ import annotations

import json
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
from std_msgs.msg import Bool, Float32, Header, Int32, String
from tf2_ros import Buffer, TransformListener

from perception_geometry.mask_depth_fusion_node import (
    estimate_target_plane_world,
    imgmsg_to_depth_meters,
    imgmsg_to_mask_u8,
    imgmsg_to_rgb8,
    rgb8_to_imgmsg,
    transform_points,
    transform_to_matrix,
)
from perception_geometry.pointcloud_ops import (
    evenly_spaced_subsample_indices,
    intrinsics_from_camera_info,
    kmeans,
    masked_rgbd_to_xyzrgbuv,
    pca_project,
    xyz_to_pixel,
)
from perception_geometry.ros_msg_utils import (
    make_bool_msg,
    make_point_stamped,
    xyz_to_pointcloud2,
)


class GiveMeScissorsKeypointNode(Node):
    """Publish indexed candidate keypoints from segmented RGB-D observations.

    This node mirrors the key idea in Give-me-scissors/ReKep: produce a compact
    set of numbered 3D candidate keypoints from visual features inside a mask,
    then let a high-level planner refer to candidate indices.
    """

    def __init__(self):
        super().__init__("give_me_scissors_keypoint_node")

        self.declare_parameter("mask_topic", "/sam3/mask")
        self.declare_parameter("color_topic", "/sim/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/sim/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/sim/camera/color/camera_info")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("prompt_topic", "/sam3/active_prompt")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("max_frame_age_sec", 0.25)
        self.declare_parameter("num_clusters", 10)
        self.declare_parameter("max_samples", 1800)
        self.declare_parameter("pca_dim", 5)
        self.declare_parameter("xyz_weight", 1.0)
        self.declare_parameter("rgb_weight", 0.35)
        self.declare_parameter("uv_weight", 0.20)
        self.declare_parameter("merge_distance_m", 0.025)
        self.declare_parameter("min_cluster_points", 8)
        self.declare_parameter("max_candidates", 12)
        self.declare_parameter("plane_local_radius_m", 0.18)
        self.declare_parameter("plane_min_points", 24)
        self.declare_parameter("plane_max_samples", 1024)
        self.declare_parameter("plane_normal_reference", [0.0, 0.0, 1.0])
        self.declare_parameter("enable_selected_keypoint_stabilizer", True)
        self.declare_parameter("selected_keypoint_filter_alpha", 0.20)
        self.declare_parameter("selected_keypoint_lock_radius_m", 0.08)
        self.declare_parameter("selected_keypoint_jump_reset_m", 0.14)
        self.declare_parameter("selected_keypoint_jump_hold_frames", 8)
        self.declare_parameter("seed", 2025)
        self.declare_parameter("selected_index", -1)
        self.declare_parameter("overlay_topic", "/gms_keypoints/overlay")
        self.declare_parameter("candidate_cloud_topic", "/gms_keypoints/candidates")
        self.declare_parameter("metadata_topic", "/gms_keypoints/metadata_json")
        self.declare_parameter("status_topic", "/gms_keypoints/status")
        self.declare_parameter("valid_topic", "/gms_keypoints/valid")
        self.declare_parameter("selected_keypoint_topic", "/gms_keypoints/selected_keypoint_3d")
        self.declare_parameter("selected_keypoint_px_topic", "/gms_keypoints/selected_keypoint_px")
        self.declare_parameter("plane_normal_topic", "/gms_keypoints/object_plane_normal")
        self.declare_parameter("plane_tangent_topic", "/gms_keypoints/object_plane_tangent")
        self.declare_parameter("selected_index_topic", "/gms_keypoints/selected_index")
        self.declare_parameter("select_index_topic", "/gms_keypoints/select_index")

        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.score_topic = str(self.get_parameter("score_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)
        self.num_clusters = max(1, int(self.get_parameter("num_clusters").value))
        self.max_samples = max(10, int(self.get_parameter("max_samples").value))
        self.pca_dim = max(1, int(self.get_parameter("pca_dim").value))
        self.xyz_weight = float(self.get_parameter("xyz_weight").value)
        self.rgb_weight = float(self.get_parameter("rgb_weight").value)
        self.uv_weight = float(self.get_parameter("uv_weight").value)
        self.merge_distance_m = max(0.0, float(self.get_parameter("merge_distance_m").value))
        self.min_cluster_points = max(1, int(self.get_parameter("min_cluster_points").value))
        self.max_candidates = max(1, int(self.get_parameter("max_candidates").value))
        self.plane_local_radius_m = max(
            1e-4,
            float(self.get_parameter("plane_local_radius_m").value),
        )
        self.plane_min_points = max(3, int(self.get_parameter("plane_min_points").value))
        self.plane_max_samples = int(self.get_parameter("plane_max_samples").value)
        self.plane_normal_reference = np.array(
            [float(value) for value in self.get_parameter("plane_normal_reference").value][
                :3
            ],
            dtype=np.float64,
        )
        if self.plane_normal_reference.shape[0] < 3:
            self.plane_normal_reference = np.pad(
                self.plane_normal_reference,
                (0, 3 - self.plane_normal_reference.shape[0]),
            )
        self.enable_selected_keypoint_stabilizer = bool(
            self.get_parameter("enable_selected_keypoint_stabilizer").value
        )
        self.selected_keypoint_filter_alpha = float(
            self.get_parameter("selected_keypoint_filter_alpha").value
        )
        self.selected_keypoint_lock_radius_m = max(
            0.0,
            float(self.get_parameter("selected_keypoint_lock_radius_m").value),
        )
        self.selected_keypoint_jump_reset_m = max(
            0.0,
            float(self.get_parameter("selected_keypoint_jump_reset_m").value),
        )
        self.selected_keypoint_jump_hold_frames = max(
            0,
            int(self.get_parameter("selected_keypoint_jump_hold_frames").value),
        )
        self.seed = int(self.get_parameter("seed").value)
        self.selected_index = int(self.get_parameter("selected_index").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.candidate_cloud_topic = str(self.get_parameter("candidate_cloud_topic").value)
        self.metadata_topic = str(self.get_parameter("metadata_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.selected_keypoint_topic = str(
            self.get_parameter("selected_keypoint_topic").value
        )
        self.selected_keypoint_px_topic = str(
            self.get_parameter("selected_keypoint_px_topic").value
        )
        self.plane_normal_topic = str(self.get_parameter("plane_normal_topic").value)
        self.plane_tangent_topic = str(self.get_parameter("plane_tangent_topic").value)
        self.selected_index_topic = str(self.get_parameter("selected_index_topic").value)
        self.select_index_topic = str(self.get_parameter("select_index_topic").value)

        self.lock = threading.Lock()
        self.latest_color_pack = None
        self.latest_depth_pack = None
        self.latest_info_pack = None
        self.latest_score = 0.0
        self.current_prompt = ""
        self.last_candidates = []
        self.last_header = None
        self.last_pixel_header = None
        self.stable_keypoint_xyz = None
        self.held_jump_count = 0
        self.last_status_log_time = 0.0

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(Image, self.color_topic, self.on_color, image_qos)
        self.create_subscription(Image, self.depth_topic, self.on_depth, image_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, image_qos)
        self.create_subscription(Image, self.mask_topic, self.on_mask, image_qos)
        self.create_subscription(Float32, self.score_topic, self.on_score, 10)
        self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
        self.create_subscription(Int32, self.select_index_topic, self.on_select_index, 10)

        self.pub_overlay = self.create_publisher(Image, self.overlay_topic, 10)
        self.pub_candidate_cloud = self.create_publisher(
            type(xyz_to_pointcloud2(np.zeros((0, 3), dtype=np.float32), CameraInfo().header)),
            self.candidate_cloud_topic,
            1,
        )
        self.pub_metadata = self.create_publisher(String, self.metadata_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_valid = self.create_publisher(Bool, self.valid_topic, 10)
        self.pub_selected_keypoint = self.create_publisher(
            PointStamped,
            self.selected_keypoint_topic,
            10,
        )
        self.pub_selected_keypoint_px = self.create_publisher(
            PointStamped,
            self.selected_keypoint_px_topic,
            10,
        )
        self.pub_plane_normal = self.create_publisher(
            Vector3Stamped,
            self.plane_normal_topic,
            10,
        )
        self.pub_plane_tangent = self.create_publisher(
            Vector3Stamped,
            self.plane_tangent_topic,
            10,
        )
        self.pub_selected_index = self.create_publisher(Int32, self.selected_index_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            "give_me_scissors_keypoint_node started. "
            f"mask_topic={self.mask_topic} target_frame={self.target_frame}"
        )

    def on_color(self, msg: Image):
        try:
            rgb = imgmsg_to_rgb8(msg)
        except Exception as exc:
            self._publish_status(f"failed_color_decode: {exc}", warn=True)
            return
        with self.lock:
            self.latest_color_pack = (msg.header, rgb)

    def on_depth(self, msg: Image):
        try:
            depth_m = imgmsg_to_depth_meters(msg, self.depth_scale)
        except Exception as exc:
            self._publish_status(f"failed_depth_decode: {exc}", warn=True)
            return
        with self.lock:
            self.latest_depth_pack = (msg.header, depth_m)

    def on_info(self, msg: CameraInfo):
        with self.lock:
            self.latest_info_pack = (msg.header, msg)

    def on_score(self, msg: Float32):
        with self.lock:
            self.latest_score = float(msg.data)

    def on_prompt(self, msg: String):
        prompt = msg.data.strip()
        with self.lock:
            if prompt != self.current_prompt:
                self._reset_selected_keypoint_stabilizer_locked()
            self.current_prompt = prompt

    def on_select_index(self, msg: Int32):
        with self.lock:
            self.selected_index = int(msg.data)
            self._reset_selected_keypoint_stabilizer_locked()
            candidates = list(self.last_candidates)
            header = self.last_header
            pixel_header = self.last_pixel_header
        self._publish_selected(candidates, header, pixel_header)
        self._publish_status(f"selected_index_set: {int(msg.data)}")

    def _header_age_sec(self, header) -> float:
        if header is None:
            return float("inf")
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return 0.0
        return (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9

    def _reset_selected_keypoint_stabilizer_locked(self):
        self.stable_keypoint_xyz = None
        self.held_jump_count = 0

    def _publish_status(self, text: str, *, warn: bool = False):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        now = time.time()
        if now - self.last_status_log_time > 2.0:
            self.last_status_log_time = now
            if warn:
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

    def _publish_invalid(self, header, rgb=None, text: str = "invalid"):
        self.pub_valid.publish(make_bool_msg(False))
        self.pub_candidate_cloud.publish(
            xyz_to_pointcloud2(np.zeros((0, 3), dtype=np.float32), header)
        )
        metadata = {
            "method": "give_me_scissors_rgbd_candidates",
            "valid": False,
            "reason": text,
            "prompt": self.current_prompt,
            "target_frame": self.target_frame,
            "num_candidates": 0,
            "selected_index": -1,
            "candidates": [],
        }
        msg = String()
        msg.data = json.dumps(metadata, ensure_ascii=False)
        self.pub_metadata.publish(msg)
        if rgb is not None:
            overlay = rgb.copy()
            cv2.rectangle(overlay, (0, 0), (overlay.shape[1] - 1, overlay.shape[0] - 1), (255, 0, 0), 3)
            cv2.putText(
                overlay,
                text,
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
            self.pub_overlay.publish(rgb8_to_imgmsg(overlay, header))
        self._publish_status(text, warn=True)

    def _camera_to_target_transform(self, source_frame: str):
        if self.target_frame == source_frame:
            return np.eye(4, dtype=np.float64)
        tf_msg = self.tf_buffer.lookup_transform(
            self.target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.15),
        )
        return transform_to_matrix(tf_msg)

    def _normalize_features(self, values: np.ndarray):
        values = np.asarray(values, dtype=np.float32)
        if values.shape[0] == 0:
            return values
        center = np.median(values, axis=0, keepdims=True)
        scale = np.std(values, axis=0, keepdims=True)
        scale = np.maximum(scale, 1e-6)
        return (values - center) / scale

    def _generate_candidates(self, xyz_target, rgb, uv, image_shape):
        keep = evenly_spaced_subsample_indices(xyz_target.shape[0], self.max_samples)
        xyz_kept = xyz_target[keep].astype(np.float32)
        rgb_kept = rgb[keep].astype(np.float32)
        uv_kept = uv[keep].astype(np.float32)

        height, width = image_shape[:2]
        uv_norm = uv_kept / np.array([[max(width - 1, 1), max(height - 1, 1)]], dtype=np.float32)
        features = np.concatenate(
            [
                self.xyz_weight * self._normalize_features(xyz_kept),
                self.rgb_weight * rgb_kept,
                self.uv_weight * uv_norm,
            ],
            axis=1,
        ).astype(np.float32)
        embedding = pca_project(features, min(self.pca_dim, features.shape[1]))
        labels, centers = kmeans(embedding, k=self.num_clusters, seed=self.seed)

        proposals = []
        for cluster_index in sorted(int(label) for label in np.unique(labels)):
            member_indices = np.flatnonzero(labels == cluster_index)
            if member_indices.shape[0] < self.min_cluster_points:
                continue
            member_embedding = embedding[member_indices]
            center = centers[cluster_index]
            closest_local = int(np.argmin(np.linalg.norm(member_embedding - center[None, :], axis=1)))
            sample_index = int(member_indices[closest_local])
            proposals.append(
                {
                    "xyz": xyz_kept[sample_index],
                    "uv": uv_kept[sample_index],
                    "cluster_size": int(member_indices.shape[0]),
                    "source_cluster": int(cluster_index),
                }
            )

        merged = self._merge_close_proposals(proposals)
        merged.sort(key=lambda item: (float(item["uv"][1]), float(item["uv"][0])))
        return merged[: self.max_candidates]

    def _merge_close_proposals(self, proposals):
        if not proposals:
            return []
        if self.merge_distance_m <= 1e-9:
            return list(proposals)

        used = [False] * len(proposals)
        merged = []
        for index, proposal in enumerate(proposals):
            if used[index]:
                continue
            xyz = np.asarray(proposal["xyz"], dtype=np.float32)
            group_indices = []
            for other_index, other in enumerate(proposals):
                if used[other_index]:
                    continue
                other_xyz = np.asarray(other["xyz"], dtype=np.float32)
                if float(np.linalg.norm(other_xyz - xyz)) <= self.merge_distance_m:
                    group_indices.append(other_index)
            if not group_indices:
                continue
            for group_index in group_indices:
                used[group_index] = True
            group = [proposals[group_index] for group_index in group_indices]
            group_xyz = np.asarray([item["xyz"] for item in group], dtype=np.float32)
            group_center = np.mean(group_xyz, axis=0)
            representative = min(
                group,
                key=lambda item: float(np.linalg.norm(np.asarray(item["xyz"]) - group_center)),
            )
            merged.append(
                {
                    "xyz": np.asarray(representative["xyz"], dtype=np.float32),
                    "uv": np.asarray(representative["uv"], dtype=np.float32),
                    "cluster_size": int(sum(item["cluster_size"] for item in group)),
                    "source_cluster": int(representative["source_cluster"]),
                }
            )
        return merged

    def _seed_selected_index(self, candidates):
        if not candidates:
            return -1
        if 0 <= self.selected_index < len(candidates):
            return self.selected_index
        xyz = np.asarray([item["xyz"] for item in candidates], dtype=np.float32)
        center = np.median(xyz, axis=0)
        return int(np.argmin(np.linalg.norm(xyz - center[None, :], axis=1)))

    def _select_candidate_index(self, candidates):
        if not candidates:
            return -1, "none"
        xyz = np.asarray([item["xyz"] for item in candidates], dtype=np.float32)
        with self.lock:
            stable = (
                None
                if self.stable_keypoint_xyz is None
                else np.asarray(self.stable_keypoint_xyz, dtype=np.float32)
            )
        if stable is not None and np.all(np.isfinite(stable)):
            distances = np.linalg.norm(xyz - stable[None, :], axis=1)
            nearest_index = int(np.argmin(distances))
            nearest_distance = float(distances[nearest_index])
            if (
                self.selected_keypoint_lock_radius_m <= 0.0
                or nearest_distance <= self.selected_keypoint_lock_radius_m
            ):
                return nearest_index, f"locked:{nearest_distance:.3f}m"
        return self._seed_selected_index(candidates), "seed"

    def _stabilize_selected_keypoint(self, raw_xyz):
        raw = np.asarray(raw_xyz, dtype=np.float32).reshape(3)
        if not self.enable_selected_keypoint_stabilizer:
            with self.lock:
                self.stable_keypoint_xyz = raw
                self.held_jump_count = 0
            return raw, "raw", 0.0

        with self.lock:
            stable = (
                None
                if self.stable_keypoint_xyz is None
                else np.asarray(self.stable_keypoint_xyz, dtype=np.float32)
            )
            held_count = int(self.held_jump_count)

        if stable is None or not np.all(np.isfinite(stable)):
            with self.lock:
                self.stable_keypoint_xyz = raw
                self.held_jump_count = 0
            return raw, "init", 0.0

        jump_m = float(np.linalg.norm(raw - stable))
        if (
            self.selected_keypoint_jump_reset_m > 0.0
            and jump_m > self.selected_keypoint_jump_reset_m
            and held_count < self.selected_keypoint_jump_hold_frames
        ):
            with self.lock:
                self.held_jump_count = held_count + 1
            return stable, f"hold_jump:{held_count + 1}", jump_m

        if (
            self.selected_keypoint_jump_reset_m > 0.0
            and jump_m > self.selected_keypoint_jump_reset_m
            and held_count >= self.selected_keypoint_jump_hold_frames
        ):
            with self.lock:
                self.stable_keypoint_xyz = raw
                self.held_jump_count = 0
            return raw, "reset", jump_m

        alpha = float(np.clip(self.selected_keypoint_filter_alpha, 0.0, 1.0))
        filtered = stable + alpha * (raw - stable)
        with self.lock:
            self.stable_keypoint_xyz = filtered.astype(np.float32)
            self.held_jump_count = 0
        return filtered.astype(np.float32), "filtered", jump_m

    def _publish_selected(
        self,
        candidates,
        header,
        pixel_header=None,
        selected_index=None,
        selected_xyz=None,
        selected_uv=None,
    ):
        if selected_index is None:
            selected_index, _select_state = self._select_candidate_index(candidates)
        idx_msg = Int32()
        idx_msg.data = int(selected_index)
        self.pub_selected_index.publish(idx_msg)
        if selected_index < 0 or header is None:
            return selected_index
        candidate = candidates[selected_index]
        point = candidate["xyz"] if selected_xyz is None else selected_xyz
        self.pub_selected_keypoint.publish(
            make_point_stamped(float(point[0]), float(point[1]), float(point[2]), header)
        )
        uv = candidate["uv"] if selected_uv is None else selected_uv
        self.pub_selected_keypoint_px.publish(
            make_point_stamped(
                float(uv[0]),
                float(uv[1]),
                0.0,
                header if pixel_header is None else pixel_header,
            )
        )
        return selected_index

    def _publish_plane_vector(self, publisher, vector, header):
        msg = Vector3Stamped()
        msg.header = header
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def _publish_metadata(self, candidates, selected_index, header, object_plane=None):
        payload = {
            "method": "give_me_scissors_rgbd_candidates",
            "valid": bool(candidates),
            "prompt": self.current_prompt,
            "target_frame": self.target_frame,
            "frame_id": "" if header is None else str(header.frame_id),
            "num_candidates": len(candidates),
            "selected_index": int(selected_index),
            "parameters": {
                "num_clusters": self.num_clusters,
                "max_samples": self.max_samples,
                "pca_dim": self.pca_dim,
                "merge_distance_m": self.merge_distance_m,
                "plane_local_radius_m": self.plane_local_radius_m,
                "enable_selected_keypoint_stabilizer": self.enable_selected_keypoint_stabilizer,
                "selected_keypoint_filter_alpha": self.selected_keypoint_filter_alpha,
                "selected_keypoint_lock_radius_m": self.selected_keypoint_lock_radius_m,
                "selected_keypoint_jump_reset_m": self.selected_keypoint_jump_reset_m,
            },
            "object_plane": None
            if object_plane is None
            else {
                "center": [float(value) for value in object_plane["center"]],
                "normal": [float(value) for value in object_plane["normal"]],
                "tangent": [float(value) for value in object_plane["tangent"]],
                "count": int(object_plane["count"]),
                "rms": float(object_plane["rms"]),
            },
            "candidates": [
                {
                    "id": index,
                    "xyz": [float(value) for value in item["xyz"]],
                    "uv": [float(value) for value in item["uv"]],
                    "cluster_size": int(item["cluster_size"]),
                    "source_cluster": int(item["source_cluster"]),
                }
                for index, item in enumerate(candidates)
            ],
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_metadata.publish(msg)

    def _draw_overlay(self, rgb, mask_u8, candidates, selected_index, status_lines, selected_uv=None):
        overlay = rgb.copy()
        if mask_u8 is not None and mask_u8.shape[:2] == overlay.shape[:2]:
            mask_bool = mask_u8 > 0
            tint = overlay.copy()
            tint[mask_bool] = np.array([0, 210, 80], dtype=np.uint8)
            overlay = cv2.addWeighted(overlay, 0.65, tint, 0.35, 0.0)

        for index, candidate in enumerate(candidates):
            draw_uv = (
                selected_uv
                if index == selected_index and selected_uv is not None
                else candidate["uv"]
            )
            u = int(round(float(draw_uv[0])))
            v = int(round(float(draw_uv[1])))
            color = (0, 255, 255) if index == selected_index else (255, 40, 40)
            radius = 8 if index == selected_index else 6
            cv2.circle(overlay, (u, v), radius, color, 2, cv2.LINE_AA)
            label = str(index)
            (tw, th), _baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                2,
            )
            x0 = min(max(u + 7, 0), max(overlay.shape[1] - tw - 6, 0))
            y0 = min(max(v - 7, th + 4), max(overlay.shape[0] - 4, th + 4))
            cv2.rectangle(
                overlay,
                (x0 - 3, y0 - th - 3),
                (x0 + tw + 3, y0 + 4),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                overlay,
                label,
                (x0, y0),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        y = 22
        for line in status_lines:
            cv2.putText(
                overlay,
                line,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 0),
                1,
                cv2.LINE_AA,
            )
            y += 18
        return overlay

    def on_mask(self, msg: Image):
        try:
            mask_u8 = imgmsg_to_mask_u8(msg)
        except Exception as exc:
            self._publish_status(f"failed_mask_decode: {exc}", warn=True)
            return

        with self.lock:
            color_pack = self.latest_color_pack
            depth_pack = self.latest_depth_pack
            info_pack = self.latest_info_pack
            score = self.latest_score
            prompt = self.current_prompt

        if score < self.min_score:
            self._publish_invalid(msg.header, text=f"score_below_min: {score:.3f}")
            return
        if color_pack is None or depth_pack is None or info_pack is None:
            self._publish_invalid(msg.header, text="waiting_for_rgbd")
            return

        color_header, rgb = color_pack
        depth_header, depth_m = depth_pack
        info_header, info = info_pack
        if (
            self._header_age_sec(color_header) > self.max_frame_age_sec
            or self._header_age_sec(depth_header) > self.max_frame_age_sec
            or self._header_age_sec(info_header) > self.max_frame_age_sec
        ):
            self._publish_invalid(msg.header, rgb=rgb, text="stale_rgbd")
            return
        if rgb.shape[:2] != mask_u8.shape[:2] or depth_m.shape[:2] != mask_u8.shape[:2]:
            self._publish_invalid(msg.header, rgb=rgb, text="mask_rgbd_size_mismatch")
            return

        fx, fy, cx, cy = intrinsics_from_camera_info(info)
        xyz_cam, rgb_samples, uv = masked_rgbd_to_xyzrgbuv(
            depth_m,
            mask_u8,
            rgb,
            fx,
            fy,
            cx,
            cy,
        )
        if xyz_cam.shape[0] < max(self.min_cluster_points, self.num_clusters):
            self._publish_invalid(msg.header, rgb=rgb, text="not_enough_masked_depth_points")
            return

        try:
            cam_to_target = self._camera_to_target_transform(msg.header.frame_id)
            xyz_target = transform_points(cam_to_target, xyz_cam)
        except Exception as exc:
            self._publish_invalid(msg.header, rgb=rgb, text=f"tf_lookup_failed: {exc}")
            return

        finite = np.all(np.isfinite(xyz_target), axis=1)
        xyz_target = xyz_target[finite]
        xyz_cam = xyz_cam[finite]
        rgb_samples = rgb_samples[finite]
        uv = uv[finite]
        if xyz_target.shape[0] < max(self.min_cluster_points, self.num_clusters):
            self._publish_invalid(msg.header, rgb=rgb, text="not_enough_finite_points")
            return

        candidates = self._generate_candidates(xyz_target, rgb_samples, uv, rgb.shape)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        if not candidates:
            self._publish_invalid(header, rgb=rgb, text="no_candidates_after_clustering")
            return

        selected_index, select_state = self._select_candidate_index(candidates)
        selected_xyz = None
        selected_uv = None
        stabilizer_state = "none"
        jump_m = 0.0
        if 0 <= selected_index < len(candidates):
            raw_selected_xyz = np.asarray(candidates[selected_index]["xyz"], dtype=np.float32)
            selected_xyz, stabilizer_state, jump_m = self._stabilize_selected_keypoint(
                raw_selected_xyz
            )
            try:
                target_to_cam = np.linalg.inv(cam_to_target)
                selected_cam = transform_points(target_to_cam, selected_xyz.reshape(1, 3))[0]
                pixel = xyz_to_pixel(
                    float(selected_cam[0]),
                    float(selected_cam[1]),
                    float(selected_cam[2]),
                    fx,
                    fy,
                    cx,
                    cy,
                )
                if pixel is not None:
                    selected_uv = np.asarray(pixel, dtype=np.float32)
            except Exception:
                selected_uv = None
        selected_index = self._publish_selected(
            candidates,
            header,
            msg.header,
            selected_index=selected_index,
            selected_xyz=selected_xyz,
            selected_uv=selected_uv,
        )
        object_plane = None
        if 0 <= selected_index < len(candidates):
            object_plane = estimate_target_plane_world(
                xyz_target,
                selected_xyz,
                local_radius_m=self.plane_local_radius_m,
                min_points=self.plane_min_points,
                max_samples=self.plane_max_samples,
                normal_reference=self.plane_normal_reference,
            )
            if object_plane is not None:
                self._publish_plane_vector(
                    self.pub_plane_normal,
                    object_plane["normal"],
                    header,
                )
                self._publish_plane_vector(
                    self.pub_plane_tangent,
                    object_plane["tangent"],
                    header,
                )
        with self.lock:
            self.last_candidates = list(candidates)
            self.last_header = header
            self.last_pixel_header = msg.header

        candidates_xyz = np.asarray([item["xyz"] for item in candidates], dtype=np.float32)
        self.pub_candidate_cloud.publish(xyz_to_pointcloud2(candidates_xyz, header))
        self.pub_valid.publish(make_bool_msg(True))
        self._publish_metadata(candidates, selected_index, header, object_plane)

        status_lines = [
            "GMS keypoint candidates",
            f"prompt={prompt or '<none>'}",
            f"score={score:.3f} candidates={len(candidates)} selected={selected_index}",
            f"select={select_state} stabilizer={stabilizer_state} jump={jump_m:.3f}m",
        ]
        if object_plane is not None:
            n = object_plane["normal"]
            status_lines.append(
                f"plane_n=({float(n[0]):.2f},{float(n[1]):.2f},{float(n[2]):.2f})"
            )
        overlay = self._draw_overlay(
            rgb,
            mask_u8,
            candidates,
            selected_index,
            status_lines,
            selected_uv=selected_uv,
        )
        self.pub_overlay.publish(rgb8_to_imgmsg(overlay, msg.header))
        self._publish_status(
            f"gms_candidates: count={len(candidates)} selected={selected_index} "
            f"select={select_state} stabilizer={stabilizer_state} "
            f"jump={jump_m:.3f} prompt={prompt}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GiveMeScissorsKeypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
