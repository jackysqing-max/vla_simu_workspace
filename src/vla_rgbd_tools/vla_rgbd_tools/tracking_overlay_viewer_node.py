#!/usr/bin/env python3
"""Display live mask, point-cloud, and keypoint views for the RGB-D pipeline."""

from __future__ import annotations

import os
import threading

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String


class TrackingOverlayViewerNode(Node):
    """Show mask, masked 3D cloud, and keypoint state in three local windows."""

    def __init__(self):
        super().__init__("tracking_overlay_viewer_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("mask_topic", "/sam3/mask")
        self.declare_parameter("points_topic", "/perception/masked_points")
        self.declare_parameter("overlay_topic", "/perception/keypoint_overlay")
        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("keypoint_topic", "/perception/keypoint_3d")
        self.declare_parameter("keypoint_px_topic", "/perception/keypoint_px")
        self.declare_parameter("valid_topic", "/perception/valid")
        self.declare_parameter("window_name", "SAM3 Keypoint")
        self.declare_parameter("mask_window_name", "SAM3 Mask")
        self.declare_parameter("cloud_window_name", "Masked 3D Point Cloud")
        self.declare_parameter("refresh_hz", 20.0)
        self.declare_parameter("keypoint_timeout_sec", 1.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("cloud_min_depth_m", 0.10)
        self.declare_parameter("cloud_max_depth_m", 2.00)
        self.declare_parameter("cloud_stride", 2)
        self.declare_parameter("cloud_display_max_points", 8000)
        self.declare_parameter("display_scale", 2.0)
        self.declare_parameter("status_panel_width", 360)
        self.declare_parameter("follow_window_resize", True)
        self.declare_parameter("prefer_overlay_image", False)
        self.declare_parameter("show_cloud_window", True)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.mask_topic = str(self.get_parameter("mask_topic").value)
        self.points_topic = str(self.get_parameter("points_topic").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.score_topic = str(self.get_parameter("score_topic").value)
        self.keypoint_topic = str(self.get_parameter("keypoint_topic").value)
        self.keypoint_px_topic = str(self.get_parameter("keypoint_px_topic").value)
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.mask_window_name = str(self.get_parameter("mask_window_name").value)
        self.cloud_window_name = str(self.get_parameter("cloud_window_name").value)
        self.refresh_hz = float(self.get_parameter("refresh_hz").value)
        self.keypoint_timeout_sec = float(self.get_parameter("keypoint_timeout_sec").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.cloud_min_depth_m = float(self.get_parameter("cloud_min_depth_m").value)
        self.cloud_max_depth_m = float(self.get_parameter("cloud_max_depth_m").value)
        self.cloud_stride = max(1, int(self.get_parameter("cloud_stride").value))
        self.cloud_display_max_points = max(
            100,
            int(self.get_parameter("cloud_display_max_points").value),
        )
        self.display_scale = max(1.0, float(self.get_parameter("display_scale").value))
        self.status_panel_width = max(
            220,
            int(self.get_parameter("status_panel_width").value),
        )
        self.follow_window_resize = bool(self.get_parameter("follow_window_resize").value)
        self.prefer_overlay_image = bool(self.get_parameter("prefer_overlay_image").value)
        self.show_cloud_window = bool(self.get_parameter("show_cloud_window").value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_color_bgr = None
        self.latest_color_header = None
        self.latest_depth_m = None
        self.latest_depth_header = None
        self.camera_intrinsics = None
        self.camera_info_header = None
        self.latest_mask_u8 = None
        self.latest_mask_header = None
        self.latest_points_xyz = None
        self.latest_points_header = None
        self.latest_overlay_bgr = None
        self.latest_overlay_header = None
        self.current_prompt = ""
        self.current_score = 0.0
        self.target_valid = False
        self.keypoint_xyz = None
        self.keypoint_header = None
        self.keypoint_uv = None
        self.keypoint_px_header = None
        self.logged_first_color = False
        self.logged_first_depth = False
        self.logged_first_info = False
        self.logged_first_mask = False
        self.logged_first_points = False
        self.logged_first_overlay = False

        self.window_enabled = bool(os.environ.get("DISPLAY"))
        if not self.window_enabled:
            self.get_logger().warning("DISPLAY is not set; viewer windows are disabled")
        else:
            window_names = [self.mask_window_name, self.window_name]
            if self.show_cloud_window:
                window_names.insert(1, self.cloud_window_name)
            for window_name in window_names:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.mask_window_name,
                int(320 * self.display_scale + self.status_panel_width),
                int(240 * self.display_scale),
            )
            if self.show_cloud_window:
                cv2.resizeWindow(self.cloud_window_name, 960, 640)
            cv2.resizeWindow(
                self.window_name,
                int(320 * self.display_scale + self.status_panel_width),
                int(240 * self.display_scale),
            )

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(Image, self.color_topic, self.on_color, image_qos)
        self.create_subscription(Image, self.depth_topic, self.on_depth, image_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, image_qos)
        self.create_subscription(Image, self.mask_topic, self.on_mask, image_qos)
        self.create_subscription(Image, self.overlay_topic, self.on_overlay, image_qos)
        self.create_subscription(PointCloud2, self.points_topic, self.on_points, 1)
        self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
        self.create_subscription(Float32, self.score_topic, self.on_score, 10)
        self.create_subscription(PointStamped, self.keypoint_topic, self.on_keypoint, 10)
        self.create_subscription(PointStamped, self.keypoint_px_topic, self.on_keypoint_px, 10)
        self.create_subscription(Bool, self.valid_topic, self.on_valid, 10)

        self.timer = self.create_timer(1.0 / max(self.refresh_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            "tracking_overlay_viewer_node started. "
            f"color_topic={self.color_topic} depth_topic={self.depth_topic} "
            f"mask_topic={self.mask_topic} "
            f"points_topic={self.points_topic} overlay_topic={self.overlay_topic}"
        )

    def on_color(self, msg: Image):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"failed to decode color image: {exc}")
            return

        with self.lock:
            self.latest_color_bgr = image_bgr.copy()
            self.latest_color_header = msg.header
        if not self.logged_first_color:
            self.logged_first_color = True
            self.get_logger().info(
                f"received first color frame: {image_bgr.shape[1]}x{image_bgr.shape[0]}"
            )

    def on_depth(self, msg: Image):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning(f"failed to decode depth image: {exc}")
            return

        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * self.depth_scale
        else:
            depth_m = depth_raw.astype(np.float32)

        with self.lock:
            self.latest_depth_m = depth_m.copy()
            self.latest_depth_header = msg.header
        if not self.logged_first_depth:
            self.logged_first_depth = True
            self.get_logger().info(
                f"received first depth frame: {depth_m.shape[1]}x{depth_m.shape[0]}"
            )

    def on_camera_info(self, msg: CameraInfo):
        with self.lock:
            self.camera_intrinsics = (
                float(msg.k[0]),
                float(msg.k[4]),
                float(msg.k[2]),
                float(msg.k[5]),
            )
            self.camera_info_header = msg.header
        if not self.logged_first_info:
            self.logged_first_info = True
            self.get_logger().info("received first camera_info")

    def on_mask(self, msg: Image):
        try:
            mask_u8 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().warning(f"failed to decode mask image: {exc}")
            return

        with self.lock:
            self.latest_mask_u8 = mask_u8.copy()
            self.latest_mask_header = msg.header
        if not self.logged_first_mask:
            self.logged_first_mask = True
            self.get_logger().info(
                f"received first mask frame: {mask_u8.shape[1]}x{mask_u8.shape[0]}"
            )

    def on_overlay(self, msg: Image):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"failed to decode overlay image: {exc}")
            return

        with self.lock:
            self.latest_overlay_bgr = image_bgr.copy()
            self.latest_overlay_header = msg.header
        if not self.logged_first_overlay:
            self.logged_first_overlay = True
            self.get_logger().info(
                f"received first overlay frame: {image_bgr.shape[1]}x{image_bgr.shape[0]}"
            )

    def on_points(self, msg: PointCloud2):
        try:
            iterator = point_cloud2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            points_xyz = np.asarray(list(iterator), dtype=np.float32)
        except Exception as exc:
            self.get_logger().warning(f"failed to decode point cloud: {exc}")
            return

        if points_xyz.ndim != 2 or points_xyz.shape[1] != 3:
            points_xyz = np.zeros((0, 3), dtype=np.float32)

        if points_xyz.shape[0] > self.cloud_display_max_points:
            keep_indices = np.linspace(
                0,
                points_xyz.shape[0] - 1,
                num=self.cloud_display_max_points,
                dtype=np.int32,
            )
            points_xyz = points_xyz[keep_indices]

        with self.lock:
            self.latest_points_xyz = points_xyz
            self.latest_points_header = msg.header
        if not self.logged_first_points:
            self.logged_first_points = True
            self.get_logger().info(
                f"received first point cloud: {points_xyz.shape[0]} points"
            )

    def on_prompt(self, msg: String):
        with self.lock:
            self.current_prompt = msg.data.strip()

    def on_score(self, msg: Float32):
        with self.lock:
            self.current_score = float(msg.data)

    def on_valid(self, msg: Bool):
        with self.lock:
            self.target_valid = bool(msg.data)

    def on_keypoint(self, msg: PointStamped):
        with self.lock:
            self.keypoint_xyz = np.array(
                [msg.point.x, msg.point.y, msg.point.z],
                dtype=np.float64,
            )
            self.keypoint_header = msg.header

    def on_keypoint_px(self, msg: PointStamped):
        with self.lock:
            self.keypoint_uv = np.array(
                [msg.point.x, msg.point.y],
                dtype=np.float64,
            )
            self.keypoint_px_header = msg.header

    def _keypoint_is_fresh(self, header) -> bool:
        if self.keypoint_timeout_sec <= 0.0:
            return True
        if header is None:
            return False
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return True
        age_sec = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9
        return age_sec <= self.keypoint_timeout_sec

    def _common_status(self):
        with self.lock:
            prompt = self.current_prompt or "<none>"
            score = float(self.current_score)
            target_valid = bool(self.target_valid)
            keypoint_xyz = (
                None
                if self.keypoint_xyz is None
                else np.array(self.keypoint_xyz, copy=True)
            )
            keypoint_header = self.keypoint_header
            keypoint_uv = None if self.keypoint_uv is None else np.array(self.keypoint_uv, copy=True)
            keypoint_px_header = self.keypoint_px_header

        return {
            "prompt": prompt,
            "score": score,
            "target_valid": target_valid,
            "keypoint_xyz": keypoint_xyz,
            "keypoint_header": keypoint_header,
            "keypoint_uv": keypoint_uv,
            "keypoint_px_header": keypoint_px_header,
            "fresh": self._keypoint_is_fresh(keypoint_header),
        }

    def _wrap_text(self, text: str, max_chars: int = 36) -> list[str]:
        words = str(text).split()
        if not words:
            return [""]

        lines = []
        current = ""
        for word in words:
            if len(word) > max_chars:
                if current:
                    lines.append(current)
                    current = ""
                for start in range(0, len(word), max_chars):
                    lines.append(word[start : start + max_chars])
                continue
            candidate = word if not current else f"{current} {word}"
            if len(candidate) <= max_chars:
                current = candidate
            else:
                lines.append(current)
                current = word
        if current:
            lines.append(current)
        return lines

    def _scaled_frame(self, frame_bgr: np.ndarray) -> np.ndarray:
        scale = max(float(self.display_scale), 1.0)
        if abs(scale - 1.0) < 1e-6:
            return frame_bgr.copy()
        height, width = frame_bgr.shape[:2]
        return cv2.resize(
            frame_bgr,
            (int(round(width * scale)), int(round(height * scale))),
            interpolation=cv2.INTER_LINEAR,
        )

    def _compose_image_panel(
        self,
        image_bgr: np.ndarray,
        lines: list[tuple[str, tuple[int, int, int]]],
        *,
        border_color: tuple[int, int, int],
        scale_image: bool = True,
    ) -> np.ndarray:
        image = self._scaled_frame(image_bgr) if scale_image else image_bgr.copy()
        image_h, image_w = image.shape[:2]
        panel_w = int(self.status_panel_width)
        canvas = np.zeros((image_h, image_w + panel_w, 3), dtype=np.uint8)
        canvas[:, :image_w] = image
        canvas[:, image_w:] = (18, 24, 32)
        cv2.rectangle(canvas, (0, 0), (image_w - 1, image_h - 1), border_color, 2)
        cv2.line(canvas, (image_w, 0), (image_w, image_h - 1), (70, 86, 104), 1)

        y = 28
        left = image_w + 14
        max_chars = max(20, int((panel_w - 28) / 8.5))
        for raw_line, color in lines:
            wrapped_lines = self._wrap_text(raw_line, max_chars=max_chars)
            for line in wrapped_lines:
                cv2.putText(
                    canvas,
                    line,
                    (left, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    color,
                    1,
                    cv2.LINE_AA,
                )
                y += 22
                if y > image_h - 14:
                    return canvas
            y += 6
        return canvas

    def _fit_display_to_window(self, window_name: str, image_bgr: np.ndarray) -> np.ndarray:
        if not self.follow_window_resize:
            return image_bgr
        try:
            _x, _y, width, height = cv2.getWindowImageRect(window_name)
        except cv2.error:
            return image_bgr
        if width <= 80 or height <= 80:
            return image_bgr
        current_h, current_w = image_bgr.shape[:2]
        if abs(width - current_w) < 12 and abs(height - current_h) < 12:
            return image_bgr
        return cv2.resize(image_bgr, (width, height), interpolation=cv2.INTER_LINEAR)

    def _annotate_keypoint(self, frame_bgr: np.ndarray) -> np.ndarray:
        status = self._common_status()

        prompt = status["prompt"]
        score = status["score"]
        target_valid = status["target_valid"]
        keypoint_xyz = status["keypoint_xyz"]
        keypoint_header = status["keypoint_header"]
        keypoint_uv = status["keypoint_uv"]
        keypoint_px_header = status["keypoint_px_header"]
        fresh = status["fresh"]

        frame = frame_bgr.copy()
        height, width = frame.shape[:2]
        status_text = "TRACKING" if target_valid and fresh else "WAITING"
        status_color = (0, 200, 0) if target_valid and fresh else (0, 140, 255)
        if keypoint_xyz is None:
            xyz_text = "xyz = waiting"
        else:
            xyz_text = (
                f"xyz = ({keypoint_xyz[0]:.3f}, {keypoint_xyz[1]:.3f}, {keypoint_xyz[2]:.3f}) m"
            )
        frame_text = "frame = unknown"
        if keypoint_header is not None:
            frame_text = f"frame = {keypoint_header.frame_id or 'unknown'}"
            if not fresh:
                frame_text += " | stale"

        scaled = self._scaled_frame(frame)
        scale_x = scaled.shape[1] / max(width, 1)
        scale_y = scaled.shape[0] / max(height, 1)
        if target_valid and keypoint_uv is not None and self._keypoint_is_fresh(keypoint_px_header):
            u = int(round(float(keypoint_uv[0]) * scale_x))
            v = int(round(float(keypoint_uv[1]) * scale_y))
            if 0 <= u < scaled.shape[1] and 0 <= v < scaled.shape[0]:
                radius = max(10, int(round(10 * min(scale_x, scale_y))))
                thickness = max(2, int(round(2 * min(scale_x, scale_y))))
                cv2.line(scaled, (u - radius, v), (u + radius, v), (0, 0, 255), thickness)
                cv2.line(scaled, (u, v - radius), (u, v + radius), (0, 0, 255), thickness)
                cv2.circle(scaled, (u, v), max(4, thickness + 2), (0, 255, 255), -1)

        border_color = (0, 200, 0) if target_valid and fresh else (0, 0, 255)
        lines = [
            ("SAM3 Keypoint", (235, 245, 255)),
            (f"prompt = {prompt}", (235, 245, 255)),
            (f"score = {score:.3f}", (200, 220, 255)),
            (f"status = {status_text}", status_color),
            (xyz_text, (255, 255, 0)),
            (frame_text, (185, 195, 205)),
        ]
        return self._compose_image_panel(
            scaled,
            lines,
            border_color=border_color,
            scale_image=False,
        )

    def _render_mask_view(self, color_bgr, mask_u8) -> np.ndarray:
        if color_bgr is None and mask_u8 is None:
            frame = np.zeros((240, 320, 3), dtype=np.uint8)
            return self._compose_image_panel(
                frame,
                [
                    ("SAM3 Mask", (235, 245, 255)),
                    ("waiting for mask/color", (0, 200, 255)),
                ],
                border_color=(0, 140, 255),
            )

        if color_bgr is not None:
            frame = color_bgr.copy()
        else:
            frame = cv2.cvtColor(mask_u8, cv2.COLOR_GRAY2BGR)

        if mask_u8 is not None:
            mask_bool = mask_u8 > 0
            frame = (frame * 0.35).astype(np.uint8)
            frame[mask_bool] = np.array([0, 220, 0], dtype=np.uint8)

        status = self._common_status()
        mask_pixels = 0 if mask_u8 is None else int(np.count_nonzero(mask_u8))
        mask_active = mask_pixels > 0
        border_color = (0, 200, 0) if mask_active else (0, 140, 255)
        lines = [
            ("SAM3 Mask", (235, 245, 255)),
            (f"prompt = {status['prompt']}", (235, 245, 255)),
            (f"score = {status['score']:.3f}", (200, 220, 255)),
            (
                f"mask = {'active' if mask_active else 'empty'}",
                (0, 220, 0) if mask_active else (0, 140, 255),
            ),
            (f"pixels = {mask_pixels}", (185, 195, 205)),
        ]
        return self._compose_image_panel(frame, lines, border_color=border_color)

    def _render_point_cloud_view(self, points_xyz, keypoint_xyz, frame_id: str) -> np.ndarray:
        width = 960
        height = 640
        plot_w = 680
        plot_h = 600
        panel_x = plot_w + 20
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (10, 14, 20)

        for x in range(40, plot_w, 80):
            cv2.line(canvas, (x, 40), (x, plot_h), (26, 34, 44), 1)
        for y in range(40, plot_h, 80):
            cv2.line(canvas, (20, y), (plot_w, y), (26, 34, 44), 1)
        cv2.rectangle(canvas, (20, 40), (plot_w, plot_h), (90, 110, 130), 1)
        cv2.rectangle(canvas, (panel_x - 12, 40), (width - 16, plot_h), (36, 46, 58), -1)
        cv2.rectangle(canvas, (panel_x - 12, 40), (width - 16, plot_h), (90, 110, 130), 1)

        if points_xyz is None or points_xyz.shape[0] == 0:
            cv2.putText(
                canvas,
                "waiting for masked point cloud",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                canvas,
                "need mask + aligned depth + camera_info",
                (20, 76),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (170, 190, 210),
                2,
                cv2.LINE_AA,
            )
            return canvas

        points_xyz = np.asarray(points_xyz, dtype=np.float32)
        finite = np.all(np.isfinite(points_xyz), axis=1)
        points_xyz = points_xyz[finite]
        if points_xyz.shape[0] == 0:
            cv2.putText(
                canvas,
                "point cloud has no finite xyz samples",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
            return canvas

        min_xyz = np.min(points_xyz, axis=0)
        max_xyz = np.max(points_xyz, axis=0)
        extent_xyz = max_xyz - min_xyz
        center = np.median(points_xyz, axis=0, keepdims=True)
        centered = points_xyz - center

        yaw = np.deg2rad(-28.0)
        pitch = np.deg2rad(28.0)
        rot_z = np.array(
            [
                [np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        rot_x = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, np.cos(pitch), -np.sin(pitch)],
                [0.0, np.sin(pitch), np.cos(pitch)],
            ],
            dtype=np.float32,
        )
        rotated = centered @ rot_z.T @ rot_x.T

        span_x = max(float(np.max(np.abs(rotated[:, 0]))), 1e-3)
        span_y = max(float(np.max(np.abs(rotated[:, 1]))), 1e-3)
        scale = 0.44 * min(plot_w / (2.0 * span_x), plot_h / (2.0 * span_y))

        u = rotated[:, 0] * scale + plot_w * 0.5
        v = -rotated[:, 1] * scale + plot_h * 0.58

        depth = rotated[:, 2]
        depth_min = float(np.min(depth))
        depth_max = float(np.max(depth))
        if depth_max - depth_min < 1e-6:
            depth_norm = np.zeros_like(depth, dtype=np.uint8)
        else:
            depth_norm = np.clip(
                255.0 * (depth - depth_min) / (depth_max - depth_min),
                0.0,
                255.0,
            ).astype(np.uint8)
        color_map = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
        colors = cv2.applyColorMap(depth_norm.reshape(-1, 1), color_map).reshape(-1, 3)

        valid_px = (
            (u >= 20)
            & (u < plot_w)
            & (v >= 40)
            & (v < plot_h)
        )
        if np.any(valid_px):
            occupancy = np.zeros((height, width), dtype=np.uint8)
            for px, py in zip(u[valid_px], v[valid_px]):
                cv2.circle(
                    occupancy,
                    (int(round(float(px))), int(round(float(py)))),
                    4,
                    255,
                    -1,
                )
            kernel = np.ones((9, 9), dtype=np.uint8)
            occupancy = cv2.morphologyEx(occupancy, cv2.MORPH_CLOSE, kernel, iterations=2)
            contours, _hier = cv2.findContours(
                occupancy,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            region_layer = canvas.copy()
            cv2.drawContours(region_layer, contours, -1, (42, 120, 210), thickness=-1)
            canvas = cv2.addWeighted(region_layer, 0.28, canvas, 0.72, 0.0)
            cv2.drawContours(canvas, contours, -1, (110, 230, 255), thickness=2)

            uv_points = np.column_stack([u[valid_px], v[valid_px]]).astype(np.float32)
            x0, y0, box_w, box_h = cv2.boundingRect(uv_points.astype(np.int32))
            cv2.rectangle(
                canvas,
                (x0, y0),
                (x0 + box_w, y0 + box_h),
                (80, 255, 180),
                1,
            )

        order = np.argsort(depth)
        point_radius = 2 if points_xyz.shape[0] <= 2500 else 1
        for index in order:
            px = int(round(float(u[index])))
            py = int(round(float(v[index])))
            if 20 <= px < plot_w and 40 <= py < plot_h:
                cv2.circle(canvas, (px, py), point_radius + 1, (0, 0, 0), -1)
                cv2.circle(
                    canvas,
                    (px, py),
                    point_radius,
                    tuple(int(value) for value in colors[index]),
                    -1,
                )

        center_rot = np.zeros((1, 3), dtype=np.float32)
        center_u = int(round(float(center_rot[0, 0] * scale + plot_w * 0.5)))
        center_v = int(round(float(-center_rot[0, 1] * scale + plot_h * 0.58)))
        cv2.drawMarker(
            canvas,
            (center_u, center_v),
            (255, 255, 255),
            cv2.MARKER_CROSS,
            14,
            2,
            cv2.LINE_AA,
        )

        if keypoint_xyz is not None and np.all(np.isfinite(keypoint_xyz)):
            kp_rot = (
                np.asarray(keypoint_xyz, dtype=np.float32).reshape(1, 3) - center
            ) @ rot_z.T @ rot_x.T
            kp_u = int(round(float(kp_rot[0, 0] * scale + plot_w * 0.5)))
            kp_v = int(round(float(-kp_rot[0, 1] * scale + plot_h * 0.58)))
            if 20 <= kp_u < plot_w and 40 <= kp_v < plot_h:
                cv2.circle(canvas, (kp_u, kp_v), 11, (0, 0, 255), 3)
                cv2.circle(canvas, (kp_u, kp_v), 3, (0, 255, 255), -1)
                cv2.putText(
                    canvas,
                    "keypoint",
                    (kp_u + 12, max(48, kp_v - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

        status = self._common_status()
        lines = [
            "Masked Point Cloud Region",
            f"frame = {frame_id or 'unknown'}",
            f"points = {points_xyz.shape[0]}",
            f"prompt = {status['prompt']}",
            f"center = ({center[0, 0]:.3f}, {center[0, 1]:.3f}, {center[0, 2]:.3f}) m",
            f"extent = ({extent_xyz[0] * 100:.1f}, {extent_xyz[1] * 100:.1f}, {extent_xyz[2] * 100:.1f}) cm",
            f"z range = {min_xyz[2]:.3f} .. {max_xyz[2]:.3f} m",
        ]
        if keypoint_xyz is not None and np.all(np.isfinite(keypoint_xyz)):
            lines.append(
                f"keypoint = ({float(keypoint_xyz[0]):.3f}, {float(keypoint_xyz[1]):.3f}, {float(keypoint_xyz[2]):.3f}) m"
            )
        else:
            lines.append("keypoint = waiting")

        y = 68
        for line in lines:
            cv2.putText(
                canvas,
                line,
                (panel_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (235, 245, 255),
                2,
                cv2.LINE_AA,
            )
            y += 20

        inset_x0 = panel_x
        inset_y0 = 390
        inset_w = width - panel_x - 36
        inset_h = 180
        cv2.rectangle(
            canvas,
            (inset_x0, inset_y0),
            (inset_x0 + inset_w, inset_y0 + inset_h),
            (12, 18, 26),
            -1,
        )
        cv2.rectangle(
            canvas,
            (inset_x0, inset_y0),
            (inset_x0 + inset_w, inset_y0 + inset_h),
            (90, 110, 130),
            1,
        )
        cv2.putText(
            canvas,
            "top view: x-z",
            (inset_x0 + 8, inset_y0 + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (170, 220, 255),
            1,
            cv2.LINE_AA,
        )
        x_span = max(float(max_xyz[0] - min_xyz[0]), 1e-4)
        z_span = max(float(max_xyz[2] - min_xyz[2]), 1e-4)
        inset_u = (points_xyz[:, 0] - min_xyz[0]) / x_span
        inset_v = (points_xyz[:, 2] - min_xyz[2]) / z_span
        inset_u = inset_x0 + 14 + inset_u * max(inset_w - 28, 1)
        inset_v = inset_y0 + inset_h - 14 - inset_v * max(inset_h - 42, 1)
        for px, py in zip(inset_u, inset_v):
            cv2.circle(
                canvas,
                (int(round(float(px))), int(round(float(py)))),
                1,
                (0, 210, 255),
                -1,
            )
        return canvas

    def _masked_cloud_from_latest_rgbd(self, mask_u8, depth_m, intrinsics):
        if mask_u8 is None or depth_m is None or intrinsics is None:
            return None
        if mask_u8.shape[:2] != depth_m.shape[:2]:
            return None

        fx, fy, cx, cy = intrinsics
        stride = max(1, int(self.cloud_stride))
        depth_s = depth_m[::stride, ::stride]
        mask_s = mask_u8[::stride, ::stride] > 0
        height, width = depth_s.shape[:2]
        rows, cols = np.meshgrid(
            np.arange(height, dtype=np.float32) * stride,
            np.arange(width, dtype=np.float32) * stride,
            indexing="ij",
        )

        valid = mask_s
        valid &= np.isfinite(depth_s)
        valid &= depth_s > self.cloud_min_depth_m
        valid &= depth_s < self.cloud_max_depth_m
        if not np.any(valid):
            return np.zeros((0, 3), dtype=np.float32)

        z = depth_s[valid].astype(np.float32)
        u = cols[valid].astype(np.float32)
        v = rows[valid].astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points_xyz = np.stack([x, y, z], axis=1).astype(np.float32)

        if points_xyz.shape[0] > self.cloud_display_max_points:
            keep_indices = np.linspace(
                0,
                points_xyz.shape[0] - 1,
                num=self.cloud_display_max_points,
                dtype=np.int32,
            )
            points_xyz = points_xyz[keep_indices]
        return points_xyz

    def _any_window_closed(self) -> bool:
        window_names = [self.mask_window_name, self.window_name]
        if self.show_cloud_window:
            window_names.insert(1, self.cloud_window_name)
        for window_name in window_names:
            try:
                visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
            except cv2.error:
                visible = -1.0
            if visible < 1.0:
                return True
        return False

    def on_timer(self):
        if not self.window_enabled:
            return

        if self._any_window_closed():
            self.get_logger().info("viewer window closed by user; shutting down viewer node")
            self.window_enabled = False
            try:
                window_names = [self.mask_window_name, self.window_name]
                if self.show_cloud_window:
                    window_names.insert(1, self.cloud_window_name)
                for window_name in window_names:
                    cv2.destroyWindow(window_name)
                cv2.waitKey(1)
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
            return

        with self.lock:
            color_bgr = None if self.latest_color_bgr is None else self.latest_color_bgr.copy()
            mask_u8 = None if self.latest_mask_u8 is None else self.latest_mask_u8.copy()
            points_from_topic = (
                None
                if self.latest_points_xyz is None
                else np.array(self.latest_points_xyz, copy=True)
            )
            depth_m = None if self.latest_depth_m is None else np.array(self.latest_depth_m, copy=True)
            intrinsics = self.camera_intrinsics
            points_header = self.latest_points_header
            depth_header = self.latest_depth_header
            keypoint_xyz = None if self.keypoint_xyz is None else np.array(self.keypoint_xyz, copy=True)
            if self.prefer_overlay_image and self.latest_overlay_bgr is not None:
                keypoint_frame = self.latest_overlay_bgr.copy()
            elif color_bgr is not None:
                keypoint_frame = color_bgr.copy()
            else:
                keypoint_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        mask_view = self._render_mask_view(color_bgr, mask_u8)
        keypoint_view = self._annotate_keypoint(keypoint_frame)

        mask_view = self._fit_display_to_window(self.mask_window_name, mask_view)
        keypoint_view = self._fit_display_to_window(self.window_name, keypoint_view)

        cv2.imshow(self.mask_window_name, mask_view)
        if self.show_cloud_window:
            points_xyz = self._masked_cloud_from_latest_rgbd(mask_u8, depth_m, intrinsics)
            if points_xyz is None:
                points_xyz = points_from_topic
                frame_id = "" if points_header is None else str(points_header.frame_id)
            else:
                frame_id = "" if depth_header is None else str(depth_header.frame_id)
            cloud_view = self._render_point_cloud_view(
                points_xyz,
                keypoint_xyz,
                frame_id,
            )
            cloud_view = self._fit_display_to_window(self.cloud_window_name, cloud_view)
            cv2.imshow(self.cloud_window_name, cloud_view)
        cv2.imshow(self.window_name, keypoint_view)
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.window_enabled:
                window_names = [self.mask_window_name, self.window_name]
                if self.show_cloud_window:
                    window_names.insert(1, self.cloud_window_name)
                for window_name in window_names:
                    cv2.destroyWindow(window_name)
                cv2.waitKey(1)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingOverlayViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
