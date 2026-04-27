#!/usr/bin/env python3
"""Display a live SAM3 tracking overlay with prompt and 3D keypoint metadata."""

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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


class TrackingOverlayViewerNode(Node):
    """Show the fused overlay in a local OpenCV window for quick visual debugging."""

    def __init__(self):
        super().__init__("tracking_overlay_viewer_node")

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("overlay_topic", "/perception/keypoint_overlay")
        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("keypoint_topic", "/perception/keypoint_3d")
        self.declare_parameter("keypoint_px_topic", "/perception/keypoint_px")
        self.declare_parameter("valid_topic", "/perception/valid")
        self.declare_parameter("window_name", "SAM3 Tracking Overlay")
        self.declare_parameter("refresh_hz", 20.0)
        self.declare_parameter("keypoint_timeout_sec", 1.0)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.overlay_topic = str(self.get_parameter("overlay_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.score_topic = str(self.get_parameter("score_topic").value)
        self.keypoint_topic = str(self.get_parameter("keypoint_topic").value)
        self.keypoint_px_topic = str(self.get_parameter("keypoint_px_topic").value)
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.refresh_hz = float(self.get_parameter("refresh_hz").value)
        self.keypoint_timeout_sec = float(self.get_parameter("keypoint_timeout_sec").value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.latest_color_bgr = None
        self.latest_color_header = None
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
        self.logged_first_overlay = False

        self.window_enabled = bool(os.environ.get("DISPLAY"))
        if not self.window_enabled:
            self.get_logger().warning(
                "DISPLAY is not set; tracking overlay window is disabled"
            )
        else:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(Image, self.color_topic, self.on_color, image_qos)
        self.create_subscription(Image, self.overlay_topic, self.on_overlay, image_qos)
        self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
        self.create_subscription(Float32, self.score_topic, self.on_score, 10)
        self.create_subscription(PointStamped, self.keypoint_topic, self.on_keypoint, 10)
        self.create_subscription(PointStamped, self.keypoint_px_topic, self.on_keypoint_px, 10)
        self.create_subscription(Bool, self.valid_topic, self.on_valid, 10)

        self.timer = self.create_timer(1.0 / max(self.refresh_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            "tracking_overlay_viewer_node started. "
            f"color_topic={self.color_topic} overlay_topic={self.overlay_topic}"
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

    def _annotate(self, frame_bgr: np.ndarray) -> np.ndarray:
        with self.lock:
            prompt = self.current_prompt or "<none>"
            score = float(self.current_score)
            target_valid = bool(self.target_valid)
            keypoint_xyz = None if self.keypoint_xyz is None else np.array(self.keypoint_xyz, copy=True)
            keypoint_header = self.keypoint_header
            keypoint_uv = None if self.keypoint_uv is None else np.array(self.keypoint_uv, copy=True)
            keypoint_px_header = self.keypoint_px_header

        frame = frame_bgr.copy()
        height, width = frame.shape[:2]
        banner_h = 88
        cv2.rectangle(frame, (0, 0), (width - 1, banner_h), (20, 20, 20), thickness=-1)

        fresh = self._keypoint_is_fresh(keypoint_header)
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

        text_lines = [
            f"prompt = {prompt}",
            f"score = {score:.3f} | status = {status_text}",
            xyz_text,
            frame_text,
        ]
        colors = [
            (255, 255, 255),
            status_color,
            (255, 255, 0),
            (180, 180, 180),
        ]

        y = 22
        for line, color in zip(text_lines, colors):
            cv2.putText(
                frame,
                line,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.56,
                color,
                2,
                cv2.LINE_AA,
            )
            y += 20

        border_color = (0, 200, 0) if target_valid and fresh else (0, 0, 255)
        cv2.rectangle(frame, (0, 0), (width - 1, height - 1), border_color, thickness=2)

        if (
            target_valid
            and keypoint_uv is not None
            and self._keypoint_is_fresh(keypoint_px_header)
        ):
            u = int(round(float(keypoint_uv[0])))
            v = int(round(float(keypoint_uv[1])))
            if 0 <= u < width and 0 <= v < height:
                radius = 10
                cv2.line(frame, (u - radius, v), (u + radius, v), (0, 0, 255), 2)
                cv2.line(frame, (u, v - radius), (u, v + radius), (0, 0, 255), 2)
                cv2.circle(frame, (u, v), 4, (0, 255, 255), -1)
        return frame

    def on_timer(self):
        if not self.window_enabled:
            return

        try:
            visible = cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE)
        except cv2.error:
            visible = -1.0
        if visible < 1.0:
            self.get_logger().info("overlay window closed by user; shutting down viewer node")
            self.window_enabled = False
            try:
                cv2.destroyWindow(self.window_name)
                cv2.waitKey(1)
            except Exception:
                pass
            if rclpy.ok():
                rclpy.shutdown()
            return

        with self.lock:
            if self.latest_overlay_bgr is not None:
                frame = self.latest_overlay_bgr.copy()
            elif self.latest_color_bgr is not None:
                frame = self.latest_color_bgr.copy()
            else:
                cv2.waitKey(1)
                return

        annotated = self._annotate(frame)
        cv2.imshow(self.window_name, annotated)
        cv2.waitKey(1)

    def destroy_node(self):
        try:
            if self.window_enabled:
                cv2.destroyWindow(self.window_name)
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
