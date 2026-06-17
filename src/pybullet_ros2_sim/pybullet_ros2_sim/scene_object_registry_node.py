#!/usr/bin/env python3
"""Maintain a compact scene object table from prompt-conditioned perception."""

from __future__ import annotations

import json
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener

from pybullet_ros2_sim.task_plan_utils import normalize_target_prompt


def transform_to_matrix(tf_msg):
    """Convert a TF transform into a homogeneous 4x4 matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    transform[:3, 3] = [t.x, t.y, t.z]
    return transform


def transform_point(transform: np.ndarray, point_xyz: np.ndarray):
    """Apply a homogeneous transform to a single 3D point."""
    point = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0], dtype=np.float64)
    transformed = transform @ point
    return transformed[:3]


class SceneObjectRegistry(Node):
    """Store prompt-conditioned object observations in a planner-friendly table."""

    def __init__(self):
        super().__init__("scene_object_registry_node")

        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("keypoint_topic", "/perception/keypoint_3d")
        self.declare_parameter("valid_topic", "/perception/valid")
        self.declare_parameter("scene_topic", "/scene/objects_json")
        self.declare_parameter("status_topic", "/scene/registry_status")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("observation_timeout_sec", 1.5)
        self.declare_parameter("visible_timeout_sec", 2.0)
        self.declare_parameter("entry_ttl_sec", 60.0)
        self.declare_parameter("open_vocabulary_targets", False)

        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.score_topic = str(self.get_parameter("score_topic").value)
        self.keypoint_topic = str(self.get_parameter("keypoint_topic").value)
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.scene_topic = str(self.get_parameter("scene_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.observation_timeout_sec = float(
            self.get_parameter("observation_timeout_sec").value
        )
        self.visible_timeout_sec = float(self.get_parameter("visible_timeout_sec").value)
        self.entry_ttl_sec = float(self.get_parameter("entry_ttl_sec").value)
        self.open_vocabulary_targets = bool(
            self.get_parameter("open_vocabulary_targets").value
        )

        self.lock = threading.Lock()
        self.current_prompt = ""
        self.current_score = 0.0
        self.target_valid = False
        self.latest_keypoint = None
        self.latest_keypoint_header = None
        self.objects = {}
        self._last_status_log_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
        self.sub_score = self.create_subscription(Float32, self.score_topic, self.on_score, 10)
        self.sub_keypoint = self.create_subscription(
            PointStamped, self.keypoint_topic, self.on_keypoint, 10
        )
        self.sub_valid = self.create_subscription(Bool, self.valid_topic, self.on_valid, 10)

        self.pub_scene = self.create_publisher(String, self.scene_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            f"scene_object_registry_node started. scene_topic={self.scene_topic}"
        )

    def _log_status(self, message: str):
        now = time.time()
        if now - self._last_status_log_time < 2.0:
            return
        self._last_status_log_time = now
        self.get_logger().info(message)

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
            self.latest_keypoint = np.array(
                [msg.point.x, msg.point.y, msg.point.z],
                dtype=np.float64,
            )
            self.latest_keypoint_header = msg.header

    def _keypoint_is_fresh(self, header) -> bool:
        if self.observation_timeout_sec <= 0.0:
            return True
        if header is None:
            return False
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9
        return age <= self.observation_timeout_sec

    def _transform_to_world(self, header, point_cam: np.ndarray):
        tf_msg = self.tf_buffer.lookup_transform(self.target_frame, header.frame_id, Time())
        transform = transform_to_matrix(tf_msg)
        return transform_point(transform, point_cam)

    def _update_active_object(self):
        with self.lock:
            prompt = self.current_prompt
            score = float(self.current_score)
            target_valid = bool(self.target_valid)
            point_cam = None if self.latest_keypoint is None else np.array(self.latest_keypoint, copy=True)
            header = self.latest_keypoint_header

        label = normalize_target_prompt(
            prompt,
            allow_open_vocabulary=self.open_vocabulary_targets,
        )
        if not label or not target_valid or point_cam is None or header is None:
            return
        if not self._keypoint_is_fresh(header):
            return

        position_world = None
        try:
            position_world = self._transform_to_world(header, point_cam)
        except Exception as exc:
            self._log_status(f"registry TF lookup pending: {exc}")

        stamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
        now_sec = time.time()

        with self.lock:
            entry = self.objects.get(label, {})
            if int(entry.get("last_source_stamp_ns", -1)) == stamp_ns:
                return

            entry["object_id"] = label.replace(" ", "_")
            entry["label"] = label
            entry["display_name"] = label
            entry["source_prompt"] = prompt
            entry["confidence"] = score
            entry["frame_id"] = str(header.frame_id)
            entry["position_camera"] = {
                "x": float(point_cam[0]),
                "y": float(point_cam[1]),
                "z": float(point_cam[2]),
            }
            if position_world is not None:
                entry["position_world"] = {
                    "x": float(position_world[0]),
                    "y": float(position_world[1]),
                    "z": float(position_world[2]),
                }
            else:
                entry["position_world"] = None

            entry["first_seen_sec"] = float(entry.get("first_seen_sec", now_sec))
            entry["last_seen_sec"] = float(now_sec)
            entry["last_source_stamp_ns"] = stamp_ns
            entry["observation_count"] = int(entry.get("observation_count", 0)) + 1
            self.objects[label] = entry
            observation_count = entry["observation_count"]

        self._log_status(
            f"updated scene object '{label}' score={score:.3f} obs={observation_count}"
        )

    def _snapshot_scene(self):
        now_sec = time.time()
        objects = []

        with self.lock:
            stale_labels = []
            for label, entry in self.objects.items():
                last_seen_sec = float(entry.get("last_seen_sec", 0.0))
                age_sec = max(0.0, now_sec - last_seen_sec)

                if self.entry_ttl_sec > 0.0 and age_sec > self.entry_ttl_sec:
                    stale_labels.append(label)
                    continue

                item = dict(entry)
                item["last_seen_age_sec"] = age_sec
                item["visible"] = age_sec <= self.visible_timeout_sec
                objects.append(item)

            for label in stale_labels:
                self.objects.pop(label, None)

        objects.sort(
            key=lambda item: (
                not bool(item.get("visible", False)),
                float(item.get("last_seen_age_sec", 0.0)),
                item.get("label", ""),
            )
        )
        return {
            "target_frame": self.target_frame,
            "updated_at_sec": now_sec,
            "objects": objects,
        }

    def on_timer(self):
        self._update_active_object()
        scene = self._snapshot_scene()

        scene_msg = String()
        scene_msg.data = json.dumps(scene, ensure_ascii=False)
        self.pub_scene.publish(scene_msg)

        status_msg = String()
        visible_count = sum(1 for item in scene["objects"] if bool(item.get("visible", False)))
        status_msg.data = f"scene_registry: {visible_count} visible / {len(scene['objects'])} total"
        self.pub_status.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SceneObjectRegistry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
