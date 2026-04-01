#!/usr/bin/env python3
"""Execute a structured task plan by updating the SAM3 prompt step by step."""

from __future__ import annotations

import json
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

from pybullet_ros2_sim.ik_utils import IiwaIkHelper
from pybullet_ros2_sim.task_plan_utils import plan_from_json


def transform_to_matrix(tf_msg):
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


def transform_point(transform, point_xyz):
    point = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0], dtype=np.float64)
    transformed = transform @ point
    return transformed[:3]


class LlmTaskExecutor(Node):
    """Run one step at a time and advance when the robot satisfies each step."""

    def __init__(self):
        super().__init__("llm_task_executor_node")

        self.declare_parameter("plan_topic", "/llm_task/plan_json")
        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("status_topic", "/llm_task/status")
        self.declare_parameter("tracking_enable_topic", "/llm_task/tracking_enabled")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("hover_offset_z", 0.10)
        self.declare_parameter("prompt_republish_sec", 1.0)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("target_timeout_sec", 1.0)
        self.declare_parameter("default_success_radius_m", 0.10)

        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.tracking_enable_topic = str(self.get_parameter("tracking_enable_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.hover_offset_z = float(self.get_parameter("hover_offset_z").value)
        self.prompt_republish_sec = float(self.get_parameter("prompt_republish_sec").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.default_success_radius_m = float(
            self.get_parameter("default_success_radius_m").value
        )

        self.lock = threading.Lock()
        self.q_now = None
        self.target_valid = False
        self.target_cam = None
        self.target_header = None
        self.plan = None
        self.step_cursor = 0
        self.step_started_ns = None
        self.step_satisfied_ns = None
        self.last_prompt_pub_ns = 0
        self.last_tracking_enabled = None

        self.pub_prompt = self.create_publisher(String, self.prompt_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_tracking_enable = self.create_publisher(Bool, self.tracking_enable_topic, 10)

        self.create_subscription(String, self.plan_topic, self.on_plan, 10)
        self.create_subscription(JointState, "/iiwa7/joint_states", self.on_joint_states, 10)
        self.create_subscription(Bool, "/perception/valid", self.on_valid, 10)
        self.create_subscription(PointStamped, "/perception/keypoint_3d", self.on_keypoint, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = IiwaIkHelper()

        self.timer = self.create_timer(1.0 / max(self.control_hz, 1e-6), self.on_timer)
        self._set_tracking_enabled(False)
        self.get_logger().info("llm_task_executor_node started")

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _publish_prompt(self, prompt: str):
        msg = String()
        msg.data = prompt
        self.pub_prompt.publish(msg)
        self.last_prompt_pub_ns = self._now_ns()

    def _set_tracking_enabled(self, enabled: bool):
        enabled = bool(enabled)
        if self.last_tracking_enabled is enabled:
            return
        msg = Bool()
        msg.data = enabled
        self.pub_tracking_enable.publish(msg)
        self.last_tracking_enabled = enabled

    def on_plan(self, msg: String):
        try:
            plan = plan_from_json(msg.data)
        except Exception as exc:
            self.get_logger().error(f"[EXEC] invalid plan: {exc}")
            self._set_tracking_enabled(False)
            self._publish_status("execution_failed: invalid_plan")
            return

        with self.lock:
            self.plan = plan
            self.step_cursor = 0
            self.step_started_ns = None
            self.step_satisfied_ns = None
            self.last_prompt_pub_ns = 0

        self.get_logger().info(
            f"[EXEC] loaded plan '{plan['task_summary']}' with {len(plan['steps'])} steps"
        )
        self._set_tracking_enabled(False)
        self._publish_status(f"execution_loaded: {plan['task_summary']}")

    def on_joint_states(self, msg: JointState):
        if len(msg.position) < 7:
            return
        with self.lock:
            self.q_now = list(msg.position[:7])

    def on_valid(self, msg: Bool):
        with self.lock:
            self.target_valid = bool(msg.data)

    def on_keypoint(self, msg: PointStamped):
        with self.lock:
            self.target_cam = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
            self.target_header = msg.header

    def _target_is_fresh(self, header) -> bool:
        if self.target_timeout_sec <= 0.0:
            return True
        if header is None:
            return False
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9
        return age <= self.target_timeout_sec

    def _advance_step(self):
        assert self.plan is not None
        completed_step = self.plan["steps"][self.step_cursor]
        self.step_cursor += 1
        self.step_started_ns = None
        self.step_satisfied_ns = None

        if self.step_cursor >= len(self.plan["steps"]):
            self.get_logger().info(f"[EXEC] plan complete: {self.plan['task_summary']}")
            self._set_tracking_enabled(False)
            self._publish_status(f"execution_complete: {self.plan['task_summary']}")
            return

        next_step = self.plan["steps"][self.step_cursor]
        self.get_logger().info(
            f"[EXEC] step {completed_step['step_index']} done -> step {next_step['step_index']}"
        )
        self._publish_status(
            f"step_started: {next_step['step_index']} {next_step['description']}"
        )

    def on_timer(self):
        with self.lock:
            plan = self.plan
            step_cursor = self.step_cursor
            q_now = None if self.q_now is None else list(self.q_now)
            target_valid = self.target_valid
            target_cam = None if self.target_cam is None else np.array(self.target_cam, copy=True)
            target_header = self.target_header

        if plan is None or not plan["steps"]:
            self._set_tracking_enabled(False)
            return
        if step_cursor >= len(plan["steps"]):
            self._set_tracking_enabled(False)
            return

        step = plan["steps"][step_cursor]
        now_ns = self._now_ns()

        if self.step_started_ns is None:
            self.step_started_ns = now_ns
            self.step_satisfied_ns = None
            self.get_logger().info(
                f"[EXEC] step {step['step_index']}: {step['description']}"
            )
            self._publish_status(
                f"step_started: {step['step_index']} {step['description']}"
            )

        if step["action"] == "wait":
            self._set_tracking_enabled(False)
            if (now_ns - self.step_started_ns) * 1e-9 >= step["wait_sec"]:
                self._advance_step()
            return

        self._set_tracking_enabled(True)
        if (now_ns - self.last_prompt_pub_ns) * 1e-9 >= self.prompt_republish_sec:
            self._publish_prompt(step["target_prompt"])

        if q_now is None or target_cam is None or target_header is None:
            return
        if not target_valid:
            return
        if not self._target_is_fresh(target_header):
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                target_header.frame_id,
                Time(),
            )
        except Exception as exc:
            self.get_logger().warn(f"[EXEC] TF lookup failed: {exc}")
            return

        transform = transform_to_matrix(tf_msg)
        point_world = transform_point(transform, target_cam)
        hover_target = np.array(
            [
                point_world[0],
                point_world[1],
                point_world[2] + self.hover_offset_z,
            ],
            dtype=np.float64,
        )

        ee_pos, _orn = self.ik.fk(q_now)
        distance = float(np.linalg.norm(np.array(ee_pos, dtype=np.float64) - hover_target))
        success_radius = (
            step["success_radius_m"]
            if step["success_radius_m"] > 0.0
            else self.default_success_radius_m
        )

        if distance <= success_radius:
            if self.step_satisfied_ns is None:
                self.step_satisfied_ns = now_ns
            if (now_ns - self.step_satisfied_ns) * 1e-9 >= step["dwell_sec"]:
                self._advance_step()
        else:
            self.step_satisfied_ns = None

    def destroy_node(self):
        try:
            self.ik.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LlmTaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
