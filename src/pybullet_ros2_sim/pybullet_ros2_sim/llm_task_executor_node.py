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
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import PointStamped, Vector3Stamped
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


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


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
        self.declare_parameter("control_hz", 200.0)
        self.declare_parameter("target_timeout_sec", 1.0)
        self.declare_parameter("default_success_radius_m", 0.06)
        self.declare_parameter("target_reacquire_delay_sec", 0.75)
        self.declare_parameter("min_target_switch_distance_m", 0.08)
        self.declare_parameter("use_confirmed_static_target", True)
        self.declare_parameter("use_static_tray_center_target", False)
        self.declare_parameter("prefer_visual_tray_center_keypoint", True)
        self.declare_parameter("visual_tray_center_max_distance_m", 0.19)
        self.declare_parameter("static_tray_center_fallback_sec", 3.0)
        self.declare_parameter("force_static_tray_release_for_medical_objects", True)
        self.declare_parameter("static_tray_center_world", [0.555, 0.150, 0.351])
        self.declare_parameter("static_tray_plane_normal_world", [0.0, 0.0, 1.0])
        self.declare_parameter("static_target_override_topic", "/llm_task/static_target_override")
        self.declare_parameter(
            "static_target_override_enabled_topic",
            "/llm_task/static_target_override_enabled",
        )
        self.declare_parameter(
            "static_target_plane_normal_topic",
            "/llm_task/static_target_plane_normal",
        )
        self.declare_parameter("open_vocabulary_targets", False)
        self.declare_parameter("enable_grasp_actions", False)
        self.declare_parameter("gripper_command_topic", "/iiwa7/gripper_command")
        self.declare_parameter("gripper_status_topic", "/iiwa7/gripper_status")
        self.declare_parameter(
            "tracking_hover_offset_topic",
            "/llm_task/tracking_hover_offset_z",
        )
        self.declare_parameter("grasp_lift_m", 0.12)
        self.declare_parameter("grasp_pregrasp_offset_m", -1.0)
        self.declare_parameter("grasp_pregrasp_success_radius_m", 0.035)
        self.declare_parameter("grasp_pregrasp_hold_sec", 0.25)
        self.declare_parameter("grasp_approach_offset_z", -1.0)
        self.declare_parameter("grasp_lift_delay_sec", 0.35)
        self.declare_parameter("grasp_confirm_hold_sec", 0.25)
        self.declare_parameter("grasp_lift_hold_sec", 1.2)
        self.declare_parameter("grasp_close_success_radius_m", 0.10)
        self.declare_parameter("grasp_lift_success_radius_m", 0.05)
        self.declare_parameter("track_gripper_center", False)
        self.declare_parameter("gripper_center_offset_link6", [0.0, 0.0, 0.0])
        self.declare_parameter("min_gripper_center_z", -1.0)
        self.declare_parameter("place_hover_offset_z", 0.12)
        self.declare_parameter("release_hover_offset_z", 0.035)
        self.declare_parameter("release_success_radius_m", 0.055)
        self.declare_parameter("release_approach_timeout_sec", 3.0)
        self.declare_parameter("place_success_radius_m", 0.040)
        self.declare_parameter("release_lower_hold_sec", 1.0)
        self.declare_parameter("release_open_wait_sec", 0.5)
        self.declare_parameter("status_heartbeat_sec", 1.0)

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
        self.target_reacquire_delay_sec = float(
            self.get_parameter("target_reacquire_delay_sec").value
        )
        self.min_target_switch_distance_m = float(
            self.get_parameter("min_target_switch_distance_m").value
        )
        self.use_confirmed_static_target = bool(
            self.get_parameter("use_confirmed_static_target").value
        )
        self.use_static_tray_center_target = bool(
            self.get_parameter("use_static_tray_center_target").value
        )
        self.prefer_visual_tray_center_keypoint = bool(
            self.get_parameter("prefer_visual_tray_center_keypoint").value
        )
        self.visual_tray_center_max_distance_m = max(
            0.0,
            float(self.get_parameter("visual_tray_center_max_distance_m").value),
        )
        self.static_tray_center_fallback_sec = float(
            self.get_parameter("static_tray_center_fallback_sec").value
        )
        self.force_static_tray_release_for_medical_objects = bool(
            self.get_parameter("force_static_tray_release_for_medical_objects").value
        )
        self.static_tray_center_world = self._parameter_vector(
            "static_tray_center_world",
            [0.555, 0.150, 0.351],
        )
        self.static_tray_plane_normal_world = self._normalized_parameter_vector(
            "static_tray_plane_normal_world",
            [0.0, 0.0, 1.0],
        )
        self.static_target_override_topic = str(
            self.get_parameter("static_target_override_topic").value
        )
        self.static_target_override_enabled_topic = str(
            self.get_parameter("static_target_override_enabled_topic").value
        )
        self.static_target_plane_normal_topic = str(
            self.get_parameter("static_target_plane_normal_topic").value
        )
        self.open_vocabulary_targets = bool(
            self.get_parameter("open_vocabulary_targets").value
        )
        self.enable_grasp_actions = bool(self.get_parameter("enable_grasp_actions").value)
        self.gripper_command_topic = str(self.get_parameter("gripper_command_topic").value)
        self.gripper_status_topic = str(self.get_parameter("gripper_status_topic").value)
        self.tracking_hover_offset_topic = str(
            self.get_parameter("tracking_hover_offset_topic").value
        )
        self.grasp_lift_m = float(self.get_parameter("grasp_lift_m").value)
        self.grasp_pregrasp_offset_m = float(
            self.get_parameter("grasp_pregrasp_offset_m").value
        )
        self.grasp_pregrasp_success_radius_m = float(
            self.get_parameter("grasp_pregrasp_success_radius_m").value
        )
        self.grasp_pregrasp_hold_sec = float(
            self.get_parameter("grasp_pregrasp_hold_sec").value
        )
        self.grasp_approach_offset_z = float(
            self.get_parameter("grasp_approach_offset_z").value
        )
        self.grasp_lift_delay_sec = float(self.get_parameter("grasp_lift_delay_sec").value)
        self.grasp_confirm_hold_sec = float(
            self.get_parameter("grasp_confirm_hold_sec").value
        )
        self.grasp_lift_hold_sec = float(self.get_parameter("grasp_lift_hold_sec").value)
        self.grasp_close_success_radius_m = float(
            self.get_parameter("grasp_close_success_radius_m").value
        )
        self.grasp_lift_success_radius_m = float(
            self.get_parameter("grasp_lift_success_radius_m").value
        )
        self.track_gripper_center = bool(self.get_parameter("track_gripper_center").value)
        self.gripper_center_offset_link6 = np.array(
            [float(value) for value in self.get_parameter("gripper_center_offset_link6").value][
                :3
            ],
            dtype=np.float64,
        )
        if self.gripper_center_offset_link6.shape[0] < 3:
            self.gripper_center_offset_link6 = np.pad(
                self.gripper_center_offset_link6,
                (0, 3 - self.gripper_center_offset_link6.shape[0]),
            )
        self.min_gripper_center_z = float(self.get_parameter("min_gripper_center_z").value)
        self.place_hover_offset_z = float(self.get_parameter("place_hover_offset_z").value)
        self.release_hover_offset_z = float(self.get_parameter("release_hover_offset_z").value)
        self.release_success_radius_m = float(
            self.get_parameter("release_success_radius_m").value
        )
        self.release_approach_timeout_sec = float(
            self.get_parameter("release_approach_timeout_sec").value
        )
        self.place_success_radius_m = float(self.get_parameter("place_success_radius_m").value)
        self.release_lower_hold_sec = float(self.get_parameter("release_lower_hold_sec").value)
        self.release_open_wait_sec = float(self.get_parameter("release_open_wait_sec").value)
        self.status_heartbeat_sec = float(self.get_parameter("status_heartbeat_sec").value)

        self.lock = threading.Lock()
        self.q_now = None
        self.target_valid = False
        self.target_cam = None
        self.target_header = None
        self.target_received_ns = 0
        self.plan = None
        self.step_cursor = 0
        self.step_started_ns = None
        self.step_satisfied_ns = None
        self.step_prompt_sent_ns = None
        self.step_target_confirmed_ns = None
        self.last_prompt_pub_ns = 0
        self.last_tracking_enabled = None
        self.last_completed_target_prompt = ""
        self.last_completed_target_cam = None
        self.last_completed_target_frame = ""
        self.last_completed_static_target = False
        self.held_object_prompt = ""
        self.step_confirmed_target_cam = None
        self.step_confirmed_target_frame = ""
        self.step_static_target = False
        self.step_gripper_command_sent = False
        self.step_gripper_command_sent_ns = None
        self.step_lower_command_sent = False
        self.step_grasp_descent_started = False
        self.step_grasp_lift_started_ns = None
        self.step_pregrasp_satisfied_ns = None
        self.gripper_holding_object = False
        self.gripper_holding_confirmed_ns = None
        self.gripper_status_text = ""
        self.gripper_status_received_ns = 0
        self.last_tracking_hover_offset_z = None
        self.last_periodic_status_text = ""
        self.last_periodic_status_ns = 0
        self.last_static_target_override_enabled = None

        self.pub_prompt = self.create_publisher(String, self.prompt_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_tracking_enable = self.create_publisher(Bool, self.tracking_enable_topic, 10)
        self.pub_static_target_override = self.create_publisher(
            PointStamped,
            self.static_target_override_topic,
            10,
        )
        self.pub_static_target_override_enabled = self.create_publisher(
            Bool,
            self.static_target_override_enabled_topic,
            10,
        )
        self.pub_static_target_plane_normal = self.create_publisher(
            Vector3Stamped,
            self.static_target_plane_normal_topic,
            10,
        )
        self.pub_gripper_command = self.create_publisher(
            String,
            self.gripper_command_topic,
            10,
        )
        self.pub_tracking_hover_offset = self.create_publisher(
            Float64,
            self.tracking_hover_offset_topic,
            10,
        )

        self.create_subscription(String, self.plan_topic, self.on_plan, 10)
        self.create_subscription(JointState, "/iiwa7/joint_states", self.on_joint_states, 10)
        self.create_subscription(Bool, "/perception/valid", self.on_valid, 10)
        self.create_subscription(PointStamped, "/perception/keypoint_3d", self.on_keypoint, 10)
        self.create_subscription(String, self.gripper_status_topic, self.on_gripper_status, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = IiwaIkHelper()

        self.timer = self.create_timer(1.0 / max(self.control_hz, 1e-6), self.on_timer)
        self._set_static_target_override(False)
        self._set_tracking_enabled(False)
        self.get_logger().info("llm_task_executor_node started")

    def _parameter_vector(self, name: str, default):
        try:
            values = list(self.get_parameter(name).value)
        except TypeError:
            values = list(default)
        vector = [float(value) for value in values[:3]]
        if len(vector) < 3:
            vector.extend(float(default[index]) for index in range(len(vector), 3))
        return np.array(vector, dtype=np.float64)

    def _normalized_parameter_vector(self, name: str, default):
        vector = self._parameter_vector(name, default)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-9:
            return np.array(default, dtype=np.float64)
        return vector / norm

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _publish_status_periodic(self, text: str, *, publish_on_change: bool = True):
        now_ns = self._now_ns()
        heartbeat_sec = max(self.status_heartbeat_sec, 0.0)
        if heartbeat_sec <= 0.0:
            self._publish_status(text)
            self.last_periodic_status_text = text
            self.last_periodic_status_ns = now_ns
            return
        if (
            (publish_on_change and text != self.last_periodic_status_text)
            or (now_ns - self.last_periodic_status_ns) * 1e-9 >= heartbeat_sec
        ):
            self._publish_status(text)
            self.last_periodic_status_text = text
            self.last_periodic_status_ns = now_ns

    def _tracking_status_text(
        self,
        step: dict,
        *,
        distance: float | None = None,
        success_radius: float | None = None,
        dwell_elapsed: float | None = None,
        target_valid: bool | None = None,
        target_fresh: bool | None = None,
        reason: str = "",
    ) -> str:
        parts = [f"step_tracking_target: {step['step_index']} {step['target_prompt']}"]
        if distance is not None:
            parts.append(f"dist={distance:.3f}m")
        if success_radius is not None:
            parts.append(f"radius={success_radius:.3f}m")
        if dwell_elapsed is not None:
            parts.append(f"dwell={dwell_elapsed:.2f}/{step['dwell_sec']:.2f}s")
        if target_valid is not None:
            parts.append(f"valid={str(bool(target_valid)).lower()}")
        if target_fresh is not None:
            parts.append(f"fresh={str(bool(target_fresh)).lower()}")
        if reason:
            parts.append(f"reason={reason}")
        return " ".join(parts)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _publish_prompt(self, prompt: str):
        msg = String()
        msg.data = prompt
        self.pub_prompt.publish(msg)
        self.last_prompt_pub_ns = self._now_ns()

    def _publish_gripper_command(self, command: str):
        msg = String()
        msg.data = str(command)
        self.pub_gripper_command.publish(msg)

    def _set_tracking_hover_offset(self, offset_z: float):
        offset_z = float(offset_z)
        if (
            self.last_tracking_hover_offset_z is not None
            and abs(offset_z - self.last_tracking_hover_offset_z) < 1e-4
        ):
            return
        msg = Float64()
        msg.data = offset_z
        self.pub_tracking_hover_offset.publish(msg)
        self.last_tracking_hover_offset_z = offset_z

    def _set_tracking_enabled(self, enabled: bool):
        enabled = bool(enabled)
        if self.last_tracking_enabled is enabled:
            return
        msg = Bool()
        msg.data = enabled
        self.pub_tracking_enable.publish(msg)
        self.last_tracking_enabled = enabled

    def _set_static_target_override(self, enabled: bool):
        enabled = bool(enabled)
        if self.last_static_target_override_enabled is enabled:
            return
        msg = Bool()
        msg.data = enabled
        self.pub_static_target_override_enabled.publish(msg)
        self.last_static_target_override_enabled = enabled

    def _publish_static_target_override(self, point_world, normal_world=None):
        now = self.get_clock().now().to_msg()
        point_msg = PointStamped()
        point_msg.header.stamp = now
        point_msg.header.frame_id = self.target_frame
        point_msg.point.x = float(point_world[0])
        point_msg.point.y = float(point_world[1])
        point_msg.point.z = float(point_world[2])
        self.pub_static_target_override.publish(point_msg)

        normal = (
            self.static_tray_plane_normal_world
            if normal_world is None
            else np.asarray(normal_world, dtype=np.float64)
        )
        norm = float(np.linalg.norm(normal))
        if norm > 1e-9:
            normal = normal / norm
        normal_msg = Vector3Stamped()
        normal_msg.header.stamp = now
        normal_msg.header.frame_id = self.target_frame
        normal_msg.vector.x = float(normal[0])
        normal_msg.vector.y = float(normal[1])
        normal_msg.vector.z = float(normal[2])
        self.pub_static_target_plane_normal.publish(normal_msg)
        self._set_static_target_override(True)

    def _is_tray_target_prompt(self, prompt: str) -> bool:
        lowered = str(prompt or "").lower()
        keywords = (
            "tray",
            "plate",
            "container",
            "sorting tray",
            "托盘",
            "盘子",
            "盘",
        )
        return any(keyword in lowered for keyword in keywords)

    def _is_medical_target_prompt(self, prompt: str) -> bool:
        lowered = str(prompt or "").lower()
        keywords = (
            "surgical instrument",
            "instrument",
            "scissors",
            "forceps",
            "scalpel",
            "器械",
            "剪刀",
            "镊子",
            "手术刀",
        )
        return any(keyword in lowered for keyword in keywords)

    def _tray_visual_center_is_plausible(self, point_world) -> tuple[bool, float]:
        if self.visual_tray_center_max_distance_m <= 0.0:
            return True, 0.0
        point = np.asarray(point_world, dtype=np.float64).reshape(3)
        distance_xy = float(np.linalg.norm(point[:2] - self.static_tray_center_world[:2]))
        return distance_xy <= self.visual_tray_center_max_distance_m, distance_xy

    def _static_target_for_step(self, step: dict):
        if not self.use_static_tray_center_target:
            return None
        if not self._is_tray_target_prompt(str(step.get("target_prompt", ""))):
            return None
        return (
            np.array(self.static_tray_center_world, copy=True),
            np.array(self.static_tray_plane_normal_world, copy=True),
        )

    def _transform_point_to_target_frame(self, point, frame_id: str):
        source_frame = str(frame_id or "")
        if not source_frame or source_frame == self.target_frame:
            return np.asarray(point, dtype=np.float64)
        tf_msg = self.tf_buffer.lookup_transform(self.target_frame, source_frame, Time())
        return transform_point(transform_to_matrix(tf_msg), point)

    def on_plan(self, msg: String):
        try:
            plan = plan_from_json(
                msg.data,
                allow_open_vocabulary=self.open_vocabulary_targets,
                allow_grasp_actions=self.enable_grasp_actions,
            )
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
            self.step_prompt_sent_ns = None
            self.step_target_confirmed_ns = None
            self.last_prompt_pub_ns = 0
            self.last_completed_target_prompt = ""
            self.last_completed_target_cam = None
            self.last_completed_target_frame = ""
            self.last_completed_static_target = False
            self.held_object_prompt = ""
            self.step_confirmed_target_cam = None
            self.step_confirmed_target_frame = ""
            self.step_static_target = False
            self.step_gripper_command_sent = False
            self.step_gripper_command_sent_ns = None
            self.step_lower_command_sent = False
            self.step_grasp_descent_started = False
            self.step_grasp_lift_started_ns = None
            self.step_pregrasp_satisfied_ns = None
            self.gripper_holding_object = False
            self.gripper_holding_confirmed_ns = None
            self.gripper_status_text = ""
            self.gripper_status_received_ns = 0
            self.last_tracking_hover_offset_z = None
            self.last_periodic_status_text = ""
            self.last_periodic_status_ns = 0
            self.last_static_target_override_enabled = None

        self.get_logger().info(
            f"[EXEC] loaded plan '{plan['task_summary']}' with {len(plan['steps'])} steps"
        )
        self._set_static_target_override(False)
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
            self.target_received_ns = self._now_ns()

    def on_gripper_status(self, msg: String):
        text = str(msg.data)
        lowered = text.lower()
        with self.lock:
            now_ns = self._now_ns()
            holding = self.gripper_holding_object
            if (
                "gripper_closed:" in lowered
                and ("grasped_body=" in lowered or "holding_body=" in lowered)
            ):
                holding = True
            elif (
                "gripper_open:" in lowered
                or "gripper_closing:" in lowered
                or "gripper_lower:" in lowered
                or "no_object" in lowered
                or "no_graspable" in lowered
            ):
                holding = False
            if holding and not self.gripper_holding_object:
                self.gripper_holding_confirmed_ns = now_ns
            elif not holding:
                self.gripper_holding_confirmed_ns = None
            self.gripper_status_text = text
            self.gripper_status_received_ns = now_ns
            self.gripper_holding_object = holding

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
        if completed_step["action"] in ("hover_target", "grasp_target") and self.step_confirmed_target_cam is not None:
            self.last_completed_target_prompt = str(completed_step.get("target_prompt", ""))
            self.last_completed_target_cam = np.array(self.step_confirmed_target_cam, copy=True)
            self.last_completed_target_frame = str(self.step_confirmed_target_frame)
            self.last_completed_static_target = bool(self.step_static_target)
        if completed_step["action"] == "grasp_target":
            self.held_object_prompt = str(completed_step.get("target_prompt", ""))
        elif completed_step["action"] == "release_gripper":
            self.held_object_prompt = ""
        self.step_cursor += 1
        self.step_started_ns = None
        self.step_satisfied_ns = None
        self.step_prompt_sent_ns = None
        self.step_target_confirmed_ns = None
        self.step_confirmed_target_cam = None
        self.step_confirmed_target_frame = ""
        self.step_static_target = False
        self.step_gripper_command_sent = False
        self.step_gripper_command_sent_ns = None
        self.step_lower_command_sent = False
        self.step_grasp_descent_started = False
        self.step_grasp_lift_started_ns = None
        self.step_pregrasp_satisfied_ns = None
        self.last_periodic_status_text = ""
        self.last_periodic_status_ns = 0

        if self.step_cursor >= len(self.plan["steps"]):
            self.get_logger().info(f"[EXEC] plan complete: {self.plan['task_summary']}")
            self._set_static_target_override(False)
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

    def _target_is_ready_for_current_step(
        self,
        *,
        step: dict,
        target_valid: bool,
        target_cam,
        target_header,
        target_received_ns: int,
    ) -> bool:
        if not target_valid:
            return False
        if target_cam is None:
            return False
        if target_header is None:
            return False
        if not self._target_is_fresh(target_header):
            return False
        if self.step_prompt_sent_ns is None:
            return False

        min_confirm_ns = self.step_prompt_sent_ns + int(
            max(self.target_reacquire_delay_sec, 0.0) * 1e9
        )
        if target_received_ns < min_confirm_ns:
            return False

        same_prompt = str(step.get("target_prompt", "")) == self.last_completed_target_prompt
        if same_prompt:
            return True

        if (
            self.last_completed_target_cam is None
            or self.min_target_switch_distance_m <= 0.0
            or str(target_header.frame_id) != self.last_completed_target_frame
        ):
            return True

        switch_distance = float(
            np.linalg.norm(
                np.asarray(target_cam, dtype=np.float64)
                - np.asarray(self.last_completed_target_cam, dtype=np.float64)
            )
        )
        return switch_distance >= self.min_target_switch_distance_m

    def _target_wait_reason(
        self,
        *,
        step: dict,
        target_valid: bool,
        target_cam,
        target_header,
        target_received_ns: int,
    ) -> str:
        if not target_valid:
            return "valid_false"
        if target_cam is None:
            return "no_keypoint"
        if target_header is None:
            return "no_keypoint_header"
        if not self._target_is_fresh(target_header):
            return f"stale_keypoint>{self.target_timeout_sec:.2f}s"
        if self.step_prompt_sent_ns is None:
            return "prompt_not_sent"

        min_confirm_ns = self.step_prompt_sent_ns + int(
            max(self.target_reacquire_delay_sec, 0.0) * 1e9
        )
        if target_received_ns < min_confirm_ns:
            wait_sec = max((min_confirm_ns - target_received_ns) * 1e-9, 0.0)
            return f"waiting_reacquire:{wait_sec:.2f}s"

        same_prompt = str(step.get("target_prompt", "")) == self.last_completed_target_prompt
        if same_prompt:
            return "ready"

        if (
            self.last_completed_target_cam is None
            or self.min_target_switch_distance_m <= 0.0
            or str(target_header.frame_id) != self.last_completed_target_frame
        ):
            return "ready"

        switch_distance = float(
            np.linalg.norm(
                np.asarray(target_cam, dtype=np.float64)
                - np.asarray(self.last_completed_target_cam, dtype=np.float64)
            )
        )
        return f"waiting_switch_distance:{switch_distance:.3f}m"

    def _next_action_is_release(self) -> bool:
        if self.plan is None:
            return False
        next_index = self.step_cursor + 1
        if next_index >= len(self.plan["steps"]):
            return False
        return self.plan["steps"][next_index].get("action") == "release_gripper"

    def on_timer(self):
        with self.lock:
            plan = self.plan
            step_cursor = self.step_cursor
            q_now = None if self.q_now is None else list(self.q_now)
            target_valid = self.target_valid
            target_cam = None if self.target_cam is None else np.array(self.target_cam, copy=True)
            target_header = self.target_header
            target_received_ns = self.target_received_ns
            gripper_holding_object = self.gripper_holding_object
            gripper_holding_confirmed_ns = self.gripper_holding_confirmed_ns
            gripper_status_text = self.gripper_status_text

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
            self.step_prompt_sent_ns = None
            self.step_target_confirmed_ns = None
            self.get_logger().info(
                f"[EXEC] step {step['step_index']}: {step['description']}"
            )
            self._publish_status(
                f"step_started: {step['step_index']} {step['description']}"
            )

        if step["action"] == "release_gripper":
            elapsed = (now_ns - self.step_started_ns) * 1e-9
            release_target_cam = (
                None
                if self.last_completed_target_cam is None
                else np.array(self.last_completed_target_cam, copy=True)
            )
            release_target_frame = str(self.last_completed_target_frame or "")
            release_target_prompt = str(self.last_completed_target_prompt or "")
            release_target_is_static = bool(self.last_completed_static_target)
            release_has_target = release_target_cam is not None and bool(release_target_frame)
            release_distance = None
            force_static_tray_release = (
                self.force_static_tray_release_for_medical_objects
                and self._is_medical_target_prompt(self.held_object_prompt)
                and not self._is_tray_target_prompt(release_target_prompt)
            )

            if force_static_tray_release:
                release_target_cam = np.array(self.static_tray_center_world, copy=True)
                release_target_frame = self.target_frame
                release_target_prompt = "white sorting tray center"
                release_target_is_static = True
                release_has_target = True

            if release_has_target:
                if release_target_is_static:
                    self._publish_static_target_override(
                        release_target_cam,
                        self.static_tray_plane_normal_world,
                    )
                else:
                    self._set_static_target_override(False)
                self._set_tracking_hover_offset(self.release_hover_offset_z)
                self._set_tracking_enabled(True)
                if q_now is not None:
                    try:
                        release_point_world = self._transform_point_to_target_frame(
                            release_target_cam,
                            release_target_frame,
                        )
                        release_target_world = np.array(
                            [
                                release_point_world[0],
                                release_point_world[1],
                                release_point_world[2] + self.release_hover_offset_z,
                            ],
                            dtype=np.float64,
                        )
                        if self.track_gripper_center and self.min_gripper_center_z >= 0.0:
                            release_target_world[2] = max(
                                release_target_world[2],
                                self.min_gripper_center_z,
                            )
                        ee_pos, ee_orn = self.ik.fk(q_now)
                        control_pos = np.array(ee_pos, dtype=np.float64)
                        if self.track_gripper_center:
                            control_pos = (
                                control_pos
                                + quat_to_matrix(ee_orn) @ self.gripper_center_offset_link6
                            )
                        release_distance = float(np.linalg.norm(control_pos - release_target_world))
                    except Exception as exc:
                        self.get_logger().warn(f"[EXEC] release target TF/FK failed: {exc}")
            else:
                self._set_static_target_override(False)
                self._set_tracking_enabled(False)
                self._set_tracking_hover_offset(self.hover_offset_z)

            if not self.step_lower_command_sent:
                self._publish_gripper_command("lower")
                self.step_lower_command_sent = True
                if release_has_target:
                    self._publish_status(
                        f"gripper_lowering_to_target: {step['step_index']} "
                        f"offset={self.release_hover_offset_z:.3f}m"
                    )
                else:
                    self._publish_status(f"gripper_lowering: {step['step_index']}")
                return

            lower_hold_sec = max(self.release_lower_hold_sec, 0.0)
            if release_has_target:
                release_radius = max(self.release_success_radius_m, 0.005)
                timeout_sec = max(self.release_approach_timeout_sec, lower_hold_sec)
                hold_ready = elapsed >= lower_hold_sec
                position_ready = (
                    release_distance is not None and release_distance <= release_radius
                )
                timed_out = elapsed >= timeout_sec
                timeout_without_distance = timed_out and release_distance is None
                if not hold_ready or (not position_ready and not timeout_without_distance):
                    distance_text = (
                        "unknown"
                        if release_distance is None
                        else f"{release_distance:.3f}m"
                    )
                    self._publish_status_periodic(
                        f"gripper_lowering_to_target: {step['step_index']} "
                        f"dist={distance_text} radius={release_radius:.3f}m "
                        f"hold={elapsed:.2f}/{lower_hold_sec:.2f}s",
                        publish_on_change=False,
                    )
                    return
                if timeout_without_distance:
                    self.get_logger().warn(
                        "[EXEC] release target distance unavailable at timeout; opening gripper"
                    )
            elif elapsed < lower_hold_sec:
                self._publish_status_periodic(
                    f"gripper_lowering: {step['step_index']} "
                    f"hold={elapsed:.2f}/{lower_hold_sec:.2f}s",
                    publish_on_change=False,
                )
                return

            if not self.step_gripper_command_sent:
                self._publish_gripper_command("open")
                self.step_gripper_command_sent = True
                self.step_gripper_command_sent_ns = now_ns
                self._set_static_target_override(False)
                self._set_tracking_enabled(False)
                self._publish_status(f"gripper_opened: {step['step_index']}")
            open_elapsed = (
                0.0
                if self.step_gripper_command_sent_ns is None
                else (now_ns - self.step_gripper_command_sent_ns) * 1e-9
            )
            if open_elapsed >= max(
                self.release_open_wait_sec,
                max(step["wait_sec"], 0.5),
            ):
                self._advance_step()
            return

        if step["action"] == "wait":
            self._set_static_target_override(False)
            self._set_tracking_enabled(False)
            if (now_ns - self.step_started_ns) * 1e-9 >= step["wait_sec"]:
                self._advance_step()
            return

        if self.step_prompt_sent_ns is None:
            self._publish_prompt(step["target_prompt"])
            self.step_prompt_sent_ns = self.last_prompt_pub_ns
            self._set_tracking_hover_offset(self.hover_offset_z)
            self._set_static_target_override(False)
            self._set_tracking_enabled(False)
            self._publish_status(
                f"step_waiting_target: {step['step_index']} {step['target_prompt']}"
            )
            return

        if (
            self.step_target_confirmed_ns is None
            and (now_ns - self.last_prompt_pub_ns) * 1e-9 >= self.prompt_republish_sec
        ):
            self._publish_prompt(step["target_prompt"])
            self._set_tracking_enabled(False)

        if self.step_target_confirmed_ns is None:
            static_target = self._static_target_for_step(step)
            if static_target is not None:
                static_point_world, static_normal_world = static_target
                visual_ready = (
                    self.prefer_visual_tray_center_keypoint
                    and self._target_is_ready_for_current_step(
                        step=step,
                        target_valid=target_valid,
                        target_cam=target_cam,
                        target_header=target_header,
                        target_received_ns=target_received_ns,
                    )
                )
                if visual_ready:
                    try:
                        visual_point_world = self._transform_point_to_target_frame(
                            target_cam,
                            "" if target_header is None else str(target_header.frame_id),
                        )
                        plausible, visual_distance_xy = self._tray_visual_center_is_plausible(
                            visual_point_world
                        )
                        if not plausible:
                            visual_ready = False
                            self._publish_status_periodic(
                                f"step_rejecting_tray_center_keypoint: {step['step_index']} "
                                f"{step['target_prompt']} dist_xy={visual_distance_xy:.3f}m "
                                f"max={self.visual_tray_center_max_distance_m:.3f}m",
                                publish_on_change=False,
                            )
                    except Exception as exc:
                        visual_ready = False
                        self.get_logger().warning(
                            f"[EXEC] tray visual center transform failed: {exc}"
                        )

                if visual_ready:
                    self.step_target_confirmed_ns = now_ns
                    self.step_satisfied_ns = None
                    self.step_confirmed_target_cam = np.array(visual_point_world, copy=True)
                    self.step_confirmed_target_frame = self.target_frame
                    self.step_static_target = True
                    self._publish_static_target_override(
                        visual_point_world,
                        static_normal_world,
                    )
                    self._set_tracking_enabled(True)
                    self.get_logger().info(
                        f"[EXEC] step {step['step_index']} visual tray center confirmed: "
                        f"{np.asarray(visual_point_world).tolist()}"
                    )
                    self._publish_status(
                        f"step_visual_tray_center_confirmed: {step['step_index']} "
                        f"{step['target_prompt']}"
                    )
                else:
                    elapsed_since_prompt = (
                        0.0
                        if self.step_prompt_sent_ns is None
                        else (now_ns - self.step_prompt_sent_ns) * 1e-9
                    )
                    use_static_fallback = (
                        not self.prefer_visual_tray_center_keypoint
                        or self.static_tray_center_fallback_sec <= 0.0
                        or elapsed_since_prompt >= self.static_tray_center_fallback_sec
                    )
                    if not use_static_fallback:
                        wait_reason = self._target_wait_reason(
                            step=step,
                            target_valid=target_valid,
                            target_cam=target_cam,
                            target_header=target_header,
                            target_received_ns=target_received_ns,
                        )
                        remaining = max(
                            self.static_tray_center_fallback_sec - elapsed_since_prompt,
                            0.0,
                        )
                        self._publish_status_periodic(
                            f"step_waiting_tray_center_keypoint: {step['step_index']} "
                            f"{step['target_prompt']} reason={wait_reason} "
                            f"fallback_in={remaining:.1f}s",
                            publish_on_change=False,
                        )
                        return

                    self.step_target_confirmed_ns = now_ns
                    self.step_satisfied_ns = None
                    self.step_confirmed_target_cam = np.array(static_point_world, copy=True)
                    self.step_confirmed_target_frame = self.target_frame
                    self.step_static_target = True
                    self._publish_static_target_override(static_point_world, static_normal_world)
                    self._set_tracking_enabled(True)
                    self.get_logger().info(
                        f"[EXEC] step {step['step_index']} static tray center fallback: "
                        f"{static_point_world.tolist()}"
                    )
                    self._publish_status(
                        f"step_static_tray_center_fallback: {step['step_index']} "
                        f"{step['target_prompt']}"
                    )
            elif not self._target_is_ready_for_current_step(
                step=step,
                target_valid=target_valid,
                target_cam=target_cam,
                target_header=target_header,
                target_received_ns=target_received_ns,
            ):
                wait_reason = self._target_wait_reason(
                    step=step,
                    target_valid=target_valid,
                    target_cam=target_cam,
                    target_header=target_header,
                    target_received_ns=target_received_ns,
                )
                self._publish_status_periodic(
                    f"step_waiting_target: {step['step_index']} {step['target_prompt']} "
                    f"reason={wait_reason}",
                    publish_on_change=False,
                )
                return
            else:
                self.step_target_confirmed_ns = now_ns
                self.step_satisfied_ns = None
                self.step_confirmed_target_cam = np.array(target_cam, copy=True)
                self.step_confirmed_target_frame = (
                    "" if target_header is None else str(target_header.frame_id)
                )
                self.step_static_target = False
                self._set_static_target_override(False)
                self._set_tracking_enabled(True)
                self.get_logger().info(
                    f"[EXEC] step {step['step_index']} target confirmed: {step['target_prompt']}"
                )
                self._publish_status(
                    f"step_target_confirmed: {step['step_index']} {step['target_prompt']}"
                )

        if self.step_static_target and self.step_confirmed_target_cam is not None:
            self._publish_static_target_override(
                self.step_confirmed_target_cam,
                self.static_tray_plane_normal_world,
            )
        self._set_tracking_enabled(True)

        eval_target_cam = target_cam
        eval_target_frame = "" if target_header is None else str(target_header.frame_id)
        target_fresh = self._target_is_fresh(target_header)
        using_confirmed_target = False
        grasp_target_occluded = step["action"] == "grasp_target" and not target_valid

        if (
            self.step_target_confirmed_ns is not None
            and self.use_confirmed_static_target
            and self.step_confirmed_target_cam is not None
            and self.step_confirmed_target_frame
        ):
            eval_target_cam = np.array(self.step_confirmed_target_cam, copy=True)
            eval_target_frame = str(self.step_confirmed_target_frame)
            using_confirmed_target = True
            target_fresh = True
        elif self.step_target_confirmed_ns is not None and (
            eval_target_cam is None
            or target_header is None
            or not target_fresh
            or grasp_target_occluded
        ):
            if self.step_confirmed_target_cam is not None and self.step_confirmed_target_frame:
                eval_target_cam = np.array(self.step_confirmed_target_cam, copy=True)
                eval_target_frame = str(self.step_confirmed_target_frame)
                using_confirmed_target = True
                target_fresh = True

        if q_now is None or eval_target_cam is None or not eval_target_frame:
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    target_valid=target_valid,
                    target_fresh=target_fresh,
                    reason="missing_data",
                ),
                publish_on_change=False,
            )
            return
        if not target_fresh and not using_confirmed_target:
            self.step_satisfied_ns = None
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    target_valid=target_valid,
                    target_fresh=False,
                    reason="stale_target",
                ),
                publish_on_change=False,
            )
            return
        if not target_valid and self.step_target_confirmed_ns is None:
            self.step_satisfied_ns = None
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    target_valid=False,
                    target_fresh=target_fresh,
                    reason="valid_false_before_confirm",
                ),
                publish_on_change=False,
            )
            return

        try:
            point_world = self._transform_point_to_target_frame(eval_target_cam, eval_target_frame)
        except Exception as exc:
            self.get_logger().warn(f"[EXEC] TF lookup failed: {exc}")
            return

        active_hover_offset_z = (
            self.place_hover_offset_z
            if step["action"] == "hover_target" and self._next_action_is_release()
            else self.hover_offset_z
        )
        grasp_lift_active = False
        close_elapsed = 0.0
        grasp_confirm_elapsed = 0.0
        if step["action"] == "grasp_target" and not self.step_gripper_command_sent:
            if self.step_grasp_descent_started:
                if self.grasp_approach_offset_z >= 0.0:
                    active_hover_offset_z = self.grasp_approach_offset_z
            else:
                active_hover_offset_z = (
                    self.grasp_pregrasp_offset_m
                    if self.grasp_pregrasp_offset_m >= 0.0
                    else self.hover_offset_z
                )
        if step["action"] == "grasp_target" and self.step_gripper_command_sent:
            close_elapsed = (
                0.0
                if self.step_gripper_command_sent_ns is None
                else (now_ns - self.step_gripper_command_sent_ns) * 1e-9
            )
            active_hover_offset_z = (
                self.grasp_approach_offset_z
                if self.grasp_approach_offset_z >= 0.0
                else 0.0
            )
            if gripper_holding_confirmed_ns is not None:
                grasp_confirm_elapsed = max(
                    (now_ns - gripper_holding_confirmed_ns) * 1e-9,
                    0.0,
                )
            grasp_confirmed_for_lift = (
                gripper_holding_object
                and grasp_confirm_elapsed >= max(self.grasp_confirm_hold_sec, 0.0)
            )
            if (
                grasp_confirmed_for_lift
                and close_elapsed >= max(self.grasp_lift_delay_sec, 0.0)
            ):
                if self.step_grasp_lift_started_ns is None:
                    self.step_grasp_lift_started_ns = now_ns
                    self.last_periodic_status_text = ""
                active_hover_offset_z = active_hover_offset_z + max(self.grasp_lift_m, 0.0)
                grasp_lift_active = True

        self._set_tracking_hover_offset(active_hover_offset_z)

        hover_target = np.array(
            [
                point_world[0],
                point_world[1],
                point_world[2] + active_hover_offset_z,
            ],
            dtype=np.float64,
        )
        if self.track_gripper_center and self.min_gripper_center_z >= 0.0:
            hover_target[2] = max(hover_target[2], self.min_gripper_center_z)

        ee_pos, ee_orn = self.ik.fk(q_now)
        control_pos = np.array(ee_pos, dtype=np.float64)
        if self.track_gripper_center:
            control_pos = control_pos + quat_to_matrix(ee_orn) @ self.gripper_center_offset_link6
        distance = float(np.linalg.norm(control_pos - hover_target))
        success_radius = (
            step["success_radius_m"]
            if step["success_radius_m"] > 0.0
            else self.default_success_radius_m
        )
        if step["action"] == "hover_target" and self._next_action_is_release():
            success_radius = min(
                success_radius,
                max(self.place_success_radius_m, 0.005),
            )
        if step["action"] == "grasp_target" and grasp_lift_active:
            success_radius = min(success_radius, max(self.grasp_lift_success_radius_m, 0.01))
        elif step["action"] == "grasp_target" and not self.step_gripper_command_sent:
            success_radius = min(success_radius, max(self.grasp_close_success_radius_m, 0.01))

        if (
            step["action"] == "grasp_target"
            and not self.step_gripper_command_sent
            and not self.step_grasp_descent_started
        ):
            pregrasp_radius = max(self.grasp_pregrasp_success_radius_m, 0.005)
            if distance <= pregrasp_radius:
                if self.step_pregrasp_satisfied_ns is None:
                    self.step_pregrasp_satisfied_ns = now_ns
                pregrasp_elapsed = (now_ns - self.step_pregrasp_satisfied_ns) * 1e-9
                if pregrasp_elapsed >= max(self.grasp_pregrasp_hold_sec, 0.0):
                    self.step_grasp_descent_started = True
                    self.step_satisfied_ns = None
                    self.last_periodic_status_text = ""
                    self._publish_status(
                        f"gripper_axis_descent_started: {step['step_index']} "
                        f"{step['target_prompt']}"
                    )
                    return
                self._publish_status_periodic(
                    self._tracking_status_text(
                        step,
                        distance=distance,
                        success_radius=pregrasp_radius,
                        dwell_elapsed=pregrasp_elapsed,
                        target_valid=target_valid,
                        target_fresh=target_fresh,
                        reason="pregrasp_plane_aligned_hold",
                    ),
                    publish_on_change=False,
                )
                return
            self.step_pregrasp_satisfied_ns = None
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    distance=distance,
                    success_radius=pregrasp_radius,
                    target_valid=target_valid,
                    target_fresh=target_fresh,
                    reason="aligning_pregrasp_on_plane_normal",
                ),
                publish_on_change=False,
            )
            return

        if (
            step["action"] == "grasp_target"
            and self.step_gripper_command_sent
            and grasp_lift_active
        ):
            lift_elapsed = (
                0.0
                if self.step_grasp_lift_started_ns is None
                else (now_ns - self.step_grasp_lift_started_ns) * 1e-9
            )
            lift_hold_sec = max(self.grasp_lift_hold_sec, 0.0)
            if lift_elapsed >= lift_hold_sec:
                self._publish_status(
                    f"gripper_lift_held: {step['step_index']} "
                    f"hold={lift_elapsed:.2f}/{lift_hold_sec:.2f}s"
                )
                self._advance_step()
                return
            self.step_satisfied_ns = None
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    distance=distance,
                    success_radius=success_radius,
                    dwell_elapsed=lift_elapsed,
                    target_valid=target_valid,
                    target_fresh=target_fresh,
                    reason=f"holding_lift_before_release:{lift_elapsed:.2f}/{lift_hold_sec:.2f}s",
                ),
                publish_on_change=False,
            )
            return

        if (
            step["action"] == "grasp_target"
            and self.step_gripper_command_sent
            and not grasp_lift_active
        ):
            self.step_satisfied_ns = None
            close_elapsed = (
                0.0
                if self.step_gripper_command_sent_ns is None
                else (now_ns - self.step_gripper_command_sent_ns) * 1e-9
            )
            if close_elapsed < max(self.grasp_lift_delay_sec, 0.0):
                wait_reason = (
                    f"waiting_gripper_close_delay:{close_elapsed:.2f}/"
                    f"{max(self.grasp_lift_delay_sec, 0.0):.2f}s"
                )
            elif not gripper_holding_object:
                status_suffix = (
                    f" status={gripper_status_text}"
                    if gripper_status_text
                    else ""
                )
                wait_reason = f"waiting_gripper_grasp_confirmation{status_suffix}"
            elif grasp_confirm_elapsed < max(self.grasp_confirm_hold_sec, 0.0):
                wait_reason = (
                    f"waiting_gripper_confirm_hold:{grasp_confirm_elapsed:.2f}/"
                    f"{max(self.grasp_confirm_hold_sec, 0.0):.2f}s"
                )
            else:
                wait_reason = "waiting_lift_start"
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    distance=distance,
                    success_radius=success_radius,
                    dwell_elapsed=close_elapsed,
                    target_valid=target_valid,
                    target_fresh=target_fresh,
                    reason=wait_reason,
                ),
                publish_on_change=False,
            )
            return

        if distance <= success_radius:
            if self.step_satisfied_ns is None:
                if step["action"] == "grasp_target" and not self.step_gripper_command_sent:
                    self._publish_gripper_command("close")
                    self.step_gripper_command_sent = True
                    self.step_gripper_command_sent_ns = now_ns
                    self._publish_status(
                        f"gripper_closing: {step['step_index']} {step['target_prompt']}"
                    )
                    self._publish_status_periodic(
                        self._tracking_status_text(
                            step,
                            distance=distance,
                            success_radius=success_radius,
                            dwell_elapsed=0.0,
                            target_valid=target_valid,
                            target_fresh=target_fresh,
                            reason="gripper_closing_before_lift",
                        ),
                        publish_on_change=True,
                    )
                    return
                self.step_satisfied_ns = now_ns
            dwell_elapsed = (now_ns - self.step_satisfied_ns) * 1e-9
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    distance=distance,
                    success_radius=success_radius,
                    dwell_elapsed=dwell_elapsed,
                    target_valid=target_valid,
                    target_fresh=target_fresh,
                    reason="inside_radius_confirmed_hold" if using_confirmed_target else "inside_radius",
                ),
                publish_on_change=False,
            )
            if (now_ns - self.step_satisfied_ns) * 1e-9 >= step["dwell_sec"]:
                self._advance_step()
        else:
            self.step_satisfied_ns = None
            self._publish_status_periodic(
                self._tracking_status_text(
                    step,
                    distance=distance,
                    success_radius=success_radius,
                    dwell_elapsed=0.0,
                    target_valid=target_valid,
                    target_fresh=target_fresh,
                    reason="outside_radius_confirmed_hold" if using_confirmed_target else "outside_radius",
                ),
                publish_on_change=False,
            )

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
