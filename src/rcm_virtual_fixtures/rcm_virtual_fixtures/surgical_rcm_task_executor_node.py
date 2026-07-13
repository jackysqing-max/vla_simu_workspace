#!/usr/bin/env python3
"""Execute an LLM-generated RCM task through deterministic motion stages."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from pybullet_ros2_sim.task_plan_utils import (
    SURGICAL_RCM_ACTIONS,
    plan_from_json,
)


class SurgicalRcmTaskExecutor(Node):
    """Gate perception and RCM control according to a constrained task plan."""

    def __init__(self):
        super().__init__("surgical_rcm_task_executor_node")

        self.declare_parameter("plan_topic", "/llm_task/plan_json")
        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("port_ready_topic", "/vlm_rcm/port_ready")
        self.declare_parameter(
            "port_point_topic",
            "/vlm_rcm/locked_port_point",
        )
        self.declare_parameter(
            "port_axis_topic",
            "/vlm_rcm/locked_port_axis",
        )
        self.declare_parameter(
            "controller_start_topic",
            "/rcm_virtual_fixtures/start",
        )
        self.declare_parameter(
            "controller_stage_topic",
            "/rcm_virtual_fixtures/stage",
        )
        self.declare_parameter(
            "controller_pivot_start_topic",
            "/rcm_virtual_fixtures/start_pivot",
        )
        self.declare_parameter(
            "controller_pivot_hold_topic",
            "/rcm_virtual_fixtures/hold_pivot",
        )
        self.declare_parameter(
            "controller_status_topic",
            "/rcm_virtual_fixtures/status",
        )
        self.declare_parameter("status_topic", "/surgical_rcm/status")
        self.declare_parameter(
            "camera_enable_topic",
            "/sim/camera/enabled",
        )
        self.declare_parameter(
            "active_step_topic",
            "/surgical_rcm/active_step_json",
        )
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("prompt_republish_sec", 1.0)
        self.declare_parameter("trajectory_speed_mps", 0.018)
        self.declare_parameter("status_heartbeat_sec", 0.5)

        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.port_ready_topic = str(
            self.get_parameter("port_ready_topic").value
        )
        self.port_point_topic = str(
            self.get_parameter("port_point_topic").value
        )
        self.port_axis_topic = str(
            self.get_parameter("port_axis_topic").value
        )
        self.controller_start_topic = str(
            self.get_parameter("controller_start_topic").value
        )
        self.controller_stage_topic = str(
            self.get_parameter("controller_stage_topic").value
        )
        self.controller_pivot_start_topic = str(
            self.get_parameter("controller_pivot_start_topic").value
        )
        self.controller_pivot_hold_topic = str(
            self.get_parameter("controller_pivot_hold_topic").value
        )
        self.controller_status_topic = str(
            self.get_parameter("controller_status_topic").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.camera_enable_topic = str(
            self.get_parameter("camera_enable_topic").value
        )
        self.active_step_topic = str(
            self.get_parameter("active_step_topic").value
        )
        self.control_hz = max(
            float(self.get_parameter("control_hz").value),
            1.0,
        )
        self.prompt_republish_sec = max(
            float(self.get_parameter("prompt_republish_sec").value),
            0.1,
        )
        self.trajectory_speed_mps = max(
            float(self.get_parameter("trajectory_speed_mps").value),
            0.001,
        )
        self.status_heartbeat_sec = max(
            float(self.get_parameter("status_heartbeat_sec").value),
            0.1,
        )

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_prompt = self.create_publisher(
            String,
            self.prompt_topic,
            10,
        )
        self.pub_controller_start = self.create_publisher(
            Bool,
            self.controller_start_topic,
            qos_latched,
        )
        self.pub_controller_pivot_start = self.create_publisher(
            Bool,
            self.controller_pivot_start_topic,
            qos_latched,
        )
        self.pub_controller_pivot_hold = self.create_publisher(
            Bool,
            self.controller_pivot_hold_topic,
            qos_latched,
        )
        self.pub_status = self.create_publisher(
            String,
            self.status_topic,
            10,
        )
        self.pub_active_step = self.create_publisher(
            String,
            self.active_step_topic,
            10,
        )
        self.pub_camera_enable = self.create_publisher(
            Bool,
            self.camera_enable_topic,
            qos_latched,
        )

        self.create_subscription(
            String,
            self.plan_topic,
            self.on_plan,
            qos_latched,
        )
        self.create_subscription(
            Bool,
            self.port_ready_topic,
            self.on_port_ready,
            qos_latched,
        )
        self.create_subscription(
            PointStamped,
            self.port_point_topic,
            self.on_port_point,
            qos_latched,
        )
        self.create_subscription(
            Vector3Stamped,
            self.port_axis_topic,
            self.on_port_axis,
            qos_latched,
        )
        self.create_subscription(
            String,
            self.controller_stage_topic,
            self.on_controller_stage,
            10,
        )
        self.create_subscription(
            String,
            self.controller_status_topic,
            self.on_controller_status,
            10,
        )

        self.plan = None
        self.step_cursor = 0
        self.step_satisfied_since_ns = None
        self.last_prompt_ns = 0
        self.last_status_ns = 0
        self.last_status_text = ""
        self.controller_start_sent = False
        self.controller_pivot_start_sent = False
        self.controller_stage = "WAITING_FOR_START"
        self.controller_status = ""
        self.port_ready = False
        self.port_point = None
        self.port_axis = None
        self.circle_started_ns = None
        self.plan_complete = False

        self._publish_controller_start(False)
        self._publish_controller_pivot_start(False)
        self._publish_controller_pivot_hold(False)
        self._publish_camera_enable(True)
        self.timer = self.create_timer(
            1.0 / self.control_hz,
            self.on_timer,
        )
        self.get_logger().info(
            "surgical_rcm_task_executor_node started; waiting for task plan"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _publish_controller_start(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_controller_start.publish(msg)

    def _publish_controller_pivot_start(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_controller_pivot_start.publish(msg)

    def _publish_controller_pivot_hold(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_controller_pivot_hold.publish(msg)

    def _publish_camera_enable(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_camera_enable.publish(msg)

    def _publish_prompt(self, prompt: str):
        msg = String()
        msg.data = str(prompt)
        self.pub_prompt.publish(msg)
        self.last_prompt_ns = self._now_ns()

    def _publish_status(self, text: str, *, force: bool = False):
        now_ns = self._now_ns()
        if (
            not force
            and text == self.last_status_text
            and (now_ns - self.last_status_ns) * 1e-9
            < self.status_heartbeat_sec
        ):
            return
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
        self.last_status_text = text
        self.last_status_ns = now_ns

    def _publish_active_step(self, step: dict | None):
        msg = String()
        msg.data = json.dumps(
            step if step is not None else {},
            ensure_ascii=False,
        )
        self.pub_active_step.publish(msg)

    def on_plan(self, msg: String):
        try:
            plan = plan_from_json(
                msg.data,
                allow_open_vocabulary=True,
                allow_rcm_actions=True,
            )
        except Exception as exc:
            self.get_logger().error(f"invalid surgical RCM plan: {exc}")
            self._publish_status(
                f"plan_rejected: invalid_json error={exc}",
                force=True,
            )
            return

        actions = tuple(step["action"] for step in plan["steps"])
        if actions != SURGICAL_RCM_ACTIONS:
            self._publish_status(
                f"plan_rejected: invalid_action_order actions={actions}",
                force=True,
            )
            return

        self.plan = plan
        self.step_cursor = 0
        self.step_satisfied_since_ns = None
        self.last_prompt_ns = 0
        self.controller_start_sent = False
        self.controller_pivot_start_sent = False
        self.circle_started_ns = None
        self.plan_complete = False
        self._publish_controller_start(False)
        self._publish_controller_pivot_start(False)
        self._publish_controller_pivot_hold(False)
        self._publish_camera_enable(True)
        self._publish_active_step(plan["steps"][0])
        self._publish_status(
            f"plan_accepted: {len(plan['steps'])} surgical RCM steps",
            force=True,
        )
        self.get_logger().info(
            f"[PLAN] accepted: {plan['task_summary']}"
        )

    def on_port_ready(self, msg: Bool):
        self.port_ready = bool(msg.data)

    def on_port_point(self, msg: PointStamped):
        self.port_point = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )

    def on_port_axis(self, msg: Vector3Stamped):
        self.port_axis = (
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
        )

    def on_controller_stage(self, msg: String):
        stage = str(msg.data).strip()
        if stage:
            if stage != self.controller_stage and stage == "RCM_PIVOT":
                self.circle_started_ns = self._now_ns()
            self.controller_stage = stage

    def on_controller_status(self, msg: String):
        self.controller_status = str(msg.data)

    def _current_step(self):
        if self.plan is None or self.step_cursor >= len(self.plan["steps"]):
            return None
        return self.plan["steps"][self.step_cursor]

    def _step_dwell_satisfied(self, condition: bool, dwell_sec: float):
        if not condition:
            self.step_satisfied_since_ns = None
            return False
        now_ns = self._now_ns()
        if self.step_satisfied_since_ns is None:
            self.step_satisfied_since_ns = now_ns
        return (
            now_ns - self.step_satisfied_since_ns
        ) * 1e-9 >= max(float(dwell_sec), 0.0)

    def _advance_step(self):
        completed = self._current_step()
        if completed is None:
            return
        self.get_logger().info(
            f"[STEP_COMPLETE] {completed['step_index']} "
            f"{completed['action']}"
        )
        if completed["action"] == "localize_rcm_port":
            self._publish_camera_enable(False)
            self.get_logger().info(
                "[PERCEPTION_LOCK] RGB-D rendering disabled after port lock"
            )
        self.step_cursor += 1
        self.step_satisfied_since_ns = None
        next_step = self._current_step()
        self._publish_active_step(next_step)
        if next_step is None:
            self.plan_complete = True
            self._publish_controller_pivot_hold(True)
            self._publish_status(
                "plan_completed: RCM circular trajectory demonstrated; "
                "controller entered RCM_HOLD",
                force=True,
            )
        else:
            self._publish_status(
                f"step_started: {next_step['step_index']} "
                f"{next_step['action']}",
                force=True,
            )

    def _run_localize(self, step: dict):
        now_ns = self._now_ns()
        if (
            now_ns - self.last_prompt_ns
        ) * 1e-9 >= self.prompt_republish_sec:
            self._publish_prompt(step["target_prompt"])
        localized = (
            self.port_ready
            and self.port_point is not None
            and self.port_axis is not None
        )
        point_text = (
            "none"
            if self.port_point is None
            else (
                f"({self.port_point[0]:.4f},"
                f"{self.port_point[1]:.4f},"
                f"{self.port_point[2]:.4f})"
            )
        )
        self._publish_status(
            f"step_1_localize: ready={str(localized).lower()} "
            f"port={point_text}"
        )
        if self._step_dwell_satisfied(localized, step["dwell_sec"]):
            self._advance_step()

    def _run_align(self, step: dict):
        if not self.controller_start_sent:
            self._publish_controller_start(True)
            self.controller_start_sent = True
        aligned_stages = {
            "APPROACH_PORT",
            "PORT_DWELL",
            "INSERT_THROUGH_PORT",
            "INSERTED_DWELL",
            "RCM_PIVOT",
        }
        aligned = self.controller_stage in aligned_stages
        self._publish_status(
            f"step_2_align: controller_stage={self.controller_stage} "
            f"aligned={str(aligned).lower()}"
        )
        if self._step_dwell_satisfied(aligned, step["dwell_sec"]):
            self._advance_step()

    def _run_establish_rcm(self, step: dict):
        established = self.controller_stage == "WAITING_FOR_PIVOT"
        self._publish_status(
            f"step_3_establish_rcm: stage={self.controller_stage} "
            f"established={str(established).lower()} "
            f"controller=({self.controller_status})"
        )
        if self._step_dwell_satisfied(established, step["dwell_sec"]):
            self._advance_step()

    def _run_circle(self, step: dict):
        if not self.controller_pivot_start_sent:
            self._publish_controller_pivot_start(True)
            self.controller_pivot_start_sent = True
        if self.controller_stage != "RCM_PIVOT":
            self.circle_started_ns = None
            self._publish_status(
                f"step_4_circle: waiting_for=RCM_PIVOT "
                f"stage={self.controller_stage}"
            )
            return
        if self.circle_started_ns is None:
            self.circle_started_ns = self._now_ns()
        radius = max(float(step["trajectory_radius_m"]), 0.001)
        cycles = max(float(step["trajectory_cycles"]), 0.25)
        duration_sec = (
            cycles * 2.0 * math.pi * radius / self.trajectory_speed_mps
        )
        elapsed_sec = (
            self._now_ns() - self.circle_started_ns
        ) * 1e-9
        self._publish_status(
            f"step_4_circle: running elapsed={elapsed_sec:.2f}/"
            f"{duration_sec:.2f}s radius={radius:.3f}m "
            f"cycles={cycles:.2f}"
        )
        if elapsed_sec >= duration_sec:
            self._advance_step()

    def on_timer(self):
        step = self._current_step()
        if step is None:
            if self.plan is None:
                self._publish_status("waiting_for_plan")
            elif self.plan_complete:
                self._publish_status(
                    "plan_completed: controller holding final RCM pose"
                )
            return

        action = step["action"]
        if action == "localize_rcm_port":
            self._run_localize(step)
        elif action == "align_tool_axis":
            self._run_align(step)
        elif action == "establish_rcm":
            self._run_establish_rcm(step)
        elif action == "execute_rcm_circle":
            self._run_circle(step)


def main(args=None):
    rclpy.init(args=args)
    node = SurgicalRcmTaskExecutor()
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
