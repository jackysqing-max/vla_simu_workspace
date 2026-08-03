#!/usr/bin/env python3
"""Replay visual prompts from an LLM task plan without executing robot motion."""

from __future__ import annotations

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String

from pybullet_ros2_sim.task_plan_utils import plan_from_json


class KeypointDebugPlanPlayer(Node):
    """Drive SAM3 from a plan and report keypoint perception state only."""

    def __init__(self):
        super().__init__("keypoint_debug_plan_player_node")

        self.declare_parameter("plan_topic", "/llm_task/plan_json")
        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("status_topic", "/llm_task/status")
        self.declare_parameter("tracking_enable_topic", "/llm_task/tracking_enabled")
        self.declare_parameter("valid_topic", "/perception/valid")
        self.declare_parameter("keypoint_topic", "/perception/keypoint_3d")
        self.declare_parameter("keypoint_px_topic", "/perception/keypoint_px")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("plane_normal_topic", "/perception/object_plane_normal")
        self.declare_parameter("plane_tangent_topic", "/perception/object_plane_tangent")
        self.declare_parameter("candidate_text_topic", "/perception/keypoint_candidates_text")
        self.declare_parameter("open_vocabulary_targets", False)
        self.declare_parameter("enable_grasp_actions", False)
        self.declare_parameter("prompt_republish_sec", 1.0)
        self.declare_parameter("status_heartbeat_sec", 1.0)
        self.declare_parameter("keypoint_timeout_sec", 1.0)
        self.declare_parameter("auto_advance_on_valid", False)
        self.declare_parameter("auto_advance_hold_sec", 1.0)

        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.tracking_enable_topic = str(
            self.get_parameter("tracking_enable_topic").value
        )
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.keypoint_topic = str(self.get_parameter("keypoint_topic").value)
        self.keypoint_px_topic = str(self.get_parameter("keypoint_px_topic").value)
        self.score_topic = str(self.get_parameter("score_topic").value)
        self.plane_normal_topic = str(self.get_parameter("plane_normal_topic").value)
        self.plane_tangent_topic = str(self.get_parameter("plane_tangent_topic").value)
        self.candidate_text_topic = str(self.get_parameter("candidate_text_topic").value)
        self.open_vocabulary_targets = bool(
            self.get_parameter("open_vocabulary_targets").value
        )
        self.enable_grasp_actions = bool(self.get_parameter("enable_grasp_actions").value)
        self.prompt_republish_sec = float(
            self.get_parameter("prompt_republish_sec").value
        )
        self.status_heartbeat_sec = float(
            self.get_parameter("status_heartbeat_sec").value
        )
        self.keypoint_timeout_sec = float(
            self.get_parameter("keypoint_timeout_sec").value
        )
        self.auto_advance_on_valid = bool(
            self.get_parameter("auto_advance_on_valid").value
        )
        self.auto_advance_hold_sec = float(
            self.get_parameter("auto_advance_hold_sec").value
        )

        self.lock = threading.Lock()
        self.plan = None
        self.visual_steps: list[dict] = []
        self.cursor = 0
        self.current_prompt = ""
        self.last_prompt_pub_ns = 0
        self.target_valid = False
        self.keypoint = None
        self.keypoint_header = None
        self.keypoint_received_ns = 0
        self.keypoint_px = None
        self.keypoint_px_header = None
        self.score = 0.0
        self.plane_normal = None
        self.plane_normal_header = None
        self.plane_tangent = None
        self.plane_tangent_header = None
        self.candidate_text = ""
        self.valid_since_ns = 0
        self.last_periodic_status_text = ""
        self.last_periodic_status_ns = 0

        self.pub_prompt = self.create_publisher(String, self.prompt_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_tracking_enable = self.create_publisher(
            Bool,
            self.tracking_enable_topic,
            10,
        )

        self.create_subscription(String, self.plan_topic, self.on_plan, 10)
        self.create_subscription(Bool, self.valid_topic, self.on_valid, 10)
        self.create_subscription(PointStamped, self.keypoint_topic, self.on_keypoint, 10)
        self.create_subscription(
            PointStamped,
            self.keypoint_px_topic,
            self.on_keypoint_px,
            10,
        )
        self.create_subscription(Float32, self.score_topic, self.on_score, 10)
        self.create_subscription(
            Vector3Stamped,
            self.plane_normal_topic,
            self.on_plane_normal,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            self.plane_tangent_topic,
            self.on_plane_tangent,
            10,
        )
        self.create_subscription(
            String,
            self.candidate_text_topic,
            self.on_candidate_text,
            10,
        )

        self.timer = self.create_timer(0.1, self.on_timer)
        self._set_tracking_enabled(False)
        self.get_logger().info(
            "keypoint_debug_plan_player_node started. "
            f"plan_topic={self.plan_topic} prompt_topic={self.prompt_topic}"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

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

    def _set_tracking_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_tracking_enable.publish(msg)

    def _publish_prompt(self, prompt: str):
        msg = String()
        msg.data = prompt
        self.pub_prompt.publish(msg)
        self.last_prompt_pub_ns = self._now_ns()

    def _header_is_fresh(self, header) -> bool:
        if self.keypoint_timeout_sec <= 0.0:
            return True
        if header is None:
            return False
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return True
        age_sec = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9
        return age_sec <= self.keypoint_timeout_sec

    def _current_step_locked(self) -> dict | None:
        if not self.visual_steps:
            return None
        if self.cursor < 0 or self.cursor >= len(self.visual_steps):
            return None
        return self.visual_steps[self.cursor]

    def _format_vec(self, vector, precision: int = 3) -> str:
        if vector is None:
            return "none"
        return "(" + ",".join(f"{float(value):.{precision}f}" for value in vector[:3]) + ")"

    def _short_candidate_text(self, text: str, limit: int = 90) -> str:
        text = " ".join((text or "").split())
        if len(text) <= limit:
            return text
        return text[: max(0, limit - 3)] + "..."

    def _reset_observation_locked(self):
        self.target_valid = False
        self.keypoint = None
        self.keypoint_header = None
        self.keypoint_received_ns = 0
        self.keypoint_px = None
        self.keypoint_px_header = None
        self.score = 0.0
        self.plane_normal = None
        self.plane_normal_header = None
        self.plane_tangent = None
        self.plane_tangent_header = None
        self.candidate_text = ""
        self.valid_since_ns = 0

    def on_plan(self, msg: String):
        try:
            plan = plan_from_json(
                msg.data,
                allow_open_vocabulary=self.open_vocabulary_targets,
                allow_grasp_actions=self.enable_grasp_actions,
            )
        except Exception as exc:
            self.get_logger().error(f"invalid plan_json for keypoint debug: {exc}")
            self._publish_status("debug_failed: invalid_plan")
            return

        visual_steps = [
            step
            for step in plan.get("steps", [])
            if str(step.get("target_prompt", "")).strip()
            and step.get("action") in {"hover_target", "grasp_target"}
        ]

        with self.lock:
            self.plan = plan
            self.visual_steps = visual_steps
            self.cursor = 0
            self.current_prompt = ""
            self.last_prompt_pub_ns = 0
            self.last_periodic_status_text = ""
            self.last_periodic_status_ns = 0
            self._reset_observation_locked()

            first_step = self._current_step_locked()
            if first_step is not None:
                self.current_prompt = str(first_step["target_prompt"])

        self._set_tracking_enabled(False)
        self._publish_status(
            f"debug_loaded: {plan['task_summary']} "
            f"visual_steps={len(visual_steps)} total_steps={len(plan.get('steps', []))}"
        )

        if not visual_steps:
            self._publish_status("debug_failed: no_visual_target_steps")
            return

        self._publish_current_prompt(announce=True)

    def _publish_current_prompt(self, *, announce: bool):
        with self.lock:
            step = self._current_step_locked()
            if step is None:
                return
            prompt = str(step["target_prompt"])
            action = str(step["action"])
            step_index = int(step["step_index"])
            self.current_prompt = prompt

        self._publish_prompt(prompt)
        if announce:
            self._publish_status(
                f"debug_prompt_sent: step={step_index} action={action} prompt={prompt}"
            )

    def on_valid(self, msg: Bool):
        now_ns = self._now_ns()
        with self.lock:
            was_valid = self.target_valid
            self.target_valid = bool(msg.data)
            if self.target_valid and not was_valid:
                self.valid_since_ns = now_ns
            elif not self.target_valid:
                self.valid_since_ns = 0

    def on_keypoint(self, msg: PointStamped):
        with self.lock:
            self.keypoint = np.array(
                [msg.point.x, msg.point.y, msg.point.z],
                dtype=np.float64,
            )
            self.keypoint_header = msg.header
            self.keypoint_received_ns = self._now_ns()

    def on_keypoint_px(self, msg: PointStamped):
        with self.lock:
            self.keypoint_px = np.array([msg.point.x, msg.point.y], dtype=np.float64)
            self.keypoint_px_header = msg.header

    def on_score(self, msg: Float32):
        with self.lock:
            self.score = float(msg.data)

    def on_plane_normal(self, msg: Vector3Stamped):
        with self.lock:
            self.plane_normal = np.array(
                [msg.vector.x, msg.vector.y, msg.vector.z],
                dtype=np.float64,
            )
            self.plane_normal_header = msg.header

    def on_plane_tangent(self, msg: Vector3Stamped):
        with self.lock:
            self.plane_tangent = np.array(
                [msg.vector.x, msg.vector.y, msg.vector.z],
                dtype=np.float64,
            )
            self.plane_tangent_header = msg.header

    def on_candidate_text(self, msg: String):
        with self.lock:
            self.candidate_text = msg.data

    def _debug_status_locked(self) -> tuple[str, bool]:
        step = self._current_step_locked()
        if step is None:
            return "debug_idle: waiting_for_plan", False

        prompt = str(step["target_prompt"])
        step_index = int(step["step_index"])
        keypoint_fresh = self._header_is_fresh(self.keypoint_header)
        px_fresh = self._header_is_fresh(self.keypoint_px_header)
        valid = bool(self.target_valid and keypoint_fresh and self.keypoint is not None)
        if valid:
            return (
                "debug_keypoint_valid: "
                f"step={step_index} prompt={prompt} score={self.score:.3f} "
                f"xyz={self._format_vec(self.keypoint)} "
                f"px={self._format_vec(self.keypoint_px, precision=1) if px_fresh else 'none'} "
                f"plane_n={self._format_vec(self.plane_normal)} "
                f"plane_t={self._format_vec(self.plane_tangent)} "
                f"candidates={self._short_candidate_text(self.candidate_text)}",
                True,
            )

        reasons = []
        if not self.target_valid:
            reasons.append("valid_false")
        if self.keypoint is None:
            reasons.append("no_keypoint")
        elif not keypoint_fresh:
            reasons.append("stale_keypoint")
        if self.last_prompt_pub_ns == 0:
            reasons.append("prompt_not_sent")
        reason = ",".join(reasons) if reasons else "waiting"
        return (
            "debug_waiting_keypoint: "
            f"step={step_index} prompt={prompt} score={self.score:.3f} reason={reason} "
            f"xyz={self._format_vec(self.keypoint)} "
            f"plane_n={self._format_vec(self.plane_normal)}",
            False,
        )

    def _maybe_auto_advance_locked(self, valid: bool) -> bool:
        if not self.auto_advance_on_valid or not valid:
            return False
        if self.cursor + 1 >= len(self.visual_steps):
            return False
        if self.valid_since_ns <= 0:
            return False
        hold_sec = max(self.auto_advance_hold_sec, 0.0)
        if (self._now_ns() - self.valid_since_ns) * 1e-9 < hold_sec:
            return False

        self.cursor += 1
        step = self._current_step_locked()
        self.current_prompt = "" if step is None else str(step["target_prompt"])
        self.last_prompt_pub_ns = 0
        self.valid_since_ns = 0
        self._reset_observation_locked()
        self.last_periodic_status_text = ""
        self.last_periodic_status_ns = 0
        return True

    def on_timer(self):
        self._set_tracking_enabled(False)

        should_publish_prompt = False
        with self.lock:
            step = self._current_step_locked()
            if step is not None:
                elapsed_since_prompt = (
                    (self._now_ns() - self.last_prompt_pub_ns) * 1e-9
                    if self.last_prompt_pub_ns
                    else float("inf")
                )
                should_publish_prompt = elapsed_since_prompt >= max(
                    self.prompt_republish_sec,
                    0.1,
                )

            status_text, valid = self._debug_status_locked()
            advanced = self._maybe_auto_advance_locked(valid)

        if advanced:
            self._publish_current_prompt(announce=True)
            return

        if should_publish_prompt:
            self._publish_current_prompt(announce=False)

        self._publish_status_periodic(status_text)


def main(args=None):
    rclpy.init(args=args)
    node = KeypointDebugPlanPlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
