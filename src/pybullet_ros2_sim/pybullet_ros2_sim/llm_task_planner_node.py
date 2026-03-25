#!/usr/bin/env python3
"""Use an LLM to decompose a natural-language instruction into executable steps."""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pybullet_ros2_sim.task_plan_utils import (
    PLAN_JSON_SCHEMA,
    SUPPORTED_TARGET_PROMPTS,
    plan_to_json,
    sanitize_plan,
)


def _extract_response_text(response_json):
    """Pull the first response text block from a Responses API payload."""
    output_text = response_json.get("output_text")
    if isinstance(output_text, str) and output_text.strip():
        return output_text

    for output_item in response_json.get("output", []):
        for content in output_item.get("content", []):
            text = content.get("text")
            if isinstance(text, str) and text.strip():
                return text
    raise ValueError("No text payload found in LLM response")


class LlmTaskPlanner(Node):
    """Convert user instructions into a sanitized JSON step plan."""

    def __init__(self):
        super().__init__("llm_task_planner_node")

        self.declare_parameter("instruction_topic", "/llm_task/instruction")
        self.declare_parameter("plan_topic", "/llm_task/plan_json")
        self.declare_parameter("status_topic", "/llm_task/status")
        self.declare_parameter("model", "gpt-5-mini")
        self.declare_parameter("reasoning_effort", "low")
        self.declare_parameter("api_base_url", "https://api.openai.com/v1/responses")
        self.declare_parameter("request_timeout_sec", 45.0)

        self.instruction_topic = str(self.get_parameter("instruction_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.model = str(self.get_parameter("model").value)
        self.reasoning_effort = str(self.get_parameter("reasoning_effort").value)
        self.api_base_url = str(self.get_parameter("api_base_url").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)

        self.pub_plan = self.create_publisher(String, self.plan_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.sub_instruction = self.create_subscription(
            String,
            self.instruction_topic,
            self.on_instruction,
            10,
        )

        self.get_logger().info(
            f"llm_task_planner_node started. instruction_topic={self.instruction_topic}"
        )

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _system_prompt(self) -> str:
        supported_targets = ", ".join(SUPPORTED_TARGET_PROMPTS)
        return (
            "You are a tabletop robot task planner. "
            "Convert the user's instruction into a short executable JSON task plan. "
            "The robot can only do two things: "
            "1) hover above one visible cube, "
            "2) wait for a short duration. "
            "The only valid hover targets are: "
            f"{supported_targets}. "
            "Use action='hover_target' when the step should move above an object. "
            "Use action='wait' when the robot should pause. "
            "If the user asks for unsupported actions like grasping or stacking, "
            "decompose only the observable hover sequence and mention the limitation in planning_notes. "
            "Return JSON only."
        )

    def _request_plan(self, instruction_text: str) -> dict:
        api_key = os.environ.get("OPENAI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY is not set")

        payload = {
            "model": self.model,
            "input": [
                {"role": "system", "content": self._system_prompt()},
                {"role": "user", "content": instruction_text},
            ],
            "text": {
                "format": {
                    "type": "json_schema",
                    "name": "robot_task_plan",
                    "schema": PLAN_JSON_SCHEMA,
                    "strict": True,
                }
            },
        }
        if self.reasoning_effort:
            payload["reasoning"] = {"effort": self.reasoning_effort}

        request = urllib.request.Request(
            self.api_base_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )

        with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
            response_json = json.loads(response.read().decode("utf-8"))

        plan_text = _extract_response_text(response_json)
        return sanitize_plan(json.loads(plan_text))

    def on_instruction(self, msg: String):
        instruction = msg.data.strip()
        if not instruction:
            return

        self.get_logger().info(f"[PLAN] instruction={instruction}")
        self._publish_status(f"planning: {instruction}")

        try:
            plan = self._request_plan(instruction)
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="ignore")
            self.get_logger().error(f"[PLAN] HTTPError {exc.code}: {detail}")
            self._publish_status(f"planning_failed: http_{exc.code}")
            return
        except Exception as exc:
            self.get_logger().error(f"[PLAN] failed: {exc}")
            self._publish_status("planning_failed")
            return

        if not plan["steps"]:
            self.get_logger().warning("[PLAN] no executable steps returned")
            self._publish_status("planning_failed: no_steps")
            return

        out = String()
        out.data = plan_to_json(plan)
        self.pub_plan.publish(out)
        self._publish_status(f"planned: {plan['task_summary']} ({len(plan['steps'])} steps)")
        self.get_logger().info(f"[PLAN] published {len(plan['steps'])} steps")


def main(args=None):
    rclpy.init(args=args)
    node = LlmTaskPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
