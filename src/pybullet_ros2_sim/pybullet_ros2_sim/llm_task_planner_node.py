#!/usr/bin/env python3
"""Use an LLM to decompose a natural-language instruction into executable steps."""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pybullet_ros2_sim.task_plan_utils import (
    PLAN_JSON_SCHEMA,
    SUPPORTED_TARGET_PROMPTS,
    infer_plan_from_instruction,
    plan_to_json,
    scene_registry_from_json,
    scene_registry_summary,
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


def _extract_chat_completion_text(response_json):
    """Pull assistant content from a chat-completions payload."""
    for choice in response_json.get("choices", []):
        message = choice.get("message", {})
        content = message.get("content")
        if isinstance(content, str) and content.strip():
            return content
        if isinstance(content, list):
            text_parts = []
            for item in content:
                if not isinstance(item, dict):
                    continue
                text = item.get("text")
                if isinstance(text, str) and text.strip():
                    text_parts.append(text)
            joined = "".join(text_parts).strip()
            if joined:
                return joined
    raise ValueError("No assistant text payload found in chat completion response")


def _extract_json_object(text: str) -> dict:
    """Best-effort JSON object extraction for local model responses."""
    cleaned = re.sub(r"<think>.*?</think>", "", text or "", flags=re.DOTALL).strip()
    if cleaned.startswith("```"):
        cleaned = re.sub(r"^```[a-zA-Z0-9_-]*\n", "", cleaned)
        cleaned = re.sub(r"\n```$", "", cleaned)

    try:
        parsed = json.loads(cleaned)
        if isinstance(parsed, dict):
            return parsed
    except Exception:
        pass

    start = cleaned.find("{")
    if start < 0:
        raise ValueError("No JSON object found in model response")

    depth = 0
    in_string = False
    escape = False
    for index in range(start, len(cleaned)):
        char = cleaned[index]
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                candidate = cleaned[start : index + 1]
                parsed = json.loads(candidate)
                if isinstance(parsed, dict):
                    return parsed
                break

    raise ValueError("Unable to isolate a valid JSON object from model response")


class LlmTaskPlanner(Node):
    """Convert user instructions into a sanitized JSON step plan."""

    def __init__(self):
        super().__init__("llm_task_planner_node")

        self.declare_parameter("instruction_topic", "/llm_task/instruction")
        self.declare_parameter("plan_topic", "/llm_task/plan_json")
        self.declare_parameter("status_topic", "/llm_task/status")
        self.declare_parameter("scene_topic", "/scene/objects_json")
        self.declare_parameter("api_protocol", "responses")
        self.declare_parameter("model", "gpt-5-mini")
        self.declare_parameter("reasoning_effort", "low")
        self.declare_parameter("api_base_url", "https://api.openai.com/v1/responses")
        self.declare_parameter("api_key", "")
        self.declare_parameter("api_key_env_var", "OPENAI_API_KEY")
        self.declare_parameter("api_key_required", True)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("top_p", 1.0)
        self.declare_parameter("max_output_tokens", 512)
        self.declare_parameter("extra_request_body_json", "{}")
        self.declare_parameter("request_timeout_sec", 45.0)

        self.instruction_topic = str(self.get_parameter("instruction_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.scene_topic = str(self.get_parameter("scene_topic").value)
        self.api_protocol = str(self.get_parameter("api_protocol").value).strip().lower()
        self.model = str(self.get_parameter("model").value)
        self.reasoning_effort = str(self.get_parameter("reasoning_effort").value)
        self.api_base_url = str(self.get_parameter("api_base_url").value)
        self.api_key = str(self.get_parameter("api_key").value)
        self.api_key_env_var = str(self.get_parameter("api_key_env_var").value)
        self.api_key_required = bool(self.get_parameter("api_key_required").value)
        self.temperature = float(self.get_parameter("temperature").value)
        self.top_p = float(self.get_parameter("top_p").value)
        self.max_output_tokens = int(self.get_parameter("max_output_tokens").value)
        self.extra_request_body = self._parse_extra_request_body(
            str(self.get_parameter("extra_request_body_json").value)
        )
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.latest_scene_registry = {"target_frame": "world", "updated_at_sec": 0.0, "objects": []}

        self.pub_plan = self.create_publisher(String, self.plan_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.sub_instruction = self.create_subscription(
            String,
            self.instruction_topic,
            self.on_instruction,
            10,
        )
        self.sub_scene = self.create_subscription(
            String,
            self.scene_topic,
            self.on_scene_registry,
            10,
        )

        self.get_logger().info(
            "llm_task_planner_node started. "
            f"instruction_topic={self.instruction_topic}, "
            f"api_protocol={self.api_protocol}, "
            f"model={self.model}"
        )

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _parse_extra_request_body(self, raw_text: str) -> dict:
        text = (raw_text or "").strip()
        if not text:
            return {}
        try:
            payload = json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"extra_request_body_json is not valid JSON: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("extra_request_body_json must decode to a JSON object")
        return payload

    def _resolve_api_key(self) -> str:
        inline_key = self.api_key.strip()
        if inline_key:
            return inline_key

        env_key = ""
        if self.api_key_env_var:
            env_key = os.environ.get(self.api_key_env_var, "").strip()
        if env_key:
            return env_key

        if self.api_key_required:
            if self.api_key_env_var:
                raise RuntimeError(
                    f"Missing API key: set parameter api_key or environment variable {self.api_key_env_var}"
                )
            raise RuntimeError("Missing API key: set parameter api_key")
        return ""

    def _request_headers(self, api_key: str) -> dict:
        headers = {"Content-Type": "application/json"}
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"
        return headers

    def _messages_for_instruction(self, instruction_text: str) -> list[dict]:
        scene_summary = scene_registry_summary(self.latest_scene_registry)
        return [
            {"role": "system", "content": self._system_prompt()},
            {"role": "user", "content": f"Current scene summary: {scene_summary}"},
            {"role": "user", "content": instruction_text},
        ]

    def _build_responses_payload(self, instruction_text: str) -> dict:
        payload = {
            "model": self.model,
            "input": self._messages_for_instruction(instruction_text),
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
        if self.max_output_tokens > 0:
            payload["max_output_tokens"] = self.max_output_tokens
        return payload

    def _build_chat_completions_payload(self, instruction_text: str) -> dict:
        payload = {
            "model": self.model,
            "messages": self._messages_for_instruction(instruction_text),
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "robot_task_plan",
                    "schema": PLAN_JSON_SCHEMA,
                },
            },
            "temperature": self.temperature,
            "top_p": self.top_p,
        }
        if self.max_output_tokens > 0:
            payload["max_tokens"] = self.max_output_tokens
        if self.extra_request_body:
            payload.update(self.extra_request_body)
        return payload

    def on_scene_registry(self, msg: String):
        self.latest_scene_registry = scene_registry_from_json(msg.data)

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
            "Use the scene summary to decide which objects are currently visible and grounded. "
            "Return JSON only."
        )

    def _request_plan(self, instruction_text: str) -> dict:
        api_key = self._resolve_api_key()

        if self.api_protocol == "responses":
            payload = self._build_responses_payload(instruction_text)
        elif self.api_protocol == "chat_completions":
            payload = self._build_chat_completions_payload(instruction_text)
        else:
            raise ValueError(
                f"Unsupported api_protocol={self.api_protocol!r}; use 'responses' or 'chat_completions'"
            )

        request = urllib.request.Request(
            self.api_base_url,
            data=json.dumps(payload).encode("utf-8"),
            headers=self._request_headers(api_key),
            method="POST",
        )

        with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
            response_json = json.loads(response.read().decode("utf-8"))

        if self.api_protocol == "responses":
            plan_text = _extract_response_text(response_json)
        else:
            plan_text = _extract_chat_completion_text(response_json)

        return sanitize_plan(_extract_json_object(plan_text))

    def _plan_with_fallback(self, instruction: str) -> tuple[dict, bool]:
        try:
            plan = self._request_plan(instruction)
            if plan["steps"]:
                return plan, False
            self.get_logger().warning(
                "[PLAN] LLM returned no executable steps after sanitization; trying local fallback"
            )
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="ignore")
            self.get_logger().error(f"[PLAN] HTTPError {exc.code}: {detail}")
        except Exception as exc:
            self.get_logger().error(f"[PLAN] failed: {exc}")

        fallback_plan = infer_plan_from_instruction(instruction)
        if fallback_plan["steps"]:
            self.get_logger().warning(
                f"[PLAN] using local fallback with {len(fallback_plan['steps'])} steps"
            )
            return fallback_plan, True
        return fallback_plan, True

    def on_instruction(self, msg: String):
        instruction = msg.data.strip()
        if not instruction:
            return

        self.get_logger().info(f"[PLAN] instruction={instruction}")
        self._publish_status("planning")

        plan, used_fallback = self._plan_with_fallback(instruction)

        if not plan["steps"]:
            self.get_logger().warning("[PLAN] no executable steps returned")
            self._publish_status("planning_failed: no_steps")
            return

        out = String()
        out.data = plan_to_json(plan)
        self.pub_plan.publish(out)
        if used_fallback:
            self._publish_status(f"planned_fallback: {len(plan['steps'])} steps")
        else:
            self._publish_status(f"planned: {len(plan['steps'])} steps")
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
