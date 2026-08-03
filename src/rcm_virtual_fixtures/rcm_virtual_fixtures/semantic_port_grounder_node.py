#!/usr/bin/env python3
"""Ground open language instructions to dynamic RCM port candidates."""

from __future__ import annotations

import base64
import json
import os
import re
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rcm_virtual_fixtures.port_grounding_expression import (
    ScoringExpressionError,
    score_candidates,
)
from rcm_virtual_fixtures.semantic_geometric_verifier import (
    VerificationThresholds,
    candidate_id_text,
    candidates_from_payload,
    verify_semantic_grounding,
)


def image_to_bgr(msg: Image) -> np.ndarray:
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    encoding = str(msg.encoding).lower()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def image_to_data_url(msg: Image | None) -> str:
    if msg is None:
        return ""
    try:
        image = image_to_bgr(msg)
        ok, encoded = cv2.imencode(".png", image)
    except Exception:
        return ""
    if not ok:
        return ""
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:image/png;base64,{payload}"


def extract_chat_completion_text(response_json: dict) -> str:
    for choice in response_json.get("choices", []):
        message = choice.get("message", {})
        content = message.get("content")
        if isinstance(content, str) and content.strip():
            return content
        if isinstance(content, list):
            parts = []
            for item in content:
                if isinstance(item, dict) and isinstance(item.get("text"), str):
                    parts.append(item["text"])
            joined = "".join(parts).strip()
            if joined:
                return joined
        for reasoning_key in ("reasoning_content", "reasoning"):
            reasoning = message.get(reasoning_key)
            if isinstance(reasoning, str) and reasoning.strip():
                return reasoning
    raise ValueError("No assistant content found in model response")


def extract_json_object(text: str) -> dict:
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
        raise ValueError("model response did not contain a JSON object")
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
                parsed = json.loads(cleaned[start:index + 1])
                if isinstance(parsed, dict):
                    return parsed
    raise ValueError("unable to isolate a valid JSON object")


class SemanticPortGrounder(Node):
    """Ask Qwen-VL for candidate semantics, then verify the result."""

    def __init__(self):
        super().__init__("semantic_port_grounder_node")

        self.declare_parameter("instruction_topic", "/vlm_rcm/language_command")
        self.declare_parameter("candidates_topic", "/vlm_rcm/hole_candidates")
        self.declare_parameter("candidate_overlay_topic", "/vlm_rcm/candidate_overlay")
        self.declare_parameter("color_topic", "/sim/camera/color/image_raw")
        self.declare_parameter("sam_prompt_topic", "/sam3/prompt")
        self.declare_parameter("language_control_topic", "/vlm_rcm/language_control")
        self.declare_parameter("publish_sam_prompt_on_instruction", True)
        self.declare_parameter("semantic_request_topic", "/vlm_rcm/semantic_grounding_request")
        self.declare_parameter("semantic_hypothesis_topic", "/vlm_rcm/semantic_hypothesis")
        self.declare_parameter("verification_result_topic", "/vlm_rcm/verification_result")
        self.declare_parameter("verified_selection_topic", "/vlm_rcm/verified_selected_port")
        self.declare_parameter("status_topic", "/vlm_rcm/semantic_status")
        self.declare_parameter("api_base_url", "http://127.0.0.1:8000/v1/chat/completions")
        self.declare_parameter("api_key", "EMPTY")
        self.declare_parameter("api_key_env_var", "")
        self.declare_parameter("api_key_required", False)
        self.declare_parameter("model", "Qwen/Qwen3-VL-4B-Instruct")
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("top_p", 1.0)
        self.declare_parameter("max_output_tokens", 768)
        self.declare_parameter("request_timeout_sec", 90.0)
        self.declare_parameter("extra_request_body_json", "{}")
        self.declare_parameter("response_format_json", False)
        self.declare_parameter("input_mode", "text")
        self.declare_parameter("enable_local_fallback", False)
        self.declare_parameter("retry_text_without_images", True)
        self.declare_parameter("require_post_instruction_candidates", True)
        self.declare_parameter("require_scoring_program", True)
        self.declare_parameter("min_model_margin", 0.08)
        self.declare_parameter("min_expression_margin", 1e-6)
        self.declare_parameter("min_temporal_stability", 0.0)
        self.declare_parameter("request_poll_hz", 2.0)

        self.instruction_topic = str(self.get_parameter("instruction_topic").value)
        self.candidates_topic = str(self.get_parameter("candidates_topic").value)
        self.candidate_overlay_topic = str(
            self.get_parameter("candidate_overlay_topic").value
        )
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.sam_prompt_topic = str(self.get_parameter("sam_prompt_topic").value)
        self.language_control_topic = str(
            self.get_parameter("language_control_topic").value
        )
        self.publish_sam_prompt_on_instruction = bool(
            self.get_parameter("publish_sam_prompt_on_instruction").value
        )
        self.api_base_url = str(self.get_parameter("api_base_url").value).strip()
        self.api_key = str(self.get_parameter("api_key").value)
        self.api_key_env_var = str(self.get_parameter("api_key_env_var").value)
        self.api_key_required = bool(self.get_parameter("api_key_required").value)
        self.model = str(self.get_parameter("model").value)
        self.temperature = float(self.get_parameter("temperature").value)
        self.top_p = float(self.get_parameter("top_p").value)
        self.max_output_tokens = int(self.get_parameter("max_output_tokens").value)
        self.request_timeout_sec = float(
            self.get_parameter("request_timeout_sec").value
        )
        self.extra_request_body = self._parse_extra_request_body(
            str(self.get_parameter("extra_request_body_json").value)
        )
        self.response_format_json = bool(
            self.get_parameter("response_format_json").value
        )
        self.input_mode = str(self.get_parameter("input_mode").value).strip().lower()
        self.enable_local_fallback = bool(
            self.get_parameter("enable_local_fallback").value
        )
        if self.input_mode in {"local", "local_only", "local-only", "rules"}:
            raise ValueError(
                "strict semantic grounding forbids local/rule input modes; "
                "start a real model backend and use input_mode=text or vision"
            )
        if self.enable_local_fallback:
            raise ValueError(
                "strict semantic grounding forbids local fallback; "
                "set enable_local_fallback:=false"
            )
        self.retry_text_without_images = bool(
            self.get_parameter("retry_text_without_images").value
        )
        self.require_post_instruction_candidates = bool(
            self.get_parameter("require_post_instruction_candidates").value
        )
        self.thresholds = VerificationThresholds(
            min_model_margin=float(self.get_parameter("min_model_margin").value),
            min_expression_margin=float(
                self.get_parameter("min_expression_margin").value
            ),
            min_temporal_stability=float(
                self.get_parameter("min_temporal_stability").value
            ),
            require_scoring_program=bool(
                self.get_parameter("require_scoring_program").value
            ),
        )

        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_image = QoSProfile(depth=1)
        qos_image.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_image.durability = DurabilityPolicy.VOLATILE

        self.pub_request = self.create_publisher(
            String,
            str(self.get_parameter("semantic_request_topic").value),
            qos_latched,
        )
        self.pub_hypothesis = self.create_publisher(
            String,
            str(self.get_parameter("semantic_hypothesis_topic").value),
            qos_latched,
        )
        self.pub_verification = self.create_publisher(
            String,
            str(self.get_parameter("verification_result_topic").value),
            qos_latched,
        )
        self.pub_verified_selection = self.create_publisher(
            String,
            str(self.get_parameter("verified_selection_topic").value),
            qos_latched,
        )
        self.pub_status = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        # Retain the latest language-derived prompt while SAM3 is still loading.
        self.pub_sam_prompt = self.create_publisher(
            String,
            self.sam_prompt_topic,
            qos_latched,
        )
        self.pub_language_control = self.create_publisher(
            String,
            self.language_control_topic,
            qos_latched,
        )

        self.create_subscription(String, self.instruction_topic, self.on_instruction, 10)
        self.create_subscription(String, self.candidates_topic, self.on_candidates, qos_latched)
        self.create_subscription(Image, self.candidate_overlay_topic, self.on_overlay, qos_image)
        self.create_subscription(Image, self.color_topic, self.on_color, qos_image)

        self.lock = threading.Lock()
        self.pending_instruction = ""
        self.pending_request_id = 0
        self.active_instruction_text = ""
        self.intent_query_inflight = False
        self.pending_instruction_received_wall_time_sec = 0.0
        self.pending_instruction_received_ros_stamp_sec = 0.0
        self.latest_candidates_payload: dict | None = None
        self.latest_candidates_version = 0
        self.latest_overlay_msg = None
        self.latest_color_msg = None
        self.inflight = False
        self.previous_selection = None
        self.last_status_time = 0.0

        poll_hz = max(float(self.get_parameter("request_poll_hz").value), 0.2)
        self.create_timer(1.0 / poll_hz, self.on_timer)
        self.get_logger().info(
            "semantic_port_grounder_node started: "
            f"instruction={self.instruction_topic}, candidates={self.candidates_topic}, "
            f"overlay={self.candidate_overlay_topic}, model={self.model}, "
            f"input_mode={self.input_mode}, strict_model_only=true, "
            f"sam_prompt_topic={self.sam_prompt_topic}"
        )

    def _parse_extra_request_body(self, raw_text: str) -> dict:
        text = (raw_text or "").strip()
        if not text:
            return {}
        parsed = json.loads(text)
        if not isinstance(parsed, dict):
            raise ValueError("extra_request_body_json must decode to an object")
        return parsed

    def _publish_status(self, text: str, *, force: bool = False):
        now = time.monotonic()
        if not force and now - self.last_status_time < 0.25:
            return
        self.last_status_time = now
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _resolve_api_key(self) -> str:
        if self.api_key.strip():
            return self.api_key.strip()
        if self.api_key_env_var:
            value = os.environ.get(self.api_key_env_var, "").strip()
            if value:
                return value
        if self.api_key_required:
            raise RuntimeError("missing Qwen-VL API key")
        return ""

    def on_instruction(self, msg: String):
        instruction = str(msg.data or "").strip()
        if not instruction:
            return
        received_wall_time_sec = time.time()
        received_ros_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        with self.lock:
            if instruction == self.active_instruction_text:
                return
            self.pending_request_id += 1
            request_id = self.pending_request_id
            self.pending_instruction = ""
            self.active_instruction_text = instruction
            self.intent_query_inflight = True
        self._publish_status(
            f"strict_llm_intent_query_started: request_id={request_id}",
            force=True,
        )
        thread = threading.Thread(
            target=self._run_instruction_interpretation,
            args=(
                request_id,
                instruction,
                received_wall_time_sec,
                received_ros_stamp_sec,
            ),
            daemon=True,
        )
        thread.start()

    def _instruction_interpreter_payload(
        self,
        instruction: str,
        *,
        invalid_sam_prompt: str = "",
    ) -> dict:
        system_prompt = (
            "You are the strict natural-language control and perception front end "
            "for an RCM port localization demo. Interpret unrestricted multilingual "
            "instructions semantically; do not use a fixed phrase lookup. Return one "
            "JSON object only with keys: action, target_description, sam_prompt, "
            "reference_frame, ambiguity. action must be unlock, locate, "
            "or reject. Use unlock only when the user asks only to release/clear the "
            "current lock. If the user asks to unlock and name a new target, use locate. "
            "For locate, sam_prompt must contain only a concise English visual class "
            "noun phrase for all relevant physical apertures, such as circular hole; "
            "exclude left/right/up/down, ordinals, and other positional modifiers from "
            "sam_prompt because candidate selection happens after segmentation. Preserve "
            "spatial meaning such as left down, upper right, relative "
            "objects, ordinals, and camera-view references in target_description. "
            "When the user explicitly says camera view, interpret left/right as image "
            "horizontal position and up/down as image vertical position; ordinary "
            "phrases such as left down are not ambiguous. Set ambiguity=true only "
            "when the target genuinely cannot be distinguished from the instruction."
        )
        control_schema = {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["unlock", "locate", "reject"]},
                "target_description": {"type": "string"},
                "sam_prompt": {"type": "string"},
                "reference_frame": {"type": "string"},
                "ambiguity": {"type": "boolean"},
            },
            "required": [
                "action",
                "target_description",
                "sam_prompt",
                "reference_frame",
                "ambiguity",
            ],
            "additionalProperties": False,
        }
        user_content = instruction
        if invalid_sam_prompt:
            user_content = (
                f"Original instruction: {instruction}\n"
                f"Your previous sam_prompt {invalid_sam_prompt!r} was invalid because "
                "the segmentation encoder requires an ASCII English visual class noun "
                "phrase. Return the corrected complete JSON object."
            )
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
            ],
            "temperature": self.temperature,
            "top_p": self.top_p,
            "chat_template_kwargs": {"enable_thinking": False},
            "response_format": {
                "type": "json_schema",
                "json_schema": {
                    "name": "rcm_language_control",
                    "strict": True,
                    "schema": control_schema,
                },
            },
        }
        if self.max_output_tokens > 0:
            payload["max_tokens"] = min(self.max_output_tokens, 384)
        if self.extra_request_body:
            payload.update(self.extra_request_body)
        return payload

    def _run_instruction_interpretation(
        self,
        request_id: int,
        instruction: str,
        received_wall_time_sec: float,
        received_ros_stamp_sec: float,
    ):
        try:
            interpretation, raw_response = self._request_model(
                self._instruction_interpreter_payload(instruction)
            )
            self.get_logger().info(
                "[LLM_INTENT_RAW] "
                + json.dumps(interpretation, ensure_ascii=False, separators=(",", ":"))
            )
            action = str(interpretation.get("action", "")).strip().lower()
            if action not in {"unlock", "locate", "reject"}:
                raise ValueError(f"model returned invalid action {action!r}")
            ambiguity = interpretation.get("ambiguity")
            if not isinstance(ambiguity, bool):
                raise ValueError("model returned non-boolean ambiguity")
            sam_prompt = str(interpretation.get("sam_prompt", "")).strip()
            if (
                action == "locate"
                and sam_prompt
                and (
                    not sam_prompt.isascii()
                    or re.search(r"[A-Za-z]", sam_prompt) is None
                )
            ):
                invalid_prompt = sam_prompt
                interpretation, corrected_raw = self._request_model(
                    self._instruction_interpreter_payload(
                        instruction,
                        invalid_sam_prompt=invalid_prompt,
                    )
                )
                raw_response = raw_response + "\nCORRECTION:\n" + corrected_raw
                self.get_logger().info(
                    "[LLM_INTENT_CORRECTED] "
                    + json.dumps(
                        interpretation,
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                )
                action = str(interpretation.get("action", "")).strip().lower()
                ambiguity = interpretation.get("ambiguity")
                sam_prompt = str(interpretation.get("sam_prompt", "")).strip()
                if action not in {"unlock", "locate", "reject"}:
                    raise ValueError(
                        f"model correction returned invalid action {action!r}"
                    )
                if not isinstance(ambiguity, bool):
                    raise ValueError(
                        "model correction returned non-boolean ambiguity"
                    )
                if (
                    action == "locate"
                    and (
                        not sam_prompt.isascii()
                        or re.search(r"[A-Za-z]", sam_prompt) is None
                    )
                ):
                    raise ValueError(
                        "model correction did not produce an ASCII English SAM prompt"
                    )
            if action == "locate" and (ambiguity or not sam_prompt):
                raise ValueError(
                    "model did not provide an unambiguous locate target and SAM prompt"
                )
        except Exception as exc:
            with self.lock:
                if self.pending_request_id == request_id:
                    self.intent_query_inflight = False
                    self.active_instruction_text = ""
            self.get_logger().error(
                f"[STRICT_LLM_INTENT_FAILED] request_id={request_id} error={exc}"
            )
            self._publish_status(
                f"strict_llm_intent_failed: request_id={request_id} error={exc}",
                force=True,
            )
            return

        with self.lock:
            if self.pending_request_id != request_id:
                return
            self.intent_query_inflight = False

        control = {
            "schema_version": "vlm_rcm_language_control.v1",
            "source": "llm",
            "model": self.model,
            "request_id": request_id,
            "action": action,
            "instruction": instruction,
            "target_description": str(
                interpretation.get("target_description", "")
            ),
            "sam_prompt": sam_prompt,
            "reference_frame": str(interpretation.get("reference_frame", "")),
            "ambiguity": ambiguity,
            "raw_model_response": raw_response[:4000],
            "instruction_received_wall_time_sec": received_wall_time_sec,
            "instruction_received_ros_stamp_sec": received_ros_stamp_sec,
        }
        self._publish_json(self.pub_language_control, control)

        if action == "unlock":
            with self.lock:
                if self.pending_request_id == request_id:
                    self.pending_instruction = ""
                    self.previous_selection = None
                    self.active_instruction_text = ""
            self._publish_status(
                f"strict_llm_unlock_accepted: request_id={request_id}",
                force=True,
            )
            return
        if action == "reject":
            with self.lock:
                if self.pending_request_id == request_id:
                    self.active_instruction_text = ""
            self._publish_status(
                f"strict_llm_instruction_rejected: request_id={request_id}",
                force=True,
            )
            return

        with self.lock:
            if self.pending_request_id != request_id:
                return
            self.pending_instruction = instruction
            self.pending_instruction_received_wall_time_sec = received_wall_time_sec
            self.pending_instruction_received_ros_stamp_sec = received_ros_stamp_sec
        if self.publish_sam_prompt_on_instruction:
            prompt_msg = String()
            prompt_msg.data = sam_prompt
            self.pub_sam_prompt.publish(prompt_msg)
        self._publish_status(
            f"strict_llm_locate_accepted: request_id={request_id} "
            f"sam_prompt={sam_prompt!r}",
            force=True,
        )

    def on_candidates(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                return
        except Exception:
            return
        with self.lock:
            self.latest_candidates_payload = payload
            self.latest_candidates_version += 1

    def on_overlay(self, msg: Image):
        with self.lock:
            self.latest_overlay_msg = msg

    def on_color(self, msg: Image):
        with self.lock:
            self.latest_color_msg = msg

    def on_timer(self):
        with self.lock:
            if self.inflight or not self.pending_instruction:
                return
            candidates_payload = self.latest_candidates_payload
            if not candidates_payload or not candidates_payload.get("holes"):
                self._publish_status("waiting_for_dynamic_port_candidates")
                return
            instruction = self.pending_instruction
            request_id = self.pending_request_id
            instruction_received_wall_time_sec = (
                self.pending_instruction_received_wall_time_sec
            )
            instruction_received_ros_stamp_sec = (
                self.pending_instruction_received_ros_stamp_sec
            )
            candidate_instruction = str(
                candidates_payload.get("language_instruction", "")
            ).strip()
            if candidate_instruction != instruction:
                self._publish_status(
                    "waiting_for_current_instruction_port_candidates"
                )
                return
            image_size = candidates_payload.get("image_size")
            if (
                not isinstance(image_size, list)
                or len(image_size) < 2
                or float(image_size[0]) <= 1.0
                or float(image_size[1]) <= 1.0
            ):
                self._publish_status(
                    "waiting_for_candidates_with_valid_image_size"
                )
                return
            try:
                candidates_stamp_sec = float(candidates_payload.get("stamp_sec", 0.0))
            except Exception:
                candidates_stamp_sec = 0.0
            if (
                self.require_post_instruction_candidates
                and instruction_received_ros_stamp_sec > 0.0
                and candidates_stamp_sec + 1e-6
                < instruction_received_ros_stamp_sec
            ):
                self._publish_status(
                    "waiting_for_post_instruction_port_candidates"
                )
                return
            candidates_version = self.latest_candidates_version
            overlay_msg = self.latest_overlay_msg
            color_msg = self.latest_color_msg
            self.inflight = True

        thread = threading.Thread(
            target=self._run_grounding_request,
            args=(
                request_id,
                instruction,
                instruction_received_wall_time_sec,
                instruction_received_ros_stamp_sec,
                candidates_version,
                candidates_payload,
                overlay_msg,
                color_msg,
            ),
            daemon=True,
        )
        thread.start()

    def _candidate_summary(self, payload: dict) -> list[dict]:
        summary = []
        for candidate in candidates_from_payload(payload):
            image_center = candidate.get("image_center")
            if isinstance(image_center, (list, tuple)) and len(image_center) >= 2:
                image_center = [
                    round(float(image_center[0]), 1),
                    round(float(image_center[1]), 1),
                ]
            summary.append(
                {
                    "candidate_id": candidate["candidate_id"],
                    "image_center": image_center,
                    "geometry_confidence": round(
                        float(candidate.get("geometry_confidence", 0.0)),
                        3,
                    ),
                    "feasible": bool(candidate.get("rcm_feasible", False))
                    and bool(candidate.get("collision_feasible", False)),
                }
            )
        return summary

    def _system_prompt(self) -> str:
        return (
            "Ground the unrestricted natural-language RCM port instruction to one of "
            "the currently visible dynamic candidate IDs. Interpret multilingual and "
            "non-canonical spatial language semantically. image_center is [u,v], where "
            "u increases rightward and v increases downward in the camera image. Do not "
            "choose or invent an ID and do not output robot motion. Write one scoring_program "
            "whose value is larger for candidates that better satisfy the instruction. "
            "The program is a single safe expression using image_leftness(), "
            "image_rightness(), image_upness(), image_downness(), image_centeredness(), "
            "visibility(), candidate.geometry_confidence, candidate.id, arithmetic and "
            "comparisons. All registered image_* and visibility functions take no "
            "arguments; never pass image coordinates into them. Include only semantic "
            "properties explicitly requested by the instruction: for example, do not "
            "add image_centeredness() unless the user requested a central target. Combine "
            "independent requested spatial requirements with equal-scale addition so one "
            "requested direction cannot erase another. Feasibility is enforced separately "
            "as a hard deterministic gate. candidate.geometry_confidence and visibility() "
            "may only be optional tie breakers with total coefficient at most 0.01; they "
            "must never dominate the requested semantic relation. Mark ambiguous rather "
            "than guessing when the instruction is genuinely unresolved."
        )

    def _should_include_images(self) -> bool:
        if self.input_mode in {
            "local",
            "local_only",
            "local-only",
            "rules",
            "text",
            "text_only",
            "text-only",
            "no_image",
            "none",
        }:
            return False
        if self.input_mode in {"vision", "vlm", "image", "images"}:
            return True
        model_text = self.model.lower()
        return any(
            token in model_text
            for token in (
                "-vl",
                "_vl",
                "vision",
                "visual",
                "llava",
                "internvl",
                "qwen2-vl",
                "qwen2.5-vl",
                "qwen3-vl",
                "omni",
            )
        )

    def _build_model_payload(
        self,
        *,
        instruction: str,
        request_snapshot: dict,
        overlay_msg: Image | None,
        color_msg: Image | None,
        include_images: bool,
    ) -> dict:
        overlay_url = image_to_data_url(overlay_msg) if include_images else ""
        color_url = image_to_data_url(color_msg) if include_images else ""
        text = (
            "Ground this RCM port instruction against the visible candidate IDs.\n\n"
            f"Instruction: {instruction}\n\n"
            "Candidate feature snapshot JSON:\n"
            f"{json.dumps(request_snapshot, ensure_ascii=False, separators=(',', ':'))}\n\n"
            "Return JSON only."
        )
        if include_images:
            content = [{"type": "text", "text": text}]
            if color_url:
                content.append({"type": "image_url", "image_url": {"url": color_url}})
            if overlay_url:
                content.append({"type": "image_url", "image_url": {"url": overlay_url}})
        else:
            content = text
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self._system_prompt()},
                {"role": "user", "content": content},
            ],
            "temperature": self.temperature,
            "top_p": self.top_p,
            "chat_template_kwargs": {"enable_thinking": False},
        }
        if self.max_output_tokens > 0:
            payload["max_tokens"] = self.max_output_tokens
        grounding_schema = {
            "type": "object",
            "properties": {
                "scoring_program": {"type": "string"},
            },
            "required": [
                "scoring_program",
            ],
            "additionalProperties": False,
        }
        payload["response_format"] = {
            "type": "json_schema",
            "json_schema": {
                "name": "rcm_candidate_grounding",
                "strict": True,
                "schema": grounding_schema,
            },
        }
        if self.extra_request_body:
            payload.update(self.extra_request_body)
        return payload

    def _request_model(self, payload: dict) -> tuple[dict, str]:
        if not self.api_base_url:
            raise RuntimeError("api_base_url is empty; no Qwen-VL endpoint configured")
        api_key = self._resolve_api_key()
        headers = {"Content-Type": "application/json"}
        if api_key:
            headers["Authorization"] = f"Bearer {api_key}"
        request = urllib.request.Request(
            self.api_base_url,
            data=json.dumps(payload).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=self.request_timeout_sec,
            ) as response:
                response_json = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            try:
                detail = exc.read().decode("utf-8", errors="replace")[:2000]
            except Exception:
                detail = ""
            raise RuntimeError(
                f"model HTTP {exc.code}: {detail or exc.reason}"
            ) from exc
        response_text = extract_chat_completion_text(response_json)
        try:
            parsed = extract_json_object(response_text)
        except Exception as exc:
            raise ValueError(
                f"{exc}; raw_model_text={response_text[:1200]!r}"
            ) from exc
        return parsed, response_text

    def _try_model_request(
        self,
        *,
        instruction: str,
        request_snapshot: dict,
        overlay_msg: Image | None,
        color_msg: Image | None,
    ) -> tuple[dict, str]:
        include_images = self._should_include_images()
        model_payload = self._build_model_payload(
            instruction=instruction,
            request_snapshot=request_snapshot,
            overlay_msg=overlay_msg,
            color_msg=color_msg,
            include_images=include_images,
        )
        try:
            return self._request_model(model_payload)
        except urllib.error.HTTPError:
            if not include_images or not self.retry_text_without_images:
                raise
            self._publish_status(
                "semantic_vision_request_failed_retrying_text_only",
                force=True,
            )
            text_payload = self._build_model_payload(
                instruction=instruction,
                request_snapshot=request_snapshot,
                overlay_msg=None,
                color_msg=None,
                include_images=False,
            )
            return self._request_model(text_payload)

    def _publish_json(self, publisher, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(msg)

    def _run_grounding_request(
        self,
        request_id: int,
        instruction: str,
        instruction_received_wall_time_sec: float,
        instruction_received_ros_stamp_sec: float,
        candidates_version: int,
        candidates_payload: dict,
        overlay_msg: Image | None,
        color_msg: Image | None,
    ):
        request_snapshot = {
            "request_id": request_id,
            "instruction": instruction,
            "instruction_received_wall_time_sec": instruction_received_wall_time_sec,
            "instruction_received_ros_stamp_sec": instruction_received_ros_stamp_sec,
            "candidate_snapshot_version": candidates_version,
            "image_size": candidates_payload.get("image_size"),
            "candidates": self._candidate_summary(candidates_payload),
            "previous_selection": self.previous_selection,
        }
        self._publish_json(self.pub_request, request_snapshot)
        self._publish_status(
            f"semantic_model_query_started: request_id={request_id} candidates={len(request_snapshot['candidates'])}",
            force=True,
        )

        raw_response = ""
        try:
            hypothesis, raw_response = self._try_model_request(
                instruction=instruction,
                request_snapshot=request_snapshot,
                overlay_msg=overlay_msg,
                color_msg=color_msg,
            )
            expression = str(hypothesis.get("scoring_program", "")).strip()
            if not expression:
                raise ValueError("model returned an empty scoring_program")
            expression_scores = score_candidates(
                expression,
                candidates_from_payload(candidates_payload),
                scene={},
                previous=self.previous_selection,
            )
            if not expression_scores:
                raise ValueError("model scoring_program produced no candidate scores")
            hypothesis["selected_candidate_id"] = expression_scores[0]["candidate_id"]
            hypothesis["ambiguous"] = False
            hypothesis["evidence"] = "strict LLM-generated scoring program"
            hypothesis["instruction"] = instruction
            hypothesis["objective_text"] = instruction
            hypothesis["allow_alternative"] = False
            hypothesis["excluded_candidate_ids"] = []
            hypothesis["raw_model_response"] = raw_response[:4000]
            hypothesis["grounding_mode"] = "strict_model_only"
            self._publish_json(self.pub_hypothesis, hypothesis)
            verification = verify_semantic_grounding(
                candidates_payload,
                hypothesis,
                previous_selection=self.previous_selection,
                thresholds=self.thresholds,
            )
        except (
            urllib.error.URLError,
            urllib.error.HTTPError,
            TimeoutError,
            ScoringExpressionError,
            Exception,
        ) as exc:
            hypothesis = {
                "instruction": instruction,
                "objective_text": instruction,
                "candidate_ranking": [],
                "scoring_program": "",
                "ambiguous": True,
                "allow_alternative": False,
                "grounding_error": str(exc),
                "raw_model_response": raw_response[:4000],
                "grounding_mode": "strict_model_failure",
            }
            self._publish_json(self.pub_hypothesis, hypothesis)
            verification = {
                "decision": "REQUERY",
                "reason": f"strict_semantic_model_failed:{exc}",
                "selected_candidate_id": None,
                "selected_id": None,
                "candidate_scores": [],
                "objective_text": instruction,
                "allow_alternative": False,
                "verification_source": "semantic_port_grounder_node",
            }

        with self.lock:
            request_is_current = self.pending_request_id == request_id
        if not request_is_current:
            self._publish_status(
                f"semantic_stale_request_discarded: request_id={request_id}",
                force=True,
            )
            with self.lock:
                self.inflight = False
            return

        verification["instruction"] = instruction
        verification["request_id"] = request_id
        verification["candidate_snapshot_version"] = candidates_version
        verification["instruction_received_wall_time_sec"] = (
            instruction_received_wall_time_sec
        )
        verification["instruction_received_ros_stamp_sec"] = (
            instruction_received_ros_stamp_sec
        )
        semantic_completed_wall_time_sec = time.time()
        semantic_completed_ros_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        verification["semantic_completed_wall_time_sec"] = (
            semantic_completed_wall_time_sec
        )
        verification["semantic_completed_ros_stamp_sec"] = (
            semantic_completed_ros_stamp_sec
        )
        if instruction_received_wall_time_sec > 0.0:
            verification["semantic_latency_sec"] = max(
                0.0,
                semantic_completed_wall_time_sec
                - instruction_received_wall_time_sec,
            )
        self._publish_json(self.pub_verification, verification)

        if verification.get("decision") == "ACCEPT":
            selected_id = candidate_id_text(verification.get("selected_candidate_id"))
            verification["selected_candidate_id"] = selected_id
            self._publish_json(self.pub_verified_selection, verification)
            self.previous_selection = self._previous_selection_from_payload(
                candidates_payload,
                selected_id,
            )
            self._publish_status(
                "semantic_verified_accept: "
                f"selected={selected_id} "
                f"semantic_latency={verification.get('semantic_latency_sec', 0.0):.3f}s",
                force=True,
            )
            with self.lock:
                if self.pending_request_id == request_id:
                    self.pending_instruction = ""
        else:
            self._publish_status(
                "semantic_verified_"
                f"{str(verification.get('decision', 'REQUERY')).lower()}: "
                f"{verification.get('reason', '')}",
                force=True,
            )
            with self.lock:
                if self.pending_request_id == request_id:
                    self.pending_instruction = ""

        with self.lock:
            self.inflight = False

    def _previous_selection_from_payload(self, payload: dict, selected_candidate_id: str):
        for candidate in candidates_from_payload(payload):
            if candidate.get("candidate_id") == selected_candidate_id:
                return {
                    "candidate_id": selected_candidate_id,
                    "position_world": candidate.get("position_world"),
                    "center_world": candidate.get("center_world"),
                }
        return {"candidate_id": selected_candidate_id}


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPortGrounder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
