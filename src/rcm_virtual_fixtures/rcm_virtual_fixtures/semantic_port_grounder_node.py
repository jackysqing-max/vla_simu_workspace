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

        self.create_subscription(String, self.instruction_topic, self.on_instruction, 10)
        self.create_subscription(String, self.candidates_topic, self.on_candidates, qos_latched)
        self.create_subscription(Image, self.candidate_overlay_topic, self.on_overlay, qos_image)
        self.create_subscription(Image, self.color_topic, self.on_color, qos_image)

        self.lock = threading.Lock()
        self.pending_instruction = ""
        self.pending_request_id = 0
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
            f"overlay={self.candidate_overlay_topic}, model={self.model}"
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
        with self.lock:
            self.pending_instruction = instruction
            self.pending_request_id += 1
        self._publish_status(
            f"semantic_request_received: {instruction}",
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
            candidates_version = self.latest_candidates_version
            overlay_msg = self.latest_overlay_msg
            color_msg = self.latest_color_msg
            self.inflight = True

        thread = threading.Thread(
            target=self._run_grounding_request,
            args=(request_id, instruction, candidates_version, candidates_payload, overlay_msg, color_msg),
            daemon=True,
        )
        thread.start()

    def _candidate_summary(self, payload: dict) -> list[dict]:
        summary = []
        for candidate in candidates_from_payload(payload):
            summary.append(
                {
                    "candidate_id": candidate["candidate_id"],
                    "image_center": candidate.get("image_center"),
                    "position_world": candidate.get("position_world"),
                    "axis_world": candidate.get("axis_world"),
                    "surface_normal": candidate.get("surface_normal"),
                    "geometry_confidence": candidate.get("geometry_confidence"),
                    "perception_quality": candidate.get("perception_quality"),
                    "temporal_stability": candidate.get("temporal_stability"),
                    "visibility": candidate.get("visibility"),
                    "aperture_area": candidate.get("aperture_area"),
                    "circularity": candidate.get("circularity"),
                    "plane_fit_rms": candidate.get("plane_fit_rms"),
                    "rcm_feasible": candidate.get("rcm_feasible"),
                    "ik_margin": candidate.get("ik_margin"),
                    "collision_feasible": candidate.get("collision_feasible"),
                    "semantic_risk": candidate.get("semantic_risk"),
                }
            )
        return summary

    def _system_prompt(self) -> str:
        return (
            "You ground open natural-language RCM port instructions to the currently "
            "visible dynamic candidate IDs. Use the marked image and the candidate "
            "feature JSON. Do not invent candidate IDs. Do not output robot poses, "
            "joint commands, torques, or trajectories. Return JSON only with: "
            "objective_text, selected_candidate_id, candidate_ranking, "
            "scoring_program, evidence, ambiguous, allow_alternative, "
            "excluded_candidate_ids, reference_description, resolved_frame_id. "
            "candidate_ranking is a list of objects with candidate_id, semantic_score "
            "in [0,1], and reason. scoring_program must be one single expression "
            "over candidate features and registered functions only. Useful functions: "
            "distance_to(entity), clearance_to(entity), alignment_with(entity), "
            "image_leftness(), image_rightness(), image_upness(), image_downness(), "
            "between(entity_a, entity_b), visibility(), "
            "previous_selection_similarity(), semantic_risk(), "
            "unknown_space_exposure(). Use names from scene_features when needed. "
            "If the request is ambiguous or infeasible, set ambiguous=true and still "
            "explain the uncertainty in evidence."
        )

    def _build_model_payload(
        self,
        *,
        instruction: str,
        request_snapshot: dict,
        overlay_msg: Image | None,
        color_msg: Image | None,
    ) -> dict:
        overlay_url = image_to_data_url(overlay_msg)
        color_url = image_to_data_url(color_msg)
        text = (
            "Ground this RCM port instruction against the visible candidate IDs.\n\n"
            f"Instruction: {instruction}\n\n"
            "Candidate feature snapshot JSON:\n"
            f"{json.dumps(request_snapshot, ensure_ascii=False, separators=(',', ':'))}\n\n"
            "Return JSON only."
        )
        content = [{"type": "text", "text": text}]
        if color_url:
            content.append({"type": "image_url", "image_url": {"url": color_url}})
        if overlay_url:
            content.append({"type": "image_url", "image_url": {"url": overlay_url}})
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self._system_prompt()},
                {"role": "user", "content": content},
            ],
            "temperature": self.temperature,
            "top_p": self.top_p,
        }
        if self.max_output_tokens > 0:
            payload["max_tokens"] = self.max_output_tokens
        if self.response_format_json:
            payload["response_format"] = {"type": "json_object"}
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
        with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
            response_json = json.loads(response.read().decode("utf-8"))
        response_text = extract_chat_completion_text(response_json)
        return extract_json_object(response_text), response_text

    def _publish_json(self, publisher, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(msg)

    def _run_grounding_request(
        self,
        request_id: int,
        instruction: str,
        candidates_version: int,
        candidates_payload: dict,
        overlay_msg: Image | None,
        color_msg: Image | None,
    ):
        request_snapshot = {
            "request_id": request_id,
            "instruction": instruction,
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
            model_payload = self._build_model_payload(
                instruction=instruction,
                request_snapshot=request_snapshot,
                overlay_msg=overlay_msg,
                color_msg=color_msg,
            )
            hypothesis, raw_response = self._request_model(model_payload)
            hypothesis.setdefault("instruction", instruction)
            hypothesis["raw_model_response"] = raw_response[:4000]
            self._publish_json(self.pub_hypothesis, hypothesis)
            verification = verify_semantic_grounding(
                candidates_payload,
                hypothesis,
                previous_selection=self.previous_selection,
                thresholds=self.thresholds,
            )
        except (urllib.error.URLError, urllib.error.HTTPError, TimeoutError, Exception) as exc:
            hypothesis = {
                "instruction": instruction,
                "objective_text": instruction,
                "candidate_ranking": [],
                "scoring_program": "",
                "ambiguous": True,
                "allow_alternative": False,
                "grounding_error": str(exc),
                "raw_model_response": raw_response[:4000],
            }
            self._publish_json(self.pub_hypothesis, hypothesis)
            verification = {
                "decision": "REQUERY",
                "reason": f"semantic_model_failed:{exc}",
                "selected_candidate_id": None,
                "selected_id": None,
                "candidate_scores": [],
                "objective_text": instruction,
                "allow_alternative": False,
                "verification_source": "semantic_port_grounder_node",
            }

        verification["instruction"] = instruction
        verification["request_id"] = request_id
        verification["candidate_snapshot_version"] = candidates_version
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
                f"semantic_verified_accept: selected={selected_id}",
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
