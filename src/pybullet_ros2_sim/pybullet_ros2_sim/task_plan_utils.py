"""Common task-plan schema and validation helpers for LLM execution."""

from __future__ import annotations

import json
import re


SUPPORTED_ACTIONS = ("hover_target", "wait")
SUPPORTED_TARGET_PROMPTS = (
    "red cube",
    "green cube",
    "blue cube",
    "yellow cube",
)


PLAN_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "task_summary": {"type": "string"},
        "planning_notes": {"type": "string"},
        "steps": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "step_index": {"type": "integer"},
                    "action": {
                        "type": "string",
                        "enum": list(SUPPORTED_ACTIONS),
                    },
                    "target_prompt": {"type": "string"},
                    "description": {"type": "string"},
                    "success_radius_m": {"type": "number"},
                    "dwell_sec": {"type": "number"},
                    "wait_sec": {"type": "number"},
                },
                "required": [
                    "step_index",
                    "action",
                    "target_prompt",
                    "description",
                    "success_radius_m",
                    "dwell_sec",
                    "wait_sec",
                ],
                "additionalProperties": False,
            },
        },
    },
    "required": ["task_summary", "planning_notes", "steps"],
    "additionalProperties": False,
}


_TARGET_SYNONYMS = {
    "red cube": ("red cube", "red", "red block", "红", "红色", "红色方块", "红色方块儿"),
    "green cube": ("green cube", "green", "green block", "绿", "绿色", "绿色方块"),
    "blue cube": ("blue cube", "blue", "blue block", "蓝", "蓝色", "蓝色方块"),
    "yellow cube": ("yellow cube", "yellow", "yellow block", "黄", "黄色", "黄色方块"),
}

_ALIAS_TO_CANONICAL = {
    alias.lower(): canonical
    for canonical, aliases in _TARGET_SYNONYMS.items()
    for alias in aliases
}
_TARGET_PATTERN = re.compile(
    "|".join(
        re.escape(alias)
        for alias in sorted(_ALIAS_TO_CANONICAL, key=len, reverse=True)
    ),
    flags=re.IGNORECASE,
)


def normalize_target_prompt(raw_prompt: str) -> str:
    """Map free-form color mentions onto the supported tabletop objects."""
    text = (raw_prompt or "").strip().lower()
    for canonical, aliases in _TARGET_SYNONYMS.items():
        if any(alias in text for alias in aliases):
            return canonical
    if text in SUPPORTED_TARGET_PROMPTS:
        return text
    return ""


def _positive_float(value, default: float) -> float:
    try:
        value = float(value)
    except Exception:
        return float(default)
    if value < 0.0:
        return float(default)
    return value


def sanitize_plan(raw_plan: dict) -> dict:
    """Normalize model output into a predictable execution plan."""
    plan = raw_plan if isinstance(raw_plan, dict) else {}
    steps = []
    for fallback_index, raw_step in enumerate(plan.get("steps", []), start=1):
        if not isinstance(raw_step, dict):
            continue

        action = str(raw_step.get("action", "hover_target")).strip().lower()
        if action not in SUPPORTED_ACTIONS:
            action = "hover_target"

        description = str(raw_step.get("description", "")).strip()
        step_index = raw_step.get("step_index", fallback_index)
        try:
            step_index = int(step_index)
        except Exception:
            step_index = fallback_index

        target_prompt = normalize_target_prompt(str(raw_step.get("target_prompt", "")))
        success_radius_m = _positive_float(raw_step.get("success_radius_m", 0.06), 0.06)
        dwell_sec = _positive_float(raw_step.get("dwell_sec", 1.0), 1.0)
        wait_sec = _positive_float(raw_step.get("wait_sec", 1.0), 1.0)

        if action == "wait":
            target_prompt = ""
            success_radius_m = 0.0
            dwell_sec = 0.0
        elif not target_prompt:
            # Drop malformed hover steps instead of sending the robot to nowhere.
            continue

        steps.append(
            {
                "step_index": step_index,
                "action": action,
                "target_prompt": target_prompt,
                "description": description or f"{action} {target_prompt}".strip(),
                "success_radius_m": success_radius_m,
                "dwell_sec": dwell_sec,
                "wait_sec": wait_sec,
            }
        )

    steps.sort(key=lambda item: item["step_index"])
    return {
        "task_summary": str(plan.get("task_summary", "LLM task")).strip() or "LLM task",
        "planning_notes": str(plan.get("planning_notes", "")).strip(),
        "steps": steps,
    }


def infer_plan_from_instruction(instruction_text: str) -> dict:
    """Build a deterministic hover-only plan from color mentions in the instruction.

    This fallback keeps the demo usable even if the remote planner is unavailable.
    """
    text = (instruction_text or "").strip()
    lowered = text.lower()
    matches = []
    for match in _TARGET_PATTERN.finditer(lowered):
        canonical = _ALIAS_TO_CANONICAL.get(match.group(0).lower())
        if canonical:
            matches.append((match.start(), canonical))

    ordered_targets = [canonical for _pos, canonical in sorted(matches, key=lambda item: item[0])]
    if not ordered_targets:
        return {
            "task_summary": "Fallback task",
            "planning_notes": "No supported tabletop target was found in the instruction.",
            "steps": [],
        }

    steps = []
    for step_index, target_prompt in enumerate(ordered_targets, start=1):
        steps.append(
            {
                "step_index": step_index,
                "action": "hover_target",
                "target_prompt": target_prompt,
                "description": f"hover above {target_prompt}",
                "success_radius_m": 0.06,
                "dwell_sec": 1.0,
                "wait_sec": 0.0,
            }
        )

    return sanitize_plan(
        {
            "task_summary": text if text else "Fallback task",
            "planning_notes": "Generated locally from ordered color mentions.",
            "steps": steps,
        }
    )


def plan_to_json(plan: dict) -> str:
    return json.dumps(plan, ensure_ascii=False)


def plan_from_json(text: str) -> dict:
    return sanitize_plan(json.loads(text))


def scene_registry_from_json(text: str) -> dict:
    """Parse the scene registry JSON published by the object-registry node."""
    try:
        data = json.loads(text)
    except Exception:
        return {"target_frame": "world", "updated_at_sec": 0.0, "objects": []}

    objects = data.get("objects", [])
    if not isinstance(objects, list):
        objects = []

    return {
        "target_frame": str(data.get("target_frame", "world")),
        "updated_at_sec": _positive_float(data.get("updated_at_sec", 0.0), 0.0),
        "objects": [obj for obj in objects if isinstance(obj, dict)],
    }


def scene_registry_summary(scene_registry: dict, max_objects: int = 8) -> str:
    """Render a compact scene summary for planner prompts."""
    objects = [obj for obj in scene_registry.get("objects", []) if isinstance(obj, dict)]
    if not objects:
        return "No scene objects have been observed yet."

    visible_objects = [obj for obj in objects if bool(obj.get("visible", False))]
    hidden_objects = [obj for obj in objects if not bool(obj.get("visible", False))]

    def _format_object(obj: dict) -> str:
        label = str(obj.get("label", "object")).strip() or "object"
        score = _positive_float(obj.get("confidence", 0.0), 0.0)
        world = obj.get("position_world") if isinstance(obj.get("position_world"), dict) else {}
        if {"x", "y", "z"} <= set(world):
            try:
                return (
                    f"{label} at "
                    f"({float(world['x']):.2f}, {float(world['y']):.2f}, {float(world['z']):.2f}) "
                    f"score={score:.2f}"
                )
            except Exception:
                pass
        age_sec = _positive_float(obj.get("last_seen_age_sec", 0.0), 0.0)
        return f"{label} score={score:.2f} age={age_sec:.1f}s"

    parts = []
    if visible_objects:
        visible_text = "; ".join(_format_object(obj) for obj in visible_objects[:max_objects])
        parts.append(f"Visible objects: {visible_text}.")
    else:
        parts.append("Visible objects: none.")

    if hidden_objects:
        hidden_text = ", ".join(
            str(obj.get("label", "object")).strip() or "object"
            for obj in hidden_objects[:max_objects]
        )
        parts.append(f"Previously observed objects: {hidden_text}.")

    return " ".join(parts)
