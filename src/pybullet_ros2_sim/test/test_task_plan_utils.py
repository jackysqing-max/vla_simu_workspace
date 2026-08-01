"""Tests for deterministic task-plan safety normalization."""

from pybullet_ros2_sim.task_plan_utils import (
    SURGICAL_RCM_TERMINAL_OPERATIONS,
    infer_plan_from_instruction,
    sanitize_rcm_task_request,
)


def test_rcm_fallback_uses_high_level_task_request():
    task = infer_plan_from_instruction(
        "locate the port and execute a circle",
        allow_rcm_actions=True,
    )

    assert task["instruction"] == "locate the port and execute a circle"
    assert task["objective_text"] == task["instruction"]
    assert task["terminal_operation"] == "execute_rcm_circle"
    assert task["allow_alternative"] is False
    assert "steps" not in task


def test_rcm_task_sanitizer_rejects_motion_plan_shape():
    task = sanitize_rcm_task_request(
        {
            "instruction": "use the safest port",
            "objective_text": "maximize clearance",
            "terminal_operation": "execute_rcm_circle",
            "selected_candidate_id": "H4",
            "joint_positions": [1, 2, 3],
            "steps": [{"action": "execute_rcm_circle"}],
        }
    )

    assert task["instruction"] == "use the safest port"
    assert task["selected_candidate_id"] == "H4"
    assert task["terminal_operation"] in SURGICAL_RCM_TERMINAL_OPERATIONS
    assert "steps" not in task
    assert "joint_positions" not in task


def test_empty_rcm_instruction_does_not_start_motion():
    task = infer_plan_from_instruction("", allow_rcm_actions=True)

    assert task["terminal_operation"] == "stop"
    assert task["verification"]["decision"] == "REJECT"
