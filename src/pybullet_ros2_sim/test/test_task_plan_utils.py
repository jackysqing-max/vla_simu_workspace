"""Tests for deterministic task-plan safety normalization."""

from pybullet_ros2_sim.task_plan_utils import (
    SURGICAL_RCM_ACTIONS,
    infer_plan_from_instruction,
    sanitize_plan,
)


def test_rcm_fallback_uses_fixed_safety_sequence():
    plan = infer_plan_from_instruction(
        "locate the port and execute a circle",
        allow_rcm_actions=True,
    )

    assert [step["action"] for step in plan["steps"]] == list(
        SURGICAL_RCM_ACTIONS
    )
    assert [step["step_index"] for step in plan["steps"]] == [1, 2, 3, 4]
    assert plan["steps"][2]["insertion_depth_m"] == 0.077
    assert plan["steps"][3]["trajectory_radius_m"] == 0.020
    assert plan["steps"][3]["trajectory_cycles"] == 1.0


def test_rcm_sanitizer_ignores_model_motion_parameters_and_order():
    plan = sanitize_plan(
        {
            "steps": [
                {
                    "step_index": 99,
                    "action": "execute_rcm_circle",
                    "insertion_depth_m": 9.0,
                    "trajectory_radius_m": 9.0,
                    "trajectory_cycles": 9.0,
                }
            ]
        },
        allow_rcm_actions=True,
    )

    assert [step["action"] for step in plan["steps"]] == list(
        SURGICAL_RCM_ACTIONS
    )
    assert plan["steps"][2]["insertion_depth_m"] == 0.077
    assert plan["steps"][3]["trajectory_radius_m"] == 0.020
    assert plan["steps"][3]["trajectory_cycles"] == 1.0


def test_empty_rcm_instruction_does_not_start_motion():
    plan = infer_plan_from_instruction("", allow_rcm_actions=True)

    assert plan["steps"] == []
