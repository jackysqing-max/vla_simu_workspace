"""Tests for text-only semantic port grounding fallback."""

from rcm_virtual_fixtures.semantic_port_grounder_node import (
    build_local_grounding_hypothesis,
    local_scoring_expression_from_instruction,
    sam_prompt_from_instruction,
)


def candidate(candidate_id, center_px, **extra):
    item = {
        "candidate_id": candidate_id,
        "center_px": list(center_px),
        "center_world": [0.0, 0.0, 0.0],
        "axis": [0.0, 0.0, -1.0],
        "support_normal": [0.0, 0.0, 1.0],
        "geometric_validity": True,
        "geometry_confidence": 0.9,
        "rcm_feasible": True,
        "collision_feasible": True,
    }
    item.update(extra)
    return item


def payload(*holes):
    return {
        "schema_version": "dynamic_rcm_port_candidates.v2",
        "image_size": [800, 600],
        "holes": list(holes),
    }


def test_camera_upper_left_instruction_selects_image_upper_left_candidate():
    hypothesis = build_local_grounding_hypothesis(
        "定位相机视角左上角的孔",
        payload(
            candidate("H1", [650, 80]),
            candidate("H2", [400, 300]),
            candidate("H3", [120, 90]),
            candidate("H4", [120, 500]),
        ),
        fallback_reason="unit_test",
    )

    assert hypothesis is not None
    assert hypothesis["selected_candidate_id"] == "H3"
    assert hypothesis["scoring_program"].startswith(
        "image_leftness() + image_upness()"
    )


def test_direct_candidate_id_instruction_is_preserved():
    expression, reference = local_scoring_expression_from_instruction("选择 H7 号孔")

    assert expression == "candidate.id == 7"
    assert reference == "explicit candidate id H7"


def test_center_instruction_uses_centeredness_expression():
    hypothesis = build_local_grounding_hypothesis(
        "定位中间的孔",
        payload(
            candidate("H1", [100, 100]),
            candidate("H5", [402, 298]),
            candidate("H9", [700, 500]),
        ),
        fallback_reason="unit_test",
    )

    assert hypothesis is not None
    assert hypothesis["selected_candidate_id"] == "H5"
    assert "image_centeredness()" in hypothesis["scoring_program"]


def test_language_instruction_builds_targeted_sam_prompt():
    assert (
        sam_prompt_from_instruction("定位相机视角左上角的孔")
        == "upper left circular hole on the phantom"
    )
    assert sam_prompt_from_instruction("定位phantom上的孔") == (
        "circular hole on the phantom"
    )
