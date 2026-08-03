"""Tests for restricted semantic port scoring expressions."""

import pytest

from rcm_virtual_fixtures.port_grounding_expression import (
    ScoringExpressionError,
    score_candidates,
)


def candidate(candidate_id, image_center, position, **extra):
    out = {
        "candidate_id": candidate_id,
        "image_center": image_center,
        "image_width": 640,
        "image_height": 480,
        "position_world": position,
        "axis_world": [0.0, 0.0, -1.0],
        "geometry_confidence": 0.8,
        "temporal_stability": 1.0,
        "visibility": 1.0,
        "semantic_risk": 0.0,
        "unknown_space_exposure": 0.0,
        "ik_margin": 0.0,
    }
    out.update(extra)
    return out


def top_id(expression, candidates, scene=None, previous=None):
    return score_candidates(
        expression,
        candidates,
        scene=scene or {},
        previous=previous,
    )[0]["candidate_id"]


def test_image_upper_left_objective_uses_dynamic_candidates():
    candidates = [
        candidate("H1", [580, 70], [0.0, 0.0, 0.0]),
        candidate("H2", [320, 240], [0.1, 0.0, 0.0]),
        candidate("H10", [70, 60], [0.2, 0.0, 0.0]),
    ]

    assert top_id("image_leftness() + image_upness()", candidates) == "H10"


def test_distance_clearance_between_previous_and_robot_features():
    candidates = [
        candidate("H1", [100, 100], [-0.2, 0.0, 0.0], ik_margin=0.1),
        candidate("H2", [200, 100], [0.4, 0.0, 0.0], ik_margin=0.9),
        candidate("H3", [300, 100], [0.8, 0.0, 0.0], semantic_risk=0.8),
    ]
    scene = {
        "target": [0.35, 0.0, 0.0],
        "vessel": [0.0, 0.0, 0.0],
        "protected_a": [0.0, 0.0, 0.0],
        "protected_b": [0.8, 0.0, 0.0],
    }

    assert top_id("-distance_to(target)", candidates, scene) == "H2"
    assert top_id("clearance_to(vessel)", candidates, scene) == "H3"
    assert top_id("between(protected_a, protected_b)", candidates, scene) == "H2"
    assert top_id("candidate.ik_margin", candidates, scene) == "H2"
    assert top_id("-semantic_risk()", candidates, scene) == "H1"
    assert (
        top_id(
            "-distance_to(target) + 10 * (clearance_to(vessel) >= 0.2)",
            candidates,
            scene,
        )
        == "H2"
    )
    assert (
        top_id(
            "previous_selection_similarity()",
            candidates,
            scene,
            previous={"position_world": [0.42, 0.0, 0.0]},
        )
        == "H2"
    )


@pytest.mark.parametrize(
    "expression",
    [
        "__import__('os').system('echo bad')",
        "open('/tmp/x').read()",
        "candidate.__class__",
        "[x for x in [1, 2, 3]]",
        "import os",
        "return image_leftness(); image_rightness()",
        "unknown_feature + 1",
        "candidate.not_approved",
    ],
)
def test_rejects_unsafe_or_unknown_expressions(expression):
    candidates = [candidate("H1", [100, 100], [0.0, 0.0, 0.0])]

    with pytest.raises(ScoringExpressionError):
        score_candidates(expression, candidates)
