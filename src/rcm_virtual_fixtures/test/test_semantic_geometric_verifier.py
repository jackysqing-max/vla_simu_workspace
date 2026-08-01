"""Tests for semantic-geometric RCM port verification."""

from rcm_virtual_fixtures.semantic_geometric_verifier import (
    VerificationThresholds,
    verify_semantic_grounding,
)


def hole(index, u=None, v=None, *, feasible=True, stability=1.0):
    if u is None:
        u = 40.0 + 45.0 * index
    if v is None:
        v = 60.0 + 20.0 * index
    return {
        "id": index,
        "candidate_id": f"H{index}",
        "center_px": [u, v],
        "center_world": [0.05 * index, 0.0, 0.4],
        "axis": [0.0, 0.0, -1.0],
        "support_normal": [0.0, 0.0, 1.0],
        "hole_area_px": 120 + index,
        "depth_hole_circularity": 0.8,
        "depth_hole_overlap_fraction": 0.9,
        "plane_fit_rms": 0.002,
        "temporal_stability": stability,
        "geometric_validity": True,
        "rcm_feasible": feasible,
        "collision_feasible": feasible,
        "semantic_risk": 0.0,
    }


def payload(count):
    return {
        "image_size": [640, 480],
        "holes": [hole(index) for index in range(1, count + 1)],
    }


def hypothesis(selected, score=0.9, second=0.4, expression="candidate.hole_area"):
    return {
        "objective_text": "prefer the leftmost visible port",
        "selected_candidate_id": selected,
        "candidate_ranking": [
            {"candidate_id": selected, "semantic_score": score, "reason": "best"},
            {"candidate_id": "H1", "semantic_score": second, "reason": "next"},
        ],
        "scoring_program": expression,
        "ambiguous": False,
        "allow_alternative": False,
    }


def assert_decision(result, decision):
    assert result["decision"] == decision, result


def test_accepts_dynamic_candidate_sets_beyond_nine_slots():
    data = payload(12)
    data["holes"][9]["center_px"] = [10.0, 80.0]
    result = verify_semantic_grounding(
        data,
        hypothesis("H10", expression="image_leftness() + 0.001 * candidate.hole_area"),
    )

    assert_decision(result, "ACCEPT")
    assert result["selected_candidate_id"] == "H10"


def test_rejects_nonexistent_candidate_id():
    result = verify_semantic_grounding(payload(3), hypothesis("H7"))

    assert_decision(result, "REJECT")
    assert "missing_candidate" in result["reason"]


def test_requeries_ambiguous_or_low_margin_model_ranking():
    ambiguous = hypothesis("H3")
    ambiguous["ambiguous"] = True
    assert_decision(verify_semantic_grounding(payload(3), ambiguous), "REQUERY")

    low_margin = hypothesis("H3", score=0.51, second=0.49)
    assert_decision(verify_semantic_grounding(payload(3), low_margin), "REQUERY")


def test_rejects_invalid_ast_and_unknown_feature():
    bad_ast = hypothesis("H3", expression="__import__('os').system('id')")
    assert_decision(verify_semantic_grounding(payload(3), bad_ast), "REJECT")

    bad_feature = hypothesis("H3", expression="candidate.row + 1")
    assert_decision(verify_semantic_grounding(payload(3), bad_feature), "REJECT")


def test_rejects_infeasible_or_excluded_candidate():
    data = payload(3)
    data["holes"][2] = hole(3, feasible=False)
    assert_decision(verify_semantic_grounding(data, hypothesis("H3")), "REJECT")

    excluded = hypothesis("H3")
    excluded["excluded_candidate_ids"] = ["H3"]
    assert_decision(verify_semantic_grounding(payload(3), excluded), "REJECT")


def test_requeries_unstable_or_empty_candidate_set():
    data = {"image_size": [640, 480], "holes": []}
    assert_decision(verify_semantic_grounding(data, hypothesis("H1")), "REQUERY")

    unstable = payload(3)
    unstable["holes"][2]["temporal_stability"] = 0.1
    result = verify_semantic_grounding(
        unstable,
        hypothesis("H3"),
        thresholds=VerificationThresholds(min_temporal_stability=0.5),
    )
    assert_decision(result, "REQUERY")
