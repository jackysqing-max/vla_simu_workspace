"""Semantic-geometric verification for dynamic RCM port selection."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Mapping

from rcm_virtual_fixtures.port_grounding_expression import (
    ScoringExpressionError,
    score_candidates,
)


_CANDIDATE_ID_PATTERN = re.compile(r"^H?(?P<id>\d+)$", re.IGNORECASE)


@dataclass(frozen=True)
class VerificationThresholds:
    min_model_margin: float = 0.08
    min_expression_margin: float = 1e-6
    min_temporal_stability: float = 0.0
    require_scoring_program: bool = True


def candidate_id_text(value: Any) -> str:
    text = str(value if value is not None else "").strip()
    match = _CANDIDATE_ID_PATTERN.match(text)
    if not match:
        return text
    return f"H{int(match.group('id'))}"


def candidate_id_number(value: Any) -> int | None:
    text = candidate_id_text(value)
    match = _CANDIDATE_ID_PATTERN.match(text)
    if not match:
        return None
    return int(match.group("id"))


def _finite_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return out if math.isfinite(out) else float(default)


def _boolish(value: Any, default: bool = True) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return bool(default)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "yes", "ok", "feasible", "valid"}:
            return True
        if lowered in {"false", "no", "infeasible", "invalid"}:
            return False
    return bool(value)


def _norm01(value: Any, default: float = 0.0) -> float:
    return max(0.0, min(1.0, _finite_float(value, default)))


def _geometry_confidence(hole: Mapping[str, Any]) -> float:
    explicit = hole.get("geometry_confidence")
    if explicit is not None:
        return _norm01(explicit)
    circularity = _norm01(hole.get("depth_hole_circularity"))
    overlap = _norm01(hole.get("depth_hole_overlap_fraction"))
    plane_rms = _finite_float(hole.get("plane_fit_rms", hole.get("plane_rms")), 0.0)
    plane_quality = max(0.0, min(1.0, 1.0 - plane_rms / 0.012))
    return 0.45 * circularity + 0.35 * overlap + 0.20 * plane_quality


def candidate_features_from_hole(
    hole: Mapping[str, Any],
    *,
    image_size: list[Any] | tuple[Any, ...] | None = None,
) -> dict[str, Any]:
    candidate_id = candidate_id_text(hole.get("candidate_id", hole.get("id")))
    width = _finite_float(image_size[0], 1.0) if image_size and len(image_size) >= 2 else 1.0
    height = _finite_float(image_size[1], 1.0) if image_size and len(image_size) >= 2 else 1.0
    geometry_confidence = _geometry_confidence(hole)
    return {
        "id": candidate_id_number(candidate_id),
        "candidate_id": candidate_id,
        "position_world": hole.get("center_world") or hole.get("position_world"),
        "center_world": hole.get("center_world") or hole.get("position_world"),
        "axis_world": hole.get("axis") or hole.get("axis_world"),
        "surface_normal": hole.get("support_normal") or hole.get("surface_normal"),
        "surface_axis": hole.get("surface_equivalent_axis") or hole.get("surface_axis"),
        "image_center": hole.get("center_px") or hole.get("image_center") or [0.0, 0.0],
        "image_width": _finite_float(hole.get("image_width"), width),
        "image_height": _finite_float(hole.get("image_height"), height),
        "geometry_confidence": geometry_confidence,
        "perception_quality": _norm01(hole.get("perception_quality"), geometry_confidence),
        "geometric_validity": 1.0 if _boolish(hole.get("geometric_validity"), True) else 0.0,
        "temporal_stability": _norm01(hole.get("temporal_stability"), 0.0),
        "visibility": _norm01(hole.get("visibility"), 1.0),
        "aperture_area": _finite_float(hole.get("hole_area_px", hole.get("aperture_area")), 0.0),
        "hole_area": _finite_float(hole.get("hole_area_px", hole.get("hole_area")), 0.0),
        "circularity": _norm01(hole.get("depth_hole_circularity", hole.get("circularity"))),
        "plane_fit_rms": _finite_float(hole.get("plane_fit_rms", hole.get("plane_rms")), 0.0),
        "depth_validity": _norm01(hole.get("depth_validity"), 1.0),
        "rcm_feasibility": _norm01(hole.get("rcm_feasibility"), 1.0),
        "rcm_feasible": 1.0 if _boolish(hole.get("rcm_feasible"), True) else 0.0,
        "ik_margin": _finite_float(hole.get("ik_margin"), 0.0),
        "collision_feasibility": _norm01(hole.get("collision_feasibility"), 1.0),
        "collision_feasible": 1.0 if _boolish(hole.get("collision_feasible"), True) else 0.0,
        "semantic_risk": _norm01(hole.get("semantic_risk"), 0.0),
        "unknown_space_exposure": _norm01(hole.get("unknown_space_exposure"), 0.0),
        "phantom_leftness": _finite_float(hole.get("phantom_leftness"), 0.0),
        "phantom_upness": _finite_float(hole.get("phantom_upness"), 0.0),
    }


def candidates_from_payload(payload: Mapping[str, Any]) -> list[dict[str, Any]]:
    holes = payload.get("holes")
    if not isinstance(holes, list):
        holes = payload.get("candidates")
    if not isinstance(holes, list):
        holes = []
    image_size = payload.get("image_size") if isinstance(payload, Mapping) else None
    out = []
    for hole in holes:
        if not isinstance(hole, Mapping):
            continue
        features = candidate_features_from_hole(hole, image_size=image_size)
        features["_raw"] = dict(hole)
        out.append(features)
    return out


def _ranking_from_hypothesis(hypothesis: Mapping[str, Any]) -> list[dict[str, Any]]:
    ranking = hypothesis.get("candidate_ranking")
    if not isinstance(ranking, list):
        ranking = hypothesis.get("ranked_candidates")
    if not isinstance(ranking, list):
        ranking = []
    out = []
    for item in ranking:
        if not isinstance(item, Mapping):
            continue
        candidate_id = candidate_id_text(
            item.get("candidate_id", item.get("id", item.get("candidate")))
        )
        if not candidate_id:
            continue
        score = _finite_float(
            item.get("semantic_score", item.get("score", item.get("utility"))),
            0.0,
        )
        out.append(
            {
                "candidate_id": candidate_id,
                "semantic_score": score,
                "reason": str(item.get("reason", "")),
            }
        )
    out.sort(key=lambda item: float(item["semantic_score"]), reverse=True)
    return out


def _decision(
    decision: str,
    reason: str,
    *,
    selected_candidate_id: str | None = None,
    candidate_scores: list[dict[str, Any]] | None = None,
    hypothesis: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    selected_id = candidate_id_number(selected_candidate_id)
    return {
        "decision": decision,
        "reason": reason,
        "selected_candidate_id": selected_candidate_id,
        "selected_id": selected_id,
        "candidate_scores": candidate_scores or [],
        "objective_text": "" if hypothesis is None else str(hypothesis.get("objective_text", "")),
        "allow_alternative": False if hypothesis is None else bool(hypothesis.get("allow_alternative", False)),
        "verification_source": "semantic_geometric_verifier",
    }


def _candidate_is_feasible(candidate: Mapping[str, Any]) -> bool:
    if _finite_float(candidate.get("geometric_validity"), 1.0) <= 0.0:
        return False
    if _finite_float(candidate.get("rcm_feasible"), 1.0) <= 0.0:
        return False
    if _finite_float(candidate.get("collision_feasible"), 1.0) <= 0.0:
        return False
    return True


def verify_semantic_grounding(
    candidates_payload: Mapping[str, Any],
    hypothesis: Mapping[str, Any],
    *,
    scene: Mapping[str, Any] | None = None,
    previous_selection: Mapping[str, Any] | None = None,
    thresholds: VerificationThresholds | None = None,
) -> dict[str, Any]:
    thresholds = thresholds or VerificationThresholds()
    candidates = candidates_from_payload(candidates_payload)
    if not candidates:
        return _decision("REQUERY", "no_candidate_visible", hypothesis=hypothesis)

    by_id = {str(candidate["candidate_id"]): candidate for candidate in candidates}
    ranking = _ranking_from_hypothesis(hypothesis)
    if bool(hypothesis.get("ambiguous", False)):
        return _decision("REQUERY", "semantic_model_marked_ambiguous", hypothesis=hypothesis)

    selected_candidate_id = candidate_id_text(
        hypothesis.get("selected_candidate_id")
        or hypothesis.get("candidate_id")
        or (ranking[0]["candidate_id"] if ranking else "")
    )
    if not selected_candidate_id:
        return _decision("REQUERY", "language_unresolved", hypothesis=hypothesis)
    if selected_candidate_id not in by_id:
        return _decision(
            "REJECT",
            f"model_selected_missing_candidate:{selected_candidate_id}",
            selected_candidate_id=selected_candidate_id,
            hypothesis=hypothesis,
        )

    excluded = {
        candidate_id_text(value)
        for value in hypothesis.get("excluded_candidate_ids", [])
        if candidate_id_text(value)
    }
    if selected_candidate_id in excluded:
        return _decision(
            "REJECT",
            f"instruction_exclusion_violated:{selected_candidate_id}",
            selected_candidate_id=selected_candidate_id,
            hypothesis=hypothesis,
        )

    expression = str(hypothesis.get("scoring_program", "") or "").strip()
    expression_scores = []
    if expression:
        try:
            expression_scores = score_candidates(
                expression,
                candidates,
                scene=scene or {},
                previous=previous_selection,
            )
        except ScoringExpressionError as exc:
            return _decision(
                "REJECT",
                f"invalid_scoring_program:{exc}",
                selected_candidate_id=selected_candidate_id,
                hypothesis=hypothesis,
            )
        if expression_scores:
            top_expression_id = expression_scores[0]["candidate_id"]
            if top_expression_id != selected_candidate_id:
                return _decision(
                    "REQUERY",
                    "semantic_geometric_disagreement:"
                    f"selected={selected_candidate_id},expression_top={top_expression_id}",
                    selected_candidate_id=selected_candidate_id,
                    candidate_scores=expression_scores,
                    hypothesis=hypothesis,
                )
            if len(expression_scores) > 1:
                margin = float(expression_scores[0]["score"]) - float(expression_scores[1]["score"])
                if margin < thresholds.min_expression_margin:
                    return _decision(
                        "REQUERY",
                        f"expression_margin_too_small:{margin:.6f}",
                        selected_candidate_id=selected_candidate_id,
                        candidate_scores=expression_scores,
                        hypothesis=hypothesis,
                    )
    elif thresholds.require_scoring_program:
        return _decision(
            "REQUERY",
            "missing_scoring_program",
            selected_candidate_id=selected_candidate_id,
            hypothesis=hypothesis,
        )

    if ranking:
        known_ranking = [item for item in ranking if item["candidate_id"] in by_id]
        if len(known_ranking) != len(ranking):
            missing = [
                item["candidate_id"]
                for item in ranking
                if item["candidate_id"] not in by_id
            ]
            return _decision(
                "REJECT",
                "ranking_mentions_missing_candidate:" + ",".join(missing),
                selected_candidate_id=selected_candidate_id,
                candidate_scores=expression_scores,
                hypothesis=hypothesis,
            )
        if known_ranking and known_ranking[0]["candidate_id"] != selected_candidate_id:
            return _decision(
                "REQUERY",
                "selected_candidate_not_top_ranked",
                selected_candidate_id=selected_candidate_id,
                candidate_scores=expression_scores,
                hypothesis=hypothesis,
            )
        if len(known_ranking) > 1:
            margin = (
                float(known_ranking[0]["semantic_score"])
                - float(known_ranking[1]["semantic_score"])
            )
            if margin < thresholds.min_model_margin:
                return _decision(
                    "REQUERY",
                    f"model_margin_too_small:{margin:.3f}",
                    selected_candidate_id=selected_candidate_id,
                    candidate_scores=expression_scores,
                    hypothesis=hypothesis,
                )

    selected_candidate = by_id[selected_candidate_id]
    if _finite_float(selected_candidate.get("temporal_stability"), 0.0) < thresholds.min_temporal_stability:
        return _decision(
            "REQUERY",
            "selected_candidate_not_temporally_stable",
            selected_candidate_id=selected_candidate_id,
            candidate_scores=expression_scores,
            hypothesis=hypothesis,
        )
    if not _candidate_is_feasible(selected_candidate):
        return _decision(
            "REJECT",
            f"selected_candidate_infeasible:{selected_candidate_id}",
            selected_candidate_id=selected_candidate_id,
            candidate_scores=expression_scores,
            hypothesis=hypothesis,
        )

    if ranking:
        top_semantic_id = ranking[0]["candidate_id"]
        if top_semantic_id in by_id and top_semantic_id != selected_candidate_id:
            top_candidate = by_id[top_semantic_id]
            if not _candidate_is_feasible(top_candidate) and not bool(
                hypothesis.get("allow_alternative", False)
            ):
                return _decision(
                    "REJECT",
                    f"top_semantic_candidate_infeasible_without_alternative:{top_semantic_id}",
                    selected_candidate_id=selected_candidate_id,
                    candidate_scores=expression_scores,
                    hypothesis=hypothesis,
                )

    return _decision(
        "ACCEPT",
        "semantic_geometric_verified",
        selected_candidate_id=selected_candidate_id,
        candidate_scores=expression_scores or ranking,
        hypothesis=hypothesis,
    )
