"""Restricted scoring expressions for semantic RCM port grounding."""

from __future__ import annotations

import ast
import math
from dataclasses import dataclass
from typing import Any, Callable, Mapping


class ScoringExpressionError(ValueError):
    """Raised when a model-generated scoring expression is not safe or valid."""


def _as_float(value: Any, *, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    if not math.isfinite(out):
        return float(default)
    return out


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return bool(_as_float(value))


def _clamp01(value: Any) -> float:
    value = _as_float(value)
    return max(0.0, min(1.0, value))


def _as_vector(value: Any) -> tuple[float, float, float] | None:
    if isinstance(value, FeatureNamespace):
        value = value.data
    if isinstance(value, Mapping):
        for keys in (("x", "y", "z"), ("0", "1", "2")):
            if all(key in value for key in keys):
                return tuple(_as_float(value[key]) for key in keys)
        for key in ("position_world", "center_world", "point", "vector"):
            if key in value:
                return _as_vector(value[key])
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        return tuple(_as_float(value[index]) for index in range(3))
    return None


def _distance(a: Any, b: Any) -> float:
    va = _as_vector(a)
    vb = _as_vector(b)
    if va is None or vb is None:
        return float("inf")
    return math.sqrt(sum((va[index] - vb[index]) ** 2 for index in range(3)))


def _dot_normalized(a: Any, b: Any) -> float:
    va = _as_vector(a)
    vb = _as_vector(b)
    if va is None or vb is None:
        return 0.0
    na = math.sqrt(sum(value * value for value in va))
    nb = math.sqrt(sum(value * value for value in vb))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return sum(va[index] * vb[index] for index in range(3)) / (na * nb)


@dataclass(frozen=True)
class FeatureNamespace:
    """Read-only namespace exposed to the restricted expression interpreter."""

    data: Mapping[str, Any]
    allowed: frozenset[str] | None = None
    name: str = "namespace"

    def get(self, field: str) -> Any:
        if field.startswith("_"):
            raise ScoringExpressionError(f"private attribute is blocked: {field}")
        if self.allowed is not None and field not in self.allowed:
            raise ScoringExpressionError(
                f"{self.name}.{field} is not an approved feature"
            )
        if field not in self.data:
            raise ScoringExpressionError(f"unknown feature: {self.name}.{field}")
        value = self.data[field]
        if isinstance(value, Mapping):
            return FeatureNamespace(value, None, f"{self.name}.{field}")
        return value


@dataclass
class ScoringContext:
    """Runtime values available while scoring one candidate."""

    candidate: Mapping[str, Any]
    scene: Mapping[str, Any]
    previous: Mapping[str, Any] | None = None

    def name_value(self, name: str) -> Any:
        if name == "candidate":
            return FeatureNamespace(
                self.candidate,
                APPROVED_CANDIDATE_FEATURES,
                "candidate",
            )
        if name == "scene":
            return FeatureNamespace(self.scene, None, "scene")
        if name == "previous":
            return FeatureNamespace(self.previous or {}, None, "previous")
        if name in self.scene:
            value = self.scene[name]
            if isinstance(value, Mapping):
                return FeatureNamespace(value, None, name)
            return value
        raise ScoringExpressionError(f"unknown name: {name}")


APPROVED_CANDIDATE_FEATURES = frozenset(
    {
        "id",
        "candidate_id",
        "position_world",
        "center_world",
        "axis_world",
        "surface_normal",
        "surface_axis",
        "image_center",
        "geometry_confidence",
        "perception_quality",
        "geometric_validity",
        "temporal_stability",
        "visibility",
        "aperture_area",
        "hole_area",
        "circularity",
        "plane_fit_rms",
        "depth_validity",
        "rcm_feasibility",
        "rcm_feasible",
        "ik_margin",
        "collision_feasibility",
        "collision_feasible",
        "semantic_risk",
        "unknown_space_exposure",
        "image_width",
        "image_height",
        "phantom_leftness",
        "phantom_upness",
    }
)


def _candidate_position(context: ScoringContext) -> Any:
    return (
        context.candidate.get("position_world")
        or context.candidate.get("center_world")
        or context.candidate.get("point")
    )


def distance_to(context: ScoringContext, entity: Any = None) -> float:
    if entity is None:
        entity = context.scene.get("target") or context.scene.get("target_position_world")
    return _distance(_candidate_position(context), entity)


def clearance_to(context: ScoringContext, entity: Any = None) -> float:
    return distance_to(context, entity)


def alignment_with(context: ScoringContext, vector_or_entity: Any = None) -> float:
    if vector_or_entity is None:
        vector_or_entity = context.scene.get("approach_axis")
    axis = context.candidate.get("axis_world") or context.candidate.get("surface_axis")
    return _dot_normalized(axis, vector_or_entity)


def image_leftness(context: ScoringContext) -> float:
    center = context.candidate.get("image_center") or (0.0, 0.0)
    width = _as_float(context.candidate.get("image_width") or context.scene.get("image_width"), default=1.0)
    if width <= 1e-9:
        return 0.0
    return _clamp01((width - _as_float(center[0])) / width)


def image_rightness(context: ScoringContext) -> float:
    center = context.candidate.get("image_center") or (0.0, 0.0)
    width = _as_float(context.candidate.get("image_width") or context.scene.get("image_width"), default=1.0)
    if width <= 1e-9:
        return 0.0
    return _clamp01(_as_float(center[0]) / width)


def image_upness(context: ScoringContext) -> float:
    center = context.candidate.get("image_center") or (0.0, 0.0)
    height = _as_float(context.candidate.get("image_height") or context.scene.get("image_height"), default=1.0)
    if height <= 1e-9:
        return 0.0
    return _clamp01((height - _as_float(center[1])) / height)


def image_downness(context: ScoringContext) -> float:
    center = context.candidate.get("image_center") or (0.0, 0.0)
    height = _as_float(context.candidate.get("image_height") or context.scene.get("image_height"), default=1.0)
    if height <= 1e-9:
        return 0.0
    return _clamp01(_as_float(center[1]) / height)


def phantom_leftness(context: ScoringContext) -> float:
    return _as_float(context.candidate.get("phantom_leftness"), default=0.0)


def phantom_upness(context: ScoringContext) -> float:
    return _as_float(context.candidate.get("phantom_upness"), default=0.0)


def between(context: ScoringContext, entity_a: Any, entity_b: Any) -> float:
    point = _as_vector(_candidate_position(context))
    a = _as_vector(entity_a)
    b = _as_vector(entity_b)
    if point is None or a is None or b is None:
        return 0.0
    ab = tuple(b[index] - a[index] for index in range(3))
    ap = tuple(point[index] - a[index] for index in range(3))
    ab_len2 = sum(value * value for value in ab)
    if ab_len2 < 1e-12:
        return 1.0 / (1.0 + _distance(point, a))
    t = max(0.0, min(1.0, sum(ap[index] * ab[index] for index in range(3)) / ab_len2))
    closest = tuple(a[index] + t * ab[index] for index in range(3))
    return 1.0 / (1.0 + _distance(point, closest))


def visibility(context: ScoringContext) -> float:
    return _as_float(context.candidate.get("visibility"), default=1.0)


def previous_selection_similarity(context: ScoringContext) -> float:
    if not context.previous:
        return 0.0
    distance = _distance(_candidate_position(context), context.previous)
    if not math.isfinite(distance):
        return 0.0
    return 1.0 / (1.0 + distance)


def semantic_risk(context: ScoringContext) -> float:
    return _as_float(context.candidate.get("semantic_risk"), default=0.0)


def unknown_space_exposure(context: ScoringContext) -> float:
    return _as_float(context.candidate.get("unknown_space_exposure"), default=0.0)


REGISTERED_FUNCTIONS: dict[str, Callable[..., float]] = {
    "distance_to": distance_to,
    "clearance_to": clearance_to,
    "alignment_with": alignment_with,
    "image_leftness": image_leftness,
    "image_rightness": image_rightness,
    "image_upness": image_upness,
    "image_downness": image_downness,
    "phantom_leftness": phantom_leftness,
    "phantom_upness": phantom_upness,
    "between": between,
    "visibility": visibility,
    "previous_selection_similarity": previous_selection_similarity,
    "semantic_risk": semantic_risk,
    "unknown_space_exposure": unknown_space_exposure,
}


class RestrictedScoringExpression:
    """Interpret a tiny, read-only arithmetic expression for each candidate."""

    def __init__(self, source: str):
        self.source = normalize_scoring_program(source)
        try:
            self.tree = ast.parse(self.source, mode="eval")
        except SyntaxError as exc:
            raise ScoringExpressionError(f"invalid scoring expression: {exc}") from exc
        self._validate(self.tree)

    def score(
        self,
        candidate: Mapping[str, Any],
        *,
        scene: Mapping[str, Any] | None = None,
        previous: Mapping[str, Any] | None = None,
    ) -> float:
        context = ScoringContext(candidate, scene or {}, previous)
        value = self._eval(self.tree.body, context)
        return _as_float(value)

    def _validate(self, node: ast.AST):
        if isinstance(
            node,
            (
                ast.Expression,
                ast.BinOp,
                ast.UnaryOp,
                ast.BoolOp,
                ast.Compare,
                ast.Call,
                ast.Name,
                ast.Load,
                ast.Attribute,
                ast.Constant,
                ast.IfExp,
            ),
        ):
            pass
        elif isinstance(
            node,
            (
                ast.Add,
                ast.Sub,
                ast.Mult,
                ast.Div,
                ast.Pow,
                ast.Mod,
                ast.USub,
                ast.UAdd,
                ast.And,
                ast.Or,
                ast.Not,
                ast.Eq,
                ast.NotEq,
                ast.Lt,
                ast.LtE,
                ast.Gt,
                ast.GtE,
            ),
        ):
            return
        else:
            raise ScoringExpressionError(
                f"disallowed expression element: {type(node).__name__}"
            )

        if isinstance(node, ast.Constant):
            if isinstance(node.value, (int, float, bool)):
                return
            raise ScoringExpressionError("only numeric and boolean constants are allowed")
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name):
                raise ScoringExpressionError("only registered function calls are allowed")
            if node.func.id not in REGISTERED_FUNCTIONS:
                raise ScoringExpressionError(f"unregistered function: {node.func.id}")
            if node.keywords:
                raise ScoringExpressionError("keyword arguments are not allowed")
        if isinstance(node, ast.Attribute) and node.attr.startswith("_"):
            raise ScoringExpressionError("private attributes are blocked")

        for child in ast.iter_child_nodes(node):
            self._validate(child)

    def _eval(self, node: ast.AST, context: ScoringContext) -> Any:
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Name):
            return context.name_value(node.id)
        if isinstance(node, ast.Attribute):
            parent = self._eval(node.value, context)
            if not isinstance(parent, FeatureNamespace):
                raise ScoringExpressionError("attribute access is only allowed on feature namespaces")
            return parent.get(node.attr)
        if isinstance(node, ast.UnaryOp):
            value = self._eval(node.operand, context)
            if isinstance(node.op, ast.USub):
                return -_as_float(value)
            if isinstance(node.op, ast.UAdd):
                return _as_float(value)
            if isinstance(node.op, ast.Not):
                return not _as_bool(value)
        if isinstance(node, ast.BinOp):
            left = _as_float(self._eval(node.left, context))
            right = _as_float(self._eval(node.right, context))
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                if abs(right) < 1e-12:
                    raise ScoringExpressionError("division by zero")
                return left / right
            if isinstance(node.op, ast.Mod):
                if abs(right) < 1e-12:
                    raise ScoringExpressionError("modulo by zero")
                return left % right
            if isinstance(node.op, ast.Pow):
                if abs(right) > 8.0:
                    raise ScoringExpressionError("power exponent too large")
                return left ** right
        if isinstance(node, ast.BoolOp):
            values = [_as_bool(self._eval(value, context)) for value in node.values]
            return all(values) if isinstance(node.op, ast.And) else any(values)
        if isinstance(node, ast.Compare):
            left = self._eval(node.left, context)
            for op, comparator in zip(node.ops, node.comparators):
                right = self._eval(comparator, context)
                if not self._compare(left, op, right):
                    return False
                left = right
            return True
        if isinstance(node, ast.IfExp):
            return self._eval(node.body if _as_bool(self._eval(node.test, context)) else node.orelse, context)
        if isinstance(node, ast.Call):
            func = REGISTERED_FUNCTIONS[node.func.id]
            args = [self._eval(arg, context) for arg in node.args]
            return func(context, *args)
        raise ScoringExpressionError(f"unsupported expression node: {type(node).__name__}")

    @staticmethod
    def _compare(left: Any, op: ast.cmpop, right: Any) -> bool:
        left_f = _as_float(left)
        right_f = _as_float(right)
        if isinstance(op, ast.Eq):
            return left_f == right_f
        if isinstance(op, ast.NotEq):
            return left_f != right_f
        if isinstance(op, ast.Lt):
            return left_f < right_f
        if isinstance(op, ast.LtE):
            return left_f <= right_f
        if isinstance(op, ast.Gt):
            return left_f > right_f
        if isinstance(op, ast.GtE):
            return left_f >= right_f
        raise ScoringExpressionError(f"unsupported comparison: {type(op).__name__}")


def normalize_scoring_program(source: str) -> str:
    text = (source or "").strip()
    if not text:
        raise ScoringExpressionError("empty scoring expression")
    if text.startswith("return "):
        text = text[len("return "):].strip()
    if "\n" in text or ";" in text:
        raise ScoringExpressionError("scoring expression must be a single expression")
    return text


def score_candidates(
    expression: str,
    candidates: list[Mapping[str, Any]],
    *,
    scene: Mapping[str, Any] | None = None,
    previous: Mapping[str, Any] | None = None,
) -> list[dict[str, Any]]:
    compiled = RestrictedScoringExpression(expression)
    scored = []
    for candidate in candidates:
        scored.append(
            {
                "candidate_id": str(candidate.get("candidate_id") or candidate.get("id")),
                "score": compiled.score(candidate, scene=scene, previous=previous),
            }
        )
    scored.sort(key=lambda item: float(item["score"]), reverse=True)
    return scored
