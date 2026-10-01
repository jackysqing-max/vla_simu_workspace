"""Map touchless gesture deltas into end-effector pose offsets."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence


@dataclass(frozen=True)
class EeGestureMappingConfig:
    """Scale factors and limits for the robot gesture demo."""

    pan_y_m_per_px: float = 0.0015
    pan_z_m_per_px: float = 0.0015
    zoom_x_m_per_px: float = 0.0015
    rotate_pitch_rad_per_px: float = 0.004
    rotate_yaw_rad_per_px: float = 0.004
    max_position_offset_xyz_m: tuple[float, float, float] = (0.30, 0.30, 0.30)
    max_orientation_offset_rpy_rad: tuple[float, float, float] = (0.90, 0.90, 0.90)
    orientation_reference_frame: str = "world"


@dataclass
class EePoseTarget:
    """Current target pose represented relative to the locked initial pose."""

    position: list[float]
    orientation: list[float]
    position_offset: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    orientation_offset_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class EeGestureMapper:
    """Accumulate gesture commands as relative Cartesian pose targets."""

    def __init__(
        self,
        initial_position: Sequence[float],
        initial_orientation_xyzw: Sequence[float],
        config: EeGestureMappingConfig,
    ):
        self.initial_position = [float(value) for value in initial_position[:3]]
        self.initial_orientation = _normalize_quat(
            [float(value) for value in initial_orientation_xyzw[:4]]
        )
        self.config = config
        self.position_offset = [0.0, 0.0, 0.0]
        self.orientation_offset_rpy = [0.0, 0.0, 0.0]

    def reset_offsets(self) -> EePoseTarget:
        """Return to the locked initial pose."""
        self.position_offset = [0.0, 0.0, 0.0]
        self.orientation_offset_rpy = [0.0, 0.0, 0.0]
        return self.target()

    def apply_command(self, payload: dict) -> EePoseTarget:
        """Apply one gesture JSON payload and return the new target pose."""
        action = str(payload.get("action", "")).upper()
        if action == "PAN":
            self._apply_pan(float(payload.get("dx", 0.0)), float(payload.get("dy", 0.0)))
        elif action == "ZOOM":
            self._apply_zoom(float(payload.get("delta_pinch", 0.0)))
        elif action == "ROTATE":
            self._apply_rotate(float(payload.get("dx", 0.0)), float(payload.get("dy", 0.0)))
        return self.target()

    def target(self) -> EePoseTarget:
        """Return the absolute target pose."""
        position = [
            self.initial_position[index] + self.position_offset[index]
            for index in range(3)
        ]

        delta_quat = _quat_from_euler_xyz(self.orientation_offset_rpy)
        if self.config.orientation_reference_frame.strip().lower() == "tool":
            orientation = _quat_multiply(self.initial_orientation, delta_quat)
        else:
            orientation = _quat_multiply(delta_quat, self.initial_orientation)

        return EePoseTarget(
            position=position,
            orientation=_normalize_quat(orientation),
            position_offset=list(self.position_offset),
            orientation_offset_rpy=list(self.orientation_offset_rpy),
        )

    def _apply_pan(self, dx_pix: float, dy_pix: float) -> None:
        # Screen horizontal motion maps to robot Y. Screen upward motion has
        # negative dy in image coordinates, so invert dy for robot Z.
        self.position_offset[1] += dx_pix * self.config.pan_y_m_per_px
        self.position_offset[2] += -dy_pix * self.config.pan_z_m_per_px
        self._clamp_position_offset()

    def _apply_zoom(self, delta_pinch_pix: float) -> None:
        self.position_offset[0] += delta_pinch_pix * self.config.zoom_x_m_per_px
        self._clamp_position_offset()

    def _apply_rotate(self, dx_pix: float, dy_pix: float) -> None:
        # Horizontal rotation gesture maps to yaw about Z; vertical maps to
        # pitch about Y. Roll is intentionally left unmapped for this demo.
        self.orientation_offset_rpy[1] += -dy_pix * self.config.rotate_pitch_rad_per_px
        self.orientation_offset_rpy[2] += dx_pix * self.config.rotate_yaw_rad_per_px
        self._clamp_orientation_offset()

    def _clamp_position_offset(self) -> None:
        limits = self.config.max_position_offset_xyz_m
        for index in range(3):
            limit = abs(float(limits[index]))
            self.position_offset[index] = _clamp(self.position_offset[index], -limit, limit)

    def _clamp_orientation_offset(self) -> None:
        limits = self.config.max_orientation_offset_rpy_rad
        for index in range(3):
            limit = abs(float(limits[index]))
            self.orientation_offset_rpy[index] = _clamp(
                self.orientation_offset_rpy[index],
                -limit,
                limit,
            )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _quat_from_euler_xyz(rpy: Sequence[float]) -> list[float]:
    roll, pitch, yaw = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def _quat_multiply(lhs: Sequence[float], rhs: Sequence[float]) -> list[float]:
    x1, y1, z1, w1 = (float(lhs[0]), float(lhs[1]), float(lhs[2]), float(lhs[3]))
    x2, y2, z2, w2 = (float(rhs[0]), float(rhs[1]), float(rhs[2]), float(rhs[3]))
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def _normalize_quat(quat: Sequence[float]) -> list[float]:
    norm = math.sqrt(sum(float(value) * float(value) for value in quat[:4]))
    if norm < 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [float(value) / norm for value in quat[:4]]
