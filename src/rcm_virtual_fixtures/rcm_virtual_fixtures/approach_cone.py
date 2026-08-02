"""Geometry helpers for cone-constrained RCM approach selection."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def unit_vector(values: Iterable[float], fallback=(0.0, 0.0, 1.0)) -> np.ndarray:
    vector = np.asarray(list(values), dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return np.asarray(fallback, dtype=np.float64)
    return vector / norm


def cone_basis(center_axis: Iterable[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return an orthonormal frame with ``z`` on the cone center axis."""
    z_axis = unit_vector(center_axis)
    reference = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(z_axis, reference))) > 0.9:
        reference = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    x_axis = unit_vector(np.cross(reference, z_axis), fallback=(1.0, 0.0, 0.0))
    y_axis = unit_vector(np.cross(z_axis, x_axis), fallback=(0.0, 1.0, 0.0))
    return x_axis, y_axis, z_axis


def axis_at_cone_angles(
    center_axis: Iterable[float],
    tilt_deg: float,
    azimuth_deg: float,
) -> np.ndarray:
    """Construct an inward tool axis at polar/azimuth angles in the cone frame."""
    x_axis, y_axis, z_axis = cone_basis(center_axis)
    tilt = math.radians(float(tilt_deg))
    azimuth = math.radians(float(azimuth_deg))
    return unit_vector(
        math.cos(tilt) * z_axis
        + math.sin(tilt)
        * (math.cos(azimuth) * x_axis + math.sin(azimuth) * y_axis),
        fallback=z_axis,
    )


def sample_cone_axes(
    center_axis: Iterable[float],
    half_angle_deg: float,
    radial_samples: int,
    azimuth_samples: int,
) -> list[tuple[np.ndarray, float, float]]:
    """Sample the spherical cap, including its center and boundary."""
    half_angle = max(float(half_angle_deg), 0.0)
    radial_count = max(int(radial_samples), 1)
    azimuth_count = max(int(azimuth_samples), 4)
    samples = [(unit_vector(center_axis), 0.0, 0.0)]
    for radial_index in range(1, radial_count + 1):
        tilt = half_angle * radial_index / radial_count
        for azimuth_index in range(azimuth_count):
            azimuth = 360.0 * azimuth_index / azimuth_count
            samples.append(
                (axis_at_cone_angles(center_axis, tilt, azimuth), tilt, azimuth)
            )
    return samples


def angular_distance_deg(first: Iterable[float], second: Iterable[float]) -> float:
    dot = float(np.clip(np.dot(unit_vector(first), unit_vector(second)), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def wrapped_joint_distance(q: Iterable[float], reference: Iterable[float]) -> float:
    """RMS shortest angular displacement between two joint configurations."""
    delta = np.asarray(list(q), dtype=np.float64) - np.asarray(
        list(reference), dtype=np.float64
    )
    delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
    return float(np.sqrt(np.mean(delta * delta)))
