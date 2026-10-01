"""Tests for cone-constrained RCM approach geometry."""

import numpy as np

from rcm_virtual_fixtures.approach_cone import (
    angular_distance_deg,
    axis_at_cone_angles,
    sample_cone_axes,
    wrapped_joint_distance,
)


def test_cone_sampling_stays_inside_half_angle():
    center = np.asarray([0.3, -0.1, -0.95])
    samples = sample_cone_axes(center, 20.0, 4, 16)

    assert len(samples) == 65
    assert max(angular_distance_deg(center, axis) for axis, _tilt, _azimuth in samples) <= 20.0001


def test_explicit_angles_reconstruct_requested_axis():
    center = [0.0, 0.0, -1.0]
    axis = axis_at_cone_angles(center, 12.0, 75.0)

    assert abs(angular_distance_deg(center, axis) - 12.0) < 1e-6
    assert abs(np.linalg.norm(axis) - 1.0) < 1e-9


def test_joint_distance_uses_shortest_wrapped_rotation():
    distance = wrapped_joint_distance([3.13], [-3.13])

    assert distance < 0.03
