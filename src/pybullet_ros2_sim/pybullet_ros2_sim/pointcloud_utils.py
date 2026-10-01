"""Depth/RGB to point cloud conversion helpers for the RGB-D sim node."""

import numpy as np


def depth_to_xyz(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float):
    """Project a full depth image into XYZ camera coordinates."""
    height, width = depth_m.shape
    us, vs = np.meshgrid(np.arange(width), np.arange(height))

    z = depth_m
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    return np.stack([x, y, z], axis=-1).astype(np.float32)


def depth_rgb_to_xyzrgb(
    depth_m: np.ndarray,
    rgb: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
):
    """Drop invalid pixels and return packed XYZ and RGB arrays."""
    xyz = depth_to_xyz(depth_m, fx, fy, cx, cy)
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    return xyz[valid], rgb[valid]
