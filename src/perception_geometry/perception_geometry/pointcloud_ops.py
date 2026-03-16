"""Geometry helpers used by the perception fusion node."""

import numpy as np


def intrinsics_from_camera_info(msg):
    """Extract `(fx, fy, cx, cy)` from `sensor_msgs/CameraInfo`."""
    return msg.k[0], msg.k[4], msg.k[2], msg.k[5]


def mask_centroid(mask_u8: np.ndarray):
    """Return the centroid `(u, v)` of a binary mask or `None` if empty."""
    ys, xs = np.nonzero(mask_u8 > 0)
    if len(xs) == 0:
        return None
    return float(xs.mean()), float(ys.mean())


def masked_valid_depth_values(mask_u8: np.ndarray, depth_m: np.ndarray):
    """Return valid depth samples inside the mask and the corresponding mask."""
    mask = mask_u8 > 0
    valid = mask & np.isfinite(depth_m) & (depth_m > 0.0)
    return depth_m[valid], valid


def pixel_to_xyz(u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float):
    """Project a single pixel and depth value into camera coordinates."""
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return float(x), float(y), float(z)


def masked_depth_to_xyz(
    depth_m: np.ndarray,
    mask_u8: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
):
    """Project all valid mask pixels into a dense XYZ point set."""
    height, width = depth_m.shape
    us, vs = np.meshgrid(np.arange(width), np.arange(height))

    valid = (mask_u8 > 0) & np.isfinite(depth_m) & (depth_m > 0.0)
    if valid.sum() == 0:
        return np.zeros((0, 3), dtype=np.float32)

    z = depth_m[valid]
    u = us[valid].astype(np.float32)
    v = vs[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=-1).astype(np.float32)
