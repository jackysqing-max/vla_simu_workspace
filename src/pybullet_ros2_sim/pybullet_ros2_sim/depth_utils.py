"""Depth-buffer conversion helpers."""

import numpy as np


def depth_buffer_to_meters(depth_buf: np.ndarray, near: float, far: float) -> np.ndarray:
    """Convert PyBullet's non-linear depth buffer into metric depth values."""
    depth_m = (far * near) / (far - (far - near) * depth_buf)
    return depth_m.astype(np.float32)
