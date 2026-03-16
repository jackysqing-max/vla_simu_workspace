"""Small camera wrapper used by the RGB-D simulation node."""

import math
from dataclasses import dataclass

import numpy as np
import pybullet as p


@dataclass
class SimCameraConfig:
    width: int = 640
    height: int = 480
    fov_y_deg: float = 58.0
    near: float = 0.02
    far: float = 3.0
    target_pos: tuple = (0.6, 0.0, 0.05)
    distance: float = 1.0
    yaw_deg: float = 90.0
    pitch_deg: float = -45.0
    roll_deg: float = 0.0
    up_axis_index: int = 2


class SimRGBDCamera:
    """Bundle view/projection/intrinsics logic around a PyBullet camera."""

    def __init__(self, physics_client_id: int, cfg: SimCameraConfig):
        self.client = physics_client_id
        self.cfg = cfg

    def aspect(self) -> float:
        return self.cfg.width / float(self.cfg.height)

    def projection_matrix(self):
        return p.computeProjectionMatrixFOV(
            fov=self.cfg.fov_y_deg,
            aspect=self.aspect(),
            nearVal=self.cfg.near,
            farVal=self.cfg.far,
        )

    def view_matrix(self):
        return p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.cfg.target_pos,
            distance=self.cfg.distance,
            yaw=self.cfg.yaw_deg,
            pitch=self.cfg.pitch_deg,
            roll=self.cfg.roll_deg,
            upAxisIndex=self.cfg.up_axis_index,
        )

    def render(self):
        """Render and return RGB, depth-buffer, and segmentation images."""
        width, height, rgba, depth_buf, seg = p.getCameraImage(
            width=self.cfg.width,
            height=self.cfg.height,
            viewMatrix=self.view_matrix(),
            projectionMatrix=self.projection_matrix(),
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            physicsClientId=self.client,
        )

        rgba = np.reshape(rgba, (height, width, 4))
        rgb = rgba[:, :, :3].astype(np.uint8)
        depth_buf = np.reshape(depth_buf, (height, width)).astype(np.float32)
        seg = np.reshape(seg, (height, width)).astype(np.int32)
        return rgb, depth_buf, seg

    def intrinsics(self):
        """Approximate pinhole intrinsics from FOV and image size."""
        height = self.cfg.height
        width = self.cfg.width
        fov_y = math.radians(self.cfg.fov_y_deg)

        fy = height / (2.0 * math.tan(fov_y / 2.0))
        fx = fy * (width / float(height))
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        return fx, fy, cx, cy
