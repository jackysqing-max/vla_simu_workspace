"""ROS 2 node that applies touchless commands to a PyVista MRI viewer."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from touchless_project.dependencies import import_dependency
from touchless_project.gesture_core import make_config


class TouchlessMriViewerNode(Node):
    """Render an MRI volume and react to touchless gesture commands."""

    def __init__(self):
        super().__init__("touchless_mri_viewer_node")

        self.declare_parameter("command_topic", "/touchless/command_json")
        self.declare_parameter("nifti_path", "")
        self.declare_parameter("window_title", "MRI 3D - Gesture Controlled")
        self.declare_parameter("sensitivity", 1.0)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.window_title = str(self.get_parameter("window_title").value)
        self.sensitivity = float(self.get_parameter("sensitivity").value)

        self.config = make_config(self.sensitivity)
        self.numpy = import_dependency("numpy", "numpy")
        self.nibabel = import_dependency("nibabel", "nibabel")
        self.pyvista = import_dependency("pyvista", "pyvista")
        self.pyvistaqt = import_dependency("pyvistaqt", "pyvistaqt")

        configured_path = str(self.get_parameter("nifti_path").value).strip()
        self.nifti_path = self._resolve_nifti_path(configured_path)

        self.plotter = self.pyvistaqt.BackgroundPlotter(title=self.window_title)
        self._load_volume()

        self.create_subscription(String, self.command_topic, self.on_command, 10)
        self.get_logger().info(
            "touchless_mri_viewer_node started "
            f"(nifti_path={self.nifti_path}, command_topic={self.command_topic})"
        )

    def _resolve_nifti_path(self, configured_path: str) -> Path:
        raw_path = configured_path or os.environ.get(
            "TOUCHLESS_PROJECT_NIFTI_PATH",
            "",
        ).strip()
        if not raw_path:
            raise FileNotFoundError(
                "No MRI volume configured. Set the 'nifti_path' parameter or "
                "TOUCHLESS_PROJECT_NIFTI_PATH."
            )

        candidate = Path(raw_path).expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"NIfTI file not found: {candidate}")
        return candidate

    def _load_volume(self) -> None:
        image = self.nibabel.load(str(self.nifti_path))
        data = image.get_fdata().astype(self.numpy.float32)
        if data.ndim == 4:
            data = data[..., 0]

        volume = self.pyvista.wrap(data)
        self.plotter.add_volume(volume, cmap="gray", opacity="sigmoid", shade=False)
        self.plotter.add_axes()
        self.plotter.show_grid()

    def on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Received malformed gesture payload")
            return

        action = str(payload.get("action", "")).upper()
        if not action:
            return

        if action == "PAN":
            self._pan_camera(
                float(payload.get("dx", 0.0)),
                float(payload.get("dy", 0.0)),
            )
        elif action == "ROTATE":
            self._rotate_camera(
                float(payload.get("dx", 0.0)),
                float(payload.get("dy", 0.0)),
            )
        elif action == "ZOOM":
            self._zoom_camera(float(payload.get("delta_pinch", 0.0)))
        else:
            self.get_logger().warning(f"Unsupported gesture action: {action}")
            return

        self.plotter.render()

    def _pan_camera(self, dx_pix: float, dy_pix: float) -> None:
        camera = self.plotter.camera

        position = self.numpy.array(camera.position, dtype=float)
        focal_point = self.numpy.array(camera.focal_point, dtype=float)
        view_up = self.numpy.array(camera.up, dtype=float)

        view_dir = focal_point - position
        view_dir /= self.numpy.linalg.norm(view_dir) + 1e-9

        right = self.numpy.cross(view_dir, view_up)
        right /= self.numpy.linalg.norm(right) + 1e-9

        up = view_up / (self.numpy.linalg.norm(view_up) + 1e-9)
        offset = right * (dx_pix * self.config.pan_scale_world_per_px) + up * (
            -dy_pix * self.config.pan_scale_world_per_px
        )

        camera.position = tuple(position + offset)
        camera.focal_point = tuple(focal_point + offset)

    def _rotate_camera(self, dx_pix: float, dy_pix: float) -> None:
        camera = self.plotter.camera
        camera.Azimuth(float(dx_pix) * self.config.rot_deg_per_px)
        camera.Elevation(float(-dy_pix) * self.config.rot_deg_per_px)
        camera.OrthogonalizeViewUp()

    def _zoom_camera(self, delta_pinch: float) -> None:
        camera = self.plotter.camera
        factor = math.exp(float(delta_pinch) * self.config.zoom_gain_per_px)
        camera.Zoom(factor)

    def destroy_node(self):
        if hasattr(self, "plotter") and self.plotter is not None:
            self.plotter.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TouchlessMriViewerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
