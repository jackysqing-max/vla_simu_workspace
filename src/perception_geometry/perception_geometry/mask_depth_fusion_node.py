"""Fuse SAM3 mask output with depth to produce a stable 3D keypoint."""

from collections import deque
import math
import threading
import time

import numpy as np
import rclpy
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PointStamped, Vector3Stamped
from tf2_ros import Buffer, TransformListener

from perception_geometry.pointcloud_ops import (
    intrinsics_from_camera_info,
    masked_depth_to_xyz,
    masked_rgbd_to_xyzrgbuv,
    rekep_rgbd_candidates,
    select_primary_candidate,
    xyz_to_pixel,
)
from perception_geometry.ros_msg_utils import (
    make_bool_msg,
    make_float32_msg,
    make_point_stamped,
    xyz_to_pointcloud2,
)


def imgmsg_to_mask_u8(msg: Image) -> np.ndarray:
    height, width = msg.height, msg.width
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width).copy()


def imgmsg_to_depth32f(msg: Image) -> np.ndarray:
    height, width = msg.height, msg.width
    return np.frombuffer(msg.data, dtype=np.float32).reshape(height, width).copy()


def imgmsg_to_depth_meters(msg: Image, depth_scale: float) -> np.ndarray:
    """Convert common ROS depth encodings into meters as float32."""
    height, width = msg.height, msg.width
    encoding = str(msg.encoding).lower()

    if encoding in {"16uc1", "mono16"}:
        depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
        return depth_u16.astype(np.float32) * float(depth_scale)

    if encoding in {"32fc1", "32fc"}:
        return imgmsg_to_depth32f(msg)

    # Some drivers leave the encoding blank but still set the row stride.
    bytes_per_pixel = int(msg.step / max(width, 1)) if width > 0 else 0
    if bytes_per_pixel == 2:
        depth_u16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, width)
        return depth_u16.astype(np.float32) * float(depth_scale)
    if bytes_per_pixel == 4:
        return imgmsg_to_depth32f(msg)

    raise ValueError(f"unsupported depth encoding: {msg.encoding!r}")


def imgmsg_to_rgb8(msg: Image) -> np.ndarray:
    height, width = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
    if msg.encoding.lower() == "bgr8":
        return arr[:, :, ::-1].copy()
    return arr.copy()


def rgb8_to_imgmsg(rgb: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


def transform_to_matrix(tf_msg):
    """Convert a TF transform into a homogeneous 4x4 matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    transform[:3, 3] = [t.x, t.y, t.z]
    return transform


def transform_points(transform: np.ndarray, points_xyz: np.ndarray):
    """Apply a homogeneous transform to an `Nx3` point cloud."""
    if points_xyz.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    homog = np.concatenate(
        [points_xyz.astype(np.float64), np.ones((points_xyz.shape[0], 1), dtype=np.float64)],
        axis=1,
    )
    transformed = (transform @ homog.T).T
    return transformed[:, :3].astype(np.float32)


def parse_float_vector(value, default, length: int = 3) -> np.ndarray:
    if isinstance(value, str):
        text = value.strip().strip("[]")
        parts = [part.strip() for part in text.split(",") if part.strip()]
    else:
        try:
            parts = list(value)
        except TypeError:
            parts = []

    vector = []
    for part in parts[:length]:
        try:
            vector.append(float(part))
        except Exception:
            break
    if len(vector) < length:
        vector.extend(float(default[index]) for index in range(len(vector), length))
    return np.array(vector[:length], dtype=np.float64)


def select_top_surface_point(
    xyz_cam: np.ndarray,
    cam_to_world: np.ndarray,
    *,
    top_band_m: float,
    min_fraction: float,
    surface_percentile: float,
):
    """
    Estimate the center of the visible top surface in camera coordinates.

    We convert the masked cloud into the world frame, take the points near the
    maximum world `z`, and average that slice. This yields a point on the cube's
    top face instead of the object centroid.
    """
    if xyz_cam.shape[0] == 0:
        return None, None, np.zeros((0, 3), dtype=np.float32)

    xyz_world = transform_points(cam_to_world, xyz_cam)
    z_world = xyz_world[:, 2]
    finite_mask = np.isfinite(z_world)
    if not np.any(finite_mask):
        return None, None, np.zeros((0, 3), dtype=np.float32)

    xyz_world = xyz_world[finite_mask]
    z_world = z_world[finite_mask]

    band_m = max(float(top_band_m), 1e-4)
    percentile = float(np.clip(surface_percentile, 50.0, 99.5))
    z_ref = float(np.percentile(z_world, percentile))
    top_mask = (z_world >= z_ref - band_m) & (z_world <= z_ref + 2.0 * band_m)
    min_count = max(8, int(np.ceil(float(min_fraction) * xyz_world.shape[0])))

    if int(np.count_nonzero(top_mask)) < min_count:
        keep_count = min(min_count, xyz_world.shape[0])
        top_indices = np.argsort(np.abs(z_world - z_ref))[:keep_count]
    else:
        top_indices = np.flatnonzero(top_mask)

    top_world = xyz_world[top_indices]
    center_world = np.array(
        [
            float(np.median(top_world[:, 0])),
            float(np.median(top_world[:, 1])),
            float(np.median(top_world[:, 2])),
        ],
        dtype=np.float64,
    )

    world_to_cam = np.linalg.inv(cam_to_world)
    center_cam = transform_points(world_to_cam, center_world.reshape(1, 3))[0]
    return center_cam.astype(np.float32), center_world.astype(np.float32), top_world


def select_container_center_point(
    xyz_cam: np.ndarray,
    cam_to_world: np.ndarray,
    *,
    inner_fraction: float,
    z_percentile: float,
    z_offset_m: float,
    min_points: int,
    expected_world: np.ndarray | None = None,
    expected_max_distance_m: float = 0.0,
):
    """Estimate a placement point at the center of a container footprint."""
    if xyz_cam.shape[0] == 0:
        return None, None, np.zeros((0, 3), dtype=np.float32)

    xyz_world = transform_points(cam_to_world, xyz_cam)
    finite_mask = np.all(np.isfinite(xyz_world), axis=1)
    xyz_world = xyz_world[finite_mask]
    if xyz_world.shape[0] < max(int(min_points), 4):
        return None, None, np.zeros((0, 3), dtype=np.float32)

    if expected_world is not None and expected_max_distance_m > 0.0:
        expected = np.asarray(expected_world, dtype=np.float64).reshape(3)
        if np.all(np.isfinite(expected)):
            distances_xy = np.linalg.norm(xyz_world[:, :2] - expected[None, :2], axis=1)
            near_expected = distances_xy <= float(expected_max_distance_m)
            if int(np.count_nonzero(near_expected)) < max(int(min_points), 4):
                return None, None, np.zeros((0, 3), dtype=np.float32)
            xyz_world = xyz_world[near_expected]

    xy = xyz_world[:, :2]
    lo = np.percentile(xy, 5.0, axis=0)
    hi = np.percentile(xy, 95.0, axis=0)
    center_xy = (lo + hi) * 0.5

    half_extent = np.maximum((hi - lo) * 0.5, 1e-4)
    fraction = float(np.clip(inner_fraction, 0.05, 1.0))
    normalized = np.abs((xy - center_xy[None, :]) / half_extent[None, :])
    inner_mask = np.logical_and(normalized[:, 0] <= fraction, normalized[:, 1] <= fraction)
    inner_points = xyz_world[inner_mask]
    if inner_points.shape[0] < max(int(min_points), 4):
        inner_points = xyz_world

    percentile = float(np.clip(z_percentile, 0.0, 100.0))
    center_z = float(np.percentile(inner_points[:, 2], percentile)) + float(z_offset_m)
    center_world = np.array(
        [float(center_xy[0]), float(center_xy[1]), center_z],
        dtype=np.float64,
    )

    world_to_cam = np.linalg.inv(cam_to_world)
    center_cam = transform_points(world_to_cam, center_world.reshape(1, 3))[0]
    return center_cam.astype(np.float32), center_world.astype(np.float32), inner_points


def normalize_axis_yaw(yaw_rad: float) -> float:
    """Normalize an undirected principal-axis yaw into [-pi/2, pi/2]."""
    yaw = math.atan2(math.sin(float(yaw_rad)), math.cos(float(yaw_rad)))
    if yaw > math.pi / 2.0:
        yaw -= math.pi
    elif yaw < -math.pi / 2.0:
        yaw += math.pi
    return yaw


def estimate_principal_yaw_world(points_world: np.ndarray):
    """Estimate target long-axis yaw from masked world-frame points."""
    if points_world is None or points_world.shape[0] < 8:
        return None, 0
    xy = np.asarray(points_world[:, :2], dtype=np.float64)
    finite = np.all(np.isfinite(xy), axis=1)
    xy = xy[finite]
    if xy.shape[0] < 8:
        return None, int(xy.shape[0])
    centered = xy - np.median(xy, axis=0, keepdims=True)
    cov = centered.T @ centered / max(xy.shape[0] - 1, 1)
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        return None, int(xy.shape[0])
    axis = eigvecs[:, int(np.argmax(eigvals))]
    yaw = normalize_axis_yaw(math.atan2(float(axis[1]), float(axis[0])))
    return yaw, int(xy.shape[0])


def normalize_vector(vector: np.ndarray):
    vec = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9 or not np.all(np.isfinite(vec)):
        return None
    return vec / norm


def estimate_target_plane_world(
    points_world: np.ndarray,
    keypoint_world: np.ndarray,
    *,
    local_radius_m: float,
    min_points: int,
    max_samples: int,
    normal_reference: np.ndarray,
):
    """Fit a local object plane around the selected keypoint using PCA."""
    if points_world is None or points_world.shape[0] < max(int(min_points), 3):
        return None
    points = np.asarray(points_world, dtype=np.float64)
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if points.shape[0] < max(int(min_points), 3):
        return None

    keypoint = np.asarray(keypoint_world, dtype=np.float64).reshape(3)
    radius = max(float(local_radius_m), 1e-4)
    distances_xy = np.linalg.norm(points[:, :2] - keypoint[None, :2], axis=1)
    local = points[distances_xy <= radius]
    if local.shape[0] < max(int(min_points), 3):
        distances_3d = np.linalg.norm(points - keypoint[None, :], axis=1)
        keep_count = min(max(int(min_points), 3), points.shape[0])
        local = points[np.argsort(distances_3d)[:keep_count]]

    if max_samples > 0 and local.shape[0] > max_samples:
        keep = np.linspace(0, local.shape[0] - 1, num=max_samples, dtype=np.int32)
        local = local[keep]
    if local.shape[0] < max(int(min_points), 3):
        return None

    center = np.median(local, axis=0)
    centered = local - center[None, :]
    cov = centered.T @ centered / max(local.shape[0] - 1, 1)
    try:
        eigvals, eigvecs = np.linalg.eigh(cov)
    except np.linalg.LinAlgError:
        return None

    normal = normalize_vector(eigvecs[:, int(np.argmin(eigvals))])
    tangent = normalize_vector(eigvecs[:, int(np.argmax(eigvals))])
    reference = normalize_vector(normal_reference)
    if normal is None or tangent is None:
        return None
    if reference is not None and float(np.dot(normal, reference)) < 0.0:
        normal = -normal

    tangent = tangent - float(np.dot(tangent, normal)) * normal
    tangent = normalize_vector(tangent)
    if tangent is None:
        return None
    rms = float(np.sqrt(np.mean((centered @ normal) ** 2)))
    return {
        "center": center.astype(np.float32),
        "normal": normal.astype(np.float32),
        "tangent": tangent.astype(np.float32),
        "count": int(local.shape[0]),
        "rms": rms,
    }


def draw_overlay(
    rgb: np.ndarray,
    u: float = None,
    v: float = None,
    candidate_pixels=None,
    lines=None,
    valid: bool = True,
):
    """Draw the current perception result on top of the source RGB frame."""
    image = PILImage.fromarray(rgb.copy())
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    width, height = image.size

    if not valid:
        draw.rectangle((0, 0, width - 1, height - 1), outline=(255, 0, 0), width=4)

    if candidate_pixels:
        for candidate_index, pixel in enumerate(candidate_pixels):
            cu = int(round(pixel[0]))
            cv = int(round(pixel[1]))
            radius = 5
            draw.ellipse(
                (cu - radius, cv - radius, cu + radius, cv + radius),
                outline=(0, 255, 0),
                width=2,
            )
            draw.text((cu + 8, cv - 8), str(candidate_index), fill=(0, 255, 0), font=font)

    if u is not None and v is not None:
        cx = int(round(u))
        cy = int(round(v))
        radius = 8
        draw.line((cx - radius, cy, cx + radius, cy), fill=(255, 0, 0), width=2)
        draw.line((cx, cy - radius, cx, cy + radius), fill=(255, 0, 0), width=2)
        draw.ellipse((cx - 3, cy - 3, cx + 3, cy + 3), fill=(255, 255, 0))

    if lines:
        x0 = 10
        y0 = 10
        spacing = 3
        widths = []
        heights = []

        for line in lines:
            try:
                bbox = font.getbbox(line)
                text_width = bbox[2] - bbox[0]
                text_height = bbox[3] - bbox[1]
            except Exception:
                text_width, text_height = font.getmask(line).size
            widths.append(text_width)
            heights.append(max(text_height, 10))

        text_w = max(widths) if widths else 0
        text_h = sum(heights) + spacing * max(len(lines) - 1, 0)
        draw.rectangle((x0 - 4, y0 - 4, x0 + text_w + 4, y0 + text_h + 4), fill=(0, 0, 0))

        cursor_y = y0
        for line, text_h in zip(lines, heights):
            draw.text((x0, cursor_y), line, fill=(255, 255, 0), font=font)
            cursor_y += text_h + spacing

    return np.array(image, dtype=np.uint8)


class MaskDepthFusionNode(Node):
    """Join mask, depth, and camera intrinsics into a stable 3D keypoint."""

    def __init__(self):
        super().__init__("mask_depth_fusion_node")

        self.declare_parameter("mask_topic", "/sam3/mask")
        self.declare_parameter("prompt_topic", "/sam3/active_prompt")
        self.declare_parameter("color_topic", "/sim/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/sim/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/sim/camera/color/camera_info")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("overlay_topic", "/perception/keypoint_overlay")
        self.declare_parameter("candidate_text_topic", "/perception/keypoint_candidates_text")
        self.declare_parameter("candidate_text_limit", 4)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("max_frame_age_sec", 0.2)
        self.declare_parameter("frame_buffer_sec", 30.0)
        self.declare_parameter("cluster_count", 6)
        self.declare_parameter("cluster_max_samples", 2048)
        self.declare_parameter("cluster_pca_dim", 3)
        self.declare_parameter("cluster_meanshift_bandwidth_m", 0.06)
        self.declare_parameter("cluster_xyz_weight", 1.0)
        self.declare_parameter("cluster_rgb_weight", 0.35)
        self.declare_parameter("cluster_seed", 0)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("use_top_surface_estimator", True)
        self.declare_parameter("top_surface_band_m", 0.012)
        self.declare_parameter("top_surface_min_fraction", 0.15)
        self.declare_parameter("top_surface_percentile", 92.0)
        self.declare_parameter("enable_keypoint_stabilizer", True)
        self.declare_parameter("keypoint_filter_alpha", 0.25)
        self.declare_parameter("keypoint_jump_reset_m", 0.18)
        self.declare_parameter("keypoint_jump_hold_frames", 5)
        self.declare_parameter("candidate_lock_radius_m", 0.08)
        self.declare_parameter("invalid_reset_frames", 30)
        self.declare_parameter("plane_cluster_radius_m", 0.10)
        self.declare_parameter("plane_min_points", 24)
        self.declare_parameter("plane_max_samples", 1024)
        self.declare_parameter("plane_normal_reference", [0.0, 0.0, 1.0])
        self.declare_parameter("prompt_settle_sec", 0.8)
        self.declare_parameter("enable_container_center_keypoint", True)
        self.declare_parameter(
            "container_center_prompt_keywords",
            [
                "tray",
                "tray center",
                "sorting tray",
                "sorting tray center",
                "plate",
                "container",
                "托盘",
                "托盘中心",
                "盘子",
                "盘子中心",
            ],
        )
        self.declare_parameter("container_center_inner_fraction", 0.55)
        self.declare_parameter("container_center_z_percentile", 20.0)
        self.declare_parameter("container_center_z_offset_m", 0.0)
        self.declare_parameter("container_center_min_points", 48)
        self.declare_parameter("container_center_expected_world", [0.0, 0.0, 0.0])
        self.declare_parameter("container_center_expected_max_distance_m", 0.0)

        self.mask_topic = self.get_parameter("mask_topic").value
        self.prompt_topic = str(self.get_parameter("prompt_topic").value)
        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.score_topic = self.get_parameter("score_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value
        self.candidate_text_topic = str(self.get_parameter("candidate_text_topic").value)
        self.candidate_text_limit = max(
            1, int(self.get_parameter("candidate_text_limit").value)
        )
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.min_score = float(self.get_parameter("min_score").value)
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)
        self.frame_buffer_sec = float(self.get_parameter("frame_buffer_sec").value)
        self.cluster_count = int(self.get_parameter("cluster_count").value)
        self.cluster_max_samples = int(self.get_parameter("cluster_max_samples").value)
        self.cluster_pca_dim = int(self.get_parameter("cluster_pca_dim").value)
        self.cluster_meanshift_bandwidth_m = float(
            self.get_parameter("cluster_meanshift_bandwidth_m").value
        )
        self.cluster_xyz_weight = float(self.get_parameter("cluster_xyz_weight").value)
        self.cluster_rgb_weight = float(self.get_parameter("cluster_rgb_weight").value)
        self.cluster_seed = int(self.get_parameter("cluster_seed").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.use_top_surface_estimator = bool(
            self.get_parameter("use_top_surface_estimator").value
        )
        self.top_surface_band_m = float(self.get_parameter("top_surface_band_m").value)
        self.top_surface_min_fraction = float(
            self.get_parameter("top_surface_min_fraction").value
        )
        self.top_surface_percentile = float(self.get_parameter("top_surface_percentile").value)
        self.enable_keypoint_stabilizer = bool(
            self.get_parameter("enable_keypoint_stabilizer").value
        )
        self.keypoint_filter_alpha = float(self.get_parameter("keypoint_filter_alpha").value)
        self.keypoint_jump_reset_m = float(self.get_parameter("keypoint_jump_reset_m").value)
        self.keypoint_jump_hold_frames = max(
            0,
            int(self.get_parameter("keypoint_jump_hold_frames").value),
        )
        self.candidate_lock_radius_m = float(self.get_parameter("candidate_lock_radius_m").value)
        self.invalid_reset_frames = max(0, int(self.get_parameter("invalid_reset_frames").value))
        self.plane_cluster_radius_m = float(self.get_parameter("plane_cluster_radius_m").value)
        self.plane_min_points = max(3, int(self.get_parameter("plane_min_points").value))
        self.plane_max_samples = int(self.get_parameter("plane_max_samples").value)
        self.plane_normal_reference = parse_float_vector(
            self.get_parameter("plane_normal_reference").value,
            [0.0, 0.0, 1.0],
        )
        self.prompt_settle_sec = max(0.0, float(self.get_parameter("prompt_settle_sec").value))
        self.enable_container_center_keypoint = bool(
            self.get_parameter("enable_container_center_keypoint").value
        )
        self.container_center_prompt_keywords = tuple(
            str(value).strip().lower()
            for value in self.get_parameter("container_center_prompt_keywords").value
            if str(value).strip()
        )
        self.container_center_inner_fraction = float(
            self.get_parameter("container_center_inner_fraction").value
        )
        self.container_center_z_percentile = float(
            self.get_parameter("container_center_z_percentile").value
        )
        self.container_center_z_offset_m = float(
            self.get_parameter("container_center_z_offset_m").value
        )
        self.container_center_min_points = max(
            4,
            int(self.get_parameter("container_center_min_points").value),
        )
        self.container_center_expected_world = parse_float_vector(
            self.get_parameter("container_center_expected_world").value,
            [0.0, 0.0, 0.0],
        )
        self.container_center_expected_max_distance_m = max(
            0.0,
            float(self.get_parameter("container_center_expected_max_distance_m").value),
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub_mask = self.create_subscription(Image, self.mask_topic, self.on_mask, qos)
        self.sub_prompt = self.create_subscription(String, self.prompt_topic, self.on_prompt, 10)
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, qos)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, qos)
        self.sub_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.on_info,
            qos,
        )
        self.sub_score = self.create_subscription(Float32, self.score_topic, self.on_score, 10)

        self.pub_keypoint_px = self.create_publisher(PointStamped, "/perception/keypoint_px", 10)
        self.pub_keypoint_3d = self.create_publisher(PointStamped, "/perception/keypoint_3d", 10)
        self.pub_keypoint_x = self.create_publisher(Float32, "/perception/keypoint_3d/x", 10)
        self.pub_keypoint_y = self.create_publisher(Float32, "/perception/keypoint_3d/y", 10)
        self.pub_keypoint_z = self.create_publisher(Float32, "/perception/keypoint_3d/z", 10)
        self.pub_object_yaw = self.create_publisher(Float32, "/perception/object_yaw_rad", 10)
        self.pub_object_plane_normal = self.create_publisher(
            Vector3Stamped,
            "/perception/object_plane_normal",
            10,
        )
        self.pub_object_plane_tangent = self.create_publisher(
            Vector3Stamped,
            "/perception/object_plane_tangent",
            10,
        )
        self.pub_points = self.create_publisher(
            type(xyz_to_pointcloud2(np.zeros((0, 3), dtype=np.float32), CameraInfo().header)),
            "/perception/masked_points",
            1,
        )
        self.pub_candidates = self.create_publisher(
            type(xyz_to_pointcloud2(np.zeros((0, 3), dtype=np.float32), CameraInfo().header)),
            "/perception/keypoint_candidates",
            1,
        )
        self.pub_valid = self.create_publisher(type(make_bool_msg(True)), "/perception/valid", 10)
        self.pub_overlay = self.create_publisher(Image, self.overlay_topic, 10)
        self.pub_candidate_text = self.create_publisher(
            String,
            self.candidate_text_topic,
            10,
        )

        self.lock = threading.Lock()
        self.color_buffer = deque()
        self.depth_buffer = deque()
        self.info_buffer = deque()
        self.latest_score = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_status_log_time = 0.0
        self._stable_keypoint_xyz = None
        self._held_jump_count = 0
        self._invalid_count = 0
        self.current_prompt = ""
        self.prompt_changed_time = 0.0

        self.get_logger().info(
            f"mask_depth_fusion_node started prompt_topic={self.prompt_topic}"
        )

    def _log_status(self, message: str, *, level: str = "info"):
        now = time.time()
        if now - self._last_status_log_time < 2.0:
            return
        self._last_status_log_time = now
        if level == "warn":
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def publish_keypoint_xyz(self, x: float, y: float, z: float):
        self.pub_keypoint_x.publish(make_float32_msg(x))
        self.pub_keypoint_y.publish(make_float32_msg(y))
        self.pub_keypoint_z.publish(make_float32_msg(z))

    def publish_plane_vector(self, publisher, vector, header):
        msg = Vector3Stamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.target_frame
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        publisher.publish(msg)

    def reset_stabilizer(self):
        self._stable_keypoint_xyz = None
        self._held_jump_count = 0
        self._invalid_count = 0

    def on_prompt(self, msg: String):
        prompt = str(msg.data or "").strip()
        with self.lock:
            if prompt != self.current_prompt:
                self.reset_stabilizer()
                self.prompt_changed_time = time.time()
            self.current_prompt = prompt

    def _prompt_requests_container_center(self, prompt: str) -> bool:
        if not self.enable_container_center_keypoint:
            return False
        lowered = str(prompt or "").lower()
        if not lowered:
            return False
        return any(keyword in lowered for keyword in self.container_center_prompt_keywords)

    def select_candidate_index(self, candidates_xyz: np.ndarray, reference_xyz: np.ndarray):
        if candidates_xyz.shape[0] == 0:
            return None, "none"

        if self._stable_keypoint_xyz is not None and np.all(
            np.isfinite(self._stable_keypoint_xyz)
        ):
            previous = self._stable_keypoint_xyz.astype(np.float32)
            distances = np.linalg.norm(candidates_xyz - previous[None, :], axis=1)
            nearest_index = int(np.argmin(distances))
            nearest_distance = float(distances[nearest_index])
            if (
                self.candidate_lock_radius_m <= 0.0
                or nearest_distance <= self.candidate_lock_radius_m
            ):
                return nearest_index, f"locked:{nearest_distance:.3f}m"

        primary_index = select_primary_candidate(candidates_xyz, reference_xyz)
        if primary_index is None:
            return None, "none"
        return primary_index, "centroid"

    def stabilize_keypoint(self, raw_xyz: np.ndarray):
        raw = np.asarray(raw_xyz, dtype=np.float32)
        if not self.enable_keypoint_stabilizer:
            self._stable_keypoint_xyz = raw
            self._held_jump_count = 0
            return raw, "raw", 0.0

        if self._stable_keypoint_xyz is None or not np.all(np.isfinite(self._stable_keypoint_xyz)):
            self._stable_keypoint_xyz = raw
            self._held_jump_count = 0
            return raw, "init", 0.0

        previous = self._stable_keypoint_xyz.astype(np.float32)
        jump_m = float(np.linalg.norm(raw - previous))
        if (
            self.keypoint_jump_reset_m > 0.0
            and jump_m > self.keypoint_jump_reset_m
            and self._held_jump_count < self.keypoint_jump_hold_frames
        ):
            self._held_jump_count += 1
            return previous, f"hold_jump:{self._held_jump_count}", jump_m

        alpha = float(np.clip(self.keypoint_filter_alpha, 0.0, 1.0))
        stable = previous + alpha * (raw - previous)
        self._stable_keypoint_xyz = stable.astype(np.float32)
        self._held_jump_count = 0
        return self._stable_keypoint_xyz, "filtered", jump_m

    def publish_invalid(self, *, clear_candidates: bool = True):
        nan = float("nan")
        self.publish_keypoint_xyz(nan, nan, nan)
        self.pub_valid.publish(make_bool_msg(False))
        if clear_candidates:
            self._invalid_count += 1
            if self.invalid_reset_frames == 0 or self._invalid_count >= self.invalid_reset_frames:
                self.reset_stabilizer()
                self.publish_candidate_text(None, None)

    def publish_candidate_text(self, header, candidates_xyz: np.ndarray | None):
        msg = String()
        frame_id = "unknown"
        if header is not None:
            frame_id = str(getattr(header, "frame_id", "") or "unknown")

        if candidates_xyz is None or candidates_xyz.shape[0] == 0:
            msg.data = f"Candidates[{frame_id}]: none"
            self.pub_candidate_text.publish(msg)
            return

        limit = min(int(self.candidate_text_limit), int(candidates_xyz.shape[0]))
        parts = [f"Candidates[{frame_id}]"]
        for candidate_index in range(limit):
            candidate = candidates_xyz[candidate_index]
            parts.append(
                f"#{candidate_index}=({float(candidate[0]):.3f},"
                f"{float(candidate[1]):.3f},{float(candidate[2]):.3f})"
            )
        if candidates_xyz.shape[0] > limit:
            parts.append(f"... total={int(candidates_xyz.shape[0])}")
        msg.data = " ".join(parts)
        self.pub_candidate_text.publish(msg)

    def publish_overlay(
        self,
        color_pack,
        u: float = None,
        v: float = None,
        candidate_pixels=None,
        lines=None,
        valid: bool = True,
    ):
        if color_pack is None:
            return
        rgb, header = color_pack
        overlay = draw_overlay(
            rgb,
            u=u,
            v=v,
            candidate_pixels=candidate_pixels,
            lines=lines,
            valid=valid,
        )
        self.pub_overlay.publish(rgb8_to_imgmsg(overlay, header))

    def on_color(self, msg: Image):
        with self.lock:
            self._buffer_push(self.color_buffer, imgmsg_to_rgb8(msg), msg.header)

    def on_depth(self, msg: Image):
        try:
            depth_m = imgmsg_to_depth_meters(msg, self.depth_scale)
        except Exception as exc:
            self._log_status(f"unsupported depth frame: {exc}", level="warn")
            return
        with self.lock:
            self._buffer_push(self.depth_buffer, depth_m, msg.header)

    def on_info(self, msg: CameraInfo):
        with self.lock:
            self._buffer_push(self.info_buffer, msg, msg.header)

    def on_score(self, msg: Float32):
        with self.lock:
            self.latest_score = float(msg.data)

    def _buffer_push(self, buffer: deque, payload, header):
        now_sec = time.time()
        buffer.append((payload, header, now_sec))
        if self.frame_buffer_sec <= 0.0:
            while len(buffer) > 1:
                buffer.popleft()
            return

        cutoff = now_sec - self.frame_buffer_sec
        while buffer and buffer[0][2] < cutoff:
            buffer.popleft()

    def _header_delta_sec(self, ref_header, other_header):
        if ref_header is None or other_header is None:
            return None
        ref_zero = ref_header.stamp.sec == 0 and ref_header.stamp.nanosec == 0
        other_zero = other_header.stamp.sec == 0 and other_header.stamp.nanosec == 0
        if ref_zero or other_zero:
            return 0.0
        ref_time = Time.from_msg(ref_header.stamp)
        other_time = Time.from_msg(other_header.stamp)
        return abs((ref_time - other_time).nanoseconds) * 1e-9

    def _headers_are_aligned(self, ref_header, other_header) -> bool:
        if self.max_frame_age_sec <= 0.0:
            return True
        age = self._header_delta_sec(ref_header, other_header)
        if age is None:
            return False
        return age <= self.max_frame_age_sec

    def _find_buffer_match(self, ref_header, buffer: deque):
        if not buffer:
            return None

        best_payload = None
        best_header = None
        best_age = None

        for payload, header, _recv_time in reversed(buffer):
            age = self._header_delta_sec(ref_header, header)
            if age is None:
                continue
            if best_age is None or age < best_age:
                best_payload = payload
                best_header = header
                best_age = age
                if age <= 1e-9:
                    break

        if best_payload is None:
            return None
        if self.max_frame_age_sec > 0.0 and (
            best_age is None or best_age > self.max_frame_age_sec
        ):
            return None
        return best_payload, best_header

    def on_mask(self, msg: Image):
        mask = imgmsg_to_mask_u8(msg)

        with self.lock:
            score = self.latest_score
            current_prompt = self.current_prompt
            prompt_changed_time = self.prompt_changed_time
            color_pack = self._find_buffer_match(msg.header, self.color_buffer)
            depth_pack = self._find_buffer_match(msg.header, self.depth_buffer)
            info_pack = self._find_buffer_match(msg.header, self.info_buffer)

        info = None if info_pack is None else info_pack[0]

        if depth_pack is None or info is None:
            self._log_status("waiting for depth/camera_info", level="warn")
            self.publish_overlay(color_pack, lines=["waiting for depth/camera_info"], valid=False)
            self.publish_invalid()
            return

        if color_pack is None:
            self._log_status("waiting for color frame", level="warn")
            self.publish_invalid()
            return

        if not self._headers_are_aligned(
            msg.header, depth_pack[1]
        ) or not self._headers_are_aligned(msg.header, info.header):
            self._log_status("stale depth or camera_info", level="warn")
            self.publish_overlay(color_pack, lines=["stale depth or camera_info"], valid=False)
            self.publish_invalid()
            return

        if color_pack is not None and not self._headers_are_aligned(msg.header, color_pack[1]):
            self._log_status("stale color frame", level="warn")
            self.publish_overlay(color_pack, lines=["stale color frame"], valid=False)
            self.publish_invalid()
            return

        prompt_settle_remaining = (
            self.prompt_settle_sec - (time.time() - float(prompt_changed_time))
            if prompt_changed_time > 0.0
            else 0.0
        )
        if prompt_settle_remaining > 0.0:
            self.publish_overlay(
                color_pack,
                lines=[
                    f"prompt={current_prompt or '<none>'}",
                    f"waiting prompt settle {prompt_settle_remaining:.2f}s",
                ],
                valid=False,
            )
            self.publish_invalid(clear_candidates=False)
            return

        if score < self.min_score:
            self._log_status(
                f"score {score:.3f} below min_score {self.min_score:.3f}",
                level="warn",
            )
            self.publish_overlay(
                color_pack,
                lines=[f"score={score:.2f} < {self.min_score:.2f}"],
                valid=False,
            )
            self.publish_invalid()
            return

        rgb, _color_header = color_pack
        depth, _depth_header = depth_pack

        if depth.shape != mask.shape:
            self.get_logger().warn(
                f"mask/depth shape mismatch: mask={mask.shape}, depth={depth.shape}"
            )
            self.publish_overlay(color_pack, lines=["mask/depth shape mismatch"], valid=False)
            self.publish_invalid()
            return

        fx, fy, cx, cy = intrinsics_from_camera_info(info)

        xyz_samples, rgb_samples, uv_samples = masked_rgbd_to_xyzrgbuv(
            depth,
            mask,
            rgb,
            fx,
            fy,
            cx,
            cy,
        )
        if xyz_samples.shape[0] == 0:
            self._log_status("no valid depth in mask", level="warn")
            self.publish_overlay(color_pack, lines=["no valid depth in mask"], valid=False)
            self.publish_invalid()
            return

        candidates_xyz, _candidate_uv = rekep_rgbd_candidates(
            xyz_samples,
            rgb_samples,
            uv_samples,
            num_clusters=self.cluster_count,
            max_samples=self.cluster_max_samples,
            pca_dim=self.cluster_pca_dim,
            meanshift_bandwidth=self.cluster_meanshift_bandwidth_m,
            xyz_weight=self.cluster_xyz_weight,
            rgb_weight=self.cluster_rgb_weight,
            seed=self.cluster_seed,
        )
        if candidates_xyz.shape[0] == 0:
            self._log_status("no cluster proposals", level="warn")
            self.publish_overlay(color_pack, lines=["no cluster proposals"], valid=False)
            self.publish_invalid()
            return

        self.publish_candidate_text(msg.header, candidates_xyz)

        primary_index, candidate_select_state = self.select_candidate_index(
            candidates_xyz,
            xyz_samples,
        )
        if primary_index is None:
            self._log_status("no primary proposal", level="warn")
            self.publish_overlay(color_pack, lines=["no primary proposal"], valid=False)
            self.publish_invalid(clear_candidates=False)
            return

        selected_candidate_xyz = candidates_xyz[primary_index]
        container_center_active = self._prompt_requests_container_center(current_prompt)
        cam_to_world = None
        primary_world_xyz = None
        top_world = np.zeros((0, 3), dtype=np.float32)
        try:
            if self.target_frame == msg.header.frame_id:
                cam_to_world = np.eye(4, dtype=np.float64)
            else:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time(),
                )
                cam_to_world = transform_to_matrix(tf_msg)

            if container_center_active:
                primary_xyz, primary_world_xyz, top_world = select_container_center_point(
                    xyz_samples,
                    cam_to_world,
                    inner_fraction=self.container_center_inner_fraction,
                    z_percentile=self.container_center_z_percentile,
                    z_offset_m=self.container_center_z_offset_m,
                    min_points=self.container_center_min_points,
                    expected_world=self.container_center_expected_world,
                    expected_max_distance_m=self.container_center_expected_max_distance_m,
                )
                candidate_select_state = "container_center"
            elif self.use_top_surface_estimator:
                local_radius = max(
                    self.candidate_lock_radius_m,
                    self.cluster_meanshift_bandwidth_m,
                )
                local_xy_dist = np.linalg.norm(
                    xyz_samples[:, :2] - selected_candidate_xyz[None, :2],
                    axis=1,
                )
                local_mask = local_xy_dist <= max(local_radius, 1e-4)
                top_input_xyz = (
                    xyz_samples[local_mask]
                    if int(np.count_nonzero(local_mask)) >= 32
                    else xyz_samples
                )
                primary_xyz, primary_world_xyz, top_world = select_top_surface_point(
                    top_input_xyz,
                    cam_to_world,
                    top_band_m=self.top_surface_band_m,
                    min_fraction=self.top_surface_min_fraction,
                    surface_percentile=self.top_surface_percentile,
                )
            else:
                primary_xyz = selected_candidate_xyz.astype(np.float32)
                primary_world_xyz = transform_points(cam_to_world, primary_xyz.reshape(1, 3))[0]
        except Exception as exc:
            self.get_logger().warn(f"keypoint frame/TF lookup failed: {exc}")
            primary_xyz = selected_candidate_xyz
            primary_world_xyz = None
            top_world = np.zeros((0, 3), dtype=np.float32)

        if primary_xyz is None:
            self._log_status("no keypoint estimate", level="warn")
            self.publish_overlay(color_pack, lines=["no keypoint estimate"], valid=False)
            self.publish_invalid(clear_candidates=False)
            return

        raw_primary_xyz = np.asarray(primary_xyz, dtype=np.float32)
        primary_xyz, stabilizer_state, jump_m = self.stabilize_keypoint(raw_primary_xyz)
        if cam_to_world is not None:
            primary_world_xyz = transform_points(cam_to_world, primary_xyz.reshape(1, 3))[0]

        object_yaw_rad = None
        object_yaw_points = 0
        object_plane = None
        if cam_to_world is not None:
            yaw_points_world = top_world
            if yaw_points_world.shape[0] < 8:
                yaw_points_world = transform_points(cam_to_world, xyz_samples)
            object_yaw_rad, object_yaw_points = estimate_principal_yaw_world(yaw_points_world)
            plane_points_world = yaw_points_world
            if plane_points_world.shape[0] < self.plane_min_points:
                plane_points_world = transform_points(cam_to_world, xyz_samples)
            if primary_world_xyz is not None:
                object_plane = estimate_target_plane_world(
                    plane_points_world,
                    primary_world_xyz,
                    local_radius_m=max(
                        self.plane_cluster_radius_m,
                        self.cluster_meanshift_bandwidth_m,
                    ),
                    min_points=self.plane_min_points,
                    max_samples=self.plane_max_samples,
                    normal_reference=self.plane_normal_reference,
                )

        primary_uv = xyz_to_pixel(
            float(primary_xyz[0]),
            float(primary_xyz[1]),
            float(primary_xyz[2]),
            fx,
            fy,
            cx,
            cy,
        )
        if primary_uv is None:
            self._log_status("invalid primary proposal", level="warn")
            self.publish_overlay(color_pack, lines=["invalid primary proposal"], valid=False)
            self.publish_invalid(clear_candidates=False)
            return

        candidate_pixels = []
        for candidate_xyz in candidates_xyz:
            pixel = xyz_to_pixel(
                float(candidate_xyz[0]),
                float(candidate_xyz[1]),
                float(candidate_xyz[2]),
                fx,
                fy,
                cx,
                cy,
            )
            if pixel is not None:
                candidate_pixels.append(pixel)

        u, v = primary_uv
        x, y, z = [float(value) for value in primary_xyz]

        self.pub_keypoint_px.publish(make_point_stamped(u, v, 0.0, msg.header))
        self.pub_keypoint_3d.publish(make_point_stamped(x, y, z, msg.header))
        if object_yaw_rad is not None and math.isfinite(object_yaw_rad):
            self.pub_object_yaw.publish(make_float32_msg(float(object_yaw_rad)))
        if object_plane is not None:
            self.publish_plane_vector(
                self.pub_object_plane_normal,
                object_plane["normal"],
                msg.header,
            )
            self.publish_plane_vector(
                self.pub_object_plane_tangent,
                object_plane["tangent"],
                msg.header,
            )
        self.pub_candidates.publish(xyz_to_pointcloud2(candidates_xyz, msg.header))
        self.publish_keypoint_xyz(x, y, z)
        self.publish_overlay(
            color_pack,
            u=u,
            v=v,
            candidate_pixels=candidate_pixels,
            lines=[
                f"prompt={current_prompt or '<none>'}",
                f"candidates={candidates_xyz.shape[0]}",
                f"surface_pts={top_world.shape[0]}",
                f"candidate={candidate_select_state}",
                f"stabilizer={stabilizer_state} jump={jump_m:.3f} m",
                f"px=({u:.1f}, {v:.1f})",
                f"x={x:.3f} m",
                f"y={y:.3f} m",
                f"z={z:.3f} m",
                (
                    f"yaw={math.degrees(object_yaw_rad):.1f} deg pts={object_yaw_points}"
                    if object_yaw_rad is not None
                    else "yaw=n/a"
                ),
                (
                    "plane_n="
                    f"({float(object_plane['normal'][0]):.2f},"
                    f"{float(object_plane['normal'][1]):.2f},"
                    f"{float(object_plane['normal'][2]):.2f}) "
                    f"pts={object_plane['count']} rms={object_plane['rms']:.3f}"
                    if object_plane is not None
                    else "plane=n/a"
                ),
                (
                    f"world_z={float(primary_world_xyz[2]):.3f} m"
                    if primary_world_xyz is not None
                    else "world_z=n/a"
                ),
            ],
            valid=True,
        )
        self._log_status(
            f"valid target score={score:.3f} keypoint=({x:.3f}, {y:.3f}, {z:.3f}) "
            f"candidate={candidate_select_state} stabilizer={stabilizer_state} jump={jump_m:.3f} "
            f"candidates={candidates_xyz.shape[0]} "
            + (
                f"yaw={math.degrees(object_yaw_rad):.1f}deg"
                if object_yaw_rad is not None
                else "yaw=n/a"
            )
            + (
                f" plane_n=({float(object_plane['normal'][0]):.2f},"
                f"{float(object_plane['normal'][1]):.2f},"
                f"{float(object_plane['normal'][2]):.2f})"
                if object_plane is not None
                else " plane=n/a"
            )
        )

        xyz = masked_depth_to_xyz(depth, mask, fx, fy, cx, cy)
        if xyz.shape[0] > 0:
            self.pub_points.publish(xyz_to_pointcloud2(xyz, msg.header))

        self._invalid_count = 0
        self.pub_valid.publish(make_bool_msg(True))


def main(args=None):
    rclpy.init(args=args)
    node = MaskDepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
