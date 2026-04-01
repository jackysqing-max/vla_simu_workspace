"""Fuse SAM3 mask output with depth to produce a stable 3D keypoint."""

import threading

import numpy as np
import rclpy
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
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


def select_top_surface_point(
    xyz_cam: np.ndarray,
    cam_to_world: np.ndarray,
    *,
    top_band_m: float,
    min_fraction: float,
):
    """Estimate the center of the visible top surface in camera coordinates.

    We convert the masked cloud into the world frame, take the points near the
    maximum world `z`, and average that slice. This yields a point on the cube's
    top face instead of the object centroid.
    """
    if xyz_cam.shape[0] == 0:
        return None, None, np.zeros((0, 3), dtype=np.float32)

    xyz_world = transform_points(cam_to_world, xyz_cam)
    z_world = xyz_world[:, 2]
    z_max = float(np.max(z_world))

    top_mask = z_world >= z_max - max(float(top_band_m), 1e-4)
    min_count = max(8, int(np.ceil(float(min_fraction) * xyz_world.shape[0])))

    if int(np.count_nonzero(top_mask)) < min_count:
        keep_count = min(min_count, xyz_world.shape[0])
        top_indices = np.argsort(z_world)[-keep_count:]
    else:
        top_indices = np.flatnonzero(top_mask)

    top_world = xyz_world[top_indices]
    center_world = np.array(
        [
            float(np.median(top_world[:, 0])),
            float(np.median(top_world[:, 1])),
            float(np.mean(top_world[:, 2])),
        ],
        dtype=np.float64,
    )

    world_to_cam = np.linalg.inv(cam_to_world)
    center_cam = transform_points(world_to_cam, center_world.reshape(1, 3))[0]
    return center_cam.astype(np.float32), center_world.astype(np.float32), top_world


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
        self.declare_parameter("color_topic", "/sim/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/sim/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/sim/camera/color/camera_info")
        self.declare_parameter("score_topic", "/sam3/score")
        self.declare_parameter("overlay_topic", "/perception/keypoint_overlay")
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("max_frame_age_sec", 0.2)
        self.declare_parameter("cluster_count", 6)
        self.declare_parameter("cluster_max_samples", 2048)
        self.declare_parameter("cluster_pca_dim", 3)
        self.declare_parameter("cluster_meanshift_bandwidth_m", 0.06)
        self.declare_parameter("cluster_xyz_weight", 1.0)
        self.declare_parameter("cluster_rgb_weight", 0.35)
        self.declare_parameter("cluster_seed", 0)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("top_surface_band_m", 0.012)
        self.declare_parameter("top_surface_min_fraction", 0.15)

        self.mask_topic = self.get_parameter("mask_topic").value
        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.score_topic = self.get_parameter("score_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value
        self.min_score = float(self.get_parameter("min_score").value)
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)
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
        self.top_surface_band_m = float(self.get_parameter("top_surface_band_m").value)
        self.top_surface_min_fraction = float(
            self.get_parameter("top_surface_min_fraction").value
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub_mask = self.create_subscription(Image, self.mask_topic, self.on_mask, qos)
        self.sub_color = self.create_subscription(Image, self.color_topic, self.on_color, qos)
        self.sub_depth = self.create_subscription(Image, self.depth_topic, self.on_depth, qos)
        self.sub_info = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, 10)
        self.sub_score = self.create_subscription(Float32, self.score_topic, self.on_score, 10)

        self.pub_keypoint_px = self.create_publisher(PointStamped, "/perception/keypoint_px", 10)
        self.pub_keypoint_3d = self.create_publisher(PointStamped, "/perception/keypoint_3d", 10)
        self.pub_keypoint_x = self.create_publisher(Float32, "/perception/keypoint_3d/x", 10)
        self.pub_keypoint_y = self.create_publisher(Float32, "/perception/keypoint_3d/y", 10)
        self.pub_keypoint_z = self.create_publisher(Float32, "/perception/keypoint_3d/z", 10)
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

        self.lock = threading.Lock()
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
        self.latest_score = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("mask_depth_fusion_node started")

    def publish_keypoint_xyz(self, x: float, y: float, z: float):
        self.pub_keypoint_x.publish(make_float32_msg(x))
        self.pub_keypoint_y.publish(make_float32_msg(y))
        self.pub_keypoint_z.publish(make_float32_msg(z))

    def publish_invalid(self):
        nan = float("nan")
        self.publish_keypoint_xyz(nan, nan, nan)
        self.pub_valid.publish(make_bool_msg(False))

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
            self.latest_color = (imgmsg_to_rgb8(msg), msg.header)

    def on_depth(self, msg: Image):
        with self.lock:
            self.latest_depth = (imgmsg_to_depth32f(msg), msg.header)

    def on_info(self, msg: CameraInfo):
        with self.lock:
            self.latest_info = msg

    def on_score(self, msg: Float32):
        with self.lock:
            self.latest_score = float(msg.data)

    def _headers_are_aligned(self, ref_header, other_header) -> bool:
        if self.max_frame_age_sec <= 0.0:
            return True
        if ref_header is None or other_header is None:
            return False
        ref_zero = ref_header.stamp.sec == 0 and ref_header.stamp.nanosec == 0
        other_zero = other_header.stamp.sec == 0 and other_header.stamp.nanosec == 0
        if ref_zero or other_zero:
            return True
        ref_time = Time.from_msg(ref_header.stamp)
        other_time = Time.from_msg(other_header.stamp)
        age = abs((ref_time - other_time).nanoseconds) * 1e-9
        return age <= self.max_frame_age_sec

    def on_mask(self, msg: Image):
        mask = imgmsg_to_mask_u8(msg)

        with self.lock:
            color_pack = self.latest_color
            depth_pack = self.latest_depth
            info = self.latest_info
            score = self.latest_score

        if depth_pack is None or info is None:
            self.publish_overlay(color_pack, lines=["waiting for depth/camera_info"], valid=False)
            self.publish_invalid()
            return

        if color_pack is None:
            self.publish_invalid()
            return

        if not self._headers_are_aligned(msg.header, depth_pack[1]) or not self._headers_are_aligned(
            msg.header,
            info.header,
        ):
            self.publish_overlay(color_pack, lines=["stale depth or camera_info"], valid=False)
            self.publish_invalid()
            return

        if color_pack is not None and not self._headers_are_aligned(msg.header, color_pack[1]):
            self.publish_overlay(color_pack, lines=["stale color frame"], valid=False)
            self.publish_invalid()
            return

        if score < self.min_score:
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
            self.publish_overlay(color_pack, lines=["no cluster proposals"], valid=False)
            self.publish_invalid()
            return

        primary_index = select_primary_candidate(candidates_xyz, xyz_samples)
        if primary_index is None:
            self.publish_overlay(color_pack, lines=["no primary proposal"], valid=False)
            self.publish_invalid()
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, Time())
            cam_to_world = transform_to_matrix(tf_msg)
            primary_xyz, primary_world_xyz, top_world = select_top_surface_point(
                xyz_samples,
                cam_to_world,
                top_band_m=self.top_surface_band_m,
                min_fraction=self.top_surface_min_fraction,
            )
        except Exception as exc:
            self.get_logger().warn(f"top-surface TF lookup failed: {exc}")
            primary_xyz = candidates_xyz[primary_index]
            primary_world_xyz = None
            top_world = np.zeros((0, 3), dtype=np.float32)

        if primary_xyz is None:
            self.publish_overlay(color_pack, lines=["no top-surface point"], valid=False)
            self.publish_invalid()
            return

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
            self.publish_overlay(color_pack, lines=["invalid primary proposal"], valid=False)
            self.publish_invalid()
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
        self.pub_candidates.publish(xyz_to_pointcloud2(candidates_xyz, msg.header))
        self.publish_keypoint_xyz(x, y, z)
        self.publish_overlay(
            color_pack,
            u=u,
            v=v,
            candidate_pixels=candidate_pixels,
            lines=[
                f"candidates={candidates_xyz.shape[0]}",
                f"surface_pts={top_world.shape[0]}",
                f"px=({u:.1f}, {v:.1f})",
                f"x={x:.3f} m",
                f"y={y:.3f} m",
                f"z={z:.3f} m",
                (
                    f"world_z={float(primary_world_xyz[2]):.3f} m"
                    if primary_world_xyz is not None
                    else "world_z=n/a"
                ),
            ],
            valid=True,
        )

        xyz = masked_depth_to_xyz(depth, mask, fx, fy, cx, cy)
        if xyz.shape[0] > 0:
            self.pub_points.publish(xyz_to_pointcloud2(xyz, msg.header))

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
