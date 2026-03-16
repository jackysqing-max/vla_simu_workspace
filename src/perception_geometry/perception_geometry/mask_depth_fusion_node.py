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

from perception_geometry.pointcloud_ops import (
    intrinsics_from_camera_info,
    mask_centroid,
    masked_depth_to_xyz,
    masked_valid_depth_values,
    pixel_to_xyz,
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


def draw_overlay(rgb: np.ndarray, u: float = None, v: float = None, lines=None, valid: bool = True):
    """Draw the current perception result on top of the source RGB frame."""
    image = PILImage.fromarray(rgb.copy())
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()
    width, height = image.size

    if not valid:
        draw.rectangle((0, 0, width - 1, height - 1), outline=(255, 0, 0), width=4)

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

        self.mask_topic = self.get_parameter("mask_topic").value
        self.color_topic = self.get_parameter("color_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.score_topic = self.get_parameter("score_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value
        self.min_score = float(self.get_parameter("min_score").value)
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)

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
        self.pub_valid = self.create_publisher(type(make_bool_msg(True)), "/perception/valid", 10)
        self.pub_overlay = self.create_publisher(Image, self.overlay_topic, 10)

        self.lock = threading.Lock()
        self.latest_color = None
        self.latest_depth = None
        self.latest_info = None
        self.latest_score = 0.0

        self.get_logger().info("mask_depth_fusion_node started")

    def publish_keypoint_xyz(self, x: float, y: float, z: float):
        self.pub_keypoint_x.publish(make_float32_msg(x))
        self.pub_keypoint_y.publish(make_float32_msg(y))
        self.pub_keypoint_z.publish(make_float32_msg(z))

    def publish_invalid(self):
        nan = float("nan")
        self.publish_keypoint_xyz(nan, nan, nan)
        self.pub_valid.publish(make_bool_msg(False))

    def publish_overlay(self, color_pack, u: float = None, v: float = None, lines=None, valid: bool = True):
        if color_pack is None:
            return
        rgb, header = color_pack
        overlay = draw_overlay(rgb, u=u, v=v, lines=lines, valid=valid)
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

        depth, _depth_header = depth_pack

        if depth.shape != mask.shape:
            self.get_logger().warn(
                f"mask/depth shape mismatch: mask={mask.shape}, depth={depth.shape}"
            )
            self.publish_overlay(color_pack, lines=["mask/depth shape mismatch"], valid=False)
            self.publish_invalid()
            return

        centroid = mask_centroid(mask)
        if centroid is None:
            self.publish_overlay(color_pack, lines=["mask empty"], valid=False)
            self.publish_invalid()
            return

        u, v = centroid
        fx, fy, cx, cy = intrinsics_from_camera_info(info)

        # Using the median valid depth inside the mask suppresses outliers from
        # isolated bad pixels and gives a more stable 3D target.
        depth_values, _ = masked_valid_depth_values(mask, depth)
        if depth_values.size == 0:
            self.publish_overlay(color_pack, u=u, v=v, lines=["no valid depth in mask"], valid=False)
            self.publish_invalid()
            return

        z_med = float(np.median(depth_values))
        x, y, z = pixel_to_xyz(u, v, z_med, fx, fy, cx, cy)

        self.pub_keypoint_px.publish(make_point_stamped(u, v, 0.0, msg.header))
        self.pub_keypoint_3d.publish(make_point_stamped(x, y, z, msg.header))
        self.publish_keypoint_xyz(x, y, z)
        self.publish_overlay(
            color_pack,
            u=u,
            v=v,
            lines=[
                f"px=({u:.1f}, {v:.1f})",
                f"x={x:.3f} m",
                f"y={y:.3f} m",
                f"z={z:.3f} m",
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
