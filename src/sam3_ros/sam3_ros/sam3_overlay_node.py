import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
import threading


def imgmsg_to_rgb(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return arr[:, :, ::-1].copy()
    return arr.copy()


def imgmsg_to_mask_u8(msg: Image) -> np.ndarray:
    # mono8 HxW
    h, w = msg.height, msg.width
    m = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
    return m.copy()


def rgb_to_imgmsg(rgb: np.ndarray, header) -> Image:
    out = Image()
    out.header = header
    out.height, out.width = rgb.shape[0], rgb.shape[1]
    out.encoding = "rgb8"
    out.is_bigendian = 0
    out.step = out.width * 3
    out.data = rgb.tobytes()
    return out


def overlay(rgb: np.ndarray, mask_u8: np.ndarray, alpha: float) -> np.ndarray:
    # mask_u8: 0/255
    if mask_u8 is None:
        return rgb
    m = mask_u8.astype(bool)
    if m.sum() == 0:
        return rgb
    out = rgb.astype(np.float32).copy()
    color = np.array([255, 0, 0], dtype=np.float32)
    out[m] = out[m] * (1 - alpha) + color * alpha
    return np.clip(out, 0, 255).astype(np.uint8)


class Sam3OverlayNode(Node):
    def __init__(self):
        super().__init__("sam3_overlay_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("mask_topic", "/sam3/mask")
        self.declare_parameter("alpha", 0.45)

        self.image_topic = self.get_parameter("image_topic").value
        self.mask_topic = self.get_parameter("mask_topic").value
        self.alpha = float(self.get_parameter("alpha").value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub_img = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.sub_mask = self.create_subscription(Image, self.mask_topic, self.on_mask, qos)
        self.pub = self.create_publisher(Image, "/sam3/overlay", 1)

        self.lock = threading.Lock()
        self.last_mask = None

        self.get_logger().info(f"Subscribed: {self.image_topic} + {self.mask_topic} | Publishing: /sam3/overlay")

    def on_mask(self, msg: Image):
        m = imgmsg_to_mask_u8(msg)
        with self.lock:
            self.last_mask = m

    def on_image(self, msg: Image):
        rgb = imgmsg_to_rgb(msg)
        with self.lock:
            m = self.last_mask
        ov = overlay(rgb, m, alpha=self.alpha)
        self.pub.publish(rgb_to_imgmsg(ov, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = Sam3OverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()