import threading, queue, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image

import torch
from PIL import Image as PILImage
from transformers import Sam3Model, Sam3Processor


def imgmsg_to_rgb(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> RGB uint8 (H,W,3)"""
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return arr[:, :, ::-1].copy()
    # v4l2_camera 常见输出 rgb8
    return arr.copy()


def rgb_to_imgmsg(rgb: np.ndarray, header) -> Image:
    """RGB uint8 (H,W,3) -> sensor_msgs/Image (rgb8)"""
    out = Image()
    out.header = header
    out.height, out.width = rgb.shape[0], rgb.shape[1]
    out.encoding = "rgb8"
    out.is_bigendian = 0
    out.step = out.width * 3
    out.data = rgb.tobytes()
    return out


def resize_keep_aspect(rgb: np.ndarray, max_side: int) -> np.ndarray:
    """缩放图像，保证最长边 <= max_side（保持比例）"""
    h, w = rgb.shape[:2]
    if max(h, w) <= max_side:
        return rgb
    scale = max_side / float(max(h, w))
    nh, nw = int(round(h * scale)), int(round(w * scale))
    # PIL resize 不依赖 opencv
    return np.array(PILImage.fromarray(rgb).resize((nw, nh)))


def overlay_mask(rgb: np.ndarray, mask: np.ndarray, alpha: float = 0.45) -> np.ndarray:
    """在 RGB 上叠加单个 mask（红色透明层）"""
    if mask is None:
        return rgb
    out = rgb.astype(np.float32).copy()
    color = np.array([255, 0, 0], dtype=np.float32)
    m = mask.astype(bool)
    out[m] = out[m] * (1 - alpha) + color * alpha
    return np.clip(out, 0, 255).astype(np.uint8)


class Sam3Node(Node):
    def __init__(self):
        super().__init__("sam3_node")

        # ---- parameters ----
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("prompt", "glasses")
        self.declare_parameter("score_th", 0.3)
        self.declare_parameter("mask_th", 0.5)
        self.declare_parameter("max_side", 640)
        self.declare_parameter("alpha", 0.45)

        # 推理频率：SAM3 更新 overlay 的频率（慢也没关系）
        self.declare_parameter("infer_hz", 0.5)

        # 发布频率：overlay topic 的发布频率（保证“视频流连续”）
        self.declare_parameter("pub_hz", 15.0)

        # device: "cpu" 或 "cuda"
        self.declare_parameter("device", "cpu")

        self.image_topic = self.get_parameter("image_topic").value
        self.prompt = self.get_parameter("prompt").value
        self.score_th = float(self.get_parameter("score_th").value)
        self.mask_th = float(self.get_parameter("mask_th").value)
        self.max_side = int(self.get_parameter("max_side").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.infer_hz = float(self.get_parameter("infer_hz").value)
        self.pub_hz = float(self.get_parameter("pub_hz").value)
        self.device = self.get_parameter("device").value

        # ---- QoS: 只取最新帧，避免堆积延迟 ----
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.pub = self.create_publisher(Image, "/sam3/overlay", 1)

        # ---- frame buffer (size=1) ----
        self.q = queue.Queue(maxsize=1)

        # ---- shared state for publisher ----
        self.last_header = None
        self.last_overlay = None
        self.lock = threading.Lock()

        # ---- load model ----
        self.get_logger().info(f"Loading SAM3 on {self.device} ...")
        self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
        self.processor = Sam3Processor.from_pretrained("facebook/sam3")
        self.model.eval()

        # speed knobs (safe-ish)
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        torch.backends.cudnn.benchmark = True

        # ---- worker thread: inference ----
        self.worker = threading.Thread(target=self.infer_loop, daemon=True)
        self.worker.start()

        # ---- timer: publish overlay at fixed rate ----
        self.timer = self.create_timer(1.0 / max(self.pub_hz, 1e-6), self.publish_overlay)

        self.get_logger().info(
            f"Subscribed: {self.image_topic} | Publishing: /sam3/overlay | infer_hz={self.infer_hz} | pub_hz={self.pub_hz}"
        )

    def on_image(self, msg: Image):
        rgb = imgmsg_to_rgb(msg)
        self.last_header = msg.header

        # 预缩放：减少后续推理和发布带宽（可选）
        rgb_small = resize_keep_aspect(rgb, self.max_side)

        # 如果还没有 overlay，就先用原图占位，保证 rqt 立刻有画面
        with self.lock:
            if self.last_overlay is None:
                self.last_overlay = rgb_small

        # 覆盖式缓存：永远只保留最新帧给推理线程
        if self.q.full():
            try:
                _ = self.q.get_nowait()
            except Exception:
                pass
        self.q.put_nowait(rgb_small)

    def publish_overlay(self):
        # 固定频率发布最新 overlay（即使推理很慢也能保持“视频流连续”）
        hdr = self.last_header
        if hdr is None:
            return
        with self.lock:
            ov = self.last_overlay
        if ov is None:
            return
        self.pub.publish(rgb_to_imgmsg(ov, hdr))

    def infer_loop(self):
        period = 1.0 / max(self.infer_hz, 1e-6)
        last_run = 0.0

        while rclpy.ok():
            now = time.time()
            if now - last_run < period:
                time.sleep(0.001)
                continue

            try:
                rgb_small = self.q.get(timeout=0.5)
            except queue.Empty:
                continue

            last_run = time.time()

            try:
                pil = PILImage.fromarray(rgb_small)
                inputs = self.processor(images=pil, text=self.prompt, return_tensors="pt").to(self.device)

                # 推理：cuda 用 autocast，cpu 不用
                if self.device == "cuda":
                    ctx = torch.autocast("cuda", dtype=torch.float16)
                else:
                    class _NullCtx:
                        def __enter__(self): return None
                        def __exit__(self, *args): return False
                    ctx = _NullCtx()

                with torch.inference_mode(), ctx:
                    outputs = self.model(**inputs)

                res = self.processor.post_process_instance_segmentation(
                    outputs,
                    threshold=0.0,
                    mask_threshold=self.mask_th,
                    target_sizes=inputs.get("original_sizes").tolist()
                )[0]

                masks = res["masks"].detach().cpu().numpy()    # [N,H,W]
                scores = res["scores"].detach().cpu().numpy()  # [N]

                top_mask = None
                if scores.size > 0:
                    keep = scores >= self.score_th
                    if keep.sum() > 0:
                        idx = int(np.argmax(scores * keep))
                        top_mask = masks[idx].astype(bool)

                ov = overlay_mask(rgb_small, top_mask, alpha=self.alpha)

                with self.lock:
                    self.last_overlay = ov

            except Exception as e:
                # 任何异常都不让推理线程“死掉”
                self.get_logger().error(f"Infer loop error: {repr(e)}")
                time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = Sam3Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()