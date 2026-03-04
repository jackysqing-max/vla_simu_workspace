import threading, queue, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import torch
from PIL import Image as PILImage
from transformers import Sam3Model, Sam3Processor


def imgmsg_to_rgb(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return arr[:, :, ::-1].copy()
    return arr.copy()


def mask_to_imgmsg(mask_u8: np.ndarray, header) -> Image:
    """mask_u8: HxW uint8, values 0/255 -> mono8 Image"""
    out = Image()
    out.header = header
    out.height, out.width = mask_u8.shape
    out.encoding = "mono8"
    out.is_bigendian = 0
    out.step = out.width
    out.data = mask_u8.tobytes()
    return out


def resize_keep_aspect(rgb: np.ndarray, max_side: int):
    """返回 resized_rgb, (orig_h, orig_w)"""
    h, w = rgb.shape[:2]
    if max(h, w) <= max_side:
        return rgb, (h, w)
    scale = max_side / float(max(h, w))
    nh, nw = int(round(h * scale)), int(round(w * scale))
    small = np.array(PILImage.fromarray(rgb).resize((nw, nh)))
    return small, (h, w)


class Sam3MaskNode(Node):
    def __init__(self):
        super().__init__("sam3_mask_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("prompt", "glasses")
        self.declare_parameter("score_th", 0.3)
        self.declare_parameter("mask_th", 0.5)
        self.declare_parameter("max_side", 640)
        self.declare_parameter("infer_hz", 0.5)
        self.declare_parameter("device", "cpu")  # 你现在 RTX5080 torch 不兼容，先 cpu

        self.image_topic = self.get_parameter("image_topic").value
        self.prompt = self.get_parameter("prompt").value
        self.score_th = float(self.get_parameter("score_th").value)
        self.mask_th = float(self.get_parameter("mask_th").value)
        self.max_side = int(self.get_parameter("max_side").value)
        self.infer_hz = float(self.get_parameter("infer_hz").value)
        self.device = self.get_parameter("device").value

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.pub_mask = self.create_publisher(Image, "/sam3/mask", 1)
        self.pub_score = self.create_publisher(Float32, "/sam3/score", 1)

        self.q = queue.Queue(maxsize=1)
        self.last_header = None

        self.get_logger().info(f"Loading SAM3 on {self.device} ...")
        self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
        self.processor = Sam3Processor.from_pretrained("facebook/sam3")
        self.model.eval()

        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        torch.backends.cudnn.benchmark = True

        self.worker = threading.Thread(target=self.infer_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(f"Subscribed: {self.image_topic} | Publishing: /sam3/mask (/sam3/score) | infer_hz={self.infer_hz}")

    def on_image(self, msg: Image):
        rgb = imgmsg_to_rgb(msg)
        self.last_header = msg.header

        if self.q.full():
            try:
                _ = self.q.get_nowait()
            except Exception:
                pass
        self.q.put_nowait(rgb)

    def infer_loop(self):
        period = 1.0 / max(self.infer_hz, 1e-6)
        last_run = 0.0

        while rclpy.ok():
            now = time.time()
            if now - last_run < period:
                time.sleep(0.001)
                continue

            try:
                rgb = self.q.get(timeout=0.5)
            except queue.Empty:
                continue

            last_run = time.time()

            try:
                rgb_small, (oh, ow) = resize_keep_aspect(rgb, self.max_side)
                pil = PILImage.fromarray(rgb_small)
                inputs = self.processor(images=pil, text=self.prompt, return_tensors="pt").to(self.device)

                # CPU 不用 autocast；CUDA 用 autocast（但你现在建议先 cpu）
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
                top_score = 0.0

                if scores.size > 0:
                    keep = scores >= self.score_th
                    if keep.sum() > 0:
                        idx = int(np.argmax(scores * keep))
                        top_mask = masks[idx].astype(np.uint8) * 255
                        top_score = float(scores[idx])

                if top_mask is None:
                    # 发布空mask（全0），overlay节点会显示原图
                    top_mask = np.zeros((rgb_small.shape[0], rgb_small.shape[1]), dtype=np.uint8)

                # 如果推理时缩放过，mask 用最近邻 resize 回原始尺寸，确保与 /image_raw 尺寸一致
                if (rgb_small.shape[0], rgb_small.shape[1]) != (oh, ow):
                    top_mask = np.array(PILImage.fromarray(top_mask).resize((ow, oh), resample=PILImage.NEAREST))

                hdr = self.last_header
                if hdr is not None:
                    self.pub_mask.publish(mask_to_imgmsg(top_mask, hdr))
                    s = Float32()
                    s.data = top_score
                    self.pub_score.publish(s)

            except Exception as e:
                self.get_logger().error(f"Infer error: {repr(e)}")
                time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = Sam3MaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()