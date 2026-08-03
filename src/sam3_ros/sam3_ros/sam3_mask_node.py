"""Prompt-driven SAM3 segmentation node for the RGB-D camera stream."""

import queue
import threading
import time
from contextlib import nullcontext

import numpy as np
import rclpy
import torch
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from transformers import Sam3Model, Sam3Processor

from .instance_selection import select_instance_indices


def imgmsg_to_rgb(msg: Image) -> np.ndarray:
    """Convert `sensor_msgs/Image` into an RGB uint8 array."""
    height, width = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, width, 3)
    if msg.encoding.lower() == "bgr8":
        return arr[:, :, ::-1].copy()
    return arr.copy()


def mask_to_imgmsg(mask_u8: np.ndarray, header) -> Image:
    """Convert a uint8 mask with values 0/255 into a `mono8` ROS image."""
    msg = Image()
    msg.header = header
    msg.height, msg.width = mask_u8.shape
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = msg.width
    msg.data = mask_u8.tobytes()
    return msg


def resize_keep_aspect(rgb: np.ndarray, max_side: int):
    """Resize an image so the longest side is no larger than `max_side`."""
    height, width = rgb.shape[:2]
    if max(height, width) <= max_side:
        return rgb, (height, width)
    scale = max_side / float(max(height, width))
    resized_h = int(round(height * scale))
    resized_w = int(round(width * scale))
    return np.array(PILImage.fromarray(rgb).resize((resized_w, resized_h))), (height, width)


class Sam3MaskNode(Node):
    """Run low-rate text-guided segmentation and publish a mask + score."""

    def __init__(self):
        super().__init__("sam3_mask_node")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("prompt", "glasses")
        self.declare_parameter("score_th", 0.3)
        self.declare_parameter("relative_score_th", 0.20)
        self.declare_parameter("max_instances", 16)
        self.declare_parameter("mask_th", 0.5)
        self.declare_parameter("max_side", 640)
        self.declare_parameter("infer_hz", 0.5)
        self.declare_parameter("device", "cpu")
        self.declare_parameter("prompt_topic", "/sam3/prompt")
        self.declare_parameter("active_prompt_topic", "/sam3/active_prompt")
        self.declare_parameter("prompt_republish_sec", 1.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.prompt = self.get_parameter("prompt").value
        self.score_th = float(self.get_parameter("score_th").value)
        self.relative_score_th = max(
            float(self.get_parameter("relative_score_th").value),
            0.0,
        )
        self.max_instances = max(
            int(self.get_parameter("max_instances").value),
            1,
        )
        self.mask_th = float(self.get_parameter("mask_th").value)
        self.max_side = int(self.get_parameter("max_side").value)
        self.infer_hz = float(self.get_parameter("infer_hz").value)
        self.device = str(self.get_parameter("device").value)
        self.prompt_topic = self.get_parameter("prompt_topic").value
        self.active_prompt_topic = self.get_parameter("active_prompt_topic").value
        self.prompt_republish_sec = float(self.get_parameter("prompt_republish_sec").value)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        qos_prompt = QoSProfile(depth=1)
        qos_prompt.reliability = ReliabilityPolicy.RELIABLE
        qos_prompt.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.sub_prompt = self.create_subscription(
            String,
            self.prompt_topic,
            self.on_prompt,
            qos_prompt,
        )
        self.pub_active_prompt = self.create_publisher(String, self.active_prompt_topic, 10)
        self.pub_mask = self.create_publisher(Image, "/sam3/mask", 1)
        self.pub_score = self.create_publisher(Float32, "/sam3/score", 1)

        # The worker always consumes the newest frame only; this keeps slow model
        # inference from building up stale backlog.
        self.q = queue.Queue(maxsize=1)
        self._last_status_log_time = 0.0
        self._last_prompt_broadcast = ""

        if self.device == "cuda" and not torch.cuda.is_available():
            self.get_logger().warning(
                "device='cuda' requested but torch.cuda.is_available() is False; "
                "falling back to CPU"
            )
            self.device = "cpu"

        self.get_logger().info(f"Loading SAM3 on {self.device} ...")
        self.model = Sam3Model.from_pretrained("facebook/sam3").to(self.device)
        self.processor = Sam3Processor.from_pretrained("facebook/sam3")
        self.model.eval()

        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        torch.backends.cudnn.benchmark = True

        self.worker = threading.Thread(target=self.infer_loop, daemon=True)
        self.worker.start()
        if self.prompt_republish_sec > 0.0:
            self.prompt_timer = self.create_timer(
                self.prompt_republish_sec,
                self.publish_prompt_if_needed,
            )
        self.publish_prompt(force=True)

        self.get_logger().info(
            f"Subscribed: {self.image_topic} + {self.prompt_topic} | "
            f"Publishing: /sam3/mask + /sam3/score + {self.active_prompt_topic} | "
            f"infer_hz={self.infer_hz}"
        )

    def on_prompt(self, msg: String):
        new_prompt = msg.data.strip()
        if not new_prompt or new_prompt == self.prompt:
            return
        self.prompt = new_prompt
        self.get_logger().info(f"Updated prompt: {self.prompt}")
        self.publish_prompt(force=True)

    def publish_prompt(self, *, force: bool = False):
        prompt = str(self.prompt).strip()
        if not prompt:
            return
        if not force and prompt == self._last_prompt_broadcast:
            return
        msg = String()
        msg.data = prompt
        self.pub_active_prompt.publish(msg)
        self._last_prompt_broadcast = prompt

    def publish_prompt_if_needed(self):
        self.publish_prompt(force=False)

    def on_image(self, msg: Image):
        rgb = imgmsg_to_rgb(msg)

        if self.q.full():
            try:
                self.q.get_nowait()
            except Exception:
                pass
        self.q.put_nowait((rgb, msg.header))

    def _log_status(self, message: str):
        now = time.time()
        if now - self._last_status_log_time < 2.0:
            return
        self._last_status_log_time = now
        self.get_logger().info(message)

    def infer_loop(self):
        period = 1.0 / max(self.infer_hz, 1e-6)
        last_run = 0.0

        while rclpy.ok():
            now = time.time()
            if now - last_run < period:
                time.sleep(0.001)
                continue

            try:
                rgb, header = self.q.get(timeout=0.5)
            except queue.Empty:
                continue

            last_run = time.time()

            try:
                inference_prompt = str(self.prompt)
                rgb_small, (orig_h, orig_w) = resize_keep_aspect(rgb, self.max_side)
                pil_image = PILImage.fromarray(rgb_small)
                inputs = self.processor(
                    images=pil_image,
                    text=inference_prompt,
                    return_tensors="pt",
                ).to(self.device)

                amp_context = (
                    torch.autocast("cuda", dtype=torch.float16)
                    if self.device == "cuda"
                    else nullcontext()
                )

                with torch.inference_mode(), amp_context:
                    outputs = self.model(**inputs)

                result = self.processor.post_process_instance_segmentation(
                    outputs,
                    threshold=0.0,
                    mask_threshold=self.mask_th,
                    target_sizes=inputs.get("original_sizes").tolist(),
                )[0]

                # A prompt can change while GPU inference is in flight.  Never
                # publish an old-prompt mask into the new language request.
                if inference_prompt != self.prompt:
                    self.get_logger().info(
                        "Discarded stale inference result: "
                        f"prompt={inference_prompt!r} active={self.prompt!r}"
                    )
                    continue

                masks = result["masks"].detach().cpu().numpy()
                scores = result["scores"].detach().cpu().numpy()

                top_mask = None
                top_score = 0.0
                kept_count = 0
                best_score = float(np.max(scores)) if scores.size > 0 else 0.0

                if scores.size > 0:
                    kept_indices = select_instance_indices(
                        scores,
                        absolute_threshold=self.score_th,
                        relative_threshold=self.relative_score_th,
                        max_instances=self.max_instances,
                    )
                    if kept_indices.size > 0:
                        kept_count = int(kept_indices.size)
                        kept_masks = masks[kept_indices] > 0
                        top_mask = np.any(kept_masks, axis=0).astype(np.uint8) * 255
                        top_score = float(np.max(scores[kept_indices]))

                if top_mask is None:
                    top_mask = np.zeros(
                        (rgb_small.shape[0], rgb_small.shape[1]),
                        dtype=np.uint8,
                    )
                    self._log_status(
                        f"prompt='{inference_prompt}' no mask above threshold "
                        f"(best_score={best_score:.3f}, score_th={self.score_th:.3f})"
                    )
                else:
                    mask_area = int(np.count_nonzero(top_mask))
                    self._log_status(
                        f"prompt='{inference_prompt}' union_masks={kept_count} "
                        f"top_score={top_score:.3f} area={mask_area}"
                    )

                if (rgb_small.shape[0], rgb_small.shape[1]) != (orig_h, orig_w):
                    top_mask = np.array(
                        PILImage.fromarray(top_mask).resize(
                            (orig_w, orig_h),
                            resample=PILImage.NEAREST,
                        )
                    )

                score_msg = Float32()
                # Publish the actual best score even when it is below the mask
                # threshold.  Consumers can then explain why no mask was emitted.
                score_msg.data = best_score
                self.pub_score.publish(score_msg)
                self.pub_mask.publish(mask_to_imgmsg(top_mask, header))

            except Exception as exc:
                self.get_logger().error(f"Infer error: {exc!r}")
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
