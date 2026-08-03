"""ROS 2 node that publishes gesture commands from a webcam feed."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from touchless_project.dependencies import import_dependency
from touchless_project.gesture_core import GestureInterpreter, make_config


class TouchlessGestureNode(Node):
    """Detect hand gestures and publish them as ROS messages."""

    def __init__(self):
        super().__init__("touchless_gesture_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("command_topic", "/touchless/command_json")
        self.declare_parameter("mode_topic", "/touchless/mode")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("sensitivity", 1.0)
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("publish_idle_frames", False)
        self.declare_parameter("max_num_hands", 1)
        self.declare_parameter("min_detection_confidence", 0.5)
        self.declare_parameter("min_tracking_confidence", 0.5)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.sensitivity = float(self.get_parameter("sensitivity").value)
        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.publish_idle_frames = bool(self.get_parameter("publish_idle_frames").value)
        self.max_num_hands = int(self.get_parameter("max_num_hands").value)
        self.min_detection_confidence = float(
            self.get_parameter("min_detection_confidence").value
        )
        self.min_tracking_confidence = float(
            self.get_parameter("min_tracking_confidence").value
        )

        self.cv2 = import_dependency("cv2", "opencv-python")
        self.mediapipe = import_dependency("mediapipe", "mediapipe")
        self.mp_hands = self.mediapipe.solutions.hands
        self.mp_drawing = self.mediapipe.solutions.drawing_utils

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)

        self.interpreter = GestureInterpreter(make_config(self.sensitivity))
        self.last_mode = None
        self.last_capture_warn_ns = 0

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.max_num_hands,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )

        self.capture = self.cv2.VideoCapture(self.camera_index)
        if not self.capture.isOpened():
            raise RuntimeError(f"Unable to open camera index {self.camera_index}")

        self.timer = self.create_timer(1.0 / max(self.control_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            "touchless_gesture_node started "
            f"(camera_index={self.camera_index}, command_topic={self.command_topic})"
        )

    def on_timer(self) -> None:
        ok, frame = self.capture.read()
        if not ok:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_capture_warn_ns > int(2e9):
                self.get_logger().warning("Camera frame capture failed")
                self.last_capture_warn_ns = now_ns
            return

        frame = self.cv2.flip(frame, 1)
        height, width, _ = frame.shape

        rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        result = self.hands.process(rgb)

        landmarks = None
        if result.multi_hand_landmarks:
            landmarks = result.multi_hand_landmarks[0].landmark

        gesture_frame = self.interpreter.process_landmarks(landmarks, width, height)
        stamp_ns = self.get_clock().now().nanoseconds

        if gesture_frame.mode_changed or gesture_frame.mode != self.last_mode:
            mode_msg = String()
            mode_msg.data = gesture_frame.mode
            self.mode_pub.publish(mode_msg)
            self.last_mode = gesture_frame.mode

        if (
            gesture_frame.action
            or gesture_frame.mode_changed
            or self.publish_idle_frames
        ):
            payload = gesture_frame.to_payload(stamp_ns)
            cmd_msg = String()
            cmd_msg.data = json.dumps(payload)
            self.command_pub.publish(cmd_msg)

        if self.show_debug_window:
            self._draw_debug_frame(frame, result, gesture_frame)
            self.cv2.imshow("Touchless Gesture Node", frame)
            if self.cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("Shutdown requested from debug window")
                rclpy.shutdown()

    def _draw_debug_frame(self, frame, result, gesture_frame) -> None:
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                )

        point_color = (0, 0, 255)
        if gesture_frame.mode != "OFF":
            point_color = (0, 255, 0)

        for point in (
            gesture_frame.index_px,
            gesture_frame.middle_px,
            gesture_frame.thumb_px,
        ):
            if point:
                self.cv2.circle(frame, point, 8, point_color, -1)

        lines = [
            f"MODE: {gesture_frame.mode} {gesture_frame.action}",
            f"z_i={gesture_frame.z_index:.3f} z_m={gesture_frame.z_middle:.3f}",
            f"pinch={gesture_frame.pinch_distance:.1f} dp={gesture_frame.delta_pinch:.1f}",
        ]
        y = 30
        for line in lines:
            self.cv2.putText(
                frame,
                line,
                (10, y),
                self.cv2.FONT_HERSHEY_SIMPLEX,
                0.7 if y == 30 else 0.6,
                (0, 255, 255),
                2 if y == 30 else 1,
            )
            y += 28

    def destroy_node(self):
        if hasattr(self, "capture") and self.capture is not None:
            self.capture.release()
        if hasattr(self, "hands") and self.hands is not None:
            self.hands.close()
        if getattr(self, "show_debug_window", False):
            self.cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TouchlessGestureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
