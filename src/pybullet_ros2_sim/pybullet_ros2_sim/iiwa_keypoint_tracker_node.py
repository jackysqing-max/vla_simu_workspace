#!/usr/bin/env python3
"""Convert perception targets into joint-space commands for the iiwa arm."""

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int8
from tf2_ros import Buffer, TransformListener

from pybullet_ros2_sim.ik_utils import IiwaIkHelper


def clamp_step(q_now, q_des, max_step):
    """Clamp each joint update so IK output remains continuous."""
    out = []
    for current, desired in zip(q_now, q_des):
        delta = desired - current
        delta = max(-max_step, min(max_step, delta))
        out.append(current + delta)
    return out


def transform_to_matrix(tf_msg):
    """Convert a TF message into a homogeneous 4x4 transform matrix."""
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


def transform_point(transform, point_xyz):
    """Apply a homogeneous transform to a 3D point."""
    point = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0], dtype=np.float64)
    transformed = transform @ point
    return transformed[:3]


class IiwaKeypointTracker(Node):
    """Track a perceived 3D keypoint and publish hover pose joint targets."""

    def __init__(self):
        super().__init__("iiwa_keypoint_tracker_node")

        self.lock = threading.Lock()
        self.n = 7

        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("hover_offset_z", 0.10)
        self.declare_parameter("max_joint_step_rad", 0.02)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("keep_initial_orientation", True)
        self.declare_parameter("control_mode_value", 1)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("enable_topic", "/llm_task/tracking_enabled")
        self.declare_parameter("start_enabled", True)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.hover_offset_z = float(self.get_parameter("hover_offset_z").value)
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.keep_initial_orientation = bool(
            self.get_parameter("keep_initial_orientation").value
        )
        self.control_mode_value = int(self.get_parameter("control_mode_value").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.tracking_enabled = bool(self.get_parameter("start_enabled").value)

        self.sub_keypoint = self.create_subscription(
            PointStamped,
            "/perception/keypoint_3d",
            self.on_keypoint,
            10,
        )
        self.sub_valid = self.create_subscription(Bool, "/perception/valid", self.on_valid, 10)
        self.sub_js = self.create_subscription(JointState, "/iiwa7/joint_states", self.on_js, 10)
        self.sub_enable = self.create_subscription(Bool, self.enable_topic, self.on_enable, 10)

        self.pub_mode = self.create_publisher(Int8, "/iiwa7/control_mode", 10)
        self.pub_qdes = self.create_publisher(Float64MultiArray, "/iiwa7/joint_desired", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = IiwaIkHelper()

        self.q_now = None
        self.target_cam = None
        self.target_cam_header = None
        self.target_valid = False

        self.have_initial_ee = False
        self.ee_orn0 = None

        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            "iiwa_keypoint_tracker_node started "
            f"(tracking_enabled={self.tracking_enabled}, enable_topic={self.enable_topic})"
        )

    def on_enable(self, msg: Bool):
        with self.lock:
            self.tracking_enabled = bool(msg.data)

    def on_valid(self, msg: Bool):
        with self.lock:
            self.target_valid = bool(msg.data)

    def on_keypoint(self, msg: PointStamped):
        with self.lock:
            self.target_cam = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
            self.target_cam_header = msg.header

    def on_js(self, msg: JointState):
        if len(msg.position) < self.n:
            return

        q_now = list(msg.position[:self.n])
        with self.lock:
            self.q_now = q_now

        # Preserve the initial end-effector orientation so the tracker only
        # translates the arm unless the user explicitly disables this behavior.
        if not self.have_initial_ee:
            _pos, orn = self.ik.fk(q_now)
            self.ee_orn0 = orn
            self.have_initial_ee = True

    def _publish_position_mode(self):
        msg = Int8()
        msg.data = self.control_mode_value
        self.pub_mode.publish(msg)

    def _target_is_fresh(self, header) -> bool:
        if self.target_timeout_sec <= 0.0:
            return True
        if header is None:
            return False
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return True
        age = (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9
        return age <= self.target_timeout_sec

    def on_timer(self):
        with self.lock:
            q_now = None if self.q_now is None else list(self.q_now)
            target_valid = self.target_valid
            target_cam = None if self.target_cam is None else np.array(self.target_cam, copy=True)
            header = self.target_cam_header
            tracking_enabled = self.tracking_enabled

        if not tracking_enabled:
            return
        if q_now is None or target_cam is None or header is None:
            return
        if not target_valid:
            return
        if not self._target_is_fresh(header):
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(self.target_frame, header.frame_id, Time())
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        transform = transform_to_matrix(tf_msg)
        point_world = transform_point(transform, target_cam)

        # Track a hover point above the detected keypoint so perception jitter
        # does not immediately drive the end-effector into the object.
        hover_point = np.array(
            [
                point_world[0],
                point_world[1],
                point_world[2] + self.hover_offset_z,
            ],
            dtype=np.float64,
        )

        if self.keep_initial_orientation and self.ee_orn0 is not None:
            q_des = self.ik.solve_ik(q_now, hover_point.tolist(), self.ee_orn0)
        else:
            q_des = self.ik.solve_ik(q_now, hover_point.tolist(), None)

        q_des = clamp_step(q_now, q_des, self.max_joint_step_rad)

        self._publish_position_mode()

        msg = Float64MultiArray()
        msg.data = q_des
        self.pub_qdes.publish(msg)

    def destroy_node(self):
        try:
            self.ik.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IiwaKeypointTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
