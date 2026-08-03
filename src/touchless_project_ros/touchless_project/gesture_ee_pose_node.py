"""Drive the simulated iiwa end-effector pose from touchless gestures."""

from __future__ import annotations

import json
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int8, String

from pybullet_ros2_sim.ik_utils import IiwaIkHelper
from touchless_project.ee_pose_mapping import EeGestureMapper, EeGestureMappingConfig


def _clamp_joint_step(q_now, q_des, max_step):
    out = []
    limit = abs(float(max_step))
    for current, desired in zip(q_now, q_des):
        delta = float(desired) - float(current)
        delta = max(-limit, min(limit, delta))
        out.append(float(current) + delta)
    return out


class GestureEePoseNode(Node):
    """Map gesture commands to Cartesian target pose and publish IK joint targets."""

    def __init__(self):
        super().__init__("gesture_ee_pose_node")

        self.n = 7
        self.lock = threading.Lock()

        self.declare_parameter("command_topic", "/touchless/command_json")
        self.declare_parameter("joint_states_topic", "/iiwa7/joint_states")
        self.declare_parameter("joint_desired_topic", "/iiwa7/joint_desired")
        self.declare_parameter("control_mode_topic", "/iiwa7/control_mode")
        self.declare_parameter("target_pose_topic", "/touchless/ee_target_pose")
        self.declare_parameter("init_done_topic", "/iiwa7/init_done")
        self.declare_parameter("wait_for_init_done", True)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("control_mode_value", 1)
        self.declare_parameter("max_joint_step_rad", 0.02)
        self.declare_parameter("pan_y_m_per_px", 0.0015)
        self.declare_parameter("pan_z_m_per_px", 0.0015)
        self.declare_parameter("zoom_x_m_per_px", 0.0015)
        self.declare_parameter("rotate_pitch_rad_per_px", 0.004)
        self.declare_parameter("rotate_yaw_rad_per_px", 0.004)
        self.declare_parameter("max_position_offset_xyz_m", [0.30, 0.30, 0.30])
        self.declare_parameter("max_orientation_offset_rpy_rad", [0.90, 0.90, 0.90])
        self.declare_parameter("orientation_reference_frame", "world")
        self.declare_parameter("reset_on_off_mode", False)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.joint_desired_topic = str(self.get_parameter("joint_desired_topic").value)
        self.control_mode_topic = str(self.get_parameter("control_mode_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.init_done_topic = str(self.get_parameter("init_done_topic").value)
        self.wait_for_init_done = bool(self.get_parameter("wait_for_init_done").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.control_mode_value = int(self.get_parameter("control_mode_value").value)
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.reset_on_off_mode = bool(self.get_parameter("reset_on_off_mode").value)

        self.mapping_config = EeGestureMappingConfig(
            pan_y_m_per_px=float(self.get_parameter("pan_y_m_per_px").value),
            pan_z_m_per_px=float(self.get_parameter("pan_z_m_per_px").value),
            zoom_x_m_per_px=float(self.get_parameter("zoom_x_m_per_px").value),
            rotate_pitch_rad_per_px=float(
                self.get_parameter("rotate_pitch_rad_per_px").value
            ),
            rotate_yaw_rad_per_px=float(self.get_parameter("rotate_yaw_rad_per_px").value),
            max_position_offset_xyz_m=self._three_float_tuple(
                self.get_parameter("max_position_offset_xyz_m").value,
                "max_position_offset_xyz_m",
            ),
            max_orientation_offset_rpy_rad=self._three_float_tuple(
                self.get_parameter("max_orientation_offset_rpy_rad").value,
                "max_orientation_offset_rpy_rad",
            ),
            orientation_reference_frame=str(
                self.get_parameter("orientation_reference_frame").value
            ),
        )

        self.ik = IiwaIkHelper()
        self.q_now = None
        self.q_seed = None
        self.init_done = not self.wait_for_init_done
        self.mapper = None
        self.latest_target = None
        self.last_command_summary = "none"

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, self.init_done_topic, self.on_init_done, qos_latch)
        self.create_subscription(JointState, self.joint_states_topic, self.on_joint_states, 10)
        self.create_subscription(String, self.command_topic, self.on_command, 10)

        self.pub_control_mode = self.create_publisher(Int8, self.control_mode_topic, 10)
        self.pub_joint_desired = self.create_publisher(
            Float64MultiArray,
            self.joint_desired_topic,
            10,
        )
        self.pub_target_pose = self.create_publisher(PoseStamped, self.target_pose_topic, 10)

        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            "gesture_ee_pose_node started. "
            "Mapping: PAN dx/dy -> EE Y/Z, ZOOM -> EE X, ROTATE dx/dy -> yaw/pitch."
        )

    def _three_float_tuple(self, raw_value, parameter_name: str):
        values = [float(value) for value in raw_value]
        if len(values) != 3:
            raise ValueError(f"{parameter_name} must contain exactly 3 values")
        return tuple(values)

    def on_init_done(self, msg: Bool):
        with self.lock:
            self.init_done = bool(msg.data)

    def on_joint_states(self, msg: JointState):
        if len(msg.position) < self.n:
            return

        q_now = [float(value) for value in msg.position[: self.n]]
        with self.lock:
            self.q_now = q_now
            should_lock_initial = self.init_done and self.mapper is None

        if should_lock_initial:
            position, orientation = self.ik.fk(q_now)
            mapper = EeGestureMapper(position, orientation, self.mapping_config)
            target = mapper.target()
            with self.lock:
                if self.mapper is None:
                    self.mapper = mapper
                    self.latest_target = target
                    self.q_seed = q_now[:]
            self.get_logger().info(
                "[INIT] Locked EE pose "
                f"p=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
                f"q=({orientation[0]:.3f}, {orientation[1]:.3f}, "
                f"{orientation[2]:.3f}, {orientation[3]:.3f})"
            )

    def on_command(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring malformed gesture command JSON")
            return

        with self.lock:
            mapper = self.mapper
            if mapper is None:
                return
            mode = str(payload.get("mode", "")).upper()
            if mode == "OFF" and self.reset_on_off_mode:
                target = mapper.reset_offsets()
            else:
                target = mapper.apply_command(payload)
            self.latest_target = target
            self.last_command_summary = self._command_summary(payload, target)

    def _command_summary(self, payload: dict, target) -> str:
        action = str(payload.get("action", "")).upper() or "MODE"
        return (
            f"{action}: offset_xyz=({target.position_offset[0]:+.3f}, "
            f"{target.position_offset[1]:+.3f}, {target.position_offset[2]:+.3f}), "
            f"offset_rpy=({target.orientation_offset_rpy[0]:+.3f}, "
            f"{target.orientation_offset_rpy[1]:+.3f}, "
            f"{target.orientation_offset_rpy[2]:+.3f})"
        )

    def on_timer(self):
        with self.lock:
            q_now = None if self.q_now is None else list(self.q_now)
            q_seed = None if self.q_seed is None else list(self.q_seed)
            target = self.latest_target

        if q_now is None or q_seed is None or target is None:
            return

        q_des = self.ik.solve_ik(q_seed, target.position, target.orientation)
        q_des = _clamp_joint_step(q_now, q_des, self.max_joint_step_rad)

        with self.lock:
            self.q_seed = q_des[:]

        self._publish_control_mode()
        self._publish_joint_target(q_des)
        self._publish_target_pose(target)

    def _publish_control_mode(self):
        msg = Int8()
        msg.data = self.control_mode_value
        self.pub_control_mode.publish(msg)

    def _publish_joint_target(self, q_des):
        msg = Float64MultiArray()
        msg.data = [float(value) for value in q_des[: self.n]]
        self.pub_joint_desired.publish(msg)

    def _publish_target_pose(self, target):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(target.position[0])
        msg.pose.position.y = float(target.position[1])
        msg.pose.position.z = float(target.position[2])
        msg.pose.orientation.x = float(target.orientation[0])
        msg.pose.orientation.y = float(target.orientation[1])
        msg.pose.orientation.z = float(target.orientation[2])
        msg.pose.orientation.w = float(target.orientation[3])
        self.pub_target_pose.publish(msg)

    def destroy_node(self):
        try:
            self.ik.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureEePoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
