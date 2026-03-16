#!/usr/bin/env python3
"""Bridge low-level iiwa topics into `robot_control_msgs/RobotState`."""

import rclpy
from rclpy.node import Node
from robot_control_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RobotStateBridge(Node):
    """Publish the monitor-friendly 28-element state vector.

    Layout:
      [q(7), qdot(7), tau(7), q_des(7)]

    `robot_monitor` reads the array by fixed offset, so keeping this contract
    explicit is more important than making the message schema fancy.
    """

    def __init__(self):
        super().__init__("robotstate_bridge")
        self.n = 7

        self.q = [0.0] * self.n
        self.qdot = [0.0] * self.n
        self.tau_feedback = [0.0] * self.n
        self.tau_command = [0.0] * self.n
        self.qdes = [0.0] * self.n
        self.have_tau_command = False

        self.pub = self.create_publisher(RobotState, "robot_states", 10)

        self.create_subscription(JointState, "/iiwa7/joint_states", self.cb_js, 10)
        self.create_subscription(Float64MultiArray, "/iiwa7/joint_desired", self.cb_qdes, 10)
        self.create_subscription(Float64MultiArray, "/iiwa7/joint_torques", self.cb_tau, 10)

        self.declare_parameter("rate_hz", 200.0)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f"RobotState bridge started. Publishing 'robot_states' at {rate_hz} Hz"
        )
        self.get_logger().info("robot_state layout: [q(7), qdot(7), tau(7), q_des(7)]")

    def _take7(self, arr):
        arr = list(arr) if arr is not None else []
        out = arr[:self.n]
        if len(out) < self.n:
            out += [0.0] * (self.n - len(out))
        return out

    def cb_js(self, msg: JointState):
        if msg.position and len(msg.position) >= self.n:
            self.q = self._take7(msg.position)
        if msg.velocity and len(msg.velocity) >= self.n:
            self.qdot = self._take7(msg.velocity)
        if msg.effort and len(msg.effort) >= self.n:
            self.tau_feedback = self._take7(msg.effort)

    def cb_qdes(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            self.qdes = self._take7(msg.data)

    def cb_tau(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            self.tau_command = self._take7(msg.data)
            self.have_tau_command = True

    def tick(self):
        msg = RobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "iiwa7"

        tau = self.tau_command if self.have_tau_command else self.tau_feedback
        msg.robot_state = [float(value) for value in (self.q + self.qdot + tau + self.qdes)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
