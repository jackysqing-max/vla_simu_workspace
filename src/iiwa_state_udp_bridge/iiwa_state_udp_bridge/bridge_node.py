#!/usr/bin/env python3
import socket
import struct
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class IiwaStateUdpBridge(Node):
    """
    UDP payload layout must match C++:
      struct RobotData { double t; float q[4][7]; };
    We map:
      q[0][:] = joint position (rad)
      q[1][:] = joint velocity (rad/s)
      q[2][:] = joint torque/effort (Nm)  (from JointState.effort or /iiwa7/joint_torques)
      q[3][:] = desired joint position (rad) (from /iiwa7/joint_desired)
    """
    def __init__(self):
        super().__init__('iiwa_state_udp_bridge')

        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('port', 7755)
        self.declare_parameter('rate_hz', 100.0)

        self.ip = self.get_parameter('ip').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # latest cache
        self.pos = [0.0]*7
        self.vel = [0.0]*7
        self.tau = [0.0]*7
        self.qdes = [0.0]*7

        self.sub_js = self.create_subscription(JointState, '/iiwa7/joint_states', self.cb_joint_state, 10)
        self.sub_tau = self.create_subscription(Float64MultiArray, '/iiwa7/joint_torques', self.cb_tau, 10)
        self.sub_qdes = self.create_subscription(Float64MultiArray, '/iiwa7/joint_desired', self.cb_qdes, 10)

        self.timer = self.create_timer(1.0/self.rate_hz, self.tick)

        self.get_logger().info(f"UDP bridge -> {self.ip}:{self.port}, rate={self.rate_hz}Hz")
        self.get_logger().info("Subscribing: /iiwa7/joint_states, /iiwa7/joint_torques, /iiwa7/joint_desired")

    def _take7(self, arr: List[float]) -> List[float]:
        if arr is None:
            return [0.0]*7
        out = list(arr[:7])
        if len(out) < 7:
            out += [0.0]*(7-len(out))
        return out

    def cb_joint_state(self, msg: JointState):
        # 注意：JointState 的顺序可能有 name 排序；如果你的 name 固定且顺序正确，可直接取前7个
        self.pos = self._take7(msg.position)
        self.vel = self._take7(msg.velocity)
        # 如果 JointState.effort 有内容，也可作为 tau 的后备
        eff = self._take7(msg.effort) if len(msg.effort) else None
        if eff is not None:
            self.tau = eff

    def cb_tau(self, msg: Float64MultiArray):
        self.tau = self._take7(msg.data)

    def cb_qdes(self, msg: Float64MultiArray):
        self.qdes = self._take7(msg.data)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        # pack: little-endian '<' : double + 28 floats
        floats = self.pos + self.vel + self.tau + self.qdes  # 4*7 = 28
        payload = struct.pack('<d' + 'f'*28, t, *[float(x) for x in floats])

        try:
            self.sock.sendto(payload, (self.ip, self.port))
        except Exception as e:
            self.get_logger().error(f"UDP send failed: {e}")

def main():
    rclpy.init()
    node = IiwaStateUdpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
