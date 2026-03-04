#!/usr/bin/env python3
import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class MonitorUdpSender(Node):
    """
    Send RobotData (double t + float q[4][7]) to robot_monitor via UDP.
    Layout must match C++ exactly: 8 bytes + 28*4 bytes = 120 bytes.
    """
    def __init__(self):
        super().__init__('monitor_udp_sender')

        self.declare_parameter('ip', '127.0.0.1')
        self.declare_parameter('port', 7755)
        self.declare_parameter('rate_hz', 100.0)

        self.ip = self.get_parameter('ip').value
        self.port = int(self.get_parameter('port').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.n = 7
        self.q = [0.0]*self.n
        self.qdot = [0.0]*self.n
        self.tau = [0.0]*self.n      # prefer /iiwa7/joint_torques
        self.q_des = [0.0]*self.n

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sub_js = self.create_subscription(JointState, '/iiwa7/joint_states', self.cb_js, 10)
        self.sub_tau = self.create_subscription(Float64MultiArray, '/iiwa7/joint_torques', self.cb_tau, 10)
        self.sub_qd = self.create_subscription(Float64MultiArray, '/iiwa7/joint_desired', self.cb_qd, 10)

        self.timer = self.create_timer(1.0/self.rate_hz, self.tick)

        self.get_logger().info(f"Sending RobotData UDP -> {self.ip}:{self.port} @ {self.rate_hz} Hz")

    def _take7(self, arr):
        arr = list(arr) if arr is not None else []
        out = arr[:self.n]
        if len(out) < self.n:
            out += [0.0]*(self.n-len(out))
        return out

    def cb_js(self, msg: JointState):
        if len(msg.position) >= self.n:
            self.q = self._take7(msg.position)
        if len(msg.velocity) >= self.n:
            self.qdot = self._take7(msg.velocity)
        # effort 作为 tau 的后备（如果你不发 /iiwa7/joint_torques 也能看到）
        if len(msg.effort) >= self.n:
            # 只有当外部没发 tau 时，你才可以用 effort 覆盖；这里简单示例不覆盖
            pass

    def cb_tau(self, msg: Float64MultiArray):
        if len(msg.data) >= self.n:
            self.tau = self._take7(msg.data)

    def cb_qd(self, msg: Float64MultiArray):
        if len(msg.data) >= self.n:
            self.q_des = self._take7(msg.data)

    def tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9  # double seconds

        # q[4][7] flatten: group0(7) + group1(7) + group2(7) + group3(7)
        floats = self.q + self.qdot + self.tau + self.q_des  # 28 floats

        # little-endian: '<' ; double + 28 floats
        payload = struct.pack('<d' + 'f'*28, float(t), *[float(x) for x in floats])

        # send
        self.sock.sendto(payload, (self.ip, self.port))

def main():
    rclpy.init()
    node = MonitorUdpSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
