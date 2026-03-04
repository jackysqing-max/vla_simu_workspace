#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from robot_control_msgs.msg import RobotState

class RobotStateBridge(Node):
    """
    Publish robot_control_msgs/RobotState on topic 'robot_states'.
    RobotState.robot_state is float64[] with length 28:
      [q(7), qdot(7), tau(7), q_des(7)]
    Here tau comes from JointState.effort (PyBullet motor torque under POSITION_CONTROL).
    """
    def __init__(self):
        super().__init__('robotstate_bridge')
        self.n = 7

        self.q = [0.0]*self.n
        self.qdot = [0.0]*self.n
        self.tau = [0.0]*self.n
        self.qdes = [0.0]*self.n

        self.pub = self.create_publisher(RobotState, 'robot_states', 10)

        self.create_subscription(JointState, '/iiwa7/joint_states', self.cb_js, 10)
        self.create_subscription(Float64MultiArray, '/iiwa7/joint_desired', self.cb_qdes, 10)

        self.declare_parameter('rate_hz', 200.0)
        rate_hz = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0/rate_hz, self.tick)

        self.get_logger().info(f"RobotState bridge started. Publishing 'robot_states' at {rate_hz} Hz")
        self.get_logger().info("robot_state layout: [q(7), qdot(7), tau(7), q_des(7)]")

    def _take7(self, arr):
        arr = list(arr) if arr is not None else []
        out = arr[:self.n]
        if len(out) < self.n:
            out += [0.0]*(self.n-len(out))
        return out

    def cb_js(self, msg: JointState):
        if msg.position and len(msg.position) >= self.n:
            self.q = self._take7(msg.position)
        if msg.velocity and len(msg.velocity) >= self.n:
            self.qdot = self._take7(msg.velocity)

        if msg.effort and len(msg.effort) >= self.n:
            self.tau = self._take7(msg.effort)

    def cb_qdes(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            self.qdes = self._take7(msg.data)


    def tick(self):
        m = RobotState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "iiwa7"
        err = [float(self.qdes[i] - self.q[i]) for i in range(self.n)]
        m.robot_state = [float(x) for x in (self.q + err + self.tau + self.qdes)]
        self.pub.publish(m)

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

if __name__ == '__main__':
    main()
