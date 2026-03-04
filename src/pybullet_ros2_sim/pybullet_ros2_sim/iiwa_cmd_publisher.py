#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import pybullet as p
import pybullet_data


class IIWACommandPublisher(Node):
    """
    Cartesian circular motion (end-effector circle) -> IK -> joint commands.
    Uses a separate PyBullet DIRECT client only for IK computation.
    """

    def __init__(self):
        super().__init__('iiwa_cmd_publisher')

        self.cmd_publisher = self.create_publisher(
            Float64MultiArray,
            '/iiwa7/joint_commands',
            10
        )

        # ------- parameters -------
        self.publish_hz = 200.0          # IK + 发布频率，建议 50~200
        self.motion_hz = 0.1             # 10秒一圈
        self.iiwa_joint_num = 7

        # 圆心与半径（单位：米，坐标在 base frame）
        self.cx, self.cy, self.cz = 0.55, 0.0, 0.45
        self.radius = 0.08               # 8 cm
        # --------------------------

        # PyBullet IK client (DIRECT)
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # 常见 iiwa URDF：pybullet_data 里通常有 kuka_iiwa/model.urdf
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

        # 末端 link index：通常最后一个关节对应的 link（iiwa 7dof 常为 6）
        # 保险起见：取最后一个关节的 link index
        self.ee_link = self.iiwa_joint_num - 1

        # 固定末端姿态（可按需调整）
        self.target_orn = p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info("IIWA Cartesian circle publisher (IK) started.")
        self.get_logger().info(f"publish_hz={self.publish_hz}, motion_hz={self.motion_hz}")
        self.get_logger().info(f"center=({self.cx},{self.cy},{self.cz}), r={self.radius}, ee_link={self.ee_link}")

    def on_timer(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        w = 2.0 * math.pi * self.motion_hz

        # 圆：xy 平面
        x = self.cx + self.radius * math.cos(w * t)
        y = self.cy + self.radius * math.sin(w * t)
        z = self.cz

        # IK 求解
        q_full = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link,
            targetPosition=[x, y, z],
            targetOrientation=self.target_orn
        )

        # 取前 7 个关节
        q = list(q_full[:self.iiwa_joint_num])

        msg = Float64MultiArray()
        msg.data = q
        self.cmd_publisher.publish(msg)

    def destroy_node(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IIWACommandPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

