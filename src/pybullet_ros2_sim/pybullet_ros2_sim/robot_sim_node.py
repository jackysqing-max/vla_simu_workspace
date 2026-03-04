#!/usr/bin/env python3
import sys
# 添加PyBullet路径
sys.path.append("/home/siqin/.local/lib/python3.10/site-packages")

# ROS2相关导入
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray  # 机械臂关节指令类型
from nav_msgs.msg import Odometry          # 保留里程计（可选，用于发布关节状态）

# PyBullet相关导入
import pybullet as p
import pybullet_data
import time
import math

class IIWA7Ros2Sim(Node):
    def __init__(self):
        super().__init__('iiwa7_ros2_sim_node')
        
        # 1. 订阅iiwa7关节角度指令话题
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/iiwa7/joint_commands',
            self.joint_cmd_callback,
            10
        )
        
        # 2. 发布iiwa7关节状态
        self.joint_state_pub = self.create_publisher(Float64MultiArray, '/iiwa7/joint_states_array', 10)
        
        # 3. 初始化PyBullet仿真环境
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        
        # 4. 加载地面和KUKA iiwa7机械臂
        self.plane_id = p.loadURDF("plane.urdf")
        # iiwa7初始位姿（基座固定在地面）
        self.iiwa_start_pos = [0, 0, 0.0]
        self.iiwa_start_ori = p.getQuaternionFromEuler([0, 0, 0])
        self.iiwa_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            self.iiwa_start_pos,
            self.iiwa_start_ori,
            useFixedBase=True  # 基座固定，关键！
        )
        
        # 5. 筛选iiwa7的可动关节（旋转关节）
        self.iiwa_joint_indices = []
        for j in range(p.getNumJoints(self.iiwa_id)):
            joint_info = p.getJointInfo(self.iiwa_id, j)
            if joint_info[2] == p.JOINT_REVOLUTE:  # 只保留旋转关节
                self.iiwa_joint_indices.append(j)
        self.get_logger().info(f"iiwa7可动关节数量：{len(self.iiwa_joint_indices)}，索引：{self.iiwa_joint_indices}")
        
        # 6. 初始化目标关节角度（默认全0，竖直初始位姿）
        self.target_joint_angles = [0.0] * len(self.iiwa_joint_indices)
        
        # 7. 仿真循环定时器（240Hz）
        self.timer = self.create_timer(1/240, self.simulation_loop)
        self.get_logger().info("KUKA iiwa7 ROS2仿真节点已启动！")

    def joint_cmd_callback(self, msg):
        """接收ROS2发布的关节角度指令（弧度）"""
        if len(msg.data) == len(self.iiwa_joint_indices):
            self.target_joint_angles = msg.data
        else:
            self.get_logger().warn(f"指令长度错误！期望{len(self.iiwa_joint_indices)}个关节角度，实际{len(msg.data)}个")

    def simulation_loop(self):
        """iiwa7仿真主循环：关节控制+状态发布"""
        # 1. 控制iiwa7关节到目标角度
        for idx, joint_id in enumerate(self.iiwa_joint_indices):
            p.setJointMotorControl2(
                bodyUniqueId=self.iiwa_id,
                jointIndex=joint_id,
                controlMode=p.POSITION_CONTROL,
                targetPosition=self.target_joint_angles[idx],
                force=500  # 关节驱动力，足够带动iiwa7
            )
        
        # 2. 执行物理仿真步骤
        p.stepSimulation()
        
        # 3. 获取当前关节角度，发布到ROS2
        current_joint_angles = []
        for joint_id in self.iiwa_joint_indices:
            joint_state = p.getJointState(self.iiwa_id, joint_id)
            current_joint_angles.append(joint_state[0])  # joint_state[0]是当前关节角度（弧度）
        
        # 构造关节状态消息并发布
        state_msg = Float64MultiArray()
        state_msg.data = current_joint_angles
        self.joint_state_pub.publish(state_msg)

    def destroy_node(self):
        """关闭节点时断开PyBullet"""
        p.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    sim_node = IIWA7Ros2Sim()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

