#!/usr/bin/env python3
"""Camera-enabled iiwa execution node for the perception loop."""

import threading
import time
from typing import List

import pybullet as p
import pybullet_data
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import Bool, Float64MultiArray, Header, Int8
from tf2_ros import TransformBroadcaster

from pybullet_ros2_sim.camera_tf_utils import view_matrix_to_world_optical_tf
from pybullet_ros2_sim.depth_utils import depth_buffer_to_meters
from pybullet_ros2_sim.pointcloud_utils import depth_rgb_to_xyzrgb
from pybullet_ros2_sim.ros_msg_utils import (
    depth32f_to_imgmsg,
    make_camera_info,
    rgb_to_imgmsg,
    xyzrgb_to_pointcloud2,
)
from pybullet_ros2_sim.sim_camera import SimCameraConfig, SimRGBDCamera


class IiwaPybulletRGBDSim(Node):
    """Run a PyBullet world, publish RGB-D data, and accept the `/iiwa7/*` API.

    This node is the camera-enabled sibling of `iiwa_pybullet_sim_node`. It
    keeps the same public control topics so visual demos can still feed the
    standard monitor and control stack.
    """

    MODE_FREE = 0
    MODE_POSITION = 1
    MODE_TORQUE = 2

    def __init__(self):
        super().__init__("iiwa_pybullet_rgbd_sim_node")

        self._lock = threading.Lock()
        self.cb_timer = ReentrantCallbackGroup()

        self.declare_parameter("gui", True)
        self.declare_parameter("sim_hz", 240.0)

        self.declare_parameter("color_hz", 15.0)
        self.declare_parameter("points_hz", 5.0)
        self.declare_parameter("joint_state_hz", 30.0)

        self.declare_parameter("init_q", [0.0, 0.3, 0.0, -1.2, 0.0, 1.0, 0.0])
        self.declare_parameter("position_force", 200.0)
        self.declare_parameter("tau_limit", 200.0)

        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("cam_fov_y_deg", 58.0)
        self.declare_parameter("cam_near", 0.02)
        self.declare_parameter("cam_far", 3.0)

        self.declare_parameter("cam_target", [0.6, 0.0, 0.05])
        self.declare_parameter("cam_distance", 1.0)
        self.declare_parameter("cam_yaw_deg", 90.0)
        self.declare_parameter("cam_pitch_deg", -45.0)
        self.declare_parameter("cam_roll_deg", 0.0)

        self.gui = bool(self.get_parameter("gui").value)
        self.sim_hz = float(self.get_parameter("sim_hz").value)
        self.color_hz = float(self.get_parameter("color_hz").value)
        self.points_hz = float(self.get_parameter("points_hz").value)
        self.joint_state_hz = float(self.get_parameter("joint_state_hz").value)

        self.position_force = float(self.get_parameter("position_force").value)
        self.tau_limit = float(self.get_parameter("tau_limit").value)
        self.init_q = [float(value) for value in self.get_parameter("init_q").value]

        self.cam_cfg = SimCameraConfig(
            width=int(self.get_parameter("cam_width").value),
            height=int(self.get_parameter("cam_height").value),
            fov_y_deg=float(self.get_parameter("cam_fov_y_deg").value),
            near=float(self.get_parameter("cam_near").value),
            far=float(self.get_parameter("cam_far").value),
            target_pos=tuple(self.get_parameter("cam_target").value),
            distance=float(self.get_parameter("cam_distance").value),
            yaw_deg=float(self.get_parameter("cam_yaw_deg").value),
            pitch_deg=float(self.get_parameter("cam_pitch_deg").value),
            roll_deg=float(self.get_parameter("cam_roll_deg").value),
        )

        qos_img = QoSProfile(depth=1)
        qos_img.reliability = ReliabilityPolicy.BEST_EFFORT
        qos_img.durability = DurabilityPolicy.VOLATILE

        self.pub_color = self.create_publisher(Image, "/sim/camera/color/image_raw", qos_img)
        self.pub_depth = self.create_publisher(
            Image,
            "/sim/camera/aligned_depth_to_color/image_raw",
            qos_img,
        )
        self.pub_info = self.create_publisher(CameraInfo, "/sim/camera/color/camera_info", 10)
        self.pub_points = self.create_publisher(
            PointCloud2,
            "/sim/camera/depth/color/points",
            1,
        )
        self.pub_joint_states = self.create_publisher(JointState, "/iiwa7/joint_states", 10)

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_init_done = self.create_publisher(Bool, "/iiwa7/init_done", qos_latch)

        self.sub_mode = self.create_subscription(Int8, "/iiwa7/control_mode", self.on_mode, 10)
        self.sub_qdes = self.create_subscription(
            Float64MultiArray,
            "/iiwa7/joint_desired",
            self.on_qdes,
            10,
        )
        self.sub_tau = self.create_subscription(
            Float64MultiArray,
            "/iiwa7/joint_torques",
            self.on_tau,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client)
        p.setTimeStep(1.0 / max(self.sim_hz, 1e-6), physicsClientId=self.client)
        p.setRealTimeSimulation(0, physicsClientId=self.client)

        self._load_scene()
        self.camera = SimRGBDCamera(self.client, self.cam_cfg)

        self.latest_rgb = None
        self.latest_depth = None

        self.n_joints = 7
        self.joint_indices = list(range(self.n_joints))
        self.joint_names = [
            p.getJointInfo(self.robot_id, index, physicsClientId=self.client)[1].decode(
                "utf-8"
            )
            for index in self.joint_indices
        ]

        if len(self.init_q) < self.n_joints:
            self.init_q += [0.0] * (self.n_joints - len(self.init_q))
        self.init_q = self.init_q[:self.n_joints]

        self.control_mode = self.MODE_POSITION
        self.q_des = list(self.init_q)
        self.tau_cmd = [0.0] * self.n_joints
        self.have_tau = False
        self._init_done_sent = False

        for joint_index, q_init in zip(self.joint_indices, self.init_q):
            p.resetJointState(
                self.robot_id,
                joint_index,
                q_init,
                physicsClientId=self.client,
            )

        self._running = True
        self.sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.sim_thread.start()

        self.color_timer = self.create_timer(
            1.0 / max(self.color_hz, 1e-6),
            self._publish_rgbd,
            callback_group=self.cb_timer,
        )
        self.points_timer = self.create_timer(
            1.0 / max(self.points_hz, 1e-6),
            self._publish_points,
            callback_group=self.cb_timer,
        )
        self.js_timer = self.create_timer(
            1.0 / max(self.joint_state_hz, 1e-6),
            self._publish_joint_states,
            callback_group=self.cb_timer,
        )

        self._send_init_done_once()
        self.get_logger().info("iiwa_pybullet_rgbd_sim_node started")

    def _load_scene(self):
        p.loadURDF("plane.urdf", physicsClientId=self.client)
        p.loadURDF(
            "table/table.urdf",
            basePosition=[0.6, 0.0, -0.65],
            useFixedBase=True,
            physicsClientId=self.client,
        )

        self.robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            basePosition=[0.0, 0.0, 0.0],
            useFixedBase=True,
            physicsClientId=self.client,
        )

        self.obj_id = p.loadURDF(
            "cube_small.urdf",
            basePosition=[0.65, 0.0, 0.02],
            useFixedBase=False,
            physicsClientId=self.client,
        )
        p.changeVisualShape(
            self.obj_id,
            -1,
            rgbaColor=[1.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )

    def on_mode(self, msg: Int8):
        mode = int(msg.data)
        if mode not in (self.MODE_FREE, self.MODE_POSITION, self.MODE_TORQUE):
            self.get_logger().warning(f"[MODE] Invalid control_mode={mode}, ignore.")
            return
        with self._lock:
            self.control_mode = mode

    def on_qdes(self, msg: Float64MultiArray):
        if len(msg.data) < self.n_joints:
            return
        with self._lock:
            self.q_des = [float(value) for value in msg.data[:self.n_joints]]

    def on_tau(self, msg: Float64MultiArray):
        if len(msg.data) < self.n_joints:
            return
        with self._lock:
            self.tau_cmd = [float(value) for value in msg.data[:self.n_joints]]
            self.have_tau = True

    def _send_init_done_once(self):
        if self._init_done_sent:
            return
        msg = Bool()
        msg.data = True
        self.pub_init_done.publish(msg)
        self._init_done_sent = True

    def _release_motors(self):
        for joint_index in self.joint_indices:
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0,
                physicsClientId=self.client,
            )

    def _clip_tau(self, tau: float) -> float:
        limit = max(self.tau_limit, 0.0)
        if limit <= 0.0:
            return tau
        if tau > limit:
            return limit
        if tau < -limit:
            return -limit
        return tau

    def _sim_loop(self):
        dt = 1.0 / max(self.sim_hz, 1e-6)

        while self._running and rclpy.ok():
            with self._lock:
                mode = self.control_mode
                q_des = list(self.q_des)
                tau_cmd = list(self.tau_cmd)

                if mode == self.MODE_POSITION:
                    p.setJointMotorControlArray(
                        bodyUniqueId=self.robot_id,
                        jointIndices=self.joint_indices,
                        controlMode=p.POSITION_CONTROL,
                        targetPositions=q_des,
                        forces=[self.position_force] * self.n_joints,
                        physicsClientId=self.client,
                    )
                elif mode == self.MODE_TORQUE:
                    self._release_motors()
                    for array_index, joint_index in enumerate(self.joint_indices):
                        p.setJointMotorControl2(
                            self.robot_id,
                            joint_index,
                            controlMode=p.TORQUE_CONTROL,
                            force=self._clip_tau(tau_cmd[array_index]),
                            physicsClientId=self.client,
                        )
                else:
                    self._release_motors()

                p.stepSimulation(physicsClientId=self.client)

            time.sleep(dt)

    def _make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "sim_camera_color_optical_frame"
        return header

    def _publish_camera_tf(self, header: Header):
        translation, quat = view_matrix_to_world_optical_tf(self.camera.view_matrix())

        msg = TransformStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = "world"
        msg.child_frame_id = "sim_camera_color_optical_frame"
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(quat[0])
        msg.transform.rotation.y = float(quat[1])
        msg.transform.rotation.z = float(quat[2])
        msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(msg)

    def _publish_rgbd(self):
        with self._lock:
            rgb, depth_buf, _seg = self.camera.render()

        depth_m = depth_buffer_to_meters(
            depth_buf,
            near=self.cam_cfg.near,
            far=self.cam_cfg.far,
        )

        self.latest_rgb = rgb
        self.latest_depth = depth_m

        header = self._make_header()
        fx, fy, cx, cy = self.camera.intrinsics()

        self.pub_color.publish(rgb_to_imgmsg(rgb, header))
        self.pub_depth.publish(depth32f_to_imgmsg(depth_m, header))
        self.pub_info.publish(
            make_camera_info(
                width=self.cam_cfg.width,
                height=self.cam_cfg.height,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                header=header,
            )
        )
        self._publish_camera_tf(header)

    def _publish_points(self):
        if self.latest_rgb is None or self.latest_depth is None:
            return

        header = self._make_header()
        fx, fy, cx, cy = self.camera.intrinsics()
        xyz, rgb = depth_rgb_to_xyzrgb(self.latest_depth, self.latest_rgb, fx, fy, cx, cy)
        self.pub_points.publish(xyzrgb_to_pointcloud2(xyz, rgb, header))

    def _publish_joint_states(self):
        with self._lock:
            states = p.getJointStates(
                self.robot_id,
                self.joint_indices,
                physicsClientId=self.client,
            )

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.name = list(self.joint_names)
        msg.position = [float(state[0]) for state in states]
        msg.velocity = [float(state[1]) for state in states]
        msg.effort = [float(state[3]) for state in states]
        self.pub_joint_states.publish(msg)

    def destroy_node(self):
        self._running = False
        try:
            self.sim_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            p.disconnect(physicsClientId=self.client)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IiwaPybulletRGBDSim()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
