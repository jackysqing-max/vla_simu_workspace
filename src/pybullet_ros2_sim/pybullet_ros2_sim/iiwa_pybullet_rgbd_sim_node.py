#!/usr/bin/env python3
"""Camera-enabled iiwa execution node for the perception loop."""

import json
import math
import os
import threading
import time

import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from std_msgs.msg import Bool, Float64MultiArray, Header, Int8, String
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
    """
    Run a PyBullet world, publish RGB-D data, and accept the `/iiwa7/*` API.

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
        self.declare_parameter("position_gain", 0.25)
        self.declare_parameter("velocity_gain", 1.0)
        self.declare_parameter("max_velocity", 2.0)
        self.declare_parameter("tau_limit", 200.0)

        self.declare_parameter("cam_width", 640)
        self.declare_parameter("cam_height", 480)
        self.declare_parameter("cam_fov_y_deg", 58.0)
        self.declare_parameter("cam_near", 0.02)
        self.declare_parameter("cam_far", 3.0)

        self.declare_parameter("cam_target", [0.72, 0.0, 0.28])
        self.declare_parameter("cam_distance", 1.0)
        self.declare_parameter("cam_yaw_deg", 90.0)
        self.declare_parameter("cam_pitch_deg", -45.0)
        self.declare_parameter("cam_roll_deg", 0.0)

        self.declare_parameter("table_center_xy", [0.72, 0.0])
        self.declare_parameter("table_size_xyz", [0.72, 0.52, 0.04])
        self.declare_parameter("table_surface_z", 0.34)
        self.declare_parameter("table_leg_width", 0.05)
        self.declare_parameter("cube_size", 0.05)
        self.declare_parameter("cube_mass", 0.08)
        self.declare_parameter("scene_preset", "cubes")
        self.declare_parameter("enable_gripper", False)
        self.declare_parameter("gripper_model", "kuka_parallel")
        self.declare_parameter("gripper_command_topic", "/iiwa7/gripper_command")
        self.declare_parameter("gripper_status_topic", "/iiwa7/gripper_status")
        self.declare_parameter("gripper_open_width_m", 0.110)
        self.declare_parameter("gripper_closed_width_m", 0.025)
        self.declare_parameter("gripper_grasp_radius_m", 0.130)
        self.declare_parameter("gripper_contact_distance_m", 0.012)
        self.declare_parameter("gripper_require_actual_contact", True)
        self.declare_parameter("gripper_grasp_mode", "keypoint")
        self.declare_parameter("gripper_keypoint_snap_to_center", False)
        self.declare_parameter("gripper_keypoint_snap_local_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("gripper_physical_contact_guard", True)
        self.declare_parameter("gripper_physical_preload_rad", 0.025)
        self.declare_parameter("gripper_physical_force", 35.0)
        self.declare_parameter("gripper_physical_position_gain", 0.35)
        self.declare_parameter("gripper_physical_max_velocity", 0.55)
        self.declare_parameter("gripper_grasp_settle_sec", 0.35)
        self.declare_parameter("gripper_grasp_timeout_sec", 1.20)
        self.declare_parameter("gripper_auto_lift_height_m", 0.100)
        self.declare_parameter("gripper_auto_lift_speed_mps", 0.100)
        self.declare_parameter("gripper_auto_lower_speed_mps", 0.120)
        self.declare_parameter("medical_grasp_scissors_fixed", True)
        self.declare_parameter("medical_grasp_scissors_graspable", False)
        self.declare_parameter("medical_sorting_tray_enabled", True)
        self.declare_parameter("medical_sorting_tray_offset_xy", [-0.165, 0.150])
        self.declare_parameter("medical_sorting_tray_size_xy", [0.260, 0.170])
        self.declare_parameter("medical_sorting_tray_wall_height_m", 0.035)
        self.declare_parameter("external_scene_objects_json", "[]")
        self.declare_parameter("external_scene_objects_file", "")
        self.declare_parameter("stabilize_resting_graspables", True)
        self.declare_parameter("resting_table_clearance_m", 0.0)
        self.declare_parameter("resting_snap_margin_m", 0.025)
        self.declare_parameter("resting_linear_speed_th_mps", 0.020)
        self.declare_parameter("resting_angular_speed_th_radps", 0.080)
        self.declare_parameter("resting_reanchor_delay_sec", 0.60)
        self.declare_parameter("resting_interaction_release_radius_m", 0.160)
        self.declare_parameter("show_keypoint_overlay", False)
        self.declare_parameter("keypoint_overlay_topic", "/perception/keypoint_3d")
        self.declare_parameter("keypoint_overlay_valid_topic", "/perception/valid")
        self.declare_parameter(
            "keypoint_overlay_plane_normal_topic",
            "/perception/object_plane_normal",
        )
        self.declare_parameter("keypoint_overlay_radius_m", 0.012)
        self.declare_parameter("keypoint_overlay_cross_size_m", 0.045)
        self.declare_parameter("keypoint_overlay_normal_length_m", 0.080)
        self.declare_parameter("keypoint_overlay_max_age_sec", 2.0)
        self.declare_parameter("keypoint_overlay_require_valid", True)
        self.declare_parameter("keypoint_overlay_raise_m", 0.0)
        self.declare_parameter("keypoint_overlay_label", "keypoint")

        self.gui = bool(self.get_parameter("gui").value)
        self.sim_hz = float(self.get_parameter("sim_hz").value)
        self.color_hz = float(self.get_parameter("color_hz").value)
        self.points_hz = float(self.get_parameter("points_hz").value)
        self.joint_state_hz = float(self.get_parameter("joint_state_hz").value)

        self.position_force = float(self.get_parameter("position_force").value)
        self.position_gain = float(self.get_parameter("position_gain").value)
        self.velocity_gain = float(self.get_parameter("velocity_gain").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.tau_limit = float(self.get_parameter("tau_limit").value)
        self.init_q = [float(value) for value in self.get_parameter("init_q").value]

        table_center_xy = [float(value) for value in self.get_parameter("table_center_xy").value]
        table_size_xyz = [float(value) for value in self.get_parameter("table_size_xyz").value]
        self.table_center_xy = table_center_xy[:2]
        self.table_size_xyz = table_size_xyz[:3]
        self.table_surface_z = float(self.get_parameter("table_surface_z").value)
        self.table_leg_width = float(self.get_parameter("table_leg_width").value)
        self.cube_size = float(self.get_parameter("cube_size").value)
        self.cube_mass = float(self.get_parameter("cube_mass").value)
        self.scene_preset = str(self.get_parameter("scene_preset").value).strip().lower()
        self.enable_gripper = bool(self.get_parameter("enable_gripper").value)
        self.gripper_model = str(self.get_parameter("gripper_model").value).strip().lower()
        self.gripper_command_topic = str(self.get_parameter("gripper_command_topic").value)
        self.gripper_status_topic = str(self.get_parameter("gripper_status_topic").value)
        self.gripper_open_width_m = float(self.get_parameter("gripper_open_width_m").value)
        self.gripper_closed_width_m = float(
            self.get_parameter("gripper_closed_width_m").value
        )
        self.gripper_grasp_radius_m = float(self.get_parameter("gripper_grasp_radius_m").value)
        self.gripper_contact_distance_m = float(
            self.get_parameter("gripper_contact_distance_m").value
        )
        self.gripper_require_actual_contact = bool(
            self.get_parameter("gripper_require_actual_contact").value
        )
        self.gripper_grasp_mode = str(
            self.get_parameter("gripper_grasp_mode").value
        ).strip().lower()
        self.gripper_keypoint_snap_to_center = bool(
            self.get_parameter("gripper_keypoint_snap_to_center").value
        )
        self.gripper_keypoint_snap_local_offset = np.array(
            [
                float(value)
                for value in self.get_parameter("gripper_keypoint_snap_local_offset").value
            ][:3],
            dtype=np.float64,
        )
        if self.gripper_keypoint_snap_local_offset.shape[0] < 3:
            self.gripper_keypoint_snap_local_offset = np.pad(
                self.gripper_keypoint_snap_local_offset,
                (0, 3 - self.gripper_keypoint_snap_local_offset.shape[0]),
            )
        self.gripper_physical_contact_guard = bool(
            self.get_parameter("gripper_physical_contact_guard").value
        )
        self.gripper_physical_preload_rad = float(
            self.get_parameter("gripper_physical_preload_rad").value
        )
        self.gripper_physical_force = float(
            self.get_parameter("gripper_physical_force").value
        )
        self.gripper_physical_position_gain = float(
            self.get_parameter("gripper_physical_position_gain").value
        )
        self.gripper_physical_max_velocity = float(
            self.get_parameter("gripper_physical_max_velocity").value
        )
        self.gripper_grasp_settle_sec = float(
            self.get_parameter("gripper_grasp_settle_sec").value
        )
        self.gripper_grasp_timeout_sec = float(
            self.get_parameter("gripper_grasp_timeout_sec").value
        )
        self.gripper_auto_lift_height_m = float(
            self.get_parameter("gripper_auto_lift_height_m").value
        )
        self.gripper_auto_lift_speed_mps = float(
            self.get_parameter("gripper_auto_lift_speed_mps").value
        )
        self.gripper_auto_lower_speed_mps = float(
            self.get_parameter("gripper_auto_lower_speed_mps").value
        )
        self.medical_grasp_scissors_fixed = bool(
            self.get_parameter("medical_grasp_scissors_fixed").value
        )
        self.medical_grasp_scissors_graspable = bool(
            self.get_parameter("medical_grasp_scissors_graspable").value
        )
        self.medical_sorting_tray_enabled = bool(
            self.get_parameter("medical_sorting_tray_enabled").value
        )
        tray_offset_xy = [
            float(value)
            for value in self.get_parameter("medical_sorting_tray_offset_xy").value
        ]
        tray_size_xy = [
            float(value)
            for value in self.get_parameter("medical_sorting_tray_size_xy").value
        ]
        self.medical_sorting_tray_offset_xy = (tray_offset_xy + [-0.165, 0.150])[:2]
        self.medical_sorting_tray_size_xy = (tray_size_xy + [0.260, 0.170])[:2]
        self.medical_sorting_tray_wall_height_m = float(
            self.get_parameter("medical_sorting_tray_wall_height_m").value
        )
        self.external_scene_objects_json = str(
            self.get_parameter("external_scene_objects_json").value
        )
        self.external_scene_objects_file = str(
            self.get_parameter("external_scene_objects_file").value
        )
        self.stabilize_resting_graspables = bool(
            self.get_parameter("stabilize_resting_graspables").value
        )
        self.resting_table_clearance_m = float(
            self.get_parameter("resting_table_clearance_m").value
        )
        self.resting_snap_margin_m = float(
            self.get_parameter("resting_snap_margin_m").value
        )
        self.resting_linear_speed_th_mps = float(
            self.get_parameter("resting_linear_speed_th_mps").value
        )
        self.resting_angular_speed_th_radps = float(
            self.get_parameter("resting_angular_speed_th_radps").value
        )
        self.resting_reanchor_delay_sec = float(
            self.get_parameter("resting_reanchor_delay_sec").value
        )
        self.resting_interaction_release_radius_m = float(
            self.get_parameter("resting_interaction_release_radius_m").value
        )
        self.show_keypoint_overlay = bool(self.get_parameter("show_keypoint_overlay").value)
        self.keypoint_overlay_topic = str(
            self.get_parameter("keypoint_overlay_topic").value
        )
        self.keypoint_overlay_valid_topic = str(
            self.get_parameter("keypoint_overlay_valid_topic").value
        )
        self.keypoint_overlay_plane_normal_topic = str(
            self.get_parameter("keypoint_overlay_plane_normal_topic").value
        )
        self.keypoint_overlay_radius_m = float(
            self.get_parameter("keypoint_overlay_radius_m").value
        )
        self.keypoint_overlay_cross_size_m = float(
            self.get_parameter("keypoint_overlay_cross_size_m").value
        )
        self.keypoint_overlay_normal_length_m = float(
            self.get_parameter("keypoint_overlay_normal_length_m").value
        )
        self.keypoint_overlay_max_age_sec = float(
            self.get_parameter("keypoint_overlay_max_age_sec").value
        )
        self.keypoint_overlay_require_valid = bool(
            self.get_parameter("keypoint_overlay_require_valid").value
        )
        self.keypoint_overlay_raise_m = float(
            self.get_parameter("keypoint_overlay_raise_m").value
        )
        self.keypoint_overlay_label = str(
            self.get_parameter("keypoint_overlay_label").value
        )

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
        self.pub_gripper_status = self.create_publisher(
            String,
            self.gripper_status_topic,
            10,
        )
        self.sub_gripper_command = None
        if self.enable_gripper:
            self.sub_gripper_command = self.create_subscription(
                String,
                self.gripper_command_topic,
                self.on_gripper_command,
                10,
            )
        self.sub_keypoint_overlay = None
        self.sub_keypoint_overlay_valid = None
        self.sub_keypoint_overlay_plane_normal = None
        self.keypoint_overlay_raw_xyz = None
        self.keypoint_overlay_frame_id = ""
        self.keypoint_overlay_received_time = 0.0
        self.keypoint_overlay_valid = False
        self.keypoint_overlay_plane_normal_raw = None
        self.keypoint_overlay_plane_normal_frame_id = ""
        self.keypoint_overlay_plane_normal_received_time = 0.0
        self.keypoint_overlay_body_id = None
        self.keypoint_overlay_cross_line_ids = [-1, -1, -1]
        self.keypoint_overlay_normal_line_id = -1
        self.keypoint_overlay_label_id = -1
        self.keypoint_overlay_warned_frames = set()
        if self.show_keypoint_overlay:
            self.sub_keypoint_overlay = self.create_subscription(
                PointStamped,
                self.keypoint_overlay_topic,
                self.on_keypoint_overlay,
                10,
            )
            self.sub_keypoint_overlay_valid = self.create_subscription(
                Bool,
                self.keypoint_overlay_valid_topic,
                self.on_keypoint_overlay_valid,
                10,
            )
            self.sub_keypoint_overlay_plane_normal = self.create_subscription(
                Vector3Stamped,
                self.keypoint_overlay_plane_normal_topic,
                self.on_keypoint_overlay_plane_normal,
                10,
            )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.0, 0.0, -9.81, physicsClientId=self.client)
        p.setTimeStep(1.0 / max(self.sim_hz, 1e-6), physicsClientId=self.client)
        p.setRealTimeSimulation(0, physicsClientId=self.client)
        self._create_keypoint_overlay_marker()

        self.table_body_ids = []
        self.object_ids = []
        self.graspable_object_ids = []
        self.resting_graspable_anchors = {}
        self.resting_anchor_pause_until = {}
        self.gripper_body_ids = []
        self.franka_gripper_id = None
        self.franka_gripper_constraint_id = None
        self.franka_finger_joint_indices = []
        self.franka_grasptarget_link_index = None
        self.franka_grasp_center_local = [0.0, 0.0, 0.105]
        self.wsg50_joint_indices = {}
        self.wsg50_fingertip_link_indices = []
        self.wsg50_open_joint_targets = {}
        self.wsg50_closed_joint_targets = {}
        self.wsg50_joint_targets = {}
        self.gripper_constraint_parent_link_index = 6
        self.gripper_width_m = self.gripper_open_width_m
        self.grasp_constraint_id = None
        self.held_body_id = None
        self.held_parent_local_pos = None
        self.held_parent_local_orn = None
        self.held_lift_start_time = None
        self.held_lift_start_z = None
        self.held_lower_start_time = None
        self.held_lower_start_z = None
        self.held_lower_target_z = None
        self.pending_grasp_start_time = None
        self.pending_grasp_last_status_time = 0.0
        self.last_physical_grasp_status_time = 0.0
        self.physical_grasp_lost_since = None
        self.ee_link_index = 6
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

    def _create_keypoint_overlay_marker(self):
        if not self.show_keypoint_overlay:
            return
        radius = max(float(self.keypoint_overlay_radius_m), 0.001)
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=[1.0, 0.86, 0.05, 0.95],
            physicsClientId=self.client,
        )
        self.keypoint_overlay_body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape,
            basePosition=[0.0, 0.0, -10.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        self.get_logger().info(
            "[KEYPOINT_OVERLAY] enabled: "
            f"topic={self.keypoint_overlay_topic} "
            f"valid_topic={self.keypoint_overlay_valid_topic} "
            f"radius={radius:.3f}m"
        )

    def _quat_to_matrix(self, quat):
        matrix = p.getMatrixFromQuaternion(quat)
        return [
            [matrix[0], matrix[1], matrix[2]],
            [matrix[3], matrix[4], matrix[5]],
            [matrix[6], matrix[7], matrix[8]],
        ]

    def _mat_vec(self, matrix, vector):
        return [
            matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
            matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
            matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
        ]

    def _camera_optical_to_world_tf(self):
        translation, quat = view_matrix_to_world_optical_tf(self.camera.view_matrix())
        return [float(value) for value in translation], self._quat_to_matrix(quat)

    def _normalize_vector(self, vector):
        norm = math.sqrt(sum(float(value) * float(value) for value in vector))
        if norm <= 1e-9 or not math.isfinite(norm):
            return None
        return [float(value) / norm for value in vector]

    def _keypoint_point_to_world(self, point_xyz, frame_id: str):
        frame_id = (frame_id or "").strip()
        point = [float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2])]
        if frame_id in ("", "world", "/world"):
            return point
        if (
            frame_id == "sim_camera_color_optical_frame"
            or "camera_color_optical_frame" in frame_id
            or frame_id.endswith("_optical_frame")
        ):
            translation, rotation = self._camera_optical_to_world_tf()
            rotated = self._mat_vec(rotation, point)
            return [
                translation[0] + rotated[0],
                translation[1] + rotated[1],
                translation[2] + rotated[2],
            ]
        if frame_id not in self.keypoint_overlay_warned_frames:
            self.keypoint_overlay_warned_frames.add(frame_id)
            self.get_logger().warning(
                f"[KEYPOINT_OVERLAY] unsupported keypoint frame '{frame_id}', "
                "showing it as world coordinates"
            )
        return point

    def _keypoint_vector_to_world(self, vector_xyz, frame_id: str):
        frame_id = (frame_id or "").strip()
        vector = [float(vector_xyz[0]), float(vector_xyz[1]), float(vector_xyz[2])]
        if frame_id in ("", "world", "/world"):
            return self._normalize_vector(vector)
        if (
            frame_id == "sim_camera_color_optical_frame"
            or "camera_color_optical_frame" in frame_id
            or frame_id.endswith("_optical_frame")
        ):
            _translation, rotation = self._camera_optical_to_world_tf()
            return self._normalize_vector(self._mat_vec(rotation, vector))
        if frame_id not in self.keypoint_overlay_warned_frames:
            self.keypoint_overlay_warned_frames.add(frame_id)
            self.get_logger().warning(
                f"[KEYPOINT_OVERLAY] unsupported vector frame '{frame_id}', "
                "showing it as world coordinates"
            )
        return self._normalize_vector(vector)

    def _set_debug_line(self, index: int, start, end, color, width: float):
        line_id = self.keypoint_overlay_cross_line_ids[index]
        self.keypoint_overlay_cross_line_ids[index] = p.addUserDebugLine(
            start,
            end,
            lineColorRGB=color,
            lineWidth=width,
            lifeTime=0.0,
            replaceItemUniqueId=line_id,
            physicsClientId=self.client,
        )

    def _hide_keypoint_overlay(self):
        if self.keypoint_overlay_body_id is not None:
            p.resetBasePositionAndOrientation(
                self.keypoint_overlay_body_id,
                [0.0, 0.0, -10.0],
                [0.0, 0.0, 0.0, 1.0],
                physicsClientId=self.client,
            )
        hidden = [0.0, 0.0, -10.0]
        for index in range(3):
            self._set_debug_line(index, hidden, hidden, [1.0, 0.86, 0.05], 1.0)
        self.keypoint_overlay_normal_line_id = p.addUserDebugLine(
            hidden,
            hidden,
            lineColorRGB=[0.0, 0.85, 1.0],
            lineWidth=1.0,
            lifeTime=0.0,
            replaceItemUniqueId=self.keypoint_overlay_normal_line_id,
            physicsClientId=self.client,
        )
        self.keypoint_overlay_label_id = p.addUserDebugText(
            "",
            hidden,
            textColorRGB=[1.0, 0.86, 0.05],
            textSize=1.0,
            lifeTime=0.0,
            replaceItemUniqueId=self.keypoint_overlay_label_id,
            physicsClientId=self.client,
        )

    def _update_keypoint_overlay(self):
        if not self.show_keypoint_overlay or self.keypoint_overlay_body_id is None:
            return

        now = time.time()
        raw_xyz = self.keypoint_overlay_raw_xyz
        if raw_xyz is None:
            self._hide_keypoint_overlay()
            return
        if (
            self.keypoint_overlay_max_age_sec > 0.0
            and now - self.keypoint_overlay_received_time > self.keypoint_overlay_max_age_sec
        ):
            self._hide_keypoint_overlay()
            return
        if self.keypoint_overlay_require_valid and not self.keypoint_overlay_valid:
            self._hide_keypoint_overlay()
            return

        point_world = self._keypoint_point_to_world(raw_xyz, self.keypoint_overlay_frame_id)
        point_world = [
            point_world[0],
            point_world[1],
            point_world[2] + float(self.keypoint_overlay_raise_m),
        ]
        if any(not math.isfinite(value) for value in point_world):
            self._hide_keypoint_overlay()
            return

        p.resetBasePositionAndOrientation(
            self.keypoint_overlay_body_id,
            point_world,
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )

        half = max(float(self.keypoint_overlay_cross_size_m), 0.004) * 0.5
        line_width = 3.0 if self.gui else 1.0
        axes = (
            (
                [point_world[0] - half, point_world[1], point_world[2]],
                [point_world[0] + half, point_world[1], point_world[2]],
            ),
            (
                [point_world[0], point_world[1] - half, point_world[2]],
                [point_world[0], point_world[1] + half, point_world[2]],
            ),
            (
                [point_world[0], point_world[1], point_world[2] - half],
                [point_world[0], point_world[1], point_world[2] + half],
            ),
        )
        for index, (start, end) in enumerate(axes):
            self._set_debug_line(index, start, end, [1.0, 0.86, 0.05], line_width)

        normal = None
        if (
            self.keypoint_overlay_plane_normal_raw is not None
            and now - self.keypoint_overlay_plane_normal_received_time
            <= max(self.keypoint_overlay_max_age_sec, 0.1)
        ):
            normal = self._keypoint_vector_to_world(
                self.keypoint_overlay_plane_normal_raw,
                self.keypoint_overlay_plane_normal_frame_id,
            )
        if normal is not None:
            length = max(float(self.keypoint_overlay_normal_length_m), 0.0)
            end = [
                point_world[0] + normal[0] * length,
                point_world[1] + normal[1] * length,
                point_world[2] + normal[2] * length,
            ]
        else:
            end = point_world
        self.keypoint_overlay_normal_line_id = p.addUserDebugLine(
            point_world,
            end,
            lineColorRGB=[0.0, 0.85, 1.0],
            lineWidth=line_width,
            lifeTime=0.0,
            replaceItemUniqueId=self.keypoint_overlay_normal_line_id,
            physicsClientId=self.client,
        )

        if self.keypoint_overlay_label:
            label_pos = [
                point_world[0],
                point_world[1],
                point_world[2] + max(0.035, half),
            ]
            self.keypoint_overlay_label_id = p.addUserDebugText(
                self.keypoint_overlay_label,
                label_pos,
                textColorRGB=[1.0, 0.86, 0.05],
                textSize=1.1,
                lifeTime=0.0,
                replaceItemUniqueId=self.keypoint_overlay_label_id,
                physicsClientId=self.client,
            )

    def _create_box_body(
        self,
        half_extents,
        rgba,
        base_position,
        mass=0.0,
        base_orientation=None,
    ):
        """Create a simple box rigid body with explicit visual + collision shape."""
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            physicsClientId=self.client,
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=rgba,
            physicsClientId=self.client,
        )
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0, 1.0]
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=base_position,
            baseOrientation=base_orientation,
            physicsClientId=self.client,
        )
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=1.0,
            spinningFriction=0.01,
            rollingFriction=0.001,
            restitution=0.0,
            physicsClientId=self.client,
        )
        return body_id

    def _create_compound_box_body(
        self,
        half_extents_list,
        rgba_list,
        frame_positions,
        base_position,
        mass=0.0,
        base_orientation=None,
        frame_orientations=None,
    ):
        """Create one rigid body from multiple small box shapes."""
        shape_count = len(half_extents_list)
        if frame_orientations is None:
            frame_orientations = [[0.0, 0.0, 0.0, 1.0] for _ in range(shape_count)]
        collision_shape = p.createCollisionShapeArray(
            shapeTypes=[p.GEOM_BOX] * shape_count,
            halfExtents=half_extents_list,
            collisionFramePositions=frame_positions,
            collisionFrameOrientations=frame_orientations,
            physicsClientId=self.client,
        )
        visual_shape = p.createVisualShapeArray(
            shapeTypes=[p.GEOM_BOX] * shape_count,
            halfExtents=half_extents_list,
            rgbaColors=rgba_list,
            visualFramePositions=frame_positions,
            visualFrameOrientations=frame_orientations,
            physicsClientId=self.client,
        )
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0, 1.0]
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=base_position,
            baseOrientation=base_orientation,
            physicsClientId=self.client,
        )
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=1.0,
            spinningFriction=0.01,
            rollingFriction=0.001,
            restitution=0.0,
            physicsClientId=self.client,
        )
        return body_id

    def _create_cylinder_body(
        self,
        radius,
        height,
        rgba,
        base_position,
        mass=0.0,
        base_orientation=None,
    ):
        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=radius,
            height=height,
            physicsClientId=self.client,
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=radius,
            length=height,
            rgbaColor=rgba,
            physicsClientId=self.client,
        )
        if base_orientation is None:
            base_orientation = [0.0, 0.0, 0.0, 1.0]
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=base_position,
            baseOrientation=base_orientation,
            physicsClientId=self.client,
        )
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=1.0,
            spinningFriction=0.01,
            rollingFriction=0.001,
            restitution=0.0,
            physicsClientId=self.client,
        )
        return body_id

    def _create_sphere_body(self, radius, rgba, base_position, mass=0.0):
        collision_shape = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=radius,
            physicsClientId=self.client,
        )
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=rgba,
            physicsClientId=self.client,
        )
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=base_position,
            physicsClientId=self.client,
        )
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=1.0,
            spinningFriction=0.01,
            rollingFriction=0.001,
            restitution=0.0,
            physicsClientId=self.client,
        )
        return body_id

    def _load_urdf_body(
        self,
        urdf_path,
        *,
        base_position,
        base_euler=(0.0, 0.0, 0.0),
        use_fixed_base=True,
        global_scaling=1.0,
    ):
        body_id = p.loadURDF(
            urdf_path,
            basePosition=base_position,
            baseOrientation=p.getQuaternionFromEuler(base_euler),
            useFixedBase=use_fixed_base,
            globalScaling=global_scaling,
            physicsClientId=self.client,
        )
        p.changeDynamics(
            body_id,
            -1,
            lateralFriction=1.0,
            spinningFriction=0.01,
            rollingFriction=0.001,
            restitution=0.0,
            physicsClientId=self.client,
        )
        return body_id

    def _resolve_model_path(self, model_path: str) -> str | None:
        expanded = os.path.expandvars(os.path.expanduser(model_path))
        candidates = [expanded]
        if not os.path.isabs(expanded):
            candidates.append(os.path.join(os.getcwd(), expanded))
            candidates.append(os.path.join(pybullet_data.getDataPath(), expanded))
        for candidate in candidates:
            if os.path.exists(candidate):
                return os.path.abspath(candidate)
        return None

    def _scene_position(self, config: dict) -> list[float]:
        position = config.get("position", config.get("xyz", [0.0, 0.0, 0.02]))
        if not isinstance(position, list) or len(position) != 3:
            raise ValueError("position/xyz must be a 3-element list")
        position = [float(value) for value in position]
        frame = str(config.get("position_frame", config.get("frame", "table"))).lower()
        if frame in ("table", "table_surface"):
            return [
                self.table_center_xy[0] + position[0],
                self.table_center_xy[1] + position[1],
                self.table_surface_z + position[2],
            ]
        if frame == "world":
            return position
        raise ValueError("position_frame must be 'table' or 'world'")

    def _scene_orientation(self, config: dict):
        rpy = config.get("rpy", config.get("euler", [0.0, 0.0, 0.0]))
        if not isinstance(rpy, list) or len(rpy) != 3:
            raise ValueError("rpy/euler must be a 3-element list")
        return p.getQuaternionFromEuler([float(value) for value in rpy])

    def _mesh_scale(self, scale):
        if isinstance(scale, list):
            if len(scale) != 3:
                raise ValueError("mesh scale list must have 3 elements")
            return [float(value) for value in scale]
        value = float(scale)
        return [value, value, value]

    def _mesh_bounds(self, model_path: str):
        ext = os.path.splitext(model_path)[1].lower()
        mins = [float("inf"), float("inf"), float("inf")]
        maxs = [float("-inf"), float("-inf"), float("-inf")]

        def _add_point(point):
            for axis, value in enumerate(point):
                mins[axis] = min(mins[axis], float(value))
                maxs[axis] = max(maxs[axis], float(value))

        if ext == ".stl":
            import struct

            with open(model_path, "rb") as handle:
                data = handle.read()
            if len(data) >= 84:
                triangle_count = struct.unpack("<I", data[80:84])[0]
                expected_size = 84 + triangle_count * 50
                if expected_size == len(data):
                    offset = 84
                    for _ in range(triangle_count):
                        values = struct.unpack("<12fH", data[offset:offset + 50])
                        for start in (3, 6, 9):
                            _add_point(values[start:start + 3])
                        offset += 50
                    return mins, maxs
            text = data.decode("utf-8", errors="ignore")
            for line in text.splitlines():
                parts = line.strip().split()
                if len(parts) == 4 and parts[0].lower() == "vertex":
                    _add_point([float(parts[1]), float(parts[2]), float(parts[3])])
        elif ext == ".obj":
            with open(model_path, "r", encoding="utf-8", errors="ignore") as handle:
                for line in handle:
                    parts = line.strip().split()
                    if len(parts) >= 4 and parts[0] == "v":
                        _add_point([float(parts[1]), float(parts[2]), float(parts[3])])
        else:
            return None

        if any(not math.isfinite(value) for value in mins + maxs):
            return None
        return mins, maxs

    def _rotate_vector(self, orientation, vector):
        matrix = p.getMatrixFromQuaternion(orientation)
        return [
            matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
            matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
            matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
        ]

    def _oriented_box_min_z(self, orientation, half_extents):
        matrix = p.getMatrixFromQuaternion(orientation)
        min_z = float("inf")
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    x = sx * half_extents[0]
                    y = sy * half_extents[1]
                    z = sz * half_extents[2]
                    world_z = matrix[6] * x + matrix[7] * y + matrix[8] * z
                    min_z = min(min_z, world_z)
        return min_z

    def _recentered_mesh_base_pose(self, config: dict, orientation, half_extents):
        base_position = self._scene_position(config)
        placement = str(config.get("placement", "raw")).lower()
        if placement in ("center_on_table", "mesh_center_on_table", "table_center"):
            min_z = self._oriented_box_min_z(orientation, half_extents)
            return [base_position[0], base_position[1], base_position[2] - min_z]
        return base_position

    def _placed_mesh_base_pose(self, model_path: str, config: dict, scale, orientation):
        base_position = self._scene_position(config)
        placement = str(config.get("placement", "raw")).lower()
        if placement == "raw":
            return base_position
        bounds = self._mesh_bounds(model_path)
        if bounds is None:
            self.get_logger().warning(
                f"[SCENE] could not compute mesh bounds for placement='{placement}': {model_path}"
            )
            return base_position
        mins, maxs = bounds
        scaled_mins = [mins[index] * scale[index] for index in range(3)]
        scaled_maxs = [maxs[index] * scale[index] for index in range(3)]
        center = [
            0.5 * (scaled_mins[index] + scaled_maxs[index])
            for index in range(3)
        ]
        if placement in ("center", "mesh_center"):
            offset = self._rotate_vector(orientation, center)
            return [base_position[index] - offset[index] for index in range(3)]
        if placement in ("center_on_table", "mesh_center_on_table", "table_center"):
            offset = self._rotate_vector(orientation, [center[0], center[1], 0.0])
            return [
                base_position[0] - offset[0],
                base_position[1] - offset[1],
                base_position[2] - scaled_mins[2],
            ]
        self.get_logger().warning(f"[SCENE] unsupported mesh placement='{placement}', using raw")
        return base_position

    def _spawn_external_mesh_object(self, model_path: str, config: dict) -> list[int]:
        scale = self._mesh_scale(config.get("scale", 1.0))
        rgba = config.get("rgba", [0.78, 0.82, 0.86, 1.0])
        if not isinstance(rgba, list) or len(rgba) != 4:
            raise ValueError("rgba must be a 4-element list")
        orientation = self._scene_orientation(config)
        collision_mode = str(config.get("collision", "mesh")).lower()
        recenter_to_bounds = bool(config.get("recenter_to_bounds", config.get("recenter", False)))
        bounds = None
        if collision_mode in ("box", "aabb_box", "primitive_box") or recenter_to_bounds:
            bounds = self._mesh_bounds(model_path)
            if bounds is None:
                if collision_mode in ("box", "aabb_box", "primitive_box"):
                    self.get_logger().warning(
                        f"[SCENE] collision='box' requested but mesh bounds failed; "
                        f"using mesh collision: {model_path}"
                    )
                    collision_mode = "mesh"
                if recenter_to_bounds:
                    self.get_logger().warning(
                        "[SCENE] recenter_to_bounds requested but mesh bounds failed; "
                        f"using raw mesh frame: {model_path}"
                    )
                    recenter_to_bounds = False
        base_position = None
        visual_frame_position = [0.0, 0.0, 0.0]
        if collision_mode in ("box", "aabb_box", "primitive_box"):
            mins, maxs = bounds
            margin = config.get("collision_margin", [0.002, 0.002, 0.001])
            if not isinstance(margin, list):
                margin = [float(margin), float(margin), float(margin)]
            elif len(margin) != 3:
                raise ValueError("collision_margin list must have 3 elements")
            half_extents = []
            center = []
            scaled_mins = []
            scaled_maxs = []
            min_half_z = float(config.get("collision_min_half_z", 0.004))
            for axis in range(3):
                scaled_min = float(mins[axis]) * scale[axis]
                scaled_max = float(maxs[axis]) * scale[axis]
                scaled_mins.append(scaled_min)
                scaled_maxs.append(scaled_max)
                half = 0.5 * (scaled_max - scaled_min) + float(margin[axis])
                if axis == 2:
                    half = max(half, min_half_z)
                    center.append(scaled_min + half)
                else:
                    center.append(0.5 * (scaled_min + scaled_max))
                half_extents.append(half)
            if recenter_to_bounds:
                mesh_center = [
                    0.5 * (scaled_mins[index] + scaled_maxs[index])
                    for index in range(3)
                ]
                center = [0.0, 0.0, 0.0]
                visual_frame_position = [
                    -mesh_center[0],
                    -mesh_center[1],
                    -half_extents[2] - scaled_mins[2],
                ]
                base_position = self._recentered_mesh_base_pose(
                    config,
                    orientation,
                    half_extents,
                )
            collision_shape = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                collisionFramePosition=center,
                physicsClientId=self.client,
            )
        else:
            if recenter_to_bounds and bounds is not None:
                mins, maxs = bounds
                scaled_mins = [float(mins[index]) * scale[index] for index in range(3)]
                scaled_maxs = [float(maxs[index]) * scale[index] for index in range(3)]
                half_extents = [
                    0.5 * (scaled_maxs[index] - scaled_mins[index])
                    for index in range(3)
                ]
                mesh_center = [
                    0.5 * (scaled_mins[index] + scaled_maxs[index])
                    for index in range(3)
                ]
                visual_frame_position = [
                    -mesh_center[0],
                    -mesh_center[1],
                    -half_extents[2] - scaled_mins[2],
                ]
                base_position = self._recentered_mesh_base_pose(
                    config,
                    orientation,
                    half_extents,
                )
            collision_shape = p.createCollisionShape(
                p.GEOM_MESH,
                fileName=model_path,
                meshScale=scale,
                collisionFramePosition=visual_frame_position,
                physicsClientId=self.client,
            )
        if base_position is None:
            base_position = self._placed_mesh_base_pose(model_path, config, scale, orientation)
        visual_shape = p.createVisualShape(
            p.GEOM_MESH,
            fileName=model_path,
            meshScale=scale,
            rgbaColor=[float(value) for value in rgba],
            visualFramePosition=visual_frame_position,
            physicsClientId=self.client,
        )
        fixed = bool(config.get("fixed", True))
        body_id = p.createMultiBody(
            baseMass=0.0 if fixed else float(config.get("mass", 0.05)),
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=base_position,
            baseOrientation=orientation,
            physicsClientId=self.client,
        )
        dynamics_kwargs = {
            "lateralFriction": float(config.get("lateral_friction", 1.0)),
            "spinningFriction": float(config.get("spinning_friction", 0.01)),
            "rollingFriction": float(config.get("rolling_friction", 0.001)),
            "restitution": float(config.get("restitution", 0.0)),
            "linearDamping": float(config.get("linear_damping", 0.2)),
            "angularDamping": float(config.get("angular_damping", 0.8)),
            "physicsClientId": self.client,
        }
        if "contact_stiffness" in config:
            dynamics_kwargs["contactStiffness"] = float(config["contact_stiffness"])
        if "contact_damping" in config:
            dynamics_kwargs["contactDamping"] = float(config["contact_damping"])
        p.changeDynamics(body_id, -1, **dynamics_kwargs)
        return [body_id]

    def _spawn_external_scene_objects(self):
        raw = ""
        external_file = self.external_scene_objects_file.strip()
        if external_file and external_file != "__none__":
            resolved_file = self._resolve_model_path(external_file)
            if resolved_file is None:
                self.get_logger().error(
                    f"[SCENE] external_scene_objects_file not found: {external_file}"
                )
                return
            try:
                with open(resolved_file, "r", encoding="utf-8") as handle:
                    raw = handle.read().strip()
            except OSError as exc:
                self.get_logger().error(
                    f"[SCENE] failed to read external_scene_objects_file: {exc}"
                )
                return
        else:
            raw = self.external_scene_objects_json.strip()
        if not raw or raw == "[]":
            return
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"[SCENE] external_scene_objects_json invalid: {exc}")
            return
        if isinstance(parsed, dict):
            parsed = parsed.get("objects", [parsed])
        if not isinstance(parsed, list):
            self.get_logger().error("[SCENE] external_scene_objects_json must be a list or object")
            return

        for index, config in enumerate(parsed, start=1):
            if not isinstance(config, dict):
                self.get_logger().warning(
                    f"[SCENE] external object #{index} is not an object; skipped"
                )
                continue
            name = str(config.get("name", f"external_{index}"))
            model_path = self._resolve_model_path(str(config.get("path", "")))
            if model_path is None:
                self.get_logger().error(f"[SCENE] external object '{name}' path not found")
                continue
            model_type = str(
                config.get("type", os.path.splitext(model_path)[1].lstrip("."))
            ).lower()
            fixed = bool(config.get("fixed", True))
            graspable = bool(config.get("graspable", False))
            try:
                if model_type == "urdf":
                    body_ids = [
                        p.loadURDF(
                            model_path,
                            basePosition=self._scene_position(config),
                            baseOrientation=self._scene_orientation(config),
                            useFixedBase=fixed,
                            globalScaling=float(config.get("scale", 1.0)),
                            physicsClientId=self.client,
                        )
                    ]
                elif model_type == "sdf":
                    body_ids = list(
                        p.loadSDF(
                            model_path,
                            globalScaling=float(config.get("scale", 1.0)),
                            physicsClientId=self.client,
                        )
                    )
                    if len(body_ids) == 1:
                        p.resetBasePositionAndOrientation(
                            body_ids[0],
                            self._scene_position(config),
                            self._scene_orientation(config),
                            physicsClientId=self.client,
                        )
                elif model_type == "mjcf":
                    body_ids = list(p.loadMJCF(model_path, physicsClientId=self.client))
                elif model_type in ("obj", "stl", "dae", "mesh"):
                    body_ids = self._spawn_external_mesh_object(model_path, config)
                else:
                    self.get_logger().warning(
                        f"[SCENE] external object '{name}' unsupported type "
                        f"'{model_type}'; skipped"
                    )
                    continue
            except Exception as exc:
                self.get_logger().error(f"[SCENE] external object '{name}' failed: {exc}")
                continue

            self.object_ids.extend(body_ids)
            if graspable:
                if fixed:
                    self.get_logger().warning(
                        f"[SCENE] external object '{name}' is fixed; not adding it "
                        "to graspable objects"
                    )
                else:
                    self.graspable_object_ids.extend(body_ids)
            self.get_logger().info(
                f"[SCENE] external_object_loaded name={name} type={model_type} "
                f"bodies={body_ids} fixed={fixed} graspable={graspable}"
            )

    def _publish_gripper_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_gripper_status.publish(msg)
        self.get_logger().info(f"[GRIPPER] {text}")

    def _local_to_world(self, base_pos, base_orn, local_pos):
        world_pos, _world_orn = p.multiplyTransforms(
            base_pos,
            base_orn,
            local_pos,
            [0.0, 0.0, 0.0, 1.0],
        )
        return world_pos

    def _local_pose_to_world(self, base_pos, base_orn, local_pos, local_orn):
        world_pos, world_orn = p.multiplyTransforms(
            base_pos,
            base_orn,
            local_pos,
            local_orn,
        )
        return world_pos, world_orn

    def _create_visual_mesh_body(self, mesh_path, *, rgba, mesh_scale=None):
        if mesh_scale is None:
            mesh_scale = [1.0, 1.0, 1.0]
        visual_shape = p.createVisualShape(
            p.GEOM_MESH,
            fileName=mesh_path,
            meshScale=mesh_scale,
            rgbaColor=rgba,
            physicsClientId=self.client,
        )
        body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape,
            basePosition=[0.0, 0.0, 0.0],
            baseOrientation=[0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        p.setCollisionFilterGroupMask(
            body_id,
            -1,
            collisionFilterGroup=0,
            collisionFilterMask=0,
            physicsClientId=self.client,
        )
        return body_id

    def _spawn_franka_hand_gripper(self):
        """Attach a compact Franka Panda hand URDF to the iiwa wrist."""
        gripper_urdf_template = os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
            "assets",
            "franka_hand",
            "franka_hand_gripper.urdf",
        )
        if not os.path.exists(gripper_urdf_template):
            self.get_logger().warning(f"Franka hand URDF not found: {gripper_urdf_template}")
            return
        gripper_urdf = self._make_runtime_franka_hand_urdf(gripper_urdf_template)

        ee_state = p.getLinkState(
            self.robot_id,
            self.ee_link_index,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        self.franka_gripper_id = p.loadURDF(
            gripper_urdf,
            basePosition=ee_state[4],
            baseOrientation=ee_state[5],
            useFixedBase=False,
            flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
            physicsClientId=self.client,
        )
        self.gripper_body_ids = [self.franka_gripper_id]
        self.gripper_constraint_parent_link_index = self.ee_link_index
        self.franka_gripper_constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.ee_link_index,
            childBodyUniqueId=self.franka_gripper_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0.0, 0.0, 0.0],
            parentFramePosition=[0.0, 0.0, 0.0],
            childFramePosition=[0.0, 0.0, 0.0],
            parentFrameOrientation=[0.0, 0.0, 0.0, 1.0],
            childFrameOrientation=[0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        p.changeConstraint(
            self.franka_gripper_constraint_id,
            maxForce=5000.0,
            erp=0.9,
            physicsClientId=self.client,
        )

        self.franka_finger_joint_indices = []
        self.franka_grasptarget_link_index = None
        for joint_index in range(
            p.getNumJoints(self.franka_gripper_id, physicsClientId=self.client)
        ):
            info = p.getJointInfo(
                self.franka_gripper_id,
                joint_index,
                physicsClientId=self.client,
            )
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            if joint_name in ("panda_finger_joint1", "panda_finger_joint2"):
                self.franka_finger_joint_indices.append(joint_index)
            if link_name == "panda_grasptarget":
                self.franka_grasptarget_link_index = joint_index

        for link_index in range(
            -1,
            p.getNumJoints(self.franka_gripper_id, physicsClientId=self.client),
        ):
            p.setCollisionFilterGroupMask(
                self.franka_gripper_id,
                link_index,
                collisionFilterGroup=0,
                collisionFilterMask=0,
                physicsClientId=self.client,
            )

        self._drive_franka_hand_gripper()
        self._publish_gripper_status(
            "gripper_ready: franka_hand_urdf_open "
            "source=assets/franka_hand/franka_hand_gripper.urdf"
        )

    def _make_runtime_franka_hand_urdf(self, template_urdf):
        data_root = pybullet_data.getDataPath()
        mesh_replacements = {
            "franka_panda/meshes/visual/hand.obj": os.path.join(
                data_root, "franka_panda", "meshes", "visual", "hand.obj"
            ),
            "franka_panda/meshes/collision/hand.obj": os.path.join(
                data_root, "franka_panda", "meshes", "collision", "hand.obj"
            ),
            "franka_panda/meshes/visual/finger.obj": os.path.join(
                data_root, "franka_panda", "meshes", "visual", "finger.obj"
            ),
            "franka_panda/meshes/collision/finger.obj": os.path.join(
                data_root, "franka_panda", "meshes", "collision", "finger.obj"
            ),
        }
        for mesh_path in mesh_replacements.values():
            if not os.path.exists(mesh_path):
                self.get_logger().warning(
                    f"Franka hand mesh not found; loading template path instead: {mesh_path}"
                )
                return template_urdf

        try:
            with open(template_urdf, "r", encoding="utf-8") as handle:
                urdf_text = handle.read()
            for relative_path, absolute_path in mesh_replacements.items():
                urdf_text = urdf_text.replace(
                    f'filename="{relative_path}"',
                    f'filename="{absolute_path}"',
                )
            repo_root = os.path.dirname(
                os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            )
            run_logs_dir = os.path.join(repo_root, "run_logs")
            os.makedirs(run_logs_dir, exist_ok=True)
            runtime_urdf = os.path.join(run_logs_dir, "franka_hand_gripper_runtime.urdf")
            with open(runtime_urdf, "w", encoding="utf-8") as handle:
                handle.write(urdf_text)
            return runtime_urdf
        except OSError as exc:
            self.get_logger().warning(
                f"Could not create runtime Franka hand URDF; loading template path instead: {exc}"
            )
            return template_urdf

    def _spawn_simple_parallel_gripper(self):
        """Create a lightweight visual parallel-jaw gripper attached to the iiwa wrist."""
        palm_rgba = [0.16, 0.17, 0.18, 1.0]
        finger_rgba = [0.72, 0.76, 0.78, 1.0]
        fingertip_rgba = [0.08, 0.08, 0.09, 1.0]

        self.gripper_body_ids = [
            self._create_box_body(
                half_extents=[0.040, 0.014, 0.010],
                rgba=palm_rgba,
                base_position=[0.0, 0.0, 0.0],
                mass=0.0,
            ),
            self._create_box_body(
                half_extents=[0.008, 0.010, 0.045],
                rgba=finger_rgba,
                base_position=[0.0, 0.0, 0.0],
                mass=0.0,
            ),
            self._create_box_body(
                half_extents=[0.008, 0.010, 0.045],
                rgba=finger_rgba,
                base_position=[0.0, 0.0, 0.0],
                mass=0.0,
            ),
            self._create_box_body(
                half_extents=[0.009, 0.012, 0.010],
                rgba=fingertip_rgba,
                base_position=[0.0, 0.0, 0.0],
                mass=0.0,
            ),
            self._create_box_body(
                half_extents=[0.009, 0.012, 0.010],
                rgba=fingertip_rgba,
                base_position=[0.0, 0.0, 0.0],
                mass=0.0,
            ),
        ]
        for body_id in self.gripper_body_ids:
            p.setCollisionFilterGroupMask(
                body_id,
                -1,
                collisionFilterGroup=0,
                collisionFilterMask=0,
                physicsClientId=self.client,
            )
        self._update_simple_parallel_gripper()
        self._publish_gripper_status("gripper_ready: open")

    def _joint_index_by_name(self, joint_name: str) -> int | None:
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client)
            if info[1].decode("utf-8") == joint_name:
                return joint_index
        return None

    def _configure_wsg50_gripper(self):
        """Configure the PyBullet-data KUKA iiwa parallel gripper."""
        required_joint_names = {
            "left_base": "base_left_finger_joint",
            "left_tip": "left_base_tip_joint",
            "right_base": "base_right_finger_joint",
            "right_tip": "right_base_tip_joint",
        }
        indices = {
            role: self._joint_index_by_name(name)
            for role, name in required_joint_names.items()
        }
        if any(index is None for index in indices.values()):
            missing = [
                name
                for role, name in required_joint_names.items()
                if indices.get(role) is None
            ]
            self.get_logger().warning(
                "WSG50 model is missing expected joints: " + ", ".join(missing)
            )
            return

        self.wsg50_joint_indices = {role: int(index) for role, index in indices.items()}
        self.wsg50_fingertip_link_indices = [
            self.wsg50_joint_indices["left_tip"],
            self.wsg50_joint_indices["right_tip"],
        ]
        base_link_index = self._joint_index_by_name("gripper_to_arm")
        self.gripper_constraint_parent_link_index = (
            int(base_link_index) if base_link_index is not None else self.ee_link_index
        )

        # The PyBullet WSG50 SDF uses revolute finger joints. These target
        # angles give a visibly open pair of fingers and a narrow pinch pose.
        self.wsg50_open_joint_targets = {
            self.wsg50_joint_indices["left_base"]: -0.22,
            self.wsg50_joint_indices["left_tip"]: 0.22,
            self.wsg50_joint_indices["right_base"]: 0.22,
            self.wsg50_joint_indices["right_tip"]: -0.22,
        }
        self.wsg50_closed_joint_targets = {
            self.wsg50_joint_indices["left_base"]: 0.22,
            self.wsg50_joint_indices["left_tip"]: -0.22,
            self.wsg50_joint_indices["right_base"]: -0.22,
            self.wsg50_joint_indices["right_tip"]: 0.22,
        }
        self.wsg50_joint_targets = dict(self.wsg50_open_joint_targets)

        for joint_index in self.wsg50_joint_targets:
            p.resetJointState(
                self.robot_id,
                joint_index,
                self.wsg50_joint_targets[joint_index],
                physicsClientId=self.client,
            )
            p.changeDynamics(
                self.robot_id,
                joint_index,
                lateralFriction=3.0,
                spinningFriction=0.10,
                rollingFriction=0.01,
                restitution=0.0,
                contactStiffness=12000,
                contactDamping=180,
                physicsClientId=self.client,
            )

        self._publish_gripper_status(
            f"gripper_ready: {self.gripper_model}_open source=kuka_iiwa/kuka_with_gripper.sdf"
        )

    def _wsg50_close_direction(self, joint_index: int) -> float:
        if joint_index not in self.wsg50_open_joint_targets:
            return 0.0
        open_target = float(self.wsg50_open_joint_targets[joint_index])
        closed_target = float(self.wsg50_closed_joint_targets.get(joint_index, open_target))
        if closed_target > open_target:
            return 1.0
        if closed_target < open_target:
            return -1.0
        return 0.0

    def _set_wsg50_contact_guard_targets(self):
        if not self.wsg50_closed_joint_targets:
            return
        preload = max(self.gripper_physical_preload_rad, 0.0)
        guarded_targets = {}
        for joint_index, closed_target in self.wsg50_closed_joint_targets.items():
            try:
                current_position = p.getJointState(
                    self.robot_id,
                    joint_index,
                    physicsClientId=self.client,
                )[0]
            except Exception:
                current_position = float(closed_target)
            direction = self._wsg50_close_direction(joint_index)
            guarded = float(current_position) + direction * preload
            if direction > 0.0:
                guarded = min(guarded, float(closed_target))
            elif direction < 0.0:
                guarded = max(guarded, float(closed_target))
            else:
                guarded = float(closed_target)
            guarded_targets[joint_index] = guarded
        self.wsg50_joint_targets = guarded_targets

    def _drive_wsg50_gripper(self):
        if not self.enable_gripper or not self.wsg50_joint_targets:
            return
        physical_mode = self.gripper_grasp_mode == "physical"
        for joint_index, target_position in self.wsg50_joint_targets.items():
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(target_position),
                positionGain=(
                    self.gripper_physical_position_gain if physical_mode else 0.65
                ),
                velocityGain=1.0,
                force=(self.gripper_physical_force if physical_mode else 120.0),
                maxVelocity=(
                    self.gripper_physical_max_velocity if physical_mode else 1.8
                ),
                physicsClientId=self.client,
            )

    def _update_simple_parallel_gripper(self):
        if not self.enable_gripper or len(self.gripper_body_ids) != 5:
            return
        if not hasattr(self, "robot_id"):
            return

        link_state = p.getLinkState(
            self.robot_id,
            self.ee_link_index,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        ee_pos = link_state[4]
        ee_orn = link_state[5]
        width = float(
            max(
                self.gripper_closed_width_m,
                min(self.gripper_open_width_m, self.gripper_width_m),
            )
        )

        local_offsets = [
            [0.0, 0.0, 0.030],
            [0.0, 0.5 * width, 0.082],
            [0.0, -0.5 * width, 0.082],
            [0.0, 0.5 * width, 0.130],
            [0.0, -0.5 * width, 0.130],
        ]
        for body_id, local_pos in zip(self.gripper_body_ids, local_offsets):
            p.resetBasePositionAndOrientation(
                body_id,
                self._local_to_world(ee_pos, ee_orn, local_pos),
                ee_orn,
                physicsClientId=self.client,
            )

    def _drive_franka_hand_gripper(self):
        if not self.enable_gripper or self.gripper_model != "franka_hand":
            return
        if self.franka_gripper_id is None or not self.franka_finger_joint_indices:
            return
        target_width = float(
            max(self.gripper_closed_width_m, min(0.080, self.gripper_width_m))
        )
        joint_target = 0.5 * max(0.0, min(0.080, target_width))
        for joint_index in self.franka_finger_joint_indices:
            p.setJointMotorControl2(
                self.franka_gripper_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=joint_target,
                positionGain=0.65,
                velocityGain=1.0,
                force=40.0,
                maxVelocity=0.4,
                physicsClientId=self.client,
            )

    def _release_grasp(self):
        if self.grasp_constraint_id is not None:
            try:
                p.removeConstraint(self.grasp_constraint_id, physicsClientId=self.client)
            except Exception:
                pass
        if self.wsg50_open_joint_targets:
            self.wsg50_joint_targets = dict(self.wsg50_open_joint_targets)
        released_body = self.held_body_id
        released_z = None
        if released_body is not None:
            try:
                released_pos, _released_orn = p.getBasePositionAndOrientation(
                    released_body,
                    physicsClientId=self.client,
                )
                released_z = released_pos[2]
            except Exception:
                released_z = None
        self.grasp_constraint_id = None
        self.held_body_id = None
        self.held_parent_local_pos = None
        self.held_parent_local_orn = None
        self.held_lift_start_time = None
        self.held_lift_start_z = None
        self.held_lower_start_time = None
        self.held_lower_start_z = None
        self.held_lower_target_z = None
        self.pending_grasp_start_time = None
        self.physical_grasp_lost_since = None
        if released_body is not None:
            self.resting_graspable_anchors.pop(released_body, None)
            self.resting_anchor_pause_until[released_body] = (
                time.monotonic() + max(self.resting_reanchor_delay_sec, 0.0)
            )
        if released_body is None:
            self._publish_gripper_status("gripper_open: no_object")
        elif released_z is None:
            self._publish_gripper_status(f"gripper_open: released_body={released_body}")
        else:
            self._publish_gripper_status(
                f"gripper_open: released_body={released_body} z={released_z:.3f}m"
            )

    def _nearest_graspable_body(self):
        if not self.graspable_object_ids:
            return None, None

        grasp_center = self._gripper_grasp_center_world()
        if grasp_center is None:
            link_state = p.getLinkState(
                self.robot_id,
                self.ee_link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            grasp_center = link_state[4]

        nearest_body = None
        nearest_distance = None
        for body_id in self.graspable_object_ids:
            try:
                aabb_min, aabb_max = p.getAABB(
                    body_id,
                    physicsClientId=self.client,
                )
            except Exception:
                try:
                    obj_pos, _obj_orn = p.getBasePositionAndOrientation(
                        body_id,
                        physicsClientId=self.client,
                    )
                except Exception:
                    continue
                distance = math.sqrt(
                    (obj_pos[0] - grasp_center[0]) ** 2
                    + (obj_pos[1] - grasp_center[1]) ** 2
                    + (obj_pos[2] - grasp_center[2]) ** 2
                )
            else:
                dx = max(aabb_min[0] - grasp_center[0], 0.0, grasp_center[0] - aabb_max[0])
                dy = max(aabb_min[1] - grasp_center[1], 0.0, grasp_center[1] - aabb_max[1])
                dz = max(aabb_min[2] - grasp_center[2], 0.0, grasp_center[2] - aabb_max[2])
                distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if nearest_distance is None or distance < nearest_distance:
                nearest_body = body_id
                nearest_distance = distance

        return nearest_body, nearest_distance

    def _gripper_grasp_center_world(self):
        if self.gripper_model == "franka_hand":
            if (
                self.franka_gripper_id is not None
                and self.franka_grasptarget_link_index is not None
            ):
                try:
                    link_state = p.getLinkState(
                        self.franka_gripper_id,
                        self.franka_grasptarget_link_index,
                        computeForwardKinematics=True,
                        physicsClientId=self.client,
                    )
                    return list(link_state[4])
                except Exception:
                    pass
            try:
                link_state = p.getLinkState(
                    self.robot_id,
                    self.ee_link_index,
                    computeForwardKinematics=True,
                    physicsClientId=self.client,
                )
            except Exception:
                return None
            return list(
                self._local_to_world(
                    link_state[4],
                    link_state[5],
                    self.franka_grasp_center_local,
                )
            )
        if not self.wsg50_fingertip_link_indices:
            return None
        positions = []
        for link_index in self.wsg50_fingertip_link_indices:
            try:
                link_state = p.getLinkState(
                    self.robot_id,
                    link_index,
                    computeForwardKinematics=True,
                    physicsClientId=self.client,
                )
            except Exception:
                continue
            positions.append(link_state[4])
        if not positions:
            return None
        return [
            sum(position[axis] for position in positions) / len(positions)
            for axis in range(3)
        ]

    def _gripper_grasp_frame_world(self):
        center = self._gripper_grasp_center_world()
        if center is None:
            return None
        try:
            parent_state = p.getLinkState(
                self.robot_id,
                self.gripper_constraint_parent_link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            frame_orn = parent_state[5]
        except Exception:
            try:
                ee_state = p.getLinkState(
                    self.robot_id,
                    self.ee_link_index,
                    computeForwardKinematics=True,
                    physicsClientId=self.client,
                )
                frame_orn = ee_state[5]
            except Exception:
                frame_orn = [0.0, 0.0, 0.0, 1.0]
        return center, frame_orn

    def _gripper_interacting_with_body(self, body_id) -> bool:
        if not self.enable_gripper:
            return False
        try:
            contact_points = p.getContactPoints(
                self.robot_id,
                body_id,
                physicsClientId=self.client,
            )
        except Exception:
            contact_points = []
        if contact_points:
            return True

        release_radius = max(self.resting_interaction_release_radius_m, 0.0)
        if release_radius <= 0.0:
            return False
        grasp_center = self._gripper_grasp_center_world()
        if grasp_center is None:
            try:
                link_state = p.getLinkState(
                    self.robot_id,
                    self.ee_link_index,
                    computeForwardKinematics=True,
                    physicsClientId=self.client,
                )
                grasp_center = link_state[4]
            except Exception:
                return False
        return self._body_aabb_distance_to_point(body_id, grasp_center) <= release_radius

    def _body_aabb_distance_to_point(self, body_id, point) -> float:
        try:
            aabb_min, aabb_max = p.getAABB(
                body_id,
                physicsClientId=self.client,
            )
        except Exception:
            try:
                obj_pos, _obj_orn = p.getBasePositionAndOrientation(
                    body_id,
                    physicsClientId=self.client,
                )
            except Exception:
                return float("inf")
            return math.sqrt(
                (obj_pos[0] - point[0]) ** 2
                + (obj_pos[1] - point[1]) ** 2
                + (obj_pos[2] - point[2]) ** 2
            )

        dx = max(aabb_min[0] - point[0], 0.0, point[0] - aabb_max[0])
        dy = max(aabb_min[1] - point[1], 0.0, point[1] - aabb_max[1])
        dz = max(aabb_min[2] - point[2], 0.0, point[2] - aabb_max[2])
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _fingertip_distances_to_body(self, body_id) -> list[float]:
        distances = []
        max_distance = max(self.gripper_contact_distance_m * 2.5, 0.050)
        for link_index in self.wsg50_fingertip_link_indices:
            try:
                closest_points = p.getClosestPoints(
                    self.robot_id,
                    body_id,
                    distance=max_distance,
                    linkIndexA=link_index,
                    linkIndexB=-1,
                    physicsClientId=self.client,
                )
            except Exception:
                closest_points = []
            if closest_points:
                distances.append(min(float(point[8]) for point in closest_points))
            else:
                distances.append(float("inf"))
        return distances

    def _fingertip_contact_counts_to_body(self, body_id) -> list[int]:
        contact_counts = []
        for link_index in self.wsg50_fingertip_link_indices:
            try:
                contact_points = p.getContactPoints(
                    self.robot_id,
                    body_id,
                    linkIndexA=link_index,
                    linkIndexB=-1,
                    physicsClientId=self.client,
                )
            except Exception:
                contact_points = []
            contact_counts.append(len(contact_points))
        return contact_counts

    def _grasp_has_physical_contact(self, body_id, center_distance: float):
        if self.gripper_grasp_mode == "keypoint":
            return True, f"keypoint_center_distance={center_distance:.3f}m"

        contact_threshold = max(self.gripper_contact_distance_m, 0.0)
        center_threshold = max(contact_threshold * 2.0, 0.040)
        if center_distance > center_threshold:
            return (
                False,
                f"center_distance={center_distance:.3f}m>{center_threshold:.3f}m",
            )

        if not self.wsg50_fingertip_link_indices:
            return True, f"center_distance={center_distance:.3f}m"

        contact_counts = self._fingertip_contact_counts_to_body(body_id)
        if len(contact_counts) >= 2 and all(count > 0 for count in contact_counts[:2]):
            return True, f"fingertip_contacts={contact_counts}"

        fingertip_distances = self._fingertip_distances_to_body(body_id)
        if len(fingertip_distances) < 2:
            return False, "missing_fingertip_links"
        near_count = sum(distance <= contact_threshold for distance in fingertip_distances)
        distance_text = ",".join(
            "inf" if not math.isfinite(distance) else f"{distance:.3f}"
            for distance in fingertip_distances
        )
        if self.gripper_require_actual_contact:
            return (
                False,
                f"fingertip_contacts={contact_counts} "
                f"fingertip_distances=[{distance_text}] require_actual_contact=true",
            )
        if near_count < 2:
            return (
                False,
                f"fingertip_distances=[{distance_text}] threshold={contact_threshold:.3f}m",
            )
        return True, f"fingertip_distances=[{distance_text}]"

    def _set_body_flat_on_table(self, body_id):
        target_bottom_z = self.table_surface_z + max(self.resting_table_clearance_m, 0.0)
        base_pos, base_orn = p.getBasePositionAndOrientation(
            body_id,
            physicsClientId=self.client,
        )
        yaw = p.getEulerFromQuaternion(base_orn)[2]
        flat_orn = p.getQuaternionFromEuler([0.0, 0.0, yaw])
        p.resetBasePositionAndOrientation(
            body_id,
            base_pos,
            flat_orn,
            physicsClientId=self.client,
        )
        aabb_min, _aabb_max = p.getAABB(body_id, physicsClientId=self.client)
        flat_pos = list(base_pos)
        flat_pos[2] += target_bottom_z - float(aabb_min[2])
        p.resetBasePositionAndOrientation(
            body_id,
            flat_pos,
            flat_orn,
            physicsClientId=self.client,
        )
        p.resetBaseVelocity(
            body_id,
            linearVelocity=[0.0, 0.0, 0.0],
            angularVelocity=[0.0, 0.0, 0.0],
            physicsClientId=self.client,
        )
        return tuple(flat_pos), tuple(flat_orn)

    def _anchor_resting_graspable(self, body_id):
        try:
            stable_pose = self._set_body_flat_on_table(body_id)
        except Exception:
            return False
        self.resting_graspable_anchors[body_id] = stable_pose
        self.resting_anchor_pause_until.pop(body_id, None)
        return True

    def _attach_graspable_body(self, body_id, distance: float, contact_info: str):
        if self.gripper_grasp_mode == "keypoint":
            grasp_frame = self._gripper_grasp_frame_world()
            if grasp_frame is None:
                self._publish_gripper_status("gripper_closed: no_grasp_frame")
                return
            grasp_pos, grasp_orn = grasp_frame
            obj_pos, obj_orn = p.getBasePositionAndOrientation(
                body_id,
                physicsClientId=self.client,
            )
            inv_grasp_pos, inv_grasp_orn = p.invertTransform(grasp_pos, grasp_orn)
            local_pos, local_orn = p.multiplyTransforms(
                inv_grasp_pos,
                inv_grasp_orn,
                obj_pos,
                obj_orn,
            )
            if self.gripper_keypoint_snap_to_center:
                local_pos = tuple(
                    float(value)
                    for value in self.gripper_keypoint_snap_local_offset
                )
                _unused_pos, local_orn = p.multiplyTransforms(
                    inv_grasp_pos,
                    inv_grasp_orn,
                    grasp_pos,
                    obj_orn,
                )
                snapped_pos, snapped_orn = p.multiplyTransforms(
                    grasp_pos,
                    grasp_orn,
                    local_pos,
                    local_orn,
                )
                p.resetBasePositionAndOrientation(
                    body_id,
                    snapped_pos,
                    snapped_orn,
                    physicsClientId=self.client,
                )
                p.resetBaseVelocity(
                    body_id,
                    linearVelocity=[0.0, 0.0, 0.0],
                    angularVelocity=[0.0, 0.0, 0.0],
                    physicsClientId=self.client,
                )
                obj_pos, obj_orn = snapped_pos, snapped_orn
            self.resting_graspable_anchors.pop(body_id, None)
            self.resting_anchor_pause_until.pop(body_id, None)
            self.held_body_id = body_id
            self.held_parent_local_pos = list(local_pos)
            self.held_parent_local_orn = list(local_orn)
            self.held_lift_start_time = None
            self.held_lift_start_z = obj_pos[2]
            self.held_lower_start_time = None
            self.held_lower_start_z = None
            self.held_lower_target_z = None
            self.pending_grasp_start_time = None
            self.physical_grasp_lost_since = None
            self._publish_gripper_status(
                f"gripper_closed: grasped_body={body_id} mode=keypoint "
                f"distance={distance:.3f}m z={obj_pos[2]:.3f}m "
                f"snap_center={str(self.gripper_keypoint_snap_to_center).lower()} "
                f"contact={contact_info}"
            )
            return

        if self.gripper_grasp_mode == "physical":
            if self.gripper_physical_contact_guard:
                self._set_wsg50_contact_guard_targets()
            self.resting_graspable_anchors.pop(body_id, None)
            self.resting_anchor_pause_until.pop(body_id, None)
            self.held_body_id = body_id
            self.held_parent_local_pos = None
            self.held_parent_local_orn = None
            self.held_lift_start_time = time.monotonic()
            try:
                obj_pos, _obj_orn = p.getBasePositionAndOrientation(
                    body_id,
                    physicsClientId=self.client,
                )
                obj_z = obj_pos[2]
            except Exception:
                obj_z = float("nan")
            self.held_lift_start_z = obj_z
            self.held_lower_start_time = None
            self.held_lower_start_z = None
            self.held_lower_target_z = None
            self.pending_grasp_start_time = None
            self.last_physical_grasp_status_time = time.monotonic()
            self.physical_grasp_lost_since = None
            self._publish_gripper_status(
                f"gripper_closed: grasped_body={body_id} mode=physical "
                f"distance={distance:.3f}m z={obj_z:.3f}m contact={contact_info}"
            )
            return

        ee_state = p.getLinkState(
            self.robot_id,
            self.gripper_constraint_parent_link_index,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        ee_pos = ee_state[4]
        ee_orn = ee_state[5]
        obj_pos, obj_orn = p.getBasePositionAndOrientation(
            body_id,
            physicsClientId=self.client,
        )
        inv_ee_pos, inv_ee_orn = p.invertTransform(ee_pos, ee_orn)
        local_pos, local_orn = p.multiplyTransforms(
            inv_ee_pos,
            inv_ee_orn,
            obj_pos,
            obj_orn,
        )
        self.grasp_constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.gripper_constraint_parent_link_index,
            childBodyUniqueId=body_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0.0, 0.0, 0.0],
            parentFramePosition=local_pos,
            childFramePosition=[0.0, 0.0, 0.0],
            parentFrameOrientation=local_orn,
            childFrameOrientation=[0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        p.changeConstraint(
            self.grasp_constraint_id,
            maxForce=1000.0,
            erp=0.85,
            physicsClientId=self.client,
        )
        self.resting_graspable_anchors.pop(body_id, None)
        self.resting_anchor_pause_until.pop(body_id, None)
        self.held_body_id = body_id
        self.held_parent_local_pos = list(local_pos)
        self.held_parent_local_orn = list(local_orn)
        obj_z = obj_pos[2]
        self.held_lift_start_time = time.monotonic()
        self.held_lift_start_z = obj_z
        self.held_lower_start_time = None
        self.held_lower_start_z = None
        self.held_lower_target_z = None
        self.pending_grasp_start_time = None
        self._publish_gripper_status(
            f"gripper_closed: grasped_body={body_id} distance={distance:.3f}m "
            f"z={obj_z:.3f}m contact={contact_info}"
        )

    def _try_attach_nearest_graspable(self) -> bool:
        if self.grasp_constraint_id is not None:
            self._publish_gripper_status(f"gripper_closed: holding_body={self.held_body_id}")
            self.pending_grasp_start_time = None
            return True
        if self.gripper_grasp_mode == "keypoint" and self.held_body_id is not None:
            self._publish_gripper_status(
                f"gripper_closed: holding_body={self.held_body_id} mode=keypoint"
            )
            self.pending_grasp_start_time = None
            return True
        if self.gripper_grasp_mode == "physical" and self.held_body_id is not None:
            self._publish_gripper_status(
                f"gripper_closed: holding_body={self.held_body_id} mode=physical"
            )
            self.pending_grasp_start_time = None
            return True

        body_id, distance = self._nearest_graspable_body()
        if body_id is None or distance is None:
            self._publish_gripper_status("gripper_closed: no_graspable_objects")
            return False
        if distance > self.gripper_grasp_radius_m:
            self._publish_gripper_status(
                f"gripper_closed: no_object_close_enough distance={distance:.3f}m"
            )
            return False

        contact_ok, contact_info = self._grasp_has_physical_contact(body_id, distance)
        if not contact_ok:
            self._publish_gripper_status(
                f"gripper_closed: no_physical_contact body={body_id} {contact_info}"
            )
            return False

        self._attach_graspable_body(body_id, distance, contact_info)
        return True

    def _update_pending_grasp(self):
        if self.pending_grasp_start_time is None:
            return
        if self.held_body_id is not None or self.grasp_constraint_id is not None:
            self.pending_grasp_start_time = None
            return

        now = time.monotonic()
        elapsed = now - self.pending_grasp_start_time
        if elapsed < max(self.gripper_grasp_settle_sec, 0.0):
            return

        if self._try_attach_nearest_graspable():
            return

        timeout_sec = max(self.gripper_grasp_timeout_sec, 0.0)
        if timeout_sec <= 0.0 or elapsed < timeout_sec:
            return
        if now - self.pending_grasp_last_status_time > 0.5:
            self.pending_grasp_last_status_time = now
            self._publish_gripper_status(
                f"gripper_closed: waiting_physical_contact elapsed={elapsed:.2f}s"
            )

    def _update_physical_grasp_state(self):
        if self.gripper_grasp_mode != "physical":
            return
        if self.held_body_id is None:
            return
        if not self.enable_gripper:
            return
        now = time.monotonic()
        try:
            contact_counts = self._fingertip_contact_counts_to_body(self.held_body_id)
            aabb_min, _aabb_max = p.getAABB(
                self.held_body_id,
                physicsClientId=self.client,
            )
        except Exception:
            contact_counts = []
            aabb_min = [0.0, 0.0, self.table_surface_z]
        two_sided_contact = len(contact_counts) >= 2 and all(
            count > 0 for count in contact_counts[:2]
        )
        bottom_lift = float(aabb_min[2]) - self.table_surface_z
        if two_sided_contact:
            self.physical_grasp_lost_since = None
            if now - self.last_physical_grasp_status_time >= 0.5:
                self.last_physical_grasp_status_time = now
                self._publish_gripper_status(
                    f"gripper_closed: holding_body={self.held_body_id} mode=physical "
                    f"fingertip_contacts={contact_counts} bottom_lift={bottom_lift:.3f}m"
                )
            return

        if self.physical_grasp_lost_since is None:
            self.physical_grasp_lost_since = now
            return
        lost_elapsed = now - self.physical_grasp_lost_since
        if lost_elapsed < 0.45:
            return
        if now - self.last_physical_grasp_status_time >= 0.5:
            self.last_physical_grasp_status_time = now
            self._publish_gripper_status(
                f"gripper_closed: no_object physical_contact_lost "
                f"body={self.held_body_id} fingertip_contacts={contact_counts}"
            )
        self.held_body_id = None
        self.held_parent_local_pos = None
        self.held_parent_local_orn = None
        self.held_lift_start_time = None
        self.held_lift_start_z = None
        self.held_lower_start_time = None
        self.held_lower_start_z = None
        self.held_lower_target_z = None
        self.physical_grasp_lost_since = None

    def _attach_nearest_graspable(self):
        if self.wsg50_closed_joint_targets:
            self.wsg50_joint_targets = dict(self.wsg50_closed_joint_targets)

        if self.grasp_constraint_id is not None:
            self._publish_gripper_status(f"gripper_closed: holding_body={self.held_body_id}")
            return

        self.pending_grasp_start_time = time.monotonic()
        self.pending_grasp_last_status_time = 0.0
        self._publish_gripper_status("gripper_closing: waiting_fingertip_contact")

    def _lower_held_body(self):
        if self.held_body_id is None:
            self._publish_gripper_status("gripper_lower: no_object")
            return
        if self.gripper_grasp_mode == "keypoint":
            self._publish_gripper_status(
                f"gripper_lowering: body={self.held_body_id} mode=keypoint "
                "object_pose_locked_to_gripper_center"
            )
            return
        if self.gripper_grasp_mode == "physical":
            self._publish_gripper_status(
                f"gripper_lowering: body={self.held_body_id} mode=physical "
                "object_follows_only_by_contact"
            )
            return
        try:
            held_pos, _held_orn = p.getBasePositionAndOrientation(
                self.held_body_id,
                physicsClientId=self.client,
            )
            current_z = held_pos[2]
        except Exception:
            current_z = self.held_lift_start_z
        self.held_lower_start_time = time.monotonic()
        self.held_lower_start_z = current_z
        self.held_lower_target_z = (
            self.held_lift_start_z
            if self.held_lift_start_z is not None
            else self.table_surface_z + 0.018
        )
        self._publish_gripper_status(
            f"gripper_lowering: body={self.held_body_id} "
            f"from_z={current_z:.3f}m target_z={self.held_lower_target_z:.3f}m"
        )

    def _stabilize_keypoint_held_body_pose(self):
        if self.held_body_id is None:
            return
        if self.held_parent_local_pos is None or self.held_parent_local_orn is None:
            return
        grasp_frame = self._gripper_grasp_frame_world()
        if grasp_frame is None:
            return
        grasp_pos, grasp_orn = grasp_frame
        try:
            held_pos, held_orn = p.multiplyTransforms(
                grasp_pos,
                grasp_orn,
                self.held_parent_local_pos,
                self.held_parent_local_orn,
            )
            p.resetBasePositionAndOrientation(
                self.held_body_id,
                held_pos,
                held_orn,
                physicsClientId=self.client,
            )
            p.resetBaseVelocity(
                self.held_body_id,
                linearVelocity=[0.0, 0.0, 0.0],
                angularVelocity=[0.0, 0.0, 0.0],
                physicsClientId=self.client,
            )
        except Exception:
            return

    def _stabilize_held_body_pose(self):
        if self.gripper_grasp_mode == "keypoint":
            self._stabilize_keypoint_held_body_pose()
            return
        if self.gripper_grasp_mode == "physical":
            return
        if self.held_body_id is None:
            return
        if self.held_parent_local_pos is None or self.held_parent_local_orn is None:
            return
        try:
            parent_state = p.getLinkState(
                self.robot_id,
                self.gripper_constraint_parent_link_index,
                computeForwardKinematics=True,
                physicsClientId=self.client,
            )
            parent_pos = parent_state[4]
            parent_orn = parent_state[5]
            held_pos, held_orn = p.multiplyTransforms(
                parent_pos,
                parent_orn,
                self.held_parent_local_pos,
                self.held_parent_local_orn,
            )
            held_pos = list(held_pos)
            if (
                self.held_lower_start_time is not None
                and self.held_lower_start_z is not None
                and self.held_lower_target_z is not None
            ):
                elapsed = max(time.monotonic() - self.held_lower_start_time, 0.0)
                lower_speed = max(self.gripper_auto_lower_speed_mps, 0.0)
                commanded_drop = (
                    abs(self.held_lower_start_z - self.held_lower_target_z)
                    if lower_speed <= 0.0
                    else lower_speed * elapsed
                )
                held_pos[2] = max(
                    self.held_lower_target_z,
                    self.held_lower_start_z - commanded_drop,
                )
            elif self.held_lift_start_time is not None and self.held_lift_start_z is not None:
                elapsed = max(time.monotonic() - self.held_lift_start_time, 0.0)
                lift_height = max(self.gripper_auto_lift_height_m, 0.0)
                lift_speed = max(self.gripper_auto_lift_speed_mps, 0.0)
                commanded_lift = (
                    lift_height
                    if lift_speed <= 0.0
                    else min(lift_height, lift_speed * elapsed)
                )
                held_pos[2] = max(held_pos[2], self.held_lift_start_z + commanded_lift)
            p.resetBasePositionAndOrientation(
                self.held_body_id,
                held_pos,
                held_orn,
                physicsClientId=self.client,
            )
            p.resetBaseVelocity(
                self.held_body_id,
                linearVelocity=[0.0, 0.0, 0.0],
                angularVelocity=[0.0, 0.0, 0.0],
                physicsClientId=self.client,
            )
        except Exception:
            return

    def _stabilize_resting_graspables(self):
        if not self.stabilize_resting_graspables:
            return
        if not self.graspable_object_ids:
            return

        snap_margin = max(self.resting_snap_margin_m, 0.0)
        linear_th = max(self.resting_linear_speed_th_mps, 0.0)
        angular_th = max(self.resting_angular_speed_th_radps, 0.0)
        now = time.monotonic()

        for body_id in self.graspable_object_ids:
            if body_id == self.held_body_id:
                continue
            if now < self.resting_anchor_pause_until.get(body_id, 0.0):
                continue
            try:
                if self._gripper_interacting_with_body(body_id):
                    self.resting_graspable_anchors.pop(body_id, None)
                    continue

                anchor_pose = self.resting_graspable_anchors.get(body_id)
                if anchor_pose is not None:
                    anchor_pos, anchor_orn = anchor_pose
                    p.resetBasePositionAndOrientation(
                        body_id,
                        anchor_pos,
                        anchor_orn,
                        physicsClientId=self.client,
                    )
                    p.resetBaseVelocity(
                        body_id,
                        linearVelocity=[0.0, 0.0, 0.0],
                        angularVelocity=[0.0, 0.0, 0.0],
                        physicsClientId=self.client,
                    )
                    continue

                linear_velocity, angular_velocity = p.getBaseVelocity(
                    body_id,
                    physicsClientId=self.client,
                )
                linear_speed = math.sqrt(sum(float(value) ** 2 for value in linear_velocity))
                angular_speed = math.sqrt(sum(float(value) ** 2 for value in angular_velocity))
                if linear_speed > linear_th or angular_speed > angular_th:
                    continue

                aabb_min, _aabb_max = p.getAABB(body_id, physicsClientId=self.client)
                if abs(float(aabb_min[2]) - self.table_surface_z) > snap_margin:
                    continue

                self._anchor_resting_graspable(body_id)
            except Exception:
                continue

    def on_gripper_command(self, msg: String):
        command = msg.data.strip().lower()
        with self._lock:
            if command in ("open", "release"):
                self.gripper_width_m = self.gripper_open_width_m
                self._release_grasp()
            elif command in ("lower", "lower_held"):
                self._lower_held_body()
            elif command in ("close", "grasp"):
                self.gripper_width_m = self.gripper_closed_width_m
                self._attach_nearest_graspable()
            else:
                self._publish_gripper_status(f"gripper_ignored: unknown_command={command}")

    def _spawn_table(self):
        """Build a visible table above the plane instead of burying a URDF base."""
        table_x, table_y = self.table_center_xy
        size_x, size_y, thickness = self.table_size_xyz
        half_extents = [0.5 * size_x, 0.5 * size_y, 0.5 * thickness]

        top_center_z = self.table_surface_z - 0.5 * thickness
        top_id = self._create_box_body(
            half_extents=half_extents,
            rgba=[0.58, 0.42, 0.26, 1.0],
            base_position=[table_x, table_y, top_center_z],
            mass=0.0,
        )
        self.table_body_ids.append(top_id)

        top_bottom_z = top_center_z - 0.5 * thickness
        leg_height = max(top_bottom_z, 0.08)
        leg_half_height = 0.5 * leg_height
        leg_half_width = 0.5 * self.table_leg_width
        leg_inset_x = max(0.08, 0.5 * size_x - self.table_leg_width)
        leg_inset_y = max(0.08, 0.5 * size_y - self.table_leg_width)

        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                leg_id = self._create_box_body(
                    half_extents=[leg_half_width, leg_half_width, leg_half_height],
                    rgba=[0.35, 0.24, 0.16, 1.0],
                    base_position=[
                        table_x + sign_x * leg_inset_x,
                        table_y + sign_y * leg_inset_y,
                        leg_half_height,
                    ],
                    mass=0.0,
                )
                self.table_body_ids.append(leg_id)

    def _spawn_demo_cubes(self):
        """Spawn several colored cubes on the table so prompt-based selection is meaningful."""
        cube_half = 0.5 * self.cube_size
        table_x, table_y = self.table_center_xy
        cube_z = self.table_surface_z + cube_half + 0.002

        cube_specs = [
            {"name": "red cube", "offset": (-0.14, -0.10), "rgba": [0.92, 0.18, 0.18, 1.0]},
            {"name": "green cube", "offset": (0.12, -0.02), "rgba": [0.18, 0.72, 0.28, 1.0]},
            {"name": "blue cube", "offset": (-0.03, 0.11), "rgba": [0.18, 0.36, 0.92, 1.0]},
            {"name": "yellow cube", "offset": (0.16, 0.13), "rgba": [0.92, 0.78, 0.12, 1.0]},
        ]

        for cube_spec in cube_specs:
            cube_id = self._create_box_body(
                half_extents=[cube_half, cube_half, cube_half],
                rgba=cube_spec["rgba"],
                base_position=[
                    table_x + cube_spec["offset"][0],
                    table_y + cube_spec["offset"][1],
                    cube_z,
                ],
                mass=self.cube_mass,
            )
            self.object_ids.append(cube_id)

    def _spawn_household_objects(self):
        """Spawn a richer fixed tabletop scene for open-vocabulary prompt demos."""
        table_x, table_y = self.table_center_xy
        table_z = self.table_surface_z

        household_objects = [
            self._load_urdf_body(
                "objects/mug.urdf",
                base_position=[table_x - 0.18, table_y - 0.11, table_z + 0.045],
                base_euler=(0.0, 0.0, 0.35),
                use_fixed_base=True,
                global_scaling=0.85,
            ),
            self._create_cylinder_body(
                radius=0.022,
                height=0.18,
                rgba=[0.16, 0.63, 0.79, 1.0],
                base_position=[table_x + 0.02, table_y - 0.11, table_z + 0.09],
                mass=0.0,
            ),
            self._create_box_body(
                half_extents=[0.060, 0.085, 0.012],
                rgba=[0.18, 0.38, 0.82, 1.0],
                base_position=[table_x - 0.02, table_y + 0.04, table_z + 0.012],
                mass=0.0,
                base_orientation=p.getQuaternionFromEuler([0.0, 0.0, -0.28]),
            ),
            self._create_box_body(
                half_extents=[0.028, 0.050, 0.006],
                rgba=[0.10, 0.10, 0.12, 1.0],
                base_position=[table_x + 0.16, table_y + 0.02, table_z + 0.006],
                mass=0.0,
                base_orientation=p.getQuaternionFromEuler([0.0, 0.0, 0.10]),
            ),
            self._create_cylinder_body(
                radius=0.060,
                height=0.008,
                rgba=[0.90, 0.90, 0.92, 1.0],
                base_position=[table_x + 0.14, table_y + 0.13, table_z + 0.004],
                mass=0.0,
            ),
            self._create_cylinder_body(
                radius=0.045,
                height=0.030,
                rgba=[0.95, 0.95, 0.96, 1.0],
                base_position=[table_x - 0.12, table_y + 0.13, table_z + 0.015],
                mass=0.0,
            ),
            self._create_sphere_body(
                radius=0.028,
                rgba=[0.86, 0.16, 0.16, 1.0],
                base_position=[table_x + 0.04, table_y + 0.14, table_z + 0.028],
                mass=0.0,
            ),
        ]

        scissor_center_x = table_x + 0.20
        scissor_center_y = table_y - 0.14
        scissor_z = table_z + 0.006
        blade_rgba = [0.82, 0.84, 0.88, 1.0]
        handle_red = [0.92, 0.20, 0.20, 1.0]
        handle_blue = [0.18, 0.42, 0.92, 1.0]
        pivot_rgba = [0.18, 0.18, 0.20, 1.0]

        def _offset_xy(distance: float, angle_rad: float) -> tuple[float, float]:
            return (
                distance * math.cos(angle_rad),
                distance * math.sin(angle_rad),
            )

        blade_angles = (0.62, -0.62)
        handle_colors = (handle_red, handle_blue)
        for angle_rad, handle_rgba in zip(blade_angles, handle_colors):
            blade_dx, blade_dy = _offset_xy(0.022, angle_rad)
            household_objects.append(
                self._create_box_body(
                    half_extents=[0.065, 0.005, 0.003],
                    rgba=blade_rgba,
                    base_position=[
                        scissor_center_x + blade_dx,
                        scissor_center_y + blade_dy,
                        scissor_z,
                    ],
                    mass=0.0,
                    base_orientation=p.getQuaternionFromEuler([0.0, 0.0, angle_rad]),
                )
            )

            handle_bar_dx, handle_bar_dy = _offset_xy(-0.040, angle_rad)
            household_objects.append(
                self._create_box_body(
                    half_extents=[0.028, 0.007, 0.004],
                    rgba=handle_rgba,
                    base_position=[
                        scissor_center_x + handle_bar_dx,
                        scissor_center_y + handle_bar_dy,
                        scissor_z + 0.001,
                    ],
                    mass=0.0,
                    base_orientation=p.getQuaternionFromEuler([0.0, 0.0, angle_rad]),
                )
            )

            handle_ball_dx, handle_ball_dy = _offset_xy(-0.080, angle_rad)
            household_objects.append(
                self._create_sphere_body(
                    radius=0.018,
                    rgba=handle_rgba,
                    base_position=[
                        scissor_center_x + handle_ball_dx,
                        scissor_center_y + handle_ball_dy,
                        scissor_z + 0.010,
                    ],
                    mass=0.0,
                )
            )

        household_objects.append(
            self._create_cylinder_body(
                radius=0.010,
                height=0.012,
                rgba=pivot_rgba,
                base_position=[scissor_center_x, scissor_center_y, scissor_z + 0.002],
                mass=0.0,
            )
        )

        self.object_ids.extend(household_objects)

    def _spawn_medical_objects(self):
        """Spawn an abstract surgical-training tabletop scene for medical demos."""
        table_x, table_y = self.table_center_xy
        table_z = self.table_surface_z
        objects = []

        def _box(half_extents, rgba, offset_xy, z_offset, yaw=0.0):
            body_id = self._create_box_body(
                half_extents=half_extents,
                rgba=rgba,
                base_position=[
                    table_x + offset_xy[0],
                    table_y + offset_xy[1],
                    table_z + z_offset,
                ],
                mass=0.0,
                base_orientation=p.getQuaternionFromEuler([0.0, 0.0, yaw]),
            )
            objects.append(body_id)
            return body_id

        def _cylinder(radius, height, rgba, offset_xy, z_offset, yaw=0.0):
            body_id = self._create_cylinder_body(
                radius=radius,
                height=height,
                rgba=rgba,
                base_position=[
                    table_x + offset_xy[0],
                    table_y + offset_xy[1],
                    table_z + z_offset,
                ],
                mass=0.0,
                base_orientation=p.getQuaternionFromEuler([0.0, 0.0, yaw]),
            )
            objects.append(body_id)
            return body_id

        def _sphere(radius, rgba, offset_xy, z_offset):
            body_id = self._create_sphere_body(
                radius=radius,
                rgba=rgba,
                base_position=[
                    table_x + offset_xy[0],
                    table_y + offset_xy[1],
                    table_z + z_offset,
                ],
                mass=0.0,
            )
            objects.append(body_id)
            return body_id

        drape_rgba = [0.04, 0.42, 0.46, 1.0]
        tissue_rgba = [0.96, 0.62, 0.62, 1.0]
        tissue_edge_rgba = [0.78, 0.35, 0.38, 1.0]
        red_marker_rgba = [0.92, 0.08, 0.06, 1.0]
        warning_rgba = [0.96, 0.18, 0.12, 0.78]
        metal_rgba = [0.78, 0.82, 0.86, 1.0]
        dark_metal_rgba = [0.18, 0.20, 0.22, 1.0]
        blue_handle_rgba = [0.10, 0.32, 0.82, 1.0]
        gauze_rgba = [0.94, 0.94, 0.90, 1.0]
        gauze_line_rgba = [0.62, 0.68, 0.72, 1.0]
        suture_rgba = [0.08, 0.08, 0.10, 1.0]

        # Sterile drape and tray region.
        _box([0.335, 0.235, 0.003], drape_rgba, (0.0, 0.0), 0.003)
        _box([0.150, 0.085, 0.005], [0.72, 0.74, 0.76, 1.0], (0.145, 0.105), 0.008, 0.02)
        _box([0.150, 0.006, 0.012], metal_rgba, (0.145, 0.190), 0.020, 0.02)
        _box([0.150, 0.006, 0.012], metal_rgba, (0.145, 0.020), 0.020, 0.02)
        _box([0.006, 0.085, 0.012], metal_rgba, (-0.005, 0.105), 0.020, 0.02)
        _box([0.006, 0.085, 0.012], metal_rgba, (0.295, 0.105), 0.020, 0.02)

        # Tissue phantom with a marked entry point and a no-touch area.
        _box([0.155, 0.105, 0.012], tissue_rgba, (-0.085, -0.010), 0.018, -0.10)
        _box([0.158, 0.006, 0.004], tissue_edge_rgba, (-0.085, 0.096), 0.032, -0.10)
        _box([0.158, 0.006, 0.004], tissue_edge_rgba, (-0.085, -0.116), 0.032, -0.10)
        _cylinder(0.033, 0.005, dark_metal_rgba, (-0.110, 0.020), 0.034)
        _cylinder(0.021, 0.007, red_marker_rgba, (-0.110, 0.020), 0.040)
        _box([0.052, 0.034, 0.003], warning_rgba, (-0.010, -0.050), 0.037, 0.24)

        # Gauze pad with simple grid fibers.
        _box([0.070, 0.048, 0.006], gauze_rgba, (-0.230, 0.112), 0.012, -0.18)
        for line_offset in (-0.030, 0.0, 0.030):
            _box(
                [0.068, 0.002, 0.002],
                gauze_line_rgba,
                (-0.230, 0.112 + line_offset),
                0.019,
                -0.18,
            )
        for line_offset in (-0.040, 0.0, 0.040):
            _box(
                [0.002, 0.046, 0.002],
                gauze_line_rgba,
                (-0.230 + line_offset, 0.112),
                0.020,
                -0.18,
            )

        # Forceps/tweezers: two thin metallic arms with a small hinge.
        forceps_center = (-0.205, -0.118)
        for angle in (0.20, -0.20):
            dx = 0.050 * math.cos(angle)
            dy = 0.050 * math.sin(angle)
            _box(
                [0.080, 0.005, 0.003],
                metal_rgba,
                (forceps_center[0] + dx, forceps_center[1] + dy),
                0.014,
                angle,
            )
            _box(
                [0.018, 0.006, 0.004],
                dark_metal_rgba,
                (
                    forceps_center[0] - 0.072 * math.cos(angle),
                    forceps_center[1] - 0.072 * math.sin(angle),
                ),
                0.016,
                angle,
            )
        _cylinder(0.010, 0.010, dark_metal_rgba, forceps_center, 0.019)

        # Scalpel with blue handle and silver blade.
        _box([0.080, 0.012, 0.006], blue_handle_rgba, (0.118, -0.135), 0.014, -0.34)
        _box([0.040, 0.010, 0.004], metal_rgba, (0.192, -0.163), 0.015, -0.34)
        _box([0.020, 0.007, 0.004], [0.86, 0.88, 0.90, 1.0], (0.223, -0.175), 0.015, -0.34)

        # Curved suture needle approximated by short metallic arc segments.
        needle_center = (-0.078, 0.034)
        needle_radius = 0.052
        segment_half_length = 0.012
        for theta in (-0.95, -0.62, -0.30, 0.03, 0.36, 0.68):
            seg_x = needle_center[0] + needle_radius * math.cos(theta)
            seg_y = needle_center[1] + needle_radius * math.sin(theta)
            _box(
                [segment_half_length, 0.003, 0.003],
                metal_rgba,
                (seg_x, seg_y),
                0.051,
                theta + math.pi / 2.0,
            )
        _sphere(
            0.006,
            metal_rgba,
            (
                needle_center[0] + needle_radius * math.cos(-1.08),
                needle_center[1] + needle_radius * math.sin(-1.08),
            ),
            0.052,
        )
        _sphere(
            0.004,
            dark_metal_rgba,
            (
                needle_center[0] + needle_radius * math.cos(0.80),
                needle_center[1] + needle_radius * math.sin(0.80),
            ),
            0.052,
        )

        # Suture thread leading away from the needle.
        for index, (offset_xy, yaw) in enumerate(
            [
                ((-0.020, 0.092), 0.52),
                ((0.020, 0.116), 0.20),
                ((0.064, 0.120), -0.15),
            ]
        ):
            _box(
                [0.030 + 0.006 * index, 0.002, 0.002],
                suture_rgba,
                offset_xy,
                0.048,
                yaw,
            )

        self.object_ids.extend(objects)

    def _spawn_medical_grasp_asset_objects(self) -> bool:
        target_path = self._resolve_model_path(
            "assets/surgical_instruments_4729067/files/split/blue_scissors_component_1.stl"
        )
        if target_path is None:
            return False

        silver_rgba = [0.78, 0.80, 0.82, 1.0]
        split_specs = [
            (
                "left silver surgical instrument",
                "assets/surgical_instruments_4729067/files/split/blue_scissors_component_1.stl",
                [0.040, -0.060, 0.0],
                -0.10,
                0.055,
            ),
            (
                "right silver surgical instrument",
                "assets/surgical_instruments_4729067/files/split/blue_scissors_component_2.stl",
                [0.040, 0.060, 0.0],
                -0.10,
                0.035,
            ),
        ]
        loaded_count = 0
        for name, relative_path, position, yaw, mass in split_specs:
            model_path = self._resolve_model_path(relative_path)
            if model_path is None:
                self.get_logger().warning(f"[SCENE] missing split surgical asset: {relative_path}")
                continue
            body_ids = self._spawn_external_mesh_object(
                model_path,
                {
                    "name": name,
                    "position": position,
                    "position_frame": "table",
                    "rpy": [0.0, 0.0, yaw],
                    "scale": 0.0050,
                    "rgba": silver_rgba,
                    "fixed": False,
                    "mass": mass,
                    "placement": "center_on_table",
                    "recenter_to_bounds": True,
                    "collision": "box",
                    "collision_margin": [0.0015, 0.0015, 0.001],
                    "collision_min_half_z": 0.003,
                    "lateral_friction": 1.8,
                    "spinning_friction": 0.08,
                    "rolling_friction": 0.02,
                    "linear_damping": 0.45,
                    "angular_damping": 0.75,
                    "contact_stiffness": 8000,
                    "contact_damping": 120,
                },
            )
            self.object_ids.extend(body_ids)
            self.graspable_object_ids.extend(body_ids)
            loaded_count += len(body_ids)

        self.get_logger().info(
            f"[SCENE] loaded {loaded_count} split silver surgical instrument assets "
            "for medical_grasp"
        )
        return loaded_count > 0

    def _spawn_medical_sorting_tray(self):
        if not self.medical_sorting_tray_enabled:
            return

        table_x, table_y = self.table_center_xy
        table_z = self.table_surface_z
        offset_x, offset_y = self.medical_sorting_tray_offset_xy
        size_x = max(0.120, float(self.medical_sorting_tray_size_xy[0]))
        size_y = max(0.100, float(self.medical_sorting_tray_size_xy[1]))
        wall_height = max(0.012, float(self.medical_sorting_tray_wall_height_m))

        floor_half_z = 0.005
        wall_half_thickness = 0.005
        wall_half_height = 0.5 * wall_height
        tray_half_x = 0.5 * size_x
        tray_half_y = 0.5 * size_y
        wall_z = floor_half_z + wall_half_height

        tray_id = self._create_compound_box_body(
            half_extents_list=[
                [tray_half_x, tray_half_y, floor_half_z],
                [tray_half_x + wall_half_thickness, wall_half_thickness, wall_half_height],
                [tray_half_x + wall_half_thickness, wall_half_thickness, wall_half_height],
                [wall_half_thickness, tray_half_y + wall_half_thickness, wall_half_height],
                [wall_half_thickness, tray_half_y + wall_half_thickness, wall_half_height],
            ],
            rgba_list=[
                [0.96, 0.96, 0.94, 1.0],
                [1.00, 1.00, 0.98, 1.0],
                [1.00, 1.00, 0.98, 1.0],
                [1.00, 1.00, 0.98, 1.0],
                [1.00, 1.00, 0.98, 1.0],
            ],
            frame_positions=[
                [0.0, 0.0, 0.0],
                [0.0, tray_half_y + wall_half_thickness, wall_z],
                [0.0, -tray_half_y - wall_half_thickness, wall_z],
                [tray_half_x + wall_half_thickness, 0.0, wall_z],
                [-tray_half_x - wall_half_thickness, 0.0, wall_z],
            ],
            base_position=[
                table_x + offset_x,
                table_y + offset_y,
                table_z + floor_half_z + 0.001,
            ],
            mass=0.0,
        )
        self.object_ids.append(tray_id)
        self.get_logger().info(
            "[SCENE] added white sorting tray "
            f"center=({table_x + offset_x:.3f}, {table_y + offset_y:.3f}) "
            f"size=({size_x:.3f}, {size_y:.3f})"
        )

    def _spawn_medical_grasp_objects(self):
        """Spawn a sparse medical tabletop grasp scene."""
        self._spawn_medical_sorting_tray()
        if self._spawn_medical_grasp_asset_objects():
            return

        table_x, table_y = self.table_center_xy
        table_z = self.table_surface_z

        blue_rgba = [0.04, 0.26, 0.88, 1.0]
        blue_tip_rgba = [0.08, 0.12, 0.22, 1.0]
        metal_rgba = [0.78, 0.82, 0.86, 1.0]
        red_rgba = [0.78, 0.05, 0.08, 1.0]
        pivot_rgba = [0.16, 0.17, 0.19, 1.0]

        tweezer_yaw = -0.32
        arm_open_yaw = 0.075
        blue_tweezers = self._create_compound_box_body(
            half_extents_list=[
                [0.098, 0.0065, 0.006],
                [0.098, 0.0065, 0.006],
                [0.020, 0.020, 0.008],
                [0.020, 0.0045, 0.005],
                [0.020, 0.0045, 0.005],
            ],
            rgba_list=[
                blue_rgba,
                blue_rgba,
                blue_rgba,
                blue_tip_rgba,
                blue_tip_rgba,
            ],
            frame_positions=[
                [0.000, 0.011, 0.000],
                [0.000, -0.011, 0.000],
                [-0.090, 0.000, 0.001],
                [0.094, 0.018, 0.000],
                [0.094, -0.018, 0.000],
            ],
            frame_orientations=[
                p.getQuaternionFromEuler([0.0, 0.0, arm_open_yaw]),
                p.getQuaternionFromEuler([0.0, 0.0, -arm_open_yaw]),
                [0.0, 0.0, 0.0, 1.0],
                p.getQuaternionFromEuler([0.0, 0.0, arm_open_yaw]),
                p.getQuaternionFromEuler([0.0, 0.0, -arm_open_yaw]),
            ],
            base_position=[table_x + 0.075, table_y - 0.075, table_z + 0.018],
            mass=0.070,
            base_orientation=p.getQuaternionFromEuler([0.0, 0.0, tweezer_yaw]),
        )
        p.changeDynamics(
            blue_tweezers,
            -1,
            lateralFriction=1.2,
            spinningFriction=0.02,
            rollingFriction=0.002,
            restitution=0.0,
            physicsClientId=self.client,
        )
        self.object_ids.append(blue_tweezers)
        self.graspable_object_ids.append(blue_tweezers)

        scissors = self._create_compound_box_body(
            half_extents_list=[
                [0.080, 0.005, 0.004],
                [0.080, 0.005, 0.004],
                [0.040, 0.005, 0.005],
                [0.040, 0.005, 0.005],
                [0.028, 0.014, 0.006],
                [0.028, 0.014, 0.006],
                [0.012, 0.012, 0.006],
            ],
            rgba_list=[
                metal_rgba,
                metal_rgba,
                red_rgba,
                red_rgba,
                red_rgba,
                red_rgba,
                pivot_rgba,
            ],
            frame_positions=[
                [0.050, 0.010, 0.000],
                [0.050, -0.010, 0.000],
                [-0.050, 0.017, 0.000],
                [-0.050, -0.017, 0.000],
                [-0.105, 0.040, 0.000],
                [-0.105, -0.040, 0.000],
                [-0.010, 0.000, 0.002],
            ],
            frame_orientations=[
                p.getQuaternionFromEuler([0.0, 0.0, 0.13]),
                p.getQuaternionFromEuler([0.0, 0.0, -0.13]),
                p.getQuaternionFromEuler([0.0, 0.0, -0.38]),
                p.getQuaternionFromEuler([0.0, 0.0, 0.38]),
                p.getQuaternionFromEuler([0.0, 0.0, -0.15]),
                p.getQuaternionFromEuler([0.0, 0.0, 0.15]),
                [0.0, 0.0, 0.0, 1.0],
            ],
            base_position=[table_x - 0.165, table_y + 0.105, table_z + 0.015],
            mass=0.0 if self.medical_grasp_scissors_fixed else 0.060,
            base_orientation=p.getQuaternionFromEuler([0.0, 0.0, 0.58]),
        )
        p.changeDynamics(
            scissors,
            -1,
            lateralFriction=1.2,
            spinningFriction=0.02,
            rollingFriction=0.002,
            restitution=0.0,
            physicsClientId=self.client,
        )
        self.object_ids.append(scissors)
        if self.medical_grasp_scissors_graspable:
            if self.medical_grasp_scissors_fixed:
                self.get_logger().warning(
                    "[SCENE] medical scissors are fixed; not adding them to graspable objects"
                )
            else:
                self.graspable_object_ids.append(scissors)

    def _load_scene(self):
        p.loadURDF("plane.urdf", physicsClientId=self.client)
        self._spawn_table()

        if self.enable_gripper:
            if self.gripper_model not in ("kuka_parallel", "kuka_wsg50", "wsg50", "franka_hand"):
                self.get_logger().warning(
                    f"unsupported gripper_model='{self.gripper_model}', using kuka_parallel"
                )
                self.gripper_model = "kuka_parallel"
            if self.gripper_model == "franka_hand":
                self.robot_id = p.loadURDF(
                    "kuka_iiwa/model.urdf",
                    basePosition=[0.0, 0.0, 0.0],
                    useFixedBase=True,
                    physicsClientId=self.client,
                )
                self._spawn_franka_hand_gripper()
            else:
                self.robot_id = p.loadSDF(
                    "kuka_iiwa/kuka_with_gripper.sdf",
                    physicsClientId=self.client,
                )[0]
                self._configure_wsg50_gripper()
        else:
            self.robot_id = p.loadURDF(
                "kuka_iiwa/model.urdf",
                basePosition=[0.0, 0.0, 0.0],
                useFixedBase=True,
                physicsClientId=self.client,
            )

        if self.scene_preset == "household":
            self._spawn_household_objects()
        elif self.scene_preset == "medical":
            self._spawn_medical_objects()
        elif self.scene_preset == "medical_grasp":
            self._spawn_medical_grasp_objects()
        elif self.scene_preset == "mixed":
            self._spawn_demo_cubes()
            self._spawn_household_objects()
        elif self.scene_preset == "medical_mixed":
            self._spawn_demo_cubes()
            self._spawn_medical_objects()
        elif self.scene_preset == "empty":
            pass
        else:
            self._spawn_demo_cubes()

        self._spawn_external_scene_objects()

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

    def on_keypoint_overlay(self, msg: PointStamped):
        with self._lock:
            self.keypoint_overlay_raw_xyz = [
                float(msg.point.x),
                float(msg.point.y),
                float(msg.point.z),
            ]
            self.keypoint_overlay_frame_id = str(msg.header.frame_id or "")
            self.keypoint_overlay_received_time = time.time()

    def on_keypoint_overlay_valid(self, msg: Bool):
        with self._lock:
            self.keypoint_overlay_valid = bool(msg.data)

    def on_keypoint_overlay_plane_normal(self, msg: Vector3Stamped):
        with self._lock:
            self.keypoint_overlay_plane_normal_raw = [
                float(msg.vector.x),
                float(msg.vector.y),
                float(msg.vector.z),
            ]
            self.keypoint_overlay_plane_normal_frame_id = str(msg.header.frame_id or "")
            self.keypoint_overlay_plane_normal_received_time = time.time()

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
                    # Mirror the tuned position controller used by the
                    # non-RGBD sim so hover tracking converges promptly.
                    for array_index, joint_index in enumerate(self.joint_indices):
                        p.setJointMotorControl2(
                            self.robot_id,
                            joint_index,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=float(q_des[array_index]),
                            positionGain=float(self.position_gain),
                            velocityGain=float(self.velocity_gain),
                            force=float(self.position_force),
                            maxVelocity=float(self.max_velocity),
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

                self._drive_wsg50_gripper()
                self._drive_franka_hand_gripper()
                p.stepSimulation(physicsClientId=self.client)
                self._update_pending_grasp()
                self._update_physical_grasp_state()
                self._stabilize_held_body_pose()
                self._stabilize_resting_graspables()
                self._update_keypoint_overlay()

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
