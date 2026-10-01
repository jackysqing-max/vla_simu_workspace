#!/usr/bin/env python3
"""Core iiwa execution node backed by a single PyBullet simulation."""

import json
import math
import threading
from pathlib import Path
from typing import List, Tuple

import pybullet as p
import pybullet_data
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped, Vector3Stamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, Header, Int8, String
from tf2_ros import TransformBroadcaster

from pybullet_ros2_sim.camera_tf_utils import view_matrix_to_world_optical_tf
from pybullet_ros2_sim.depth_utils import depth_buffer_to_meters
from pybullet_ros2_sim.ros_msg_utils import (
    depth32f_to_imgmsg,
    make_camera_info,
    rgb_to_imgmsg,
)
from pybullet_ros2_sim.sim_camera import SimCameraConfig, SimRGBDCamera


def min_jerk_s(u: float) -> float:
    """Map progress in ``[0, 1]`` to a smooth min-jerk interpolation factor."""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


class IiwaPybulletSim(Node):
    """
    Expose the main `/iiwa7/*` control contract on top of PyBullet.

    The node owns the physics world and accepts two kinds of commands:
    - `/iiwa7/joint_desired` in position mode
    - `/iiwa7/joint_torques` in torque mode

    A short GOTO phase is used at startup so all upstream nodes can lock to a
    stable initial pose before the normal control loop begins.
    """

    MODE_FREE = 0
    MODE_POSITION = 1
    MODE_TORQUE = 2
    MODE_NAMES = {
        MODE_FREE: "FREE",
        MODE_POSITION: "POSITION",
        MODE_TORQUE: "TORQUE",
    }

    def __init__(self):
        super().__init__("iiwa_pybullet_sim_node")

        self.n = 7
        self.joint_indices = list(range(self.n))
        self.joint_names = [f"iiwa_joint_{index + 1}" for index in range(self.n)]

        self._lock = threading.Lock()
        self.cb_sub = ReentrantCallbackGroup()
        # PyBullet is not thread-safe. Serializing both timers prevents slow GUI
        # frames from allowing overlapping physics steps.
        self.cb_timer = MutuallyExclusiveCallbackGroup()

        self._declare_parameters()
        self._load_parameters()

        # Cache the latest upstream commands. The sim reads these values on each
        # physics tick so callbacks stay lightweight.
        self.q_cmd = self.init_q[:]
        self.tau_cmd = [0.0] * self.n
        self.have_des = False
        self.have_tau = False
        self._warned_no_tau = False
        self._init_done_sent = False
        self.latest_rcm_point = None
        self.latest_rcm_point_time = 0.0
        self.latest_tip_point = None
        self.latest_tip_point_time = 0.0
        self.latest_port_point = None
        self.latest_port_axis = None
        self.latest_surface_axis = None
        self.latest_port_candidates = []
        self.latest_port_candidates_time = 0.0
        self.latest_port_pose_time = 0.0
        self.latest_approach_selection = None

        self.pub_js = self.create_publisher(JointState, "/iiwa7/joint_states", 10)
        self.camera = None
        self.tf_broadcaster = None
        self.last_rgbd_time = float("-inf")
        self.rgbd_runtime_enabled = self.enable_rgbd_camera
        if self.enable_rgbd_camera:
            qos_img = QoSProfile(depth=1)
            qos_img.reliability = ReliabilityPolicy.BEST_EFFORT
            qos_img.durability = DurabilityPolicy.VOLATILE
            self.pub_color = self.create_publisher(
                Image,
                self.rgbd_color_topic,
                qos_img,
            )
            self.pub_depth = self.create_publisher(
                Image,
                self.rgbd_depth_topic,
                qos_img,
            )
            self.pub_camera_info = self.create_publisher(
                CameraInfo,
                self.rgbd_camera_info_topic,
                qos_img,
            )
            self.tf_broadcaster = TransformBroadcaster(self)
            self.sub_rgbd_enable = self.create_subscription(
                Bool,
                self.rgbd_enable_topic,
                self.on_rgbd_enable,
                10,
                callback_group=self.cb_sub,
            )

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_init_done = self.create_publisher(Bool, "/iiwa7/init_done", qos_latch)

        self.sub_qdes = self.create_subscription(
            Float64MultiArray,
            "/iiwa7/joint_desired",
            self.on_q_des,
            10,
            callback_group=self.cb_sub,
        )
        self.sub_tau = self.create_subscription(
            Float64MultiArray,
            "/iiwa7/joint_torques",
            self.on_tau,
            10,
            callback_group=self.cb_sub,
        )
        self.sub_mode = self.create_subscription(
            Int8,
            "/iiwa7/control_mode",
            self.on_control_mode,
            10,
            callback_group=self.cb_sub,
        )
        self.sub_enable_position = self.create_subscription(
            Bool,
            "/iiwa7/enable_position",
            self.on_enable_position_compat,
            10,
            callback_group=self.cb_sub,
        )
        if self.show_rcm_debug_markers:
            self.sub_rcm_point = self.create_subscription(
                PointStamped,
                self.rcm_point_topic,
                self.on_rcm_point,
                10,
                callback_group=self.cb_sub,
            )
            self.sub_tip_point = self.create_subscription(
                PointStamped,
                self.rcm_tip_point_topic,
                self.on_rcm_tip_point,
                10,
                callback_group=self.cb_sub,
            )
        if self.show_port_detection_overlay:
            self.sub_port_point = self.create_subscription(
                PointStamped,
                self.locked_port_point_topic,
                self.on_locked_port_point,
                qos_latch,
                callback_group=self.cb_sub,
            )
            self.sub_port_axis = self.create_subscription(
                Vector3Stamped,
                self.locked_port_axis_topic,
                self.on_locked_port_axis,
                qos_latch,
                callback_group=self.cb_sub,
            )
            self.sub_surface_axis = self.create_subscription(
                Vector3Stamped,
                self.locked_surface_axis_topic,
                self.on_locked_surface_axis,
                qos_latch,
                callback_group=self.cb_sub,
            )
            self.sub_port_candidates = self.create_subscription(
                String,
                self.port_candidates_topic,
                self.on_port_candidates,
                qos_latch,
                callback_group=self.cb_sub,
            )
            self.sub_approach_selection = self.create_subscription(
                String,
                self.approach_selection_topic,
                self.on_approach_selection,
                qos_latch,
                callback_group=self.cb_sub,
            )

        self.client = p.connect(p.GUI if self.gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0, physicsClientId=self.client)
        p.setTimeStep(self.dt, physicsClientId=self.client)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=1,
            numSolverIterations=200,
            physicsClientId=self.client,
        )
        p.setGravity(0.0, 0.0, -9.8, physicsClientId=self.client)

        p.loadURDF("plane.urdf", physicsClientId=self.client)
        self.table_id = p.loadURDF(
            "table/table.urdf",
            basePosition=[
                self.table_x_m,
                self.table_y_m,
                self.table_top_z_m - 0.625,
            ],
            useFixedBase=True,
            physicsClientId=self.client,
        )
        self.robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            useFixedBase=True,
            physicsClientId=self.client,
        )
        if self.reset_to_init_on_start:
            for array_index, joint_index in enumerate(self.joint_indices):
                p.resetJointState(
                    self.robot_id,
                    joint_index,
                    self.init_q[array_index],
                    targetVelocity=0.0,
                    physicsClientId=self.client,
                )
        self._setup_rcm_phantom()
        self._configure_gui_view()
        self._setup_rcm_debug_overlay()
        if self.enable_rgbd_camera:
            self.camera = SimRGBDCamera(self.client, self.rgbd_camera_config)

        self.q_start = [
            float(
                p.getJointState(
                    self.robot_id,
                    joint_index,
                    physicsClientId=self.client,
                )[0]
            )
            for joint_index in self.joint_indices
        ]

        self.phase = "GOTO" if self.use_goto else "RUN"
        self.t_goto_start = self.get_clock().now().nanoseconds * 1e-9
        self.goto_settle_count = 0

        self.control_mode = self.initial_control_mode
        self._apply_mode_change(None, self.control_mode)

        self.timer = self.create_timer(self.dt, self.step, callback_group=self.cb_timer)
        self._dbg_timer = self.create_timer(
            1.0,
            self.debug_status,
            callback_group=self.cb_timer,
        )
        self.get_logger().info("Iiwa PyBullet sim node started.")
        self.get_logger().info(
            "Control mode topic: /iiwa7/control_mode (0=FREE, 1=POSITION, 2=TORQUE)"
        )
        if self.show_rcm_debug_markers:
            self.get_logger().info(
                "[RCM_OVERLAY] enabled: red=RCM point, yellow=desired tip, "
                "green=actual tool-tip trace"
            )
        if self.show_port_detection_overlay:
            self.get_logger().info(
                "[PORT_OVERLAY] enabled: yellow sphere=exact locked port point, "
                "green spheres=candidate centers, cyan ring/arrow=locked inward port axis"
            )
        if self.enable_rgbd_camera:
            self.get_logger().info(
                "[RGBD] enabled: "
                f"{self.rgbd_width}x{self.rgbd_height} @ {self.rgbd_hz:.1f} Hz, "
                f"color={self.rgbd_color_topic}, depth={self.rgbd_depth_topic}"
            )
        if self.show_rcm_tool:
            self.get_logger().info(
                "[RCM_TOOL] enabled: "
                f"length={self.rcm_tool_length_m:.3f} m, "
                f"diameter={2.0 * self.rcm_tool_radius_m:.3f} m, "
                "axis=link-7 +Z"
            )
        if self.dvrk_lnd_body_id is not None:
            self.get_logger().info(
                "[DVRK_LND] visual-only Large Needle Driver 420006 loaded: "
                f"{self.dvrk_lnd_urdf_path}"
            )
        if self.rcm_phantom_body_id is not None:
            self.get_logger().info(
                "[RCM_PHANTOM] enabled: "
                f"position=({self.rcm_phantom_x_m:.3f}, "
                f"{self.rcm_phantom_y_m:.3f}, {self.rcm_phantom_z_m:.3f}) m, "
                f"scale={self.rcm_phantom_scale:.4f}"
            )
        self.get_logger().info(
            "[TABLE] "
            f"center=({self.table_x_m:.3f}, {self.table_y_m:.3f}) m, "
            f"top_z={self.table_top_z_m:.3f} m"
        )

    def _declare_parameters(self):
        self.declare_parameter("gui", True)
        self.declare_parameter("use_goto", True)
        self.declare_parameter("init_q", [0.0, 0.6, 0.0, -1.2, 0.0, 1.0, 0.0])
        self.declare_parameter("initial_control_mode", self.MODE_TORQUE)
        self.declare_parameter("reset_to_init_on_start", False)

        self.declare_parameter("goto_duration", 2.0)
        self.declare_parameter("goto_use_reset", False)
        self.declare_parameter("goto_force", 120.0)
        self.declare_parameter("goto_pos_gain", 0.20)
        self.declare_parameter("goto_vel_gain", 1.00)
        self.declare_parameter("goto_max_vel", 1.0)
        self.declare_parameter("goto_pos_tolerance_rad", 0.015)
        self.declare_parameter("goto_vel_tolerance_rad_s", 0.05)
        self.declare_parameter("goto_settle_cycles", 5)

        self.declare_parameter("track_force", 120.0)
        self.declare_parameter("track_pos_gain", 0.25)
        self.declare_parameter("track_vel_gain", 1.00)
        self.declare_parameter("track_max_vel", 2.0)

        self.declare_parameter("sim_hz", 200.0)
        self.declare_parameter("tau_limit", 200.0)
        self.declare_parameter("table_x_m", 0.6)
        self.declare_parameter("table_y_m", 0.0)
        self.declare_parameter("table_top_z_m", 0.005)
        self.declare_parameter("clean_gui", False)
        self.declare_parameter("camera_distance_m", 1.35)
        self.declare_parameter("camera_yaw_deg", 42.0)
        self.declare_parameter("camera_pitch_deg", -28.0)
        self.declare_parameter("camera_target", [0.55, 0.0, 0.30])
        self.declare_parameter("enable_rgbd_camera", False)
        self.declare_parameter("rgbd_hz", 8.0)
        self.declare_parameter("rgbd_width", 640)
        self.declare_parameter("rgbd_height", 480)
        self.declare_parameter("rgbd_fov_y_deg", 58.0)
        self.declare_parameter("rgbd_near_m", 0.02)
        self.declare_parameter("rgbd_far_m", 2.0)
        self.declare_parameter("rgbd_target", [0.701726, 0.0, 0.36])
        self.declare_parameter("rgbd_distance_m", 0.55)
        self.declare_parameter("rgbd_yaw_deg", 90.0)
        self.declare_parameter("rgbd_pitch_deg", -65.0)
        self.declare_parameter("rgbd_roll_deg", 0.0)
        self.declare_parameter(
            "rgbd_color_topic",
            "/sim/camera/color/image_raw",
        )
        self.declare_parameter(
            "rgbd_depth_topic",
            "/sim/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "rgbd_camera_info_topic",
            "/sim/camera/color/camera_info",
        )
        self.declare_parameter(
            "rgbd_optical_frame",
            "sim_camera_color_optical_frame",
        )
        self.declare_parameter(
            "rgbd_enable_topic",
            "/sim/camera/enabled",
        )
        self.declare_parameter("show_rcm_debug_markers", True)
        self.declare_parameter("rcm_point_topic", "/rcm_virtual_fixtures/rcm_point")
        self.declare_parameter("rcm_tip_point_topic", "/rcm_virtual_fixtures/tip_point")
        self.declare_parameter("rcm_marker_radius_m", 0.018)
        self.declare_parameter("rcm_tip_marker_radius_m", 0.012)
        self.declare_parameter("ee_trace_point_radius_m", 0.006)
        self.declare_parameter("ee_trace_min_dist_m", 0.008)
        self.declare_parameter("ee_trace_max_points", 260)
        self.declare_parameter("rcm_trace_min_inserted_depth_m", 0.0)
        self.declare_parameter("rcm_marker_max_age_sec", 2.0)
        self.declare_parameter("show_port_detection_overlay", False)
        self.declare_parameter(
            "locked_port_point_topic",
            "/rcm_virtual_fixtures/locked_port_point",
        )
        self.declare_parameter(
            "locked_port_axis_topic",
            "/rcm_virtual_fixtures/locked_port_axis",
        )
        self.declare_parameter(
            "locked_surface_axis_topic",
            "/vlm_rcm/locked_surface_axis",
        )
        self.declare_parameter(
            "port_candidates_topic",
            "/vlm_rcm/hole_candidates",
        )
        self.declare_parameter(
            "approach_selection_topic",
            "/rcm_virtual_fixtures/approach_selection_json",
        )
        self.declare_parameter("port_overlay_max_age_sec", 2.0)
        self.declare_parameter("port_overlay_ring_radius_m", 0.010)
        self.declare_parameter("port_overlay_axis_outside_m", 0.050)
        self.declare_parameter("port_overlay_axis_inside_m", 0.090)
        self.declare_parameter("show_rcm_tool", False)
        self.declare_parameter("rcm_tool_length_m", 0.220)
        self.declare_parameter("rcm_tool_radius_m", 0.006)
        self.declare_parameter("rcm_tool_mount_radius_m", 0.014)
        self.declare_parameter("rcm_tool_mount_length_m", 0.030)
        self.declare_parameter("show_dvrk_lnd_gripper", False)
        self.declare_parameter("dvrk_lnd_urdf_path", "")
        self.declare_parameter("dvrk_lnd_jaw_angle_rad", 0.45)
        self.declare_parameter("dvrk_lnd_tip_offset_m", 0.026)
        self.declare_parameter("trace_rcm_tool_tip", True)
        self.declare_parameter("show_rcm_phantom", False)
        self.declare_parameter("rcm_phantom_mesh_path", "")
        self.declare_parameter("rcm_phantom_scale", 0.001)
        self.declare_parameter("rcm_phantom_x_m", 0.701726)
        self.declare_parameter("rcm_phantom_y_m", 0.0)
        self.declare_parameter("rcm_phantom_z_m", 0.240701)
        self.declare_parameter("rcm_phantom_roll_deg", 0.0)
        self.declare_parameter("rcm_phantom_pitch_deg", 0.0)
        self.declare_parameter("rcm_phantom_yaw_deg", 0.0)
        self.declare_parameter("rcm_phantom_alpha", 0.42)
        self.declare_parameter("rcm_phantom_collision", True)

    def _load_parameters(self):
        self.gui = bool(self.get_parameter("gui").value)
        self.use_goto = bool(self.get_parameter("use_goto").value)
        self.initial_control_mode = int(
            self.get_parameter("initial_control_mode").value
        )
        if self.initial_control_mode not in self.MODE_NAMES:
            self.get_logger().warning(
                "initial_control_mode must be 0, 1, or 2; using TORQUE"
            )
            self.initial_control_mode = self.MODE_TORQUE
        self.reset_to_init_on_start = bool(
            self.get_parameter("reset_to_init_on_start").value
        )

        init_q = [float(value) for value in self.get_parameter("init_q").value]
        if len(init_q) < self.n:
            init_q += [0.0] * (self.n - len(init_q))
        self.init_q = init_q[:self.n]

        self.goto_duration = float(self.get_parameter("goto_duration").value)
        self.goto_use_reset = bool(self.get_parameter("goto_use_reset").value)
        self.goto_force = float(self.get_parameter("goto_force").value)
        self.goto_pos_gain = float(self.get_parameter("goto_pos_gain").value)
        self.goto_vel_gain = float(self.get_parameter("goto_vel_gain").value)
        self.goto_max_vel = float(self.get_parameter("goto_max_vel").value)
        self.goto_pos_tolerance_rad = max(
            float(self.get_parameter("goto_pos_tolerance_rad").value),
            0.001,
        )
        self.goto_vel_tolerance_rad_s = max(
            float(self.get_parameter("goto_vel_tolerance_rad_s").value),
            0.001,
        )
        self.goto_settle_cycles = max(
            int(self.get_parameter("goto_settle_cycles").value),
            1,
        )

        self.track_force = float(self.get_parameter("track_force").value)
        self.track_pos_gain = float(self.get_parameter("track_pos_gain").value)
        self.track_vel_gain = float(self.get_parameter("track_vel_gain").value)
        self.track_max_vel = float(self.get_parameter("track_max_vel").value)

        sim_hz = float(self.get_parameter("sim_hz").value)
        self.dt = 1.0 / max(sim_hz, 1e-6)
        self.tau_limit = float(self.get_parameter("tau_limit").value)
        self.table_x_m = float(self.get_parameter("table_x_m").value)
        self.table_y_m = float(self.get_parameter("table_y_m").value)
        self.table_top_z_m = float(
            self.get_parameter("table_top_z_m").value
        )
        self.clean_gui = bool(self.get_parameter("clean_gui").value)
        self.camera_distance_m = max(
            float(self.get_parameter("camera_distance_m").value),
            0.1,
        )
        self.camera_yaw_deg = float(self.get_parameter("camera_yaw_deg").value)
        self.camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        camera_target = [
            float(value) for value in self.get_parameter("camera_target").value
        ]
        if len(camera_target) < 3:
            camera_target += [0.0] * (3 - len(camera_target))
        self.camera_target = camera_target[:3]
        self.enable_rgbd_camera = bool(
            self.get_parameter("enable_rgbd_camera").value
        )
        self.rgbd_hz = max(
            float(self.get_parameter("rgbd_hz").value),
            0.2,
        )
        self.rgbd_width = max(
            int(self.get_parameter("rgbd_width").value),
            64,
        )
        self.rgbd_height = max(
            int(self.get_parameter("rgbd_height").value),
            48,
        )
        self.rgbd_color_topic = str(
            self.get_parameter("rgbd_color_topic").value
        )
        self.rgbd_depth_topic = str(
            self.get_parameter("rgbd_depth_topic").value
        )
        self.rgbd_camera_info_topic = str(
            self.get_parameter("rgbd_camera_info_topic").value
        )
        self.rgbd_optical_frame = str(
            self.get_parameter("rgbd_optical_frame").value
        )
        self.rgbd_enable_topic = str(
            self.get_parameter("rgbd_enable_topic").value
        )
        rgbd_target = [
            float(value)
            for value in self.get_parameter("rgbd_target").value
        ]
        if len(rgbd_target) < 3:
            rgbd_target += [0.0] * (3 - len(rgbd_target))
        self.rgbd_camera_config = SimCameraConfig(
            width=self.rgbd_width,
            height=self.rgbd_height,
            fov_y_deg=float(self.get_parameter("rgbd_fov_y_deg").value),
            near=float(self.get_parameter("rgbd_near_m").value),
            far=float(self.get_parameter("rgbd_far_m").value),
            target_pos=tuple(rgbd_target[:3]),
            distance=float(self.get_parameter("rgbd_distance_m").value),
            yaw_deg=float(self.get_parameter("rgbd_yaw_deg").value),
            pitch_deg=float(self.get_parameter("rgbd_pitch_deg").value),
            roll_deg=float(self.get_parameter("rgbd_roll_deg").value),
        )
        self.show_rcm_debug_markers = bool(
            self.get_parameter("show_rcm_debug_markers").value
        )
        self.rcm_point_topic = str(self.get_parameter("rcm_point_topic").value)
        self.rcm_tip_point_topic = str(self.get_parameter("rcm_tip_point_topic").value)
        self.rcm_marker_radius_m = max(
            float(self.get_parameter("rcm_marker_radius_m").value),
            0.001,
        )
        self.rcm_tip_marker_radius_m = max(
            float(self.get_parameter("rcm_tip_marker_radius_m").value),
            0.001,
        )
        self.ee_trace_point_radius_m = max(
            float(self.get_parameter("ee_trace_point_radius_m").value),
            0.001,
        )
        self.ee_trace_min_dist_m = max(
            float(self.get_parameter("ee_trace_min_dist_m").value),
            0.001,
        )
        self.ee_trace_max_points = max(
            int(self.get_parameter("ee_trace_max_points").value),
            1,
        )
        self.rcm_trace_min_inserted_depth_m = max(
            float(
                self.get_parameter(
                    "rcm_trace_min_inserted_depth_m"
                ).value
            ),
            0.0,
        )
        self.rcm_marker_max_age_sec = max(
            float(self.get_parameter("rcm_marker_max_age_sec").value),
            0.0,
        )
        self.show_port_detection_overlay = bool(
            self.get_parameter("show_port_detection_overlay").value
        )
        self.locked_port_point_topic = str(
            self.get_parameter("locked_port_point_topic").value
        )
        self.locked_port_axis_topic = str(
            self.get_parameter("locked_port_axis_topic").value
        )
        self.locked_surface_axis_topic = str(
            self.get_parameter("locked_surface_axis_topic").value
        )
        self.port_candidates_topic = str(
            self.get_parameter("port_candidates_topic").value
        )
        self.approach_selection_topic = str(
            self.get_parameter("approach_selection_topic").value
        )
        self.port_overlay_max_age_sec = max(
            float(self.get_parameter("port_overlay_max_age_sec").value),
            0.0,
        )
        self.port_overlay_ring_radius_m = max(
            float(self.get_parameter("port_overlay_ring_radius_m").value),
            0.002,
        )
        self.port_overlay_axis_outside_m = max(
            float(self.get_parameter("port_overlay_axis_outside_m").value),
            0.005,
        )
        self.port_overlay_axis_inside_m = max(
            float(self.get_parameter("port_overlay_axis_inside_m").value),
            0.005,
        )
        self.show_rcm_tool = bool(self.get_parameter("show_rcm_tool").value)
        self.rcm_tool_length_m = max(
            float(self.get_parameter("rcm_tool_length_m").value),
            0.02,
        )
        self.rcm_tool_radius_m = max(
            float(self.get_parameter("rcm_tool_radius_m").value),
            0.001,
        )
        self.rcm_tool_mount_radius_m = max(
            float(self.get_parameter("rcm_tool_mount_radius_m").value),
            self.rcm_tool_radius_m,
        )
        self.rcm_tool_mount_length_m = min(
            max(float(self.get_parameter("rcm_tool_mount_length_m").value), 0.005),
            self.rcm_tool_length_m,
        )
        self.show_dvrk_lnd_gripper = bool(
            self.get_parameter("show_dvrk_lnd_gripper").value
        )
        self.dvrk_lnd_urdf_path = str(
            self.get_parameter("dvrk_lnd_urdf_path").value
        ).strip()
        self.dvrk_lnd_jaw_angle_rad = min(
            max(float(self.get_parameter("dvrk_lnd_jaw_angle_rad").value), 0.0),
            1.4,
        )
        self.dvrk_lnd_tip_offset_m = min(
            max(float(self.get_parameter("dvrk_lnd_tip_offset_m").value), 0.005),
            0.45 * self.rcm_tool_length_m,
        )
        self.trace_rcm_tool_tip = bool(
            self.get_parameter("trace_rcm_tool_tip").value
        )
        self.show_rcm_phantom = bool(
            self.get_parameter("show_rcm_phantom").value
        )
        self.rcm_phantom_mesh_path = str(
            self.get_parameter("rcm_phantom_mesh_path").value
        ).strip()
        self.rcm_phantom_scale = max(
            float(self.get_parameter("rcm_phantom_scale").value),
            1e-6,
        )
        self.rcm_phantom_x_m = float(
            self.get_parameter("rcm_phantom_x_m").value
        )
        self.rcm_phantom_y_m = float(
            self.get_parameter("rcm_phantom_y_m").value
        )
        self.rcm_phantom_z_m = float(
            self.get_parameter("rcm_phantom_z_m").value
        )
        self.rcm_phantom_roll_deg = float(
            self.get_parameter("rcm_phantom_roll_deg").value
        )
        self.rcm_phantom_pitch_deg = float(
            self.get_parameter("rcm_phantom_pitch_deg").value
        )
        self.rcm_phantom_yaw_deg = float(
            self.get_parameter("rcm_phantom_yaw_deg").value
        )
        self.rcm_phantom_alpha = min(
            max(float(self.get_parameter("rcm_phantom_alpha").value), 0.05),
            1.0,
        )
        self.rcm_phantom_collision = bool(
            self.get_parameter("rcm_phantom_collision").value
        )

    def _setup_rcm_phantom(self):
        self.rcm_phantom_body_id = None
        if not self.show_rcm_phantom:
            return

        mesh_path = Path(self.rcm_phantom_mesh_path).expanduser()
        if not mesh_path.is_file():
            self.get_logger().warning(
                f"[RCM_PHANTOM] mesh not found: {mesh_path}"
            )
            return

        scale = [self.rcm_phantom_scale] * 3
        position = [
            self.rcm_phantom_x_m,
            self.rcm_phantom_y_m,
            self.rcm_phantom_z_m,
        ]
        orientation = p.getQuaternionFromEuler(
            [
                self.rcm_phantom_roll_deg * 3.141592653589793 / 180.0,
                self.rcm_phantom_pitch_deg * 3.141592653589793 / 180.0,
                self.rcm_phantom_yaw_deg * 3.141592653589793 / 180.0,
            ]
        )

        try:
            visual_shape = p.createVisualShape(
                p.GEOM_MESH,
                fileName=str(mesh_path),
                meshScale=scale,
                rgbaColor=[0.66, 0.78, 0.84, self.rcm_phantom_alpha],
                specularColor=[0.25, 0.25, 0.25],
                physicsClientId=self.client,
            )
            collision_shape = -1
            if self.rcm_phantom_collision:
                collision_shape = p.createCollisionShape(
                    p.GEOM_MESH,
                    fileName=str(mesh_path),
                    meshScale=scale,
                    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                    physicsClientId=self.client,
                )
            self.rcm_phantom_body_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=collision_shape,
                baseVisualShapeIndex=visual_shape,
                basePosition=position,
                baseOrientation=orientation,
                physicsClientId=self.client,
            )
            p.changeDynamics(
                self.rcm_phantom_body_id,
                -1,
                lateralFriction=0.8,
                restitution=0.0,
                physicsClientId=self.client,
            )
        except Exception as exc:
            self.rcm_phantom_body_id = None
            self.get_logger().error(
                f"[RCM_PHANTOM] failed to load {mesh_path}: {exc}"
            )

    def _configure_gui_view(self):
        if not self.gui:
            return
        if self.clean_gui:
            p.configureDebugVisualizer(
                p.COV_ENABLE_GUI,
                0,
                physicsClientId=self.client,
            )
        p.configureDebugVisualizer(
            p.COV_ENABLE_SHADOWS,
            1,
            physicsClientId=self.client,
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=self.camera_distance_m,
            cameraYaw=self.camera_yaw_deg,
            cameraPitch=self.camera_pitch_deg,
            cameraTargetPosition=self.camera_target,
            physicsClientId=self.client,
        )

    def _make_rgbd_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.rgbd_optical_frame
        return header

    def _publish_rgbd(self):
        if self.camera is None:
            return
        rgb, depth_buffer, _segmentation = self.camera.render()
        depth_m = depth_buffer_to_meters(
            depth_buffer,
            near=self.rgbd_camera_config.near,
            far=self.rgbd_camera_config.far,
        )
        header = self._make_rgbd_header()
        fx, fy, cx, cy = self.camera.intrinsics()
        self.pub_color.publish(rgb_to_imgmsg(rgb, header))
        self.pub_depth.publish(depth32f_to_imgmsg(depth_m, header))
        self.pub_camera_info.publish(
            make_camera_info(
                width=self.rgbd_width,
                height=self.rgbd_height,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                header=header,
            )
        )

        translation, quat = view_matrix_to_world_optical_tf(
            self.camera.view_matrix()
        )
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = self.rgbd_optical_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quat[0])
        transform.transform.rotation.y = float(quat[1])
        transform.transform.rotation.z = float(quat[2])
        transform.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(transform)

    def _publish_rgbd_if_due(self, now_sec: float):
        if (
            self.camera is None
            or not self.rgbd_runtime_enabled
            or now_sec - self.last_rgbd_time < 1.0 / self.rgbd_hz
        ):
            return
        self._publish_rgbd()
        self.last_rgbd_time = now_sec

    def on_rgbd_enable(self, msg: Bool):
        enabled = bool(msg.data) and self.enable_rgbd_camera
        if enabled == self.rgbd_runtime_enabled:
            return
        self.rgbd_runtime_enabled = enabled
        if enabled:
            self.last_rgbd_time = float("-inf")
        self.get_logger().info(
            f"[RGBD] runtime enabled={str(enabled).lower()}"
        )

    def on_q_des(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            with self._lock:
                self.q_cmd = [float(value) for value in msg.data[:self.n]]
                self.have_des = True

    def on_tau(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            with self._lock:
                self.tau_cmd = [float(value) for value in msg.data[:self.n]]
                self.have_tau = True

    def on_control_mode(self, msg: Int8):
        mode = int(msg.data)
        if mode not in self.MODE_NAMES:
            self.get_logger().warning(
                f"[MODE] Invalid control_mode={mode}, expected one of 0/1/2."
            )
            return
        self.set_mode(mode)

    def on_enable_position_compat(self, msg: Bool):
        # Compatibility shim for older launch files that only toggled position
        # control on and off.
        if bool(msg.data):
            self.set_mode(self.MODE_POSITION)
        elif self.control_mode == self.MODE_POSITION:
            self.set_mode(self.MODE_FREE)

    def on_rcm_point(self, msg: PointStamped):
        point = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        with self._lock:
            self.latest_rcm_point = point
            self.latest_rcm_point_time = self.get_clock().now().nanoseconds * 1e-9

    def on_rcm_tip_point(self, msg: PointStamped):
        point = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        with self._lock:
            self.latest_tip_point = point
            self.latest_tip_point_time = self.get_clock().now().nanoseconds * 1e-9

    def on_locked_port_point(self, msg: PointStamped):
        point = [float(msg.point.x), float(msg.point.y), float(msg.point.z)]
        with self._lock:
            self.latest_port_point = point
            self.latest_port_pose_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )

    def on_locked_port_axis(self, msg: Vector3Stamped):
        axis = [
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
        ]
        with self._lock:
            self.latest_port_axis = axis
            self.latest_port_pose_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )

    def on_locked_surface_axis(self, msg: Vector3Stamped):
        axis = [
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
        ]
        with self._lock:
            self.latest_surface_axis = axis
            self.latest_port_pose_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )

    def on_port_candidates(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        selected_id = payload.get("selected_id")
        candidates = []
        for hole in payload.get("holes", []):
            center = hole.get("center_world", None)
            if not isinstance(center, list) or len(center) != 3:
                continue
            try:
                candidate = {
                    "id": int(hole.get("id", -1)),
                    "center_world": [float(value) for value in center],
                    "score": float(hole.get("selection_score", 0.0)),
                    "rank": int(hole.get("selection_rank", 0)),
                    "selected": (
                        selected_id is not None
                        and int(hole.get("id", -1)) == int(selected_id)
                    ),
                }
            except (TypeError, ValueError):
                continue
            candidates.append(candidate)
        with self._lock:
            self.latest_port_candidates = candidates
            self.latest_port_candidates_time = (
                self.get_clock().now().nanoseconds * 1e-9
            )

    def on_approach_selection(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self.latest_approach_selection = payload

    def set_mode(self, new_mode: int):
        with self._lock:
            old_mode = self.control_mode
            if new_mode == old_mode:
                return
            self.control_mode = new_mode
        self._apply_mode_change(old_mode, new_mode)

    def _apply_mode_change(self, old_mode: int, new_mode: int):
        if new_mode in (self.MODE_FREE, self.MODE_TORQUE):
            self.release_motors()
        self.get_logger().info(
            f"[MODE] {self.MODE_NAMES.get(old_mode, 'INIT')} -> {self.MODE_NAMES[new_mode]}"
        )

    def release_motors(self):
        """Disable the default motor controllers before free or torque control."""
        for joint_index in self.joint_indices:
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0,
                physicsClientId=self.client,
            )

    def _get_q_qd_tau(self) -> Tuple[List[float], List[float], List[float]]:
        q, qd, tau = [], [], []
        for joint_index in self.joint_indices:
            state = p.getJointState(
                self.robot_id,
                joint_index,
                physicsClientId=self.client,
            )
            q.append(float(state[0]))
            qd.append(float(state[1]))
            tau.append(float(state[3]) if state[3] is not None else 0.0)
        return q, qd, tau

    def publish_joint_states(self):
        """Publish the simulated state after each physics step."""
        q, qd, tau = self._get_q_qd_tau()

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.name = list(self.joint_names)
        msg.position = q
        msg.velocity = qd
        msg.effort = tau
        self.pub_js.publish(msg)

    def _setup_rcm_debug_overlay(self):
        self.rcm_marker_body_id = None
        self.rcm_tip_marker_body_id = None
        self.rcm_label_id = -1
        self.rcm_tip_label_id = -1
        self.ee_trace_visual_shape_id = None
        self.ee_trace_body_ids = []
        self.ee_trace_last_point = None
        self.rcm_trace_started = False
        self.rcm_tool_shaft_body_id = None
        self.rcm_tool_mount_body_id = None
        self.rcm_tool_tip_body_id = None
        self.dvrk_lnd_body_id = None
        self.dvrk_lnd_joint_indices = {}
        self.port_ring_line_ids = []
        self.port_axis_line_ids = []
        self.port_surface_axis_line_ids = []
        self.approach_cone_line_ids = []
        self.approach_selected_axis_line_id = -1
        self.approach_cone_label_id = -1
        self.approach_overlay_signature = None
        self.port_candidate_label_ids = []
        self.port_center_marker_body_id = None
        self.port_candidate_marker_visual_shape_id = None
        self.port_candidate_marker_body_ids = []
        self.port_label_id = -1
        self.port_surface_label_id = -1
        self.port_overlay_signature = None
        self.port_overlay_visible = False

        if (
            not self.show_rcm_debug_markers
            and not self.show_rcm_tool
            and not self.show_dvrk_lnd_gripper
            and not self.show_port_detection_overlay
        ):
            return

        hidden = [0.0, 0.0, -10.0]
        if self.show_rcm_debug_markers:
            rcm_visual = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=self.rcm_marker_radius_m,
                rgbaColor=[1.0, 0.05, 0.05, 0.95],
                physicsClientId=self.client,
            )
            tip_visual = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=self.rcm_tip_marker_radius_m,
                rgbaColor=[1.0, 0.86, 0.05, 0.90],
                physicsClientId=self.client,
            )
            self.ee_trace_visual_shape_id = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=self.ee_trace_point_radius_m,
                rgbaColor=[0.05, 0.95, 0.35, 0.82],
                physicsClientId=self.client,
            )
            self.rcm_marker_body_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=rcm_visual,
                basePosition=hidden,
                physicsClientId=self.client,
            )
            self.rcm_tip_marker_body_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=tip_visual,
                basePosition=hidden,
                physicsClientId=self.client,
            )

        if self.show_rcm_tool:
            shaft_length = self.rcm_tool_length_m
            if self.show_dvrk_lnd_gripper:
                shaft_length -= self.dvrk_lnd_tip_offset_m
            shaft_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=self.rcm_tool_radius_m,
                length=shaft_length,
                rgbaColor=[0.68, 0.72, 0.76, 1.0],
                specularColor=[0.9, 0.9, 0.9],
                physicsClientId=self.client,
            )
            mount_visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=self.rcm_tool_mount_radius_m,
                length=self.rcm_tool_mount_length_m,
                rgbaColor=[0.16, 0.19, 0.22, 1.0],
                specularColor=[0.45, 0.45, 0.45],
                physicsClientId=self.client,
            )
            self.rcm_tool_shaft_body_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=shaft_visual,
                basePosition=hidden,
                physicsClientId=self.client,
            )
            self.rcm_tool_mount_body_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=mount_visual,
                basePosition=hidden,
                physicsClientId=self.client,
            )
            if not self.show_dvrk_lnd_gripper:
                tip_visual = p.createVisualShape(
                    p.GEOM_SPHERE,
                    radius=1.15 * self.rcm_tool_radius_m,
                    rgbaColor=[0.82, 0.85, 0.88, 1.0],
                    specularColor=[1.0, 1.0, 1.0],
                    physicsClientId=self.client,
                )
                self.rcm_tool_tip_body_id = p.createMultiBody(
                    baseMass=0.0,
                    baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=tip_visual,
                    basePosition=hidden,
                    physicsClientId=self.client,
                )

        if self.show_port_detection_overlay:
            port_center_visual = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.006,
                rgbaColor=[1.0, 0.86, 0.05, 1.0],
                specularColor=[0.9, 0.9, 0.9],
                physicsClientId=self.client,
            )
            self.port_candidate_marker_visual_shape_id = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.0035,
                rgbaColor=[0.0, 0.95, 0.2, 0.82],
                specularColor=[0.4, 0.4, 0.4],
                physicsClientId=self.client,
            )
            self.port_center_marker_body_id = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=port_center_visual,
                basePosition=hidden,
                physicsClientId=self.client,
            )

        if self.show_dvrk_lnd_gripper:
            urdf_path = Path(self.dvrk_lnd_urdf_path).expanduser()
            if not urdf_path.is_file():
                self.get_logger().warning(
                    f"[DVRK_LND] URDF not found: {urdf_path}"
                )
                return
            try:
                self.dvrk_lnd_body_id = p.loadURDF(
                    str(urdf_path.resolve()),
                    basePosition=hidden,
                    useFixedBase=True,
                    flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                    physicsClientId=self.client,
                )
                for joint_index in range(
                    p.getNumJoints(
                        self.dvrk_lnd_body_id,
                        physicsClientId=self.client,
                    )
                ):
                    info = p.getJointInfo(
                        self.dvrk_lnd_body_id,
                        joint_index,
                        physicsClientId=self.client,
                    )
                    name = info[1].decode("utf-8")
                    self.dvrk_lnd_joint_indices[name] = joint_index
            except p.error as exc:
                self.dvrk_lnd_body_id = None
                self.get_logger().error(f"[DVRK_LND] failed to load URDF: {exc}")

    def _hide_body(self, body_id):
        if body_id is None:
            return
        p.resetBasePositionAndOrientation(
            body_id,
            [0.0, 0.0, -10.0],
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )

    def _set_marker_body(self, body_id, point):
        if body_id is None:
            return
        p.resetBasePositionAndOrientation(
            body_id,
            point,
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )

    def _set_marker_label(self, label_id, text, point, color):
        return p.addUserDebugText(
            text,
            [point[0], point[1], point[2] + 0.035],
            textColorRGB=color,
            textSize=1.0,
            lifeTime=0.0,
            replaceItemUniqueId=label_id,
            physicsClientId=self.client,
        )

    def _set_debug_line(self, line_id, start, end, color, width):
        return p.addUserDebugLine(
            start,
            end,
            lineColorRGB=color,
            lineWidth=width,
            lifeTime=0.0,
            replaceItemUniqueId=line_id,
            physicsClientId=self.client,
        )

    def _hide_port_detection_overlay(self):
        if not self.port_overlay_visible:
            return
        hidden = [0.0, 0.0, -10.0]
        for line_id in (
            self.port_ring_line_ids
            + self.port_axis_line_ids
            + self.port_surface_axis_line_ids
        ):
            self._set_debug_line(
                line_id,
                hidden,
                hidden,
                [0.0, 0.0, 0.0],
                1.0,
            )
        self.port_label_id = self._set_marker_label(
            self.port_label_id,
            "",
            hidden,
            [0.0, 0.9, 1.0],
        )
        self.port_surface_label_id = self._set_marker_label(
            self.port_surface_label_id,
            "",
            hidden,
            [1.0, 0.0, 1.0],
        )
        self._hide_body(self.port_center_marker_body_id)
        for body_id in self.port_candidate_marker_body_ids:
            self._hide_body(body_id)
        for label_id in self.port_candidate_label_ids:
            self._set_marker_label(
                label_id,
                "",
                hidden,
                [0.0, 0.9, 0.2],
            )
        self.port_candidate_label_ids = []
        self.port_overlay_signature = None
        self.port_overlay_visible = False

    def _ensure_port_candidate_marker_bodies(self, count):
        if self.port_candidate_marker_visual_shape_id is None:
            return
        hidden = [0.0, 0.0, -10.0]
        while len(self.port_candidate_marker_body_ids) < count:
            self.port_candidate_marker_body_ids.append(
                p.createMultiBody(
                    baseMass=0.0,
                    baseCollisionShapeIndex=-1,
                    baseVisualShapeIndex=self.port_candidate_marker_visual_shape_id,
                    basePosition=hidden,
                    physicsClientId=self.client,
                )
            )

    def _update_port_candidate_score_labels(self, candidates):
        hidden = [0.0, 0.0, -10.0]
        self._ensure_port_candidate_marker_bodies(len(candidates or []))
        new_label_ids = []
        for index, candidate in enumerate(candidates or []):
            label_id = (
                self.port_candidate_label_ids[index]
                if index < len(self.port_candidate_label_ids)
                else -1
            )
            point = candidate["center_world"]
            color = [1.0, 0.85, 0.0] if candidate.get("selected", False) else [0.0, 0.95, 0.2]
            text = f"H{candidate['id']} {candidate['score']:.2f}"
            if index < len(self.port_candidate_marker_body_ids):
                self._set_marker_body(self.port_candidate_marker_body_ids[index], point)
            new_label_ids.append(
                self._set_marker_label(label_id, text, point, color)
            )
        for body_id in self.port_candidate_marker_body_ids[len(candidates or []):]:
            self._hide_body(body_id)
        for label_id in self.port_candidate_label_ids[len(new_label_ids):]:
            self._set_marker_label(
                label_id,
                "",
                hidden,
                [0.0, 0.0, 0.0],
            )
        self.port_candidate_label_ids = new_label_ids

    def _update_port_detection_overlay(
        self,
        point,
        axis,
        surface_axis=None,
        candidates=None,
    ):
        axis_norm = sum(value * value for value in axis) ** 0.5
        if axis_norm < 1e-8:
            self._hide_port_detection_overlay()
            return
        axis = [value / axis_norm for value in axis]
        if surface_axis is not None:
            surface_norm = sum(value * value for value in surface_axis) ** 0.5
            if surface_norm >= 1e-8:
                surface_axis = [value / surface_norm for value in surface_axis]
            else:
                surface_axis = None
        signature_values = point + axis
        if surface_axis is not None:
            signature_values += surface_axis
        for candidate in candidates or []:
            signature_values += [
                float(candidate["id"]),
                *candidate["center_world"],
                float(candidate["score"]),
                1.0 if candidate.get("selected", False) else 0.0,
            ]
        signature = tuple(round(value, 5) for value in signature_values)
        if self.port_overlay_visible and signature == self.port_overlay_signature:
            return

        def cross(a, b):
            return [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]

        def unit(vector):
            norm = max(sum(value * value for value in vector) ** 0.5, 1e-9)
            return [value / norm for value in vector]

        reference = [0.0, 0.0, 1.0]
        if abs(sum(axis[index] * reference[index] for index in range(3))) > 0.9:
            reference = [0.0, 1.0, 0.0]
        basis_1 = unit(cross(axis, reference))
        basis_2 = unit(cross(axis, basis_1))
        cyan = [0.0, 0.9, 1.0]
        magenta = [1.0, 0.0, 1.0]
        self._set_marker_body(self.port_center_marker_body_id, point)

        ring_points = []
        ring_segments = 24
        for index in range(ring_segments):
            angle = 2.0 * 3.141592653589793 * index / ring_segments
            ring_points.append(
                [
                    point[dimension]
                    + self.port_overlay_ring_radius_m
                    * (
                        math.cos(angle) * basis_1[dimension]
                        + math.sin(angle) * basis_2[dimension]
                    )
                    for dimension in range(3)
                ]
            )
        new_ring_ids = []
        for index in range(ring_segments):
            line_id = (
                self.port_ring_line_ids[index]
                if index < len(self.port_ring_line_ids)
                else -1
            )
            new_ring_ids.append(
                self._set_debug_line(
                    line_id,
                    ring_points[index],
                    ring_points[(index + 1) % ring_segments],
                    cyan,
                    3.0,
                )
            )
        self.port_ring_line_ids = new_ring_ids

        outside = [
            point[index] - self.port_overlay_axis_outside_m * axis[index]
            for index in range(3)
        ]
        inside = [
            point[index] + self.port_overlay_axis_inside_m * axis[index]
            for index in range(3)
        ]
        arrow_back = [
            inside[index] - 0.018 * axis[index]
            for index in range(3)
        ]
        arrow_left = [
            arrow_back[index] + 0.007 * basis_1[index]
            for index in range(3)
        ]
        arrow_right = [
            arrow_back[index] - 0.007 * basis_1[index]
            for index in range(3)
        ]
        axis_segments = [
            (outside, inside),
            (inside, arrow_left),
            (inside, arrow_right),
        ]
        new_axis_ids = []
        for index, (start, end) in enumerate(axis_segments):
            line_id = (
                self.port_axis_line_ids[index]
                if index < len(self.port_axis_line_ids)
                else -1
            )
            new_axis_ids.append(
                self._set_debug_line(line_id, start, end, cyan, 4.0)
            )
        self.port_axis_line_ids = new_axis_ids
        if surface_axis is not None:
            surface_outside = [
                point[index]
                - 0.75 * self.port_overlay_axis_outside_m * surface_axis[index]
                for index in range(3)
            ]
            surface_inside = [
                point[index]
                + 0.75 * self.port_overlay_axis_inside_m * surface_axis[index]
                for index in range(3)
            ]
            surface_arrow_back = [
                surface_inside[index] - 0.014 * surface_axis[index]
                for index in range(3)
            ]
            surface_arrow_left = [
                surface_arrow_back[index] + 0.006 * basis_2[index]
                for index in range(3)
            ]
            surface_arrow_right = [
                surface_arrow_back[index] - 0.006 * basis_2[index]
                for index in range(3)
            ]
            surface_segments = [
                (surface_outside, surface_inside),
                (surface_inside, surface_arrow_left),
                (surface_inside, surface_arrow_right),
            ]
            new_surface_axis_ids = []
            for index, (start, end) in enumerate(surface_segments):
                line_id = (
                    self.port_surface_axis_line_ids[index]
                    if index < len(self.port_surface_axis_line_ids)
                    else -1
                )
                new_surface_axis_ids.append(
                    self._set_debug_line(line_id, start, end, magenta, 3.0)
                )
            self.port_surface_axis_line_ids = new_surface_axis_ids
        elif self.port_surface_axis_line_ids:
            hidden = [0.0, 0.0, -10.0]
            for line_id in self.port_surface_axis_line_ids:
                self._set_debug_line(line_id, hidden, hidden, [0.0, 0.0, 0.0], 1.0)
            self.port_surface_axis_line_ids = []
        label_point = [
            point[index] + 0.018 * basis_2[index]
            for index in range(3)
        ]
        self.port_label_id = self._set_marker_label(
            self.port_label_id,
            "PORT CENTER",
            label_point,
            cyan,
        )
        if surface_axis is not None:
            surface_label_point = [
                point[index] - 0.024 * basis_2[index]
                for index in range(3)
            ]
            self.port_surface_label_id = self._set_marker_label(
                self.port_surface_label_id,
                "SURFACE AXIS",
                surface_label_point,
                magenta,
            )
        else:
            hidden = [0.0, 0.0, -10.0]
            self.port_surface_label_id = self._set_marker_label(
                self.port_surface_label_id,
                "",
                hidden,
                magenta,
            )
        self._update_port_candidate_score_labels(candidates or [])
        self.port_overlay_signature = signature
        self.port_overlay_visible = True

    def _update_approach_cone_overlay(self, payload):
        """Draw the outward-opening cone and selected inward tool axis."""
        if not isinstance(payload, dict):
            return
        point = payload.get("port_point")
        center_axis = payload.get("center_axis")
        if not isinstance(point, list) or not isinstance(center_axis, list):
            return
        if len(point) != 3 or len(center_axis) != 3:
            return
        status = str(payload.get("status", ""))
        half_angle_deg = float(payload.get("cone_half_angle_deg", 0.0))
        selected_axis = payload.get("selected_axis")
        signature_values = [*point, *center_axis, half_angle_deg]
        if isinstance(selected_axis, list) and len(selected_axis) == 3:
            signature_values += selected_axis
        signature_values.append(1.0 if status == "SELECTED" else 0.0)
        signature = tuple(round(float(value), 5) for value in signature_values)
        if signature == self.approach_overlay_signature:
            return

        def unit(vector):
            norm = max(sum(value * value for value in vector) ** 0.5, 1e-9)
            return [value / norm for value in vector]

        def cross(first, second):
            return [
                first[1] * second[2] - first[2] * second[1],
                first[2] * second[0] - first[0] * second[2],
                first[0] * second[1] - first[1] * second[0],
            ]

        inward = unit(center_axis)
        outward = [-value for value in inward]
        reference = [0.0, 0.0, 1.0]
        if abs(sum(outward[index] * reference[index] for index in range(3))) > 0.9:
            reference = [1.0, 0.0, 0.0]
        basis_1 = unit(cross(reference, outward))
        basis_2 = unit(cross(outward, basis_1))
        length = 0.11
        tilt = math.radians(half_angle_deg)
        boundary = []
        segment_count = 16
        for index in range(segment_count):
            angle = 2.0 * math.pi * index / segment_count
            direction = [
                math.cos(tilt) * outward[dimension]
                + math.sin(tilt)
                * (
                    math.cos(angle) * basis_1[dimension]
                    + math.sin(angle) * basis_2[dimension]
                )
                for dimension in range(3)
            ]
            boundary.append(
                [point[dimension] + length * direction[dimension] for dimension in range(3)]
            )
        new_ids = []
        cone_color = [1.0, 0.65, 0.0] if status == "SELECTED" else [1.0, 0.1, 0.1]
        for index in range(segment_count):
            for start, end in (
                (point, boundary[index]),
                (boundary[index], boundary[(index + 1) % segment_count]),
            ):
                old_id = (
                    self.approach_cone_line_ids[len(new_ids)]
                    if len(new_ids) < len(self.approach_cone_line_ids)
                    else -1
                )
                new_ids.append(
                    self._set_debug_line(old_id, start, end, cone_color, 1.5)
                )
        self.approach_cone_line_ids = new_ids

        if isinstance(selected_axis, list) and len(selected_axis) == 3:
            selected_outside = [
                point[index] - length * float(selected_axis[index])
                for index in range(3)
            ]
            self.approach_selected_axis_line_id = self._set_debug_line(
                self.approach_selected_axis_line_id,
                point,
                selected_outside,
                [0.2, 0.3, 1.0],
                5.0,
            )
        label = (
            f"APPROACH CONE {half_angle_deg:.1f} deg"
            if status == "SELECTED"
            else "NO REACHABLE APPROACH"
        )
        self.approach_cone_label_id = self._set_marker_label(
            self.approach_cone_label_id,
            label,
            boundary[0],
            cone_color,
        )
        self.approach_overlay_signature = signature

    def _get_ee_pose_and_tool_tip(self):
        link_state = p.getLinkState(
            self.robot_id,
            self.n - 1,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        ee_point = [float(value) for value in link_state[4]]
        ee_quat = [float(value) for value in link_state[5]]
        tool_tip, _ = p.multiplyTransforms(
            ee_point,
            ee_quat,
            [0.0, 0.0, self.rcm_tool_length_m],
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        return ee_point, ee_quat, [float(value) for value in tool_tip]

    def _update_rcm_tool_visual(self, ee_point, ee_quat, tool_tip):
        if not self.show_rcm_tool and self.dvrk_lnd_body_id is None:
            return

        shaft_length = self.rcm_tool_length_m
        if self.dvrk_lnd_body_id is not None:
            shaft_length -= self.dvrk_lnd_tip_offset_m
        shaft_center, _ = p.multiplyTransforms(
            ee_point,
            ee_quat,
            [0.0, 0.0, 0.5 * shaft_length],
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        mount_center, _ = p.multiplyTransforms(
            ee_point,
            ee_quat,
            [0.0, 0.0, 0.5 * self.rcm_tool_mount_length_m],
            [0.0, 0.0, 0.0, 1.0],
            physicsClientId=self.client,
        )
        if self.show_rcm_tool:
            p.resetBasePositionAndOrientation(
                self.rcm_tool_shaft_body_id,
                shaft_center,
                ee_quat,
                physicsClientId=self.client,
            )
            p.resetBasePositionAndOrientation(
                self.rcm_tool_mount_body_id,
                mount_center,
                ee_quat,
                physicsClientId=self.client,
            )
            if self.rcm_tool_tip_body_id is not None:
                p.resetBasePositionAndOrientation(
                    self.rcm_tool_tip_body_id,
                    tool_tip,
                    ee_quat,
                    physicsClientId=self.client,
                )

        if self.dvrk_lnd_body_id is not None:
            gripper_root, _ = p.multiplyTransforms(
                ee_point,
                ee_quat,
                [0.0, 0.0, shaft_length],
                [0.0, 0.0, 0.0, 1.0],
                physicsClientId=self.client,
            )
            p.resetBasePositionAndOrientation(
                self.dvrk_lnd_body_id,
                gripper_root,
                ee_quat,
                physicsClientId=self.client,
            )
            for name in ("wrist_pitch", "wrist_yaw"):
                if name in self.dvrk_lnd_joint_indices:
                    p.resetJointState(
                        self.dvrk_lnd_body_id,
                        self.dvrk_lnd_joint_indices[name],
                        0.0,
                        physicsClientId=self.client,
                    )
            for name in ("jaw_1", "jaw_2"):
                if name in self.dvrk_lnd_joint_indices:
                    p.resetJointState(
                        self.dvrk_lnd_body_id,
                        self.dvrk_lnd_joint_indices[name],
                        self.dvrk_lnd_jaw_angle_rad,
                        physicsClientId=self.client,
                    )

    def _update_rcm_debug_overlay(self, record_trace=True):
        if (
            not self.show_rcm_debug_markers
            and not self.show_rcm_tool
            and self.dvrk_lnd_body_id is None
            and not self.show_port_detection_overlay
        ):
            return

        ee_point, ee_quat, tool_tip = self._get_ee_pose_and_tool_tip()
        self._update_rcm_tool_visual(ee_point, ee_quat, tool_tip)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.show_port_detection_overlay:
            with self._lock:
                port_point = (
                    None
                    if self.latest_port_point is None
                    else list(self.latest_port_point)
                )
                port_axis = (
                    None
                    if self.latest_port_axis is None
                    else list(self.latest_port_axis)
                )
                surface_axis = (
                    None
                    if self.latest_surface_axis is None
                    else list(self.latest_surface_axis)
                )
                port_candidates = list(self.latest_port_candidates)
                approach_selection = self.latest_approach_selection
                port_candidates_time = self.latest_port_candidates_time
                port_time = self.latest_port_pose_time
            candidates_fresh = (
                self.port_overlay_max_age_sec <= 0.0
                or now_sec - port_candidates_time <= self.port_overlay_max_age_sec
            )
            port_pose_fresh = (
                port_point is not None
                and port_axis is not None
                and (
                    self.port_overlay_max_age_sec <= 0.0
                    or now_sec - port_time <= self.port_overlay_max_age_sec
                )
            )
            if port_pose_fresh:
                self._update_port_detection_overlay(
                    port_point,
                    port_axis,
                    surface_axis,
                    port_candidates if candidates_fresh else [],
                )
            else:
                self._hide_port_detection_overlay()
            if approach_selection is not None:
                self._update_approach_cone_overlay(approach_selection)

        if not self.show_rcm_debug_markers:
            return

        with self._lock:
            rcm_point = None if self.latest_rcm_point is None else list(self.latest_rcm_point)
            rcm_time = self.latest_rcm_point_time
            tip_point = None if self.latest_tip_point is None else list(self.latest_tip_point)
            tip_time = self.latest_tip_point_time

        if (
            rcm_point is not None
            and (
                self.rcm_marker_max_age_sec <= 0.0
                or now_sec - rcm_time <= self.rcm_marker_max_age_sec
            )
        ):
            if not self.rcm_trace_started:
                self.ee_trace_last_point = None
                self.rcm_trace_started = True
            self._set_marker_body(self.rcm_marker_body_id, rcm_point)
            self.rcm_label_id = self._set_marker_label(
                self.rcm_label_id,
                "RCM",
                rcm_point,
                [1.0, 0.05, 0.05],
            )
        else:
            self._hide_body(self.rcm_marker_body_id)
            self.rcm_label_id = self._set_marker_label(
                self.rcm_label_id,
                "",
                [0.0, 0.0, -10.0],
                [1.0, 0.05, 0.05],
            )

        if (
            tip_point is not None
            and (
                self.rcm_marker_max_age_sec <= 0.0
                or now_sec - tip_time <= self.rcm_marker_max_age_sec
            )
        ):
            self._set_marker_body(self.rcm_tip_marker_body_id, tip_point)
            self.rcm_tip_label_id = self._set_marker_label(
                self.rcm_tip_label_id,
                "TIP",
                tip_point,
                [1.0, 0.86, 0.05],
            )
        else:
            self._hide_body(self.rcm_tip_marker_body_id)
            self.rcm_tip_label_id = self._set_marker_label(
                self.rcm_tip_label_id,
                "",
                [0.0, 0.0, -10.0],
                [1.0, 0.86, 0.05],
            )

        if not record_trace:
            return

        if rcm_point is not None and self.rcm_trace_min_inserted_depth_m > 0.0:
            shaft = [
                tool_tip[index] - ee_point[index]
                for index in range(3)
            ]
            shaft_norm = max(
                sum(value * value for value in shaft) ** 0.5,
                1e-9,
            )
            shaft = [value / shaft_norm for value in shaft]
            inserted_depth = sum(
                (tool_tip[index] - rcm_point[index]) * shaft[index]
                for index in range(3)
            )
            if inserted_depth < self.rcm_trace_min_inserted_depth_m:
                return

        trace_point = (
            tool_tip
            if self.show_rcm_tool and self.trace_rcm_tool_tip
            else ee_point
        )
        if self.ee_trace_last_point is not None:
            dist_sq = sum(
                (trace_point[index] - self.ee_trace_last_point[index]) ** 2
                for index in range(3)
            )
            if dist_sq < self.ee_trace_min_dist_m * self.ee_trace_min_dist_m:
                return

        body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=self.ee_trace_visual_shape_id,
            basePosition=trace_point,
            physicsClientId=self.client,
        )
        self.ee_trace_body_ids.append(body_id)
        self.ee_trace_last_point = list(trace_point)

        while len(self.ee_trace_body_ids) > self.ee_trace_max_points:
            old_body_id = self.ee_trace_body_ids.pop(0)
            p.removeBody(old_body_id, physicsClientId=self.client)

    def _send_init_done_once(self):
        if self._init_done_sent:
            return
        msg = Bool()
        msg.data = True
        self.pub_init_done.publish(msg)
        self._init_done_sent = True
        self.get_logger().info("[INIT] Published latched /iiwa7/init_done = True")

    def debug_status(self):
        with self._lock:
            mode = self.control_mode
            have_des = self.have_des
            have_tau = self.have_tau
            q0 = self.q_cmd[0]
            tau0 = self.tau_cmd[0]

        self.get_logger().info(
            "[DBG] "
            f"phase={self.phase}, "
            f"mode={self.MODE_NAMES.get(mode, mode)}, "
            f"have_des={have_des}, "
            f"have_tau={have_tau}, "
            f"q0={q0:.3f}, "
            f"tau0={tau0:.2f}"
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

    def _apply_position_targets(
        self,
        q_target: List[float],
        force: float,
        pos_gain: float,
        vel_gain: float,
        max_vel: float,
    ):
        for array_index, joint_index in enumerate(self.joint_indices):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=float(q_target[array_index]),
                positionGain=float(pos_gain),
                velocityGain=float(vel_gain),
                force=float(force),
                maxVelocity=float(max_vel),
                physicsClientId=self.client,
            )

    def _apply_torque_targets(self, tau_target: List[float]):
        self.release_motors()

        if (not self.have_tau) and (not self._warned_no_tau):
            self.get_logger().warning(
                "[TORQUE] No /iiwa7/joint_torques received yet. Applying zero torques."
            )
            self._warned_no_tau = True

        for array_index, joint_index in enumerate(self.joint_indices):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.TORQUE_CONTROL,
                force=self._clip_tau(float(tau_target[array_index])),
                physicsClientId=self.client,
            )

    def _step_goto(self, now_sec: float):
        """Move the robot smoothly to the configured initial configuration."""
        progress = (now_sec - self.t_goto_start) / max(self.goto_duration, 1e-6)
        scale = min_jerk_s(progress)
        q_ref = [
            self.q_start[index] + scale * (self.init_q[index] - self.q_start[index])
            for index in range(self.n)
        ]

        if self.goto_use_reset:
            self.release_motors()
            for array_index, joint_index in enumerate(self.joint_indices):
                p.resetJointState(
                    self.robot_id,
                    joint_index,
                    q_ref[array_index],
                    targetVelocity=0.0,
                    physicsClientId=self.client,
                )
        else:
            self._apply_position_targets(
                q_ref,
                self.goto_force,
                self.goto_pos_gain,
                self.goto_vel_gain,
                self.goto_max_vel,
            )

        p.stepSimulation(physicsClientId=self.client)
        if self.goto_use_reset:
            for array_index, joint_index in enumerate(self.joint_indices):
                p.resetJointState(
                    self.robot_id,
                    joint_index,
                    q_ref[array_index],
                    targetVelocity=0.0,
                    physicsClientId=self.client,
                )
        self.publish_joint_states()
        self._update_rcm_debug_overlay(record_trace=False)

        q_actual, qd_actual, _ = self._get_q_qd_tau()
        position_error = max(
            abs(q_actual[index] - self.init_q[index])
            for index in range(self.n)
        )
        max_velocity = max(abs(value) for value in qd_actual)
        if (
            progress >= 1.0
            and position_error <= self.goto_pos_tolerance_rad
            and max_velocity <= self.goto_vel_tolerance_rad_s
        ):
            self.goto_settle_count += 1
        else:
            self.goto_settle_count = 0

        if self.goto_settle_count >= self.goto_settle_cycles:
            self.phase = "RUN"
            self._send_init_done_once()
            self.get_logger().info(
                "[GOTO] settled -> RUN: "
                f"position_error={position_error:.4f} rad, "
                f"max_velocity={max_velocity:.4f} rad/s"
            )

    def _step_run(self):
        with self._lock:
            mode = self.control_mode
            q_target = list(self.q_cmd)
            tau_target = list(self.tau_cmd)

        if mode == self.MODE_FREE:
            self.release_motors()
        elif mode == self.MODE_POSITION:
            self._apply_position_targets(
                q_target,
                self.track_force,
                self.track_pos_gain,
                self.track_vel_gain,
                self.track_max_vel,
            )
        else:
            self._apply_torque_targets(tau_target)

        p.stepSimulation(physicsClientId=self.client)
        self.publish_joint_states()
        self._update_rcm_debug_overlay()

    def step(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.phase == "GOTO":
            self._step_goto(now_sec)
            self._publish_rgbd_if_due(now_sec)
            return
        self._step_run()
        self._publish_rgbd_if_due(now_sec)

    def destroy_node(self):
        try:
            p.disconnect(physicsClientId=self.client)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IiwaPybulletSim()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
