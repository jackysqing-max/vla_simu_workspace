#!/usr/bin/env python3
"""Kinematic virtual-fixture demos for the PyBullet KUKA iiwa interface.

The original Franka demos in ``franka_task.zip`` used libfranka callbacks and
joint torque / velocity commands.  This node keeps the task-space geometry of
those demos but adapts the runtime interface to the existing PyBullet KUKA
simulation:

* input:  ``/iiwa7/joint_states`` and an optional locked VLM port pose
* output: ``/iiwa7/joint_desired``
* mode:   ``/iiwa7/control_mode`` set to POSITION

It is intentionally a kinematic/position-command demonstration, not a
drop-in torque-level Franka controller.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int8, String

from .approach_cone import (
    angular_distance_deg,
    axis_at_cone_angles,
    sample_cone_axes,
    wrapped_joint_distance,
)


IIWA_LOWER_LIMITS = [
    -2.96705972839,
    -2.09439510239,
    -2.96705972839,
    -2.09439510239,
    -2.96705972839,
    -2.09439510239,
    -3.05432619099,
]
IIWA_UPPER_LIMITS = [
    2.96705972839,
    2.09439510239,
    2.96705972839,
    2.09439510239,
    2.96705972839,
    2.09439510239,
    3.05432619099,
]
IIWA_JOINT_RANGES = [
    upper - lower for lower, upper in zip(IIWA_LOWER_LIMITS, IIWA_UPPER_LIMITS)
]


def min_jerk(u: float) -> float:
    """Smooth interpolation factor in [0, 1]."""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def triangle01(phase: float) -> float:
    """Triangle wave in [0, 1]."""
    phase = phase - math.floor(phase)
    return 1.0 - abs(2.0 * phase - 1.0)


def normalize(vec: Iterable[float], fallback=(1.0, 0.0, 0.0)) -> np.ndarray:
    arr = np.asarray(list(vec), dtype=np.float64)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9:
        return np.asarray(fallback, dtype=np.float64)
    return arr / norm


def parse_vec3(value, default) -> np.ndarray:
    """Parse a ROS parameter value as a 3-vector."""
    if value is None:
        return np.asarray(default, dtype=np.float64)
    if isinstance(value, str):
        text = value.strip().strip("[]")
        if not text:
            return np.asarray(default, dtype=np.float64)
        pieces = [piece.strip() for piece in text.split(",")]
        try:
            arr = [float(piece) for piece in pieces if piece]
        except ValueError:
            return np.asarray(default, dtype=np.float64)
    else:
        try:
            arr = [float(item) for item in value]
        except TypeError:
            return np.asarray(default, dtype=np.float64)
    if len(arr) != 3:
        return np.asarray(default, dtype=np.float64)
    return np.asarray(arr, dtype=np.float64)


def rotation_matrix_from_z_axis(
    z_axis: np.ndarray,
    preferred_x: np.ndarray | None = None,
) -> np.ndarray:
    """Build a continuous right-handed frame whose z axis follows ``z_axis``."""
    z = normalize(z_axis, fallback=(0.0, 0.0, 1.0))
    if preferred_x is not None:
        projected_x = np.asarray(preferred_x, dtype=np.float64)
        projected_x = projected_x - float(np.dot(projected_x, z)) * z
        if float(np.linalg.norm(projected_x)) >= 1e-6:
            x = normalize(projected_x)
            y = normalize(np.cross(z, x), fallback=(0.0, 1.0, 0.0))
            x = normalize(np.cross(y, z), fallback=x)
            return np.column_stack((x, y, z))

    ref = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(z, ref))) > 0.92:
        ref = np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    x = normalize(np.cross(ref, z), fallback=(1.0, 0.0, 0.0))
    y = normalize(np.cross(z, x), fallback=(0.0, 1.0, 0.0))
    return np.column_stack((x, y, z))


def quaternion_from_matrix(rot: np.ndarray) -> list[float]:
    """Convert a 3x3 rotation matrix to PyBullet xyzw quaternion."""
    m = np.asarray(rot, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    else:
        axis = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        if axis == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif axis == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
    quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-9)
    return quat.tolist()


def quat_to_matrix_xyzw(quat) -> np.ndarray:
    mat = p.getMatrixFromQuaternion([float(value) for value in quat])
    return np.asarray(mat, dtype=np.float64).reshape(3, 3)


@dataclass
class FixtureTarget:
    ee_pos: np.ndarray
    ee_quat: list[float]
    desired_tip: np.ndarray | None = None
    desired_rcm: np.ndarray | None = None
    plane_error: float = 0.0
    line_error: float = 0.0


class SideIiwaModel:
    """A lightweight DIRECT PyBullet model used only for FK/IK."""

    def __init__(self):
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        self.robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            useFixedBase=True,
            physicsClientId=self.client,
        )
        self.n = 7
        self.ee_link = self.n - 1

    def reset(self, q):
        for index in range(self.n):
            p.resetJointState(
                self.robot_id,
                index,
                float(q[index]),
                physicsClientId=self.client,
            )

    def fk(self, q):
        self.reset(q)
        state = p.getLinkState(
            self.robot_id,
            self.ee_link,
            computeForwardKinematics=True,
            physicsClientId=self.client,
        )
        return np.asarray(state[4], dtype=np.float64), list(state[5])

    def solve_ik(
        self,
        q_seed,
        target_pos,
        target_quat,
        rest_poses,
        damping,
        max_iterations,
        residual_threshold,
    ):
        self.reset(q_seed)
        solution = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.ee_link,
            targetPosition=[float(value) for value in target_pos],
            targetOrientation=[float(value) for value in target_quat],
            lowerLimits=IIWA_LOWER_LIMITS,
            upperLimits=IIWA_UPPER_LIMITS,
            jointRanges=IIWA_JOINT_RANGES,
            restPoses=[float(value) for value in rest_poses],
            jointDamping=[float(value) for value in damping],
            maxNumIterations=int(max_iterations),
            residualThreshold=float(residual_threshold),
            physicsClientId=self.client,
        )
        return list(solution[: self.n])

    def close(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass


class RcmVirtualFixtureNode(Node):
    """Publish KUKA joint targets for RCM, line, and plane virtual fixtures."""

    MODE_POSITION = 1

    def __init__(self):
        super().__init__("rcm_virtual_fixture_node")

        self.n = 7
        self.model = SideIiwaModel()

        self._declare_parameters()
        self._load_parameters()
        self.metrics_csv_file = None
        self.metrics_csv_writer = None
        self.metrics_csv_rows_since_flush = 0
        self.metrics_csv_failed = False
        self._open_metrics_csv()

        self.pub_qdes = self.create_publisher(
            Float64MultiArray,
            self.joint_desired_topic,
            10,
        )
        self.pub_mode = self.create_publisher(Int8, self.control_mode_topic, 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)
        self.pub_stage = self.create_publisher(String, self.stage_topic, 10)
        self.pub_metrics = self.create_publisher(Float64MultiArray, self.metrics_topic, 10)
        self.pub_rcm_point = self.create_publisher(PointStamped, self.rcm_point_topic, 10)
        self.pub_tip_point = self.create_publisher(PointStamped, self.tip_point_topic, 10)
        self.pub_locked_port_point = self.create_publisher(
            PointStamped,
            self.locked_port_point_topic,
            10,
        )
        self.pub_locked_port_axis = self.create_publisher(
            Vector3Stamped,
            self.locked_port_axis_topic,
            10,
        )
        self.pub_approach_selection = self.create_publisher(
            String,
            self.approach_selection_topic,
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.latest_q = None
        self.latest_qdot = None
        self.visual_port_point = None
        self.visual_port_axis = None
        self.visual_port_ready = not self.use_vlm_port_pose
        self.visual_pose_rejection = ""
        self.approach_constraint = {
            "mode": "auto_closest_reachable",
            "cone_half_angle_deg": self.approach_cone_half_angle_deg,
            "preferred_tilt_deg": None,
            "preferred_azimuth_deg": None,
        }
        self.approach_search_failed = False
        self.start_requested = not self.wait_for_start_command
        self.pivot_requested = not self.wait_for_pivot_command
        self.held_rcm_target = None

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, self.init_done_topic, self.on_init_done, qos_latch)
        self.create_subscription(
            String,
            self.approach_constraint_topic,
            self.on_approach_constraint,
            qos_latch,
        )
        self.create_subscription(JointState, self.joint_states_topic, self.on_joint_states, 10)
        if self.use_vlm_port_pose:
            self.create_subscription(
                PointStamped,
                self.visual_port_point_topic,
                self.on_visual_port_point,
                qos_latch,
            )
            self.create_subscription(
                Vector3Stamped,
                self.visual_port_axis_topic,
                self.on_visual_port_axis,
                qos_latch,
            )
            self.create_subscription(
                Bool,
                self.visual_port_ready_topic,
                self.on_visual_port_ready,
                qos_latch,
            )
        if self.wait_for_start_command:
            self.create_subscription(
                Bool,
                self.start_topic,
                self.on_start_command,
                qos_latch,
            )
        if self.wait_for_pivot_command:
            self.create_subscription(
                Bool,
                self.pivot_start_topic,
                self.on_pivot_start_command,
                qos_latch,
            )
        self.create_subscription(
            Bool,
            self.pivot_hold_topic,
            self.on_pivot_hold_command,
            qos_latch,
        )
        self.pub_started = self.create_publisher(
            Bool,
            self.started_topic,
            qos_latch,
        )
        self._publish_started(False)

        self.init_done = not self.require_init_done
        self.locked = False
        self.stable_count = 0
        self.start_time = None
        self.stage_start_time = None
        if self.wait_for_start_command:
            self.motion_stage = "WAITING_FOR_START"
        elif self.use_vlm_port_pose:
            self.motion_stage = "WAITING_FOR_VLM_PORT"
        else:
            self.motion_stage = "WAITING_FOR_INIT"
        self.stage_settle_count = 0
        self.last_status_time = 0.0
        self.last_mode_time = 0.0

        self.q0 = None
        self.q_seed = None
        self.ee0 = None
        self.orn0 = None
        self.initial_tip = None
        self.tip0 = None
        self.rcm0 = None
        self.rcm_shaft0 = None
        self.plane_point = None
        self.line_anchor = None
        self.line_direction = normalize(self.line_direction_param, fallback=(0.0, 1.0, 0.0))
        self.plane_normal = normalize(self.plane_normal_param, fallback=(0.0, 0.0, 1.0))
        self.rcm_ee_distance = None
        self.rcm_tip_distance = None
        self.rcm_circle_radius = None
        self.rcm_circle_tilt = None
        self.rcm_circle_axis = None
        self.rcm_circle_basis_1 = None
        self.rcm_circle_basis_2 = None
        self.tool_frame_x0 = None

        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info(
            "rcm_virtual_fixture_node started: "
            f"mode={self.fixture_mode}, "
            f"joint_states={self.joint_states_topic}, "
            f"joint_desired={self.joint_desired_topic}"
        )
        self.get_logger().info(
            "Adapted from Franka RCM / lineVF / planeVF demos to the "
            "PyBullet KUKA iiwa position-command interface."
        )
        if self.use_vlm_port_pose:
            self.get_logger().info(
                "[VLM_RCM] waiting for a stable locked port pose: "
                f"point={self.visual_port_point_topic}, "
                f"axis={self.visual_port_axis_topic}"
            )
        if self.metrics_csv_file is not None:
            self.get_logger().info(
                f"[METRICS] recording RCM and trajectory errors to {self.metrics_csv_path}"
            )

    def _declare_parameters(self):
        self.declare_parameter("fixture_mode", "rcm")
        self.declare_parameter("publish_hz", 100.0)
        self.declare_parameter("joint_states_topic", "/iiwa7/joint_states")
        self.declare_parameter("joint_desired_topic", "/iiwa7/joint_desired")
        self.declare_parameter("control_mode_topic", "/iiwa7/control_mode")
        self.declare_parameter("init_done_topic", "/iiwa7/init_done")
        self.declare_parameter("status_topic", "/rcm_virtual_fixtures/status")
        self.declare_parameter("stage_topic", "/rcm_virtual_fixtures/stage")
        self.declare_parameter("metrics_topic", "/rcm_virtual_fixtures/metrics")
        self.declare_parameter("rcm_point_topic", "/rcm_virtual_fixtures/rcm_point")
        self.declare_parameter("tip_point_topic", "/rcm_virtual_fixtures/tip_point")
        self.declare_parameter(
            "locked_port_point_topic",
            "/rcm_virtual_fixtures/locked_port_point",
        )
        self.declare_parameter(
            "locked_port_axis_topic",
            "/rcm_virtual_fixtures/locked_port_axis",
        )
        self.declare_parameter("require_init_done", True)
        self.declare_parameter("wait_for_start_command", False)
        self.declare_parameter("start_topic", "/rcm_virtual_fixtures/start")
        self.declare_parameter("started_topic", "/rcm_virtual_fixtures/started")
        self.declare_parameter("wait_for_pivot_command", False)
        self.declare_parameter(
            "pivot_start_topic",
            "/rcm_virtual_fixtures/start_pivot",
        )
        self.declare_parameter(
            "pivot_hold_topic",
            "/rcm_virtual_fixtures/hold_pivot",
        )
        self.declare_parameter("lock_vel_eps", 0.03)
        self.declare_parameter("lock_count", 15)
        self.declare_parameter("max_joint_step_rad", 0.025)
        self.declare_parameter("joint_damping", [0.12] * 7)
        self.declare_parameter("ik_max_iterations", 120)
        self.declare_parameter("ik_residual_threshold", 1e-6)

        self.declare_parameter("tool_length_m", 0.22)
        self.declare_parameter("rcm_lambda", 0.65)
        self.declare_parameter("use_initial_rcm", True)
        self.declare_parameter("rcm_world", [0.70, 0.0, 0.405])
        self.declare_parameter("use_vlm_port_pose", False)
        self.declare_parameter(
            "visual_port_point_topic",
            "/vlm_rcm/locked_port_point",
        )
        self.declare_parameter(
            "visual_port_axis_topic",
            "/vlm_rcm/locked_port_axis",
        )
        self.declare_parameter(
            "visual_port_ready_topic",
            "/vlm_rcm/port_ready",
        )
        self.declare_parameter("visual_port_target_frame", "world")
        self.declare_parameter("visual_port_max_offset_m", 0.080)
        self.declare_parameter(
            "visual_port_reference_axis",
            [0.335067, 0.0, -0.942194],
        )
        self.declare_parameter("visual_port_max_axis_angle_deg", 25.0)
        self.declare_parameter("enable_approach_cone", True)
        self.declare_parameter("approach_cone_half_angle_deg", 20.0)
        self.declare_parameter("approach_cone_radial_samples", 4)
        self.declare_parameter("approach_cone_azimuth_samples", 16)
        self.declare_parameter("approach_ik_position_tolerance_m", 0.008)
        self.declare_parameter("approach_ik_axis_tolerance_deg", 6.0)
        self.declare_parameter("approach_joint_limit_margin_rad", 0.02)
        self.declare_parameter(
            "approach_constraint_topic",
            "/rcm_virtual_fixtures/approach_constraint_json",
        )
        self.declare_parameter(
            "approach_selection_topic",
            "/rcm_virtual_fixtures/approach_selection_json",
        )
        self.declare_parameter("enable_safe_insertion", False)
        self.declare_parameter("preinsert_clearance_m", 0.040)
        self.declare_parameter("port_standoff_m", 0.012)
        self.declare_parameter("preinsert_hold_sec", 1.0)
        self.declare_parameter("approach_duration_sec", 2.0)
        self.declare_parameter("port_dwell_sec", 0.8)
        self.declare_parameter("insertion_speed_mps", 0.018)
        self.declare_parameter("inserted_dwell_sec", 0.8)
        self.declare_parameter("stage_position_tolerance_m", 0.006)
        self.declare_parameter("stage_settle_cycles", 12)
        self.declare_parameter("trajectory_radius_m", 0.025)
        self.declare_parameter("trajectory_length_m", 0.12)
        self.declare_parameter("speed_mps", 0.025)
        self.declare_parameter("plane_normal", [0.0, 0.0, 1.0])
        self.declare_parameter("line_direction", [0.0, 1.0, 0.0])
        self.declare_parameter("status_hz", 2.0)
        self.declare_parameter("record_metrics_csv", False)
        self.declare_parameter("metrics_csv_path", "")
        self.declare_parameter("metrics_csv_flush_every", 10)

    def _load_parameters(self):
        self.fixture_mode = str(self.get_parameter("fixture_mode").value).strip().lower()
        if self.fixture_mode not in {"rcm", "line", "plane"}:
            self.get_logger().warning(
                f"unsupported fixture_mode={self.fixture_mode!r}; using 'rcm'"
            )
            self.fixture_mode = "rcm"

        self.publish_hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.joint_desired_topic = str(self.get_parameter("joint_desired_topic").value)
        self.control_mode_topic = str(self.get_parameter("control_mode_topic").value)
        self.init_done_topic = str(self.get_parameter("init_done_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.stage_topic = str(self.get_parameter("stage_topic").value)
        self.metrics_topic = str(self.get_parameter("metrics_topic").value)
        self.rcm_point_topic = str(self.get_parameter("rcm_point_topic").value)
        self.tip_point_topic = str(self.get_parameter("tip_point_topic").value)
        self.locked_port_point_topic = str(
            self.get_parameter("locked_port_point_topic").value
        )
        self.locked_port_axis_topic = str(
            self.get_parameter("locked_port_axis_topic").value
        )
        self.require_init_done = bool(self.get_parameter("require_init_done").value)
        self.wait_for_start_command = bool(
            self.get_parameter("wait_for_start_command").value
        )
        self.start_topic = str(self.get_parameter("start_topic").value)
        self.started_topic = str(self.get_parameter("started_topic").value)
        self.wait_for_pivot_command = bool(
            self.get_parameter("wait_for_pivot_command").value
        )
        self.pivot_start_topic = str(
            self.get_parameter("pivot_start_topic").value
        )
        self.pivot_hold_topic = str(
            self.get_parameter("pivot_hold_topic").value
        )
        self.lock_vel_eps = max(float(self.get_parameter("lock_vel_eps").value), 0.0)
        self.lock_count = max(int(self.get_parameter("lock_count").value), 1)
        self.max_joint_step_rad = max(
            float(self.get_parameter("max_joint_step_rad").value),
            0.001,
        )
        damping = [float(value) for value in self.get_parameter("joint_damping").value]
        if len(damping) < self.n:
            damping += [0.12] * (self.n - len(damping))
        self.joint_damping = damping[: self.n]
        self.ik_max_iterations = max(
            int(self.get_parameter("ik_max_iterations").value),
            20,
        )
        self.ik_residual_threshold = max(
            float(self.get_parameter("ik_residual_threshold").value),
            1e-9,
        )

        self.tool_length_m = max(float(self.get_parameter("tool_length_m").value), 0.02)
        self.rcm_lambda = float(np.clip(float(self.get_parameter("rcm_lambda").value), 0.05, 0.95))
        self.use_initial_rcm = bool(self.get_parameter("use_initial_rcm").value)
        self.rcm_world_param = parse_vec3(
            self.get_parameter("rcm_world").value,
            [0.70, 0.0, 0.405],
        )
        self.use_vlm_port_pose = bool(
            self.get_parameter("use_vlm_port_pose").value
        )
        self.visual_port_point_topic = str(
            self.get_parameter("visual_port_point_topic").value
        )
        self.visual_port_axis_topic = str(
            self.get_parameter("visual_port_axis_topic").value
        )
        self.visual_port_ready_topic = str(
            self.get_parameter("visual_port_ready_topic").value
        )
        self.visual_port_target_frame = str(
            self.get_parameter("visual_port_target_frame").value
        ).strip()
        self.visual_port_max_offset_m = max(
            float(self.get_parameter("visual_port_max_offset_m").value),
            0.0,
        )
        self.visual_port_reference_axis = normalize(
            parse_vec3(
                self.get_parameter("visual_port_reference_axis").value,
                [0.335067, 0.0, -0.942194],
            ),
            fallback=(0.335067, 0.0, -0.942194),
        )
        self.visual_port_max_axis_angle_deg = float(
            np.clip(
                float(
                    self.get_parameter(
                        "visual_port_max_axis_angle_deg"
                    ).value
                ),
                1.0,
                90.0,
            )
        )
        self.enable_approach_cone = bool(
            self.get_parameter("enable_approach_cone").value
        )
        self.approach_cone_half_angle_deg = float(
            np.clip(
                float(self.get_parameter("approach_cone_half_angle_deg").value),
                1.0,
                45.0,
            )
        )
        self.approach_cone_radial_samples = max(
            int(self.get_parameter("approach_cone_radial_samples").value), 1
        )
        self.approach_cone_azimuth_samples = max(
            int(self.get_parameter("approach_cone_azimuth_samples").value), 4
        )
        self.approach_ik_position_tolerance_m = max(
            float(self.get_parameter("approach_ik_position_tolerance_m").value),
            0.001,
        )
        self.approach_ik_axis_tolerance_deg = max(
            float(self.get_parameter("approach_ik_axis_tolerance_deg").value),
            0.5,
        )
        self.approach_joint_limit_margin_rad = max(
            float(self.get_parameter("approach_joint_limit_margin_rad").value),
            0.0,
        )
        self.approach_constraint_topic = str(
            self.get_parameter("approach_constraint_topic").value
        )
        self.approach_selection_topic = str(
            self.get_parameter("approach_selection_topic").value
        )
        if self.use_vlm_port_pose and self.use_initial_rcm:
            self.get_logger().warning(
                "use_vlm_port_pose=true overrides use_initial_rcm=true"
            )
            self.use_initial_rcm = False
        self.enable_safe_insertion = bool(
            self.get_parameter("enable_safe_insertion").value
        )
        self.preinsert_clearance_m = max(
            float(self.get_parameter("preinsert_clearance_m").value),
            0.002,
        )
        self.port_standoff_m = float(
            np.clip(
                float(self.get_parameter("port_standoff_m").value),
                0.001,
                0.9 * self.preinsert_clearance_m,
            )
        )
        self.preinsert_hold_sec = max(
            float(self.get_parameter("preinsert_hold_sec").value),
            0.0,
        )
        self.approach_duration_sec = max(
            float(self.get_parameter("approach_duration_sec").value),
            0.1,
        )
        self.port_dwell_sec = max(
            float(self.get_parameter("port_dwell_sec").value),
            0.0,
        )
        self.insertion_speed_mps = max(
            float(self.get_parameter("insertion_speed_mps").value),
            0.001,
        )
        self.inserted_dwell_sec = max(
            float(self.get_parameter("inserted_dwell_sec").value),
            0.0,
        )
        self.stage_position_tolerance_m = max(
            float(self.get_parameter("stage_position_tolerance_m").value),
            0.001,
        )
        self.stage_settle_cycles = max(
            int(self.get_parameter("stage_settle_cycles").value),
            1,
        )
        self.safe_insertion_active = (
            self.fixture_mode == "rcm"
            and self.enable_safe_insertion
            and not self.use_initial_rcm
        )
        if (
            self.fixture_mode == "rcm"
            and self.enable_safe_insertion
            and self.use_initial_rcm
        ):
            self.get_logger().warning(
                "safe insertion requires a fixed rcm_world; disabling it because "
                "use_initial_rcm=true"
            )
        self.trajectory_radius_m = max(
            float(self.get_parameter("trajectory_radius_m").value),
            0.001,
        )
        self.trajectory_length_m = max(
            float(self.get_parameter("trajectory_length_m").value),
            0.001,
        )
        self.speed_mps = max(float(self.get_parameter("speed_mps").value), 0.001)
        self.plane_normal_param = parse_vec3(
            self.get_parameter("plane_normal").value,
            [0.0, 0.0, 1.0],
        )
        self.line_direction_param = parse_vec3(
            self.get_parameter("line_direction").value,
            [0.0, 1.0, 0.0],
        )
        self.status_hz = max(float(self.get_parameter("status_hz").value), 0.1)
        self.record_metrics_csv = bool(
            self.get_parameter("record_metrics_csv").value
        )
        self.metrics_csv_path = str(
            self.get_parameter("metrics_csv_path").value
        ).strip()
        self.metrics_csv_flush_every = max(
            int(self.get_parameter("metrics_csv_flush_every").value),
            1,
        )

    def _open_metrics_csv(self):
        if not self.record_metrics_csv or self.fixture_mode != "rcm":
            return
        if not self.metrics_csv_path:
            self.get_logger().warning(
                "[METRICS] record_metrics_csv=true but metrics_csv_path is empty"
            )
            return

        path = Path(self.metrics_csv_path).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            self.metrics_csv_file = path.open(
                "w",
                newline="",
                encoding="utf-8",
                buffering=1,
            )
            self.metrics_csv_path = str(path.resolve())
            self.metrics_csv_writer = csv.writer(self.metrics_csv_file)
            self.metrics_csv_writer.writerow(
                [
                    "ros_time_sec",
                    "elapsed_sec",
                    "rcm_error_norm_m",
                    "rcm_radial_error_m",
                    "rcm_axial_error_m",
                    "ee_tracking_error_norm_m",
                    "tool_tip_tracking_error_norm_m",
                    "desired_rcm_x_m",
                    "desired_rcm_y_m",
                    "desired_rcm_z_m",
                    "actual_rcm_x_m",
                    "actual_rcm_y_m",
                    "actual_rcm_z_m",
                    "rcm_error_x_m",
                    "rcm_error_y_m",
                    "rcm_error_z_m",
                    "desired_ee_x_m",
                    "desired_ee_y_m",
                    "desired_ee_z_m",
                    "actual_ee_x_m",
                    "actual_ee_y_m",
                    "actual_ee_z_m",
                    "ee_error_x_m",
                    "ee_error_y_m",
                    "ee_error_z_m",
                    "desired_tip_x_m",
                    "desired_tip_y_m",
                    "desired_tip_z_m",
                    "actual_tip_x_m",
                    "actual_tip_y_m",
                    "actual_tip_z_m",
                    "tip_error_x_m",
                    "tip_error_y_m",
                    "tip_error_z_m",
                ]
            )
            self.metrics_csv_file.flush()
        except OSError as exc:
            self.metrics_csv_file = None
            self.metrics_csv_writer = None
            self.get_logger().error(
                f"[METRICS] failed to open CSV {path}: {exc}"
            )

    def _record_metrics_csv(
        self,
        ros_time_sec,
        elapsed,
        target,
        desired_rcm,
        current_rcm,
        rcm_error_vec,
        rcm_error,
        rcm_radial_error,
        rcm_axial_error,
        current_ee,
        ee_error_vec,
        ee_tracking_error,
        desired_tip,
        current_tip,
        tip_error_vec,
        tip_tracking_error,
    ):
        if self.metrics_csv_writer is None or self.metrics_csv_failed:
            return
        try:
            row = [
                ros_time_sec,
                elapsed,
                rcm_error,
                rcm_radial_error,
                rcm_axial_error,
                ee_tracking_error,
                tip_tracking_error,
                *desired_rcm,
                *current_rcm,
                *rcm_error_vec,
                *target.ee_pos,
                *current_ee,
                *ee_error_vec,
                *desired_tip,
                *current_tip,
                *tip_error_vec,
            ]
            self.metrics_csv_writer.writerow(
                [f"{float(value):.9f}" for value in row]
            )
            self.metrics_csv_rows_since_flush += 1
            if self.metrics_csv_rows_since_flush >= self.metrics_csv_flush_every:
                self.metrics_csv_file.flush()
                self.metrics_csv_rows_since_flush = 0
        except (OSError, ValueError) as exc:
            self.metrics_csv_failed = True
            self.get_logger().error(f"[METRICS] failed to write CSV: {exc}")

    def on_init_done(self, msg: Bool):
        self.init_done = bool(msg.data) or not self.require_init_done
        self._try_lock_initial_state()

    def _publish_started(self, started: bool):
        msg = Bool()
        msg.data = bool(started)
        self.pub_started.publish(msg)

    def _publish_stage(self):
        msg = String()
        msg.data = self.motion_stage
        self.pub_stage.publish(msg)

    def on_start_command(self, msg: Bool):
        if self.locked:
            return
        self.start_requested = bool(msg.data)
        if self.start_requested:
            self.motion_stage = (
                "WAITING_FOR_VLM_PORT"
                if self.use_vlm_port_pose
                else "WAITING_FOR_INIT"
            )
            self.get_logger().info(
                "[TASK_GATE] start command accepted; validating locked port pose"
            )
            self._try_lock_initial_state()
        else:
            self.motion_stage = "WAITING_FOR_START"
            self._publish_started(False)
        self._publish_stage()

    def on_approach_constraint(self, msg: String):
        """Accept planner geometry without interpreting natural language locally."""
        try:
            raw = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warning(f"[APPROACH_CONE] invalid JSON: {exc}")
            return
        if not isinstance(raw, dict):
            return
        mode = str(raw.get("mode", "auto_closest_reachable")).strip()
        if mode not in {"auto_closest_reachable", "preferred_cone_angle"}:
            mode = "auto_closest_reachable"
        try:
            half_angle = float(
                raw.get("cone_half_angle_deg", self.approach_cone_half_angle_deg)
            )
        except (TypeError, ValueError):
            half_angle = self.approach_cone_half_angle_deg
        half_angle = float(np.clip(half_angle, 1.0, 45.0))
        try:
            tilt = raw.get("preferred_tilt_deg", None)
            azimuth = raw.get("preferred_azimuth_deg", None)
            tilt = None if tilt is None else float(tilt)
            azimuth = None if azimuth is None else float(azimuth)
        except (TypeError, ValueError):
            tilt = None
            azimuth = None
        if mode != "preferred_cone_angle" or tilt is None or azimuth is None:
            mode = "auto_closest_reachable"
            tilt = None
            azimuth = None
        else:
            tilt = float(np.clip(tilt, 0.0, half_angle))
            azimuth %= 360.0
        self.approach_constraint = {
            "mode": mode,
            "cone_half_angle_deg": half_angle,
            "preferred_tilt_deg": tilt,
            "preferred_azimuth_deg": azimuth,
        }
        self.approach_search_failed = False
        self.get_logger().info(
            "[APPROACH_CONE] constraint received: "
            f"mode={mode} half_angle={half_angle:.1f}deg "
            f"preferred=({tilt},{azimuth})"
        )
        self._try_lock_initial_state()

    def on_pivot_start_command(self, msg: Bool):
        self.pivot_requested = bool(msg.data)
        if (
            self.pivot_requested
            and self.motion_stage == "WAITING_FOR_PIVOT"
        ):
            self._set_motion_stage("RCM_PIVOT")
        self.get_logger().info(
            "[TASK_GATE] pivot command="
            f"{str(self.pivot_requested).lower()}"
        )

    def on_pivot_hold_command(self, msg: Bool):
        if not msg.data or self.motion_stage != "RCM_PIVOT":
            return
        self.held_rcm_target = self._rcm_target(self._stage_elapsed())
        self._set_motion_stage("RCM_HOLD")
        self.get_logger().info(
            "[TASK_GATE] circular trajectory complete; holding final RCM pose"
        )

    def on_joint_states(self, msg: JointState):
        if not msg.position or len(msg.position) < self.n:
            return
        self.latest_q = [float(value) for value in msg.position[: self.n]]
        if msg.velocity and len(msg.velocity) >= self.n:
            self.latest_qdot = [float(value) for value in msg.velocity[: self.n]]
        else:
            self.latest_qdot = [0.0] * self.n

        if not self.locked and self.init_done:
            vmax = max(abs(value) for value in self.latest_qdot[: self.n])
            if vmax <= self.lock_vel_eps:
                self.stable_count += 1
            else:
                self.stable_count = 0
            if self.stable_count >= self.lock_count:
                self._try_lock_initial_state()

    def on_visual_port_point(self, msg: PointStamped):
        if self.locked:
            return
        frame_id = str(msg.header.frame_id).strip()
        if (
            self.visual_port_target_frame
            and frame_id != self.visual_port_target_frame
        ):
            self.visual_pose_rejection = (
                f"point frame {frame_id!r} != "
                f"{self.visual_port_target_frame!r}"
            )
            return
        point = np.asarray(
            [msg.point.x, msg.point.y, msg.point.z],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(point)):
            self.visual_pose_rejection = "port point contains non-finite values"
            return
        self.visual_port_point = point
        self.approach_search_failed = False
        self._try_lock_initial_state()

    def on_visual_port_axis(self, msg: Vector3Stamped):
        if self.locked:
            return
        frame_id = str(msg.header.frame_id).strip()
        if (
            self.visual_port_target_frame
            and frame_id != self.visual_port_target_frame
        ):
            self.visual_pose_rejection = (
                f"axis frame {frame_id!r} != "
                f"{self.visual_port_target_frame!r}"
            )
            return
        raw_axis = np.asarray(
            [msg.vector.x, msg.vector.y, msg.vector.z],
            dtype=np.float64,
        )
        if (
            not np.all(np.isfinite(raw_axis))
            or float(np.linalg.norm(raw_axis)) < 1e-6
        ):
            self.visual_pose_rejection = "port axis is invalid"
            return
        axis = normalize(raw_axis, fallback=self.visual_port_reference_axis)
        if float(np.dot(axis, self.visual_port_reference_axis)) < 0.0:
            axis = -axis
        self.visual_port_axis = axis
        self.approach_search_failed = False
        self._try_lock_initial_state()

    def on_visual_port_ready(self, msg: Bool):
        if self.locked:
            return
        self.visual_port_ready = bool(msg.data)
        if not self.visual_port_ready:
            self.visual_pose_rejection = "VLM port estimator is not locked"
        self._try_lock_initial_state()

    def _visual_port_pose_is_valid(self):
        if not self.use_vlm_port_pose:
            return True, ""
        if not self.visual_port_ready:
            return False, "waiting for VLM port lock"
        if self.visual_port_point is None:
            return False, "waiting for locked port point"
        if self.visual_port_axis is None:
            return False, "waiting for locked port axis"

        point_offset = float(
            np.linalg.norm(self.visual_port_point - self.rcm_world_param)
        )
        if (
            self.visual_port_max_offset_m > 0.0
            and point_offset > self.visual_port_max_offset_m
        ):
            return (
                False,
                "visual port outside workspace gate: "
                f"offset={point_offset:.3f}m",
            )

        alignment = float(
            np.clip(
                np.dot(
                    self.visual_port_axis,
                    self.visual_port_reference_axis,
                ),
                -1.0,
                1.0,
            )
        )
        axis_angle_deg = math.degrees(math.acos(alignment))
        if axis_angle_deg > self.visual_port_max_axis_angle_deg:
            return (
                False,
                "visual port axis outside safety gate: "
                f"angle={axis_angle_deg:.1f}deg",
            )
        return True, ""

    def _publish_approach_selection(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_approach_selection.publish(msg)

    def _select_cone_approach_axis(
        self,
        center_axis: np.ndarray,
        port_point: np.ndarray,
        q_reference: list[float],
        preferred_x: np.ndarray,
    ):
        """Choose an IK-reachable inward axis from the admissible cone."""
        constraint = self.approach_constraint
        half_angle = float(constraint["cone_half_angle_deg"])
        samples = sample_cone_axes(
            center_axis,
            half_angle,
            self.approach_cone_radial_samples,
            self.approach_cone_azimuth_samples,
        )
        preferred_axis = None
        if constraint["mode"] == "preferred_cone_angle":
            preferred_axis = axis_at_cone_angles(
                center_axis,
                constraint["preferred_tilt_deg"],
                constraint["preferred_azimuth_deg"],
            )
            samples.insert(
                0,
                (
                    preferred_axis,
                    float(constraint["preferred_tilt_deg"]),
                    float(constraint["preferred_azimuth_deg"]),
                ),
            )

        reachable = []
        for axis, tilt_deg, azimuth_deg in samples:
            desired_quat = quaternion_from_matrix(
                rotation_matrix_from_z_axis(axis, preferred_x=preferred_x)
            )
            lower = np.asarray(IIWA_LOWER_LIMITS) + self.approach_joint_limit_margin_rad
            upper = np.asarray(IIWA_UPPER_LIMITS) - self.approach_joint_limit_margin_rad
            waypoint_offsets = (
                -self.preinsert_clearance_m,
                -self.port_standoff_m,
                (1.0 - self.rcm_lambda) * self.tool_length_m,
            )
            waypoint_seed = q_reference
            waypoint_solutions = []
            position_errors = []
            axis_errors = []
            path_reachable = True
            for tip_offset in waypoint_offsets:
                desired_tip = port_point + tip_offset * axis
                desired_ee = desired_tip - self.tool_length_m * axis
                q_solution = self.model.solve_ik(
                    waypoint_seed,
                    desired_ee,
                    desired_quat,
                    q_reference,
                    self.joint_damping,
                    self.ik_max_iterations,
                    self.ik_residual_threshold,
                )
                q_array = np.asarray(q_solution, dtype=np.float64)
                if (
                    not np.all(np.isfinite(q_array))
                    or np.any(q_array < lower)
                    or np.any(q_array > upper)
                ):
                    path_reachable = False
                    break
                actual_ee, actual_quat = self.model.fk(q_solution)
                position_error = float(np.linalg.norm(actual_ee - desired_ee))
                actual_axis = quat_to_matrix_xyzw(actual_quat)[:, 2]
                axis_error = angular_distance_deg(actual_axis, axis)
                if (
                    position_error > self.approach_ik_position_tolerance_m
                    or axis_error > self.approach_ik_axis_tolerance_deg
                ):
                    path_reachable = False
                    break
                waypoint_solutions.append(q_solution)
                position_errors.append(position_error)
                axis_errors.append(axis_error)
                waypoint_seed = q_solution
            if not path_reachable:
                continue
            joint_motion = wrapped_joint_distance(
                waypoint_solutions[0], q_reference
            )
            preference_error = (
                angular_distance_deg(axis, preferred_axis)
                if preferred_axis is not None
                else 0.0
            )
            score = (
                (preference_error, joint_motion, tilt_deg)
                if preferred_axis is not None
                else (joint_motion, tilt_deg, position_error)
            )
            reachable.append(
                {
                    "score": score,
                    "axis": axis,
                    "tilt_deg": float(tilt_deg),
                    "azimuth_deg": float(azimuth_deg),
                    "q": waypoint_solutions[0],
                    "joint_motion_rad_rms": joint_motion,
                    "position_error_m": max(position_errors),
                    "axis_error_deg": max(axis_errors),
                }
            )

        if not reachable:
            self._publish_approach_selection(
                {
                    "schema_version": "rcm_approach_selection.v1",
                    "status": "NO_REACHABLE_AXIS",
                    "mode": constraint["mode"],
                    "port_point": port_point.tolist(),
                    "center_axis": center_axis.tolist(),
                    "cone_half_angle_deg": half_angle,
                    "sample_count": len(samples),
                }
            )
            return None

        selected = min(reachable, key=lambda candidate: candidate["score"])
        self._publish_approach_selection(
            {
                "schema_version": "rcm_approach_selection.v1",
                "status": "SELECTED",
                "mode": constraint["mode"],
                "port_point": port_point.tolist(),
                "center_axis": center_axis.tolist(),
                "cone_outward_axis": (-center_axis).tolist(),
                "cone_half_angle_deg": half_angle,
                "selected_axis": selected["axis"].tolist(),
                "selected_tilt_deg": selected["tilt_deg"],
                "selected_azimuth_deg": selected["azimuth_deg"],
                "joint_motion_rad_rms": selected["joint_motion_rad_rms"],
                "position_error_m": selected["position_error_m"],
                "axis_error_deg": selected["axis_error_deg"],
                "sample_count": len(samples),
                "reachable_count": len(reachable),
            }
        )
        return selected

    def _try_lock_initial_state(self):
        if (
            self.locked
            or self.approach_search_failed
            or not self.start_requested
            or not self.init_done
            or self.latest_q is None
            or self.stable_count < self.lock_count
        ):
            return
        valid, reason = self._visual_port_pose_is_valid()
        if not valid:
            self.visual_pose_rejection = reason
            return
        self.visual_pose_rejection = ""
        self._lock_initial_state()

    def _lock_initial_state(self):
        self.q0 = self.latest_q[:]
        self.q_seed = self.q0[:]
        self.ee0, self.orn0 = self.model.fk(self.q0)
        rot0 = quat_to_matrix_xyzw(self.orn0)
        self.tool_frame_x0 = rot0[:, 0].copy()
        tool_axis = normalize(rot0[:, 2], fallback=(0.0, 0.0, -1.0))
        self.initial_tip = self.ee0 + tool_axis * self.tool_length_m
        if self.use_initial_rcm:
            self.rcm0 = (
                (1.0 - self.rcm_lambda) * self.ee0
                + self.rcm_lambda * self.initial_tip
            )
        elif self.use_vlm_port_pose:
            self.rcm0 = self.visual_port_point.copy()
        else:
            self.rcm0 = self.rcm_world_param.copy()

        self.plane_point = self.ee0.copy()
        self.line_anchor = self.ee0.copy()

        if self.use_vlm_port_pose:
            shaft = self.visual_port_axis.copy()
            if self.enable_approach_cone:
                selection = self._select_cone_approach_axis(
                    shaft,
                    self.rcm0,
                    self.q0,
                    self.tool_frame_x0,
                )
                if selection is None:
                    self.approach_search_failed = True
                    self.visual_pose_rejection = (
                        "no IK-reachable approach axis inside admissible cone"
                    )
                    self.motion_stage = "NO_REACHABLE_APPROACH"
                    self._publish_stage()
                    self.get_logger().error(
                        "[APPROACH_CONE] no reachable pre-insertion pose; motion blocked"
                    )
                    return
                shaft = selection["axis"].copy()
            aligned_rotation = rotation_matrix_from_z_axis(
                shaft,
                preferred_x=self.tool_frame_x0,
            )
            self.orn0 = quaternion_from_matrix(aligned_rotation)
            self.rcm_ee_distance = self.rcm_lambda * self.tool_length_m
            self.rcm_tip_distance = (
                1.0 - self.rcm_lambda
            ) * self.tool_length_m
            self.tip0 = self.rcm0 + self.rcm_tip_distance * shaft
        elif self.safe_insertion_active:
            shaft = tool_axis
            self.rcm_ee_distance = self.rcm_lambda * self.tool_length_m
            self.rcm_tip_distance = (
                1.0 - self.rcm_lambda
            ) * self.tool_length_m
            self.tip0 = self.rcm0 + self.rcm_tip_distance * shaft
        else:
            self.tip0 = self.initial_tip.copy()
            shaft = normalize(self.tip0 - self.rcm0, fallback=tool_axis)
            self.rcm_ee_distance = float(np.linalg.norm(self.rcm0 - self.ee0))
            self.rcm_tip_distance = float(np.linalg.norm(self.tip0 - self.rcm0))
        self.rcm_shaft0 = shaft.copy()

        ref = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(shaft, ref))) > 0.9:
            ref = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        transverse = normalize(
            np.cross(shaft, ref),
            fallback=(0.0, 1.0, 0.0),
        )
        self.rcm_circle_radius = min(
            self.trajectory_radius_m,
            0.95 * self.rcm_tip_distance,
        )
        self.rcm_circle_tilt = math.asin(
            self.rcm_circle_radius / max(self.rcm_tip_distance, 1e-9)
        )
        cos_tilt = math.cos(self.rcm_circle_tilt)
        sin_tilt = math.sin(self.rcm_circle_tilt)
        self.rcm_circle_axis = normalize(
            cos_tilt * shaft + sin_tilt * transverse,
            fallback=shaft,
        )
        self.rcm_circle_basis_1 = normalize(
            (shaft - cos_tilt * self.rcm_circle_axis) / max(sin_tilt, 1e-9),
            fallback=transverse,
        )
        self.rcm_circle_basis_2 = normalize(
            np.cross(self.rcm_circle_axis, self.rcm_circle_basis_1),
            fallback=np.cross(shaft, transverse),
        )

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self.start_time = now_sec
        self.stage_start_time = now_sec
        self.motion_stage = (
            "PREINSERT_HOLD" if self.safe_insertion_active else "RCM_PIVOT"
        )
        self.stage_settle_count = 0
        self.locked = True
        self._publish_started(True)
        self._publish_stage()

        self.get_logger().info(
            f"[INIT] q0={['%.3f' % value for value in self.q0]}"
        )
        self.get_logger().info(
            "[INIT] "
            f"ee0=({self.ee0[0]:.3f},{self.ee0[1]:.3f},{self.ee0[2]:.3f}) "
            f"initial_tip=({self.initial_tip[0]:.3f},"
            f"{self.initial_tip[1]:.3f},{self.initial_tip[2]:.3f}) "
            f"rcm=({self.rcm0[0]:.3f},{self.rcm0[1]:.3f},{self.rcm0[2]:.3f})"
        )
        self.get_logger().info(
            "[INIT] rigid RCM tool: "
            f"wrist_to_rcm={self.rcm_ee_distance:.3f} m "
            f"rcm_to_tip={self.rcm_tip_distance:.3f} m "
            f"tip_circle_radius={self.rcm_circle_radius:.3f} m"
        )
        if self.safe_insertion_active:
            self.get_logger().info(
                "[SAFE_INSERTION] PREINSERT_HOLD -> APPROACH_PORT -> "
                "PORT_DWELL -> INSERT_THROUGH_PORT -> INSERTED_DWELL -> "
                + (
                    "WAITING_FOR_PIVOT -> RCM_PIVOT"
                    if self.wait_for_pivot_command
                    else "RCM_PIVOT"
                )
            )
        if self.use_vlm_port_pose:
            self.get_logger().info(
                "[VLM_RCM] accepted and frozen visual port pose: "
                f"point=({self.rcm0[0]:.4f},{self.rcm0[1]:.4f},"
                f"{self.rcm0[2]:.4f}) "
                f"axis=({self.rcm_shaft0[0]:.3f},"
                f"{self.rcm_shaft0[1]:.3f},"
                f"{self.rcm_shaft0[2]:.3f})"
            )
            self._publish_point(self.pub_locked_port_point, self.rcm0)
            self._publish_vector(self.pub_locked_port_axis, self.rcm_shaft0)

    def _publish_position_mode(self, now_sec: float):
        if now_sec - self.last_mode_time < 0.5:
            return
        msg = Int8()
        msg.data = self.MODE_POSITION
        self.pub_mode.publish(msg)
        self.last_mode_time = now_sec

    def _elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return max(self.get_clock().now().nanoseconds * 1e-9 - self.start_time, 0.0)

    def _stage_elapsed(self) -> float:
        if self.stage_start_time is None:
            return 0.0
        return max(
            self.get_clock().now().nanoseconds * 1e-9 - self.stage_start_time,
            0.0,
        )

    def _set_motion_stage(self, stage: str):
        if stage == self.motion_stage:
            return
        self.motion_stage = stage
        self.stage_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.stage_settle_count = 0
        self.get_logger().info(f"[SAFE_INSERTION] stage={stage}")
        self._publish_stage()

    def _axial_insertion_target(
        self,
        tip_offset_m: float,
        stage: str,
    ) -> FixtureTarget:
        self.motion_stage = stage
        desired_tip = self.rcm0 + tip_offset_m * self.rcm_shaft0
        ee_pos = desired_tip - self.tool_length_m * self.rcm_shaft0
        return FixtureTarget(
            ee_pos=ee_pos,
            ee_quat=self.orn0,
            desired_tip=desired_tip,
            desired_rcm=self.rcm0,
        )

    def _safe_insertion_target(self) -> FixtureTarget:
        elapsed = self._stage_elapsed()
        preinsert_offset = -self.preinsert_clearance_m
        port_offset = -self.port_standoff_m
        inserted_offset = self.rcm_tip_distance

        if self.motion_stage == "PREINSERT_HOLD":
            return self._axial_insertion_target(
                preinsert_offset,
                "PREINSERT_HOLD",
            )
        if self.motion_stage == "APPROACH_PORT":
            u = min(elapsed / self.approach_duration_sec, 1.0)
            offset = preinsert_offset + min_jerk(u) * (
                port_offset - preinsert_offset
            )
            return self._axial_insertion_target(offset, "APPROACH_PORT")
        if self.motion_stage == "PORT_DWELL":
            return self._axial_insertion_target(port_offset, "PORT_DWELL")
        if self.motion_stage == "INSERT_THROUGH_PORT":
            distance = inserted_offset - port_offset
            duration = distance / self.insertion_speed_mps
            u = min(elapsed / max(duration, 1e-6), 1.0)
            offset = port_offset + min_jerk(u) * distance
            return self._axial_insertion_target(
                offset,
                "INSERT_THROUGH_PORT",
            )
        if self.motion_stage == "INSERTED_DWELL":
            return self._axial_insertion_target(
                inserted_offset,
                "INSERTED_DWELL",
            )
        if self.motion_stage == "WAITING_FOR_PIVOT":
            return self._axial_insertion_target(
                inserted_offset,
                "WAITING_FOR_PIVOT",
            )
        if (
            self.motion_stage == "RCM_HOLD"
            and self.held_rcm_target is not None
        ):
            return self.held_rcm_target
        return self._rcm_target(elapsed)

    def _safe_stage_ready(self, target: FixtureTarget) -> bool:
        current_ee, current_quat = self.model.fk(self.latest_q)
        current_axis = normalize(quat_to_matrix_xyzw(current_quat)[:, 2])
        current_tip = current_ee + self.tool_length_m * current_axis
        position_error = float(np.linalg.norm(current_tip - target.desired_tip))
        if position_error <= self.stage_position_tolerance_m:
            self.stage_settle_count += 1
        else:
            self.stage_settle_count = 0
        return self.stage_settle_count >= self.stage_settle_cycles

    def _maybe_advance_safe_stage(self, target: FixtureTarget):
        if (
            not self.safe_insertion_active
            or self.motion_stage in {"RCM_PIVOT", "RCM_HOLD"}
        ):
            return
        if self.motion_stage == "WAITING_FOR_PIVOT":
            if self.pivot_requested:
                self._set_motion_stage("RCM_PIVOT")
            return

        elapsed = self._stage_elapsed()
        minimum_duration = 0.0
        next_stage = None
        if self.motion_stage == "PREINSERT_HOLD":
            minimum_duration = self.preinsert_hold_sec
            next_stage = "APPROACH_PORT"
        elif self.motion_stage == "APPROACH_PORT":
            minimum_duration = self.approach_duration_sec
            next_stage = "PORT_DWELL"
        elif self.motion_stage == "PORT_DWELL":
            minimum_duration = self.port_dwell_sec
            next_stage = "INSERT_THROUGH_PORT"
        elif self.motion_stage == "INSERT_THROUGH_PORT":
            distance = self.rcm_tip_distance + self.port_standoff_m
            minimum_duration = distance / self.insertion_speed_mps
            next_stage = "INSERTED_DWELL"
        elif self.motion_stage == "INSERTED_DWELL":
            minimum_duration = self.inserted_dwell_sec
            next_stage = (
                "WAITING_FOR_PIVOT"
                if self.wait_for_pivot_command and not self.pivot_requested
                else "RCM_PIVOT"
            )

        if elapsed < minimum_duration:
            self.stage_settle_count = 0
            return
        if self._safe_stage_ready(target):
            self._set_motion_stage(next_stage)

    def _rcm_target(self, elapsed: float) -> FixtureTarget:
        # Pivot a rigid-length tool around the fixed RCM point. The tip follows
        # a spherical small circle, so wrist-to-tip length never changes.
        omega = self.speed_mps / max(self.rcm_circle_radius, 1e-6)
        angle = omega * elapsed
        cos_tilt = math.cos(self.rcm_circle_tilt)
        sin_tilt = math.sin(self.rcm_circle_tilt)
        shaft_axis = normalize(
            cos_tilt * self.rcm_circle_axis
            + sin_tilt
            * (
                math.cos(angle) * self.rcm_circle_basis_1
                + math.sin(angle) * self.rcm_circle_basis_2
            ),
            fallback=self.rcm_shaft0,
        )
        desired_tip = self.rcm0 + self.rcm_tip_distance * shaft_axis
        ee_pos = self.rcm0 - self.rcm_ee_distance * shaft_axis
        ee_quat = quaternion_from_matrix(
            rotation_matrix_from_z_axis(shaft_axis, preferred_x=self.tool_frame_x0)
        )
        return FixtureTarget(
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            desired_tip=desired_tip,
            desired_rcm=self.rcm0,
        )

    def _line_target(self, elapsed: float) -> FixtureTarget:
        period = 2.0 * self.trajectory_length_m / max(self.speed_mps, 1e-6)
        s = triangle01(elapsed / period)
        centered = (s - 0.5) * self.trajectory_length_m
        ee_pos = self.line_anchor + centered * self.line_direction
        raw_offset = self.ee0 - ee_pos
        line_error = float(np.linalg.norm(raw_offset - np.dot(raw_offset, self.line_direction) * self.line_direction))
        return FixtureTarget(
            ee_pos=ee_pos,
            ee_quat=self.orn0,
            line_error=line_error,
        )

    def _plane_target(self, elapsed: float) -> FixtureTarget:
        basis_1 = normalize(np.cross(self.plane_normal, [1.0, 0.0, 0.0]), fallback=(0.0, 1.0, 0.0))
        basis_2 = normalize(np.cross(self.plane_normal, basis_1), fallback=(1.0, 0.0, 0.0))
        omega = self.speed_mps / max(self.trajectory_radius_m, 1e-6)
        raw = (
            self.plane_point
            + self.trajectory_radius_m * math.cos(omega * elapsed) * basis_1
            + self.trajectory_radius_m * math.sin(omega * elapsed) * basis_2
        )
        distance = float(np.dot(raw - self.plane_point, self.plane_normal))
        ee_pos = raw - distance * self.plane_normal
        return FixtureTarget(
            ee_pos=ee_pos,
            ee_quat=self.orn0,
            plane_error=distance,
        )

    def _target_for_mode(self, elapsed: float) -> FixtureTarget:
        if self.fixture_mode == "line":
            return self._line_target(elapsed)
        if self.fixture_mode == "plane":
            return self._plane_target(elapsed)
        if self.safe_insertion_active:
            return self._safe_insertion_target()
        return self._rcm_target(elapsed)

    def _limit_joint_step(self, q_des):
        if self.q_seed is None:
            return q_des[:]
        out = q_des[:]
        for index, value in enumerate(out):
            delta = float(value) - float(self.q_seed[index])
            if delta > self.max_joint_step_rad:
                out[index] = self.q_seed[index] + self.max_joint_step_rad
            elif delta < -self.max_joint_step_rad:
                out[index] = self.q_seed[index] - self.max_joint_step_rad
        return out

    def _publish_point(self, pub, point: np.ndarray):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.point.x = float(point[0])
        msg.point.y = float(point[1])
        msg.point.z = float(point[2])
        pub.publish(msg)

    def _publish_vector(self, pub, vector: np.ndarray):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.vector.x = float(vector[0])
        msg.vector.y = float(vector[1])
        msg.vector.z = float(vector[2])
        pub.publish(msg)

    def _publish_debug(self, elapsed: float, target: FixtureTarget, q_actual):
        current_ee, current_quat = self.model.fk(q_actual)
        rot = quat_to_matrix_xyzw(current_quat)
        shaft_axis = normalize(rot[:, 2])
        current_tip = current_ee + shaft_axis * self.tool_length_m
        if self.safe_insertion_active and target.desired_rcm is not None:
            projection = float(
                np.dot(target.desired_rcm - current_ee, shaft_axis)
            )
            current_rcm = current_ee + projection * shaft_axis
        else:
            current_rcm = (
                (1.0 - self.rcm_lambda) * current_ee
                + self.rcm_lambda * current_tip
            )
        if target.desired_rcm is not None:
            desired_rcm = target.desired_rcm
            rcm_error_vec = current_rcm - desired_rcm
            rcm_error = float(np.linalg.norm(rcm_error_vec))
            desired_rcm_from_ee = desired_rcm - current_ee
            rcm_radial_error = float(
                np.linalg.norm(
                    desired_rcm_from_ee
                    - float(np.dot(desired_rcm_from_ee, shaft_axis)) * shaft_axis
                )
            )
            if self.safe_insertion_active and target.desired_tip is not None:
                rcm_axial_error = float(
                    np.dot(current_tip - target.desired_tip, shaft_axis)
                )
            else:
                rcm_axial_error = float(np.dot(rcm_error_vec, shaft_axis))
            self._publish_point(self.pub_rcm_point, desired_rcm)
        else:
            desired_rcm = current_rcm.copy()
            rcm_error_vec = np.zeros(3, dtype=np.float64)
            rcm_error = 0.0
            rcm_radial_error = 0.0
            rcm_axial_error = 0.0

        ee_error_vec = current_ee - target.ee_pos
        ee_tracking_error = float(np.linalg.norm(ee_error_vec))

        if target.desired_tip is not None:
            desired_tip = target.desired_tip
            tip_error_vec = current_tip - desired_tip
            tip_error = float(np.linalg.norm(tip_error_vec))
            self._publish_point(self.pub_tip_point, desired_tip)
        else:
            desired_tip = current_tip.copy()
            tip_error_vec = np.zeros(3, dtype=np.float64)
            tip_error = 0.0

        if self.fixture_mode == "rcm":
            self._publish_point(self.pub_locked_port_point, self.rcm0)
            self._publish_vector(self.pub_locked_port_axis, self.rcm_shaft0)

        metrics = Float64MultiArray()
        metrics.data = [
            float(elapsed),
            rcm_error,
            tip_error,
            float(target.plane_error),
            float(target.line_error),
            float(target.ee_pos[0]),
            float(target.ee_pos[1]),
            float(target.ee_pos[2]),
            ee_tracking_error,
            rcm_radial_error,
            rcm_axial_error,
        ]
        self.pub_metrics.publish(metrics)

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._record_metrics_csv(
            ros_time_sec=now_sec,
            elapsed=elapsed,
            target=target,
            desired_rcm=desired_rcm,
            current_rcm=current_rcm,
            rcm_error_vec=rcm_error_vec,
            rcm_error=rcm_error,
            rcm_radial_error=rcm_radial_error,
            rcm_axial_error=rcm_axial_error,
            current_ee=current_ee,
            ee_error_vec=ee_error_vec,
            ee_tracking_error=ee_tracking_error,
            desired_tip=desired_tip,
            current_tip=current_tip,
            tip_error_vec=tip_error_vec,
            tip_tracking_error=tip_error,
        )
        if now_sec - self.last_status_time >= 1.0 / self.status_hz:
            msg = String()
            msg.data = (
                f"mode={self.fixture_mode} "
                f"stage={self.motion_stage} "
                f"t={elapsed:.2f} "
                f"rcm_err={rcm_error:.4f} "
                f"rcm_radial={rcm_radial_error:.4f} "
                f"ee_err={ee_tracking_error:.4f} "
                f"tip_err={tip_error:.4f} "
                f"plane_err={target.plane_error:.4f} "
                f"line_err={target.line_error:.4f}"
            )
            self.pub_status.publish(msg)
            self._publish_stage()
            self.last_status_time = now_sec

    def on_timer(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._publish_position_mode(now_sec)

        if not self.locked or self.latest_q is None:
            if now_sec - self.last_status_time >= 1.0 / self.status_hz:
                waiting_for = "robot initialization"
                if self.wait_for_start_command and not self.start_requested:
                    waiting_for = "surgical task start command"
                elif (
                    self.use_vlm_port_pose
                    and self.visual_pose_rejection
                ):
                    waiting_for = self.visual_pose_rejection
                msg = String()
                msg.data = (
                    f"mode={self.fixture_mode} "
                    f"stage={self.motion_stage} "
                    f"waiting_for={waiting_for}"
                )
                self.pub_status.publish(msg)
                self._publish_stage()
                self.last_status_time = now_sec
            return

        elapsed = self._elapsed()
        target = self._target_for_mode(elapsed)
        q_seed = self.q_seed if self.q_seed is not None else self.latest_q
        q_des = self.model.solve_ik(
            q_seed=q_seed,
            target_pos=target.ee_pos,
            target_quat=target.ee_quat,
            rest_poses=self.q0,
            damping=self.joint_damping,
            max_iterations=self.ik_max_iterations,
            residual_threshold=self.ik_residual_threshold,
        )
        q_des = self._limit_joint_step(q_des)
        self.q_seed = q_des[:]

        msg = Float64MultiArray()
        msg.data = [float(value) for value in q_des]
        self.pub_qdes.publish(msg)
        self._maybe_advance_safe_stage(target)
        self._publish_debug(elapsed, target, self.latest_q)

    def destroy_node(self):
        if self.metrics_csv_file is not None:
            try:
                self.metrics_csv_file.flush()
                self.metrics_csv_file.close()
            except OSError:
                pass
            self.metrics_csv_file = None
            self.metrics_csv_writer = None
        self.model.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RcmVirtualFixtureNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
