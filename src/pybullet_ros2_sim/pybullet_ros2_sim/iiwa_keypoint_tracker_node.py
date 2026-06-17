#!/usr/bin/env python3
"""Convert perception targets into joint-space commands for the iiwa arm."""

import threading
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray, Int8, String
from tf2_ros import Buffer, TransformListener

from pybullet_ros2_sim.ik_utils import IiwaIkHelper


def clamp_step(q_now, q_des, max_step):
    """Clamp each joint update so IK output remains continuous."""
    out = []
    for current, desired in zip(q_now, q_des):
        delta = desired - current
        delta = max(-max_step, min(max_step, delta))
        out.append(current + delta)
    return out


def transform_to_matrix(tf_msg):
    """Convert a TF message into a homogeneous 4x4 transform matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w

    rot = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    transform[:3, 3] = [t.x, t.y, t.z]
    return transform


def transform_point(transform, point_xyz):
    """Apply a homogeneous transform to a 3D point."""
    point = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0], dtype=np.float64)
    transformed = transform @ point
    return transformed[:3]


def transform_vector(transform, vector_xyz):
    vector = np.array(vector_xyz, dtype=np.float64).reshape(3)
    return transform[:3, :3] @ vector


def normalize_vector(vector):
    vec = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9 or not np.all(np.isfinite(vec)):
        return None
    return vec / norm


def normalize_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


def yaw_to_quat(yaw_rad: float):
    half = 0.5 * float(yaw_rad)
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def quat_multiply(q_a, q_b):
    ax, ay, az, aw = q_a
    bx, by, bz, bw = q_b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_yaw(q):
    x, y, z, w = q
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotate_about_axis(vector, axis, angle_rad: float):
    vec = np.asarray(vector, dtype=np.float64).reshape(3)
    ax = normalize_vector(axis)
    if ax is None:
        return vec
    angle = float(angle_rad)
    return (
        vec * math.cos(angle)
        + np.cross(ax, vec) * math.sin(angle)
        + ax * float(np.dot(ax, vec)) * (1.0 - math.cos(angle))
    )


def matrix_to_quat(rot: np.ndarray):
    m = np.asarray(rot, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [x / norm, y / norm, z / norm, w / norm]


class IiwaKeypointTracker(Node):
    """Track a perceived 3D keypoint and publish hover pose joint targets."""

    def __init__(self):
        super().__init__("iiwa_keypoint_tracker_node")

        self.lock = threading.Lock()
        self.n = 7

        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("hover_offset_z", 0.10)
        self.declare_parameter("max_joint_step_rad", 0.02)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("keep_initial_orientation", True)
        self.declare_parameter("control_mode_value", 1)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("lock_target_on_tracking_enable", False)
        self.declare_parameter("use_last_target_on_occlusion", False)
        self.declare_parameter("occluded_target_hold_sec", 5.0)
        self.declare_parameter("keypoint_topic", "/perception/keypoint_3d")
        self.declare_parameter("valid_topic", "/perception/valid")
        self.declare_parameter("target_override_topic", "/llm_task/static_target_override")
        self.declare_parameter(
            "target_override_enabled_topic",
            "/llm_task/static_target_override_enabled",
        )
        self.declare_parameter(
            "target_override_plane_normal_topic",
            "/llm_task/static_target_plane_normal",
        )
        self.declare_parameter("static_override_keep_current_orientation", True)
        self.declare_parameter("status_topic", "/iiwa7/keypoint_tracker_status")
        self.declare_parameter("status_publish_period_sec", 0.5)
        self.declare_parameter("enable_topic", "/llm_task/tracking_enabled")
        self.declare_parameter("hover_offset_topic", "/llm_task/tracking_hover_offset_z")
        self.declare_parameter("object_yaw_topic", "/perception/object_yaw_rad")
        self.declare_parameter("object_plane_normal_topic", "/perception/object_plane_normal")
        self.declare_parameter("object_plane_tangent_topic", "/perception/object_plane_tangent")
        self.declare_parameter("align_to_object_yaw", True)
        self.declare_parameter("use_object_plane_frame", True)
        self.declare_parameter("require_object_plane_frame", False)
        self.declare_parameter("object_plane_timeout_sec", 1.0)
        self.declare_parameter("object_yaw_timeout_sec", 1.0)
        self.declare_parameter("grasp_yaw_offset_rad", 1.57079632679)
        self.declare_parameter("align_hover_offset_margin_m", 0.04)
        self.declare_parameter("use_explicit_grasp_frame", False)
        self.declare_parameter("gripper_forward_down_angle_rad", 0.93)
        self.declare_parameter("keep_gripper_vertical_to_table", False)
        self.declare_parameter("track_gripper_center", False)
        self.declare_parameter("gripper_center_offset_link6", [0.0, 0.0, 0.0])
        self.declare_parameter("min_gripper_center_z", -1.0)
        self.declare_parameter("ik_use_nullspace_rest", False)
        self.declare_parameter("ik_nullspace_initial_weight", 0.70)
        self.declare_parameter("ik_joint_damping", [0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12])
        self.declare_parameter("start_enabled", True)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.hover_offset_z = float(self.get_parameter("hover_offset_z").value)
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.keep_initial_orientation = bool(
            self.get_parameter("keep_initial_orientation").value
        )
        self.control_mode_value = int(self.get_parameter("control_mode_value").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.lock_target_on_tracking_enable = bool(
            self.get_parameter("lock_target_on_tracking_enable").value
        )
        self.use_last_target_on_occlusion = bool(
            self.get_parameter("use_last_target_on_occlusion").value
        )
        self.occluded_target_hold_sec = float(
            self.get_parameter("occluded_target_hold_sec").value
        )
        self.keypoint_topic = str(self.get_parameter("keypoint_topic").value)
        self.valid_topic = str(self.get_parameter("valid_topic").value)
        self.target_override_topic = str(self.get_parameter("target_override_topic").value)
        self.target_override_enabled_topic = str(
            self.get_parameter("target_override_enabled_topic").value
        )
        self.target_override_plane_normal_topic = str(
            self.get_parameter("target_override_plane_normal_topic").value
        )
        self.static_override_keep_current_orientation = bool(
            self.get_parameter("static_override_keep_current_orientation").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.status_publish_period_sec = max(
            0.0,
            float(self.get_parameter("status_publish_period_sec").value),
        )
        self.enable_topic = str(self.get_parameter("enable_topic").value)
        self.hover_offset_topic = str(self.get_parameter("hover_offset_topic").value)
        self.object_yaw_topic = str(self.get_parameter("object_yaw_topic").value)
        self.object_plane_normal_topic = str(
            self.get_parameter("object_plane_normal_topic").value
        )
        self.object_plane_tangent_topic = str(
            self.get_parameter("object_plane_tangent_topic").value
        )
        self.align_to_object_yaw = bool(self.get_parameter("align_to_object_yaw").value)
        self.use_object_plane_frame = bool(
            self.get_parameter("use_object_plane_frame").value
        )
        self.require_object_plane_frame = bool(
            self.get_parameter("require_object_plane_frame").value
        )
        self.object_plane_timeout_sec = float(
            self.get_parameter("object_plane_timeout_sec").value
        )
        self.object_yaw_timeout_sec = float(self.get_parameter("object_yaw_timeout_sec").value)
        self.grasp_yaw_offset_rad = float(self.get_parameter("grasp_yaw_offset_rad").value)
        self.align_hover_offset_margin_m = float(
            self.get_parameter("align_hover_offset_margin_m").value
        )
        self.use_explicit_grasp_frame = bool(
            self.get_parameter("use_explicit_grasp_frame").value
        )
        self.gripper_forward_down_angle_rad = float(
            self.get_parameter("gripper_forward_down_angle_rad").value
        )
        self.keep_gripper_vertical_to_table = bool(
            self.get_parameter("keep_gripper_vertical_to_table").value
        )
        self.track_gripper_center = bool(self.get_parameter("track_gripper_center").value)
        self.gripper_center_offset_link6 = np.array(
            [float(value) for value in self.get_parameter("gripper_center_offset_link6").value][
                :3
            ],
            dtype=np.float64,
        )
        if self.gripper_center_offset_link6.shape[0] < 3:
            self.gripper_center_offset_link6 = np.pad(
                self.gripper_center_offset_link6,
                (0, 3 - self.gripper_center_offset_link6.shape[0]),
            )
        self.min_gripper_center_z = float(self.get_parameter("min_gripper_center_z").value)
        self.ik_use_nullspace_rest = bool(
            self.get_parameter("ik_use_nullspace_rest").value
        )
        self.ik_nullspace_initial_weight = float(
            self.get_parameter("ik_nullspace_initial_weight").value
        )
        self.ik_joint_damping = np.array(
            [float(value) for value in self.get_parameter("ik_joint_damping").value][: self.n],
            dtype=np.float64,
        )
        if self.ik_joint_damping.shape[0] < self.n:
            self.ik_joint_damping = np.pad(
                self.ik_joint_damping,
                (0, self.n - self.ik_joint_damping.shape[0]),
                constant_values=0.12,
            )
        self.tracking_enabled = bool(self.get_parameter("start_enabled").value)
        self.base_hover_offset_z = float(self.hover_offset_z)

        self.sub_keypoint = self.create_subscription(
            PointStamped,
            self.keypoint_topic,
            self.on_keypoint,
            10,
        )
        self.sub_valid = self.create_subscription(Bool, self.valid_topic, self.on_valid, 10)
        self.sub_target_override_enabled = self.create_subscription(
            Bool,
            self.target_override_enabled_topic,
            self.on_target_override_enabled,
            10,
        )
        self.sub_target_override = self.create_subscription(
            PointStamped,
            self.target_override_topic,
            self.on_target_override,
            10,
        )
        self.sub_target_override_plane_normal = self.create_subscription(
            Vector3Stamped,
            self.target_override_plane_normal_topic,
            self.on_target_override_plane_normal,
            10,
        )
        self.sub_js = self.create_subscription(JointState, "/iiwa7/joint_states", self.on_js, 10)
        self.sub_enable = self.create_subscription(Bool, self.enable_topic, self.on_enable, 10)
        self.sub_hover_offset = self.create_subscription(
            Float64,
            self.hover_offset_topic,
            self.on_hover_offset,
            10,
        )
        self.sub_object_yaw = self.create_subscription(
            Float32,
            self.object_yaw_topic,
            self.on_object_yaw,
            10,
        )
        self.sub_object_plane_normal = self.create_subscription(
            Vector3Stamped,
            self.object_plane_normal_topic,
            self.on_object_plane_normal,
            10,
        )
        self.sub_object_plane_tangent = self.create_subscription(
            Vector3Stamped,
            self.object_plane_tangent_topic,
            self.on_object_plane_tangent,
            10,
        )

        self.pub_mode = self.create_publisher(Int8, "/iiwa7/control_mode", 10)
        self.pub_qdes = self.create_publisher(Float64MultiArray, "/iiwa7/joint_desired", 10)
        self.pub_status = self.create_publisher(String, self.status_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik = IiwaIkHelper()

        self.q_now = None
        self.target_cam = None
        self.target_cam_header = None
        self.target_valid = False
        self.target_override_enabled = False
        self.target_override_cam = None
        self.target_override_header = None
        self.target_override_received_ns = 0
        self.target_override_plane_normal = None
        self.target_override_plane_normal_header = None
        self.target_override_plane_normal_received_ns = 0
        self.last_valid_target_cam = None
        self.last_valid_target_header = None
        self.last_valid_target_received_ns = 0
        self.last_valid_plane_normal = None
        self.last_valid_plane_normal_header = None
        self.last_valid_plane_normal_received_ns = 0
        self.last_valid_plane_tangent = None
        self.last_valid_plane_tangent_header = None
        self.last_valid_plane_tangent_received_ns = 0
        self.locked_target_cam = None
        self.locked_target_header = None
        self.locked_plane_normal = None
        self.locked_plane_normal_header = None
        self.locked_plane_tangent = None
        self.locked_plane_tangent_header = None
        self.object_yaw_rad = None
        self.object_yaw_received_ns = 0
        self.object_plane_normal = None
        self.object_plane_normal_header = None
        self.object_plane_normal_received_ns = 0
        self.object_plane_tangent = None
        self.object_plane_tangent_header = None
        self.object_plane_tangent_received_ns = 0

        self.have_initial_ee = False
        self.ee_orn0 = None
        self.ee_yaw0 = None
        self.tracking_start_orn = None
        self.q_nullspace_rest0 = None
        self.last_status_text = ""
        self.last_status_ns = 0

        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1e-6), self.on_timer)
        self.get_logger().info(
            "iiwa_keypoint_tracker_node started "
            f"(tracking_enabled={self.tracking_enabled}, "
            f"keypoint_topic={self.keypoint_topic}, valid_topic={self.valid_topic}, "
            f"target_override_topic={self.target_override_topic}, "
            f"enable_topic={self.enable_topic}, status_topic={self.status_topic})"
        )

    def on_enable(self, msg: Bool):
        q_snapshot = None
        should_capture_orientation = False
        with self.lock:
            was_enabled = self.tracking_enabled
            self.tracking_enabled = bool(msg.data)
            if self.tracking_enabled and not was_enabled:
                self.locked_target_cam = None
                self.locked_target_header = None
                self.locked_plane_normal = None
                self.locked_plane_normal_header = None
                self.locked_plane_tangent = None
                self.locked_plane_tangent_header = None
                q_snapshot = None if self.q_now is None else list(self.q_now)
                should_capture_orientation = q_snapshot is not None
            elif not self.tracking_enabled:
                self.tracking_start_orn = None
        if should_capture_orientation:
            try:
                _pos, orn = self.ik.fk(q_snapshot)
            except Exception:
                return
            with self.lock:
                self.tracking_start_orn = orn

    def on_hover_offset(self, msg: Float64):
        with self.lock:
            self.hover_offset_z = float(msg.data)

    def on_object_yaw(self, msg: Float32):
        with self.lock:
            self.object_yaw_rad = float(msg.data)
            self.object_yaw_received_ns = self.get_clock().now().nanoseconds

    def on_object_plane_normal(self, msg: Vector3Stamped):
        normal = normalize_vector([msg.vector.x, msg.vector.y, msg.vector.z])
        if normal is None:
            return
        with self.lock:
            self.object_plane_normal = normal
            self.object_plane_normal_header = msg.header
            self.object_plane_normal_received_ns = self.get_clock().now().nanoseconds

    def on_object_plane_tangent(self, msg: Vector3Stamped):
        tangent = normalize_vector([msg.vector.x, msg.vector.y, msg.vector.z])
        if tangent is None:
            return
        with self.lock:
            self.object_plane_tangent = tangent
            self.object_plane_tangent_header = msg.header
            self.object_plane_tangent_received_ns = self.get_clock().now().nanoseconds

    def on_valid(self, msg: Bool):
        with self.lock:
            self.target_valid = bool(msg.data)

    def on_keypoint(self, msg: PointStamped):
        with self.lock:
            self.target_cam = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
            self.target_cam_header = msg.header

    def on_target_override_enabled(self, msg: Bool):
        with self.lock:
            self.target_override_enabled = bool(msg.data)

    def on_target_override(self, msg: PointStamped):
        with self.lock:
            self.target_override_cam = np.array(
                [msg.point.x, msg.point.y, msg.point.z],
                dtype=np.float64,
            )
            self.target_override_header = msg.header
            self.target_override_received_ns = self.get_clock().now().nanoseconds

    def on_target_override_plane_normal(self, msg: Vector3Stamped):
        normal = normalize_vector([msg.vector.x, msg.vector.y, msg.vector.z])
        if normal is None:
            return
        with self.lock:
            self.target_override_plane_normal = normal
            self.target_override_plane_normal_header = msg.header
            self.target_override_plane_normal_received_ns = self.get_clock().now().nanoseconds

    def on_js(self, msg: JointState):
        if len(msg.position) < self.n:
            return

        q_now = list(msg.position[:self.n])
        with self.lock:
            self.q_now = q_now

        # Preserve the initial end-effector orientation so the tracker only
        # translates the arm unless the user explicitly disables this behavior.
        if not self.have_initial_ee:
            _pos, orn = self.ik.fk(q_now)
            self.ee_orn0 = orn
            self.ee_yaw0 = quat_yaw(orn)
            self.q_nullspace_rest0 = np.array(q_now, dtype=np.float64)
            self.have_initial_ee = True

    def _ik_rest_pose(self, q_now):
        if not self.ik_use_nullspace_rest or self.q_nullspace_rest0 is None:
            return None
        current = np.asarray(q_now, dtype=np.float64)
        initial = np.asarray(self.q_nullspace_rest0, dtype=np.float64)
        if current.shape[0] < self.n or initial.shape[0] < self.n:
            return None
        weight = float(np.clip(self.ik_nullspace_initial_weight, 0.0, 1.0))
        rest = weight * initial[: self.n] + (1.0 - weight) * current[: self.n]
        return rest.tolist()

    def _object_yaw_is_fresh(self) -> bool:
        if self.object_yaw_rad is None:
            return False
        if self.object_yaw_timeout_sec <= 0.0:
            return True
        age_sec = (self.get_clock().now().nanoseconds - self.object_yaw_received_ns) * 1e-9
        return age_sec <= self.object_yaw_timeout_sec

    def _vector_is_fresh(self, received_ns: int) -> bool:
        if received_ns <= 0:
            return False
        if self.object_plane_timeout_sec <= 0.0:
            return True
        age_sec = (self.get_clock().now().nanoseconds - int(received_ns)) * 1e-9
        return age_sec <= self.object_plane_timeout_sec

    def _vector_in_target_frame(self, vector, header):
        vec = normalize_vector(vector)
        if vec is None:
            return None
        frame_id = "" if header is None else str(header.frame_id)
        if not frame_id or frame_id == self.target_frame:
            return vec
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.target_frame, frame_id, Time())
        except Exception:
            return None
        out = normalize_vector(transform_vector(transform_to_matrix(tf_msg), vec))
        return out

    def _target_orientation(self, plane_normal=None, plane_tangent=None):
        if not self.keep_initial_orientation or self.ee_orn0 is None:
            return None
        if self.use_object_plane_frame and plane_normal is not None:
            normal = normalize_vector(plane_normal)
            if normal is not None:
                if normal[2] < 0.0:
                    normal = -normal
                forward_axis = -normal
                opening_axis = None
                if plane_tangent is not None:
                    opening_axis = normalize_vector(
                        rotate_about_axis(plane_tangent, normal, self.grasp_yaw_offset_rad)
                    )
                if opening_axis is None and self.object_yaw_rad is not None and self._object_yaw_is_fresh():
                    target_yaw = normalize_angle(self.object_yaw_rad + self.grasp_yaw_offset_rad)
                    opening_axis = np.array(
                        [math.cos(target_yaw), math.sin(target_yaw), 0.0],
                        dtype=np.float64,
                    )
                if opening_axis is None:
                    opening_axis = quat_to_matrix(self.ee_orn0)[:, 0]
                opening_axis = opening_axis - float(np.dot(opening_axis, normal)) * normal
                opening_axis = normalize_vector(opening_axis)
                if opening_axis is None:
                    opening_axis = normalize_vector(np.cross([0.0, 1.0, 0.0], normal))
                if opening_axis is None:
                    opening_axis = normalize_vector(np.cross([1.0, 0.0, 0.0], normal))
                if opening_axis is not None:
                    palm_axis = np.cross(forward_axis, opening_axis)
                    palm_axis = normalize_vector(palm_axis)
                    if palm_axis is not None:
                        forward_axis = normalize_vector(np.cross(opening_axis, palm_axis))
                        if forward_axis is not None:
                            rot = np.column_stack((opening_axis, palm_axis, forward_axis))
                            return matrix_to_quat(rot)
        in_alignment_height = (
            self.hover_offset_z
            <= self.base_hover_offset_z + max(self.align_hover_offset_margin_m, 0.0)
        )
        if (
            in_alignment_height
            and self.align_to_object_yaw
            and self.object_yaw_rad is not None
            and self.ee_yaw0 is not None
            and self._object_yaw_is_fresh()
        ):
            target_yaw = normalize_angle(self.object_yaw_rad + self.grasp_yaw_offset_rad)
            if self.use_explicit_grasp_frame:
                opening_axis = np.array(
                    [math.cos(target_yaw), math.sin(target_yaw), 0.0],
                    dtype=np.float64,
                )
                if self.keep_gripper_vertical_to_table:
                    forward_axis = np.array([0.0, 0.0, -1.0], dtype=np.float64)
                else:
                    forward_yaw = normalize_angle(target_yaw - math.pi / 2.0)
                    down_angle = float(
                        np.clip(self.gripper_forward_down_angle_rad, 0.05, 1.45)
                    )
                    forward_axis = np.array(
                        [
                            math.cos(forward_yaw) * math.cos(down_angle),
                            math.sin(forward_yaw) * math.cos(down_angle),
                            -math.sin(down_angle),
                        ],
                        dtype=np.float64,
                    )
                palm_axis = np.cross(forward_axis, opening_axis)
                palm_norm = float(np.linalg.norm(palm_axis))
                if palm_norm > 1e-9:
                    palm_axis = palm_axis / palm_norm
                    forward_axis = np.cross(opening_axis, palm_axis)
                    forward_axis = forward_axis / max(float(np.linalg.norm(forward_axis)), 1e-9)
                    rot = np.column_stack((opening_axis, palm_axis, forward_axis))
                    return matrix_to_quat(rot)
            delta_yaw = normalize_angle(target_yaw - self.ee_yaw0)
            return quat_multiply(yaw_to_quat(delta_yaw), self.ee_orn0)
        return self.ee_orn0

    def _publish_position_mode(self):
        msg = Int8()
        msg.data = self.control_mode_value
        self.pub_mode.publish(msg)

    def _publish_status(self, text: str):
        now_ns = self.get_clock().now().nanoseconds
        period_ns = int(self.status_publish_period_sec * 1e9)
        if (
            text == self.last_status_text
            and period_ns > 0
            and now_ns - self.last_status_ns < period_ns
        ):
            return
        self.last_status_text = text
        self.last_status_ns = now_ns
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _header_age_sec(self, header):
        if header is None:
            return None
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return 0.0
        return (self.get_clock().now() - Time.from_msg(header.stamp)).nanoseconds * 1e-9

    def _target_is_fresh(self, header) -> bool:
        if self.target_timeout_sec <= 0.0:
            return True
        if header is None:
            return False
        age = self._header_age_sec(header)
        if age is None:
            return False
        return age <= self.target_timeout_sec

    def _cached_target_is_usable(self, now_ns: int, received_ns: int) -> bool:
        if not self.use_last_target_on_occlusion:
            return False
        if received_ns <= 0:
            return False
        hold_sec = max(self.occluded_target_hold_sec, 0.0)
        if hold_sec <= 0.0:
            return False
        age_sec = (now_ns - int(received_ns)) * 1e-9
        return age_sec <= hold_sec

    def on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        with self.lock:
            q_now = None if self.q_now is None else list(self.q_now)
            target_valid = self.target_valid
            target_cam = None if self.target_cam is None else np.array(self.target_cam, copy=True)
            header = self.target_cam_header
            target_override_enabled = self.target_override_enabled
            target_override_cam = (
                None
                if self.target_override_cam is None
                else np.array(self.target_override_cam, copy=True)
            )
            target_override_header = self.target_override_header
            target_override_received_ns = self.target_override_received_ns
            target_override_plane_normal = (
                None
                if self.target_override_plane_normal is None
                else np.array(self.target_override_plane_normal, copy=True)
            )
            target_override_plane_normal_header = self.target_override_plane_normal_header
            target_override_plane_normal_received_ns = (
                self.target_override_plane_normal_received_ns
            )
            tracking_enabled = self.tracking_enabled
            hover_offset_z = self.hover_offset_z
            plane_normal = (
                None
                if self.object_plane_normal is None
                else np.array(self.object_plane_normal, copy=True)
            )
            plane_normal_header = self.object_plane_normal_header
            plane_normal_received_ns = self.object_plane_normal_received_ns
            plane_tangent = (
                None
                if self.object_plane_tangent is None
                else np.array(self.object_plane_tangent, copy=True)
            )
            plane_tangent_header = self.object_plane_tangent_header
            plane_tangent_received_ns = self.object_plane_tangent_received_ns
            last_target_cam = (
                None
                if self.last_valid_target_cam is None
                else np.array(self.last_valid_target_cam, copy=True)
            )
            last_target_header = self.last_valid_target_header
            last_target_received_ns = self.last_valid_target_received_ns
            last_plane_normal = (
                None
                if self.last_valid_plane_normal is None
                else np.array(self.last_valid_plane_normal, copy=True)
            )
            last_plane_normal_header = self.last_valid_plane_normal_header
            last_plane_normal_received_ns = self.last_valid_plane_normal_received_ns
            last_plane_tangent = (
                None
                if self.last_valid_plane_tangent is None
                else np.array(self.last_valid_plane_tangent, copy=True)
            )
            last_plane_tangent_header = self.last_valid_plane_tangent_header
            last_plane_tangent_received_ns = self.last_valid_plane_tangent_received_ns
            locked_target_cam = (
                None
                if self.locked_target_cam is None
                else np.array(self.locked_target_cam, copy=True)
            )
            locked_target_header = self.locked_target_header
            locked_plane_normal = (
                None
                if self.locked_plane_normal is None
                else np.array(self.locked_plane_normal, copy=True)
            )
            locked_plane_normal_header = self.locked_plane_normal_header
            locked_plane_tangent = (
                None
                if self.locked_plane_tangent is None
                else np.array(self.locked_plane_tangent, copy=True)
            )
            locked_plane_tangent_header = self.locked_plane_tangent_header
            tracking_start_orn = self.tracking_start_orn

        if not tracking_enabled:
            self._publish_status("disabled")
            return
        if q_now is None:
            self._publish_status("waiting_for_joint_states")
            return

        target_fresh = (
            target_cam is not None and header is not None and self._target_is_fresh(header)
        )
        using_occluded_target = False
        using_locked_target = False
        using_static_override = False
        if (
            target_override_enabled
            and target_override_cam is not None
            and target_override_header is not None
            and self._target_is_fresh(target_override_header)
        ):
            target_cam = target_override_cam
            header = target_override_header
            target_valid = True
            target_fresh = True
            using_static_override = True
            if target_override_plane_normal is not None:
                plane_normal = target_override_plane_normal
                plane_normal_header = target_override_plane_normal_header
                plane_normal_received_ns = target_override_plane_normal_received_ns
            with self.lock:
                self.last_valid_target_cam = np.array(target_cam, copy=True)
                self.last_valid_target_header = header
                self.last_valid_target_received_ns = target_override_received_ns or now_ns
                if self.lock_target_on_tracking_enable:
                    self.locked_target_cam = np.array(target_cam, copy=True)
                    self.locked_target_header = header
                    if plane_normal is not None:
                        self.locked_plane_normal = np.array(plane_normal, copy=True)
                        self.locked_plane_normal_header = plane_normal_header
            using_locked_target = bool(self.lock_target_on_tracking_enable)
        elif (
            self.lock_target_on_tracking_enable
            and locked_target_cam is not None
            and locked_target_header is not None
        ):
            target_cam = locked_target_cam
            header = locked_target_header
            target_valid = True
            target_fresh = True
            using_locked_target = True
            if locked_plane_normal is not None:
                plane_normal = locked_plane_normal
                plane_normal_header = locked_plane_normal_header
                plane_normal_received_ns = now_ns
            if locked_plane_tangent is not None:
                plane_tangent = locked_plane_tangent
                plane_tangent_header = locked_plane_tangent_header
                plane_tangent_received_ns = now_ns
        elif target_valid and target_fresh:
            with self.lock:
                self.last_valid_target_cam = np.array(target_cam, copy=True)
                self.last_valid_target_header = header
                self.last_valid_target_received_ns = now_ns
                if plane_normal is not None and self._vector_is_fresh(plane_normal_received_ns):
                    self.last_valid_plane_normal = np.array(plane_normal, copy=True)
                    self.last_valid_plane_normal_header = plane_normal_header
                    self.last_valid_plane_normal_received_ns = now_ns
                if plane_tangent is not None and self._vector_is_fresh(plane_tangent_received_ns):
                    self.last_valid_plane_tangent = np.array(plane_tangent, copy=True)
                    self.last_valid_plane_tangent_header = plane_tangent_header
                    self.last_valid_plane_tangent_received_ns = now_ns
                if self.lock_target_on_tracking_enable:
                    self.locked_target_cam = np.array(target_cam, copy=True)
                    self.locked_target_header = header
                    if plane_normal is not None and self._vector_is_fresh(plane_normal_received_ns):
                        self.locked_plane_normal = np.array(plane_normal, copy=True)
                        self.locked_plane_normal_header = plane_normal_header
                    if plane_tangent is not None and self._vector_is_fresh(plane_tangent_received_ns):
                        self.locked_plane_tangent = np.array(plane_tangent, copy=True)
                        self.locked_plane_tangent_header = plane_tangent_header
                    using_locked_target = True
        elif (
            self._cached_target_is_usable(now_ns, last_target_received_ns)
            and last_target_cam is not None
            and last_target_header is not None
        ):
            target_cam = last_target_cam
            header = last_target_header
            target_valid = True
            target_fresh = True
            using_occluded_target = True
            if (
                last_plane_normal is not None
                and self._cached_target_is_usable(now_ns, last_plane_normal_received_ns)
            ):
                plane_normal = last_plane_normal
                plane_normal_header = last_plane_normal_header
                plane_normal_received_ns = now_ns
            if (
                last_plane_tangent is not None
                and self._cached_target_is_usable(now_ns, last_plane_tangent_received_ns)
            ):
                plane_tangent = last_plane_tangent
                plane_tangent_header = last_plane_tangent_header
                plane_tangent_received_ns = now_ns

        if target_cam is None or header is None:
            missing = []
            if target_cam is None or header is None:
                missing.append("keypoint")
            self._publish_status("waiting_for_" + "_and_".join(missing))
            return
        if not target_valid:
            self._publish_status(f"waiting_valid_false topic={self.valid_topic}")
            return
        if not target_fresh:
            age = self._header_age_sec(header)
            age_text = "unknown" if age is None else f"{age:.2f}s"
            self._publish_status(
                f"waiting_stale_keypoint age={age_text} timeout={self.target_timeout_sec:.2f}s"
            )
            return

        source_frame = str(header.frame_id or "")
        if not source_frame or source_frame == self.target_frame:
            point_world = np.asarray(target_cam, dtype=np.float64)
        else:
            try:
                tf_msg = self.tf_buffer.lookup_transform(self.target_frame, source_frame, Time())
            except Exception as exc:
                self._publish_status(
                    f"tf_failed target={self.target_frame} source={source_frame}"
                )
                self.get_logger().warn(f"TF lookup failed: {exc}")
                return
            transform = transform_to_matrix(tf_msg)
            point_world = transform_point(transform, target_cam)
        approach_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        plane_normal_world = None
        plane_tangent_world = None
        if self.use_object_plane_frame and self._vector_is_fresh(plane_normal_received_ns):
            plane_normal_world = self._vector_in_target_frame(plane_normal, plane_normal_header)
            if plane_normal_world is not None:
                if plane_normal_world[2] < 0.0:
                    plane_normal_world = -plane_normal_world
                approach_axis = plane_normal_world
        if (
            self.use_object_plane_frame
            and self.require_object_plane_frame
            and plane_normal_world is None
        ):
            self._publish_status(f"waiting_object_plane_normal topic={self.object_plane_normal_topic}")
            return
        if self.use_object_plane_frame and self._vector_is_fresh(plane_tangent_received_ns):
            plane_tangent_world = self._vector_in_target_frame(plane_tangent, plane_tangent_header)

        # Track a hover point above the detected keypoint so perception jitter
        # does not immediately drive the end-effector into the object.
        hover_point = np.array(point_world + approach_axis * hover_offset_z, dtype=np.float64)
        if self.track_gripper_center and self.min_gripper_center_z >= 0.0:
            hover_point[2] = max(hover_point[2], self.min_gripper_center_z)

        target_orn = self._target_orientation(plane_normal_world, plane_tangent_world)
        if (
            using_static_override
            and self.static_override_keep_current_orientation
            and tracking_start_orn is not None
        ):
            target_orn = tracking_start_orn
        ik_target = np.array(hover_point, copy=True)
        if self.track_gripper_center and target_orn is not None:
            # The IK model targets iiwa link 6, while the grasp task is expressed
            # at the WSG50 fingertip midpoint. Back-project that control point
            # through the desired wrist orientation.
            ik_target = ik_target - quat_to_matrix(target_orn) @ self.gripper_center_offset_link6

        q_des = self.ik.solve_ik(
            q_now,
            ik_target.tolist(),
            target_orn,
            rest_poses=self._ik_rest_pose(q_now),
            joint_damping=self.ik_joint_damping.tolist(),
        )

        q_des = clamp_step(q_now, q_des, self.max_joint_step_rad)

        self._publish_position_mode()

        msg = Float64MultiArray()
        msg.data = q_des
        self.pub_qdes.publish(msg)
        if using_static_override:
            source_text = "static_target_override"
        elif using_locked_target:
            source_text = "locked_keypoint"
        elif using_occluded_target:
            source_text = "cached_occluded_keypoint"
        else:
            source_text = "live_keypoint"
        self._publish_status(
            f"following hover={hover_offset_z:.3f}m keypoint_topic={self.keypoint_topic} "
            f"plane={'yes' if plane_normal_world is not None else 'no'} source={source_text}"
        )

    def destroy_node(self):
        try:
            self.ik.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IiwaKeypointTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
