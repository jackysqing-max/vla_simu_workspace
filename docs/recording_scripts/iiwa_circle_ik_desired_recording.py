#!/usr/bin/env python3
"""Recording-only circular Cartesian trajectory publisher for the iiwa demo."""

from __future__ import annotations

import math

import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


class IiwaCircleIKDesired(Node):
    """Publish smooth joint targets for a circular end-effector path."""

    def __init__(self):
        super().__init__("iiwa_circle_ik_desired_recording")

        self.n = 7
        self.pub = self.create_publisher(Float64MultiArray, "/iiwa7/joint_desired", 10)

        self.declare_parameter("publish_hz", 200.0)
        self.declare_parameter("speed_mps", 0.04)
        self.declare_parameter("radius_m", 0.035)
        self.declare_parameter("plane", "xy")
        self.declare_parameter("use_initial_orientation", True)
        self.declare_parameter("enable_joint_step_limit", True)
        self.declare_parameter("max_joint_step_rad", 0.018)
        self.declare_parameter("enable_velocity_lock", False)
        self.declare_parameter("lock_vel_eps", 0.02)
        self.declare_parameter("lock_count", 30)
        self.declare_parameter("allow_lock_without_init_done", True)
        self.declare_parameter("startup_delay_sec", 2.8)

        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.radius_m = max(1e-4, float(self.get_parameter("radius_m").value))
        self.plane = str(self.get_parameter("plane").value).strip().lower()
        self.use_initial_orientation = bool(
            self.get_parameter("use_initial_orientation").value
        )
        self.enable_joint_step_limit = bool(
            self.get_parameter("enable_joint_step_limit").value
        )
        self.max_joint_step_rad = float(self.get_parameter("max_joint_step_rad").value)
        self.enable_velocity_lock = bool(
            self.get_parameter("enable_velocity_lock").value
        )
        self.lock_vel_eps = float(self.get_parameter("lock_vel_eps").value)
        self.lock_count = int(self.get_parameter("lock_count").value)
        self.allow_lock_without_init_done = bool(
            self.get_parameter("allow_lock_without_init_done").value
        )
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.period = 2.0 * math.pi * self.radius_m / max(self.speed_mps, 1e-6)

        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        self.ee_link = self.n - 1

        self.init_done = False
        self.have_init = False
        self.q_seed = None
        self.p0 = None
        self.orn0 = None
        self.t0 = None
        self.node_start = self.get_clock().now()
        self._stable_count = 0
        self._fallback_init_logged = False

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, "/iiwa7/init_done", self.cb_init_done, qos_latch)
        self.create_subscription(JointState, "/iiwa7/joint_states", self.cb_joint_states, 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info(
            "Iiwa circle IK recording node started. "
            f"radius={self.radius_m:.3f}m speed={self.speed_mps:.3f}m/s "
            f"period={self.period:.2f}s plane={self.plane} "
            f"velocity_lock={self.enable_velocity_lock}"
        )

    def cb_init_done(self, msg: Bool):
        self.init_done = bool(msg.data)
        if self.init_done:
            self.get_logger().info("[INIT] received /iiwa7/init_done=True")

    def cb_joint_states(self, msg: JointState):
        if self.have_init:
            return
        if not msg.position or len(msg.position) < self.n:
            return

        if not self.init_done:
            if not self.allow_lock_without_init_done:
                return
            elapsed = (self.get_clock().now() - self.node_start).nanoseconds * 1e-9
            if elapsed < self.startup_delay_sec:
                return
            if not self._fallback_init_logged:
                self.get_logger().warning(
                    "[INIT] /iiwa7/init_done not observed; locking current joint state "
                    f"after {elapsed:.1f}s for recording."
                )
                self._fallback_init_logged = True

        if self.enable_velocity_lock:
            if (not msg.velocity) or len(msg.velocity) < self.n:
                return
            vmax = max(abs(value) for value in msg.velocity[: self.n])
            if vmax < self.lock_vel_eps:
                self._stable_count += 1
            else:
                self._stable_count = 0
            if self._stable_count < self.lock_count:
                return

        q0 = list(msg.position[: self.n])
        for joint_index in range(self.n):
            p.resetJointState(self.robot_id, joint_index, q0[joint_index])

        link_state = p.getLinkState(
            self.robot_id,
            self.ee_link,
            computeForwardKinematics=True,
        )
        self.p0 = link_state[4]
        self.orn0 = link_state[5]
        self.q_seed = q0[:]
        self.have_init = True
        self.t0 = None
        self.get_logger().info(
            f"[INIT] locked q0={['%.3f' % value for value in q0]}"
        )
        self.get_logger().info(
            f"[INIT] circular Cartesian center=({self.p0[0]:.3f},{self.p0[1]:.3f},{self.p0[2]:.3f})"
        )

    def _target_position(self, theta: float):
        c = math.cos(theta)
        s = math.sin(theta)
        dx = self.radius_m * (c - 1.0)
        dy = self.radius_m * s

        if self.plane == "xz":
            return [self.p0[0] + dx, self.p0[1], self.p0[2] + dy]
        if self.plane == "yz":
            return [self.p0[0], self.p0[1] + dx, self.p0[2] + dy]
        return [self.p0[0] + dx, self.p0[1] + dy, self.p0[2]]

    def on_timer(self):
        if not self.have_init:
            return

        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
            elapsed = 0.0
        else:
            elapsed = (now - self.t0).nanoseconds * 1e-9

        theta = 2.0 * math.pi * ((elapsed / self.period) % 1.0)
        target_pos = self._target_position(theta)
        target_orn = self.orn0 if self.use_initial_orientation else None

        q_full = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            restPoses=self.q_seed,
        )
        q_des = list(q_full[: self.n])

        if self.enable_joint_step_limit and self.q_seed is not None:
            for joint_index in range(self.n):
                delta = q_des[joint_index] - self.q_seed[joint_index]
                if delta > self.max_joint_step_rad:
                    q_des[joint_index] = self.q_seed[joint_index] + self.max_joint_step_rad
                elif delta < -self.max_joint_step_rad:
                    q_des[joint_index] = self.q_seed[joint_index] - self.max_joint_step_rad

        self.q_seed = q_des[:]
        msg = Float64MultiArray()
        msg.data = q_des
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IiwaCircleIKDesired()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
