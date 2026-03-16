#!/usr/bin/env python3
"""Joint-space impedance controller for the main iiwa torque-control chain."""

from collections import deque
from typing import List

import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class IiwaImpedanceController(Node):
    """Compute torque commands from joint state and joint-space targets.

    The control law is intentionally simple and well-bounded:
    1. PD impedance in joint space
    2. optional gravity compensation from a dedicated DIRECT PyBullet model
    3. moving-average filtering
    4. per-tick rate limiting
    """

    def __init__(self):
        super().__init__("iiwa_impedance_controller")

        self.n = 7

        self.declare_parameter("k", 2000.0)
        self.declare_parameter("d", 90.0)
        self.declare_parameter("tau_lim", 500.0)
        self.declare_parameter("ctrl_hz", 200.0)

        self.declare_parameter("ma_window", 50)
        self.declare_parameter("max_delta_tau", 1.0)

        self.declare_parameter("use_gravity_comp", True)
        self.declare_parameter("gravity_comp_scale", 1.0)
        self.declare_parameter("pb_urdf", "kuka_iiwa/model.urdf")
        self.declare_parameter("pb_use_fixed_base", True)

        k = float(self.get_parameter("k").value)
        d = float(self.get_parameter("d").value)
        tau_lim = float(self.get_parameter("tau_lim").value)
        self.ctrl_hz = float(self.get_parameter("ctrl_hz").value)

        self.ma_window = max(1, int(self.get_parameter("ma_window").value))
        self.max_delta_tau = float(self.get_parameter("max_delta_tau").value)

        self.K = [k] * self.n
        self.D = [d] * self.n
        self.tau_lim = [tau_lim] * self.n

        self.use_gravity_comp = bool(self.get_parameter("use_gravity_comp").value)
        self.gravity_comp_scale = float(self.get_parameter("gravity_comp_scale").value)
        self.pb_urdf = str(self.get_parameter("pb_urdf").value)
        self.pb_use_fixed_base = bool(self.get_parameter("pb_use_fixed_base").value)

        self.sub_js = self.create_subscription(
            JointState,
            "/iiwa7/joint_states",
            self.on_js,
            10,
        )
        self.sub_qd = self.create_subscription(
            Float64MultiArray,
            "/iiwa7/joint_desired",
            self.on_q_des,
            10,
        )
        self.pub_tau = self.create_publisher(
            Float64MultiArray,
            "/iiwa7/joint_torques",
            10,
        )

        self.q = None
        self.qdot = None
        self.q_des = [0.0] * self.n
        self._init_locked = False

        self.tau_buf = [deque(maxlen=self.ma_window) for _ in range(self.n)]
        self.tau_last_pub = [0.0] * self.n
        self.have_tau_last = False

        self._pb_ready = False
        self._pb_client = None
        self._pb_robot = None
        if self.use_gravity_comp and abs(self.gravity_comp_scale) > 0.0:
            try:
                self._pb_client = p.connect(p.DIRECT)
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                p.setGravity(0.0, 0.0, -9.8, physicsClientId=self._pb_client)
                self._pb_robot = p.loadURDF(
                    self.pb_urdf,
                    useFixedBase=self.pb_use_fixed_base,
                    physicsClientId=self._pb_client,
                )
                self._pb_ready = True
            except Exception as exc:
                self.get_logger().error(
                    f"[GRAV] PyBullet init failed, gravity compensation disabled: {exc}"
                )
                self._pb_ready = False
                self.use_gravity_comp = False

        self.timer = self.create_timer(1.0 / self.ctrl_hz, self.control)

        self.get_logger().info("Iiwa impedance controller started.")
        self.get_logger().info(
            f"[PARAM] k={k}, d={d}, tau_lim={tau_lim}, ctrl_hz={self.ctrl_hz}"
        )
        self.get_logger().info(
            f"[FILTER] ma_window={self.ma_window}, max_delta_tau={self.max_delta_tau}"
        )
        self.get_logger().info(
            "[GRAV] "
            f"use_gravity_comp={self.use_gravity_comp}, "
            f"scale={self.gravity_comp_scale}, "
            f"pb_ready={self._pb_ready}"
        )

    def destroy_node(self):
        try:
            if self._pb_client is not None:
                p.disconnect(self._pb_client)
        except Exception:
            pass
        super().destroy_node()

    def on_js(self, msg: JointState):
        if len(msg.position) >= self.n:
            self.q = list(msg.position[:self.n])
        if len(msg.velocity) >= self.n:
            self.qdot = list(msg.velocity[:self.n])

        # Lock the initial target to the measured pose so the controller does
        # not kick the arm before an explicit desired command arrives.
        if self.q is not None and not self._init_locked:
            self.q_des = self.q.copy()
            self._init_locked = True

    def on_q_des(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            self.q_des = list(msg.data[:self.n])

    def _clip(self, value: float, limit: float) -> float:
        if value > limit:
            return limit
        if value < -limit:
            return -limit
        return value

    def _moving_average(self, joint_index: int, value: float) -> float:
        buf = self.tau_buf[joint_index]
        buf.append(float(value))
        return sum(buf) / float(len(buf))

    def _rate_limit(self, target: List[float], previous: List[float]) -> List[float]:
        out = [0.0] * self.n
        max_delta = abs(self.max_delta_tau)
        for joint_index in range(self.n):
            if max_delta <= 0.0:
                out[joint_index] = float(target[joint_index])
                continue

            delta = float(target[joint_index] - previous[joint_index])
            if delta > max_delta:
                out[joint_index] = float(previous[joint_index] + max_delta)
            elif delta < -max_delta:
                out[joint_index] = float(previous[joint_index] - max_delta)
            else:
                out[joint_index] = float(target[joint_index])
        return out

    def _gravity_comp(self, q: List[float]) -> List[float]:
        """Return the gravity term g(q) from the side PyBullet model."""
        if (
            (not self.use_gravity_comp)
            or (not self._pb_ready)
            or abs(self.gravity_comp_scale) <= 0.0
        ):
            return [0.0] * self.n

        qd_zero = [0.0] * self.n
        qdd_zero = [0.0] * self.n

        try:
            tau = p.calculateInverseDynamics(
                self._pb_robot,
                [float(value) for value in q[:self.n]],
                qd_zero,
                qdd_zero,
                physicsClientId=self._pb_client,
            )
            scale = float(self.gravity_comp_scale)
            return [scale * float(value) for value in list(tau)[:self.n]]
        except Exception as exc:
            self.get_logger().warning(
                f"[GRAV] calculateInverseDynamics failed, output zeros: {exc}"
            )
            return [0.0] * self.n

    def control(self):
        if self.q is None or self.qdot is None:
            return

        tau_pd = [0.0] * self.n
        for joint_index in range(self.n):
            error = self.q_des[joint_index] - self.q[joint_index]
            error_rate = -self.qdot[joint_index]
            tau_pd[joint_index] = (
                self.K[joint_index] * error + self.D[joint_index] * error_rate
            )

        tau_g = self._gravity_comp(self.q)

        tau_raw = [0.0] * self.n
        for joint_index in range(self.n):
            tau_raw[joint_index] = self._clip(
                float(tau_pd[joint_index] + tau_g[joint_index]),
                abs(self.tau_lim[joint_index]),
            )

        tau_ma = [
            self._moving_average(joint_index, tau_raw[joint_index])
            for joint_index in range(self.n)
        ]

        if not self.have_tau_last:
            tau_out = [float(value) for value in tau_ma]
            self.have_tau_last = True
        else:
            tau_out = self._rate_limit(tau_ma, self.tau_last_pub)

        self.tau_last_pub = list(tau_out)

        msg = Float64MultiArray()
        msg.data = tau_out
        self.pub_tau.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IiwaImpedanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
