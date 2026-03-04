#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import pybullet as p
import pybullet_data


class IiwaImpedanceController(Node):
    def __init__(self):
        super().__init__('iiwa_impedance_controller')

        self.n = 7

        self.declare_parameter('k', 2000.0)
        self.declare_parameter('d', 90.0)
        self.declare_parameter('tau_lim', 500.0)
        self.declare_parameter('ctrl_hz', 200.0)

        self.declare_parameter('ma_window', 50)          # moving average window length
        self.declare_parameter('max_delta_tau', 1.0)     # max change per cycle (N*m per tick)

        # -------- gravity compensation params --------
        self.declare_parameter('use_gravity_comp', True)
        self.declare_parameter('gravity_comp_scale', 1.0)     # 1.0 = full g(q), 0.0 = off
        self.declare_parameter('pb_urdf', "kuka_iiwa/model.urdf")
        self.declare_parameter('pb_use_fixed_base', True)

        k = float(self.get_parameter('k').value)
        d = float(self.get_parameter('d').value)
        tau_lim = float(self.get_parameter('tau_lim').value)
        self.ctrl_hz = float(self.get_parameter('ctrl_hz').value)

        self.ma_window = int(self.get_parameter('ma_window').value)
        if self.ma_window < 1:
            self.ma_window = 1
        self.max_delta_tau = float(self.get_parameter('max_delta_tau').value)

        self.K = [k] * self.n
        self.D = [d] * self.n
        self.tau_lim = [tau_lim] * self.n

        # gravity comp
        self.use_gravity_comp = bool(self.get_parameter('use_gravity_comp').value)
        self.gravity_comp_scale = float(self.get_parameter('gravity_comp_scale').value)
        self.pb_urdf = str(self.get_parameter('pb_urdf').value)
        self.pb_use_fixed_base = bool(self.get_parameter('pb_use_fixed_base').value)

        # ---- ROS I/O ----
        self.sub_js = self.create_subscription(JointState, '/iiwa7/joint_states', self.on_js, 10)
        self.sub_qd = self.create_subscription(Float64MultiArray, '/iiwa7/joint_desired', self.on_q_des, 10)
        self.pub_tau = self.create_publisher(Float64MultiArray, '/iiwa7/joint_torques', 10)

        self.q = None
        self.qdot = None
        self.q_des = [0.0] * self.n
        self.qdot_des = [0.0] * self.n

        self.tau_buf = [deque(maxlen=self.ma_window) for _ in range(self.n)]
        self.tau_last_pub = [0.0] * self.n
        self.have_tau_last = False

        # ---- PyBullet (for inverse dynamics / gravity) ----
        self._pb_ready = False
        self._pb_client = None
        self._pb_robot = None
        if self.use_gravity_comp and abs(self.gravity_comp_scale) > 0.0:
            try:
                self._pb_client = p.connect(p.DIRECT)
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                # IMPORTANT: keep gravity consistent with sim
                p.setGravity(0, 0, -9.8)
                self._pb_robot = p.loadURDF(self.pb_urdf, useFixedBase=self.pb_use_fixed_base)
                self._pb_ready = True
            except Exception as e:
                self.get_logger().error(f"[GRAV] PyBullet init failed, gravity comp disabled. err={e}")
                self._pb_ready = False
                self.use_gravity_comp = False

        self.timer = self.create_timer(1.0 / self.ctrl_hz, self.control)

        self.get_logger().info("Iiwa impedance controller started.")
        self.get_logger().info(f"[PARAM] k={k}, d={d}, tau_lim={tau_lim}, ctrl_hz={self.ctrl_hz}")
        self.get_logger().info(f"[FILTER] ma_window={self.ma_window}, max_delta_tau={self.max_delta_tau} (per tick)")
        self.get_logger().info(f"[GRAV] use_gravity_comp={self.use_gravity_comp}, scale={self.gravity_comp_scale}, pb_ready={self._pb_ready}")

    def destroy_node(self):
        # clean up pybullet connection
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

        if self.q is not None and (not hasattr(self, "_init_locked")):
            self.q_des = self.q.copy()
            self._init_locked = True

    def on_q_des(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            self.q_des = list(msg.data[:self.n])

    def _clip(self, x: float, lim: float) -> float:
        if x > lim:
            return lim
        if x < -lim:
            return -lim
        return x

    def _moving_average(self, i: int, x: float) -> float:
        """Update joint i buffer and return its moving average."""
        b = self.tau_buf[i]
        b.append(float(x))
        return sum(b) / float(len(b))

    def _rate_limit(self, tau_target, tau_prev):
        """Limit |tau_target[i] - tau_prev[i]| <= max_delta_tau per tick."""
        out = [0.0] * self.n
        md = abs(self.max_delta_tau)
        for i in range(self.n):
            if md <= 0.0:
                out[i] = float(tau_target[i])
                continue
            dt = float(tau_target[i] - tau_prev[i])
            if dt > md:
                out[i] = float(tau_prev[i] + md)
            elif dt < -md:
                out[i] = float(tau_prev[i] - md)
            else:
                out[i] = float(tau_target[i])
        return out

    def _gravity_comp(self, q):
        """
        Return g(q) using PyBullet inverse dynamics with qdot=0, qddot=0.
        If pb not ready, return zeros.
        """
        if (not self.use_gravity_comp) or (not self._pb_ready) or abs(self.gravity_comp_scale) <= 0.0:
            return [0.0] * self.n

        # PyBullet expects full joint vector for the loaded robot.
        # For kuka_iiwa/model.urdf from pybullet_data, first 7 joints are the arm.
        q_in = [float(x) for x in q[:self.n]]
        qd0 = [0.0] * self.n
        qdd0 = [0.0] * self.n

        try:
            tau = p.calculateInverseDynamics(self._pb_robot, q_in, qd0, qdd0)
            # tau may be tuple length n
            tau = [float(x) for x in list(tau)[:self.n]]
            s = float(self.gravity_comp_scale)
            return [s * x for x in tau]
        except Exception as e:
            # fail-safe: disable grav comp if repeatedly failing
            self.get_logger().warning(f"[GRAV] calculateInverseDynamics failed, output 0. err={e}")
            return [0.0] * self.n

    def control(self):
        if self.q is None or self.qdot is None:
            return

        # PD impedance torque
        tau_pd = [0.0] * self.n
        for i in range(self.n):
            e = self.q_des[i] - self.q[i]
            # ed = self.qdot_des[i] - self.qdot[i]
            ed = - self.qdot[i]
            t = self.K[i] * e + self.D[i] * ed
            # 这里先不 clip 到 tau_lim，先和重力项合成后再整体限幅更合理
            tau_pd[i] = float(t)

        # gravity compensation
        tau_g = self._gravity_comp(self.q)

        # combine then clip (per-joint)
        tau_raw = [0.0] * self.n
        for i in range(self.n):
            t = float(tau_pd[i] + tau_g[i])
            t = self._clip(t, abs(self.tau_lim[i]))
            tau_raw[i] = t

        # moving average
        tau_ma = [0.0] * self.n
        for i in range(self.n):
            tau_ma[i] = self._moving_average(i, tau_raw[i])

        # rate limit
        if not self.have_tau_last:
            tau_out = [float(x) for x in tau_ma]
            self.have_tau_last = True
        else:
            tau_out = self._rate_limit(tau_ma, self.tau_last_pub)

        self.tau_last_pub = list(tau_out)
        # self.tau_last_pub = list(tau_g)

        out = Float64MultiArray()
        out.data = tau_out
        # out.data = tau_g
        self.pub_tau.publish(out)


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


if __name__ == '__main__':
    main()
