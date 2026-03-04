#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import pybullet as p
import pybullet_data


def tri01(phase: float) -> float:
    """
    Triangle wave in [0,1]: phase in [0,1) -> 0..1..0
    """
    phase = phase - math.floor(phase)
    return 1.0 - abs(2.0 * phase - 1.0)


class IiwaLineIKDesired(Node):
    def __init__(self):
        super().__init__('iiwa_line_ik_desired')

        self.n = 7
        self.pub = self.create_publisher(Float64MultiArray, '/iiwa7/joint_desired', 10)

        # ---------- Params ----------
        self.declare_parameter('publish_hz', 200.0)
        self.declare_parameter('speed_mps', 0.05)     # along the line, meters/sec
        self.declare_parameter('dx', 0.10)            # line direction offset in x (meters)
        self.declare_parameter('dy', 0.00)            # meters
        self.declare_parameter('dz', 0.00)            # meters
        self.declare_parameter('use_initial_orientation', True)

        self.declare_parameter('enable_joint_step_limit', True)
        self.declare_parameter('max_joint_step_rad', 0.02)

        self.declare_parameter('enable_velocity_lock', True)
        self.declare_parameter('lock_vel_eps', 0.02)  # rad/s
        self.declare_parameter('lock_count', 30)      # consecutive frames

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.speed_mps = float(self.get_parameter('speed_mps').value)
        self.dx = float(self.get_parameter('dx').value)
        self.dy = float(self.get_parameter('dy').value)
        self.dz = float(self.get_parameter('dz').value)
        self.use_initial_orientation = bool(self.get_parameter('use_initial_orientation').value)

        self.enable_joint_step_limit = bool(self.get_parameter('enable_joint_step_limit').value)
        self.max_joint_step_rad = float(self.get_parameter('max_joint_step_rad').value)

        self.enable_velocity_lock = bool(self.get_parameter('enable_velocity_lock').value)
        self.lock_vel_eps = float(self.get_parameter('lock_vel_eps').value)
        self.lock_count = int(self.get_parameter('lock_count').value)

        # Derived
        self.L = math.sqrt(self.dx*self.dx + self.dy*self.dy + self.dz*self.dz)
        if self.L < 1e-9:
            raise RuntimeError("Line length is zero. Set dx/dy/dz to a non-zero vector.")
        self.period = 2.0 * self.L / max(self.speed_mps, 1e-6)  # p0->p1->p0

        # ---------- PyBullet IK (DIRECT) ----------
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        self.ee_link = self.n - 1

        # ---------- Init sync ----------
        self.init_done = False
        self.have_init = False

        self.q0 = None
        self.q_seed = None
        self.p0 = None
        self.p1 = None
        self.orn0 = None

        self._stable_count = 0
        self._traj_started = False
        self.t0 = None

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.sub_init_done = self.create_subscription(
            Bool, '/iiwa7/init_done', self.cb_init_done, qos_latch
        )
        self.sub_js = self.create_subscription(
            JointState, '/iiwa7/joint_states', self.cb_joint_states, 10
        )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info("Iiwa line IK desired node started.")
        self.get_logger().info(
            f"[PARAM] publish_hz={self.publish_hz}, speed_mps={self.speed_mps}, "
            f"dir=(dx,dy,dz)=({self.dx},{self.dy},{self.dz}), L={self.L:.3f}, period={self.period:.3f}s"
        )
        self.get_logger().info("Waiting for /iiwa7/init_done and stable /iiwa7/joint_states to lock initial pose...")

    def cb_init_done(self, msg: Bool):
        self.init_done = bool(msg.data)

    def cb_joint_states(self, msg: JointState):
        if (not self.init_done) or self.have_init:
            return
        if not msg.position or len(msg.position) < self.n:
            return

        if self.enable_velocity_lock:
            if (not msg.velocity) or (len(msg.velocity) < self.n):
                return
            vmax = max(abs(v) for v in msg.velocity[:self.n])
            if vmax < self.lock_vel_eps:
                self._stable_count += 1
            else:
                self._stable_count = 0
            if self._stable_count < self.lock_count:
                return

        q0 = list(msg.position[:self.n])
        for j in range(self.n):
            p.resetJointState(self.robot_id, j, q0[j])

        link_state = p.getLinkState(self.robot_id, self.ee_link, computeForwardKinematics=True)
        p0 = link_state[4]
        orn0 = link_state[5]

        self.q0 = q0
        self.q_seed = q0[:]
        self.p0 = p0
        self.orn0 = orn0

        # endpoint p1 = p0 + d
        self.p1 = (p0[0] + self.dx, p0[1] + self.dy, p0[2] + self.dz)

        self.have_init = True
        self._traj_started = False
        self.t0 = None

        self.get_logger().info(f"[INIT] Locked q0={['%.3f'%x for x in self.q0]}")
        self.get_logger().info(f"[INIT] EE p0=({p0[0]:.3f},{p0[1]:.3f},{p0[2]:.3f})")
        self.get_logger().info(f"[INIT] Line p1=({self.p1[0]:.3f},{self.p1[1]:.3f},{self.p1[2]:.3f})")
        self.get_logger().info("[INIT] Trajectory starts with target == p0 at t=0.")

    def on_timer(self):
        if not self.have_init:
            return

        now = self.get_clock().now()
        if not self._traj_started:
            self.t0 = now
            self._traj_started = True
            t = 0.0
        else:
            t = (now - self.t0).nanoseconds * 1e-9

        # s in [0,1], triangle wave -> p0->p1->p0
        phase = (t / self.period) if self.period > 1e-9 else 0.0
        s = tri01(phase)

        x = self.p0[0] + s * self.dx
        y = self.p0[1] + s * self.dy
        z = self.p0[2] + s * self.dz

        target_orn = self.orn0 if self.use_initial_orientation else p.getQuaternionFromEuler([math.pi, 0.0, 0.0])

        q_full = p.calculateInverseKinematics(
            self.robot_id,
            self.ee_link,
            targetPosition=[x, y, z],
            targetOrientation=target_orn,
            restPoses=self.q_seed
        )
        q_des = list(q_full[:self.n])

        if self.enable_joint_step_limit and self.q_seed is not None:
            for i in range(self.n):
                dq = q_des[i] - self.q_seed[i]
                if dq > self.max_joint_step_rad:
                    q_des[i] = self.q_seed[i] + self.max_joint_step_rad
                elif dq < -self.max_joint_step_rad:
                    q_des[i] = self.q_seed[i] - self.max_joint_step_rad

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
    node = IiwaLineIKDesired()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
