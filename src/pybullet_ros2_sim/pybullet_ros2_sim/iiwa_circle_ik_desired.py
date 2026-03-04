#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import pybullet as p
import pybullet_data


class IiwaCircleIKDesired(Node):
    def __init__(self):
        super().__init__('iiwa_circle_ik_desired')

        self.n = 7
        self.pub = self.create_publisher(Float64MultiArray, '/iiwa7/joint_desired', 10)

        # ---------- Trajectory params ----------
        self.declare_parameter('publish_hz', 200.0)
        self.declare_parameter('motion_hz', 0.1)       # 10s per circle
        self.declare_parameter('radius', 0.10)         # meters
        self.declare_parameter('phase_rad', 0.0)       # φ, rad
        self.declare_parameter('use_initial_orientation', True)

        # IK continuity protection
        self.declare_parameter('enable_joint_step_limit', True)
        self.declare_parameter('max_joint_step_rad', 0.02)

        # Robust init lock (recommended)
        self.declare_parameter('enable_velocity_lock', True)
        self.declare_parameter('lock_vel_eps', 0.02)   # rad/s
        self.declare_parameter('lock_count', 30)       # consecutive frames

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.motion_hz = float(self.get_parameter('motion_hz').value)
        self.radius = float(self.get_parameter('radius').value)
        self.phase = float(self.get_parameter('phase_rad').value)
        self.use_initial_orientation = bool(self.get_parameter('use_initial_orientation').value)

        self.enable_joint_step_limit = bool(self.get_parameter('enable_joint_step_limit').value)
        self.max_joint_step_rad = float(self.get_parameter('max_joint_step_rad').value)

        self.enable_velocity_lock = bool(self.get_parameter('enable_velocity_lock').value)
        self.lock_vel_eps = float(self.get_parameter('lock_vel_eps').value)
        self.lock_count = int(self.get_parameter('lock_count').value)

        # ---------- PyBullet IK client (DIRECT) ----------
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        self.ee_link = self.n - 1

        # ---------- Init sync ----------
        self.init_done = False
        self.init_done_rx_time = None

        self.have_init = False
        self.q0 = None
        self.q_seed = None
        self.p0 = None
        self.orn0 = None

        # For stable lock
        self._stable_count = 0

        # Timing: ensure first published sample is exactly t=0
        self._traj_started = False
        self.t0 = None

        # latched init_done
        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.sub_init_done = self.create_subscription(
            Bool, '/iiwa7/init_done', self.cb_init_done, qos_latch
        )

        # joint states from sim
        self.sub_js = self.create_subscription(
            JointState, '/iiwa7/joint_states', self.cb_joint_states, 10
        )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.on_timer)

        self.get_logger().info("Iiwa circle IK desired node started.")
        self.get_logger().info(
            f"[PARAM] publish_hz={self.publish_hz}, motion_hz={self.motion_hz}, radius={self.radius}, phase_rad={self.phase}"
        )
        self.get_logger().info("Waiting for /iiwa7/init_done and stable /iiwa7/joint_states to lock initial pose...")

    def cb_init_done(self, msg: Bool):
        self.init_done = bool(msg.data)
        if self.init_done:
            self.init_done_rx_time = self.get_clock().now()
        else:
            self.init_done_rx_time = None

    def cb_joint_states(self, msg: JointState):
        # Must wait for init_done
        if (not self.init_done) or self.have_init:
            return
        if not msg.position or len(msg.position) < self.n:
            return

        # Optional: require stable velocities for lock
        if self.enable_velocity_lock:
            if (not msg.velocity) or (len(msg.velocity) < self.n):
                # If no velocity provided, do not lock yet (safer)
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
        p0 = link_state[4]   # worldLinkFramePosition
        orn0 = link_state[5] # worldLinkFrameOrientation

        self.q0 = q0
        self.q_seed = q0[:]
        self.p0 = p0
        self.orn0 = orn0

        self.have_init = True
        self._traj_started = False
        self.t0 = None

        self.get_logger().info(f"[INIT] Locked q0={['%.3f'%x for x in self.q0]}")
        self.get_logger().info(f"[INIT] EE p0=({p0[0]:.3f},{p0[1]:.3f},{p0[2]:.3f})")
        self.get_logger().info("[INIT] Trajectory will start with target == p0 at t=0.")

    def on_timer(self):
        if not self.have_init:
            return

        # Start clock at first publish to guarantee t=0 at first message
        now = self.get_clock().now()
        if not self._traj_started:
            self.t0 = now
            self._traj_started = True
            t = 0.0
        else:
            t = (now - self.t0).nanoseconds * 1e-9

        w = 2.0 * math.pi * self.motion_hz

        # Key fix: ensure (dx,dy) = (0,0) at t=0
        # dx = r*(cos(wt+φ)-cosφ), dy = r*(sin(wt+φ)-sinφ)
        wt = w * t + self.phase
        dx = self.radius * (math.cos(wt) - math.cos(self.phase))
        dy = self.radius * (math.sin(wt) - math.sin(self.phase))
        dz = 0.0

        x = self.p0[0] + dx
        y = self.p0[1] + dy
        z = self.p0[2] + dz

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
    node = IiwaCircleIKDesired()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
