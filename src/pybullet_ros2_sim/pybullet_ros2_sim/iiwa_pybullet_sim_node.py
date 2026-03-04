#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool, Int8
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import pybullet as p
import pybullet_data


def min_jerk_s(u: float) -> float:
    """u in [0,1] -> s in [0,1], 0/1 处速度加速度为 0，极大抑制振荡"""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return 10*u**3 - 15*u**4 + 6*u**5


class IiwaPybulletSim(Node):
    MODE_FREE = 0
    MODE_POSITION = 1
    MODE_TORQUE = 2

    def __init__(self):
        super().__init__('iiwa_pybullet_sim_node')

        self.n = 7
        self.joint_indices = list(range(self.n))

        self._lock = threading.Lock()
        self.cb_sub = ReentrantCallbackGroup()
        self.cb_timer = ReentrantCallbackGroup()

        # -------- ROS pub/sub --------
        self.pub_js = self.create_publisher(JointState, '/iiwa7/joint_states', 10)

        qos_latch = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.pub_init_done = self.create_publisher(Bool, '/iiwa7/init_done', qos_latch)

        # 同时接收位置与力矩命令（都缓存起来）
        self.sub_qdes = self.create_subscription(
            Float64MultiArray,
            '/iiwa7/joint_desired',
            self.on_q_des,
            10,
            callback_group=self.cb_sub
        )
        self.sub_tau = self.create_subscription(
            Float64MultiArray,
            '/iiwa7/joint_torques',
            self.on_tau,
            10,
            callback_group=self.cb_sub
        )

        # 推荐的模式切换 topic（0=FREE, 1=POS, 2=TORQUE）
        self.sub_mode = self.create_subscription(
            Int8,
            '/iiwa7/control_mode',
            self.on_control_mode,
            10,
            callback_group=self.cb_sub
        )

        # 兼容你原来的 enable_position：True->POSITION，False->FREE
        self.sub_enable_position = self.create_subscription(
            Bool,
            '/iiwa7/enable_position',
            self.on_enable_position_compat,
            10,
            callback_group=self.cb_sub
        )

        # -------- Commands cache --------
        self.q_cmd = [0.0] * self.n
        self.tau_cmd = [0.0] * self.n
        self.have_des = False
        self.have_tau = False

        self._warned_no_tau = False

        # -------- Parameters --------
        self.declare_parameter('use_goto', True)
        self.declare_parameter('init_q', [0.0, 0.6, 0.0, -1.2, 0.0, 1.0, 0.0])

        self.declare_parameter('goto_duration', 2.0)
        self.declare_parameter('goto_use_reset', False)

        self.declare_parameter('goto_force', 120.0)
        self.declare_parameter('goto_pos_gain', 0.20)
        self.declare_parameter('goto_vel_gain', 1.00)
        self.declare_parameter('goto_max_vel', 1.0)

        self.declare_parameter('track_force', 120.0)
        self.declare_parameter('track_pos_gain', 0.25)
        self.declare_parameter('track_vel_gain', 1.00)
        self.declare_parameter('track_max_vel', 2.0)

        self.declare_parameter('sim_hz', 200.0)

        # 力矩限幅（建议保留，避免数值爆掉）
        self.declare_parameter('tau_limit', 200.0)

        self.use_goto = bool(self.get_parameter('use_goto').value)

        self.init_q = [float(x) for x in self.get_parameter('init_q').value]
        if len(self.init_q) < self.n:
            self.init_q += [0.0] * (self.n - len(self.init_q))
        self.init_q = self.init_q[:self.n]

        self.goto_duration = float(self.get_parameter('goto_duration').value)
        self.goto_use_reset = bool(self.get_parameter('goto_use_reset').value)

        self.goto_force = float(self.get_parameter('goto_force').value)
        self.goto_pos_gain = float(self.get_parameter('goto_pos_gain').value)
        self.goto_vel_gain = float(self.get_parameter('goto_vel_gain').value)
        self.goto_max_vel = float(self.get_parameter('goto_max_vel').value)

        self.track_force = float(self.get_parameter('track_force').value)
        self.track_pos_gain = float(self.get_parameter('track_pos_gain').value)
        self.track_vel_gain = float(self.get_parameter('track_vel_gain').value)
        self.track_max_vel = float(self.get_parameter('track_max_vel').value)

        sim_hz = float(self.get_parameter('sim_hz').value)
        self.dt = 1.0 / max(sim_hz, 1e-6)

        self.tau_limit = float(self.get_parameter('tau_limit').value)

        # -------- PyBullet init (use dt) --------
        self.client = p.connect(p.GUI)  # no display -> p.DIRECT
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setRealTimeSimulation(0)
        p.setTimeStep(self.dt)
        p.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=1,
            numSolverIterations=200
        )

        p.setGravity(0, 0, -9.8)
        # p.setGravity(0, 0, 0)
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

        # q_start
        self.q_start = [float(p.getJointState(self.robot_id, j)[0]) for j in self.joint_indices]

        # GOTO / RUN
        self.phase = "GOTO" if self.use_goto else "RUN"
        self.t_goto_start = self.get_clock().now().nanoseconds * 1e-9
        self._init_done_sent = False

        # 初始位置命令（即使未收到 /joint_desired，也能 hold）
        self.q_cmd = self.init_q[:]

        # -------- Control mode --------
        # 默认：RUN 阶段位置控制（你也可以改成 TORQUE 或 FREE）
        # self.control_mode = self.MODE_POSITION
        self.control_mode = self.MODE_TORQUE
        # self.control_mode = self.MODE_FREE
        self._last_mode = None
        self._apply_mode_change(None, self.control_mode)

        # -------- Timers --------
        self.timer = self.create_timer(self.dt, self.step, callback_group=self.cb_timer)
        self._dbg_timer = self.create_timer(1.0, self.debug_status, callback_group=self.cb_timer)

        self.get_logger().info("Iiwa PyBullet sim node started.")
        self.get_logger().info("Mode switch via /iiwa7/control_mode (std_msgs/Int8): 0=FREE, 1=POSITION, 2=TORQUE")
        self.get_logger().info("Examples:")
        self.get_logger().info("  ros2 topic pub --once /iiwa7/control_mode std_msgs/msg/Int8 '{data: 1}'")
        self.get_logger().info("  ros2 topic pub --once /iiwa7/control_mode std_msgs/msg/Int8 '{data: 2}'")
        self.get_logger().info("  ros2 topic pub --once /iiwa7/control_mode std_msgs/msg/Int8 '{data: 0}'")

    # ------------------- ROS callbacks -------------------
    def on_q_des(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            with self._lock:
                self.q_cmd = [float(x) for x in msg.data[:self.n]]
                self.have_des = True

    def on_tau(self, msg: Float64MultiArray):
        if msg.data and len(msg.data) >= self.n:
            with self._lock:
                self.tau_cmd = [float(x) for x in msg.data[:self.n]]
                self.have_tau = True

    def on_control_mode(self, msg: Int8):
        mode = int(msg.data)
        if mode not in (self.MODE_FREE, self.MODE_POSITION, self.MODE_TORQUE):
            self.get_logger().warning(f"[MODE] Invalid control_mode={mode}, ignore. Use 0/1/2.")
            return
        self.set_mode(mode)

    # 兼容旧接口：True->POSITION, False->FREE
    def on_enable_position_compat(self, msg: Bool):
        if bool(msg.data):
            self.set_mode(self.MODE_POSITION)
        else:
            # 兼容你原语义：disable position -> FREE
            if self.control_mode == self.MODE_POSITION:
                self.set_mode(self.MODE_FREE)

    # ------------------- Helpers -------------------
    def set_mode(self, new_mode: int):
        with self._lock:
            old_mode = self.control_mode
            if new_mode == old_mode:
                return
            self.control_mode = new_mode
        self._apply_mode_change(old_mode, new_mode)

    def _apply_mode_change(self, old_mode, new_mode):
        # 注意：pybullet 调用放在 timer 线程也行，但这里被订阅线程调用也可能发生。
        # 为绝对安全起见，你也可以把“模式变更”延迟到 step() 内处理。
        # 这里我们只做“释放电机”这种幂等操作，风险很低。
        if new_mode == self.MODE_TORQUE:
            self.release_motors()
            self.get_logger().info("[MODE] -> TORQUE_CONTROL")
        elif new_mode == self.MODE_POSITION:
            # POSITION 模式无需提前释放；step() 会每周期写 POSITION_CONTROL
            self.get_logger().info("[MODE] -> POSITION_CONTROL")
        else:
            self.release_motors()
            self.get_logger().info("[MODE] -> FREE (motors released)")

    def release_motors(self):
        # 禁用默认电机，避免和外部力矩/自由模式冲突
        for j in self.joint_indices:
            p.setJointMotorControl2(self.robot_id, j, controlMode=p.VELOCITY_CONTROL, force=0.0)

    def _get_q_qd_tau(self):
        q, qd, tau = [], [], []
        for j in self.joint_indices:
            st = p.getJointState(self.robot_id, j)
            q.append(float(st[0]))
            qd.append(float(st[1]))
            tau.append(float(st[3]) if st[3] is not None else 0.0)
        return q, qd, tau

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"iiwa_joint_{k+1}" for k in range(self.n)]

        q, qd, tau = self._get_q_qd_tau()
        js.position = q
        js.velocity = qd
        js.effort = tau
        self.pub_js.publish(js)

    def _send_init_done_once(self):
        if self._init_done_sent:
            return
        m = Bool()
        m.data = True
        self.pub_init_done.publish(m)
        self._init_done_sent = True
        self.get_logger().info("[INIT] Published latched /iiwa7/init_done = True")

    def debug_status(self):
        with self._lock:
            mode = int(self.control_mode)
            have_des = bool(self.have_des)
            have_tau = bool(self.have_tau)
            q0 = float(self.q_cmd[0])
            tau0 = float(self.tau_cmd[0])
        mode_str = {0: "FREE", 1: "POS", 2: "TAU"}.get(mode, str(mode))
        self.get_logger().info(
            f"[DBG] phase={self.phase}, mode={mode_str}, have_des={have_des}, have_tau={have_tau}, q0={q0:.3f}, tau0={tau0:.2f}"
        )

    # ------------------- Main loop -------------------
    def step(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        # --- GOTO 阶段：强制位置控制把机器人平滑带到 init_q ---
        if self.phase == "GOTO":
            u = (t - self.t_goto_start) / max(self.goto_duration, 1e-6)
            s = min_jerk_s(u)
            q_ref = [self.q_start[i] + s * (self.init_q[i] - self.q_start[i]) for i in range(self.n)]

            if self.goto_use_reset:
                self.release_motors()
                for i, j in enumerate(self.joint_indices):
                    p.resetJointState(self.robot_id, j, q_ref[i], targetVelocity=0.0)
            else:
                for i, j in enumerate(self.joint_indices):
                    p.setJointMotorControl2(
                        self.robot_id, j,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=float(q_ref[i]),
                        positionGain=float(self.goto_pos_gain),
                        velocityGain=float(self.goto_vel_gain),
                        force=float(self.goto_force),
                        maxVelocity=float(self.goto_max_vel),
                    )

            p.stepSimulation()
            self.publish_joint_states()

            if u >= 1.0:
                self.phase = "RUN"
                self._send_init_done_once()
                self.get_logger().info("[GOTO] done -> RUN")
            return

        # --- RUN 阶段：根据控制模式施加 ---
        with self._lock:
            mode = int(self.control_mode)
            q_target = list(self.q_cmd)
            tau_target = list(self.tau_cmd)
            have_tau = bool(self.have_tau)

        if mode == self.MODE_FREE:
            self.release_motors()
            p.stepSimulation()
            self.publish_joint_states()
            return

        if mode == self.MODE_POSITION:
            for i, j in enumerate(self.joint_indices):
                p.setJointMotorControl2(
                    self.robot_id, j,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=float(q_target[i]),
                    positionGain=float(self.track_pos_gain),
                    velocityGain=float(self.track_vel_gain),
                    force=float(self.track_force),
                    maxVelocity=float(self.track_max_vel),
                )
            p.stepSimulation()
            self.publish_joint_states()
            return

        # mode == TORQUE
        self.release_motors()

        if (not have_tau) and (not self._warned_no_tau):
            self.get_logger().warning("[TORQUE] No /iiwa7/joint_torques received yet. Applying 0 torques.")
            self._warned_no_tau = True

        # 力矩限幅
        lim = max(self.tau_limit, 0.0)
        for i, j in enumerate(self.joint_indices):
            tau = float(tau_target[i])
            if lim > 0.0:
                if tau > lim:
                    tau = lim
                elif tau < -lim:
                    tau = -lim
            p.setJointMotorControl2(
                self.robot_id, j,
                controlMode=p.TORQUE_CONTROL,
                force=tau
            )

        p.stepSimulation()
        self.publish_joint_states()


def main(args=None):
    rclpy.init(args=args)
    node = IiwaPybulletSim()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            p.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
