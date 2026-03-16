#!/usr/bin/env python3
"""Core iiwa execution node backed by a single PyBullet simulation."""

import threading
from typing import List, Tuple

import pybullet as p
import pybullet_data
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, Int8


def min_jerk_s(u: float) -> float:
    """Map progress in ``[0, 1]`` to a smooth min-jerk interpolation factor."""
    if u <= 0.0:
        return 0.0
    if u >= 1.0:
        return 1.0
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


class IiwaPybulletSim(Node):
    """Expose the main `/iiwa7/*` control contract on top of PyBullet.

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
        self.cb_timer = ReentrantCallbackGroup()

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

        self.pub_js = self.create_publisher(JointState, "/iiwa7/joint_states", 10)

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
            basePosition=[0.6, 0.0, -0.62],
            useFixedBase=True,
            physicsClientId=self.client,
        )
        self.robot_id = p.loadURDF(
            "kuka_iiwa/model.urdf",
            useFixedBase=True,
            physicsClientId=self.client,
        )

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

        # Default to torque mode so the impedance controller chain becomes the
        # primary runtime path. Position mode is still available for debugging.
        self.control_mode = self.MODE_TORQUE
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

    def _declare_parameters(self):
        self.declare_parameter("gui", True)
        self.declare_parameter("use_goto", True)
        self.declare_parameter("init_q", [0.0, 0.6, 0.0, -1.2, 0.0, 1.0, 0.0])

        self.declare_parameter("goto_duration", 2.0)
        self.declare_parameter("goto_use_reset", False)
        self.declare_parameter("goto_force", 120.0)
        self.declare_parameter("goto_pos_gain", 0.20)
        self.declare_parameter("goto_vel_gain", 1.00)
        self.declare_parameter("goto_max_vel", 1.0)

        self.declare_parameter("track_force", 120.0)
        self.declare_parameter("track_pos_gain", 0.25)
        self.declare_parameter("track_vel_gain", 1.00)
        self.declare_parameter("track_max_vel", 2.0)

        self.declare_parameter("sim_hz", 200.0)
        self.declare_parameter("tau_limit", 200.0)

    def _load_parameters(self):
        self.gui = bool(self.get_parameter("gui").value)
        self.use_goto = bool(self.get_parameter("use_goto").value)

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

        self.track_force = float(self.get_parameter("track_force").value)
        self.track_pos_gain = float(self.get_parameter("track_pos_gain").value)
        self.track_vel_gain = float(self.get_parameter("track_vel_gain").value)
        self.track_max_vel = float(self.get_parameter("track_max_vel").value)

        sim_hz = float(self.get_parameter("sim_hz").value)
        self.dt = 1.0 / max(sim_hz, 1e-6)
        self.tau_limit = float(self.get_parameter("tau_limit").value)

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
        self.publish_joint_states()

        if progress >= 1.0:
            self.phase = "RUN"
            self._send_init_done_once()
            self.get_logger().info("[GOTO] done -> RUN")

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

    def step(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.phase == "GOTO":
            self._step_goto(now_sec)
            return
        self._step_run()

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
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
