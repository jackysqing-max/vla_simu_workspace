#!/usr/bin/env bash
set -e

WS=~/ros2_workspaces/humble/ros2_pybullet_ws
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"

mkdir -p "$LOG_DIR" "$PID_DIR"

# 关键：source ROS 前避免 set -u 触发 unbound variable
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u 2>/dev/null || true

# 并行启动（最少逻辑，不做花哨等待）
ros2 run pybullet_ros2_sim iiwa_pybullet_sim_node \
  >"$LOG_DIR/sim.log" 2>&1 &  echo $! >"$PID_DIR/sim.pid"

ros2 run pybullet_ros2_sim iiwa_line_ik_desired \
  >"$LOG_DIR/desired.log" 2>&1 &  echo $! >"$PID_DIR/desired.pid"

ros2 run pybullet_ros2_sim iiwa_impedance_controller \
  >"$LOG_DIR/impedance.log" 2>&1 &  echo $! >"$PID_DIR/impedance.pid"

ros2 run iiwa_state_udp_bridge robotstate_bridge \
  >"$LOG_DIR/bridge.log" 2>&1 &  echo $! >"$PID_DIR/bridge.pid"

ros2 run robot_monitor robot_monitor \
  >"$LOG_DIR/monitor.log" 2>&1 &  echo $! >"$PID_DIR/monitor.pid"

echo "Started: sim desired impedance bridge monitor"
echo "Logs:   $LOG_DIR"
echo "PIDs:   $PID_DIR"

# 前台等待，Ctrl+C 时一并退出
wait
