#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
CIRCLE_RADIUS_M="${CIRCLE_RADIUS_M:-0.035}"
CIRCLE_SPEED_MPS="${CIRCLE_SPEED_MPS:-0.020}"
CIRCLE_PLANE="${CIRCLE_PLANE:-xy}"
CIRCLE_CONTROL_MODE="${CIRCLE_CONTROL_MODE:-position}"
IIWA_GUI="${IIWA_GUI:-true}"
cd "$WS"
mkdir -p "$LOG_DIR" "$PID_DIR"

start_bg() {
  local name="$1"
  shift
  local log="$LOG_DIR/$name.log"
  local pidf="$PID_DIR/$name.pid"
  setsid bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '$WS/install/setup.bash'
    set -u 2>/dev/null || true
    exec $*
  " >"$log" 2>&1 &
  echo "$!" > "$pidf"
  echo "[START] $name pid=$(cat "$pidf") log=$log"
}

# OBS target: PyBullet GUI, optionally RobotMonitor. Shows a conservative
# Cartesian end-effector circle on the same /iiwa7/joint_desired interface.
# Default to position mode for a clean trajectory recording. Set
# CIRCLE_CONTROL_MODE=torque if you specifically want the impedance controller
# path instead.
start_bg sim "ros2 run pybullet_ros2_sim iiwa_pybullet_sim_node --ros-args -r __node:=iiwa_pybullet_sim_node -p gui:=$IIWA_GUI"
start_bg desired "python3 docs/recording_scripts/iiwa_circle_ik_desired_recording.py --ros-args -p radius_m:=$CIRCLE_RADIUS_M -p speed_mps:=$CIRCLE_SPEED_MPS -p plane:=$CIRCLE_PLANE -p enable_velocity_lock:=false -p allow_lock_without_init_done:=true -p startup_delay_sec:=2.8"
if [[ "$CIRCLE_CONTROL_MODE" == "torque" ]]; then
  start_bg imp "ros2 run pybullet_ros2_sim iiwa_impedance_controller --ros-args -r __node:=iiwa_impedance_controller"
else
  start_bg mode_switch "python3 docs/recording_scripts/set_iiwa_control_mode.py --delay-sec 3.0 --mode position"
fi
start_bg bridge "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p port:=7755 -p rate_hz:=100.0"
start_bg monitor "ros2 run robot_monitor robot_monitor"

echo "[OK] circle trajectory recording stack started. mode=$CIRCLE_CONTROL_MODE radius=$CIRCLE_RADIUS_M plane=$CIRCLE_PLANE logs=$LOG_DIR"
