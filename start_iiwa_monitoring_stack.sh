#!/usr/bin/env bash
set -euo pipefail

# ===================== Config =====================
WS="${HOME}/ros2_workspaces/humble/ros2_pybullet_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS}/install/setup.bash"

# --- Nodes to start ---
SIM_PKG="pybullet_ros2_sim"
SIM_EXEC="iiwa_pybullet_sim_node"
SIM_NODE="iiwa_pybullet_sim_node"

# Trajectory planner (optional). Set ENABLE_TRAJ=0 to disable.
ENABLE_TRAJ="${ENABLE_TRAJ:-1}"
TRAJ_PKG="${TRAJ_PKG:-your_traj_planner_pkg}"
TRAJ_EXEC="${TRAJ_EXEC:-your_traj_planner_exe}"
TRAJ_NODE="${TRAJ_NODE:-traj_planner}"

# The trajectory topic you want to track (set this to your planner's topic)
# Common: /joint_trajectory or /<robot>/joint_trajectory
TRAJ_TOPIC="${TRAJ_TOPIC:-/joint_trajectory}"

# --- Robot state bridge -> publishes /robot_states for robot_monitor ---
BRIDGE_PKG="iiwa_state_udp_bridge"
BRIDGE_EXEC="robotstate_bridge"
BRIDGE_NODE="robotstate_bridge"
UDP_PORT="${UDP_PORT:-7755}"
BRIDGE_RATE_HZ="${BRIDGE_RATE_HZ:-100.0}"

# Robot monitor GUI (subscriber of /robot_states)
MON_PKG="robot_monitor"
MON_EXEC="robot_monitor"
MON_NODE="robot_monitor"

# ===================== Runtime =====================
RUN_DIR="${WS}/run"
LOG_DIR="${WS}/run_logs"
PID_DIR="${WS}/run_pids"
mkdir -p "${RUN_DIR}" "${LOG_DIR}" "${PID_DIR}"

SIM_LOG="${LOG_DIR}/sim.log"
TRAJ_LOG="${LOG_DIR}/traj_planner.log"
BRIDGE_LOG="${LOG_DIR}/bridge.log"
MON_LOG="${LOG_DIR}/monitor.log"
TRAJ_HZ_LOG="${LOG_DIR}/traj_hz.log"
TRAJ_ECHO_LOG="${LOG_DIR}/traj_echo.log"

SIM_PID="${PID_DIR}/sim.pid"
TRAJ_PID="${PID_DIR}/traj.pid"
BRIDGE_PID="${PID_DIR}/bridge.pid"
MON_PID="${PID_DIR}/monitor.pid"
TRAJ_HZ_PID="${PID_DIR}/traj_hz.pid"
TRAJ_ECHO_PID="${PID_DIR}/traj_echo.pid"

WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-12}"

info(){ echo "[INFO] $*"; }
warn(){ echo "[WARN] $*" >&2; }
err(){ echo "[ERROR] $*" >&2; }

have_cmd(){ command -v "$1" >/dev/null 2>&1; }
pid_running(){
  local f="$1"
  [ -f "$f" ] || return 1
  local p; p="$(cat "$f" 2>/dev/null || true)"
  [ -n "$p" ] || return 1
  kill -0 "$p" >/dev/null 2>&1
}
stop_pidfile(){
  local f="$1"
  [ -f "$f" ] || return 0
  local p; p="$(cat "$f" 2>/dev/null || true)"
  if [ -n "$p" ] && kill -0 "$p" >/dev/null 2>&1; then
    info "Stopping pid=$p"
    kill "$p" >/dev/null 2>&1 || true
    for _ in {1..40}; do
      kill -0 "$p" >/dev/null 2>&1 || break
      sleep 0.1
    done
    kill -9 "$p" >/dev/null 2>&1 || true
  fi
  rm -f "$f"
}
source_ros(){
  # avoid conda auto messages breaking strict mode
  set +u
  source "${ROS_SETUP}"
  if [ -f "${WS_SETUP}" ]; then
    source "${WS_SETUP}"
  else
    set -u
    err "Workspace overlay not found: ${WS_SETUP}"
    err "Build first: cd ${WS} && colcon build --symlink-install"
    exit 1
  fi
  set -u
}
start_bg(){
  local name="$1" cmd="$2" log="$3" pidfile="$4"
  info "Starting ${name} ..."
  bash -lc "set +u; source '${ROS_SETUP}'; source '${WS_SETUP}'; set -u; ${cmd}" \
    >"${log}" 2>&1 &
  echo $! > "${pidfile}"
  info "${name} pid=$(cat "${pidfile}") log=${log}"
}
wait_for_topic(){
  local topic="$1" timeout="$2"
  local tries=$((timeout * 10))
  for ((i=0; i<tries; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

preflight(){
  if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    err "Do NOT run with sudo."
    exit 1
  fi
  if [ ! -d "${WS}" ]; then
    err "Workspace not found: ${WS}"
    exit 1
  fi
  source_ros
  have_cmd ros2 || { err "ros2 not found after sourcing."; exit 1; }
}

do_start(){
  preflight

  # ---- Start SIM ----
  if ! pid_running "${SIM_PID}"; then
    start_bg "sim" "ros2 run ${SIM_PKG} ${SIM_EXEC} --ros-args -r __node:=${SIM_NODE}" "${SIM_LOG}" "${SIM_PID}"
  else
    warn "sim already running."
  fi

  # If your sim publishes joint states, check (adjust topic if your namespace differs)
  info "Waiting for /iiwa7/joint_states (timeout ${WAIT_TIMEOUT_SEC}s) ..."
  wait_for_topic "/iiwa7/joint_states" "${WAIT_TIMEOUT_SEC}" || warn "No /iiwa7/joint_states yet (check ${SIM_LOG})."

  # ---- Start TRAJ planner (optional) ----
  if [ "${ENABLE_TRAJ}" = "1" ]; then
    if ! pid_running "${TRAJ_PID}"; then
      start_bg "traj_planner" "ros2 run ${TRAJ_PKG} ${TRAJ_EXEC} --ros-args -r __node:=${TRAJ_NODE}" "${TRAJ_LOG}" "${TRAJ_PID}"
    else
      warn "traj_planner already running."
    fi
  else
    warn "ENABLE_TRAJ=0 -> skipping traj planner."
  fi

  # ---- Start BRIDGE -> /robot_states ----
  if ! pid_running "${BRIDGE_PID}"; then
    start_bg "bridge" "ros2 run ${BRIDGE_PKG} ${BRIDGE_EXEC} --ros-args -p port:=${UDP_PORT} -p rate_hz:=${BRIDGE_RATE_HZ} -r __node:=${BRIDGE_NODE}" "${BRIDGE_LOG}" "${BRIDGE_PID}"
  else
    warn "bridge already running."
  fi

  info "Waiting for /robot_states (timeout ${WAIT_TIMEOUT_SEC}s) ..."
  wait_for_topic "/robot_states" "${WAIT_TIMEOUT_SEC}" || warn "No /robot_states yet (check ${BRIDGE_LOG})."

  # ---- Start Robot Monitor ----
  if ! pid_running "${MON_PID}"; then
    start_bg "robot_monitor" "ros2 run ${MON_PKG} ${MON_EXEC} --ros-args -r __node:=${MON_NODE}" "${MON_LOG}" "${MON_PID}"
  else
    warn "robot_monitor already running."
  fi

  # ---- Track trajectory topic: hz + echo ----
  info "Tracking trajectory topic: ${TRAJ_TOPIC}"

  if ! pid_running "${TRAJ_HZ_PID}"; then
    start_bg "traj_hz" "ros2 topic hz ${TRAJ_TOPIC}" "${TRAJ_HZ_LOG}" "${TRAJ_HZ_PID}"
  fi
  if ! pid_running "${TRAJ_ECHO_PID}"; then
    # echo can be heavy; throttle with --qos-* if needed
    start_bg "traj_echo" "ros2 topic echo ${TRAJ_TOPIC}" "${TRAJ_ECHO_LOG}" "${TRAJ_ECHO_PID}"
  fi

  # ---- Quick status summary ----
  echo ""
  info "Startup complete."
  info "Check publishers:"
  ros2 topic info /robot_states -v || true
  ros2 topic info "${TRAJ_TOPIC}" -v || true
  echo ""
  info "Logs: ${LOG_DIR}"
  info "Use: $0 status | logs | stop"
}

do_stop(){
  preflight
  stop_pidfile "${TRAJ_ECHO_PID}"
  stop_pidfile "${TRAJ_HZ_PID}"
  stop_pidfile "${MON_PID}"
  stop_pidfile "${BRIDGE_PID}"
  stop_pidfile "${TRAJ_PID}"
  stop_pidfile "${SIM_PID}"
  info "Stopped."
}

do_status(){
  preflight
  echo "---- PIDs ----"
  for f in "${SIM_PID}" "${TRAJ_PID}" "${BRIDGE_PID}" "${MON_PID}" "${TRAJ_HZ_PID}" "${TRAJ_ECHO_PID}"; do
    if pid_running "$f"; then
      echo "$(basename "$f"): RUNNING pid=$(cat "$f")"
    else
      echo "$(basename "$f"): STOPPED"
    fi
  done
  echo ""
  echo "---- Topics ----"
  ros2 topic list 2>/dev/null | grep -E "/robot_states|${TRAJ_TOPIC}|/iiwa7/joint_states" || true
  echo ""
  echo "---- Topic Info ----"
  ros2 topic info /robot_states -v 2>/dev/null || true
  ros2 topic info "${TRAJ_TOPIC}" -v 2>/dev/null || true
}

do_logs(){
  echo "sim:        ${SIM_LOG}"
  echo "traj:       ${TRAJ_LOG}"
  echo "bridge:     ${BRIDGE_LOG}"
  echo "monitor:    ${MON_LOG}"
  echo "traj_hz:    ${TRAJ_HZ_LOG}"
  echo "traj_echo:  ${TRAJ_ECHO_LOG}"
  echo ""
  echo "Tail all (Ctrl+C to exit):"
  tail -n 80 -f "${SIM_LOG}" "${TRAJ_LOG}" "${BRIDGE_LOG}" "${MON_LOG}" "${TRAJ_HZ_LOG}" "${TRAJ_ECHO_LOG}" 2>/dev/null || true
}

cmd="${1:-start}"
case "${cmd}" in
  start)  do_start ;;
  stop)   do_stop ;;
  status) do_status ;;
  logs)   do_logs ;;
  *)
    err "Usage: $0 {start|stop|status|logs}"
    err "Env overrides: ENABLE_TRAJ=0 TRAJ_PKG=... TRAJ_EXEC=... TRAJ_TOPIC=... UDP_PORT=..."
    exit 1
    ;;
esac

