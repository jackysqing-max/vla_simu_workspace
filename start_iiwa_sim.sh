#!/bin/bash
set -euo pipefail

WS="${HOME}/ros2_pybullet_ws"

# --------- ROS pkgs / execs ----------
SIM_PKG="pybullet_ros2_sim"
SIM_EXEC="iiwa_pybullet_sim_node"
DES_PKG="pybullet_ros2_sim"
DES_EXEC="iiwa_circle_ik_desired"
CTRL_PKG="pybullet_ros2_sim"
CTRL_EXEC="iiwa_impedance_controller"

BRIDGE_PKG="iiwa_state_udp_bridge"
BRIDGE_EXEC="robotstate_bridge"   # 如果你的 bridge 可执行名不同，下面会自动探测

MON_PKG="robot_monitor"
MON_EXEC="robot_monitor_gui"

# --------- Default params ----------
K_DEFAULT="${K_DEFAULT:-800.0}"
D_DEFAULT="${D_DEFAULT:-40.0}"
TAU_LIM_DEFAULT="${TAU_LIM_DEFAULT:-200.0}"
UDP_PORT_DEFAULT="${UDP_PORT_DEFAULT:-7755}"
BRIDGE_RATE_DEFAULT="${BRIDGE_RATE_DEFAULT:-100.0}"

# --------- Runtime dirs ----------
LOG_DIR="${WS}/run_logs"
PID_DIR="${WS}/run_pids"
mkdir -p "${LOG_DIR}" "${PID_DIR}"

# --------- Helpers ----------
info(){ echo "[INFO] $*"; }
warn(){ echo "[WARN] $*" >&2; }
err(){ echo "[ERROR] $*" >&2; }

have_cmd(){ command -v "$1" >/dev/null 2>&1; }
have_gui(){ [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; }

# Source ROS env safely (avoid AMENT_TRACE_SETUP_FILES unbound)
source_env(){
  set +u
  : "${AMENT_TRACE_SETUP_FILES:=}"
  source /opt/ros/humble/setup.bash
  source "${WS}/install/setup.bash"
  set -u
}

pidfile_for(){ echo "${PID_DIR}/$1.pid"; }
logfile_for(){ echo "${LOG_DIR}/$1.log"; }

is_running(){
  local pf; pf="$(pidfile_for "$1")"
  [[ -f "$pf" ]] || return 1
  local pid; pid="$(cat "$pf" 2>/dev/null || true)"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" >/dev/null 2>&1
}

stop_one(){
  local name="$1"
  local pf; pf="$(pidfile_for "$name")"
  [[ -f "$pf" ]] || return 0
  local pid; pid="$(cat "$pf" 2>/dev/null || true)"
  [[ -n "$pid" ]] || { rm -f "$pf"; return 0; }
  if kill -0 "$pid" >/dev/null 2>&1; then
    info "Stopping $name (PID $pid)"
    kill "$pid" >/dev/null 2>&1 || true
    for _ in {1..30}; do
      kill -0 "$pid" >/dev/null 2>&1 || break
      sleep 0.1
    done
    kill -9 "$pid" >/dev/null 2>&1 || true
  fi
  rm -f "$pf"
}

start_bg(){
  local name="$1"; shift
  local cmd="$*"
  local lf; lf="$(logfile_for "$name")"
  local pf; pf="$(pidfile_for "$name")"

  info "Starting $name..."
  bash -lc "set +u; : \"\${AMENT_TRACE_SETUP_FILES:=}\"; source /opt/ros/humble/setup.bash; source ${WS}/install/setup.bash; set -u; ${cmd}" \
    >"$lf" 2>&1 &
  echo $! >"$pf"
  info "  PID=$(cat "$pf")  LOG=$lf"
}

ros2_exe_exists(){
  local pkg="$1" exe="$2"
  ros2 pkg executables "$pkg" 2>/dev/null | awk '{print $2}' | grep -qx "$exe"
}

pick_bridge_exec(){
  # 优先你已有的 robotstate_bridge；否则选包里第一个可执行文件
  if ros2_exe_exists "$BRIDGE_PKG" "$BRIDGE_EXEC"; then
    echo "$BRIDGE_EXEC"; return 0
  fi
  local first
  first="$(ros2 pkg executables "$BRIDGE_PKG" 2>/dev/null | awk 'NR==1{print $2}')"
  if [[ -n "$first" ]]; then
    echo "$first"; return 0
  fi
  echo "" ; return 1
}

run_monitor_cmd(){
  # 1) 尝试 ros2 run
  if ros2 pkg list | grep -qx "$MON_PKG" && ros2_exe_exists "$MON_PKG" "$MON_EXEC"; then
    echo "ros2 run ${MON_PKG} ${MON_EXEC}"
    return 0
  fi
  # 2) 尝试常见 build 路径
  if [[ -x "${WS}/build/robot_monitor/robot_monitor" ]]; then
    echo "${WS}/build/robot_monitor/robot_monitor"
    return 0
  fi
  if [[ -x "${WS}/build/robot_monitor/robot_monitor_gui" ]]; then
    echo "${WS}/build/robot_monitor/robot_monitor_gui"
    return 0
  fi
  echo "" ; return 1
}

wait_topic(){
  local topic="$1" timeout="$2"
  local tries=$((timeout*10))
  for ((i=0;i<tries;i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then return 0; fi
    sleep 0.1
  done
  return 1
}

usage(){
  cat <<USG
Usage:
  ./start_iiwa_sim.sh start [--k 800] [--d 40] [--tau 200] [--port 7755] [--rate 100]
  ./start_iiwa_sim.sh stop
  ./start_iiwa_sim.sh status
  ./start_iiwa_sim.sh logs
  ./start_iiwa_sim.sh free    # mouse drag enabled (disable torque)
  ./start_iiwa_sim.sh torque  # enable torque control
USG
}

# --------- Commands ----------
cmd="${1:-start}"; shift || true

case "$cmd" in
  start)
    source_env

    # parse args
    K="$K_DEFAULT"; D="$D_DEFAULT"; TAU="$TAU_LIM_DEFAULT"; PORT="$UDP_PORT_DEFAULT"; RATE="$BRIDGE_RATE_DEFAULT"
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --k) K="$2"; shift 2;;
        --d) D="$2"; shift 2;;
        --tau) TAU="$2"; shift 2;;
        --port) PORT="$2"; shift 2;;
        --rate) RATE="$2"; shift 2;;
        *) err "Unknown arg: $1"; usage; exit 1;;
      esac
    done

    # sanity: packages/executables
    if ! ros2 pkg list | grep -qx "$SIM_PKG"; then err "Missing pkg: $SIM_PKG"; exit 1; fi
    if ! ros2_exe_exists "$SIM_PKG" "$SIM_EXEC"; then err "Missing exe: $SIM_PKG $SIM_EXEC"; exit 1; fi
    if ! ros2_exe_exists "$DES_PKG" "$DES_EXEC"; then err "Missing exe: $DES_PKG $DES_EXEC"; exit 1; fi
    if ! ros2_exe_exists "$CTRL_PKG" "$CTRL_EXEC"; then err "Missing exe: $CTRL_PKG $CTRL_EXEC"; exit 1; fi

    BR_EXE="$(pick_bridge_exec || true)"
    if [[ -z "$BR_EXE" ]]; then
      warn "Bridge pkg '$BRIDGE_PKG' has no executables (will skip bridge)."
    fi

    MON_CMD="$(run_monitor_cmd || true)"
    if [[ -z "$MON_CMD" ]]; then
      warn "Monitor executable not found (will skip monitor)."
    fi

    # start sim
    if is_running sim; then warn "sim already running"; else
      start_bg sim "ros2 run ${SIM_PKG} ${SIM_EXEC}"
    fi

    info "Waiting for /iiwa7/joint_states (timeout 12s)..."
    if wait_topic "/iiwa7/joint_states" 12; then
      info "Sim ready: topic /iiwa7/joint_states is available."
    else
      warn "Timeout waiting /iiwa7/joint_states; check sim log: $(logfile_for sim)"
    fi

    # start controller + desired
    if is_running ctrl; then warn "ctrl already running"; else
      start_bg ctrl "ros2 run ${CTRL_PKG} ${CTRL_EXEC} --ros-args -p k:=${K} -p d:=${D} -p tau_lim:=${TAU}"
    fi
    if is_running des; then warn "desired already running"; else
      start_bg des "ros2 run ${DES_PKG} ${DES_EXEC}"
    fi

    # start bridge
    if [[ -n "$BR_EXE" ]]; then
      if is_running bridge; then warn "bridge already running"; else
        start_bg bridge "ros2 run ${BRIDGE_PKG} ${BR_EXE} --ros-args -p port:=${PORT} -p rate_hz:=${RATE}"
      fi
    fi

    # start monitor
    if [[ -n "$MON_CMD" ]]; then
      if is_running mon; then warn "monitor already running"; else
        start_bg mon "${MON_CMD}"
      fi
    fi

    info "Started."
    info "  Impedance params: k=${K}, d=${D}, tau_lim=${TAU}"
    info "  UDP: port=${PORT}, rate_hz=${RATE}"
    info "  Use: ./start_iiwa_sim.sh status | logs | stop | free | torque"
    ;;

  stop)
    source_env || true
    stop_one mon
    stop_one bridge
    stop_one des
    stop_one ctrl
    stop_one sim
    info "Stopped."
    ;;

  status)
    source_env || true
    for n in sim ctrl des bridge mon; do
      if is_running "$n"; then
        echo "[OK] $n PID=$(cat "$(pidfile_for "$n")")"
      else
        echo "[--] $n not running"
      fi
    done
    ;;

  logs)
    echo "== sim ==";  tail -n 80 "$(logfile_for sim)" 2>/dev/null || true
    echo "== ctrl =="; tail -n 80 "$(logfile_for ctrl)" 2>/dev/null || true
    echo "== des ==";  tail -n 80 "$(logfile_for des)" 2>/dev/null || true
    echo "== bridge =="; tail -n 80 "$(logfile_for bridge)" 2>/dev/null || true
    echo "== mon ==";  tail -n 80 "$(logfile_for mon)" 2>/dev/null || true
    ;;

  free)
    source_env
    ros2 topic pub --once /iiwa7/enable_torque std_msgs/msg/Bool "{data: false}" >/dev/null
    info "Torque disabled -> free mode (mouse drag enabled)."
    ;;

  torque)
    source_env
    ros2 topic pub --once /iiwa7/enable_torque std_msgs/msg/Bool "{data: true}" >/dev/null
    info "Torque enabled."
    ;;

  *)
    usage
    exit 1
    ;;
esac
