#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
SAM3_PROMPT="${SAM3_PROMPT:-red cube}"
SAM3_DEVICE="${SAM3_DEVICE:-cuda}"
SAM3_INFER_HZ="${SAM3_INFER_HZ:-2.0}"
SAM3_MAX_SIDE="${SAM3_MAX_SIDE:-320}"
SAM3_SCORE_TH="${SAM3_SCORE_TH:-0.10}"
SAM3_MASK_TH="${SAM3_MASK_TH:-0.40}"
FUSION_MIN_SCORE="${FUSION_MIN_SCORE:-0.05}"
HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.10}"
TARGET_TIMEOUT_SEC="${TARGET_TIMEOUT_SEC:-2.0}"
SIM_COLOR_HZ="${SIM_COLOR_HZ:-8.0}"
SIM_POINTS_HZ="${SIM_POINTS_HZ:-1.0}"
SIM_JOINT_STATE_HZ="${SIM_JOINT_STATE_HZ:-60.0}"
SIM_POSITION_FORCE="${SIM_POSITION_FORCE:-800.0}"
TRACKER_PUBLISH_HZ="${TRACKER_PUBLISH_HZ:-80.0}"
TRACKER_MAX_JOINT_STEP_RAD="${TRACKER_MAX_JOINT_STEP_RAD:-0.10}"
TRACKER_START_ENABLED="${TRACKER_START_ENABLED:-false}"
PYTORCH_ALLOC_CONF_VALUE="${PYTORCH_ALLOC_CONF_VALUE:-expandable_segments:True}"
SIM_GUI="${SIM_GUI:-true}"

LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
mkdir -p "$LOG_DIR" "$PID_DIR"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
err() { echo "[ERROR] $*" >&2; }

pidfile_for() { echo "$PID_DIR/$1.pid"; }
logfile_for() { echo "$LOG_DIR/$1.log"; }

source_ros() {
  set +u
  source "$ROS_SETUP"
  source "$WS_SETUP"
  set -u 2>/dev/null || true
}

pid_alive() {
  local pid="$1"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

start_bg_ros() {
  local name="$1"; shift
  local cmd="$*"
  local log; log="$(logfile_for "$name")"
  local pidf; pidf="$(pidfile_for "$name")"

  setsid bash -lc "
    set +u
    source \"$ROS_SETUP\"
    source \"$WS_SETUP\"
    set -u 2>/dev/null || true
    echo \$\$ > \"$pidf\"
    exec $cmd
  " >"$log" 2>&1 &

  sleep 0.1
  local pid
  pid="$(cat "$pidf" 2>/dev/null || true)"
  info "started $name pid=${pid:-unknown}"
}

start_bg_sam3() {
  local name="$1"; shift
  local cmd="$*"
  local log; log="$(logfile_for "$name")"
  local pidf; pidf="$(pidfile_for "$name")"

  if [[ ! -f "$SAM3_VENV/bin/activate" ]]; then
    err "SAM3 virtualenv not found: $SAM3_VENV"
    exit 1
  fi

  setsid bash -lc "
    set +u
    source \"$ROS_SETUP\"
    source \"$WS_SETUP\"
    source \"$SAM3_VENV/bin/activate\"
    export PYTORCH_ALLOC_CONF=\"$PYTORCH_ALLOC_CONF_VALUE\"
    set -u 2>/dev/null || true
    echo \$\$ > \"$pidf\"
    exec $cmd
  " >"$log" 2>&1 &

  sleep 0.1
  local pid
  pid="$(cat "$pidf" 2>/dev/null || true)"
  info "started $name pid=${pid:-unknown}"
}

stop_one() {
  local name="$1"
  local pidf; pidf="$(pidfile_for "$name")"
  [[ -f "$pidf" ]] || return 0

  local pid
  pid="$(cat "$pidf" 2>/dev/null || true)"
  [[ -n "$pid" ]] || { rm -f "$pidf"; return 0; }

  for sig in INT TERM KILL; do
    if pid_alive "$pid"; then
      kill "-$sig" -- "-$pid" 2>/dev/null || kill "-$sig" "$pid" 2>/dev/null || true
      sleep 1
    fi
  done

  rm -f "$pidf"
}

wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local tries=$((timeout_sec * 10))
  for ((i=0; i<tries; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

show_status() {
  for name in sim sam3 fusion registry tracker bridge monitor; do
    local pidf
    pidf="$(pidfile_for "$name")"
    if [[ -f "$pidf" ]] && pid_alive "$(cat "$pidf" 2>/dev/null || true)"; then
      echo "[OK] $name pid=$(cat "$pidf")"
    else
      echo "[--] $name not running"
    fi
  done
}

run_prompt_shell() {
  source_ros
  python3 "$WS/scripts/semantic_prompt_cli.py"
}

send_one_prompt() {
  local text="$1"
  source_ros
  python3 "$WS/scripts/semantic_prompt_cli.py" --once "$text"
}

case "${1:-start}" in
  start)
    source_ros

    start_bg_ros sim \
      "ros2 run pybullet_ros2_sim iiwa_pybullet_rgbd_sim_node --ros-args \
       -p gui:=$SIM_GUI \
       -p cam_width:=320 -p cam_height:=240 -p color_hz:=$SIM_COLOR_HZ -p points_hz:=$SIM_POINTS_HZ \
       -p joint_state_hz:=$SIM_JOINT_STATE_HZ -p position_force:=$SIM_POSITION_FORCE"

    if ! wait_topic "/sim/camera/color/image_raw" 15; then
      warn "camera topic not ready yet; continuing anyway"
    fi

    start_bg_sam3 sam3 \
      "python -m sam3_ros.sam3_mask_node --ros-args \
       -p image_topic:=/sim/camera/color/image_raw \
       -p prompt:=\"$SAM3_PROMPT\" \
       -p device:=\"$SAM3_DEVICE\" \
       -p infer_hz:=$SAM3_INFER_HZ \
       -p max_side:=$SAM3_MAX_SIDE \
       -p score_th:=$SAM3_SCORE_TH \
       -p mask_th:=$SAM3_MASK_TH"

    start_bg_ros fusion \
      "ros2 run perception_geometry mask_depth_fusion_node --ros-args \
       -p min_score:=$FUSION_MIN_SCORE \
       -p cluster_count:=4 -p cluster_max_samples:=512 \
       -p cluster_meanshift_bandwidth_m:=0.04 \
       -p top_surface_band_m:=0.012 \
       -p top_surface_min_fraction:=0.15"

    start_bg_ros registry \
      "ros2 run pybullet_ros2_sim scene_object_registry_node --ros-args \
       -p publish_hz:=2.0 -p observation_timeout_sec:=1.5 -p visible_timeout_sec:=2.0"

    start_bg_ros tracker \
      "ros2 run pybullet_ros2_sim iiwa_keypoint_tracker_node --ros-args \
       -p publish_hz:=$TRACKER_PUBLISH_HZ -p max_joint_step_rad:=$TRACKER_MAX_JOINT_STEP_RAD \
       -p hover_offset_z:=$HOVER_OFFSET_Z \
       -p target_timeout_sec:=$TARGET_TIMEOUT_SEC \
       -p start_enabled:=$TRACKER_START_ENABLED"

    start_bg_ros bridge \
      "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p rate_hz:=60.0"

    start_bg_ros monitor \
      "ros2 run robot_monitor robot_monitor"

    info "Semantic tracking demo started"
    info "sam3 prompt=$SAM3_PROMPT device=$SAM3_DEVICE venv=$SAM3_VENV"
    info "sim gui=$SIM_GUI"
    info "sam3 infer_hz=$SAM3_INFER_HZ max_side=$SAM3_MAX_SIDE score_th=$SAM3_SCORE_TH mask_th=$SAM3_MASK_TH"
    info "fusion min_score=$FUSION_MIN_SCORE"
    info "hover_offset_z=$HOVER_OFFSET_Z target_timeout_sec=$TARGET_TIMEOUT_SEC"
    info "tracker start_enabled=$TRACKER_START_ENABLED publish_hz=$TRACKER_PUBLISH_HZ max_joint_step_rad=$TRACKER_MAX_JOINT_STEP_RAD"
    info "sim color_hz=$SIM_COLOR_HZ points_hz=$SIM_POINTS_HZ joint_state_hz=$SIM_JOINT_STATE_HZ position_force=$SIM_POSITION_FORCE"
    info "Open another terminal and run:"
    echo "  ./start_semantic_tracking_demo.sh shell"
    info "Or switch target directly:"
    echo "  ./start_semantic_tracking_demo.sh prompt 'blue cube'"
    ;;

  stop)
    stop_one monitor
    stop_one bridge
    stop_one tracker
    stop_one registry
    stop_one fusion
    stop_one sam3
    stop_one sim
    info "Semantic tracking demo stopped"
    ;;

  status)
    show_status
    ;;

  logs)
    tail -n 80 -f \
      "$(logfile_for sim)" \
      "$(logfile_for sam3)" \
      "$(logfile_for fusion)" \
      "$(logfile_for registry)" \
      "$(logfile_for tracker)" \
      "$(logfile_for bridge)" \
      "$(logfile_for monitor)" 2>/dev/null || true
    ;;

  clean)
    "$WS/clean_demo_processes.sh"
    ;;

  shell|interactive)
    run_prompt_shell
    ;;

  prompt)
    shift || true
    if [[ $# -eq 0 ]]; then
      err "Usage: $0 prompt \"target prompt\""
      exit 1
    fi
    send_one_prompt "$*"
    ;;

  *)
    echo "Usage: $0 {start|stop|status|logs|shell|interactive|prompt|clean}"
    echo "Optional env:"
    echo "  SIM_GUI=true"
    echo "  SAM3_VENV=$HOME/venvs/ros_vla"
    echo "  SAM3_PROMPT='red cube'"
    echo "  SAM3_DEVICE=cpu"
    echo "  SAM3_INFER_HZ=1.0"
    echo "  SAM3_MAX_SIDE=320"
    echo "  SAM3_SCORE_TH=0.10"
    echo "  SAM3_MASK_TH=0.40"
    echo "  FUSION_MIN_SCORE=0.05"
    echo "  HOVER_OFFSET_Z=0.10"
    echo "  TARGET_TIMEOUT_SEC=2.0"
    echo "  SIM_POSITION_FORCE=800.0"
    echo "  TRACKER_PUBLISH_HZ=80.0"
    echo "  TRACKER_MAX_JOINT_STEP_RAD=0.10"
    echo "  TRACKER_START_ENABLED=false"
    echo "Examples:"
    echo "  $0 start"
    echo "  SIM_GUI=false $0 start"
    echo "  $0 clean"
    echo "  $0 shell"
    echo "  $0 prompt 'blue cube'"
    exit 1
    ;;
esac
