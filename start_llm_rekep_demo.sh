#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
SAM3_PROMPT="${SAM3_PROMPT:-red cube}"
SAM3_DEVICE="${SAM3_DEVICE:-cuda}"

OPENAI_MODEL="${OPENAI_MODEL:-gpt-5-mini}"
OPENAI_REASONING_EFFORT="${OPENAI_REASONING_EFFORT:-low}"

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

pg_alive() {
  local pgid="$1"
  ps -o pid= --pgid "$pgid" 2>/dev/null | grep -qE '[0-9]+' || return 1
  return 0
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
    exec $cmd
  " >"$log" 2>&1 &

  local pgid=$!
  echo "$pgid" >"$pidf"
  info "started $name pgid=$pgid"
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
    set -u 2>/dev/null || true
    exec $cmd
  " >"$log" 2>&1 &

  local pgid=$!
  echo "$pgid" >"$pidf"
  info "started $name pgid=$pgid"
}

stop_one() {
  local name="$1"
  local pidf; pidf="$(pidfile_for "$name")"
  [[ -f "$pidf" ]] || return 0

  local pgid
  pgid="$(cat "$pidf" 2>/dev/null || true)"
  [[ -n "$pgid" ]] || { rm -f "$pidf"; return 0; }

  for sig in INT TERM KILL; do
    if pg_alive "$pgid"; then
      kill "-$sig" -- "-$pgid" 2>/dev/null || true
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
  for name in sim sam3 fusion tracker bridge monitor planner executor; do
    local pidf
    pidf="$(pidfile_for "$name")"
    if [[ -f "$pidf" ]] && pg_alive "$(cat "$pidf" 2>/dev/null || true)"; then
      echo "[OK] $name pgid=$(cat "$pidf")"
    else
      echo "[--] $name not running"
    fi
  done
}

case "${1:-start}" in
  start)
    source_ros

    if [[ -z "${OPENAI_API_KEY:-}" ]]; then
      err "OPENAI_API_KEY is not set"
      exit 1
    fi

    start_bg_ros sim \
      "ros2 run pybullet_ros2_sim iiwa_pybullet_rgbd_sim_node --ros-args \
       -p cam_width:=320 -p cam_height:=240 -p color_hz:=8.0 -p points_hz:=1.0 \
       -p joint_state_hz:=60.0 -p position_force:=500.0"

    if ! wait_topic "/sim/camera/color/image_raw" 15; then
      warn "camera topic not ready yet; continuing anyway"
    fi

    start_bg_sam3 sam3 \
      "python -m sam3_ros.sam3_mask_node --ros-args \
       -p image_topic:=/sim/camera/color/image_raw \
       -p prompt:=\"$SAM3_PROMPT\" \
       -p device:=\"$SAM3_DEVICE\" \
       -p infer_hz:=2.0 \
       -p max_side:=384"

    start_bg_ros fusion \
      "ros2 run perception_geometry mask_depth_fusion_node --ros-args \
       -p cluster_count:=4 -p cluster_max_samples:=512 -p cluster_meanshift_bandwidth_m:=0.04"

    start_bg_ros tracker \
      "ros2 run pybullet_ros2_sim iiwa_keypoint_tracker_node --ros-args \
       -p publish_hz:=60.0 -p max_joint_step_rad:=0.06 -p hover_offset_z:=0.05"

    start_bg_ros bridge \
      "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p rate_hz:=60.0"

    start_bg_ros monitor \
      "ros2 run robot_monitor robot_monitor"

    start_bg_ros planner \
      "ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
       -p model:=\"$OPENAI_MODEL\" \
       -p reasoning_effort:=\"$OPENAI_REASONING_EFFORT\""

    start_bg_ros executor \
      "ros2 run pybullet_ros2_sim llm_task_executor_node"

    info "LLM tabletop demo stack started"
    info "sam3 prompt=$SAM3_PROMPT device=$SAM3_DEVICE venv=$SAM3_VENV"
    info "openai model=$OPENAI_MODEL reasoning_effort=$OPENAI_REASONING_EFFORT"
    info "Send instruction:"
    echo "  ros2 topic pub --once /llm_task/instruction std_msgs/msg/String \"{data: '依次移动到红色方块、蓝色方块和黄色方块上方'}\""
    ;;

  stop)
    stop_one executor
    stop_one planner
    stop_one monitor
    stop_one bridge
    stop_one tracker
    stop_one fusion
    stop_one sam3
    stop_one sim
    info "LLM tabletop demo stack stopped"
    ;;

  status)
    show_status
    ;;

  logs)
    tail -n 80 -f \
      "$(logfile_for sim)" \
      "$(logfile_for sam3)" \
      "$(logfile_for fusion)" \
      "$(logfile_for tracker)" \
      "$(logfile_for bridge)" \
      "$(logfile_for monitor)" \
      "$(logfile_for planner)" \
      "$(logfile_for executor)" 2>/dev/null || true
    ;;

  *)
    echo "Usage: $0 {start|stop|status|logs}"
    echo "Required env:"
    echo "  OPENAI_API_KEY=..."
    echo "Optional env:"
    echo "  OPENAI_MODEL=gpt-5-mini"
    echo "  OPENAI_REASONING_EFFORT=low"
    echo "  SAM3_VENV=$HOME/venvs/ros_vla"
    echo "  SAM3_PROMPT='red cube'"
    echo "  SAM3_DEVICE=cuda"
    exit 1
    ;;
esac
