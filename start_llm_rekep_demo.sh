#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
SAM3_PROMPT="${SAM3_PROMPT:-red cube}"
SAM3_DEVICE="${SAM3_DEVICE:-cuda}"
SAM3_IMAGE_TOPIC="${SAM3_IMAGE_TOPIC:-/sim/camera/color/image_raw}"
SAM3_PROMPT_TOPIC="${SAM3_PROMPT_TOPIC:-/sam3/prompt}"
SAM3_ACTIVE_PROMPT_TOPIC="${SAM3_ACTIVE_PROMPT_TOPIC:-/sam3/active_prompt}"
SAM3_MASK_TOPIC="${SAM3_MASK_TOPIC:-/sam3/mask}"
SAM3_SCORE_TOPIC="${SAM3_SCORE_TOPIC:-/sam3/score}"
HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.10}"
SAM3_SCORE_TH="${SAM3_SCORE_TH:-0.10}"
SAM3_MASK_TH="${SAM3_MASK_TH:-0.40}"
FUSION_MIN_SCORE="${FUSION_MIN_SCORE:-0.05}"
TRACKER_TARGET_TIMEOUT_SEC="${TRACKER_TARGET_TIMEOUT_SEC:-4.0}"
TRACKER_LOCK_TARGET_ON_ENABLE="${TRACKER_LOCK_TARGET_ON_ENABLE:-false}"
TRACKER_USE_LAST_TARGET_ON_OCCLUSION="${TRACKER_USE_LAST_TARGET_ON_OCCLUSION:-false}"
TRACKER_OCCLUDED_TARGET_HOLD_SEC="${TRACKER_OCCLUDED_TARGET_HOLD_SEC:-5.0}"
EXECUTOR_SUCCESS_RADIUS_M="${EXECUTOR_SUCCESS_RADIUS_M:-0.10}"
SIM_GUI="${SIM_GUI:-true}"
GPU_MONITOR_WINDOW="${GPU_MONITOR_WINDOW:-auto}"
GPU_MONITOR_REFRESH_SEC="${GPU_MONITOR_REFRESH_SEC:-1.0}"
SIM_SCENE_PRESET="${SIM_SCENE_PRESET:-cubes}"
MEDICAL_GRASP_SCISSORS_FIXED="${MEDICAL_GRASP_SCISSORS_FIXED:-true}"
MEDICAL_GRASP_SCISSORS_GRASPABLE="${MEDICAL_GRASP_SCISSORS_GRASPABLE:-false}"
MEDICAL_SORTING_TRAY_ENABLED="${MEDICAL_SORTING_TRAY_ENABLED:-true}"
MEDICAL_SORTING_TRAY_OFFSET_XY="${MEDICAL_SORTING_TRAY_OFFSET_XY:-[0.205,0.125]}"
MEDICAL_SORTING_TRAY_SIZE_XY="${MEDICAL_SORTING_TRAY_SIZE_XY:-[0.260,0.170]}"
MEDICAL_SORTING_TRAY_WALL_HEIGHT_M="${MEDICAL_SORTING_TRAY_WALL_HEIGHT_M:-0.035}"
EXTERNAL_SCENE_OBJECTS_JSON="${EXTERNAL_SCENE_OBJECTS_JSON:-[]}"
EXTERNAL_SCENE_OBJECTS_FILE="${EXTERNAL_SCENE_OBJECTS_FILE:-}"
STABILIZE_RESTING_GRASPABLES="${STABILIZE_RESTING_GRASPABLES:-true}"
RESTING_TABLE_CLEARANCE_M="${RESTING_TABLE_CLEARANCE_M:-0.0}"
RESTING_SNAP_MARGIN_M="${RESTING_SNAP_MARGIN_M:-0.025}"
RESTING_LINEAR_SPEED_TH_MPS="${RESTING_LINEAR_SPEED_TH_MPS:-0.020}"
RESTING_ANGULAR_SPEED_TH_RADPS="${RESTING_ANGULAR_SPEED_TH_RADPS:-0.080}"
RESTING_REANCHOR_DELAY_SEC="${RESTING_REANCHOR_DELAY_SEC:-0.60}"
RESTING_INTERACTION_RELEASE_RADIUS_M="${RESTING_INTERACTION_RELEASE_RADIUS_M:-0.160}"
OPEN_VOCAB_TARGETS="${OPEN_VOCAB_TARGETS:-false}"
ENABLE_GRIPPER="${ENABLE_GRIPPER:-false}"
ENABLE_GRASP_ACTIONS="${ENABLE_GRASP_ACTIONS:-false}"
GRIPPER_MODEL="${GRIPPER_MODEL:-kuka_parallel}"
GRIPPER_GRASP_RADIUS_M="${GRIPPER_GRASP_RADIUS_M:-0.13}"
GRIPPER_CONTACT_DISTANCE_M="${GRIPPER_CONTACT_DISTANCE_M:-0.012}"
GRIPPER_REQUIRE_ACTUAL_CONTACT="${GRIPPER_REQUIRE_ACTUAL_CONTACT:-true}"
GRIPPER_GRASP_MODE="${GRIPPER_GRASP_MODE:-keypoint}"
GRIPPER_KEYPOINT_SNAP_TO_CENTER="${GRIPPER_KEYPOINT_SNAP_TO_CENTER:-false}"
GRIPPER_KEYPOINT_SNAP_LOCAL_OFFSET="${GRIPPER_KEYPOINT_SNAP_LOCAL_OFFSET:-[0.0,0.0,0.0]}"
GRIPPER_PHYSICAL_CONTACT_GUARD="${GRIPPER_PHYSICAL_CONTACT_GUARD:-true}"
GRIPPER_PHYSICAL_PRELOAD_RAD="${GRIPPER_PHYSICAL_PRELOAD_RAD:-0.025}"
GRIPPER_PHYSICAL_FORCE="${GRIPPER_PHYSICAL_FORCE:-35.0}"
GRIPPER_PHYSICAL_POSITION_GAIN="${GRIPPER_PHYSICAL_POSITION_GAIN:-0.35}"
GRIPPER_PHYSICAL_MAX_VELOCITY="${GRIPPER_PHYSICAL_MAX_VELOCITY:-0.55}"
GRIPPER_GRASP_SETTLE_SEC="${GRIPPER_GRASP_SETTLE_SEC:-0.35}"
GRIPPER_GRASP_TIMEOUT_SEC="${GRIPPER_GRASP_TIMEOUT_SEC:-1.20}"
GRIPPER_AUTO_LIFT_HEIGHT_M="${GRIPPER_AUTO_LIFT_HEIGHT_M:-0.10}"
GRIPPER_AUTO_LIFT_SPEED_MPS="${GRIPPER_AUTO_LIFT_SPEED_MPS:-0.10}"
GRIPPER_AUTO_LOWER_SPEED_MPS="${GRIPPER_AUTO_LOWER_SPEED_MPS:-0.12}"
TRACK_GRIPPER_CENTER="${TRACK_GRIPPER_CENTER:-false}"
GRIPPER_CENTER_OFFSET_LINK6_X="${GRIPPER_CENTER_OFFSET_LINK6_X:-0.0}"
GRIPPER_CENTER_OFFSET_LINK6_Y="${GRIPPER_CENTER_OFFSET_LINK6_Y:-0.0}"
GRIPPER_CENTER_OFFSET_LINK6_Z="${GRIPPER_CENTER_OFFSET_LINK6_Z:-0.0}"
GRASP_LIFT_M="${GRASP_LIFT_M:-0.12}"
GRASP_PREGRASP_OFFSET_M="${GRASP_PREGRASP_OFFSET_M:-$HOVER_OFFSET_Z}"
GRASP_PREGRASP_SUCCESS_RADIUS_M="${GRASP_PREGRASP_SUCCESS_RADIUS_M:-0.035}"
GRASP_PREGRASP_HOLD_SEC="${GRASP_PREGRASP_HOLD_SEC:-0.25}"
GRASP_APPROACH_OFFSET_Z="${GRASP_APPROACH_OFFSET_Z:--1.0}"
GRASP_LIFT_DELAY_SEC="${GRASP_LIFT_DELAY_SEC:-0.35}"
GRASP_CONFIRM_HOLD_SEC="${GRASP_CONFIRM_HOLD_SEC:-0.25}"
GRASP_LIFT_HOLD_SEC="${GRASP_LIFT_HOLD_SEC:-1.2}"
GRASP_CLOSE_SUCCESS_RADIUS_M="${GRASP_CLOSE_SUCCESS_RADIUS_M:-0.10}"
GRASP_LIFT_SUCCESS_RADIUS_M="${GRASP_LIFT_SUCCESS_RADIUS_M:-0.05}"
ALIGN_TO_OBJECT_YAW="${ALIGN_TO_OBJECT_YAW:-true}"
GRASP_YAW_OFFSET_RAD="${GRASP_YAW_OFFSET_RAD:-1.57079632679}"
OBJECT_YAW_TIMEOUT_SEC="${OBJECT_YAW_TIMEOUT_SEC:-1.0}"
ALIGN_HOVER_OFFSET_MARGIN_M="${ALIGN_HOVER_OFFSET_MARGIN_M:-0.04}"
USE_EXPLICIT_GRASP_FRAME="${USE_EXPLICIT_GRASP_FRAME:-false}"
USE_OBJECT_PLANE_FRAME="${USE_OBJECT_PLANE_FRAME:-true}"
OBJECT_PLANE_TIMEOUT_SEC="${OBJECT_PLANE_TIMEOUT_SEC:-1.0}"
GRIPPER_FORWARD_DOWN_ANGLE_RAD="${GRIPPER_FORWARD_DOWN_ANGLE_RAD:-0.93}"
KEEP_GRIPPER_VERTICAL_TO_TABLE="${KEEP_GRIPPER_VERTICAL_TO_TABLE:-false}"
MIN_GRIPPER_CENTER_Z="${MIN_GRIPPER_CENTER_Z:--1.0}"
IK_USE_NULLSPACE_REST="${IK_USE_NULLSPACE_REST:-false}"
IK_NULLSPACE_INITIAL_WEIGHT="${IK_NULLSPACE_INITIAL_WEIGHT:-0.60}"
IK_JOINT_DAMPING="${IK_JOINT_DAMPING:-[0.12,0.12,0.12,0.12,0.12,0.12,0.12]}"
STATIC_OVERRIDE_KEEP_CURRENT_ORIENTATION="${STATIC_OVERRIDE_KEEP_CURRENT_ORIENTATION:-true}"
RELEASE_HOVER_OFFSET_Z="${RELEASE_HOVER_OFFSET_Z:-0.035}"
RELEASE_SUCCESS_RADIUS_M="${RELEASE_SUCCESS_RADIUS_M:-0.055}"
RELEASE_APPROACH_TIMEOUT_SEC="${RELEASE_APPROACH_TIMEOUT_SEC:-3.0}"
PLACE_SUCCESS_RADIUS_M="${PLACE_SUCCESS_RADIUS_M:-0.040}"
PLACE_HOVER_OFFSET_Z="${PLACE_HOVER_OFFSET_Z:-0.12}"
USE_STATIC_TRAY_CENTER_TARGET="${USE_STATIC_TRAY_CENTER_TARGET:-false}"
STATIC_TRAY_CENTER_WORLD="${STATIC_TRAY_CENTER_WORLD:-[0.555,0.150,0.351]}"
STATIC_TRAY_PLANE_NORMAL_WORLD="${STATIC_TRAY_PLANE_NORMAL_WORLD:-[0.0,0.0,1.0]}"
RELEASE_LOWER_HOLD_SEC="${RELEASE_LOWER_HOLD_SEC:-1.0}"
RELEASE_OPEN_WAIT_SEC="${RELEASE_OPEN_WAIT_SEC:-0.5}"

LLM_BACKEND="${LLM_BACKEND:-openai}"
GPU_EXECUTION_MODE="${GPU_EXECUTION_MODE:-concurrent}"
OPENAI_MODEL="${OPENAI_MODEL:-gpt-5-mini}"
OPENAI_REASONING_EFFORT="${OPENAI_REASONING_EFFORT:-low}"
QWEN3_VENV="${QWEN3_VENV:-/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm}"
QWEN_MODEL="${QWEN_MODEL:-Qwen/Qwen3-4B}"
QWEN3_HOST="${QWEN3_HOST:-127.0.0.1}"
QWEN3_PORT="${QWEN3_PORT:-8000}"
QWEN3_AUTO_START="${QWEN3_AUTO_START:-true}"
QWEN3_STOP_EXTERNAL="${QWEN3_STOP_EXTERNAL:-false}"
QWEN3_READY_TIMEOUT_SEC="${QWEN3_READY_TIMEOUT_SEC:-900}"
QWEN3_GPU_MEMORY_UTILIZATION="${QWEN3_GPU_MEMORY_UTILIZATION:-0.88}"
QWEN3_MAX_MODEL_LEN="${QWEN3_MAX_MODEL_LEN:-8192}"
QWEN3_TEMPERATURE="${QWEN3_TEMPERATURE:-0.0}"
QWEN3_TOP_P="${QWEN3_TOP_P:-1.0}"
QWEN3_MAX_OUTPUT_TOKENS="${QWEN3_MAX_OUTPUT_TOKENS:-512}"
QWEN3_EXTRA_REQUEST_BODY_JSON="${QWEN3_EXTRA_REQUEST_BODY_JSON:-{\"top_k\": 20, \"chat_template_kwargs\": {\"enable_thinking\": false}}}"
ALLOW_LOCAL_PLANNER_FALLBACK="${ALLOW_LOCAL_PLANNER_FALLBACK:-true}"
PLANNING_TIMEOUT_SEC="${PLANNING_TIMEOUT_SEC:-180}"
EXECUTION_TIMEOUT_SEC="${EXECUTION_TIMEOUT_SEC:-300}"
SAM3_EXEC_DEVICE="${SAM3_EXEC_DEVICE:-cuda}"
SAM3_READY_TIMEOUT_SEC="${SAM3_READY_TIMEOUT_SEC:-120}"
KEYPOINT_DEBUG_ONLY="${KEYPOINT_DEBUG_ONLY:-false}"
KEYPOINT_DEBUG_TIMEOUT_SEC="${KEYPOINT_DEBUG_TIMEOUT_SEC:-120}"
KEYPOINT_DEBUG_AUTO_ADVANCE="${KEYPOINT_DEBUG_AUTO_ADVANCE:-false}"
KEYPOINT_DEBUG_AUTO_ADVANCE_HOLD_SEC="${KEYPOINT_DEBUG_AUTO_ADVANCE_HOLD_SEC:-1.0}"
KEYPOINT_DEBUG_PROMPT_REPUBLISH_SEC="${KEYPOINT_DEBUG_PROMPT_REPUBLISH_SEC:-1.0}"
KEYPOINT_DEBUG_STATUS_HEARTBEAT_SEC="${KEYPOINT_DEBUG_STATUS_HEARTBEAT_SEC:-1.0}"
KEYPOINT_DEBUG_VALID_TOPIC="${KEYPOINT_DEBUG_VALID_TOPIC:-/perception/valid}"
KEYPOINT_DEBUG_KEYPOINT_TOPIC="${KEYPOINT_DEBUG_KEYPOINT_TOPIC:-/perception/keypoint_3d}"
KEYPOINT_DEBUG_KEYPOINT_PX_TOPIC="${KEYPOINT_DEBUG_KEYPOINT_PX_TOPIC:-/perception/keypoint_px}"
KEYPOINT_DEBUG_PLANE_NORMAL_TOPIC="${KEYPOINT_DEBUG_PLANE_NORMAL_TOPIC:-/perception/object_plane_normal}"
KEYPOINT_DEBUG_PLANE_TANGENT_TOPIC="${KEYPOINT_DEBUG_PLANE_TANGENT_TOPIC:-/perception/object_plane_tangent}"
START_KEYPOINT_VIEWER="${START_KEYPOINT_VIEWER:-false}"
KEYPOINT_VIEWER_DISPLAY_SCALE="${KEYPOINT_VIEWER_DISPLAY_SCALE:-2.0}"
KEYPOINT_VIEWER_STATUS_PANEL_WIDTH="${KEYPOINT_VIEWER_STATUS_PANEL_WIDTH:-360}"
KEYPOINT_VIEWER_FOLLOW_WINDOW_RESIZE="${KEYPOINT_VIEWER_FOLLOW_WINDOW_RESIZE:-true}"
KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE="${KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE:-false}"
KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW="${KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW:-true}"
KEYPOINT_VIEWER_OVERLAY_TOPIC="${KEYPOINT_VIEWER_OVERLAY_TOPIC:-/perception/keypoint_overlay}"
KEYPOINT_VIEWER_POINTS_TOPIC="${KEYPOINT_VIEWER_POINTS_TOPIC:-/perception/masked_points}"
KEYPOINT_VIEWER_KEYPOINT_TOPIC="${KEYPOINT_VIEWER_KEYPOINT_TOPIC:-/perception/keypoint_3d}"
KEYPOINT_VIEWER_VALID_TOPIC="${KEYPOINT_VIEWER_VALID_TOPIC:-/perception/valid}"
START_HOVER_FOLLOWER="${START_HOVER_FOLLOWER:-false}"
FOLLOWER_KEYPOINT_TOPIC="${FOLLOWER_KEYPOINT_TOPIC:-/perception/keypoint_3d}"
FOLLOWER_VALID_TOPIC="${FOLLOWER_VALID_TOPIC:-/perception/valid}"
FOLLOWER_PLANE_NORMAL_TOPIC="${FOLLOWER_PLANE_NORMAL_TOPIC:-/perception/object_plane_normal}"
FOLLOWER_PLANE_TANGENT_TOPIC="${FOLLOWER_PLANE_TANGENT_TOPIC:-/perception/object_plane_tangent}"
FOLLOWER_ENABLE_TOPIC="${FOLLOWER_ENABLE_TOPIC:-/llm_task/tracking_enabled}"
FOLLOWER_STATUS_TOPIC="${FOLLOWER_STATUS_TOPIC:-/iiwa7/keypoint_tracker_status}"
FOLLOWER_START_ENABLED="${FOLLOWER_START_ENABLED:-true}"
FOLLOWER_HOVER_OFFSET_Z="${FOLLOWER_HOVER_OFFSET_Z:-$HOVER_OFFSET_Z}"
FOLLOWER_MAX_JOINT_STEP_RAD="${FOLLOWER_MAX_JOINT_STEP_RAD:-0.04}"
FOLLOWER_REQUIRE_OBJECT_PLANE_FRAME="${FOLLOWER_REQUIRE_OBJECT_PLANE_FRAME:-false}"
PYBULLET_SHOW_KEYPOINT_OVERLAY="${PYBULLET_SHOW_KEYPOINT_OVERLAY:-false}"
PYBULLET_KEYPOINT_OVERLAY_TOPIC="${PYBULLET_KEYPOINT_OVERLAY_TOPIC:-/perception/keypoint_3d}"
PYBULLET_KEYPOINT_OVERLAY_VALID_TOPIC="${PYBULLET_KEYPOINT_OVERLAY_VALID_TOPIC:-/perception/valid}"
PYBULLET_KEYPOINT_OVERLAY_PLANE_NORMAL_TOPIC="${PYBULLET_KEYPOINT_OVERLAY_PLANE_NORMAL_TOPIC:-/perception/object_plane_normal}"
PYBULLET_KEYPOINT_OVERLAY_RADIUS_M="${PYBULLET_KEYPOINT_OVERLAY_RADIUS_M:-0.012}"
PYBULLET_KEYPOINT_OVERLAY_CROSS_SIZE_M="${PYBULLET_KEYPOINT_OVERLAY_CROSS_SIZE_M:-0.045}"
PYBULLET_KEYPOINT_OVERLAY_NORMAL_LENGTH_M="${PYBULLET_KEYPOINT_OVERLAY_NORMAL_LENGTH_M:-0.080}"
PYBULLET_KEYPOINT_OVERLAY_REQUIRE_VALID="${PYBULLET_KEYPOINT_OVERLAY_REQUIRE_VALID:-true}"
PYBULLET_KEYPOINT_OVERLAY_MAX_AGE_SEC="${PYBULLET_KEYPOINT_OVERLAY_MAX_AGE_SEC:-2.0}"
PYBULLET_KEYPOINT_OVERLAY_RAISE_M="${PYBULLET_KEYPOINT_OVERLAY_RAISE_M:-0.0}"
ENABLE_GMS_KEYPOINT_DEMO="${ENABLE_GMS_KEYPOINT_DEMO:-false}"
GMS_KEYPOINT_NUM_CLUSTERS="${GMS_KEYPOINT_NUM_CLUSTERS:-10}"
GMS_KEYPOINT_MAX_SAMPLES="${GMS_KEYPOINT_MAX_SAMPLES:-1800}"
GMS_KEYPOINT_PCA_DIM="${GMS_KEYPOINT_PCA_DIM:-5}"
GMS_KEYPOINT_XYZ_WEIGHT="${GMS_KEYPOINT_XYZ_WEIGHT:-1.0}"
GMS_KEYPOINT_RGB_WEIGHT="${GMS_KEYPOINT_RGB_WEIGHT:-0.35}"
GMS_KEYPOINT_UV_WEIGHT="${GMS_KEYPOINT_UV_WEIGHT:-0.20}"
GMS_KEYPOINT_MERGE_DISTANCE_M="${GMS_KEYPOINT_MERGE_DISTANCE_M:-0.025}"
GMS_KEYPOINT_MIN_CLUSTER_POINTS="${GMS_KEYPOINT_MIN_CLUSTER_POINTS:-8}"
GMS_KEYPOINT_MAX_CANDIDATES="${GMS_KEYPOINT_MAX_CANDIDATES:-12}"
GMS_KEYPOINT_SELECTED_INDEX="${GMS_KEYPOINT_SELECTED_INDEX:--1}"
GMS_KEYPOINT_PLANE_LOCAL_RADIUS_M="${GMS_KEYPOINT_PLANE_LOCAL_RADIUS_M:-0.18}"
GMS_KEYPOINT_PLANE_MIN_POINTS="${GMS_KEYPOINT_PLANE_MIN_POINTS:-24}"
GMS_KEYPOINT_PLANE_MAX_SAMPLES="${GMS_KEYPOINT_PLANE_MAX_SAMPLES:-1024}"
GMS_KEYPOINT_STABILIZER="${GMS_KEYPOINT_STABILIZER:-true}"
GMS_KEYPOINT_FILTER_ALPHA="${GMS_KEYPOINT_FILTER_ALPHA:-0.20}"
GMS_KEYPOINT_LOCK_RADIUS_M="${GMS_KEYPOINT_LOCK_RADIUS_M:-0.08}"
GMS_KEYPOINT_JUMP_RESET_M="${GMS_KEYPOINT_JUMP_RESET_M:-0.14}"
GMS_KEYPOINT_JUMP_HOLD_FRAMES="${GMS_KEYPOINT_JUMP_HOLD_FRAMES:-8}"

LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
mkdir -p "$LOG_DIR" "$PID_DIR" "$ROS_LOG_DIR"
SIM_EXTERNAL_SCENE_OBJECTS_FILE="${EXTERNAL_SCENE_OBJECTS_FILE:-__none__}"
if [[ "$EXTERNAL_SCENE_OBJECTS_JSON" != "[]" ]]; then
  SIM_EXTERNAL_SCENE_OBJECTS_FILE="$LOG_DIR/external_scene_objects.json"
  printf '%s\n' "$EXTERNAL_SCENE_OBJECTS_JSON" > "$SIM_EXTERNAL_SCENE_OBJECTS_FILE"
fi

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
err() { echo "[ERROR] $*" >&2; }

pidfile_for() { echo "$PID_DIR/$1.pid"; }
logfile_for() { echo "$LOG_DIR/$1.log"; }
qwen3_params_file() { echo "$LOG_DIR/llm_task_planner_qwen3_runtime.yaml"; }

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

is_truthy() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

is_staged_single_gpu_mode() {
  [[ "$LLM_BACKEND" == "qwen3_local" && "$GPU_EXECUTION_MODE" == "staged_single_gpu" ]]
}

should_start_gpu_monitor() {
  case "${GPU_MONITOR_WINDOW,,}" in
    1|true|yes|y|on) return 0 ;;
    0|false|no|n|off) return 1 ;;
    auto)
      [[ -n "${DISPLAY:-}" ]]
      return
      ;;
    *)
      warn "unknown GPU_MONITOR_WINDOW=$GPU_MONITOR_WINDOW; use auto, true, or false"
      return 1
      ;;
  esac
}

managed_proc_running() {
  local name="$1"
  local pidf
  pidf="$(pidfile_for "$name")"
  [[ -f "$pidf" ]] || return 1
  pid_alive "$(cat "$pidf" 2>/dev/null || true)"
}

stop_external_qwen3() {
  local url="http://127.0.0.1:${QWEN3_PORT}/health"

  if ! wait_http "$url" 2; then
    return 0
  fi

  info "stopping external Qwen3 service on 127.0.0.1:${QWEN3_PORT}"

  if command -v fuser >/dev/null 2>&1; then
    fuser -k -TERM "${QWEN3_PORT}/tcp" >/dev/null 2>&1 || true
    sleep 2
    if wait_http "$url" 1; then
      fuser -k -KILL "${QWEN3_PORT}/tcp" >/dev/null 2>&1 || true
      sleep 1
    fi
  elif command -v lsof >/dev/null 2>&1; then
    local pids
    pids="$(lsof -tiTCP:"${QWEN3_PORT}" -sTCP:LISTEN 2>/dev/null || true)"
    if [[ -z "$pids" ]]; then
      err "could not find the external Qwen3 listener on tcp/${QWEN3_PORT}"
      return 1
    fi
    kill $pids 2>/dev/null || true
    sleep 2
    if wait_http "$url" 1; then
      kill -9 $pids 2>/dev/null || true
      sleep 1
    fi
  else
    err "cannot stop the external Qwen3 service automatically: neither fuser nor lsof is available"
    return 1
  fi

  if wait_http "$url" 2; then
    err "external Qwen3 service is still running on 127.0.0.1:${QWEN3_PORT}"
    err "stop it manually, then re-run the task command"
    return 1
  fi

  info "external Qwen3 service has been stopped"
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

start_bg_qwen3() {
  local name="$1"; shift
  local log; log="$(logfile_for "$name")"
  local pidf; pidf="$(pidfile_for "$name")"

  if [[ ! -x "$QWEN3_VENV/bin/vllm" ]]; then
    err "Qwen3 vLLM binary not found: $QWEN3_VENV/bin/vllm"
    exit 1
  fi

  setsid bash -lc "
    export QWEN3_VLLM_BIN=\"$QWEN3_VENV/bin/vllm\"
    export QWEN_MODEL=\"$QWEN_MODEL\"
    export QWEN3_HOST=\"$QWEN3_HOST\"
    export QWEN3_PORT=\"$QWEN3_PORT\"
    export QWEN3_GPU_MEMORY_UTILIZATION=\"$QWEN3_GPU_MEMORY_UTILIZATION\"
    export QWEN3_MAX_MODEL_LEN=\"$QWEN3_MAX_MODEL_LEN\"
    echo \$\$ > \"$pidf\"
    exec bash \"$WS/scripts/start_qwen3_vllm.sh\"
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
    set -u 2>/dev/null || true
    echo \$\$ > \"$pidf\"
    exec $cmd
  " >"$log" 2>&1 &

  sleep 0.1
  local pid
  for _ in {1..30}; do
    pid="$(cat "$pidf" 2>/dev/null || true)"
    [[ -n "$pid" ]] && break
    sleep 0.1
  done
  info "started $name pid=${pid:-unknown}"
}

start_gpu_monitor_window() {
  if ! should_start_gpu_monitor; then
    if [[ "${GPU_MONITOR_WINDOW,,}" == "auto" && -z "${DISPLAY:-}" ]]; then
      info "GPU monitor window skipped because DISPLAY is not set"
    fi
    return 0
  fi

  if ! command -v nvidia-smi >/dev/null 2>&1; then
    warn "GPU monitor window skipped because nvidia-smi is not available"
    return 0
  fi

  if ! python3 -c "import tkinter" >/dev/null 2>&1; then
    warn "GPU monitor window skipped because Python tkinter is not available"
    return 0
  fi

  start_bg_ros gpu_monitor \
    "python3 \"$WS/scripts/gpu_node_monitor.py\" \
     --pid-dir \"$PID_DIR\" \
     --refresh-sec \"$GPU_MONITOR_REFRESH_SEC\""
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

wait_topic_or_proc_exit() {
  local topic="$1"
  local timeout_sec="$2"
  local proc_name="$3"
  local pidf
  pidf="$(pidfile_for "$proc_name")"
  local tries=$((timeout_sec * 10))
  for ((i=0; i<tries; i++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    if [[ -f "$pidf" ]] && ! managed_proc_running "$proc_name"; then
      return 2
    fi
    sleep 0.1
  done
  return 1
}

wait_http() {
  local url="$1"
  local timeout_sec="$2"
  local tries=$((timeout_sec * 2))
  for ((i=0; i<tries; i++)); do
    if curl -fsS "$url" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

warn_if_not_running() {
  local name="$1"
  local pidf
  pidf="$(pidfile_for "$name")"
  if [[ -f "$pidf" ]] && ! pid_alive "$(cat "$pidf" 2>/dev/null || true)"; then
    warn "$name exited early; check $(logfile_for "$name")"
  fi
}

wait_sam3_ready_or_fail() {
  local status=0
  wait_topic_or_proc_exit "$SAM3_ACTIVE_PROMPT_TOPIC" "$SAM3_READY_TIMEOUT_SEC" sam3 || status=$?
  if [[ "$status" -eq 2 ]]; then
    err "SAM3 exited before publishing $SAM3_ACTIVE_PROMPT_TOPIC; check $(logfile_for sam3)"
    return 1
  elif [[ "$status" -ne 0 ]]; then
    err "SAM3 did not publish $SAM3_ACTIVE_PROMPT_TOPIC within ${SAM3_READY_TIMEOUT_SEC}s"
    err "check $(logfile_for sam3)"
    return 1
  fi

  status=0
  wait_topic_or_proc_exit "$SAM3_MASK_TOPIC" "$SAM3_READY_TIMEOUT_SEC" sam3 || status=$?
  if [[ "$status" -eq 2 ]]; then
    err "SAM3 exited before publishing $SAM3_MASK_TOPIC; check $(logfile_for sam3)"
    return 1
  elif [[ "$status" -ne 0 ]]; then
    err "SAM3 did not publish $SAM3_MASK_TOPIC within ${SAM3_READY_TIMEOUT_SEC}s"
    err "check $(logfile_for sam3)"
    return 1
  fi
}

start_sam3_execution_node() {
  start_bg_sam3 sam3 \
    "python -m sam3_ros.sam3_mask_node --ros-args \
     -p image_topic:=$SAM3_IMAGE_TOPIC \
     -p prompt:=\"$SAM3_PROMPT\" \
     -p prompt_topic:=$SAM3_PROMPT_TOPIC \
     -p active_prompt_topic:=$SAM3_ACTIVE_PROMPT_TOPIC \
     -p device:=\"$1\" \
     -p infer_hz:=2.0 \
     -p max_side:=384 \
     -p score_th:=$SAM3_SCORE_TH \
     -p mask_th:=$SAM3_MASK_TH"
}

ensure_qwen3_ready() {
  if managed_proc_running qwen3; then
    return 0
  fi

  if wait_http "http://127.0.0.1:${QWEN3_PORT}/health" 2; then
    if is_staged_single_gpu_mode; then
      if is_truthy "$QWEN3_STOP_EXTERNAL"; then
        warn "staged_single_gpu mode detected an external Qwen3 service on 127.0.0.1:${QWEN3_PORT}"
        warn "QWEN3_STOP_EXTERNAL=true, so the script will stop it and relaunch a managed Qwen3 only for planning"
        stop_external_qwen3 || return 1
      else
        err "staged_single_gpu mode detected an external Qwen3 service on 127.0.0.1:${QWEN3_PORT}"
        err "stop that external service first so this script can free GPU memory after planning"
        err "or re-run with QWEN3_STOP_EXTERNAL=true to let this script stop it automatically"
        return 1
      fi
    fi
    if wait_http "http://127.0.0.1:${QWEN3_PORT}/health" 2; then
      info "detected existing local Qwen3 service on 127.0.0.1:${QWEN3_PORT}"
      return 0
    fi
  fi

  if ! is_truthy "$QWEN3_AUTO_START"; then
    err "LLM_BACKEND=qwen3_local but no local service is ready on 127.0.0.1:${QWEN3_PORT}"
    err "either start Qwen3 manually or set QWEN3_AUTO_START=true"
    return 1
  fi

  start_bg_qwen3 qwen3
  info "waiting for local Qwen3 service to become ready..."
  if ! wait_http "http://127.0.0.1:${QWEN3_PORT}/health" "$QWEN3_READY_TIMEOUT_SEC"; then
    err "local Qwen3 service did not become ready within ${QWEN3_READY_TIMEOUT_SEC}s"
    err "check $(logfile_for qwen3) for download or startup errors"
    return 1
  fi
}

write_qwen3_params_file() {
  cat >"$(qwen3_params_file)" <<EOF
llm_task_planner_node:
  ros__parameters:
    api_protocol: "chat_completions"
    api_base_url: "http://127.0.0.1:${QWEN3_PORT}/v1/chat/completions"
    api_key: "EMPTY"
    api_key_required: false
    model: "${QWEN_MODEL}"
    temperature: ${QWEN3_TEMPERATURE}
    top_p: ${QWEN3_TOP_P}
    max_output_tokens: ${QWEN3_MAX_OUTPUT_TOKENS}
    request_timeout_sec: 90.0
    extra_request_body_json: '${QWEN3_EXTRA_REQUEST_BODY_JSON}'
    open_vocabulary_targets: ${OPEN_VOCAB_TARGETS}
    enable_grasp_actions: ${ENABLE_GRASP_ACTIONS}
    allow_local_fallback: ${ALLOW_LOCAL_PLANNER_FALLBACK}
EOF
}

planner_cmd() {
  if [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
    write_qwen3_params_file
    cat <<EOF
ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
 --params-file "$(qwen3_params_file)" \
 -p open_vocabulary_targets:=$OPEN_VOCAB_TARGETS \
 -p enable_grasp_actions:=$ENABLE_GRASP_ACTIONS
EOF
  else
    cat <<EOF
ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
 -p model:="$OPENAI_MODEL" \
 -p reasoning_effort:="$OPENAI_REASONING_EFFORT" \
 -p open_vocabulary_targets:=$OPEN_VOCAB_TARGETS \
 -p enable_grasp_actions:=$ENABLE_GRASP_ACTIONS
EOF
  fi
}

show_status() {
  for name in sim sam3 fusion gms_keypoints registry keypoint_debug keypoint_viewer tracker bridge monitor gpu_monitor qwen3 planner executor; do
    if [[ "$name" == "qwen3" ]]; then
      local pidf_qwen3
      pidf_qwen3="$(pidfile_for qwen3)"
      if [[ -f "$pidf_qwen3" ]] && pid_alive "$(cat "$pidf_qwen3" 2>/dev/null || true)"; then
        echo "[OK] qwen3 pid=$(cat "$pidf_qwen3")"
      elif wait_http "http://127.0.0.1:${QWEN3_PORT}/health" 1; then
        echo "[OK] qwen3 external http://127.0.0.1:${QWEN3_PORT}"
      else
        echo "[--] qwen3 not running"
      fi
      continue
    fi
    local pidf
    pidf="$(pidfile_for "$name")"
    if [[ -f "$pidf" ]] && pid_alive "$(cat "$pidf" 2>/dev/null || true)"; then
      echo "[OK] $name pid=$(cat "$pidf")"
    else
      echo "[--] $name not running"
    fi
  done
}

run_task_shell() {
  source_ros
  python3 "$WS/scripts/llm_task_cli.py"
}

send_one_task() {
  local text="$1"
  source_ros
  python3 "$WS/scripts/llm_task_cli.py" --once "$text"
}

wait_for_execution_complete() {
  source_ros
  set +e
  python3 "$WS/scripts/llm_task_cli.py" \
    --listen-only \
    --wait-status-prefix "execution_complete:" \
    --fail-status-prefix "execution_failed:" \
    --timeout-sec "$EXECUTION_TIMEOUT_SEC"
  local execution_status=$?
  set -e
  return "$execution_status"
}

wait_for_keypoint_debug_ready() {
  source_ros
  set +e
  python3 "$WS/scripts/llm_task_cli.py" \
    --listen-only \
    --wait-status-prefix "debug_keypoint_valid:" \
    --fail-status-prefix "debug_failed:" \
    --timeout-sec "$KEYPOINT_DEBUG_TIMEOUT_SEC"
  local debug_status=$?
  set -e
  return "$debug_status"
}

instruction_subscriber_ready() {
  source_ros
  local topic_info
  topic_info="$(timeout 3 ros2 topic info /llm_task/instruction 2>/dev/null || true)"
  grep -qE 'Subscription count: [1-9][0-9]*' <<< "$topic_info"
}

task_stack_ready() {
  instruction_subscriber_ready || return 1
  if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
    managed_proc_running keypoint_debug || return 1
  fi
  return 0
}

ensure_task_stack_ready() {
  if task_stack_ready; then
    return 0
  fi

  if is_truthy "$KEYPOINT_DEBUG_ONLY" && instruction_subscriber_ready; then
    warn "a task planner is already running, but keypoint_debug is not active"
    warn "restarting the managed stack in keypoint-debug-only mode to isolate execution"
    "$WS/start_llm_rekep_demo.sh" stop >/dev/null 2>&1 || true
  fi

  if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
    info "ROS task stack is not running; starting simulation/planner/keypoint-debug first"
  else
    info "ROS task stack is not running; starting simulation/planner/executor first"
  fi
  "$WS/start_llm_rekep_demo.sh" start

  for _ in {1..80}; do
    if task_stack_ready; then
      return 0
    fi
    sleep 0.25
  done

  if managed_proc_running planner; then
    warn "planner is running but /llm_task/instruction subscriber is not visible yet; continuing with repeated publish"
    return 0
  fi

  err "planner did not subscribe to /llm_task/instruction after starting the stack"
  err "check $(logfile_for planner)"
  return 1
}

case "${1:-start}" in
  start)
    source_ros

    if [[ "$LLM_BACKEND" == "openai" ]]; then
      if [[ -z "${OPENAI_API_KEY:-}" ]]; then
        err "OPENAI_API_KEY is not set"
        exit 1
      fi
    elif [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
      if is_staged_single_gpu_mode; then
        info "staged_single_gpu mode enabled: planning and SAM3 execution will time-share one GPU"
      else
        ensure_qwen3_ready || exit 1
      fi
    else
      err "Unsupported LLM_BACKEND=$LLM_BACKEND"
      err "Use LLM_BACKEND=openai or LLM_BACKEND=qwen3_local"
      exit 1
    fi

    start_bg_ros sim \
      "ros2 run pybullet_ros2_sim iiwa_pybullet_rgbd_sim_node --ros-args \
       -p gui:=$SIM_GUI \
       -p scene_preset:=$SIM_SCENE_PRESET \
       -p enable_gripper:=$ENABLE_GRIPPER \
       -p gripper_model:=$GRIPPER_MODEL \
       -p medical_grasp_scissors_fixed:=$MEDICAL_GRASP_SCISSORS_FIXED \
       -p medical_grasp_scissors_graspable:=$MEDICAL_GRASP_SCISSORS_GRASPABLE \
       -p medical_sorting_tray_enabled:=$MEDICAL_SORTING_TRAY_ENABLED \
       -p medical_sorting_tray_offset_xy:=$MEDICAL_SORTING_TRAY_OFFSET_XY \
       -p medical_sorting_tray_size_xy:=$MEDICAL_SORTING_TRAY_SIZE_XY \
       -p medical_sorting_tray_wall_height_m:=$MEDICAL_SORTING_TRAY_WALL_HEIGHT_M \
       -p external_scene_objects_file:=$SIM_EXTERNAL_SCENE_OBJECTS_FILE \
       -p stabilize_resting_graspables:=$STABILIZE_RESTING_GRASPABLES \
       -p resting_table_clearance_m:=$RESTING_TABLE_CLEARANCE_M \
       -p resting_snap_margin_m:=$RESTING_SNAP_MARGIN_M \
       -p resting_linear_speed_th_mps:=$RESTING_LINEAR_SPEED_TH_MPS \
       -p resting_angular_speed_th_radps:=$RESTING_ANGULAR_SPEED_TH_RADPS \
       -p resting_reanchor_delay_sec:=$RESTING_REANCHOR_DELAY_SEC \
       -p resting_interaction_release_radius_m:=$RESTING_INTERACTION_RELEASE_RADIUS_M \
       -p gripper_grasp_radius_m:=$GRIPPER_GRASP_RADIUS_M \
       -p gripper_contact_distance_m:=$GRIPPER_CONTACT_DISTANCE_M \
       -p gripper_require_actual_contact:=$GRIPPER_REQUIRE_ACTUAL_CONTACT \
       -p gripper_grasp_mode:=$GRIPPER_GRASP_MODE \
       -p gripper_keypoint_snap_to_center:=$GRIPPER_KEYPOINT_SNAP_TO_CENTER \
       -p gripper_keypoint_snap_local_offset:=$GRIPPER_KEYPOINT_SNAP_LOCAL_OFFSET \
       -p gripper_physical_contact_guard:=$GRIPPER_PHYSICAL_CONTACT_GUARD \
       -p gripper_physical_preload_rad:=$GRIPPER_PHYSICAL_PRELOAD_RAD \
       -p gripper_physical_force:=$GRIPPER_PHYSICAL_FORCE \
       -p gripper_physical_position_gain:=$GRIPPER_PHYSICAL_POSITION_GAIN \
       -p gripper_physical_max_velocity:=$GRIPPER_PHYSICAL_MAX_VELOCITY \
       -p gripper_grasp_settle_sec:=$GRIPPER_GRASP_SETTLE_SEC \
       -p gripper_grasp_timeout_sec:=$GRIPPER_GRASP_TIMEOUT_SEC \
       -p gripper_auto_lift_height_m:=$GRIPPER_AUTO_LIFT_HEIGHT_M \
       -p gripper_auto_lift_speed_mps:=$GRIPPER_AUTO_LIFT_SPEED_MPS \
       -p gripper_auto_lower_speed_mps:=$GRIPPER_AUTO_LOWER_SPEED_MPS \
       -p show_keypoint_overlay:=$PYBULLET_SHOW_KEYPOINT_OVERLAY \
       -p keypoint_overlay_topic:=$PYBULLET_KEYPOINT_OVERLAY_TOPIC \
       -p keypoint_overlay_valid_topic:=$PYBULLET_KEYPOINT_OVERLAY_VALID_TOPIC \
       -p keypoint_overlay_plane_normal_topic:=$PYBULLET_KEYPOINT_OVERLAY_PLANE_NORMAL_TOPIC \
       -p keypoint_overlay_radius_m:=$PYBULLET_KEYPOINT_OVERLAY_RADIUS_M \
       -p keypoint_overlay_cross_size_m:=$PYBULLET_KEYPOINT_OVERLAY_CROSS_SIZE_M \
       -p keypoint_overlay_normal_length_m:=$PYBULLET_KEYPOINT_OVERLAY_NORMAL_LENGTH_M \
       -p keypoint_overlay_require_valid:=$PYBULLET_KEYPOINT_OVERLAY_REQUIRE_VALID \
       -p keypoint_overlay_max_age_sec:=$PYBULLET_KEYPOINT_OVERLAY_MAX_AGE_SEC \
       -p keypoint_overlay_raise_m:=$PYBULLET_KEYPOINT_OVERLAY_RAISE_M \
       -p cam_width:=320 -p cam_height:=240 -p color_hz:=8.0 -p points_hz:=1.0 \
       -p joint_state_hz:=60.0 -p position_force:=500.0"

    if ! wait_topic "$SAM3_IMAGE_TOPIC" 15; then
      warn "camera topic not ready yet; continuing anyway"
    fi

    if ! is_staged_single_gpu_mode; then
      start_sam3_execution_node "$SAM3_DEVICE"
      wait_sam3_ready_or_fail || exit 1
    else
      info "staged_single_gpu mode: deferring SAM3 startup until a plan is ready"
    fi

    start_bg_ros fusion \
      "ros2 run perception_geometry mask_depth_fusion_node --ros-args \
       -p mask_topic:=$SAM3_MASK_TOPIC \
       -p score_topic:=$SAM3_SCORE_TOPIC \
       -p min_score:=$FUSION_MIN_SCORE \
       -p cluster_count:=4 -p cluster_max_samples:=512 -p cluster_meanshift_bandwidth_m:=0.04"

    if is_truthy "$ENABLE_GMS_KEYPOINT_DEMO"; then
      start_bg_ros gms_keypoints \
        "ros2 run perception_geometry give_me_scissors_keypoint_node --ros-args \
         -p mask_topic:=$SAM3_MASK_TOPIC \
         -p color_topic:=$SAM3_IMAGE_TOPIC \
         -p score_topic:=$SAM3_SCORE_TOPIC \
         -p prompt_topic:=$SAM3_ACTIVE_PROMPT_TOPIC \
         -p min_score:=$FUSION_MIN_SCORE \
         -p num_clusters:=$GMS_KEYPOINT_NUM_CLUSTERS \
         -p max_samples:=$GMS_KEYPOINT_MAX_SAMPLES \
         -p pca_dim:=$GMS_KEYPOINT_PCA_DIM \
         -p xyz_weight:=$GMS_KEYPOINT_XYZ_WEIGHT \
         -p rgb_weight:=$GMS_KEYPOINT_RGB_WEIGHT \
         -p uv_weight:=$GMS_KEYPOINT_UV_WEIGHT \
         -p merge_distance_m:=$GMS_KEYPOINT_MERGE_DISTANCE_M \
         -p min_cluster_points:=$GMS_KEYPOINT_MIN_CLUSTER_POINTS \
         -p max_candidates:=$GMS_KEYPOINT_MAX_CANDIDATES \
         -p plane_local_radius_m:=$GMS_KEYPOINT_PLANE_LOCAL_RADIUS_M \
         -p plane_min_points:=$GMS_KEYPOINT_PLANE_MIN_POINTS \
         -p plane_max_samples:=$GMS_KEYPOINT_PLANE_MAX_SAMPLES \
         -p enable_selected_keypoint_stabilizer:=$GMS_KEYPOINT_STABILIZER \
         -p selected_keypoint_filter_alpha:=$GMS_KEYPOINT_FILTER_ALPHA \
         -p selected_keypoint_lock_radius_m:=$GMS_KEYPOINT_LOCK_RADIUS_M \
         -p selected_keypoint_jump_reset_m:=$GMS_KEYPOINT_JUMP_RESET_M \
         -p selected_keypoint_jump_hold_frames:=$GMS_KEYPOINT_JUMP_HOLD_FRAMES \
         -p selected_index:=$GMS_KEYPOINT_SELECTED_INDEX"
    fi

    start_bg_ros registry \
      "ros2 run pybullet_ros2_sim scene_object_registry_node --ros-args \
       -p prompt_topic:=$SAM3_ACTIVE_PROMPT_TOPIC \
       -p open_vocabulary_targets:=$OPEN_VOCAB_TARGETS \
       -p publish_hz:=2.0 -p observation_timeout_sec:=1.5 -p visible_timeout_sec:=2.0"

    start_bg_ros planner \
      "$(planner_cmd)"

    if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
      start_bg_ros keypoint_debug \
        "ros2 run pybullet_ros2_sim keypoint_debug_plan_player_node --ros-args \
         -p open_vocabulary_targets:=$OPEN_VOCAB_TARGETS \
         -p enable_grasp_actions:=$ENABLE_GRASP_ACTIONS \
         -p prompt_republish_sec:=$KEYPOINT_DEBUG_PROMPT_REPUBLISH_SEC \
         -p status_heartbeat_sec:=$KEYPOINT_DEBUG_STATUS_HEARTBEAT_SEC \
         -p keypoint_timeout_sec:=$TRACKER_TARGET_TIMEOUT_SEC \
         -p valid_topic:=$KEYPOINT_DEBUG_VALID_TOPIC \
         -p keypoint_topic:=$KEYPOINT_DEBUG_KEYPOINT_TOPIC \
         -p keypoint_px_topic:=$KEYPOINT_DEBUG_KEYPOINT_PX_TOPIC \
         -p plane_normal_topic:=$KEYPOINT_DEBUG_PLANE_NORMAL_TOPIC \
         -p plane_tangent_topic:=$KEYPOINT_DEBUG_PLANE_TANGENT_TOPIC \
         -p auto_advance_on_valid:=$KEYPOINT_DEBUG_AUTO_ADVANCE \
         -p auto_advance_hold_sec:=$KEYPOINT_DEBUG_AUTO_ADVANCE_HOLD_SEC"

      if is_truthy "$START_HOVER_FOLLOWER"; then
        start_bg_ros tracker \
          "ros2 run pybullet_ros2_sim iiwa_keypoint_tracker_node --ros-args \
           -p publish_hz:=60.0 -p max_joint_step_rad:=$FOLLOWER_MAX_JOINT_STEP_RAD \
           -p keypoint_topic:=$FOLLOWER_KEYPOINT_TOPIC \
           -p valid_topic:=$FOLLOWER_VALID_TOPIC \
           -p object_plane_normal_topic:=$FOLLOWER_PLANE_NORMAL_TOPIC \
           -p object_plane_tangent_topic:=$FOLLOWER_PLANE_TANGENT_TOPIC \
           -p target_timeout_sec:=$TRACKER_TARGET_TIMEOUT_SEC \
           -p lock_target_on_tracking_enable:=$TRACKER_LOCK_TARGET_ON_ENABLE \
           -p use_last_target_on_occlusion:=$TRACKER_USE_LAST_TARGET_ON_OCCLUSION \
           -p occluded_target_hold_sec:=$TRACKER_OCCLUDED_TARGET_HOLD_SEC \
           -p status_topic:=$FOLLOWER_STATUS_TOPIC \
           -p hover_offset_z:=$FOLLOWER_HOVER_OFFSET_Z \
           -p enable_topic:=$FOLLOWER_ENABLE_TOPIC \
           -p start_enabled:=$FOLLOWER_START_ENABLED \
           -p align_to_object_yaw:=$ALIGN_TO_OBJECT_YAW \
           -p grasp_yaw_offset_rad:=$GRASP_YAW_OFFSET_RAD \
           -p object_yaw_timeout_sec:=$OBJECT_YAW_TIMEOUT_SEC \
           -p align_hover_offset_margin_m:=$ALIGN_HOVER_OFFSET_MARGIN_M \
           -p use_explicit_grasp_frame:=$USE_EXPLICIT_GRASP_FRAME \
           -p use_object_plane_frame:=$USE_OBJECT_PLANE_FRAME \
           -p require_object_plane_frame:=$FOLLOWER_REQUIRE_OBJECT_PLANE_FRAME \
           -p object_plane_timeout_sec:=$OBJECT_PLANE_TIMEOUT_SEC \
           -p gripper_forward_down_angle_rad:=$GRIPPER_FORWARD_DOWN_ANGLE_RAD \
           -p keep_gripper_vertical_to_table:=$KEEP_GRIPPER_VERTICAL_TO_TABLE \
           -p track_gripper_center:=$TRACK_GRIPPER_CENTER \
           -p gripper_center_offset_link6:=[$GRIPPER_CENTER_OFFSET_LINK6_X,$GRIPPER_CENTER_OFFSET_LINK6_Y,$GRIPPER_CENTER_OFFSET_LINK6_Z] \
           -p min_gripper_center_z:=$MIN_GRIPPER_CENTER_Z \
           -p ik_use_nullspace_rest:=$IK_USE_NULLSPACE_REST \
           -p ik_nullspace_initial_weight:=$IK_NULLSPACE_INITIAL_WEIGHT \
           -p ik_joint_damping:=$IK_JOINT_DAMPING \
           -p static_override_keep_current_orientation:=$STATIC_OVERRIDE_KEEP_CURRENT_ORIENTATION"
      fi

      if is_truthy "$START_KEYPOINT_VIEWER"; then
        start_bg_ros keypoint_viewer \
          "ros2 run vla_rgbd_tools tracking_overlay_viewer_node --ros-args \
           -p color_topic:=$SAM3_IMAGE_TOPIC \
           -p depth_topic:=/sim/camera/aligned_depth_to_color/image_raw \
           -p camera_info_topic:=/sim/camera/color/camera_info \
           -p mask_topic:=$SAM3_MASK_TOPIC \
           -p points_topic:=$KEYPOINT_VIEWER_POINTS_TOPIC \
           -p overlay_topic:=$KEYPOINT_VIEWER_OVERLAY_TOPIC \
           -p prompt_topic:=$SAM3_PROMPT_TOPIC \
           -p score_topic:=$SAM3_SCORE_TOPIC \
           -p keypoint_topic:=$KEYPOINT_VIEWER_KEYPOINT_TOPIC \
           -p keypoint_px_topic:=/perception/keypoint_px \
           -p valid_topic:=$KEYPOINT_VIEWER_VALID_TOPIC \
           -p display_scale:=$KEYPOINT_VIEWER_DISPLAY_SCALE \
           -p status_panel_width:=$KEYPOINT_VIEWER_STATUS_PANEL_WIDTH \
           -p follow_window_resize:=$KEYPOINT_VIEWER_FOLLOW_WINDOW_RESIZE \
           -p prefer_overlay_image:=$KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE \
           -p show_cloud_window:=$KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW"
      fi
    else
      start_bg_ros tracker \
        "ros2 run pybullet_ros2_sim iiwa_keypoint_tracker_node --ros-args \
         -p publish_hz:=60.0 -p max_joint_step_rad:=0.06 \
         -p hover_offset_z:=$HOVER_OFFSET_Z \
         -p align_to_object_yaw:=$ALIGN_TO_OBJECT_YAW \
         -p grasp_yaw_offset_rad:=$GRASP_YAW_OFFSET_RAD \
         -p object_yaw_timeout_sec:=$OBJECT_YAW_TIMEOUT_SEC \
         -p align_hover_offset_margin_m:=$ALIGN_HOVER_OFFSET_MARGIN_M \
         -p use_explicit_grasp_frame:=$USE_EXPLICIT_GRASP_FRAME \
         -p use_object_plane_frame:=$USE_OBJECT_PLANE_FRAME \
         -p object_plane_timeout_sec:=$OBJECT_PLANE_TIMEOUT_SEC \
         -p gripper_forward_down_angle_rad:=$GRIPPER_FORWARD_DOWN_ANGLE_RAD \
         -p keep_gripper_vertical_to_table:=$KEEP_GRIPPER_VERTICAL_TO_TABLE \
         -p track_gripper_center:=$TRACK_GRIPPER_CENTER \
         -p gripper_center_offset_link6:=[$GRIPPER_CENTER_OFFSET_LINK6_X,$GRIPPER_CENTER_OFFSET_LINK6_Y,$GRIPPER_CENTER_OFFSET_LINK6_Z] \
         -p min_gripper_center_z:=$MIN_GRIPPER_CENTER_Z \
         -p ik_use_nullspace_rest:=$IK_USE_NULLSPACE_REST \
         -p ik_nullspace_initial_weight:=$IK_NULLSPACE_INITIAL_WEIGHT \
         -p ik_joint_damping:=$IK_JOINT_DAMPING \
         -p static_override_keep_current_orientation:=$STATIC_OVERRIDE_KEEP_CURRENT_ORIENTATION \
         -p target_timeout_sec:=$TRACKER_TARGET_TIMEOUT_SEC \
         -p lock_target_on_tracking_enable:=$TRACKER_LOCK_TARGET_ON_ENABLE \
         -p use_last_target_on_occlusion:=$TRACKER_USE_LAST_TARGET_ON_OCCLUSION \
         -p occluded_target_hold_sec:=$TRACKER_OCCLUDED_TARGET_HOLD_SEC \
         -p start_enabled:=false"

      start_bg_ros bridge \
        "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p rate_hz:=60.0"

      start_bg_ros monitor \
        "ros2 run robot_monitor robot_monitor"

      start_bg_ros executor \
        "ros2 run pybullet_ros2_sim llm_task_executor_node --ros-args \
         -p open_vocabulary_targets:=$OPEN_VOCAB_TARGETS \
         -p enable_grasp_actions:=$ENABLE_GRASP_ACTIONS \
         -p hover_offset_z:=$HOVER_OFFSET_Z \
         -p grasp_lift_m:=$GRASP_LIFT_M \
         -p grasp_pregrasp_offset_m:=$GRASP_PREGRASP_OFFSET_M \
         -p grasp_pregrasp_success_radius_m:=$GRASP_PREGRASP_SUCCESS_RADIUS_M \
         -p grasp_pregrasp_hold_sec:=$GRASP_PREGRASP_HOLD_SEC \
         -p grasp_approach_offset_z:=$GRASP_APPROACH_OFFSET_Z \
         -p grasp_lift_delay_sec:=$GRASP_LIFT_DELAY_SEC \
         -p grasp_confirm_hold_sec:=$GRASP_CONFIRM_HOLD_SEC \
         -p grasp_lift_hold_sec:=$GRASP_LIFT_HOLD_SEC \
         -p grasp_close_success_radius_m:=$GRASP_CLOSE_SUCCESS_RADIUS_M \
         -p grasp_lift_success_radius_m:=$GRASP_LIFT_SUCCESS_RADIUS_M \
         -p track_gripper_center:=$TRACK_GRIPPER_CENTER \
         -p gripper_center_offset_link6:=[$GRIPPER_CENTER_OFFSET_LINK6_X,$GRIPPER_CENTER_OFFSET_LINK6_Y,$GRIPPER_CENTER_OFFSET_LINK6_Z] \
         -p min_gripper_center_z:=$MIN_GRIPPER_CENTER_Z \
         -p use_static_tray_center_target:=$USE_STATIC_TRAY_CENTER_TARGET \
         -p static_tray_center_world:=$STATIC_TRAY_CENTER_WORLD \
         -p static_tray_plane_normal_world:=$STATIC_TRAY_PLANE_NORMAL_WORLD \
         -p place_hover_offset_z:=$PLACE_HOVER_OFFSET_Z \
         -p release_hover_offset_z:=$RELEASE_HOVER_OFFSET_Z \
         -p release_success_radius_m:=$RELEASE_SUCCESS_RADIUS_M \
         -p release_approach_timeout_sec:=$RELEASE_APPROACH_TIMEOUT_SEC \
         -p place_success_radius_m:=$PLACE_SUCCESS_RADIUS_M \
         -p release_lower_hold_sec:=$RELEASE_LOWER_HOLD_SEC \
         -p release_open_wait_sec:=$RELEASE_OPEN_WAIT_SEC \
         -p target_timeout_sec:=$TRACKER_TARGET_TIMEOUT_SEC \
         -p default_success_radius_m:=$EXECUTOR_SUCCESS_RADIUS_M \
         -p target_reacquire_delay_sec:=0.75"
    fi

    start_gpu_monitor_window

    sleep 2
    if ! is_staged_single_gpu_mode; then
      warn_if_not_running sam3
    fi
    warn_if_not_running fusion
    if is_truthy "$ENABLE_GMS_KEYPOINT_DEMO"; then
      warn_if_not_running gms_keypoints
    fi
    warn_if_not_running registry
    if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
      warn_if_not_running keypoint_debug
      if is_truthy "$START_HOVER_FOLLOWER"; then
        warn_if_not_running tracker
      fi
      if is_truthy "$START_KEYPOINT_VIEWER"; then
        warn_if_not_running keypoint_viewer
      fi
    else
      warn_if_not_running tracker
      warn_if_not_running executor
    fi
    warn_if_not_running gpu_monitor
    warn_if_not_running planner

    info "LLM tabletop demo stack started"
    info "keypoint_debug_only=$KEYPOINT_DEBUG_ONLY start_keypoint_viewer=$START_KEYPOINT_VIEWER debug_timeout_sec=$KEYPOINT_DEBUG_TIMEOUT_SEC auto_advance=$KEYPOINT_DEBUG_AUTO_ADVANCE"
    info "keypoint_debug_topics valid=$KEYPOINT_DEBUG_VALID_TOPIC keypoint=$KEYPOINT_DEBUG_KEYPOINT_TOPIC keypoint_px=$KEYPOINT_DEBUG_KEYPOINT_PX_TOPIC plane_normal=$KEYPOINT_DEBUG_PLANE_NORMAL_TOPIC plane_tangent=$KEYPOINT_DEBUG_PLANE_TANGENT_TOPIC"
    info "hover_follower=$START_HOVER_FOLLOWER hover_offset_z=$FOLLOWER_HOVER_OFFSET_Z require_plane=$FOLLOWER_REQUIRE_OBJECT_PLANE_FRAME keypoint_topic=$FOLLOWER_KEYPOINT_TOPIC valid_topic=$FOLLOWER_VALID_TOPIC plane_normal_topic=$FOLLOWER_PLANE_NORMAL_TOPIC plane_tangent_topic=$FOLLOWER_PLANE_TANGENT_TOPIC status_topic=$FOLLOWER_STATUS_TOPIC start_enabled=$FOLLOWER_START_ENABLED"
    info "keypoint_viewer display_scale=$KEYPOINT_VIEWER_DISPLAY_SCALE status_panel_width=$KEYPOINT_VIEWER_STATUS_PANEL_WIDTH follow_window_resize=$KEYPOINT_VIEWER_FOLLOW_WINDOW_RESIZE prefer_overlay_image=$KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE show_cloud_window=$KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW"
    info "sam3 prompt=$SAM3_PROMPT device=$SAM3_DEVICE venv=$SAM3_VENV"
    info "sam3 topics image=$SAM3_IMAGE_TOPIC prompt=$SAM3_PROMPT_TOPIC active=$SAM3_ACTIVE_PROMPT_TOPIC mask=$SAM3_MASK_TOPIC score=$SAM3_SCORE_TOPIC"
    info "sim gui=$SIM_GUI"
    info "sim scene_preset=$SIM_SCENE_PRESET open_vocab_targets=$OPEN_VOCAB_TARGETS enable_gripper=$ENABLE_GRIPPER enable_grasp_actions=$ENABLE_GRASP_ACTIONS"
    info "pybullet_keypoint_overlay=$PYBULLET_SHOW_KEYPOINT_OVERLAY topic=$PYBULLET_KEYPOINT_OVERLAY_TOPIC plane_normal_topic=$PYBULLET_KEYPOINT_OVERLAY_PLANE_NORMAL_TOPIC radius=$PYBULLET_KEYPOINT_OVERLAY_RADIUS_M cross_size=$PYBULLET_KEYPOINT_OVERLAY_CROSS_SIZE_M normal_length=$PYBULLET_KEYPOINT_OVERLAY_NORMAL_LENGTH_M require_valid=$PYBULLET_KEYPOINT_OVERLAY_REQUIRE_VALID"
    info "gms_keypoint_demo=$ENABLE_GMS_KEYPOINT_DEMO clusters=$GMS_KEYPOINT_NUM_CLUSTERS max_candidates=$GMS_KEYPOINT_MAX_CANDIDATES merge_distance=$GMS_KEYPOINT_MERGE_DISTANCE_M plane_radius=$GMS_KEYPOINT_PLANE_LOCAL_RADIUS_M selected_index=$GMS_KEYPOINT_SELECTED_INDEX stabilizer=$GMS_KEYPOINT_STABILIZER lock_radius=$GMS_KEYPOINT_LOCK_RADIUS_M filter_alpha=$GMS_KEYPOINT_FILTER_ALPHA"
    info "medical_grasp scissors_fixed=$MEDICAL_GRASP_SCISSORS_FIXED scissors_graspable=$MEDICAL_GRASP_SCISSORS_GRASPABLE"
    info "resting stabilization=$STABILIZE_RESTING_GRASPABLES table_clearance=$RESTING_TABLE_CLEARANCE_M snap_margin=$RESTING_SNAP_MARGIN_M reanchor_delay=$RESTING_REANCHOR_DELAY_SEC interaction_release_radius=$RESTING_INTERACTION_RELEASE_RADIUS_M"
    if [[ "$SIM_EXTERNAL_SCENE_OBJECTS_FILE" != "__none__" ]]; then
      info "external_scene_objects_file=$SIM_EXTERNAL_SCENE_OBJECTS_FILE"
    fi
    info "gripper_grasp_radius_m=$GRIPPER_GRASP_RADIUS_M contact_distance_m=$GRIPPER_CONTACT_DISTANCE_M require_actual_contact=$GRIPPER_REQUIRE_ACTUAL_CONTACT grasp_mode=$GRIPPER_GRASP_MODE keypoint_snap_to_center=$GRIPPER_KEYPOINT_SNAP_TO_CENTER keypoint_snap_local_offset=$GRIPPER_KEYPOINT_SNAP_LOCAL_OFFSET grasp_settle_sec=$GRIPPER_GRASP_SETTLE_SEC grasp_timeout_sec=$GRIPPER_GRASP_TIMEOUT_SEC"
    info "gripper_auto_lift_height_m=$GRIPPER_AUTO_LIFT_HEIGHT_M gripper_auto_lift_speed_mps=$GRIPPER_AUTO_LIFT_SPEED_MPS gripper_auto_lower_speed_mps=$GRIPPER_AUTO_LOWER_SPEED_MPS"
    info "track_gripper_center=$TRACK_GRIPPER_CENTER gripper_center_offset_link6=[$GRIPPER_CENTER_OFFSET_LINK6_X,$GRIPPER_CENTER_OFFSET_LINK6_Y,$GRIPPER_CENTER_OFFSET_LINK6_Z] min_gripper_center_z=$MIN_GRIPPER_CENTER_Z"
    info "ik_nullspace_rest=$IK_USE_NULLSPACE_REST initial_weight=$IK_NULLSPACE_INITIAL_WEIGHT joint_damping=$IK_JOINT_DAMPING static_override_keep_current_orientation=$STATIC_OVERRIDE_KEEP_CURRENT_ORIENTATION"
    info "grasp_lift_m=$GRASP_LIFT_M grasp_approach_offset_z=$GRASP_APPROACH_OFFSET_Z grasp_lift_delay_sec=$GRASP_LIFT_DELAY_SEC grasp_confirm_hold_sec=$GRASP_CONFIRM_HOLD_SEC grasp_lift_hold_sec=$GRASP_LIFT_HOLD_SEC grasp_close_success_radius_m=$GRASP_CLOSE_SUCCESS_RADIUS_M grasp_lift_success_radius_m=$GRASP_LIFT_SUCCESS_RADIUS_M"
    info "align_to_object_yaw=$ALIGN_TO_OBJECT_YAW grasp_yaw_offset_rad=$GRASP_YAW_OFFSET_RAD object_yaw_timeout_sec=$OBJECT_YAW_TIMEOUT_SEC align_hover_offset_margin_m=$ALIGN_HOVER_OFFSET_MARGIN_M use_explicit_grasp_frame=$USE_EXPLICIT_GRASP_FRAME gripper_forward_down_angle_rad=$GRIPPER_FORWARD_DOWN_ANGLE_RAD keep_gripper_vertical_to_table=$KEEP_GRIPPER_VERTICAL_TO_TABLE"
    info "release_lower_hold_sec=$RELEASE_LOWER_HOLD_SEC release_open_wait_sec=$RELEASE_OPEN_WAIT_SEC"
    info "gpu monitor window=$GPU_MONITOR_WINDOW refresh_sec=$GPU_MONITOR_REFRESH_SEC"
    info "hover_offset_z=$HOVER_OFFSET_Z tracker_target_timeout_sec=$TRACKER_TARGET_TIMEOUT_SEC target_lock=$TRACKER_LOCK_TARGET_ON_ENABLE occlusion_cache=$TRACKER_USE_LAST_TARGET_ON_OCCLUSION occlusion_hold_sec=$TRACKER_OCCLUDED_TARGET_HOLD_SEC executor_success_radius_m=$EXECUTOR_SUCCESS_RADIUS_M"
    info "release_hover_offset_z=$RELEASE_HOVER_OFFSET_Z release_success_radius_m=$RELEASE_SUCCESS_RADIUS_M release_approach_timeout_sec=$RELEASE_APPROACH_TIMEOUT_SEC place_success_radius_m=$PLACE_SUCCESS_RADIUS_M place_hover_offset_z=$PLACE_HOVER_OFFSET_Z release_lower_hold_sec=$RELEASE_LOWER_HOLD_SEC"
    info "static_tray_center_target=$USE_STATIC_TRAY_CENTER_TARGET center_world=$STATIC_TRAY_CENTER_WORLD plane_normal_world=$STATIC_TRAY_PLANE_NORMAL_WORLD"
    info "sam3 score_th=$SAM3_SCORE_TH mask_th=$SAM3_MASK_TH fusion_min_score=$FUSION_MIN_SCORE"
    info "llm backend=$LLM_BACKEND"
    info "gpu execution mode=$GPU_EXECUTION_MODE"
    if [[ "$LLM_BACKEND" == "openai" ]]; then
      info "openai model=$OPENAI_MODEL reasoning_effort=$OPENAI_REASONING_EFFORT"
    else
      info "qwen3 model=$QWEN_MODEL api=http://127.0.0.1:${QWEN3_PORT}/v1/chat/completions"
    fi
    info "Open another terminal and run:"
    echo "  ./start_llm_rekep_demo.sh shell"
    info "Or send one command directly:"
    echo "  ./start_llm_rekep_demo.sh task '依次移动到红色方块、蓝色方块和黄色方块上方'"
    ;;

  stop)
    stop_one executor
    stop_one keypoint_viewer
    stop_one keypoint_debug
    stop_one planner
    stop_one qwen3
    stop_one gpu_monitor
    stop_one monitor
    stop_one bridge
    stop_one tracker
    stop_one registry
    stop_one gms_keypoints
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
      "$(logfile_for gms_keypoints)" \
      "$(logfile_for registry)" \
      "$(logfile_for keypoint_debug)" \
      "$(logfile_for keypoint_viewer)" \
      "$(logfile_for tracker)" \
      "$(logfile_for bridge)" \
      "$(logfile_for monitor)" \
      "$(logfile_for gpu_monitor)" \
      "$(logfile_for qwen3)" \
      "$(logfile_for planner)" \
      "$(logfile_for executor)" 2>/dev/null || true
    ;;

  gpu-monitor)
    GPU_MONITOR_WINDOW=true
    start_gpu_monitor_window
    ;;

  gpu-snapshot)
    python3 "$WS/scripts/gpu_node_monitor.py" \
      --pid-dir "$PID_DIR" \
      --refresh-sec "$GPU_MONITOR_REFRESH_SEC" \
      --once
    ;;

  clean)
    "$WS/clean_demo_processes.sh"
    ;;

  stop-qwen3-external)
    stop_external_qwen3
    ;;

  shell|interactive)
    if is_staged_single_gpu_mode; then
      err "Interactive shell is not supported in staged_single_gpu mode."
      err "Use: ./start_llm_rekep_demo.sh task \"your instruction\""
      exit 1
    fi
    run_task_shell
    ;;

  task)
    shift || true
    if [[ $# -eq 0 ]]; then
      err "Usage: $0 task \"your instruction\""
      exit 1
    fi
    ensure_task_stack_ready || exit 1
    if is_staged_single_gpu_mode; then
      qwen3_ready=true

      if managed_proc_running sam3; then
        info "stopping SAM3 to free GPU memory for planning"
        stop_one sam3
      fi

      ensure_qwen3_ready || exit 1

      info "sending task to planner and waiting for a plan..."
      source_ros
      set +e
      python3 "$WS/scripts/llm_task_cli.py" \
        --once "$*" \
        --publish-warmup-sec 10 \
        --publish-repeat-sec 3 \
        --wait-status-prefix "planned:" \
        --wait-status-prefix "planned_fallback:" \
        --fail-status-prefix "planning_failed:" \
        --timeout-sec "$PLANNING_TIMEOUT_SEC"
      planning_status=$?
      set -e

      if managed_proc_running qwen3; then
        if [[ "$planning_status" -eq 0 ]]; then
          info "plan is ready; stopping Qwen3 to free GPU memory for SAM3"
        else
          warn "planning did not complete successfully; stopping Qwen3 before exiting"
        fi
        stop_one qwen3
      fi

      if [[ "$planning_status" -ne 0 ]]; then
        exit "$planning_status"
      fi

      info "switching execution backend: Qwen3 planning finished, launching SAM3 next"
      if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
        info "starting SAM3 on ${SAM3_EXEC_DEVICE} for keypoint debug"
      else
        info "starting SAM3 on ${SAM3_EXEC_DEVICE} for execution"
      fi
      start_sam3_execution_node "$SAM3_EXEC_DEVICE"
      wait_sam3_ready_or_fail || exit 1
      if managed_proc_running sam3; then
        if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
          info "keypoint debug phase is live; streaming /llm_task/status until a valid keypoint is published"
          wait_for_keypoint_debug_ready || exit $?
        else
          info "execution phase is live; streaming /llm_task/status until execution completes"
          wait_for_execution_complete || exit $?
        fi
      else
        err "SAM3 failed to start; check $(logfile_for sam3)"
        exit 1
      fi
    else
      send_one_task "$*"
      if is_truthy "$KEYPOINT_DEBUG_ONLY"; then
        wait_for_keypoint_debug_ready || exit $?
      fi
    fi
    ;;

  *)
    echo "Usage: $0 {start|stop|status|logs|gpu-monitor|gpu-snapshot|shell|interactive|task|clean|stop-qwen3-external}"
    echo "LLM backend env:"
    echo "  LLM_BACKEND=openai"
    echo "  LLM_BACKEND=qwen3_local"
    echo "  GPU_EXECUTION_MODE=concurrent"
    echo "  GPU_EXECUTION_MODE=staged_single_gpu"
    echo "Required env for OpenAI backend:"
    echo "  OPENAI_API_KEY=..."
    echo "Optional env:"
    echo "  OPENAI_MODEL=gpt-5-mini"
    echo "  OPENAI_REASONING_EFFORT=low"
    echo "  QWEN3_VENV=/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm"
    echo "  QWEN_MODEL=Qwen/Qwen3-4B"
    echo "  QWEN3_PORT=8000"
    echo "  QWEN3_AUTO_START=true"
    echo "  QWEN3_STOP_EXTERNAL=false"
    echo "  QWEN3_READY_TIMEOUT_SEC=900"
    echo "  QWEN3_GPU_MEMORY_UTILIZATION=0.88"
    echo "  QWEN3_MAX_MODEL_LEN=8192"
    echo "  QWEN3_MAX_OUTPUT_TOKENS=512"
    echo "  ALLOW_LOCAL_PLANNER_FALLBACK=true"
    echo "  PLANNING_TIMEOUT_SEC=180"
    echo "  EXECUTION_TIMEOUT_SEC=300"
    echo "  KEYPOINT_DEBUG_ONLY=false"
    echo "  KEYPOINT_DEBUG_TIMEOUT_SEC=120"
    echo "  KEYPOINT_DEBUG_KEYPOINT_TOPIC=/perception/keypoint_3d"
    echo "  KEYPOINT_DEBUG_KEYPOINT_PX_TOPIC=/perception/keypoint_px"
    echo "  KEYPOINT_DEBUG_VALID_TOPIC=/perception/valid"
    echo "  KEYPOINT_DEBUG_PLANE_NORMAL_TOPIC=/perception/object_plane_normal"
    echo "  KEYPOINT_DEBUG_PLANE_TANGENT_TOPIC=/perception/object_plane_tangent"
    echo "  START_KEYPOINT_VIEWER=false"
    echo "  KEYPOINT_VIEWER_DISPLAY_SCALE=2.0"
    echo "  KEYPOINT_VIEWER_STATUS_PANEL_WIDTH=360"
    echo "  KEYPOINT_VIEWER_FOLLOW_WINDOW_RESIZE=true"
    echo "  KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE=false"
    echo "  KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW=true"
    echo "  KEYPOINT_VIEWER_OVERLAY_TOPIC=/perception/keypoint_overlay"
    echo "  KEYPOINT_VIEWER_KEYPOINT_TOPIC=/perception/keypoint_3d"
    echo "  START_HOVER_FOLLOWER=false"
    echo "  FOLLOWER_KEYPOINT_TOPIC=/perception/keypoint_3d"
    echo "  FOLLOWER_VALID_TOPIC=/perception/valid"
    echo "  FOLLOWER_PLANE_NORMAL_TOPIC=/perception/object_plane_normal"
    echo "  FOLLOWER_PLANE_TANGENT_TOPIC=/perception/object_plane_tangent"
    echo "  FOLLOWER_STATUS_TOPIC=/iiwa7/keypoint_tracker_status"
    echo "  FOLLOWER_HOVER_OFFSET_Z=0.10"
    echo "  FOLLOWER_MAX_JOINT_STEP_RAD=0.04"
    echo "  FOLLOWER_REQUIRE_OBJECT_PLANE_FRAME=false"
    echo "  ENABLE_GMS_KEYPOINT_DEMO=false"
    echo "  GMS_KEYPOINT_NUM_CLUSTERS=10"
    echo "  GMS_KEYPOINT_MAX_CANDIDATES=12"
    echo "  GMS_KEYPOINT_PLANE_LOCAL_RADIUS_M=0.18"
    echo "  GMS_KEYPOINT_SELECTED_INDEX=-1"
    echo "  PYBULLET_SHOW_KEYPOINT_OVERLAY=false"
    echo "  PYBULLET_KEYPOINT_OVERLAY_TOPIC=/perception/keypoint_3d"
    echo "  PYBULLET_KEYPOINT_OVERLAY_PLANE_NORMAL_TOPIC=/perception/object_plane_normal"
    echo "  PYBULLET_KEYPOINT_OVERLAY_RADIUS_M=0.012"
    echo "  PYBULLET_KEYPOINT_OVERLAY_CROSS_SIZE_M=0.045"
    echo "  PYBULLET_KEYPOINT_OVERLAY_NORMAL_LENGTH_M=0.080"
    echo "  SAM3_READY_TIMEOUT_SEC=120"
    echo "  SAM3_VENV=$HOME/venvs/ros_vla"
    echo "  SAM3_PROMPT='red cube'"
    echo "  SAM3_DEVICE=cuda"
    echo "  SAM3_EXEC_DEVICE=cuda"
    echo "  SAM3_IMAGE_TOPIC=/sim/camera/color/image_raw"
    echo "  SAM3_PROMPT_TOPIC=/sam3/prompt"
    echo "  SAM3_ACTIVE_PROMPT_TOPIC=/sam3/active_prompt"
    echo "  SAM3_MASK_TOPIC=/sam3/mask"
    echo "  SAM3_SCORE_TOPIC=/sam3/score"
    echo "  SAM3_SCORE_TH=0.10"
    echo "  SAM3_MASK_TH=0.40"
    echo "  FUSION_MIN_SCORE=0.05"
    echo "  HOVER_OFFSET_Z=0.10"
    echo "  TRACKER_TARGET_TIMEOUT_SEC=4.0"
    echo "  EXECUTOR_SUCCESS_RADIUS_M=0.10"
    echo "  SIM_GUI=true"
    echo "  SIM_SCENE_PRESET=cubes|household|medical|medical_grasp|mixed|medical_mixed|empty"
    echo "  MEDICAL_GRASP_SCISSORS_FIXED=true"
    echo "  MEDICAL_GRASP_SCISSORS_GRASPABLE=false"
    echo "  EXTERNAL_SCENE_OBJECTS_FILE=/abs/scene_objects.json"
    echo "  EXTERNAL_SCENE_OBJECTS_JSON='[{\"name\":\"forceps\",\"type\":\"urdf\",\"path\":\"/abs/model.urdf\",\"position\":[0.05,-0.06,0.02],\"rpy\":[0,0,1.57],\"scale\":1.0,\"fixed\":false,\"graspable\":true}]'"
    echo "  OPEN_VOCAB_TARGETS=false"
    echo "  ENABLE_GRIPPER=false"
    echo "  ENABLE_GRASP_ACTIONS=false"
    echo "  GRIPPER_GRASP_RADIUS_M=0.13"
    echo "  GRIPPER_CONTACT_DISTANCE_M=0.012"
    echo "  GRIPPER_REQUIRE_ACTUAL_CONTACT=true"
    echo "  GPU_MONITOR_WINDOW=auto"
    echo "  GPU_MONITOR_REFRESH_SEC=1.0"
    echo "Examples:"
    echo "  OPENAI_API_KEY=... $0 start"
    echo "  LLM_BACKEND=qwen3_local $0 start"
    echo "  ./start_medical_llm_demo.sh task 'inspect the red entry point and then the curved needle'"
    echo "  ./start_medical_grasp_demo.sh task 'pick up the blue grasp handle and then release it'"
    echo "  ./start_medical_keypoint_debug.sh task 'pick up the blue grasp handle and then release it'"
    echo "  LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu $0 start"
    echo "  QWEN3_STOP_EXTERNAL=true LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu $0 task '依次移动到红色方块、蓝色方块和黄色方块上方'"
    echo "  LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu $0 task '依次移动到红色方块、蓝色方块和黄色方块上方'"
    echo "  $0 stop-qwen3-external"
    echo "  LLM_BACKEND=qwen3_local SAM3_DEVICE=cpu $0 start"
    echo "  SIM_GUI=false LLM_BACKEND=qwen3_local GPU_EXECUTION_MODE=staged_single_gpu $0 start"
    echo "  $0 clean"
    echo "  $0 gpu-monitor"
    echo "  $0 gpu-snapshot"
    echo "  $0 shell"
    # echo "  $0 task '依次移动到红色方块、蓝色方块和黄色方块上方'"
    exit 1
    ;;
esac
