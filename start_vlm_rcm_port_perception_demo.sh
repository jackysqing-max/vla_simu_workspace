#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"

SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
SAM3_DEVICE="${SAM3_DEVICE:-cuda}"
SAM3_PROMPT="${SAM3_PROMPT:-}"
SAM3_INFER_HZ="${SAM3_INFER_HZ:-1.0}"
SAM3_MAX_SIDE="${SAM3_MAX_SIDE:-640}"
SAM3_SCORE_TH="${SAM3_SCORE_TH:-0.05}"
SAM3_MASK_TH="${SAM3_MASK_TH:-0.35}"
SAM3_MIN_FREE_GPU_MIB="${SAM3_MIN_FREE_GPU_MIB:-5500}"

VLM_RCM_GUI="${VLM_RCM_GUI:-true}"
VLM_RCM_DISPLAY_OVERLAY="${VLM_RCM_DISPLAY_OVERLAY:-true}"
VLM_RCM_DISPLAY_SCALE="${VLM_RCM_DISPLAY_SCALE:-1.25}"
VLM_RCM_RGBD_HZ="${VLM_RCM_RGBD_HZ:-4.0}"
VLM_RCM_CAMERA_DISTANCE_M="${VLM_RCM_CAMERA_DISTANCE_M:-0.34}"
VLM_RCM_CAMERA_YAW_DEG="${VLM_RCM_CAMERA_YAW_DEG:-90.0}"
VLM_RCM_CAMERA_PITCH_DEG="${VLM_RCM_CAMERA_PITCH_DEG:--70.0}"
VLM_RCM_SHOW_TOOL="${VLM_RCM_SHOW_TOOL:-false}"
VLM_RCM_SHOW_DVRK_LND="${VLM_RCM_SHOW_DVRK_LND:-false}"
VLM_RCM_TOOL_LENGTH_M="${VLM_RCM_TOOL_LENGTH_M:-0.220}"
VLM_RCM_TOOL_RADIUS_M="${VLM_RCM_TOOL_RADIUS_M:-0.006}"
VLM_RCM_TRACK_FORCE="${VLM_RCM_TRACK_FORCE:-260.0}"
VLM_RCM_TRACK_POS_GAIN="${VLM_RCM_TRACK_POS_GAIN:-0.65}"
VLM_RCM_TRACK_MAX_VEL="${VLM_RCM_TRACK_MAX_VEL:-6.0}"
VLM_RCM_EXPECTED_X_M="${VLM_RCM_EXPECTED_X_M:-0.701726}"
VLM_RCM_EXPECTED_Y_M="${VLM_RCM_EXPECTED_Y_M:-0.0}"
VLM_RCM_EXPECTED_Z_M="${VLM_RCM_EXPECTED_Z_M:-0.404701}"
VLM_RCM_AXIS_MODE="${VLM_RCM_AXIS_MODE:-calibrated}"
VLM_RCM_START_SEMANTIC_GROUNDER="${VLM_RCM_START_SEMANTIC_GROUNDER:-true}"
QWEN3_PORT="${QWEN3_PORT:-8000}"
QWEN_VL_MODEL="${QWEN_VL_MODEL:-${QWEN_MODEL:-Qwen/Qwen3-4B}}"
QWEN_VL_API_BASE_URL="${QWEN_VL_API_BASE_URL:-http://127.0.0.1:${QWEN3_PORT}/v1/chat/completions}"
QWEN_VL_MODELS_URL="${QWEN_VL_MODELS_URL:-${QWEN_VL_API_BASE_URL%/chat/completions}/models}"
QWEN_VL_API_KEY="${QWEN_VL_API_KEY:-EMPTY}"
QWEN_VL_INPUT_MODE="${QWEN_VL_INPUT_MODE:-text}"
QWEN_VL_ENABLE_LOCAL_FALLBACK="${QWEN_VL_ENABLE_LOCAL_FALLBACK:-false}"
QWEN_VL_RETRY_TEXT_WITHOUT_IMAGES="${QWEN_VL_RETRY_TEXT_WITHOUT_IMAGES:-true}"

PHANTOM_MESH="${PHANTOM_MESH:-$WS/install/rcm_virtual_fixtures/share/rcm_virtual_fixtures/meshes/phantom_multi.STL}"
if [[ ! -f "$PHANTOM_MESH" ]]; then
  PHANTOM_MESH="$WS/src/rcm_virtual_fixtures/meshes/phantom_multi.STL"
fi
DVRK_LND_URDF="${DVRK_LND_URDF:-$WS/install/rcm_virtual_fixtures/share/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf}"
if [[ ! -f "$DVRK_LND_URDF" ]]; then
  DVRK_LND_URDF="$WS/src/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf"
fi

mkdir -p "$LOG_DIR" "$PID_DIR"

source_env() {
  set +u
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  set -u 2>/dev/null || true
}

pid_alive() {
  local pid="$1"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

start_bg_ros() {
  local name="$1"
  shift
  local command="$*"
  local pid_file="$PID_DIR/${name}.pid"
  local log_file="$LOG_DIR/${name}.log"

  setsid bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '$WS/install/setup.bash'
    set -u 2>/dev/null || true
    exec $command
  " >"$log_file" 2>&1 &
  echo "$!" >"$pid_file"
  echo "[START] $name pid=$! log=$log_file"
}

start_bg_sam3() {
  local name="vlm_rcm_port_sam3"
  local pid_file="$PID_DIR/${name}.pid"
  local log_file="$LOG_DIR/${name}.log"

  if [[ ! -x "$SAM3_VENV/bin/python" ]]; then
    echo "[ERROR] SAM3 virtual environment not found: $SAM3_VENV" >&2
    exit 1
  fi

  if [[ "$SAM3_DEVICE" == "cuda" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    local free_gpu_mib
    free_gpu_mib="$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits \
      | head -n 1 | tr -d '[:space:]')"
    if [[ "$free_gpu_mib" =~ ^[0-9]+$ ]] && (( free_gpu_mib < SAM3_MIN_FREE_GPU_MIB )); then
      echo "[ERROR] refusing to start SAM3: only ${free_gpu_mib} MiB GPU memory is free; " \
           "at least ${SAM3_MIN_FREE_GPU_MIB} MiB is required." >&2
      echo "        Use staged execution, a quantized/CPU Qwen backend, or free more VRAM; " \
           "QWEN3_MAX_MODEL_LEN should remain 1792." >&2
      exit 1
    fi
  fi

  setsid bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '$WS/install/setup.bash'
    source '$SAM3_VENV/bin/activate'
    export PYTORCH_ALLOC_CONF=expandable_segments:True
    set -u 2>/dev/null || true
    exec python -m sam3_ros.sam3_mask_node --ros-args \
      -p image_topic:=/sim/camera/color/image_raw \
      -p \"prompt:='$SAM3_PROMPT'\" \
      -p device:='$SAM3_DEVICE' \
      -p infer_hz:=$SAM3_INFER_HZ \
      -p max_side:=$SAM3_MAX_SIDE \
      -p score_th:=$SAM3_SCORE_TH \
      -p mask_th:=$SAM3_MASK_TH
  " >"$log_file" 2>&1 &
  echo "$!" >"$pid_file"
  echo "[START] $name pid=$! log=$log_file"
}

wait_for_topic() {
  local topic="$1"
  local attempts="${2:-150}"
  local index
  for ((index = 0; index < attempts; index++)); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

wait_for_image_frame() {
  local topic="$1"
  local timeout_sec="${2:-20}"
  timeout "${timeout_sec}s" ros2 topic echo \
    --once \
    --qos-reliability best_effort \
    "$topic" >/dev/null 2>&1
}

stop_all() {
  local pid_file
  for pid_file in "$PID_DIR"/vlm_rcm_port_*.pid; do
    [[ -f "$pid_file" ]] || continue
    local pid
    pid="$(cat "$pid_file" 2>/dev/null || true)"
    if pid_alive "$pid"; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
      sleep 0.5
    fi
    if pid_alive "$pid"; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
    rm -f "$pid_file"
  done
}

show_status() {
  local name
  for name in vlm_rcm_port_sim vlm_rcm_port_pose vlm_rcm_port_semantic vlm_rcm_port_sam3; do
    local pid_file="$PID_DIR/${name}.pid"
    local pid=""
    [[ -f "$pid_file" ]] && pid="$(cat "$pid_file" 2>/dev/null || true)"
    if pid_alive "$pid"; then
      echo "[OK] $name pid=$pid"
    else
      echo "[--] $name not running"
    fi
  done
}

case "${1:-start}" in
  start)
    source_env
    stop_all >/dev/null 2>&1 || true
    startup_complete=false
    cleanup_failed_start() {
      if [[ "$startup_complete" != "true" ]]; then
        echo "[ERROR] perception startup failed; cleaning partial processes" >&2
        stop_all >/dev/null 2>&1 || true
      fi
    }
    trap cleanup_failed_start EXIT

    if [[ "$VLM_RCM_START_SEMANTIC_GROUNDER" == "true" ]]; then
      if [[ "$QWEN_VL_INPUT_MODE" == "local" || "$QWEN_VL_INPUT_MODE" == "rules" ]]; then
        echo "[ERROR] strict grounding forbids QWEN_VL_INPUT_MODE=$QWEN_VL_INPUT_MODE" >&2
        exit 2
      fi
      if [[ "$QWEN_VL_ENABLE_LOCAL_FALLBACK" != "false" ]]; then
        echo "[ERROR] strict grounding requires QWEN_VL_ENABLE_LOCAL_FALLBACK=false" >&2
        exit 2
      fi
      if ! curl -fsS --max-time 5 "$QWEN_VL_MODELS_URL" >/dev/null 2>&1; then
        echo "[ERROR] strict grounding requires a live model backend: $QWEN_VL_MODELS_URL" >&2
        echo "        No local/rule fallback will be used." >&2
        exit 1
      fi
    fi

    start_bg_ros vlm_rcm_port_sim \
      "env __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
       ros2 run pybullet_ros2_sim iiwa_pybullet_sim_node --ros-args \
       -p gui:=$VLM_RCM_GUI \
       -p use_goto:=true \
       -p init_q:='[0.0,0.462931412,0.0,-1.132967496,0.0,1.204016934,0.0]' \
       -p initial_control_mode:=1 \
       -p reset_to_init_on_start:=true \
       -p goto_duration:=0.5 \
       -p goto_use_reset:=true \
       -p goto_force:=$VLM_RCM_TRACK_FORCE \
       -p goto_pos_gain:=$VLM_RCM_TRACK_POS_GAIN \
       -p goto_vel_gain:=1.0 \
       -p goto_max_vel:=$VLM_RCM_TRACK_MAX_VEL \
       -p track_force:=$VLM_RCM_TRACK_FORCE \
       -p track_pos_gain:=$VLM_RCM_TRACK_POS_GAIN \
       -p track_vel_gain:=1.0 \
       -p track_max_vel:=$VLM_RCM_TRACK_MAX_VEL \
       -p table_x_m:=0.9 \
       -p table_y_m:=0.0 \
       -p table_top_z_m:=0.240701 \
       -p clean_gui:=true \
       -p camera_distance_m:=0.82 \
       -p camera_yaw_deg:=42.0 \
       -p camera_pitch_deg:=-38.0 \
       -p show_rcm_debug_markers:=true \
       -p show_port_detection_overlay:=true \
       -p locked_port_point_topic:=/vlm_rcm/locked_port_point \
       -p locked_port_axis_topic:=/vlm_rcm/locked_port_axis \
       -p locked_surface_axis_topic:=/vlm_rcm/locked_surface_axis \
       -p port_candidates_topic:=/vlm_rcm/hole_candidates \
       -p port_overlay_ring_radius_m:=0.010 \
       -p port_overlay_axis_outside_m:=0.045 \
       -p port_overlay_axis_inside_m:=0.085 \
       -p show_rcm_tool:=$VLM_RCM_SHOW_TOOL \
       -p rcm_tool_length_m:=$VLM_RCM_TOOL_LENGTH_M \
       -p rcm_tool_radius_m:=$VLM_RCM_TOOL_RADIUS_M \
       -p show_dvrk_lnd_gripper:=$VLM_RCM_SHOW_DVRK_LND \
       -p dvrk_lnd_urdf_path:='$DVRK_LND_URDF' \
       -p show_rcm_phantom:=true \
       -p rcm_phantom_mesh_path:='$PHANTOM_MESH' \
       -p rcm_phantom_collision:=false \
       -p rcm_phantom_alpha:=1.0 \
       -p rcm_phantom_x_m:=0.701726 \
       -p rcm_phantom_y_m:=0.0 \
       -p rcm_phantom_z_m:=0.240701 \
       -p rcm_phantom_roll_deg:=0.0 \
       -p rcm_phantom_pitch_deg:=0.0 \
       -p rcm_phantom_yaw_deg:=0.0 \
       -p enable_rgbd_camera:=true \
       -p rgbd_hz:=$VLM_RCM_RGBD_HZ \
       -p rgbd_width:=640 \
       -p rgbd_height:=480 \
       -p rgbd_fov_y_deg:=52.0 \
       -p rgbd_target:='[$VLM_RCM_EXPECTED_X_M,$VLM_RCM_EXPECTED_Y_M,$VLM_RCM_EXPECTED_Z_M]' \
       -p rgbd_distance_m:=$VLM_RCM_CAMERA_DISTANCE_M \
       -p rgbd_yaw_deg:=$VLM_RCM_CAMERA_YAW_DEG \
       -p rgbd_pitch_deg:=$VLM_RCM_CAMERA_PITCH_DEG"

    if ! wait_for_topic /sim/camera/color/image_raw; then
      echo "[ERROR] RGB-D camera topic did not become ready" >&2
      stop_all
      exit 1
    fi
    if ! wait_for_image_frame /sim/camera/color/image_raw 25; then
      echo "[ERROR] RGB-D camera topic exists but no image frame was published" >&2
      echo "        PyBullet GUI/render initialization did not complete." >&2
      stop_all
      exit 1
    fi

    start_bg_ros vlm_rcm_port_pose \
       "ros2 run rcm_virtual_fixtures vlm_port_pose_node --ros-args \
       -p expected_port_world:='[$VLM_RCM_EXPECTED_X_M,$VLM_RCM_EXPECTED_Y_M,$VLM_RCM_EXPECTED_Z_M]' \
       -p expected_port_max_distance_m:=0.0 \
       -p normal_reference:='[0.0,0.0,1.0]' \
       -p plane_max_tilt_deg:=20.0 \
       -p axis_mode:=$VLM_RCM_AXIS_MODE \
       -p calibrated_inward_axis:='[0.335067,0.0,-0.942194]' \
       -p annulus_radius_px:=24 \
       -p hole_search_radius_px:=40 \
       -p hole_min_depth_m:=0.025 \
       -p hole_min_area_px:=80 \
	       -p min_mask_area_px:=25 \
	       -p min_score:=0.03 \
	       -p language_control_topic:=/vlm_rcm/language_control \
	       -p require_language_instruction:=true \
	       -p stable_frames:=3 \
	       -p stability_window:=5 \
	       -p max_center_spread_m:=0.006 \
	       -p max_axis_spread_deg:=6.0 \
	       -p display_overlay:=$VLM_RCM_DISPLAY_OVERLAY \
	       -p display_scale:=$VLM_RCM_DISPLAY_SCALE"

	    if [[ "$VLM_RCM_START_SEMANTIC_GROUNDER" == "true" ]]; then
	      start_bg_ros vlm_rcm_port_semantic \
	        "ros2 run rcm_virtual_fixtures semantic_port_grounder_node --ros-args \
	         -p api_base_url:='$QWEN_VL_API_BASE_URL' \
	         -p api_key:='$QWEN_VL_API_KEY' \
	         -p api_key_required:=false \
	         -p model:='$QWEN_VL_MODEL' \
	         -p temperature:=0.0 \
	         -p top_p:=1.0 \
	         -p max_output_tokens:=256 \
	         -p request_timeout_sec:=90.0 \
	         -p input_mode:='$QWEN_VL_INPUT_MODE' \
	         -p enable_local_fallback:=$QWEN_VL_ENABLE_LOCAL_FALLBACK \
	         -p retry_text_without_images:=$QWEN_VL_RETRY_TEXT_WITHOUT_IMAGES \
	         -p sam_prompt_topic:=/sam3/prompt \
	         -p language_control_topic:=/vlm_rcm/language_control \
	         -p publish_sam_prompt_on_instruction:=true \
	         -p require_post_instruction_candidates:=true \
	         -p require_scoring_program:=true \
	         -p min_model_margin:=0.08 \
	         -p min_expression_margin:=0.05"
	    fi

	    start_bg_sam3
	    startup_complete=true
	    trap - EXIT

	    echo
	    echo "[OK] SAM3 VLM-RCM port perception demo started."
	    if [[ -n "$SAM3_PROMPT" ]]; then
	      echo "     startup prompt: $SAM3_PROMPT"
	    else
	      echo "     startup prompt: <idle until language instruction>"
	    fi
	    echo "     semantic grounder: $VLM_RCM_START_SEMANTIC_GROUNDER model=$QWEN_VL_MODEL"
	    echo "     semantic input: $QWEN_VL_INPUT_MODE, strict model only (no local fallback)"
	    echo "     PyBullet: yellow sphere = exact locked point; green spheres = candidates; cyan arrow = inward axis"
	    echo "     Camera window: green = SAM3 mask, IDs = dynamic candidates"
	    echo
	    echo "Monitor:"
	    echo "  ros2 topic echo /vlm_rcm/status"
	    echo "  ros2 topic echo /vlm_rcm/semantic_status"
	    echo "  ros2 topic echo /vlm_rcm/language_control"
	    echo "  ros2 topic echo /vlm_rcm/verification_result"
	    echo "  ros2 topic echo /vlm_rcm/axis_latency"
	    echo "  ros2 topic echo /vlm_rcm/locked_port_point"
	    echo "  ros2 topic echo /vlm_rcm/locked_port_axis"
	    echo "  ros2 topic echo /vlm_rcm/locked_surface_axis"
    ;;
  stop)
    source_env >/dev/null 2>&1 || true
    stop_all
    echo "[OK] SAM3 VLM-RCM port perception demo stopped."
    ;;
  status)
    show_status
    ;;
	  logs)
	    tail -n 100 -f \
	      "$LOG_DIR/vlm_rcm_port_sim.log" \
	      "$LOG_DIR/vlm_rcm_port_pose.log" \
	      "$LOG_DIR/vlm_rcm_port_semantic.log" \
	      "$LOG_DIR/vlm_rcm_port_sam3.log"
    ;;
  prompt)
    shift || true
    if [[ $# -eq 0 ]]; then
      echo "Usage: $0 prompt \"new SAM3 prompt\"" >&2
      exit 2
    fi
    source_env
    ros2 topic pub --once /sam3/prompt std_msgs/msg/String "{data: '$*'}"
    ;;
  locate)
    shift || true
    if [[ $# -eq 0 ]]; then
      echo "Usage: $0 locate \"定位phantom上左上角的孔\"" >&2
      exit 2
    fi
    source_env
    python3 "$WS/docs/recording_scripts/vlm_rcm_locate_capture.py" \
      "$@"
    ;;
  unlock)
    source_env
    python3 "$WS/docs/recording_scripts/vlm_rcm_locate_capture.py" \
      "解锁当前RCM点"
    ;;
  *)
    echo "Usage: $0 {start|stop|status|logs|prompt \"text\"|locate \"text\"|unlock}"
    exit 2
    ;;
esac
