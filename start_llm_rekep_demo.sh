#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
SAM3_PROMPT="${SAM3_PROMPT:-red cube}"
SAM3_DEVICE="${SAM3_DEVICE:-cuda}"
HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.10}"
SAM3_SCORE_TH="${SAM3_SCORE_TH:-0.10}"
SAM3_MASK_TH="${SAM3_MASK_TH:-0.40}"
FUSION_MIN_SCORE="${FUSION_MIN_SCORE:-0.05}"
TRACKER_TARGET_TIMEOUT_SEC="${TRACKER_TARGET_TIMEOUT_SEC:-2.0}"
SIM_GUI="${SIM_GUI:-true}"
GPU_MONITOR_WINDOW="${GPU_MONITOR_WINDOW:-auto}"
GPU_MONITOR_REFRESH_SEC="${GPU_MONITOR_REFRESH_SEC:-1.0}"

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
QWEN3_GPU_MEMORY_UTILIZATION="${QWEN3_GPU_MEMORY_UTILIZATION:-0.93}"
QWEN3_MAX_MODEL_LEN="${QWEN3_MAX_MODEL_LEN:-32768}"
QWEN3_TEMPERATURE="${QWEN3_TEMPERATURE:-0.7}"
QWEN3_TOP_P="${QWEN3_TOP_P:-0.8}"
QWEN3_MAX_OUTPUT_TOKENS="${QWEN3_MAX_OUTPUT_TOKENS:-512}"
QWEN3_EXTRA_REQUEST_BODY_JSON="${QWEN3_EXTRA_REQUEST_BODY_JSON:-{\"top_k\": 20, \"chat_template_kwargs\": {\"enable_thinking\": false}}}"
PLANNING_TIMEOUT_SEC="${PLANNING_TIMEOUT_SEC:-180}"
SAM3_EXEC_DEVICE="${SAM3_EXEC_DEVICE:-cuda}"

LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
mkdir -p "$LOG_DIR" "$PID_DIR"

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
  pid="$(cat "$pidf" 2>/dev/null || true)"
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

start_sam3_execution_node() {
  start_bg_sam3 sam3 \
    "python -m sam3_ros.sam3_mask_node --ros-args \
     -p image_topic:=/sim/camera/color/image_raw \
     -p prompt:=\"$SAM3_PROMPT\" \
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
EOF
}

planner_cmd() {
  if [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
    write_qwen3_params_file
    cat <<EOF
ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
 --params-file "$(qwen3_params_file)"
EOF
  else
    cat <<EOF
ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
 -p model:="$OPENAI_MODEL" \
 -p reasoning_effort:="$OPENAI_REASONING_EFFORT"
EOF
  fi
}

show_status() {
  for name in sim sam3 fusion registry tracker bridge monitor gpu_monitor qwen3 planner executor; do
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
       -p cam_width:=320 -p cam_height:=240 -p color_hz:=8.0 -p points_hz:=1.0 \
       -p joint_state_hz:=60.0 -p position_force:=500.0"

    if ! wait_topic "/sim/camera/color/image_raw" 15; then
      warn "camera topic not ready yet; continuing anyway"
    fi

    if ! is_staged_single_gpu_mode; then
      start_sam3_execution_node "$SAM3_DEVICE"
    else
      info "staged_single_gpu mode: deferring SAM3 startup until a plan is ready"
    fi

    start_bg_ros fusion \
      "ros2 run perception_geometry mask_depth_fusion_node --ros-args \
       -p min_score:=$FUSION_MIN_SCORE \
       -p cluster_count:=4 -p cluster_max_samples:=512 -p cluster_meanshift_bandwidth_m:=0.04"

    start_bg_ros registry \
      "ros2 run pybullet_ros2_sim scene_object_registry_node --ros-args \
       -p publish_hz:=2.0 -p observation_timeout_sec:=1.5 -p visible_timeout_sec:=2.0"

    start_bg_ros tracker \
      "ros2 run pybullet_ros2_sim iiwa_keypoint_tracker_node --ros-args \
       -p publish_hz:=60.0 -p max_joint_step_rad:=0.06 \
       -p hover_offset_z:=$HOVER_OFFSET_Z \
       -p target_timeout_sec:=$TRACKER_TARGET_TIMEOUT_SEC \
       -p start_enabled:=false"

    start_bg_ros bridge \
      "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p rate_hz:=60.0"

    start_bg_ros monitor \
      "ros2 run robot_monitor robot_monitor"

    start_bg_ros planner \
      "$(planner_cmd)"

    start_bg_ros executor \
      "ros2 run pybullet_ros2_sim llm_task_executor_node --ros-args \
       -p hover_offset_z:=$HOVER_OFFSET_Z \
       -p default_success_radius_m:=0.06 \
       -p target_reacquire_delay_sec:=0.75"

    start_gpu_monitor_window

    sleep 2
    if ! is_staged_single_gpu_mode; then
      warn_if_not_running sam3
    fi
    warn_if_not_running fusion
    warn_if_not_running registry
    warn_if_not_running tracker
    warn_if_not_running gpu_monitor
    warn_if_not_running planner
    warn_if_not_running executor

    info "LLM tabletop demo stack started"
    info "sam3 prompt=$SAM3_PROMPT device=$SAM3_DEVICE venv=$SAM3_VENV"
    info "sim gui=$SIM_GUI"
    info "gpu monitor window=$GPU_MONITOR_WINDOW refresh_sec=$GPU_MONITOR_REFRESH_SEC"
    info "hover_offset_z=$HOVER_OFFSET_Z tracker_target_timeout_sec=$TRACKER_TARGET_TIMEOUT_SEC"
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
    stop_one planner
    stop_one qwen3
    stop_one gpu_monitor
    stop_one monitor
    stop_one bridge
    stop_one tracker
    stop_one registry
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
      "$(logfile_for registry)" \
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
    if is_staged_single_gpu_mode; then
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

      info "starting SAM3 on ${SAM3_EXEC_DEVICE} for execution"
      start_sam3_execution_node "$SAM3_EXEC_DEVICE"
      sleep 2
      warn_if_not_running sam3
      if managed_proc_running sam3; then
        info "execution phase is live; waiting for perception to confirm the target"
      fi
    else
      send_one_task "$*"
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
    echo "  QWEN3_GPU_MEMORY_UTILIZATION=0.93"
    echo "  QWEN3_MAX_MODEL_LEN=32768"
    echo "  PLANNING_TIMEOUT_SEC=180"
    echo "  SAM3_VENV=$HOME/venvs/ros_vla"
    echo "  SAM3_PROMPT='red cube'"
    echo "  SAM3_DEVICE=cuda"
    echo "  SAM3_EXEC_DEVICE=cuda"
    echo "  SAM3_SCORE_TH=0.10"
    echo "  SAM3_MASK_TH=0.40"
    echo "  FUSION_MIN_SCORE=0.05"
    echo "  HOVER_OFFSET_Z=0.10"
    echo "  TRACKER_TARGET_TIMEOUT_SEC=2.0"
    echo "  SIM_GUI=true"
    echo "  GPU_MONITOR_WINDOW=auto"
    echo "  GPU_MONITOR_REFRESH_SEC=1.0"
    echo "Examples:"
    echo "  OPENAI_API_KEY=... $0 start"
    echo "  LLM_BACKEND=qwen3_local $0 start"
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
    echo "  $0 task '依次移动到红色方块、蓝色方块和黄色方块上方'"
    exit 1
    ;;
esac
