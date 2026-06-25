#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
PARAMS_FILE="$LOG_DIR/llm_recording_planner.yaml"

LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
QWEN3_VENV="${QWEN3_VENV:-/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm}"
QWEN_MODEL="${QWEN_MODEL:-Qwen/Qwen3-4B}"
QWEN3_HOST="${QWEN3_HOST:-127.0.0.1}"
QWEN3_PORT="${QWEN3_PORT:-8000}"
QWEN3_AUTO_START="${QWEN3_AUTO_START:-false}"
QWEN3_READY_TIMEOUT_SEC="${QWEN3_READY_TIMEOUT_SEC:-180}"
QWEN3_GPU_MEMORY_UTILIZATION="${QWEN3_GPU_MEMORY_UTILIZATION:-0.88}"
QWEN3_MAX_MODEL_LEN="${QWEN3_MAX_MODEL_LEN:-8192}"
QWEN3_TEMPERATURE="${QWEN3_TEMPERATURE:-0.0}"
QWEN3_TOP_P="${QWEN3_TOP_P:-1.0}"
QWEN3_MAX_OUTPUT_TOKENS="${QWEN3_MAX_OUTPUT_TOKENS:-512}"
QWEN3_EXTRA_REQUEST_BODY_JSON="${QWEN3_EXTRA_REQUEST_BODY_JSON:-{\"top_k\": 20, \"chat_template_kwargs\": {\"enable_thinking\": false}}}"
ALLOW_LOCAL_PLANNER_FALLBACK="${ALLOW_LOCAL_PLANNER_FALLBACK:-true}"

mkdir -p "$LOG_DIR" "$PID_DIR" "$LOG_DIR/ros"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }
err() { echo "[ERROR] $*" >&2; }

is_truthy() {
  case "${1,,}" in
    1|true|yes|y|on) return 0 ;;
    *) return 1 ;;
  esac
}

pid_alive() {
  local pid="$1"
  [[ -n "$pid" ]] || return 1
  kill -0 "$pid" 2>/dev/null
}

wait_http() {
  local url="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if curl -fsS "$url" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

source_ros() {
  set +u
  source "$ROS_SETUP"
  source "$WS_SETUP"
  set -u 2>/dev/null || true
}

start_qwen3_if_needed() {
  local health_url="http://${QWEN3_HOST}:${QWEN3_PORT}/health"
  if wait_http "$health_url" 2; then
    info "using existing Qwen3 service: http://${QWEN3_HOST}:${QWEN3_PORT}/v1/chat/completions"
    return 0
  fi

  if ! is_truthy "$QWEN3_AUTO_START"; then
    warn "Qwen3 is not ready on ${health_url}; using fast deterministic local planner fallback for this LLM-only recording."
    warn "For a live Qwen3 clip, run: QWEN3_AUTO_START=true LLM_BACKEND=qwen3_local ./recording_scripts/01a_llm_prompt_stack_start.sh"
    LLM_BACKEND="local_fallback"
    return 0
  fi

  if [[ ! -x "$QWEN3_VENV/bin/vllm" ]]; then
    err "Qwen3 vLLM binary not found: $QWEN3_VENV/bin/vllm"
    exit 1
  fi

  local pidf="$PID_DIR/qwen3.pid"
  local log="$LOG_DIR/qwen3.log"
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

  sleep 0.2
  info "started qwen3 pid=$(cat "$pidf" 2>/dev/null || echo unknown)"
  info "waiting for local Qwen3 service to become ready..."
  if ! wait_http "$health_url" "$QWEN3_READY_TIMEOUT_SEC"; then
    err "local Qwen3 service did not become ready within ${QWEN3_READY_TIMEOUT_SEC}s"
    err "check $log"
    exit 1
  fi
}

write_params_file() {
  if [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
    cat >"$PARAMS_FILE" <<EOF
llm_task_planner_node:
  ros__parameters:
    api_protocol: "chat_completions"
    api_base_url: "http://${QWEN3_HOST}:${QWEN3_PORT}/v1/chat/completions"
    api_key: "EMPTY"
    api_key_env_var: ""
    api_key_required: false
    model: "${QWEN_MODEL}"
    temperature: ${QWEN3_TEMPERATURE}
    top_p: ${QWEN3_TOP_P}
    max_output_tokens: ${QWEN3_MAX_OUTPUT_TOKENS}
    request_timeout_sec: 90.0
    extra_request_body_json: '${QWEN3_EXTRA_REQUEST_BODY_JSON}'
    open_vocabulary_targets: true
    enable_grasp_actions: true
    allow_local_fallback: ${ALLOW_LOCAL_PLANNER_FALLBACK}
EOF
    return 0
  fi

  cat >"$PARAMS_FILE" <<EOF
llm_task_planner_node:
  ros__parameters:
    api_protocol: "chat_completions"
    api_base_url: "http://127.0.0.1:9/v1/chat/completions"
    api_key: "EMPTY"
    api_key_env_var: ""
    api_key_required: false
    model: "local-fallback"
    temperature: 0.0
    top_p: 1.0
    max_output_tokens: 512
    request_timeout_sec: 0.2
    extra_request_body_json: "{}"
    open_vocabulary_targets: true
    enable_grasp_actions: true
    allow_local_fallback: true
EOF
}

start_planner() {
  local pidf="$PID_DIR/planner.pid"
  local log="$LOG_DIR/planner.log"
  local existing_pid
  existing_pid="$(cat "$pidf" 2>/dev/null || true)"
  if pid_alive "$existing_pid"; then
    info "planner already running pid=$existing_pid"
    return 0
  fi

  setsid bash -lc "
    set +u
    source \"$ROS_SETUP\"
    source \"$WS_SETUP\"
    set -u 2>/dev/null || true
    echo \$\$ > \"$pidf\"
    exec ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args --params-file \"$PARAMS_FILE\"
  " >"$log" 2>&1 &

  sleep 0.5
  info "started llm_task_planner_node pid=$(cat "$pidf" 2>/dev/null || echo unknown)"
}

main() {
  cd "$WS"
  source_ros

  if [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
    start_qwen3_if_needed
  fi
  write_params_file
  start_planner

  cat <<EOF

LLM-only recording stack is ready.

This script starts only:
  - /llm_task/instruction subscriber
  - /llm_task/plan_json publisher
  - /llm_task/status publisher

It does not start PyBullet, SAM3, GMS, or any visualization window.

Recommended recording terminals:

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /llm_task/status

  cd $WS
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 topic echo /llm_task/plan_json

Send and pretty-print a task:

  cd $WS/docs
  ./recording_scripts/01b_llm_prompt_send_task.sh \\
    "pick up the left silver surgical instrument and place it into the tray center"

EOF
}

main "$@"
