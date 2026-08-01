#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_PARENT="$(dirname "$WS")"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
MODE_FILE="$PID_DIR/llm_vlm_rcm_mode.txt"

LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
GPU_EXECUTION_MODE="${GPU_EXECUTION_MODE:-staged_single_gpu}"
QWEN3_AUTO_START="${QWEN3_AUTO_START:-true}"
QWEN3_VENV="${QWEN3_VENV:-$WORKSPACE_PARENT/.venvs/qwen3-vllm}"
QWEN_MODEL="${QWEN_MODEL:-Qwen/Qwen3-4B}"
QWEN3_PORT="${QWEN3_PORT:-8000}"
QWEN3_READY_TIMEOUT_SEC="${QWEN3_READY_TIMEOUT_SEC:-240}"
QWEN3_GPU_MEMORY_UTILIZATION="${QWEN3_GPU_MEMORY_UTILIZATION:-0.60}"
QWEN3_MAX_MODEL_LEN="${QWEN3_MAX_MODEL_LEN:-4096}"
RCM_PREINSERT_CLEARANCE_M="${RCM_PREINSERT_CLEARANCE_M:-0.040}"
RCM_SPEED_MPS="${RCM_SPEED_MPS:-0.018}"
RCM_TRAJECTORY_RADIUS_M="${RCM_TRAJECTORY_RADIUS_M:-0.020}"
RCM_TOOL_LENGTH_M="${RCM_TOOL_LENGTH_M:-0.220}"
RCM_LAMBDA="${RCM_LAMBDA:-0.650}"
RCM_METRICS_CSV="${RCM_METRICS_CSV:-$LOG_DIR/llm_vlm_rcm_metrics.csv}"
RCM_VISUAL_PORT_MAX_OFFSET_M="${RCM_VISUAL_PORT_MAX_OFFSET_M:-0.0}"

DEFAULT_TASK="locate the circular laparoscopic port, align the surgical tool with the insertion axis, insert through the port while establishing the RCM constraint, then execute a circular trajectory while keeping the RCM point fixed"

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
  local attempt
  local pid

  for attempt in 1 2; do
    setsid bash -lc "
      set +u
      source /opt/ros/humble/setup.bash
      source '$WS/install/setup.bash'
      set -u 2>/dev/null || true
      exec $command
    " >"$log_file" 2>&1 &
    pid="$!"
    echo "$pid" >"$pid_file"
    echo "[START] $name pid=$pid log=$log_file"
    sleep 0.5
    if pid_alive "$pid"; then
      return 0
    fi
    echo "[WARN] $name exited during startup; retrying ($attempt/2)" >&2
  done

  echo "[ERROR] $name could not be started; check $log_file" >&2
  tail -n 20 "$log_file" >&2 || true
  return 1
}

http_ready() {
  curl -fsS "http://127.0.0.1:${QWEN3_PORT}/health" >/dev/null 2>&1
}

start_qwen3_if_needed() {
  if http_ready; then
    echo "[INFO] using existing Qwen3 service on port $QWEN3_PORT"
    return
  fi
  if [[ "$QWEN3_AUTO_START" != "true" ]]; then
    echo "[ERROR] Qwen3 is unavailable and QWEN3_AUTO_START is not true" >&2
    exit 1
  fi
  if [[ ! -x "$QWEN3_VENV/bin/vllm" ]]; then
    echo "[ERROR] Qwen3 vLLM environment not found: $QWEN3_VENV" >&2
    exit 1
  fi

  local name="llm_vlm_rcm_qwen3"
  local pid_file="$PID_DIR/${name}.pid"
  local log_file="$LOG_DIR/${name}.log"
  setsid bash -lc "
    source '$QWEN3_VENV/bin/activate'
    export QWEN_MODEL='$QWEN_MODEL'
    export QWEN3_HOST=127.0.0.1
    export QWEN3_PORT='$QWEN3_PORT'
    export QWEN3_GPU_MEMORY_UTILIZATION='$QWEN3_GPU_MEMORY_UTILIZATION'
    export QWEN3_MAX_MODEL_LEN='$QWEN3_MAX_MODEL_LEN'
    exec bash '$WS/scripts/start_qwen3_vllm.sh'
  " >"$log_file" 2>&1 &
  echo "$!" >"$pid_file"
  echo "[START] $name pid=$! log=$log_file"

  local attempts=$((QWEN3_READY_TIMEOUT_SEC * 2))
  local index
  for ((index = 0; index < attempts; index++)); do
    if http_ready; then
      echo "[INFO] Qwen3 service is ready"
      return
    fi
    if ! pid_alive "$(cat "$pid_file" 2>/dev/null || true)"; then
      echo "[ERROR] Qwen3 exited before becoming ready; check $log_file" >&2
      exit 1
    fi
    sleep 0.5
  done
  echo "[ERROR] Qwen3 did not become ready within ${QWEN3_READY_TIMEOUT_SEC}s" >&2
  exit 1
}

stop_owned_processes() {
  local keep_qwen="${1:-false}"
  local pid_file
  for pid_file in "$PID_DIR"/llm_vlm_rcm_*.pid; do
    [[ -f "$pid_file" ]] || continue
    if [[
      "$keep_qwen" == "true"
      && "$(basename "$pid_file")" == "llm_vlm_rcm_qwen3.pid"
    ]]; then
      continue
    fi
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

stop_named_process() {
  local name="$1"
  local pid_file="$PID_DIR/${name}.pid"
  [[ -f "$pid_file" ]] || return 0
  local pid
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  if pid_alive "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    sleep 0.7
  fi
  if pid_alive "$pid"; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    sleep 0.5
  fi
  rm -f "$pid_file"
}

start_planner() {
  if [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
    start_bg_ros llm_vlm_rcm_planner \
      "ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
       --params-file '$WS/install/pybullet_ros2_sim/share/pybullet_ros2_sim/config/llm_task_planner_qwen3_rcm.yaml' \
       -p api_base_url:=http://127.0.0.1:$QWEN3_PORT/v1/chat/completions \
       -p model:='$QWEN_MODEL'"
  else
    start_bg_ros llm_vlm_rcm_planner \
      "ros2 run pybullet_ros2_sim llm_task_planner_node --ros-args \
       -p api_protocol:=chat_completions \
       -p api_base_url:=http://127.0.0.1:1/v1/chat/completions \
       -p api_key_required:=false \
       -p request_timeout_sec:=0.2 \
       -p open_vocabulary_targets:=true \
       -p enable_rcm_actions:=true \
       -p allow_local_fallback:=true"
  fi
}

start_executor() {
  start_bg_ros llm_vlm_rcm_executor \
    "ros2 run rcm_virtual_fixtures surgical_rcm_task_executor_node --ros-args \
     -p trajectory_speed_mps:=$RCM_SPEED_MPS"
}

ensure_planning_nodes() {
  local planner_pid=""
  local executor_pid=""
  [[ -f "$PID_DIR/llm_vlm_rcm_planner.pid" ]] \
    && planner_pid="$(cat "$PID_DIR/llm_vlm_rcm_planner.pid" 2>/dev/null || true)"
  [[ -f "$PID_DIR/llm_vlm_rcm_executor.pid" ]] \
    && executor_pid="$(cat "$PID_DIR/llm_vlm_rcm_executor.pid" 2>/dev/null || true)"

  if ! pid_alive "$planner_pid"; then
    if [[ "$LLM_BACKEND" == "qwen3_local" ]] && ! http_ready; then
      echo "[ERROR] Qwen3 and the planner are not running; run '$0 start' first" >&2
      return 1
    fi
    echo "[WARN] planner was not running; restarting it"
    start_planner
  fi
  if ! pid_alive "$executor_pid"; then
    echo "[WARN] task executor was not running; restarting it"
    start_executor
  fi
  sleep 1
}

start_execution_runtime() {
  VLM_RCM_SHOW_TOOL=true \
  VLM_RCM_SHOW_DVRK_LND=true \
  VLM_RCM_TOOL_LENGTH_M="$RCM_TOOL_LENGTH_M" \
    ./start_vlm_rcm_port_perception_demo.sh start

  start_bg_ros llm_vlm_rcm_controller \
    "ros2 run rcm_virtual_fixtures rcm_virtual_fixture_node --ros-args \
     -p fixture_mode:=rcm \
     -p publish_hz:=100.0 \
     -p speed_mps:=$RCM_SPEED_MPS \
     -p trajectory_radius_m:=$RCM_TRAJECTORY_RADIUS_M \
     -p tool_length_m:=$RCM_TOOL_LENGTH_M \
     -p rcm_lambda:=$RCM_LAMBDA \
     -p use_initial_rcm:=false \
     -p use_vlm_port_pose:=true \
     -p visual_port_point_topic:=/vlm_rcm/locked_port_point \
     -p visual_port_axis_topic:=/vlm_rcm/locked_port_axis \
     -p visual_port_ready_topic:=/vlm_rcm/port_ready \
     -p visual_port_max_offset_m:=$RCM_VISUAL_PORT_MAX_OFFSET_M \
     -p visual_port_reference_axis:='[0.335067,0.0,-0.942194]' \
     -p visual_port_max_axis_angle_deg:=12.0 \
     -p preinsert_clearance_m:=$RCM_PREINSERT_CLEARANCE_M \
     -p wait_for_start_command:=true \
     -p wait_for_pivot_command:=true \
     -p enable_safe_insertion:=true \
     -p preinsert_clearance_m:=0.040 \
     -p port_standoff_m:=0.012 \
     -p preinsert_hold_sec:=1.0 \
     -p approach_duration_sec:=2.0 \
     -p port_dwell_sec:=0.8 \
     -p insertion_speed_mps:=0.018 \
     -p inserted_dwell_sec:=0.8 \
     -p stage_position_tolerance_m:=0.006 \
     -p stage_settle_cycles:=12 \
     -p max_joint_step_rad:=0.018 \
     -p record_metrics_csv:=true \
     -p metrics_csv_path:='$RCM_METRICS_CSV'"
}

show_status() {
  ./start_vlm_rcm_port_perception_demo.sh status
  local name
  for name in \
    llm_vlm_rcm_qwen3 \
    llm_vlm_rcm_planner \
    llm_vlm_rcm_executor \
    llm_vlm_rcm_controller; do
    local pid_file="$PID_DIR/${name}.pid"
    local pid=""
    [[ -f "$pid_file" ]] && pid="$(cat "$pid_file" 2>/dev/null || true)"
    if pid_alive "$pid"; then
      echo "[OK] $name pid=$pid"
    elif [[ "$name" == "llm_vlm_rcm_qwen3" ]] && http_ready; then
      echo "[OK] external Qwen3 service on port $QWEN3_PORT"
    else
      echo "[--] $name not running"
    fi
  done
}

case "${1:-start}" in
  start)
    source_env
    printf '%s\n%s\n' "$LLM_BACKEND" "$GPU_EXECUTION_MODE" >"$MODE_FILE"
    if [[ "$LLM_BACKEND" == "qwen3_local" ]] && http_ready; then
      stop_owned_processes true >/dev/null 2>&1 || true
    else
      stop_owned_processes false >/dev/null 2>&1 || true
    fi
    ./start_vlm_rcm_port_perception_demo.sh stop >/dev/null 2>&1 || true

    if [[ "$LLM_BACKEND" == "qwen3_local" ]]; then
      start_qwen3_if_needed
    elif [[ "$LLM_BACKEND" != "local_fallback" ]]; then
      echo "[ERROR] LLM_BACKEND must be qwen3_local or local_fallback" >&2
      exit 2
    fi

    start_planner
    start_executor
    if [[
      "$LLM_BACKEND" != "qwen3_local"
      || "$GPU_EXECUTION_MODE" != "staged_single_gpu"
    ]]; then
      start_execution_runtime
    fi

    echo
    echo "[OK] LLM + VLM + RCM planning stack is ready."
    echo "Send the default task:"
    echo "  ./start_llm_vlm_rcm_demo.sh task"
    if [[
      "$LLM_BACKEND" == "qwen3_local"
      && "$GPU_EXECUTION_MODE" == "staged_single_gpu"
    ]]; then
      echo "After planning, Qwen3 will stop and the SAM3/RCM runtime will start automatically."
    fi
    echo
    echo "Monitor:"
    echo "  ros2 topic echo /llm_task/plan_json"
    echo "  ros2 topic echo /surgical_rcm/status"
    echo "  ros2 topic echo /rcm_virtual_fixtures/stage"
    ;;
  task)
    shift || true
    source_env
    if [[ -f "$MODE_FILE" ]]; then
      mapfile -t saved_mode <"$MODE_FILE"
      LLM_BACKEND="${saved_mode[0]:-$LLM_BACKEND}"
      GPU_EXECUTION_MODE="${saved_mode[1]:-$GPU_EXECUTION_MODE}"
    fi
    TASK="${*:-$DEFAULT_TASK}"
    ensure_planning_nodes
    if ! python3 "$WS/docs/recording_scripts/llm_plan_capture.py" \
      "$TASK" \
      --timeout-sec 240; then
      if [[
        "$LLM_BACKEND" == "qwen3_local"
        && "$GPU_EXECUTION_MODE" == "staged_single_gpu"
      ]]; then
        stop_named_process llm_vlm_rcm_planner
        stop_named_process llm_vlm_rcm_qwen3
      fi
      exit 1
    fi
    if [[
      "$LLM_BACKEND" == "qwen3_local"
      && "$GPU_EXECUTION_MODE" == "staged_single_gpu"
    ]]; then
      sleep 0.8
      stop_named_process llm_vlm_rcm_planner
      stop_named_process llm_vlm_rcm_qwen3
      sleep 2
      echo "[INFO] planning complete; GPU released for SAM3"
      start_execution_runtime
      echo "[OK] execution runtime started from the retained task plan"
    fi
    ;;
  locate)
    shift || true
    if [[ $# -eq 0 ]]; then
      echo "Usage: $0 locate \"定位phantom上左上角的孔\"" >&2
      exit 2
    fi
    source_env
    stop_owned_processes false >/dev/null 2>&1 || true
    VLM_RCM_SHOW_TOOL=false \
    VLM_RCM_SHOW_DVRK_LND=false \
      ./start_vlm_rcm_port_perception_demo.sh start
    ./start_vlm_rcm_port_perception_demo.sh locate "$@"
    ;;
  stop)
    source_env >/dev/null 2>&1 || true
    stop_owned_processes
    ./start_vlm_rcm_port_perception_demo.sh stop
    rm -f "$MODE_FILE"
    echo "[OK] LLM + VLM + RCM demo stopped."
    ;;
  status)
    show_status
    ;;
  logs)
    tail -n 100 -f \
      "$LOG_DIR/llm_vlm_rcm_planner.log" \
      "$LOG_DIR/llm_vlm_rcm_executor.log" \
      "$LOG_DIR/llm_vlm_rcm_controller.log" \
      "$LOG_DIR/vlm_rcm_port_pose.log" \
      "$LOG_DIR/vlm_rcm_port_sam3.log"
    ;;
  *)
    echo "Usage: $0 {start|task [instruction]|locate \"text\"|stop|status|logs}"
    exit 2
    ;;
esac
