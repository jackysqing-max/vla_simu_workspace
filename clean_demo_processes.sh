#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"
PID_DIR="$WS/run_pids"

info() { echo "[INFO] $*"; }
warn() { echo "[WARN] $*" >&2; }

PATTERN="start_llm_rekep_demo|start_semantic_tracking_demo|start_rekep_demo|semantic_prompt_cli|iiwa_pybullet_rgbd_sim_node|iiwa_keypoint_tracker_node|llm_task_planner_node|llm_task_executor_node|sam3_mask_node|mask_depth_fusion_node|robotstate_bridge|robot_monitor|sam3_ros|pybullet_ros2_sim|perception_geometry"

source_ros() {
  set +u
  source "$ROS_SETUP"
  if [[ -f "$WS_SETUP" ]]; then
    source "$WS_SETUP"
  fi
  set -u 2>/dev/null || true
}

kill_matches() {
  local pattern="$1"
  local pids=""
  pids="$(pgrep -f "$pattern" || true)"
  [[ -n "$pids" ]] || return 0

  info "killing pattern: $pattern"
  for sig in INT TERM KILL; do
    while read -r pid; do
      [[ -n "$pid" ]] || continue
      kill "-$sig" "$pid" 2>/dev/null || true
    done <<< "$pids"
    sleep 1
    pids="$(pgrep -f "$pattern" || true)"
    [[ -z "$pids" ]] && return 0
  done

  warn "some processes may still remain for pattern: $pattern"
}

cleanup_pid_files() {
  if [[ -d "$PID_DIR" ]]; then
    find "$PID_DIR" -type f -name '*.pid' -delete
    info "removed pid files in $PID_DIR"
  fi
}

stop_ros_daemon() {
  source_ros
  ros2 daemon stop >/dev/null 2>&1 || true
  info "stopped ros2 daemon"
}

show_remaining() {
  if command -v rg >/dev/null 2>&1; then
    ps -ef | rg "$PATTERN" | grep -v " rg " || true
  else
    ps -ef | grep -E "$PATTERN" | grep -v "grep -E" || true
  fi
}

main() {
  kill_matches "start_llm_rekep_demo.sh"
  kill_matches "start_semantic_tracking_demo.sh"
  kill_matches "start_rekep_demo.sh"
  kill_matches "semantic_prompt_cli.py"
  kill_matches "python -m sam3_ros.sam3_mask_node"
  kill_matches "sam3_mask_node"
  kill_matches "iiwa_pybullet_rgbd_sim_node"
  kill_matches "iiwa_keypoint_tracker_node"
  kill_matches "llm_task_planner_node"
  kill_matches "llm_task_executor_node"
  kill_matches "mask_depth_fusion_node"
  kill_matches "robotstate_bridge"
  kill_matches "robot_monitor"
  kill_matches "ros2 run pybullet_ros2_sim"
  kill_matches "ros2 run perception_geometry"
  kill_matches "ros2 run iiwa_state_udp_bridge"
  kill_matches "ros2 run robot_monitor"

  cleanup_pid_files
  stop_ros_daemon

  info "remaining matching processes:"
  show_remaining
}

main "$@"
