#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"

RCM_FIXTURE_MODE="${RCM_FIXTURE_MODE:-rcm}"
RCM_GUI="${RCM_GUI:-true}"
RCM_PUBLISH_HZ="${RCM_PUBLISH_HZ:-100.0}"
RCM_SPEED_MPS="${RCM_SPEED_MPS:-0.018}"
RCM_TRAJECTORY_RADIUS_M="${RCM_TRAJECTORY_RADIUS_M:-0.020}"
RCM_TRAJECTORY_LENGTH_M="${RCM_TRAJECTORY_LENGTH_M:-0.100}"
RCM_TOOL_LENGTH_M="${RCM_TOOL_LENGTH_M:-0.220}"
RCM_SHOW_TOOL="${RCM_SHOW_TOOL:-true}"
RCM_TOOL_RADIUS_M="${RCM_TOOL_RADIUS_M:-0.006}"
RCM_SHOW_DVRK_LND="${RCM_SHOW_DVRK_LND:-true}"
RCM_DVRK_JAW_ANGLE_RAD="${RCM_DVRK_JAW_ANGLE_RAD:-0.45}"
RCM_DVRK_TIP_OFFSET_M="${RCM_DVRK_TIP_OFFSET_M:-0.026}"
RCM_TRACE_TOOL_TIP="${RCM_TRACE_TOOL_TIP:-true}"
RCM_MARKER_RADIUS_M="${RCM_MARKER_RADIUS_M:-0.004}"
RCM_TIP_MARKER_RADIUS_M="${RCM_TIP_MARKER_RADIUS_M:-0.002}"
RCM_TRACE_POINT_RADIUS_M="${RCM_TRACE_POINT_RADIUS_M:-0.0025}"
RCM_TRACE_MIN_INSERTED_DEPTH_M="${RCM_TRACE_MIN_INSERTED_DEPTH_M:-0.065}"
RCM_SHOW_PORT_OVERLAY="${RCM_SHOW_PORT_OVERLAY:-true}"
RCM_PORT_RING_RADIUS_M="${RCM_PORT_RING_RADIUS_M:-0.010}"
RCM_PORT_AXIS_OUTSIDE_M="${RCM_PORT_AXIS_OUTSIDE_M:-0.050}"
RCM_PORT_AXIS_INSIDE_M="${RCM_PORT_AXIS_INSIDE_M:-0.090}"
RCM_CLEAN_GUI="${RCM_CLEAN_GUI:-true}"
RCM_CAMERA_DISTANCE_M="${RCM_CAMERA_DISTANCE_M:-0.90}"
RCM_CAMERA_YAW_DEG="${RCM_CAMERA_YAW_DEG:-42.0}"
RCM_CAMERA_PITCH_DEG="${RCM_CAMERA_PITCH_DEG:--38.0}"
RCM_TRACK_FORCE="${RCM_TRACK_FORCE:-260.0}"
RCM_TRACK_POS_GAIN="${RCM_TRACK_POS_GAIN:-0.65}"
RCM_TRACK_MAX_VEL="${RCM_TRACK_MAX_VEL:-6.0}"
RCM_LAMBDA="${RCM_LAMBDA:-0.650}"
RCM_MAX_JOINT_STEP_RAD="${RCM_MAX_JOINT_STEP_RAD:-0.018}"
RCM_SAFE_INSERTION="${RCM_SAFE_INSERTION:-true}"
RCM_PREINSERT_CLEARANCE_M="${RCM_PREINSERT_CLEARANCE_M:-0.040}"
RCM_PORT_STANDOFF_M="${RCM_PORT_STANDOFF_M:-0.012}"
RCM_PREINSERT_HOLD_SEC="${RCM_PREINSERT_HOLD_SEC:-1.0}"
RCM_APPROACH_DURATION_SEC="${RCM_APPROACH_DURATION_SEC:-2.0}"
RCM_PORT_DWELL_SEC="${RCM_PORT_DWELL_SEC:-0.8}"
RCM_INSERTION_SPEED_MPS="${RCM_INSERTION_SPEED_MPS:-0.018}"
RCM_INSERTED_DWELL_SEC="${RCM_INSERTED_DWELL_SEC:-0.8}"
RCM_STAGE_POSITION_TOLERANCE_M="${RCM_STAGE_POSITION_TOLERANCE_M:-0.006}"
RCM_INIT_Q="${RCM_INIT_Q:-[-0.0,0.462931412,0.0,-1.132967496,0.0,1.204016934,-0.0]}"
RCM_START_MONITOR="${RCM_START_MONITOR:-true}"
RCM_RECORD_METRICS="${RCM_RECORD_METRICS:-}"
RCM_SHOW_PHANTOM="${RCM_SHOW_PHANTOM:-}"
RCM_PHANTOM_COLLISION="${RCM_PHANTOM_COLLISION:-false}"
RCM_PHANTOM_ALPHA="${RCM_PHANTOM_ALPHA:-0.50}"
RCM_PHANTOM_X_M="${RCM_PHANTOM_X_M:-0.701726}"
RCM_PHANTOM_Y_M="${RCM_PHANTOM_Y_M:-0.0}"
RCM_PHANTOM_Z_M="${RCM_PHANTOM_Z_M:-0.240701}"
RCM_WORLD_X_M="${RCM_WORLD_X_M:-0.701726}"
RCM_WORLD_Y_M="${RCM_WORLD_Y_M:-0.0}"
RCM_WORLD_Z_M="${RCM_WORLD_Z_M:-0.404701}"
RCM_TABLE_X_M="${RCM_TABLE_X_M:-0.9}"
RCM_TABLE_Y_M="${RCM_TABLE_Y_M:-0.0}"
RCM_TABLE_TOP_Z_M="${RCM_TABLE_TOP_Z_M:-$RCM_PHANTOM_Z_M}"

mkdir -p "$LOG_DIR" "$PID_DIR"

if [[ -z "$RCM_RECORD_METRICS" ]]; then
  if [[ "$RCM_FIXTURE_MODE" == "rcm" ]]; then
    RCM_RECORD_METRICS="true"
  else
    RCM_RECORD_METRICS="false"
  fi
fi

RCM_METRICS_CSV="${RCM_METRICS_CSV:-$LOG_DIR/rcm_metrics_$(date +%Y%m%d_%H%M%S).csv}"

if [[ -z "$RCM_SHOW_PHANTOM" ]]; then
  if [[ "$RCM_FIXTURE_MODE" == "rcm" ]]; then
    RCM_SHOW_PHANTOM="true"
  else
    RCM_SHOW_PHANTOM="false"
  fi
fi

RCM_PHANTOM_MESH="${RCM_PHANTOM_MESH:-$WS/install/rcm_virtual_fixtures/share/rcm_virtual_fixtures/meshes/phantom_centered.stl}"
if [[ ! -f "$RCM_PHANTOM_MESH" ]]; then
  RCM_PHANTOM_MESH="$WS/src/rcm_virtual_fixtures/meshes/phantom_centered.stl"
fi

RCM_DVRK_LND_URDF="${RCM_DVRK_LND_URDF:-$WS/install/rcm_virtual_fixtures/share/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf}"
if [[ ! -f "$RCM_DVRK_LND_URDF" ]]; then
  RCM_DVRK_LND_URDF="$WS/src/rcm_virtual_fixtures/urdf/dvrk_lnd_420006_tip.urdf"
fi

source_env() {
  set +u
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  set -u 2>/dev/null || true
}

pg_alive() {
  local pgid="$1"
  ps -o pid= --pgid "$pgid" 2>/dev/null | grep -qE '[0-9]+' || return 1
  return 0
}

start_bg() {
  local name="$1"
  shift
  local cmd="$*"
  local log="$LOG_DIR/${name}.log"
  local pidf="$PID_DIR/${name}.pid"

  setsid bash -lc "
    set +u
    source /opt/ros/humble/setup.bash
    source '$WS/install/setup.bash'
    set -u 2>/dev/null || true
    exec $cmd
  " >"$log" 2>&1 &

  local pid=$!
  echo "$pid" > "$pidf"
  echo "[START] $name pgid=$pid log=$log"
}

stop_pgid() {
  local pgid="$1"
  [ -n "$pgid" ] || return 0
  for sig in INT TERM KILL; do
    if pg_alive "$pgid"; then
      echo "[STOP] pgid=$pgid sig=$sig"
      kill "-$sig" -- "-$pgid" 2>/dev/null || true
      sleep 0.6
    fi
  done
}

stop_all() {
  for f in "$PID_DIR"/rcm_*.pid; do
    [ -f "$f" ] || continue
    stop_pgid "$(cat "$f" 2>/dev/null || true)"
    rm -f "$f"
  done

  pkill -INT -f "rcm_virtual_fixture_node|iiwa_pybullet_sim_node|robotstate_bridge|robot_monitor" 2>/dev/null || true
  sleep 0.5
  pkill -TERM -f "rcm_virtual_fixture_node|iiwa_pybullet_sim_node|robotstate_bridge|robot_monitor" 2>/dev/null || true
}

print_status() {
  echo "[CONFIG] mode=$RCM_FIXTURE_MODE gui=$RCM_GUI speed=$RCM_SPEED_MPS radius=$RCM_TRAJECTORY_RADIUS_M tool_length=$RCM_TOOL_LENGTH_M"
  pgrep -af "rcm_virtual_fixture_node|iiwa_pybullet_sim_node|robotstate_bridge|robot_monitor" || true
}

case "${1:-start}" in
  start)
    case "$RCM_FIXTURE_MODE" in
      rcm|line|plane) ;;
      *)
        echo "[ERROR] RCM_FIXTURE_MODE must be one of: rcm, line, plane"
        exit 2
        ;;
    esac

    source_env
    stop_all >/dev/null 2>&1 || true

    start_bg rcm_sim \
      "ros2 run pybullet_ros2_sim iiwa_pybullet_sim_node --ros-args -r __node:=iiwa_pybullet_sim_node -p gui:=$RCM_GUI -p use_goto:=true -p init_q:='$RCM_INIT_Q' -p reset_to_init_on_start:=true -p goto_duration:=0.5 -p goto_use_reset:=true -p goto_force:=$RCM_TRACK_FORCE -p goto_pos_gain:=$RCM_TRACK_POS_GAIN -p goto_vel_gain:=1.00 -p goto_max_vel:=$RCM_TRACK_MAX_VEL -p track_force:=$RCM_TRACK_FORCE -p track_pos_gain:=$RCM_TRACK_POS_GAIN -p track_vel_gain:=1.00 -p track_max_vel:=$RCM_TRACK_MAX_VEL -p table_x_m:=$RCM_TABLE_X_M -p table_y_m:=$RCM_TABLE_Y_M -p table_top_z_m:=$RCM_TABLE_TOP_Z_M -p clean_gui:=$RCM_CLEAN_GUI -p camera_distance_m:=$RCM_CAMERA_DISTANCE_M -p camera_yaw_deg:=$RCM_CAMERA_YAW_DEG -p camera_pitch_deg:=$RCM_CAMERA_PITCH_DEG -p show_rcm_debug_markers:=true -p rcm_marker_radius_m:=$RCM_MARKER_RADIUS_M -p rcm_tip_marker_radius_m:=$RCM_TIP_MARKER_RADIUS_M -p ee_trace_point_radius_m:=$RCM_TRACE_POINT_RADIUS_M -p rcm_trace_min_inserted_depth_m:=$RCM_TRACE_MIN_INSERTED_DEPTH_M -p show_port_detection_overlay:=$RCM_SHOW_PORT_OVERLAY -p port_overlay_ring_radius_m:=$RCM_PORT_RING_RADIUS_M -p port_overlay_axis_outside_m:=$RCM_PORT_AXIS_OUTSIDE_M -p port_overlay_axis_inside_m:=$RCM_PORT_AXIS_INSIDE_M -p show_rcm_tool:=$RCM_SHOW_TOOL -p rcm_tool_length_m:=$RCM_TOOL_LENGTH_M -p rcm_tool_radius_m:=$RCM_TOOL_RADIUS_M -p show_dvrk_lnd_gripper:=$RCM_SHOW_DVRK_LND -p dvrk_lnd_urdf_path:='$RCM_DVRK_LND_URDF' -p dvrk_lnd_jaw_angle_rad:=$RCM_DVRK_JAW_ANGLE_RAD -p dvrk_lnd_tip_offset_m:=$RCM_DVRK_TIP_OFFSET_M -p trace_rcm_tool_tip:=$RCM_TRACE_TOOL_TIP -p show_rcm_phantom:=$RCM_SHOW_PHANTOM -p rcm_phantom_mesh_path:='$RCM_PHANTOM_MESH' -p rcm_phantom_collision:=$RCM_PHANTOM_COLLISION -p rcm_phantom_alpha:=$RCM_PHANTOM_ALPHA -p rcm_phantom_x_m:=$RCM_PHANTOM_X_M -p rcm_phantom_y_m:=$RCM_PHANTOM_Y_M -p rcm_phantom_z_m:=$RCM_PHANTOM_Z_M"

    start_bg rcm_fixture \
      "ros2 run rcm_virtual_fixtures rcm_virtual_fixture_node --ros-args -r __node:=rcm_virtual_fixture_node -p fixture_mode:=$RCM_FIXTURE_MODE -p publish_hz:=$RCM_PUBLISH_HZ -p speed_mps:=$RCM_SPEED_MPS -p trajectory_radius_m:=$RCM_TRAJECTORY_RADIUS_M -p trajectory_length_m:=$RCM_TRAJECTORY_LENGTH_M -p tool_length_m:=$RCM_TOOL_LENGTH_M -p rcm_lambda:=$RCM_LAMBDA -p use_initial_rcm:=false -p rcm_world:='[$RCM_WORLD_X_M,$RCM_WORLD_Y_M,$RCM_WORLD_Z_M]' -p enable_safe_insertion:=$RCM_SAFE_INSERTION -p preinsert_clearance_m:=$RCM_PREINSERT_CLEARANCE_M -p port_standoff_m:=$RCM_PORT_STANDOFF_M -p preinsert_hold_sec:=$RCM_PREINSERT_HOLD_SEC -p approach_duration_sec:=$RCM_APPROACH_DURATION_SEC -p port_dwell_sec:=$RCM_PORT_DWELL_SEC -p insertion_speed_mps:=$RCM_INSERTION_SPEED_MPS -p inserted_dwell_sec:=$RCM_INSERTED_DWELL_SEC -p stage_position_tolerance_m:=$RCM_STAGE_POSITION_TOLERANCE_M -p max_joint_step_rad:=$RCM_MAX_JOINT_STEP_RAD -p record_metrics_csv:=$RCM_RECORD_METRICS -p metrics_csv_path:='$RCM_METRICS_CSV'"

    if [[ "$RCM_RECORD_METRICS" == "true" ]]; then
      ln -sfn "$RCM_METRICS_CSV" "$LOG_DIR/rcm_metrics_latest.csv"
    fi

    start_bg rcm_bridge \
      "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p port:=7755 -p rate_hz:=100.0"

    if [[ "$RCM_START_MONITOR" == "true" ]]; then
      start_bg rcm_monitor "ros2 run robot_monitor robot_monitor"
    fi

    echo "[OK] RCM virtual fixture demo started."
    echo "     mode=$RCM_FIXTURE_MODE logs=$LOG_DIR"
    echo "     tool: length=$RCM_TOOL_LENGTH_M m radius=$RCM_TOOL_RADIUS_M m"
    echo "     safe insertion: $RCM_SAFE_INSERTION, fixed RCM=[$RCM_WORLD_X_M, $RCM_WORLD_Y_M, $RCM_WORLD_Z_M] m"
    if [[ "$RCM_SHOW_DVRK_LND" == "true" ]]; then
      echo "     dVRK Large Needle Driver 420006: $RCM_DVRK_LND_URDF"
    fi
    if [[ "$RCM_SHOW_PHANTOM" == "true" ]]; then
      echo "     phantom: $RCM_PHANTOM_MESH"
      echo "     phantom position: [$RCM_PHANTOM_X_M, $RCM_PHANTOM_Y_M, $RCM_PHANTOM_Z_M] m"
      echo "     table: center=[$RCM_TABLE_X_M, $RCM_TABLE_Y_M] m top_z=$RCM_TABLE_TOP_Z_M m"
    fi
    echo "     status: ros2 topic echo /rcm_virtual_fixtures/status"
    if [[ "$RCM_RECORD_METRICS" == "true" ]]; then
      echo "     metrics CSV: $RCM_METRICS_CSV"
      echo "     latest link: $LOG_DIR/rcm_metrics_latest.csv"
    fi
    ;;
  stop)
    source_env >/dev/null 2>&1 || true
    stop_all
    echo "[OK] RCM virtual fixture demo stopped."
    ;;
  status)
    print_status
    ;;
  logs)
    tail -n 100 -f "$LOG_DIR"/rcm_sim.log "$LOG_DIR"/rcm_fixture.log "$LOG_DIR"/rcm_bridge.log "$LOG_DIR"/rcm_monitor.log 2>/dev/null || true
    ;;
  *)
    echo "Usage: $0 {start|stop|status|logs}"
    echo "Examples:"
    echo "  RCM_FIXTURE_MODE=rcm   $0 start"
    echo "  RCM_FIXTURE_MODE=line  $0 start"
    echo "  RCM_FIXTURE_MODE=plane $0 start"
    exit 1
    ;;
esac
