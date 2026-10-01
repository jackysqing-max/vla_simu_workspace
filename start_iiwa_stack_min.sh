#!/usr/bin/env bash
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$WS/run_logs"
PID_DIR="$WS/run_pids"
mkdir -p "$LOG_DIR" "$PID_DIR"

source_env() {
  # 防止用户在外部 shell 开了 set -u 导致 source 报 AMENT_TRACE_SETUP_FILES unbound
  set +u
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  set -u 2>/dev/null || true
}

# 读取进程组是否仍存在
pg_alive() {
  local pgid="$1"
  ps -o pid= --pgid "$pgid" 2>/dev/null | grep -qE '[0-9]+' || return 1
  return 0
}

start_bg() {
  local name="$1"; shift
  local cmd="$*"
  local log="$LOG_DIR/${name}.log"
  local pidf="$PID_DIR/${name}.pid"

  mkdir -p "$LOG_DIR" "$PID_DIR"

  setsid bash -lc "
    source /opt/ros/humble/setup.bash
    source $WS/install/setup.bash
    exec $cmd
  " >"$log" 2>&1 &

  local pid=$!
  echo "$pid" > "$pidf"
  echo "[START] $name pgid=$pid  cmd=$cmd"
}


start_retry_bg() {
  local name="$1"; shift
  local cmd="$*"
  local log="$LOG_DIR/${name}.log"
  local pidf="$PID_DIR/${name}.pid"

  mkdir -p "$LOG_DIR" "$PID_DIR"

  setsid bash -lc "
    source /opt/ros/humble/setup.bash
    source $WS/install/setup.bash

    child=''
    term() {
      if [ -n \"\$child\" ]; then
        kill -TERM \"\$child\" 2>/dev/null || true
        sleep 0.2
        kill -KILL \"\$child\" 2>/dev/null || true
      fi
      exit 0
    }
    trap term INT TERM HUP

    while true; do
      $cmd &
      child=\$!
      wait \$child
      rc=\$?
      child=''

      if [ \$rc -eq 0 ]; then
        exit 0
      fi
      echo \"[retry] $name exited rc=\$rc, restart in 1s...\" >&2
      sleep 1
    done
  " >"$log" 2>&1 &

  local pid=$!
  echo "$pid" > "$pidf"
  echo "[START] $name(retry) pgid=$pid  cmd=$cmd"
}


stop_one_pgid() {
  local pgid="$1"
  [ -n "$pgid" ] || return 0

  # 先 SIGINT（ROS 更容易正常 shutdown），再 TERM，最后 KILL
  for sig in INT TERM KILL; do
    if pg_alive "$pgid"; then
      echo "[STOP] pgid=$pgid sig=$sig"
      kill "-$sig" -- "-$pgid" 2>/dev/null || true
      sleep 1
    fi
  done
}

stop_all() {
  # 1) 先按你记录的 PGID 关（优雅 -> 强制）
  for f in "$PID_DIR"/*.pid; do
    [ -f "$f" ] || continue
    pgid="$(cat "$f" 2>/dev/null || true)"
    [ -n "$pgid" ] || { rm -f "$f"; continue; }

    # 进程组信号：INT -> TERM -> KILL
    kill -INT  -- "-$pgid" 2>/dev/null || true
    sleep 1
    kill -TERM -- "-$pgid" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-$pgid" 2>/dev/null || true

    rm -f "$f"
  done

  # 2) 兜底：按可执行名/节点脚本名杀（解决 ros2 run 派生残留）
  pkill -INT  -f "iiwa_pybullet_sim_node|iiwa_line_ik_desired|iiwa_impedance_controller|robotstate_bridge|robot_monitor" 2>/dev/null || true
  sleep 1
  pkill -TERM -f "iiwa_pybullet_sim_node|iiwa_line_ik_desired|iiwa_impedance_controller|robotstate_bridge|robot_monitor" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "iiwa_pybullet_sim_node|iiwa_line_ik_desired|iiwa_impedance_controller|robotstate_bridge|robot_monitor" 2>/dev/null || true

  # 3) 刷新 ros2-daemon，避免 node list 显示旧图
  ros2 daemon stop  >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true

  # 4) 清理 pid 目录里可能遗留的 .pid
  rm -f "$PID_DIR"/*.pid 2>/dev/null || true
}


case "${1:-start}" in
  start)
    source_env

    start_bg       sim      "ros2 run pybullet_ros2_sim iiwa_pybullet_sim_node --ros-args -r __node:=iiwa_pybullet_sim_node"
    start_retry_bg desired  "ros2 run pybullet_ros2_sim iiwa_line_ik_desired --ros-args -r __node:=iiwa_line_ik_desired"
    start_retry_bg imp      "ros2 run pybullet_ros2_sim iiwa_impedance_controller --ros-args -r __node:=iiwa_impedance_controller"
    start_retry_bg bridge   "ros2 run iiwa_state_udp_bridge robotstate_bridge --ros-args -p port:=7755 -p rate_hz:=100.0"
    start_bg       monitor  "ros2 run robot_monitor robot_monitor"

    echo "[OK] started. Logs in $LOG_DIR , PGIDs in $PID_DIR"
    ;;
  stop)
    source_env >/dev/null 2>&1 || true
    stop_all
    echo "[OK] stopped"
    ;;
  logs)
    tail -n 80 -f "$LOG_DIR"/sim.log "$LOG_DIR"/desired.log "$LOG_DIR"/imp.log "$LOG_DIR"/bridge.log "$LOG_DIR"/monitor.log 2>/dev/null || true
    ;;
  *)
    echo "Usage: $0 {start|stop|logs}"
    echo "Tip: STOP_DAEMON=1 $0 stop   # also stop ros2 daemon"
    exit 1
    ;;
esac
