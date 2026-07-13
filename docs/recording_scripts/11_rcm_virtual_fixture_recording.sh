#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
MODE="${1:-rcm}"

case "$MODE" in
  rcm|line|plane) ;;
  *)
    echo "Usage: $0 {rcm|line|plane}"
    exit 2
    ;;
esac

export RCM_FIXTURE_MODE="${RCM_FIXTURE_MODE:-$MODE}"
export RCM_GUI="${RCM_GUI:-true}"
export RCM_SPEED_MPS="${RCM_SPEED_MPS:-0.018}"
export RCM_TRAJECTORY_RADIUS_M="${RCM_TRAJECTORY_RADIUS_M:-0.020}"
export RCM_TRAJECTORY_LENGTH_M="${RCM_TRAJECTORY_LENGTH_M:-0.100}"
export RCM_START_MONITOR="${RCM_START_MONITOR:-true}"

cd "$WS"
exec ./start_rcm_virtual_fixture_demo.sh start
