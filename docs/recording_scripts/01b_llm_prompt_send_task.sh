#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
TASK="${*:-pick up the left silver surgical instrument and place it into the tray center}"

cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u 2>/dev/null || true

exec python3 "$WS/docs/recording_scripts/llm_plan_capture.py" "$TASK"
