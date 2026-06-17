#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
TASK="${*:-pick up the left silver surgical instrument and then release it}"
cd "$WS"

exec docs/recording_scripts/04_robot_grasp_place_recording.sh "$TASK"
