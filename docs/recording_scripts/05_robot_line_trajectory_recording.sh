#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
cd "$WS"

# OBS target: PyBullet GUI, optionally RobotMonitor. Shows baseline straight-line
# Cartesian end-effector motion using the existing iiwa_line_ik_desired node.
exec ./start_iiwa_stack_min.sh start
