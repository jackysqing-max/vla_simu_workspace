#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
cd "$WS"

./start_llm_rekep_demo.sh stop || true
./start_semantic_tracking_demo.sh stop || true
./start_iiwa_stack_min.sh stop || true

pkill -INT -f "iiwa_circle_ik_desired_recording.py|tracking_overlay_viewer_node" 2>/dev/null || true
sleep 1
pkill -TERM -f "iiwa_circle_ik_desired_recording.py|tracking_overlay_viewer_node" 2>/dev/null || true
sleep 1
pkill -KILL -f "iiwa_circle_ik_desired_recording.py|tracking_overlay_viewer_node" 2>/dev/null || true

./clean_demo_processes.sh || true
