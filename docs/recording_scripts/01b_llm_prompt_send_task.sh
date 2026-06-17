#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
TASK="${*:-pick up the left silver surgical instrument and then release it}"
cd "$WS"

export LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
export GPU_EXECUTION_MODE="${GPU_EXECUTION_MODE:-staged_single_gpu}"
export QWEN3_AUTO_START="${QWEN3_AUTO_START:-true}"
export QWEN3_STOP_EXTERNAL="${QWEN3_STOP_EXTERNAL:-true}"
export PLANNING_TIMEOUT_SEC="${PLANNING_TIMEOUT_SEC:-120}"

exec ./start_give_me_scissors_keypoint_demo.sh task "$TASK"
