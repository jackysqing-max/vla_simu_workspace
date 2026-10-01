#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
TASK="${*:-locate the left silver surgical instrument}"
cd "$WS"

# OBS targets: enlarged SAM3 Mask and SAM3 Keypoint windows.
export SIM_GUI="${SIM_GUI:-true}"
export GPU_MONITOR_WINDOW="${GPU_MONITOR_WINDOW:-false}"
export KEYPOINT_DEBUG_ONLY="${KEYPOINT_DEBUG_ONLY:-true}"
export START_KEYPOINT_VIEWER="${START_KEYPOINT_VIEWER:-true}"
export KEYPOINT_VIEWER_DISPLAY_SCALE="${KEYPOINT_VIEWER_DISPLAY_SCALE:-3.0}"
export KEYPOINT_VIEWER_STATUS_PANEL_WIDTH="${KEYPOINT_VIEWER_STATUS_PANEL_WIDTH:-420}"
export KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE="${KEYPOINT_VIEWER_PREFER_OVERLAY_IMAGE:-true}"
export KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW="${KEYPOINT_VIEWER_SHOW_CLOUD_WINDOW:-false}"
export START_HOVER_FOLLOWER="${START_HOVER_FOLLOWER:-false}"
export ENABLE_GMS_KEYPOINT_DEMO="${ENABLE_GMS_KEYPOINT_DEMO:-true}"
export ENABLE_GRIPPER="${ENABLE_GRIPPER:-false}"
export OPEN_VOCAB_TARGETS="${OPEN_VOCAB_TARGETS:-true}"
export SIM_SCENE_PRESET="${SIM_SCENE_PRESET:-medical_grasp}"
export SAM3_PROMPT="${SAM3_PROMPT:-left silver surgical instrument}"
export LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
export GPU_EXECUTION_MODE="${GPU_EXECUTION_MODE:-staged_single_gpu}"
export QWEN3_AUTO_START="${QWEN3_AUTO_START:-true}"
export QWEN3_STOP_EXTERNAL="${QWEN3_STOP_EXTERNAL:-true}"
export ALLOW_LOCAL_PLANNER_FALLBACK="${ALLOW_LOCAL_PLANNER_FALLBACK:-false}"
export PLANNING_TIMEOUT_SEC="${PLANNING_TIMEOUT_SEC:-120}"

exec ./start_give_me_scissors_keypoint_demo.sh task "$TASK"
