#!/usr/bin/env bash
set -euo pipefail

WS="/home/siqin/ros2_workspaces/humble/ros2_pybullet_ws"
SCRIPTS="$WS/docs/recording_scripts"
TASK="${*:-pick up the left silver surgical instrument and place it into the tray center}"

cd "$WS/docs"

"$SCRIPTS/01a_llm_prompt_stack_start.sh"
sleep 1
"$SCRIPTS/01b_llm_prompt_send_task.sh" "$TASK"
