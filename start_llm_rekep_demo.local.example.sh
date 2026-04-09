#!/usr/bin/env bash
set -euo pipefail

# Copy this file to `start_llm_rekep_demo.local.sh`, customize it for your
# preferred backend, and keep that local file out of git. The `.gitignore`
# already excludes it.
#
# Recommended behavior in the current LLM demo:
# - the tracker stays idle until an LLM task plan arrives
# - each target is tracked as a hover point above the cube
# - the shared hover clearance defaults to 0.10 m
# - after the stack is up, use `./start_llm_rekep_demo.sh shell`
#   to type instructions directly in the terminal

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Backend selection:
# - `openai`: planner uses the OpenAI API
# - `qwen3_local`: planner uses the local Qwen3 vLLM service
export LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
export GPU_EXECUTION_MODE="${GPU_EXECUTION_MODE:-staged_single_gpu}"

# Replace the placeholder below with your real OpenAI API key only if you keep
# `LLM_BACKEND=openai`.
export OPENAI_API_KEY="${OPENAI_API_KEY:-PASTE_YOUR_OPENAI_API_KEY_HERE}"

# Optional runtime overrides.
export OPENAI_MODEL="${OPENAI_MODEL:-gpt-5-mini}"
export OPENAI_REASONING_EFFORT="${OPENAI_REASONING_EFFORT:-low}"
export QWEN3_VENV="${QWEN3_VENV:-/home/siqin/ros2_workspaces/humble/.venvs/qwen3-vllm}"
export QWEN_MODEL="${QWEN_MODEL:-Qwen/Qwen3-4B}"
export QWEN3_PORT="${QWEN3_PORT:-8000}"
export QWEN3_AUTO_START="${QWEN3_AUTO_START:-true}"
export SAM3_EXEC_DEVICE="${SAM3_EXEC_DEVICE:-cuda}"
export SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
export SAM3_PROMPT="${SAM3_PROMPT:-red cube}"
export SAM3_DEVICE="${SAM3_DEVICE:-cuda}"
export HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.10}"
export SIM_GUI="${SIM_GUI:-false}"

if [[ "$LLM_BACKEND" == "openai" && "$OPENAI_API_KEY" == "PASTE_YOUR_OPENAI_API_KEY_HERE" ]]; then
  echo "[ERROR] Edit $(basename "$0") and replace OPENAI_API_KEY with your real key." >&2
  exit 1
fi

exec "$WS/start_llm_rekep_demo.sh" "${1:-start}"
