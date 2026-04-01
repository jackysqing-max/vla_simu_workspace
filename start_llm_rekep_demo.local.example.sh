#!/usr/bin/env bash
set -euo pipefail

# Copy this file to `start_llm_rekep_demo.local.sh`, fill in your real API key,
# and keep that local file out of git. The `.gitignore` already excludes it.
#
# Recommended behavior in the current LLM demo:
# - the tracker stays idle until an LLM task plan arrives
# - each target is tracked as a hover point above the cube
# - the shared hover clearance defaults to 0.10 m
# - after the stack is up, use `./start_llm_rekep_demo.sh shell`
#   to type instructions directly in the terminal

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Replace the placeholder below with your real OpenAI API key.
export OPENAI_API_KEY="PASTE_YOUR_OPENAI_API_KEY_HERE"

# Optional runtime overrides.
export OPENAI_MODEL="${OPENAI_MODEL:-gpt-5-mini}"
export OPENAI_REASONING_EFFORT="${OPENAI_REASONING_EFFORT:-low}"
export SAM3_VENV="${SAM3_VENV:-$HOME/venvs/ros_vla}"
export SAM3_PROMPT="${SAM3_PROMPT:-red cube}"
export SAM3_DEVICE="${SAM3_DEVICE:-cuda}"
export HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.10}"

if [[ "$OPENAI_API_KEY" == "PASTE_YOUR_OPENAI_API_KEY_HERE" ]]; then
  echo "[ERROR] Edit $(basename "$0") and replace OPENAI_API_KEY with your real key." >&2
  exit 1
fi

exec "$WS/start_llm_rekep_demo.sh" "${1:-start}"
