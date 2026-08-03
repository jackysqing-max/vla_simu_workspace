#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${1:-}" =~ ^(task|gui|prompt-gui)$ && "$#" -eq 1 ]]; then
  exec "$WS/start_prompt_gui.sh" --demo medical
fi

export OPEN_VOCAB_TARGETS="${OPEN_VOCAB_TARGETS:-true}"
export SIM_SCENE_PRESET="${SIM_SCENE_PRESET:-medical}"
export SAM3_PROMPT="${SAM3_PROMPT:-tissue pad}"
export HOVER_OFFSET_Z="${HOVER_OFFSET_Z:-0.08}"
export EXECUTOR_SUCCESS_RADIUS_M="${EXECUTOR_SUCCESS_RADIUS_M:-0.09}"
export LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
export QWEN3_STOP_EXTERNAL="${QWEN3_STOP_EXTERNAL:-true}"
export SIM_GUI="true"

if [ "$#" -eq 0 ]; then
  exec "$WS/start_llm_rekep_demo.sh" start
fi

exec "$WS/start_llm_rekep_demo.sh" "$@"
