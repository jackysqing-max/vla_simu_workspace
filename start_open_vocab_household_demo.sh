#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export OPEN_VOCAB_TARGETS="${OPEN_VOCAB_TARGETS:-true}"
export SIM_SCENE_PRESET="${SIM_SCENE_PRESET:-household}"
export SAM3_PROMPT="${SAM3_PROMPT:-mug}"
export LLM_BACKEND="${LLM_BACKEND:-qwen3_local}"
export QWEN3_STOP_EXTERNAL="${QWEN3_STOP_EXTERNAL:-true}"

if [ "$#" -eq 0 ]; then
  exec "$WS/start_llm_rekep_demo.sh" start
fi

exec "$WS/start_llm_rekep_demo.sh" "$@"
