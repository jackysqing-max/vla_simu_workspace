#!/usr/bin/env bash
set -euo pipefail

MODEL="${QWEN_MODEL:-Qwen/Qwen3-4B}"
HOST="${QWEN3_HOST:-0.0.0.0}"
PORT="${QWEN3_PORT:-8000}"
TENSOR_PARALLEL_SIZE="${QWEN3_TENSOR_PARALLEL_SIZE:-1}"
# This workstation also runs SAM3 on the same 16 GiB GPU.  Conservative
# defaults prevent vLLM from reserving nearly all VRAM when this helper is
# launched directly instead of through the demo wrapper.
GPU_MEMORY_UTILIZATION="${QWEN3_GPU_MEMORY_UTILIZATION:-0.55}"
MAX_MODEL_LEN="${QWEN3_MAX_MODEL_LEN:-1792}"
HF_HOME="${HF_HOME:-/home/siqin/ros2_workspaces/humble/.cache/huggingface}"
HF_HUB_DISABLE_XET="${HF_HUB_DISABLE_XET:-1}"
VLLM_BIN="${QWEN3_VLLM_BIN:-vllm}"

mkdir -p "${HF_HOME}"
export HF_HOME
export HF_HUB_DISABLE_XET

exec "${VLLM_BIN}" serve "${MODEL}" \
  --host "${HOST}" \
  --port "${PORT}" \
  --tensor-parallel-size "${TENSOR_PARALLEL_SIZE}" \
  --gpu-memory-utilization "${GPU_MEMORY_UTILIZATION}" \
  --max-model-len "${MAX_MODEL_LEN}" \
  --generation-config vllm \
  --reasoning-parser qwen3 \
  --structured-outputs-config.enable_in_reasoning=True \
  "$@"
