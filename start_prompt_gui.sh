#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "$WS/scripts/llm_prompt_gui.py" "$@"
