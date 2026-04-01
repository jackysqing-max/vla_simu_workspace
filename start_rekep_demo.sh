#!/usr/bin/env bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[WARN] start_rekep_demo.sh is now a compatibility alias for start_semantic_tracking_demo.sh" >&2
exec "$WS/start_semantic_tracking_demo.sh" "$@"
