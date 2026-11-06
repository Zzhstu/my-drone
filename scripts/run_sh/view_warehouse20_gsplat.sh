#!/usr/bin/env bash
set -euo pipefail
ASTRA_VIEW_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_VIEW_OUTPUT="${1:-$ASTRA_VIEW_ROOT/gsplat_runs/warehouse20_lio_20261102_640}"
ASTRA_VIEW_PORT="${2:-8890}"
unset PYTHONPATH
exec "$ASTRA_VIEW_ROOT/.envs/roomgs/bin/python" "$ASTRA_VIEW_ROOT/integration/roomgs/viewer.py" "$ASTRA_VIEW_OUTPUT" --port "$ASTRA_VIEW_PORT"
