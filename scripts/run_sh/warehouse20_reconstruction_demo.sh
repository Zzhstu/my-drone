#!/usr/bin/env bash
set -euo pipefail
ASTRA_WAREHOUSE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$ASTRA_WAREHOUSE_ROOT/scripts/run_sh/roomgs_demo.sh" --scene warehouse20 "$@"
