#!/usr/bin/env bash
set -euo pipefail
ASTRA_ONLINE_ENDURANCE_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_ONLINE_ENDURANCE_ENTRY/half_warehouse_online_loop_demo.sh" --ten-minute "$@"
