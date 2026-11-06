#!/usr/bin/env bash
# Independent author DUFOMap map on the same two-person/600s flight inputs.
set -euo pipefail
ASTRA_DUFO_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_DUFO_ENTRY/half_warehouse_two_pedestrians_three_minute_demo.sh" --dufomap-baseline "$@"
