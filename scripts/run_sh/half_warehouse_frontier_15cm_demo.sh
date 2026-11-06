#!/usr/bin/env bash
# Matched-resolution control for the coverage-guided half-warehouse demo.
set -euo pipefail
ASTRA_FRONTIER_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_FRONTIER_ENTRY/small_indoor_exploration_demo.sh" --scene cangku-half --planner frontier-15cm "$@"
