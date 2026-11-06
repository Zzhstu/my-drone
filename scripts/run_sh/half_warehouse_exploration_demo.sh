#!/usr/bin/env bash
# Half-area cangku, sharing the tested indoor runner and yaw adapter.
set -euo pipefail
ASTRA_HALF_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_HALF_ENTRY/small_indoor_exploration_demo.sh" --scene cangku-half "$@"
