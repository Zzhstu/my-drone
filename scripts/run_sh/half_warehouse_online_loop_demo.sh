#!/usr/bin/env bash
set -euo pipefail
ASTRA_ONLINE_LOOP_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_ONLINE_LOOP_ENTRY/small_indoor_exploration_demo.sh" --scene cangku-half --ghost-revisit --online-loop "$@"
