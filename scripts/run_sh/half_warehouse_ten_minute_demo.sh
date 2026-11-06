#!/usr/bin/env bash
set -euo pipefail
ASTRA_ENDURANCE_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_ENDURANCE_ENTRY/small_indoor_exploration_demo.sh" --scene cangku-half --ghost-revisit --ten-minute "$@"
