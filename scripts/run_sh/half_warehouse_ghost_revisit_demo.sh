#!/usr/bin/env bash
# Isolated ghost-revisit variant; baseline comparison outputs are untouched.
set -euo pipefail
ASTRA_GHOST_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_GHOST_ENTRY/small_indoor_exploration_demo.sh" --scene cangku-half --ghost-revisit "$@"
