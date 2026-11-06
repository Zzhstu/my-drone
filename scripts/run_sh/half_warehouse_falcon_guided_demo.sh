#!/usr/bin/env bash
# Separate half-warehouse 15 cm coverage-guided experiment.
set -euo pipefail
ASTRA_FALCON_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_FALCON_ENTRY/small_indoor_exploration_demo.sh" --scene cangku-half --planner coverage-guided "$@"
