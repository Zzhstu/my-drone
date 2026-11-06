#!/usr/bin/env bash
# Current surface demo plus one right-middle actor, 180 simulation seconds.
set -euo pipefail
ASTRA_TWO_PEOPLE_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_TWO_PEOPLE_ENTRY/half_warehouse_surface_inspection_ten_minute_demo.sh" --right-pedestrian --ghost-score-scale 1.3 "$@"
