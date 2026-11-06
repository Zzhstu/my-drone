#!/usr/bin/env bash
# Classic bounded flight policy; paired original + native DUFO maps.
set -euo pipefail
ASTRA_DUAL_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_DUAL_ENTRY/half_warehouse_two_pedestrians_three_minute_demo.sh" \
  --dual-map --max-wall-seconds 2400 --efficient-demo \
  --evidence-truth-hz 10 --evidence-map-hz 0.02 \
  --right-pedestrian-duration 60 "$@"
