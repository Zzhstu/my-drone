#!/usr/bin/env bash
# Same dual-map flight policy; only scene and one-minute actor scenario differ.
set -euo pipefail
ASTRA_OFFICE50_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_OFFICE50_ENTRY/small_indoor_exploration_demo.sh" \
  --scene office-50 --ghost-revisit --online-loop --ten-minute \
  --mission-policy joint-channel --surface-inspection --ghost-score-scale 1.3 \
  --dual-map --keep-evidence-bags 0 "$@"
