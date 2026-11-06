#!/usr/bin/env bash
# Same half-warehouse flight, plus an independent FreeDOM static-map baseline.
set -euo pipefail
ASTRA_HALF_FREEDOM_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if command -v systemd-inhibit >/dev/null 2>&1; then
  # Keep an unattended SITL run awake; the runner still enforces its own
  # suspend-aware hard deadline if sleep is forced by the system or user.
  exec systemd-inhibit --what=sleep:idle --mode=block \
    --who=AstraDroneOpen --why='Finish half-warehouse FreeDOM demo and save maps' \
    bash "$ASTRA_HALF_FREEDOM_ENTRY/half_warehouse_exploration_demo.sh" --freedom-baseline "$@"
fi
exec bash "$ASTRA_HALF_FREEDOM_ENTRY/half_warehouse_exploration_demo.sh" --freedom-baseline "$@"
