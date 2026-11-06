#!/usr/bin/env bash
# Experimental missing-surface inspection, isolated from existing defaults.
set -euo pipefail
ASTRA_SURFACE_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_SURFACE_ENTRY/half_warehouse_joint_channel_ten_minute_demo.sh" --surface-inspection "$@"
