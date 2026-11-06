#!/usr/bin/env bash
set -euo pipefail
ASTRA_JOINT_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_JOINT_ENTRY/half_warehouse_online_loop_ten_minute_demo.sh" --mission-policy joint-channel "$@"
