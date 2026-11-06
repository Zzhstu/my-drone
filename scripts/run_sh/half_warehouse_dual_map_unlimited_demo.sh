#!/usr/bin/env bash
# Experimental simulator-only mode, kept separate from the classic demo.
set -euo pipefail
ASTRA_DUAL_UNLIMITED_ENTRY="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ASTRA_DUAL_UNLIMITED_ENTRY/half_warehouse_dual_map_demo.sh" \
  --no-time-limit "$@"
