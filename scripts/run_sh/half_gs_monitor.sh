#!/usr/bin/env bash
set -eo pipefail
ASTRA_MONITOR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
exec /usr/bin/python3 "$ASTRA_MONITOR_ROOT/integration/half_gs/monitor.py" "$@"
