#!/usr/bin/env bash
set -euo pipefail
ASTRA_DUAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
astra_dual_ros_environment() {
  set +u
  source /opt/ros/noetic/setup.bash
  source "$ASTRA_DUAL_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
  set -u
}
astra_dual_ros_environment
exec /usr/bin/python3 "$ASTRA_DUAL_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/dual_map_replay.py" "$@"
