#!/usr/bin/env bash
# For demos that already started pedestrian_relay or warehouse_pedestrian.
# This script starts only the keyboard client; it does not spawn pedestrians.
set -eo pipefail

ASTRA_DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_PEDSIM_ROOT="${PEDSIM_WS:-/home/a/pedsim_ws}"

source /opt/ros/noetic/setup.bash
source "$ASTRA_PEDSIM_ROOT/devel/setup.bash"
source "$ASTRA_DEMO_ROOT/AstraDrone_ros1_ws/devel/setup.bash"

# The workspace overlays can replace PedSim's generated message path.
export PYTHONPATH="$ASTRA_PEDSIM_ROOT/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="$ASTRA_DEMO_ROOT/AstraDrone_ros1_ws/src:$ASTRA_PEDSIM_ROOT/src:${ROS_PACKAGE_PATH:-}"

exec rosrun fastlio_bridge pedestrian_control.py --keyboard
