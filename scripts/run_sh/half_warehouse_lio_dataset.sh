#!/usr/bin/env bash
set -euo pipefail
ASTRA_HALF_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export PX4_DIR DISABLE_ROS1_EOL_WARNINGS=1
ASTRA_HALF_ARGS=("$@")
set --
set +u
source /opt/ros/noetic/setup.bash
source "$ASTRA_HALF_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
set -u
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_HALF_ROOT/simulation/sim_workspace/src:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="$ASTRA_HALF_ROOT/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$ASTRA_HALF_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$ASTRA_HALF_ROOT/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
exec /usr/bin/python3 "$ASTRA_HALF_ROOT/integration/half_gs/capture.py" "${ASTRA_HALF_ARGS[@]}"
