#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || ( "$1" != baseline && "$1" != revisit ) ]]; then
    echo 'Usage: bash run_ghost_revisit_challenge.sh baseline|revisit [max-wall-seconds]' >&2
    exit 2
fi

ASTRA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
export ASTRA_ROOT
PX4_DIR="${PX4_DIR:-/home/a/PX4-Autopilot}"
export PX4_DIR
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:$ASTRA_ROOT/AstraDrone_ros1_ws/src:${ROS_PACKAGE_PATH:-}"
export GAZEBO_PLUGIN_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:/home/a/pedsim_ws/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:/home/a/pedsim_ws/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_MODEL_PATH="$ASTRA_ROOT/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH:-}"

exec python3 "$ASTRA_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/run_small_indoor_exploration.py" \
    --scene cangku-half --ghost-revisit --ghost-challenge-arm "$1" \
    --headless --max-wall-seconds "${2:-1800}"
