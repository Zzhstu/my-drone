#!/usr/bin/env bash
# Static GitHub small-office exploration; existing warehouse demos untouched.
set -euo pipefail
ASTRA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export ASTRA_ROOT PX4_DIR
ASTRA_DEMO_ARGS=("$@")
set --
if [[ ! -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ||
      ! -d "$PX4_DIR/build/px4_sitl_default" ]]; then
  echo '错误：请先安装并构建现有项目使用的 PX4 Gazebo Classic SITL。' >&2
  exit 1
fi
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$ASTRA_ROOT/AstraDrone_ros1_ws/src:$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:${ROS_PACKAGE_PATH:-}"
export GAZEBO_PLUGIN_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
exec /usr/bin/python3 "$ASTRA_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/run_small_indoor_exploration.py" "${ASTRA_DEMO_ARGS[@]}"
