#!/usr/bin/env bash
set -euo pipefail
ASTRA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export ASTRA_ROOT PX4_DIR DISABLE_ROS1_EOL_WARNINGS=1
ASTRA_ROOM_ARGS=("$@")
set --
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
[[ -f "$ASTRA_ROOT/r3live_roomgs_ws/devel/setup.bash" ]] || {
  echo '先运行 bash scripts/run_sh/build_roomgs_demo.sh' >&2; exit 1;
}
source "$ASTRA_ROOT/r3live_roomgs_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_DIR:$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic:$ASTRA_ROOT/simulation/sim_workspace/src:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="$ASTRA_ROOT/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="$ASTRA_ROOT/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
exec /usr/bin/python3 "$ASTRA_ROOT/integration/roomgs/pipeline.py" "${ASTRA_ROOM_ARGS[@]}"
