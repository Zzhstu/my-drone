#!/usr/bin/env bash
# Static Gazebo scene only: no PX4, arming, navigation, mapping, or training.
set -euo pipefail
set +u
source /usr/share/gazebo/setup.bash
set -u
ASTRA_SMALL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_SMALL_WORLD="$ASTRA_SMALL_ROOT/simulation/astra_gazebo_worlds/cangku_20sqm.world"
[[ -f "$ASTRA_SMALL_WORLD" ]] || { echo "缺少场景：$ASTRA_SMALL_WORLD" >&2; exit 1; }
export GAZEBO_MODEL_PATH="$ASTRA_SMALL_ROOT/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$ASTRA_SMALL_ROOT/simulation/astra_gazebo_worlds:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MASTER_URI="${ASTRA_WAREHOUSE20_MASTER_URI:-http://127.0.0.1:11365}"
if [[ "${1:-}" == "--headless" ]]; then
  exec gzserver --verbose "$ASTRA_SMALL_WORLD"
fi
exec gazebo --verbose "$ASTRA_SMALL_WORLD"
