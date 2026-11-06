#!/usr/bin/env bash
set -euo pipefail
ASTRA_R3_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_R3_REF=6143a38537f28cb36eb24e9bbe2e39c8f7967157
if [[ ! -d "$ASTRA_R3_ROOT/third_party/r3live/.git" ]]; then
  mkdir -p "$ASTRA_R3_ROOT/third_party"
  git clone https://github.com/hku-mars/r3live.git "$ASTRA_R3_ROOT/third_party/r3live"
  git -C "$ASTRA_R3_ROOT/third_party/r3live" checkout "$ASTRA_R3_REF"
fi
[[ "$(git -C "$ASTRA_R3_ROOT/third_party/r3live" rev-parse HEAD)" == "$ASTRA_R3_REF" ]] || {
  echo 'R3LIVE source revision differs; refusing to overwrite existing checkout.' >&2; exit 1;
}
ASTRA_R3_PATCH="$ASTRA_R3_ROOT/integration/r3live/upstream.patch"
if git -C "$ASTRA_R3_ROOT/third_party/r3live" apply --check "$ASTRA_R3_PATCH" 2>/dev/null; then
  git -C "$ASTRA_R3_ROOT/third_party/r3live" apply "$ASTRA_R3_PATCH"
else
  git -C "$ASTRA_R3_ROOT/third_party/r3live" apply --reverse --check "$ASTRA_R3_PATCH"
fi
source /opt/ros/noetic/setup.bash
source "$ASTRA_R3_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
mkdir -p "$ASTRA_R3_ROOT/r3live_ws/src"
[[ -e "$ASTRA_R3_ROOT/r3live_ws/src/r3live" ]] || ln -s ../../third_party/r3live/r3live "$ASTRA_R3_ROOT/r3live_ws/src/r3live"
catkin_make -C "$ASTRA_R3_ROOT/r3live_ws" -j1 \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_C_COMPILER=/usr/bin/clang \
  -DR3LIVE_BUILD_MESHING=OFF -DCMAKE_BUILD_TYPE=Release
