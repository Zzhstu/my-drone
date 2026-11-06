#!/usr/bin/env bash
set -euo pipefail
ASTRA_ROOM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_ROOM_ENV="$ASTRA_ROOM_ROOT/.envs/roomgs"
ASTRA_ROOM_REF=6143a38537f28cb36eb24e9bbe2e39c8f7967157
if [[ ! -e "$ASTRA_ROOM_ROOT/third_party/r3live_roomgs/.git" ]]; then
  git -C "$ASTRA_ROOM_ROOT/third_party/r3live" worktree add --detach ../r3live_roomgs "$ASTRA_ROOM_REF"
fi
[[ "$(git -C "$ASTRA_ROOM_ROOT/third_party/r3live_roomgs" rev-parse HEAD)" == "$ASTRA_ROOM_REF" ]]
ASTRA_ROOM_PATCH="$ASTRA_ROOM_ROOT/integration/roomgs/r3live.patch"
if git -C "$ASTRA_ROOM_ROOT/third_party/r3live_roomgs" apply --check "$ASTRA_ROOM_PATCH" 2>/dev/null; then
  git -C "$ASTRA_ROOM_ROOT/third_party/r3live_roomgs" apply "$ASTRA_ROOM_PATCH"
else
  git -C "$ASTRA_ROOM_ROOT/third_party/r3live_roomgs" apply --reverse --check "$ASTRA_ROOM_PATCH"
fi
set --
source /opt/ros/noetic/setup.bash
source "$ASTRA_ROOM_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
mkdir -p "$ASTRA_ROOM_ROOT/r3live_roomgs_ws/src"
[[ -e "$ASTRA_ROOM_ROOT/r3live_roomgs_ws/src/r3live" ]] || ln -s ../../third_party/r3live_roomgs/r3live "$ASTRA_ROOM_ROOT/r3live_roomgs_ws/src/r3live"
catkin_make -C "$ASTRA_ROOM_ROOT/r3live_roomgs_ws" -j1 \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++ -DCMAKE_C_COMPILER=/usr/bin/clang \
  -DR3LIVE_BUILD_MESHING=OFF -DCMAKE_BUILD_TYPE=Release
if [[ ! -x "$ASTRA_ROOM_ENV/bin/python" ]]; then
  "${CONDA_EXE:-$HOME/miniconda3/bin/conda}" create --prefix "$ASTRA_ROOM_ENV" --override-channels \
    -c conda-forge -c nvidia python=3.11 pip ninja cuda-nvcc=12.8 cuda-cudart-dev=12.8 -y
fi
env -u PYTHONPATH "$ASTRA_ROOM_ENV/bin/python" -m pip install torch==2.7.1 torchvision==0.22.1 --index-url https://download.pytorch.org/whl/cu128
env -u PYTHONPATH "$ASTRA_ROOM_ENV/bin/python" -m pip install 'numpy<2' scipy pillow matplotlib imageio imageio-ffmpeg viser plyfile ninja rich jaxtyping tyro embreex rtree
if [[ ! -d "$ASTRA_ROOM_ROOT/third_party/gsplat/.git" ]]; then
  git clone --branch v1.5.3 --depth 1 --recurse-submodules --shallow-submodules https://github.com/nerfstudio-project/gsplat.git "$ASTRA_ROOM_ROOT/third_party/gsplat"
fi
[[ "$(git -C "$ASTRA_ROOM_ROOT/third_party/gsplat" rev-parse HEAD)" == 937e29912570c372bed6747a5c9bf85fed877bae ]]
env -u PYTHONPATH BUILD_NO_CUDA=1 "$ASTRA_ROOM_ENV/bin/python" -m pip install --no-build-isolation -e "$ASTRA_ROOM_ROOT/third_party/gsplat"
env -u PYTHONPATH CUDA_HOME="$ASTRA_ROOM_ENV" \
  PATH="$ASTRA_ROOM_ENV/bin:$PATH" \
  CPATH="$ASTRA_ROOM_ENV/targets/x86_64-linux/include" \
  LIBRARY_PATH="$ASTRA_ROOM_ENV/lib" LD_LIBRARY_PATH="$ASTRA_ROOM_ENV/lib:${LD_LIBRARY_PATH:-}" \
  MAX_JOBS=1 TORCH_CUDA_ARCH_LIST=12.0 CXX=/usr/bin/g++ CC=/usr/bin/gcc \
  "$ASTRA_ROOM_ENV/bin/python" "$ASTRA_ROOM_ROOT/integration/roomgs/smoke_gpu.py"
echo '独立R3LIVE与gsplat环境准备完成，实际CUDA渲染和梯度检查通过。'
