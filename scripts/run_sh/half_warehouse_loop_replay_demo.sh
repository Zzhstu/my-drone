#!/usr/bin/env bash
set -euo pipefail
ASTRA_LOOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "用法：bash $0 已完成航次前缀 [输出目录]"
  echo "此demo重放录包并重建去鬼影地图；不启动无人机或仿真。"
  exit 2
fi
ASTRA_LOOP_PREFIX="$1"
ASTRA_LOOP_OUTPUT="${2:-$ASTRA_LOOP_ROOT/half_warehouse_loop_demo_runs/loop_$(date +%Y%m%d_%H%M%S)}"
[[ -f "${ASTRA_LOOP_PREFIX}_evidence.bag" ]] || { echo "录包不存在：${ASTRA_LOOP_PREFIX}_evidence.bag"; exit 2; }
# Noetic setup scripts expect some unset variables.
set +u
source "$ASTRA_LOOP_ROOT/AstraDrone_ros1_ws/devel/setup.bash"
set -u
mkdir -p "$ASTRA_LOOP_OUTPUT"
/usr/bin/python3 "$ASTRA_LOOP_ROOT/evaluation/half_warehouse/replay_loop_backend.py" \
  --prefix "$ASTRA_LOOP_PREFIX" --output "$ASTRA_LOOP_OUTPUT/loop"
/usr/bin/python3 "$ASTRA_LOOP_ROOT/evaluation/half_warehouse/replay_mapper.py" \
  "${ASTRA_LOOP_PREFIX}_evidence.bag" --canonical --require-ack \
  --speed 1000 --min-wall-delay 0.001 --variant loop_corrected_maintained \
  --loop-paths "$ASTRA_LOOP_OUTPUT/loop_paths.json" --output-prefix "$ASTRA_LOOP_OUTPUT/maintained"
/usr/bin/python3 "$ASTRA_LOOP_ROOT/evaluation/half_warehouse/audit_loop_demo.py" \
  --scene-prefix "$ASTRA_LOOP_PREFIX" --output "$ASTRA_LOOP_OUTPUT/map_quality.json" \
  "$ASTRA_LOOP_OUTPUT/maintained_replay.json"
echo "回环报告：$ASTRA_LOOP_OUTPUT/loop_report.json"
echo "去鬼影维护点云：$ASTRA_LOOP_OUTPUT/maintained_map.pcd"
echo "地图质量及未解决鬼影：$ASTRA_LOOP_OUTPUT/map_quality.json"
