#!/usr/bin/env bash
# Explicit major-region baseline; never claims strict whole-scene acceptance.
set -euo pipefail
ASTRA_HALF_DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_HALF_DEMO_GUI=false
if [[ "${1:-}" == "--gui" ]]; then
  ASTRA_HALF_DEMO_GUI=true
  shift
fi
if (( $# > 1 )); then
  printf '用法：bash %s [--gui] [新航次标签]\n' "$0" >&2; exit 2
fi
if [[ "$ASTRA_HALF_DEMO_GUI" == true && -z "${DISPLAY:-}" ]]; then
  printf '图形模式需要在本机桌面终端运行（DISPLAY 未设置）。\n' >&2; exit 2
fi
ASTRA_HALF_DEMO_LABEL="${1:-$(date +%Y%m%d_%H%M%S)}"
ASTRA_HALF_DATASET="$ASTRA_HALF_DEMO_ROOT/half_gs_dataset_runs/warehouse_half_$ASTRA_HALF_DEMO_LABEL"
ASTRA_HALF_MODEL="$ASTRA_HALF_DEMO_ROOT/gsplat_runs/warehouse_half_$ASTRA_HALF_DEMO_LABEL"
if [[ ! -f "$ASTRA_HALF_DEMO_ROOT/half_gs_plans/warehouse_half_20261103/mesh_reference.npz" ]]; then
  printf '缺少冻结参考；先按docs/HALF_WAREHOUSE_GSPLAT_CN.md生成参考。\n' >&2; exit 1
fi
ASTRA_HALF_CAPTURE_ARGS=()
if [[ "$ASTRA_HALF_DEMO_GUI" != true ]]; then ASTRA_HALF_CAPTURE_ARGS+=(--headless); fi
bash "$ASTRA_HALF_DEMO_ROOT/scripts/run_sh/half_warehouse_lio_dataset.sh" "${ASTRA_HALF_CAPTURE_ARGS[@]}" \
  --plan "$ASTRA_HALF_DEMO_ROOT/integration/half_gs/config/major_regions_20261103.json" \
  --reference-dir "$ASTRA_HALF_DEMO_ROOT/half_gs_plans/warehouse_half_20261103" --output-dir "$ASTRA_HALF_DATASET"
bash "$ASTRA_HALF_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_HALF_DEMO_ROOT/integration/half_gs/pipeline.py" "$ASTRA_HALF_DATASET" \
  --output "$ASTRA_HALF_MODEL" --partial-baseline
bash "$ASTRA_HALF_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_HALF_DEMO_ROOT/evaluation/half_gs_20261103/curate_gaussians.py" \
  "$ASTRA_HALF_DATASET" "$ASTRA_HALF_MODEL" "${ASTRA_HALF_MODEL}_semantic"
bash "$ASTRA_HALF_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_HALF_DEMO_ROOT/integration/lio_gs/verify.py" "${ASTRA_HALF_MODEL}_semantic"
bash "$ASTRA_HALF_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_HALF_DEMO_ROOT/integration/half_gs/finalize.py" "${ASTRA_HALF_MODEL}_semantic"
printf '训练基线：%s\n语义剔除候选：%s\n候选效果需检查匹配视图；阴影与背景缺口仍可能存在。\n' "$ASTRA_HALF_MODEL" "${ASTRA_HALF_MODEL}_semantic"
if [[ "$ASTRA_HALF_DEMO_GUI" == true ]]; then
  bash "$ASTRA_HALF_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
    "$ASTRA_HALF_DEMO_ROOT/integration/lio_gs/desktop_viewer.py" "${ASTRA_HALF_MODEL}_semantic" \
    --title 'AstraDrone Half Warehouse 3DGS' \
    --report-dir "${ASTRA_HALF_MODEL}_semantic/native_viewer"
fi
