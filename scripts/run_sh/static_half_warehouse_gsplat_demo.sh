#!/usr/bin/env bash
# Actual actor-free capture and offline GS; fixed targets retain coverage gaps.
set -euo pipefail
ASTRA_STATIC_DEMO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_STATIC_DEMO_GUI=false
if [[ "${1:-}" == "--gui" ]]; then
  ASTRA_STATIC_DEMO_GUI=true
  shift
fi
if (( $# > 1 )); then
  printf '用法：bash %s [--gui] [新航次标签]\n' "$0" >&2; exit 2
fi
if [[ "$ASTRA_STATIC_DEMO_GUI" == true && -z "${DISPLAY:-}" ]]; then
  printf '图形模式需要在本机桌面终端运行（DISPLAY 未设置）。\n' >&2; exit 2
fi
ASTRA_STATIC_DEMO_LABEL="${1:-$(date +%Y%m%d_%H%M%S)}"
ASTRA_STATIC_DEMO_DATA="$ASTRA_STATIC_DEMO_ROOT/half_gs_dataset_runs/warehouse_half_static_$ASTRA_STATIC_DEMO_LABEL"
ASTRA_STATIC_DEMO_MODEL="$ASTRA_STATIC_DEMO_ROOT/gsplat_runs/warehouse_half_static_$ASTRA_STATIC_DEMO_LABEL"
ASTRA_STATIC_DEMO_PLAN="$ASTRA_STATIC_DEMO_ROOT/half_gs_plans/warehouse_half_static_20261103_checked"
if [[ ! -f "$ASTRA_STATIC_DEMO_PLAN/inspection.json" ]]; then
  printf '先生成三层检查航线，见docs/STATIC_HALF_WAREHOUSE_HEIGHT_CN.md。\n' >&2;exit 1
fi
ASTRA_STATIC_CAPTURE_ARGS=()
if [[ "$ASTRA_STATIC_DEMO_GUI" != true ]]; then ASTRA_STATIC_CAPTURE_ARGS+=(--headless); fi
bash "$ASTRA_STATIC_DEMO_ROOT/scripts/run_sh/half_warehouse_lio_dataset.sh" "${ASTRA_STATIC_CAPTURE_ARGS[@]}" --static \
  --plan "$ASTRA_STATIC_DEMO_PLAN/inspection.json" --reference-dir "$ASTRA_STATIC_DEMO_PLAN" \
  --output-dir "$ASTRA_STATIC_DEMO_DATA"
bash "$ASTRA_STATIC_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_STATIC_DEMO_ROOT/integration/half_gs/static_pipeline.py" "$ASTRA_STATIC_DEMO_DATA" \
  --output "$ASTRA_STATIC_DEMO_MODEL" --partial-baseline
printf '静态模型：%s\n覆盖缺口见数据目录coverage.json。\n' "$ASTRA_STATIC_DEMO_MODEL"
if [[ "$ASTRA_STATIC_DEMO_GUI" == true ]]; then
  bash "$ASTRA_STATIC_DEMO_ROOT/scripts/run_sh/half_gs_python.sh" \
    "$ASTRA_STATIC_DEMO_ROOT/integration/lio_gs/desktop_viewer.py" "$ASTRA_STATIC_DEMO_MODEL" \
    --title 'AstraDrone Static Half Warehouse 3DGS' \
    --report-dir "$ASTRA_STATIC_DEMO_MODEL/native_viewer"
fi
