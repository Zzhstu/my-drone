#!/usr/bin/env bash
set -euo pipefail
ASTRA_HALF_OPEN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_HALF_MODEL="${1:-$ASTRA_HALF_OPEN_ROOT/gsplat_runs/warehouse_half_20261103_masked_semantic}"
ASTRA_HALF_UNIT=astradrone-half-gs-desktop.service
if systemctl --user is-active --quiet "$ASTRA_HALF_UNIT"; then
  printf '半仓库原生3DGS窗口已运行，可用Alt+Tab切换。\n'
  exit 0
fi
if [[ ! -f "$ASTRA_HALF_MODEL/gsplat/model.pt" ]]; then
  printf '模型尚未训练完成：%s\n' "$ASTRA_HALF_MODEL" >&2
  exit 1
fi
systemd-run --user --collect --unit="$ASTRA_HALF_UNIT" \
  --description='Native half warehouse masked 3DGS map' \
  --property="WorkingDirectory=$ASTRA_HALF_OPEN_ROOT" \
  --setenv="DISPLAY=${DISPLAY:-:1}" --setenv="XAUTHORITY=${XAUTHORITY:-}" \
  /usr/bin/bash "$ASTRA_HALF_OPEN_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_HALF_OPEN_ROOT/integration/lio_gs/desktop_viewer.py" "$ASTRA_HALF_MODEL" \
  --title 'AstraDrone Half Warehouse 3DGS' \
  --report-dir "$ASTRA_HALF_OPEN_ROOT/evaluation/half_gs_20261103/native_interactive"
