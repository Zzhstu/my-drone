#!/usr/bin/env bash
set -euo pipefail
ASTRA_STATIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_STATIC_MODEL="${1:-$ASTRA_STATIC_ROOT/gsplat_runs/warehouse_half_static_20261103_detail}"
ASTRA_STATIC_MODEL="$(realpath "$ASTRA_STATIC_MODEL")"
ASTRA_STATIC_UNIT=astradrone-static-half-gs-desktop.service
if [[ ! -f "$ASTRA_STATIC_MODEL/gsplat/model.pt" ]]; then
  printf '静态模型尚未完成：%s\n' "$ASTRA_STATIC_MODEL" >&2; exit 1
fi
if systemctl --user is-active --quiet "$ASTRA_STATIC_UNIT"; then
  ASTRA_STATIC_PID="$(systemctl --user show "$ASTRA_STATIC_UNIT" --property=MainPID --value)"
  ASTRA_STATIC_RUNNING="$(python3 - "$ASTRA_STATIC_PID" <<'PY'
from pathlib import Path
import sys
try:
    arguments=Path('/proc/'+sys.argv[1]+'/cmdline').read_bytes().decode().split('\0')
    for i,argument in enumerate(arguments[:-1]):
        if argument.endswith('/desktop_viewer.py'):
            print(Path(arguments[i+1]).resolve());break
except (OSError,ValueError):
    pass
PY
)"
  if [[ "$ASTRA_STATIC_RUNNING" == "$ASTRA_STATIC_MODEL" ]]; then
    printf '此静态模型窗口已运行，可用Alt+Tab切换：%s\n' "$ASTRA_STATIC_MODEL"; exit 0
  fi
  printf '切换静态模型窗口：%s\n' "$ASTRA_STATIC_MODEL"
  systemctl --user stop "$ASTRA_STATIC_UNIT"
fi
ASTRA_STATIC_REPORT="$ASTRA_STATIC_ROOT/evaluation/native_${ASTRA_STATIC_MODEL##*/}"
systemd-run --user --collect --unit="$ASTRA_STATIC_UNIT" \
  --property="WorkingDirectory=$ASTRA_STATIC_ROOT" \
  --setenv="DISPLAY=${DISPLAY:-:1}" --setenv="XAUTHORITY=${XAUTHORITY:-}" \
  /usr/bin/bash "$ASTRA_STATIC_ROOT/scripts/run_sh/half_gs_python.sh" \
  "$ASTRA_STATIC_ROOT/integration/lio_gs/desktop_viewer.py" "$ASTRA_STATIC_MODEL" \
  --title "AstraDrone Static Warehouse - ${ASTRA_STATIC_MODEL##*/}" \
  --report-dir "$ASTRA_STATIC_REPORT"
