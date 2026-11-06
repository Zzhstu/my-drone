#!/usr/bin/env bash
set -euo pipefail
ASTRA_OPEN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_OPEN_UNIT=astradrone-gsplat-desktop.service
if systemctl --user is-active --quiet "$ASTRA_OPEN_UNIT"; then
  printf '3DGS桌面窗口已运行，可用Alt+Tab切换到 AstraDrone 3DGS Desktop。\n'
  exit 0
fi
if [[ -z "${DISPLAY:-}" ]]; then
  printf '请在本机图形桌面的终端中运行此入口。\n' >&2
  exit 1
fi
systemd-run --user --collect --unit="$ASTRA_OPEN_UNIT" \
  --description='AstraDrone native full-SH 3DGS desktop viewer' \
  --property="WorkingDirectory=$ASTRA_OPEN_ROOT" \
  --setenv="DISPLAY=$DISPLAY" --setenv="XAUTHORITY=${XAUTHORITY:-}" \
  /usr/bin/bash "$ASTRA_OPEN_ROOT/scripts/run_sh/view_warehouse20_gsplat_desktop.sh" "$@"
printf '正在打开原生3DGS桌面窗口。关闭窗口或按Esc退出。\n'
