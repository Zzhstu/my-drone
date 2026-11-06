#!/usr/bin/env bash
# User-session background viewer; deliberately not enabled at boot/login.
set -euo pipefail
ASTRA_VIEW_SERVICE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_VIEW_SERVICE_UNIT=astradrone-gsplat-viewer.service
ASTRA_VIEW_SERVICE_ACTION="${1:-start}"

case "$ASTRA_VIEW_SERVICE_ACTION" in
  status) exec systemctl --user status "$ASTRA_VIEW_SERVICE_UNIT" --no-pager ;;
  stop) exec systemctl --user stop "$ASTRA_VIEW_SERVICE_UNIT" ;;
  logs) exec journalctl --user -u "$ASTRA_VIEW_SERVICE_UNIT" -n 60 --no-pager ;;
  start|restart) ;;
  *) printf '用法：bash %s [start|restart|stop|status|logs]\n' "$0" >&2; exit 2 ;;
esac

ASTRA_VIEW_SERVICE_LOAD="$(systemctl --user show "$ASTRA_VIEW_SERVICE_UNIT" --property=LoadState --value 2>/dev/null || true)"
if [[ "$ASTRA_VIEW_SERVICE_LOAD" == loaded ]]; then
  systemctl --user "$ASTRA_VIEW_SERVICE_ACTION" "$ASTRA_VIEW_SERVICE_UNIT"
else
  if ss -H -ltn 'sport = :8891' | rg -q .; then
    printf '8891端口已由其他进程监听，请先检查现有查看器；未启动重复服务。\n' >&2
    exit 1
  fi
  systemd-run --user --unit="$ASTRA_VIEW_SERVICE_UNIT" \
    --description='AstraDrone warehouse full-SH gsplat viewer' \
    --property="WorkingDirectory=$ASTRA_VIEW_SERVICE_ROOT" \
    --property=Restart=on-failure --property=RestartSec=5 \
    --property=StartLimitIntervalSec=60 --property=StartLimitBurst=3 \
    /usr/bin/bash "$ASTRA_VIEW_SERVICE_ROOT/scripts/run_sh/view_warehouse20_gsplat_hq.sh"
fi

for ASTRA_VIEW_SERVICE_ATTEMPT in {1..40}; do
  if curl --noproxy '*' --silent --fail --max-time 1 http://127.0.0.1:8891/ -o /dev/null; then
    printf '查看器已在独立后台运行：http://127.0.0.1:8891\n'
    exit 0
  fi
  if systemctl --user is-failed --quiet "$ASTRA_VIEW_SERVICE_UNIT"; then
    journalctl --user -u "$ASTRA_VIEW_SERVICE_UNIT" -n 30 --no-pager
    exit 1
  fi
  sleep 1
done
printf '服务尚未就绪，请查看日志：bash %s logs\n' "$0" >&2
exit 1
