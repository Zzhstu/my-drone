#!/usr/bin/env bash
set -euo pipefail
ASTRA_DETAIL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$ASTRA_DETAIL_ROOT/scripts/run_sh/warehouse20_gsplat.sh" \
  --initialize-model "$ASTRA_DETAIL_ROOT/gsplat_runs/warehouse20_lio_20261102_640/gsplat/model.pt" \
  --steps 18000 --max-gaussians 800000 --output-gib 3 \
  --means-lr 5e-5 --means-lr-final 1e-6 --refine-until 8500 --grow-grad .0008 \
  --min-scale .0005 --eps2d .1 --edge-weight .15 --absgrad "$@"
