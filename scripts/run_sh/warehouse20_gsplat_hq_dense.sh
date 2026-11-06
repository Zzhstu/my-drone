#!/usr/bin/env bash
set -euo pipefail
ASTRA_DENSE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$ASTRA_DENSE_ROOT/scripts/run_sh/warehouse20_gsplat.sh" \
  --initialize-model "$ASTRA_DENSE_ROOT/gsplat_runs/warehouse20_lio_20261102_hq/gsplat/model.pt" \
  --steps 8000 --max-gaussians 800000 --output-gib 3 \
  --means-lr 2e-5 --means-lr-final 1e-6 --refine-until 3500 --grow-grad .0001 \
  --min-scale .0005 --eps2d .1 --edge-weight .15 --absgrad "$@"
