#!/usr/bin/env bash
set -euo pipefail
ASTRA_DETAIL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_DETAIL_ENV="$ASTRA_DETAIL_ROOT/.envs/roomgs"
export CUDA_HOME="$ASTRA_DETAIL_ENV"
export PATH="$ASTRA_DETAIL_ENV/bin:$PATH"
export CPATH="$ASTRA_DETAIL_ENV/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$ASTRA_DETAIL_ENV/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$ASTRA_DETAIL_ENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAX_JOBS=1 TORCH_CUDA_ARCH_LIST=12.0 OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
export CXX=/usr/bin/g++ CC=/usr/bin/gcc
unset PYTHONPATH
exec "$ASTRA_DETAIL_ENV/bin/python" "$ASTRA_DETAIL_ROOT/integration/lio_gs/viewer.py" \
  "${1:-$ASTRA_DETAIL_ROOT/gsplat_runs/warehouse20_lio_20261102_hq}" \
  --baseline "$ASTRA_DETAIL_ROOT/gsplat_runs/warehouse20_lio_20261102_640" --port "${2:-8891}"
