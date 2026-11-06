#!/usr/bin/env bash
set -euo pipefail
ASTRA_COMPARE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_COMPARE_ENV="$ASTRA_COMPARE_ROOT/.envs/roomgs"
export CUDA_HOME="$ASTRA_COMPARE_ENV"
export PATH="$ASTRA_COMPARE_ENV/bin:$PATH"
export CPATH="$ASTRA_COMPARE_ENV/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$ASTRA_COMPARE_ENV/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$ASTRA_COMPARE_ENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAX_JOBS=1 TORCH_CUDA_ARCH_LIST=12.0 CXX=/usr/bin/g++ CC=/usr/bin/gcc
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
unset PYTHONPATH
exec "$ASTRA_COMPARE_ENV/bin/python" "$ASTRA_COMPARE_ROOT/integration/lio_gs/compare.py" \
  "${1:-$ASTRA_COMPARE_ROOT/gsplat_runs/warehouse20_lio_20261102_640}" \
  "${2:-$ASTRA_COMPARE_ROOT/gsplat_runs/warehouse20_lio_20261102_hq}" \
  "${3:-$ASTRA_COMPARE_ROOT/evaluation/lio_gsplat_detail_20261102}"
