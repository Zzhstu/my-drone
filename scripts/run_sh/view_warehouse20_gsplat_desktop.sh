#!/usr/bin/env bash
set -euo pipefail
ASTRA_NATIVE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_NATIVE_ENV="$ASTRA_NATIVE_ROOT/.envs/roomgs"
export CUDA_HOME="$ASTRA_NATIVE_ENV"
export PATH="$ASTRA_NATIVE_ENV/bin:$PATH"
export CPATH="$ASTRA_NATIVE_ENV/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$ASTRA_NATIVE_ENV/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$ASTRA_NATIVE_ENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAX_JOBS=1 TORCH_CUDA_ARCH_LIST=12.0 CXX=/usr/bin/g++ CC=/usr/bin/gcc
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
unset PYTHONPATH
exec "$ASTRA_NATIVE_ENV/bin/python" "$ASTRA_NATIVE_ROOT/integration/lio_gs/desktop_viewer.py" \
  "$ASTRA_NATIVE_ROOT/gsplat_runs/warehouse20_lio_20261102_hq" "$@"
