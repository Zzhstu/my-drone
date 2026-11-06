#!/usr/bin/env bash
set -euo pipefail
ASTRA_GS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_GS_ENV="$ASTRA_GS_ROOT/.envs/roomgs"
export CUDA_HOME="$ASTRA_GS_ENV"
export PATH="$ASTRA_GS_ENV/bin:$PATH"
export CPATH="$ASTRA_GS_ENV/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$ASTRA_GS_ENV/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$ASTRA_GS_ENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAX_JOBS=1 TORCH_CUDA_ARCH_LIST=12.0 CXX=/usr/bin/g++ CC=/usr/bin/gcc
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
unset PYTHONPATH
if [[ "${1:-}" == "--self-test" ]]; then
  shift
  exec "$ASTRA_GS_ENV/bin/python" "$ASTRA_GS_ROOT/integration/lio_gs/smoke.py" "${1:-$ASTRA_GS_ROOT/evaluation/lio_gsplat_20261102/mask_cuda_smoke.json}"
fi
exec "$ASTRA_GS_ENV/bin/python" "$ASTRA_GS_ROOT/integration/lio_gs/train.py" "$@"
