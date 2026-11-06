#!/usr/bin/env bash
set -euo pipefail
ASTRA_HALF_GS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ASTRA_HALF_GS_ENV="$ASTRA_HALF_GS_ROOT/.envs/roomgs"
export CUDA_HOME="$ASTRA_HALF_GS_ENV"
export PATH="$ASTRA_HALF_GS_ENV/bin:$PATH"
export CPATH="$ASTRA_HALF_GS_ENV/targets/x86_64-linux/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$ASTRA_HALF_GS_ENV/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$ASTRA_HALF_GS_ENV/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MAX_JOBS=1 TORCH_CUDA_ARCH_LIST=12.0 CXX=/usr/bin/g++ CC=/usr/bin/gcc
export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
unset PYTHONPATH
exec "$ASTRA_HALF_GS_ENV/bin/python" "$@"
