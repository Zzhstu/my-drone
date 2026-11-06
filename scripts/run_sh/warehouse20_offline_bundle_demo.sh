#!/usr/bin/env bash
set -euo pipefail
ASTRA_BUNDLE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if (( $# < 2 )); then
  echo 'Usage: warehouse20_offline_bundle_demo.sh PRIMARY_CAPTURE NEW_OUTPUT [SUPPLEMENTAL_CAPTURE ...]' >&2
  exit 2
fi
ASTRA_PRIMARY_CAPTURE="$1"
ASTRA_BUNDLE_OUTPUT="$2"
shift 2
ASTRA_BUNDLE_PY="$ASTRA_BUNDLE_ROOT/.envs/roomgs/bin/python"
ASTRA_BUNDLE_MODULE="$ASTRA_BUNDLE_ROOT/integration/roomgs"
run_bundle_python() {
  env -u PYTHONPATH OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH="$ASTRA_BUNDLE_ROOT/.envs/roomgs/lib:${LD_LIBRARY_PATH:-}" "$ASTRA_BUNDLE_PY" "$@"
}
run_bundle_python "$ASTRA_BUNDLE_MODULE/multisession.py" create "$ASTRA_BUNDLE_OUTPUT" --source "$ASTRA_PRIMARY_CAPTURE"
for ASTRA_AUX_CAPTURE in "$@"; do
  run_bundle_python "$ASTRA_BUNDLE_MODULE/multisession.py" add "$ASTRA_BUNDLE_OUTPUT" --source "$ASTRA_AUX_CAPTURE"
done
run_bundle_python "$ASTRA_BUNDLE_MODULE/recheck.py" "$ASTRA_BUNDLE_OUTPUT"
exec bash "$ASTRA_BUNDLE_ROOT/scripts/run_sh/roomgs_demo.sh" --train-only "$ASTRA_BUNDLE_OUTPUT" --steps 12000
