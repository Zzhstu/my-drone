#!/usr/bin/env bash
# Isolated dependency only; preserve ROS/system Python and existing demos.
set -euo pipefail
ASTRA_DUFO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
python3 -m venv --system-site-packages "$ASTRA_DUFO_ROOT/.venvs/dufomap"
"$ASTRA_DUFO_ROOT/.venvs/dufomap/bin/python" -m pip install --disable-pip-version-check --no-deps 'dufomap==1.1.1'
"$ASTRA_DUFO_ROOT/.venvs/dufomap/bin/python" -c 'import sys,json;sys.path.insert(0,sys.argv[1]);from dufomap_core import native_provenance;print(json.dumps(native_provenance(),indent=2))' "$ASTRA_DUFO_ROOT/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts"
