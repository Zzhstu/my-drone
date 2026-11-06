#!/usr/bin/env bash
set -euo pipefail
# Usage: bash val_ultralytics_models.sh /path/data.yaml /path/yolo11_best.pt /path/yolo26_best.pt
DATA_YAML=${1:?"need dataset yaml"}
shift
for W in "$@"; do
  echo "== Validate $W =="
  yolo detect val model="$W" data="$DATA_YAML" imgsz=640 batch=1 device=0 plots=True
done
