#!/usr/bin/env bash
set -euo pipefail
# Usage:
#   bash train_ultralytics_yolo11_yolo26.sh /path/to/data.yaml 100 640 16 0
DATA_YAML=${1:?"need dataset yaml, e.g. /home/wj/datasets/p450_target/data.yaml"}
EPOCHS=${2:-100}
IMGSZ=${3:-640}
BATCH=${4:-16}
DEVICE=${5:-0}
PROJECT=${PROJECT:-runs/p450_yolo_compare}

python3 -m pip install -U ultralytics

yolo detect train model=yolo11n.pt data=${DATA_YAML} imgsz=${IMGSZ} epochs=${EPOCHS} batch=${BATCH} device=${DEVICE} project=${PROJECT} name=yolo11n_p450

yolo detect train model=yolo26n.pt data=${DATA_YAML} imgsz=${IMGSZ} epochs=${EPOCHS} batch=${BATCH} device=${DEVICE} project=${PROJECT} name=yolo26n_p450

echo "Training finished. Best weights:"
echo "  ${PROJECT}/yolo11n_p450/weights/best.pt"
echo "  ${PROJECT}/yolo26n_p450/weights/best.pt"
