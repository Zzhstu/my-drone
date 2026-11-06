#!/usr/bin/env bash
set -euo pipefail
# Usage:
#   bash train_original_yolov5.sh /path/to/data.yaml 100 640 16 0 /home/wj/yolov5
DATA_YAML=${1:?"need dataset yaml"}
EPOCHS=${2:-100}
IMGSZ=${3:-640}
BATCH=${4:-16}
DEVICE=${5:-0}
YOLOV5_DIR=${6:-$HOME/yolov5}
PROJECT=${PROJECT:-runs/p450_yolo_compare}

if [ ! -d "$YOLOV5_DIR" ]; then
  git clone https://github.com/ultralytics/yolov5 "$YOLOV5_DIR"
fi
cd "$YOLOV5_DIR"
python3 -m pip install -r requirements.txt
python3 train.py --weights yolov5s.pt --data "$DATA_YAML" --img "$IMGSZ" --epochs "$EPOCHS" --batch-size "$BATCH" --device "$DEVICE" --project "$PROJECT" --name yolov5s_p450

echo "Training finished. Best weights: ${YOLOV5_DIR}/${PROJECT}/yolov5s_p450/weights/best.pt"
