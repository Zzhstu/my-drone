#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
from ultralytics import YOLO

parser = argparse.ArgumentParser(description='Export YOLO11/YOLO26 Ultralytics .pt to TensorRT .engine on the target NVIDIA machine.')
parser.add_argument('--weights', required=True, help='best.pt or pretrained pt')
parser.add_argument('--imgsz', type=int, default=640)
parser.add_argument('--device', default=0)
parser.add_argument('--half', action='store_true', help='FP16 export; recommended on NVIDIA GPU if supported')
parser.add_argument('--int8', action='store_true', help='INT8 export; needs calibration data yaml')
parser.add_argument('--data', default=None, help='dataset yaml for INT8 calibration')
args = parser.parse_args()

model = YOLO(args.weights)
kwargs = dict(format='engine', imgsz=args.imgsz, device=args.device)
if args.half:
    kwargs['half'] = True
if args.int8:
    kwargs['int8'] = True
    if args.data:
        kwargs['data'] = args.data
print('Export args:', kwargs)
model.export(**kwargs)
