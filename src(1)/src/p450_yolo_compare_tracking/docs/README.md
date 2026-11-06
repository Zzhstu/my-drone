# p450_yolo_compare_tracking

用于 P450 + G1 吊舱的地面安全测试包。它不修改原 `p450_experiment`、`p450_mission` 和 `spirecv_ros`，新增以下能力：

1. 原 SpireCV YOLOv5 + NanoTrack 链路的不起飞测试 launch。
2. YOLO11 / YOLO26 / 自训练 YOLOv5 的统一 ROS 检测跟踪节点。
3. 发布与 SpireCV 兼容的 `/uav1/spirecv/target`，因此可以复用原 `gimbal_server` 让 G1 吊舱把目标保持在画面中心。
4. CSV 日志记录，用于 FPS、目标中心误差、跟踪状态对比。

## 安装

```bash
cd ~/p450_experiment/src
unzip ~/Downloads/p450_yolo_compare_tracking_v2.zip
cd ~/p450_experiment
catkin_make
source devel/setup.bash
```

## 原机载 YOLOv5 不起飞测试

```bash
roslaunch p450_yolo_compare_tracking g1_original_spirecv_no_flight.launch \
  start_gimbal_server:=true \
  start_detector:=true \
  start_logger:=true
```

中心点击锁定目标：

```bash
rosrun p450_yolo_compare_tracking send_spirecv_click.py _x:=0.5 _y:=0.5 _mouse:=left
```

右键/重置跟踪：

```bash
rosrun p450_yolo_compare_tracking send_spirecv_click.py _mouse:=right
```

## YOLO11 / YOLO26 不起飞测试

```bash
roslaunch p450_yolo_compare_tracking g1_yolo_no_flight_compare.launch \
  model_name:=yolo11n \
  model_path:=/home/amov/SpireCV/models/yolo11n.pt \
  backend:=ultralytics \
  start_gimbal_server:=true \
  start_logger:=true
```

YOLO26：

```bash
roslaunch p450_yolo_compare_tracking g1_yolo_no_flight_compare.launch \
  model_name:=yolo26n \
  model_path:=/home/amov/SpireCV/models/yolo26n.pt \
  backend:=ultralytics \
  start_gimbal_server:=true \
  start_logger:=true
```

原始 YOLOv5 自训练 `.pt`：

```bash
roslaunch p450_yolo_compare_tracking g1_yolo_no_flight_compare.launch \
  model_name:=yolov5s_custom \
  model_path:=/home/amov/yolov5/runs/p450_yolo_compare/yolov5s_p450/weights/best.pt \
  backend:=yolov5_hub \
  yolov5_repo:=/home/amov/yolov5 \
  start_gimbal_server:=true \
  start_logger:=true
```

## 训练脚本

见 `tools/` 目录：

```bash
bash tools/train_ultralytics_yolo11_yolo26.sh /path/to/data.yaml 100 640 16 0
bash tools/train_original_yolov5.sh /path/to/data.yaml 100 640 16 0 /home/wj/yolov5
python3 tools/export_ultralytics_engine.py --weights /path/to/best.pt --imgsz 640 --device 0 --half
```
