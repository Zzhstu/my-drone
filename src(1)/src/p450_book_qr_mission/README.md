# p450_book_qr_mission

基于你当前机载工作空间的固定场景二维码巡检任务包。

## 功能

- 复用 `p450_experiment` 的 MID360 + FAST_LIO 室内定位与建图链路。
- 复用 `spirecv_ros` 的 `gimbal_server` 提供 G1 图像流。
- 使用 `prometheus_msgs/UAVCommand` 控制 P450 按固定航点飞行。
- 在每个扫描航点调用 OpenCV `QRCodeDetector` 解码二维码文本。
- 兼容 `book_qr_demo` 的二维码文本格式，并导出 CSV / Excel。
- 退出 all-in-one launch 时，FAST_LIO 会把点云地图保存到 `Prometheus/Modules/FAST_LIO/PCD/`。

## 包内容

- `scripts/book_qr_patrol_mission.py`：主任务节点
- `scripts/pose_sampler.py`：手动飞到观察位后，打印当前位姿为 YAML 航点配置
- `launch/mid360_mapping_save.launch`：强制开启 PCD 保存的 FAST_LIO 启动文件
- `launch/book_qr_patrol_mission_only.launch`：当基础链路已经手动启动时，只启动任务节点
- `launch/book_qr_patrol_all_in_one.launch`：一条命令启动飞控 / MID360 / FAST_LIO / G1 / 任务节点
- `config/book_qr_patrol_waypoints.example.yaml`：航点模板

## 上机流程

1. 把整个 `p450_book_qr_mission` 文件夹复制到 `~/p450_experiment/src/`。
2. 编译：
   ```bash
   cd ~/p450_experiment
   catkin_make
   source devel/setup.bash
   ```
3. 确保 PCD 输出目录存在：
   ```bash
   mkdir -p ~/p450_experiment/src/Prometheus/Modules/FAST_LIO/PCD
   ```
4. 先手动采点，再改 `config/book_qr_patrol_waypoints.example.yaml`。
5. 一键启动：
   ```bash
   roslaunch p450_book_qr_mission book_qr_patrol_all_in_one.launch \
     waypoints_file:=/home/nvidia/p450_experiment/src/p450_book_qr_mission/config/book_qr_patrol_waypoints.example.yaml
   ```
6. 解锁飞机，并切到 `COMMAND_CONTROL`。
7. 任务结束后，在启动 launch 的终端按 `Ctrl+C`，FAST_LIO 会保存 PCD。

## 采样航点

先手动飞到某个安全观察位，再执行：

```bash
rosrun p450_book_qr_mission pose_sampler.py _uav_id:=1 _name:=front_left _stage:=front _scan:=true
```

脚本会打印一行 YAML，可直接粘贴到 `waypoints:` 下面。

## 结果输出

默认输出到：

```text
~/p450_results/book_qr_patrol/<时间戳>/
```

里面有：

- `book_qr_patrol_summary.csv`
- `book_qr_patrol_summary.xlsx`
- `snapshots/*.jpg`
- `summary.txt`

## PCD 输出

默认输出到：

```text
~/p450_experiment/src/Prometheus/Modules/FAST_LIO/PCD/scans.pcd
```

如果把 `pcd_save_interval` 改成正整数，比如 100，则会分段输出多个 `scans_*.pcd`。
