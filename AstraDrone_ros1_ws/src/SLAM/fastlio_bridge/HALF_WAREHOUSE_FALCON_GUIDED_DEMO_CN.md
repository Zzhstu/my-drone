# 半仓库 15 cm 覆盖分区引导自主探索 demo

这是**本项目自写的 FALCON 思路适配版**，不是原作者 FALCON 源码的复现。它保留半仓库 Gazebo/PX4、FAST-LIO/MID-360、15 cm 静态地图、行人、原有风险控制、自动返航及地图保存；单独改变前沿选点为“覆盖分区全局排序 → 区内局部视点”，并将固定飞行高度的规划格从 20 cm 设为 15 cm。不会把 Gazebo 真值送入规划器，也不启动第二个飞控发布者。

## 为什么 15 cm 可以试，但不能直接等同 FALCON 的体素地图

本项目 `config/static_map.yaml` 的 `voxel_size: 0.15` 是**三维占据点云维护的抽样间距**。原探索器 `config/warehouse_exploration.yaml` 的 `resolution: 0.20` 是**固定高度二维路径栅格**。本 demo 将后者设为 0.15 m，以减少与前者的网格尺度不一致；规划器仍从 `/uav1/local_free_space` 得到自由证据，并将维护地图与当前扫描投影到飞行高度带。点云体素自身不包含可通行的未知/自由/占据三态，不能单独当成 FALCON 三维 TSDF/ESDF 地图。

[FALCON 作者仓库](https://github.com/HKUST-Aerial-Robotics/FALCON)的 ROS1 示例中，`voxel_mapping.yaml` 配细/粗分辨率 0.10/0.20 m，`astar.yaml` 另有 0.10 m 细搜索以及更粗搜索，覆盖分区又是独立尺度。改为 0.15 m 从数据结构上可行，但须重新调节以“格数”为单位的阈值、膨胀/视线取样、地图体积与安全间隙。上游 `map_server.cpp` 对细分辨率实际读取参数名 `resolutionf_fine`，与 YAML 的 `resolution_fine` 拼写不同；直接改 YAML 不能证明上游实际用了新值。本 demo 没有编译或调用上游实现。

以相同三维边界计算，0.15 m 对比 0.10 m 的体素数量理论比例为 `(0.10/0.15)^3≈0.296`，只表示格数数量级，不保证计算时间按同一比例下降。窄通道与细障碍可能更易被离散化抹平，机身净距仍固定 0.80 m，不能因分辨率变粗而降低。

## 启动与观察

先退出已有 ROS/PX4/Gazebo 演示，终端运行：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/half_warehouse_falcon_guided_demo.sh --headless
```

有图形桌面时去掉 `--headless` 可同时看 Gazebo 与 RViz。短程生命周期验证：

```bash
bash scripts/run_sh/half_warehouse_falcon_guided_demo.sh --headless --return-after-goals 2
```

第二条只检验新选点→飞行→返航→保存，属于**目标限额部分探索**，不是完整率验收。可加 `--manual-pedestrian` 让行人待命，由另一个终端使用现有 `docs/演示启动命令/行人键盘控制.sh` 控制；默认让行人自动走一轮。运行器保留墙钟硬上限，`Ctrl-C` 请求保存当前地图并清理本轮进程。

输出在 `half_warehouse_falcon_guided_runs/warehouse_half_falcon_guided_时间戳_*`，重点查看 `_result.json`、`_exploration.json`、`_final_map.pcd`、`_shelf_audit.json` 与进程日志。探索状态的 `planner_mode=coverage_guided`、`planning_resolution_m=0.15`、`coverage_guidance.route_regions` 用于确认配置与选点模式；`coverage_guidance` 是全局顺序提示，实际路线每周期继续由原安全栅格规划。

## 算法边界与对照

`coverage_path_guidance.py` 只使用已观测安全格和已有前沿候选：按世界坐标将候选划入 2.5 m 分区，对最多 8 个候选分区计算安全栅格上的真实可达距离，用一步前瞻选择全局访问顺序，再从首分区挑局部视点。现有避障、回家路径、补扫与验收代码仍工作。相同分区内多视点收益有限加权，避免“点采样密”被误当作几何增益。这里没有复现 FALCON 的三维自由空间分解、完整覆盖路径优化、B 样条轨迹或作者 EPEE。

当前任务仍为固定飞行高度，`coverage_complete=false`；货架侧面支持率不是全三维完整率。比较原半仓库 demo 时需要同一场景/初始位姿/行人脚本、相同目标限额与重复样本，分开报告仿真秒、真实墙钟秒、已观察表面质量、离线货架侧面代理、碰撞/返航以及计算时延；规划分辨率从 0.20 改到 0.15 也是变量，不能把所有时间差归给覆盖引导。

## 本次短程实测（2026-10-17）

已通过 `catkin_make --pkg fastlio_bridge -j2 -l2`、探索相关 62 项回归测试和新模块 3 项单测；`roslaunch --dump-params` 确认 0.15 m、覆盖分区引导、原 0.80 m 净距和 0.60 m/s 速度上限。以 `--headless --return-after-goals 2` 跑了真实 PX4/Gazebo/FAST-LIO SITL，结果前缀为 `half_warehouse_falcon_guided_runs/warehouse_half_falcon_guided_20261017_171224`：选出并到达 2 个目标，返航驻留并保存 55,438 点 XYZ PCD；总墙钟 170.23 s，任务仿真 58.64 s，计算 P95 73.96 ms；货架侧面离线支持率 28.32%。这是很短的生命周期测试，**不能用 28.32% 与此前完整航次的约 99% 直接比较算法优劣**。结果JSON的 `finish_reason=explicit_goal_limit_partial`、`coverage_complete=false`。结束时 Gazebo 退出出现异常转储，任务结果已保存，原因尚未归因于新规划器；本轮自身残留进程已清理。

## 本次完整航次实测（2026-10-17）

运行 `bash scripts/run_sh/half_warehouse_falcon_guided_demo.sh --headless`，无目标数量限额；结果前缀为 `half_warehouse_falcon_guided_runs/warehouse_half_falcon_guided_20261017_171611`。结果 JSON 显示 `termination=returned_home_and_saved`、`task_success=true`、`map_saved=true`、`home_confirmed=true`，返航终点距起点约 0.179 m。23 个目标被选出，21 个到达；保存的 `map` 坐标系 XYZ PCD 有 129,518 点。任务约 500.13 仿真秒、805.70 s 任务墙钟；含启动、保存和清理的总现实耗时 856.97 s（约 14 分 17 秒）。规划计算 P95 为 103.74 ms。

离线货架侧面参考 4,704 个采样点中 4,699 个有点云支持，代理支持率 99.89%。这个审计使用仿真场景参考几何，仅在离线评价端使用；不是完整三维覆盖率。任务是按宽松的已观测地图质量条件结束，`finish_reason=observed_map_accepted_partial`，`coverage_complete=false`、`exploration_complete=false`，结果中的 `map_quality_accepted=false` 也不能改写为完整率通过。任务结束时仍有 9 个有用前沿。该单次航次证明新 demo 能自然停机、返航、保存并获得较高货架侧面代理支持率；未证明比原半仓库规划器或原版 FALCON 更快、更完整。

点云可用 `pcl_viewer half_warehouse_falcon_guided_runs/warehouse_half_falcon_guided_20261017_171611_final_map.pcd` 查看，轨迹概览是同前缀 `_map_and_trajectory.png`。行人自动脚本的实际移动情况应以同前缀 `_walk_once.log` 和 `_pedestrian.log` 为准；本次未做动态行人真值及碰撞接触的独立验收。
