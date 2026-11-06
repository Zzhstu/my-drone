# 原cangku半面积仓库自主探索

直接裁剪现有仓库的左半部分，保留仓库外观和现有货架间距。原`cangku.world`和原仓库/小办公室入口保留。

| 项目 | 原仓库 | 半面积仓库 |
|---|---:|---:|
| 主体长×宽 | 50.5×27m | 25.25×27m |
| 主体名义占地 | 1363.5m² | 681.75m² |
| 货架数量 | 52 | 24 |
| 货架碰撞尺寸 | 3.6×0.6×1.8m | 保持 |
| 顶棚下表面 | 世界Z约2.5m | 保持 |
| 起飞点 | 世界(-6,-4) | 保持 |
| 导航速度上限 | 0.30m/s | 0.40m/s |

面积口径是屋顶盒的XY投影矩形，不是可飞自由面积，也不含无限地面。长边减半，短边保持；保留的货架位姿和尺寸均不缩放。移动右墙、缩短长墙与顶棚，保留原左侧入口。原货架狭缝不保证在现有安全包络下可通行。

## 启动

先正常结束其他ROS/PX4/Gazebo演示，再运行：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/half_warehouse_exploration_demo.sh
```

默认打开Gazebo和RViz；自动等待地面/航向稳定、相对起飞1.2m、自主选点、观察、返航驻留并保存。运行副本会加入一名可移动的站立行人。无人机导航速度上限从0.30提高到0.40m/s，安全净距仍为0.80m，风险停车和紧急阈值不变。此场景不启用语义网络或回环。

默认模式中，行人先静止；无人机起飞并接近后，行人沿左侧通道自动走一轮，结束后可键盘接管。手动模式会跳过自动行走，便于启动后直接控制：

```bash
# 主终端：行人保持静止并立刻开放键盘控制
bash scripts/run_sh/half_warehouse_exploration_demo.sh --manual-pedestrian

# 第二个终端：控制0号行人
bash docs/演示启动命令/行人键盘控制.sh
```

W/S沿世界Y轴、A/D沿世界X轴，空格停止，Q退出；键盘命令超过0.4秒没有刷新会自动停。行人节点按仓库墙体/货架碰撞盒拒绝穿障移动。移动的是站立模型，不是骨骼动画；碰撞盒只保护行人移动，不提供给无人机感知或规划。默认自动行走结束后也可在第二终端运行同一键盘命令。

```bash
# 无图形界面
bash scripts/run_sh/half_warehouse_exploration_demo.sh --headless

# 完成返航保存后保留窗口，直到Ctrl-C
bash scripts/run_sh/half_warehouse_exploration_demo.sh --keep-open

# 两目标短程功能检查，明确为部分探索
bash scripts/run_sh/half_warehouse_exploration_demo.sh --return-after-goals 2 --test-seconds 360
```

首次构建方式与[小办公室说明](SMALL_INDOOR_DEMO_CN.md)相同。新入口共享已经构建的小室内运行器和速度坐标转换节点，通过`--scene cangku-half`选择新场景；默认小办公室入口仍选择office。既有FAST-LIO、严格证据地图、探索器、风险和控制算法不改。安全净距0.80m、导航上限0.30m/s和95%已测表面质量门限保持。

探索预算600仿真秒或1200墙钟秒，补扫预算240仿真秒，返航超时600仿真秒；运行器总墙钟保护2400秒。预算退出是部分探索，不能从保存成功推断完整覆盖。任务成功表示确认返航驻留并保存，分别读取`exploration_complete`、`coverage_complete`判断探索报告。速度上限提升尚需完整动态场景任务验证，不能仅凭配置推断任务耗时或安全性能提高。

## 文件

- 新world：`simulation/astra_gazebo_worlds/cangku_half.world`。
- 裁剪元数据：`simulation/astra_gazebo_worlds/cangku_half.json`。
- 生成器：`scripts/half_warehouse_scene.py`（fastlio_bridge包内）。按原文件SHA固定来源，原文件变化时要求重新审查裁剪几何。
- 本轮运行产物：项目根目录`half_warehouse_demo_runs/warehouse_half_日期_时间_*`，含维护地图PCD、元数据、状态/估计轨迹、坐标偏差、结果JSON与进程日志。
- `_shelf_audit.json`仅评价当前24个货架的碰撞盒侧面参考支持，含被遮挡或不可达面，不是全三维完整率；评估在仿真结束后运行，不把几何提供给探索器。
- [前后布局对比图](../../../../docs/演示验证/2026-10-16_半面积仓库/warehouse_area_comparison.png)仅为几何预览，不是实际重建图或预设航线。

## 验证记录

场景SDF校验、4项新增裁剪测试与13项小办公室回归通过；展开启动参数确认原安全/质量设置不变。首次尝试飞行时发现已有用户小办公室仿真正在运行，预检查拒绝并发启动，未关闭用户进程。随后等待原任务自然结束，已完成Gazebo+RViz真实SITL两目标短测：到达2/2、自主返航驻留保存58030点，航程21.04m、地图返航XY误差0.185m、退出0。属于`explicit_goal_limit_partial`，两个complete标志false，未验证默认完整任务。原办公室离线审计也重算一致。详细证据见[本轮报告](../../../../docs/演示验证/2026-10-16_半面积仓库/验证报告.md)。

关闭阶段仍复现原有仿真启动组原生段错误（具体进程/插件未定位）；本次地图已在关闭前保存、所有本轮进程最终退出。该既有卸载问题未在本轮修复。
