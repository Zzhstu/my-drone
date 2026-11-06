# 小办公室自主探索演示

新增入口复用现有FAST-LIO2、严格观测证据地图维护、自主探索器、静动态风险和MAVROS控制器。不会替换原仓库、badcase/ours或行人演示。

## 场景来源

- GitHub：[macc-n/gazebo_worlds](https://github.com/macc-n/gazebo_worlds)。
- 固定提交：`989303039221f5f432e96f8ba2fdfb400db38f11`。
- 原文件：`gazebo/worlds/office_small.world`，约10×10米的单间办公室，墙高2.5米；有桌椅、柜子、沙发与窗洞。
- 本地归档：`simulation/astra_gazebo_worlds/small_office/`，包括原world、GPL-3.0许可、原README和`SOURCE.json`。
- 轻量适配保留19个顶层模型与14份原mesh，去掉桌面书本、杯子等杂物；保留所选模型的碰撞几何和位姿。SDF材质改成本地颜色，避免联网下载材质；物理步长采用原项目已用的1ms/1000Hz。
- 无屋顶的原场景不能用来验证天花板避障。窗洞/门不是为无人机放大的通道，现有安全净距会限制通行。

## 直接启动

先退出其他ROS/PX4/Gazebo演示。主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/small_indoor_exploration_demo.sh
```

默认打开Gazebo和RViz，自动等待地面位姿稳定、启动SLAM、起飞至相对1.2米，再自主选点与飞行。这个版本为静态室内验证，没有添加行人，没有启用语义网络、FreeDOM或回环。

首次尚未构建工作区时，先执行：

```bash
cd /home/a/AstraDroneOpen/AstraDrone_ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg fastlio_bridge -j2 -l2
```

新入口另外等待启动航向沉降，并加入独立的速度坐标转换器。首轮测试发现PX4在解锁时重新对齐航向，早期固定的SLAM地图轴可能与后来的PX4局部轴相差90度；现在由新鲜、近时配对的两端机体姿态计算偏航差，将原控制器的地图速度旋转到MAVROS局部ENU后发布。向飞控发送速度的话题仍只有一个发布者；转换输入过期或不配对时输出零速度。它不改变原规划、风险和控制算法，也不使用世界真值；当前只处理两种重力对齐坐标系之间的偏航，不是完整三维坐标复位补偿。

没有固定飞行路线，也不向探索器加载办公室平面图。场景几何仅供Gazebo物理仿真和飞行结束后的离线审计。

常用变体分别运行：

```bash
# 只保留RViz，减少图形负担
bash scripts/run_sh/small_indoor_exploration_demo.sh --no-gazebo-gui

# 完全无图形界面，仍运行真实PX4/Gazebo SITL
bash scripts/run_sh/small_indoor_exploration_demo.sh --headless

# 返回起点并保存成功后，保持窗口与仿真，直到Ctrl-C
bash scripts/run_sh/small_indoor_exploration_demo.sh --keep-open

# 短程功能测试：自主到达两个观察目标后提前返航
# 明确属于目标限额部分探索，不是完整建图测试
bash scripts/run_sh/small_indoor_exploration_demo.sh --headless --return-after-goals 2 --test-seconds 300
```

Ctrl-C会先保存当前维护点云，再清理本轮启动的进程。不要同时另开普通规划器、键盘飞控或第二个探索器。

## 任务和显示

RViz显示维护地图、当前回波、实际飞过的轨迹、当前规划路线与候选视点；新RViz配置包含相机移动工具，可以旋转、平移和缩放。

观察状态：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo /uav1/exploration/status
```

同一终端可暂停、恢复或请求返航：

```bash
rosservice call /uav1/exploration/set_enabled "data: false"
rosservice call /uav1/exploration/set_enabled "data: true"
rosservice call /uav1/exploration/return_home '{}'
```

正常流程为探索→补扫→返航→起点附近连续驻留5秒→保存。继承0.80米正常规划净距、约0.40米机体包络、0.30米/秒导航上限和95%已观测表面质量门限，未通过缩小安全距离使场景跑通。

小场景专属预算：探索300仿真秒或600墙钟秒、补扫120仿真秒、返航300仿真秒，运行器另有1200墙钟秒上限。预算返航会明确标注部分结果；`--test-seconds`截断也不能算任务成功。

## 产物

文件保存在项目根目录 `small_indoor_demo_runs/`，前缀为 `office_日期_时间`：

| 后缀 | 内容 |
|---|---|
| `_final_map.pcd`、`_final_map.json` | 实际维护地图和来源、时间、点数、校验和 |
| `_result.json` | 退出原因、`task_success`、保存和返航状态、末次探索状态 |
| `_status_history.json` | 逐次任务状态和指标 |
| `_flight_samples.json` | 采样的实际里程计轨迹 |
| `_frame_offsets.json` | 运行时地图与PX4局部坐标的偏航差 |
| `_map_and_trajectory.png` | 保存地图与飞行轨迹的离线图 |
| `_wall_audit.json` | 原房间内侧墙面支持代理 |
| `_wall_missing_reference.pcd` | 没有地图支持的参考样本，不能当建图点云 |
| `_office.world`、`_scene.json` | 当次运行副本和来源 |
| `_core.log`、`_gazebo.log`等 | 本轮进程日志 |

`task_success=true`只表示确认返航并保存且未报运行失败。`mission.exploration_complete`与`mission.coverage_complete`另行读取，不能用文件名final或退出码0推断全屋完整。

直接查看地图：

```bash
pcl_viewer /home/a/AstraDroneOpen/small_indoor_demo_runs/office_日期_时间_final_map.pcd
```

离线墙面审计只抽取原房间朝内的墙体盒面，世界高度0.25—2.20米、参考间隔0.20米、欧氏容差0.25米；排除墙角被相邻墙遮蔽的部分，不包含家具、地面、天花、外侧和窗洞边缘。它不是全三维完整率、静态误删率或动态分割指标。世界/map仅初始对齐，不校正长期漂移。

## 验证范围

2026-10-16已完成真实SITL验证：两目标短测返航保存成功；默认Gazebo+RViz任务到达20/22目标、估计航程49.65m、返航XY误差0.081m、保存13374点。默认结果为`inspection_budget_partial`，已测表面质量94.13%、两个complete标志均为false，不能称完整建图。13项新测试与51项既有回归、包编译通过。截图、地图、负结果和边界见[验证报告](../../../../docs/演示验证/2026-10-16_小办公室自主探索/验证报告.md)。静态场景没有提供动态过滤效果证据；本演示仅用于仿真，不代表真机无GPS定位、碰撞安全或实际降落已经验证。
