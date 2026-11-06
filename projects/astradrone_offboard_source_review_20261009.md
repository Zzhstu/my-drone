# Offboard 飞行控制源码核对——组会素材补充

日期：2026-10-09，Asia/Shanghai。
核对范围：用户指定的 `astra_uavoffbard_frame` 中的全部 C++ 源码、头文件、CMake 构建目标和 launch；仅阅读，没有修改飞行代码、重新编译或启动仿真。

## 1. 路径与讲述边界

实际源码位置是：

`/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src`

用户给出的包外层下面没有直接的 `src`，需要进入 `offboard`。共有 7 个 C++ 文件：六个可执行程序的源码以及一个位置控制库。`position_control.cpp` 与 `position_control_lib.cpp` 合起来构成一个节点，不能计作两套独立控制器。

这些程序是“给飞控发送目标的任务/外环示例”，并不直接控制电机，也没有在该目录实现 FAST-LIO、点云聚类或 A*。基础 `pc_example.sh` 启动其中的键盘节点；目前 badcase/ours 与探索演示使用的是 `fastlio_bridge` 下的独立避障速度控制器。PPT 应清楚说明基础能力、专项测试航线与当前主链的不同职责，不能把七个文件画成同时运行的一条管线。

## 2. 文件职责、输入、输出与对应源码

统一源码前缀：`/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/`。

| 文件 | 主要职责 | 输入 | 输出 | 实现边界 |
|---|---|---|---|---|
| `src/position_control.cpp` | ROS 节点入口，实例化 DroneControl | ROS 初始化 | 调用控制库 | 没有规划/控制计算 |
| `src/position_control_lib.cpp` | 将外部目标 XYZ 转发给 MAVROS | `/drone_control/goal_position` | `mavros/setpoint_position/local` | 不自动切模式/解锁，不验证障碍 |
| `src/autoarming_control.cpp` | 自动解锁、起飞、方形/圆形航线、下降 | `mavros/state`、`mavros/local_position/pose`、参数 | 位置设定、模式/解锁/CommandLong 服务 | 预定义轨迹，不自主探索、不避障 |
| `src/keyboard_control.cpp` | 键盘增量修改位置与高度目标 | 终端按键、FCU 状态、局部位姿 | 位置设定、模式和解锁服务 | 离开键盘仍持有上一个位置目标；没有点云安全检查 |
| `src/long_corridor_baseline_flight.cpp` | 可重复长廊去返航线 | FCU 状态、局部位姿、航线参数 | 20Hz 位置设定、模式/解锁服务 | 用于退化测试，跟踪异常悬停不是避障规划 |
| `src/loop_closure_demo_flight.cpp` | 可重复方形重访航线与结束通知 | FCU 状态、局部位姿、航线参数 | 20Hz 位置设定、模式服务、`loop_closure_demo/finished` | 给回环实验制造重访，不在该文件检测回环 |
| `src/pub_origin.cpp` | 发布地理原点消息 | latitude/longitude/altitude 参数 | `/mavros/global_position/gp_origin` | 不是 LIO 坐标对齐，也不生成 GNSS 或定位真值 |

上述相对 `mavros/...` 名称由节点命名空间/remap 决定最终完整话题，不能无条件写成 `/uav1/mavros/...`。

## 3. 位置控制入口与库：事件输入，持续转发

`position_control.cpp:9` 创建 `DroneControl` 对象；构造函数立即进入 `DroneControl_main()`，并在自己的 while 中执行发布与 `spinOnce()`。因此入口后面的 `ros::spin()` 不是当前主要调度循环。

`position_control_lib.cpp:23` 订阅绝对目标话题；回调只复制 x/y/z；主循环不断发布 `goal_position`。可用如下关系解释：

```text
外部目标 PoseStamped
    → position_CallBack：更新 goal_position 的 XYZ
    → DroneControl_main：持续发布该目标
    → MAVROS 位置接口
    → PX4 内部位置/速度/姿态/电机控制
```

当前代码需要保留的局限：

- `ReadParams()` 定义了读取 `/goal_init_x/y/z` 的逻辑，但当前调用链没有调用它；不能声称 launch 的初始目标已经被该函数读入。
- while 中没有 `ros::Rate::sleep()`；不要把这个节点讲成固定 20Hz 控制。
- 回调没有复制 orientation、stamp/frame_id；主循环没有补新时间戳。默认 Pose 的四元数不是有效单位四元数。
- 没有等待位姿有效、自动 OFFBOARD、解锁、目标时效或碰撞检查。

这些是本次源码静态检查结论，不是本轮复现过的飞行故障；当前 PPT 工作没有授权修复飞控源码。

## 4. 自动起飞与圆/方轨迹：状态机与计算规则

主要入口 `autoarming_control.cpp:81`；状态枚举 `:21`；轨迹函数 `get_square_position()` 与 `get_circle_position()`；核心阶段逻辑 `:192` 开始。

```text
等待 FCU connected
    →记录 initial_height
    →100 次位置预发送（20Hz，名义约5s）
    →请求 OFFBOARD / 解锁（请求间隔5s）
    →TAKEOFF：目标(0,0,hight)
    →|当前Z-hight|<0.1m：TRACKING
    →圆形或方形参考点逐步推进
    →完成 target_laps：LANDING
    →回到(0,0,initial_height)
    →高度差≤0.1m：调用 Lock → COMPLETED
```

圆形：`x = radius cos(2πt)`，`y = radius sin(2πt)`。方形将总周长按四个边分段，中心位于原点。`t` 是归一化轨迹进度，不是墙钟秒；位置用绝对局部坐标，不是自动以起飞 XY 平移的航线。

实际推进规则：当前与参考点距离 `d<1.0m` 时，`t += (1-d)/trajectory_length`。没有乘 dt，也没有读取 launch 的 `speed` 参数。其意义是“飞机贴近参考点后推进参考”，不能称严格恒速轨迹。

关键源码/launch 不一致：

- 代码真正读取 `hight`（保留原拼写）作为目标局部 Z，不是相对地面增高；launch 的 `takeoff_height` 并未在此源码读取。
- launch 的 `speed=1.5` 也未读取，不能当作实测或实际配置巡航速度。
- 只等待 connected，没有独立 have_pose 或稳定地面窗口；`initial_height` 在预发送之前锁定。
- `Lock()` 用 CommandLong command=400，param1=0、param2=21196 请求强制 disarm；不能作为真机推荐下降/解锁协议。代码进入 COMPLETED 没有要求先确认服务成功。

这一部分可用作“基础自动飞行机制的教学”，优点是简单透明，缺点是欠缺初始化、时效、安全与执行反馈，不应作为当前仓库自主避障的安全证明。

## 5. 键盘控制：位置增量而非直接电机/速度控制

主要类 `KeyboardControl`；`run()` 在 `keyboard_control.cpp:50`；`readKeys()` 在 `:123` 附近。

```text
等待 connected + 第一条局部位姿
    →当前位置作为初始目标，记录 initial_z
    →100 次预发送（20Hz）
    →每2s尝试 OFFBOARD 或解锁
    →非阻塞读取键盘并更新 target
    →20Hz持续发送位置目标
    →l：回初始高度，接近后请求 disarm
```

默认位置增量 `step=.2m`，按键映射是局部坐标轴：w/s 修改 X，a/d 修改 Y，r/f 修改 Z，t 把 Z 设为 `initial_z+takeoff_height`，l 把 Z 设为 initial_z 并设降落标志；不是按当前机头方向计算前后左右。默认 `takeoff_height=1.0m`。

悬停解释：不继续按键时，程序仍持续发最后一个目标，由 PX4 追踪该位置。它不是键盘停止就自动断开发送。

本次静态核验注意事项：`min_height` 虽读取，却没有用于限制；实际只用 `max(initial_z,target_z)`。`land_requested` 后续没有撤销，按 t/r 不能被讲成完整安全取消降落流程。预发送阶段 `publishTarget()` 不逐次刷新 header，进入主循环后才补当前时间戳。没有雷达与风险输入、没有陈旧位姿停车。不要与当前 ours 的看门狗控制器混淆。

## 6. 长走廊基线航线：可重复的退化试验

完整状态：PRESTREAM → ARMING → TAKEOFF → SETTLE → OUTBOUND → TURN_HOLD → RETURNING → LANDING → DONE；异常进入 ABORT_HOLD。

读取默认参数：相对起飞 1.5m、前进 60m、参考速度 .8m/s、起飞驻留5s、尽头驻留3s、位置容差 .30m、降落高度容差 .12m、最大跟踪误差3m。具体 launch 可覆盖，参数不是本次实测结果。

航线相对于启动局部位姿：起点 XY/Z 被保存，起飞目标=start_z+takeoff_height；去程 target_x 随 `speed×dt` 增长，返程减少，dt 截断在[0,.20]s；Y固定起点Y。每20Hz持续发布位置设定。当前与参考距离超过 max_tracking_error 就把参考改为当前位置并 ABORT_HOLD，不再让参考无限跑远。

价值：对照长廊几何退化时可以使用同一种去返航线；局限：不读取点云/地图，不检查障碍或路径可达性，没有在本文件实现退化检测。异常悬停不能代替安全规划；降落到容差后尝试 disarm 并进入 DONE，不以 disarm 服务确认作为 DONE 必要条件。

## 7. 回环专项飞行：重访航线不等于正确回环

完整状态：PRESTREAM → ARMING → TAKEOFF → SETTLE → ROUTE → CORNER_HOLD → FINAL_HOLD → LANDING → DONE；异常 ABORT_HOLD。

航线在启动位姿上加五个相对位置：(0,0)、(s,0)、(s,s)、(0,s)、(0,0)，保持相同巡航高度和 yaw。默认 side_length=10m、takeoff_height=2.5m、reference_speed=.75m/s、corner_hold=1.5s、final_hold=12s。返回后驻留用于向后端提供多帧 MID360 扫描，不是宣布已经检测到回环。

`moveTargetToward()` 以 .75×dt 的步长沿三维方向推进目标；只有参考到拐角且实际位姿在 .30m容差内才转驻留。最大跟踪误差2.5m后 ABORT_HOLD。

降落采用 AUTO.LAND 请求并等待 PX4 disarm，避免单凭小Z差就退出 OFFBOARD；完成时发布 latched Bool `loop_closure_demo/finished=true`。回环候选、Scan Context、ICP、图优化属于 `fastlio_bridge/src/loop_closure_backend.cpp`，应放在后面的回环尝试与负结果讨论，不在 FAST-LIO2 开场抢先定性“只是前端”。

## 8. 当前 ours 控制：另一条单写者链路

实际源码：`/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/mavros_avoidance_controller.cpp`。
实际 launch：同包 `launch/astra_dynamic_avoidance.launch`；探索覆盖 `launch/warehouse_exploration.launch`。

```text
目标导航(A*) 或 黑箱探索(Dijkstra+frontier)，二选一
    →导航航点 / 专门短恢复目标
    →位姿误差比例速度 + 速度限幅
    →位姿/扫描/目标/风险的采集与接收新鲜度
    →静态净空与动态CPA优先覆盖
    →恢复目标与新扫描二次核验
    →30Hz MAVROS速度设定
    →PX4执行与下一帧观测反馈
```

这里只核对现有实现和接线，不新增性能实验。探索 launch 当前普通导航限速 .30m/s、require_fresh_scan=true、hold_on_level2=true、static_recovery_speed=0；不能照旧资料把最后有效探索巡航速度说成 .55m/s。有条件短退路是另一专用接口，不等于开放普通紧急斥力。

七个基础 Offboard 文件与这个控制器共享 MAVROS 设定接口，演示时只可有一个主动设定值发布者。PPT 的模块关系必须区分“可替换控制模式”和“同时运行的数据流”。

## 9. 用于 PPT 的事实约束

用户要求先介绍 FAST-LIO2 的论文、创新、工作流程与原理，再介绍新增模块、源码与真实效果，然后仓库/黑箱探索/FreeDOM，最后 LMNet 与未来边缘在线人体/动态去除。

本补充是源码内容材料，不是已确认的页数/排版方案，也不是已生成的幻灯片。页面应该用输入→处理→输出、状态转移与规则解释，不铺满整段代码。原理文字与流程图可以丰富，具体页数、文字密度与视觉方案需要在 PPT Master 的确认阶段确定。

论文/作者方法、工程集成、历史实测与未来方案分别标识；基础控制器静态发现的缺点不擅自修复，也不声称此轮编译或实飞通过。
