# 在 FAST-LIO2 基础上，我们实际做了什么

核查日期：2026-10-10。项目根目录：`/home/a/AstraDroneOpen`。

本次按完整交流/学术记录、当前源码、CMake与launch、Git历史整理，不把旧助手的“已完成”声明直接当作当前运行证据。Git基线取`000b725`，它是本系列键盘/场景提交`1d8719f`之前的项目版本；当前HEAD为`8808ad5`，还包含用户未提交的资料。不是将整个AstraDrone、FAST-LIO、EXP或FreeDOM都归为我们原创。

## 0. 先给结论和阅读方式

我们的工作不是重新发明FAST-LIO2的滤波器，而是把已有的“激光与惯性定位、当前扫描配准”扩展成一套可运行、可交互、可评估的无人机系统：动态候选与跟踪 → 静动态即时风险 → 可撤销长期地图与近场地图 → 指定目标规划或未知区域探索 → 飞控执行 → 持久化地图与实验评价。还做过独立回环后端、传感器/行人仿真、内部badcase/ours演示、外部FreeDOM旁路基线、离线LMNet思路图像及研究/汇报资料。

完整逐文件入口：

- [逐文件作用全集.md](逐文件作用全集.md)：按目录逐项列出所有本系列可追溯文件，包括第三方引入、留档、图表/PPT/备份等辅助产物。
- [逐文件作用全集.csv](逐文件作用全集.csv)：同样内容，便于搜索文件名、按组过滤；含路径、归属、状态、用途、大小及适用源码哈希。
- [文件盘点核验.json](文件盘点核验.json)：盘点范围与覆盖检查。
- [build_inventory.py](build_inventory.py)：生成与重新核验上述全集的脚本，仅写本报告目录。

“逐文件不遗漏”的范围是基线到HEAD的Git路径、当前暂存/未暂存/未跟踪路径，另加入少量未修改的原底座作职责说明。外部PX4/PedSim安装、忽略的build/devel和全部运行日志不能由本仓库Git证明逐个做过；它们单列用途，不伪造作者或编辑历史。SVG/预览/备份等每个路径在全集里保留，但只解释产物职责，不假称逐字审读上千个重复页面。

下文路径前缀约定：

| 简称 | 项目根目录下的位置 |
|---|---|
| F | `AstraDrone_ros1_ws/src/SLAM/FAST_LIO/` |
| B | `AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/` |
| O | `AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/` |
| D | `AstraDrone_ros1_ws/src/SLAM/freedom/` |

## 1. 我们保留了哪些原有能力，改了哪些底层文件

FAST-LIO2本身提供IMU初始化/惯性传播、激光与状态时间组织、通常的逐点运动补偿、当前点云到局部地图的点面约束、迭代状态校正、ikd-Tree近邻/增量管理和里程计/扫描输出。**这些滤波方程、直接配准及ikd-Tree属于原工作，不是本项目新发明。**

| 文件 | 我们的工作/职责 |
|---|---|
| F`include/dynamic_point_filter.hpp` | 新增早期几何时序三态过滤器，给每点静态/动态/未知标签，历史空间变化与原地图支持相结合，不接神经网络。 |
| F`src/laserMapping.cpp` | 接入过滤器；在本帧位姿优化之后分类，控制后续增量入图；输出三类诊断云，首帧建树/历史预热单独处理。没有把它写成先过滤所有匹配点的新估计器。 |
| F`src/laserMapping_color.cpp` | 给原有彩色版本同步接入入图门控与诊断输出，原相机着色能力不是新部署R3LIVE。普通/彩色源码中的地图支持距离实现有差异，不能声称两者逐参数完全相同。 |
| F`launch/mapping_mid360.launch` | 增加`sim_lidar`和`dynamic_filter`开关：真实Livox使用CustomMsg，仿真使用PointCloud2/MARSIM，提供过滤参数。 |
| F`launch/mapping_mid360_color.launch` | 为彩色建图提供独立过滤开关/参数。 |
| F`config/mid360.yaml` | 核对/调整MID360话题、发布与外参等配置及注释；launch可覆盖传感器解析类型。 |
| F`CMakeLists.txt`、`package.xml` | 注册基线记录脚本、运行依赖等，保证新增实验入口可构建/安装。 |
| F`src/preprocess.cpp`、`src/IMU_Processing.hpp` | 本系列Git范围未修改；我们复用其中仿真分支，不应列为新写的补偿算法。MARSIM逐点时间为0，跳过逐点反向补偿，不能把当前仿真当成真实MID360逐点去畸变已经验证。 |
| F`include/ikd-Tree/ikd_Tree.*`及状态传播接口 | 原有树和滤波底座，不是维护导航地图模块的实现。 |
| F`Log/mat_pre.txt` | 运行生成日志，不是新算法；当前用户暂存/本地改动保留。 |

早期过滤遇到非重复采样、自运动、遮挡导致大量未知/误动态等问题；后续主演示显式`dynamic_filter:=false`。因此不能说“现在无人机主要靠FAST-LIO内部的LMNet式过滤工作”。当前主线主要在B包维护地图，且**不会自动修改FAST-LIO内部用于配准的ikd-Tree**。

## 2. 把原EXP动态模块迁移成可运行的AstraDrone主链

用户原`EXP-main/`是感知/避障包的留档。B包承接其接口与已有模块，补足AstraDrone的MAVROS、地图规划、探索、生命周期和测试。Git显示整个B包新增，只能证明迁移/引入，不证明其中每一行是新创算法。全集对EXP原包43个文件逐项保留说明。

### 2.1 坐标和数据衔接

| B下文件 | 作用和规则 |
|---|---|
| `src/fastlio_bridge.cpp` | `/Odometry`转换到`/uav1/fastlio/odom`；仿真可只在初始化时以PX4本地位姿建立刚体对齐，之后由FAST-LIO逐帧驱动；发布map下航迹，5次消息抽样一次，最多12000个位姿。现用演示关闭视觉位姿回灌，避免估计反馈环。 |
| `src/cloud_bridge.cpp` | 当前`/cloud_registered`做同一坐标转换后发布`/uav1/fastlio/registered_scan`；不把世界点云再次当传感器坐标输入。 |
| `msg/DynamicObject.msg` | 一个对象的ID、位姿、速度、尺寸、置信度、可选类别和预测标记。 |
| `msg/DynamicObjectArray.msg` | 一组对象统一时间/坐标头，空数组也是合法的“当前无目标”。 |
| `msg/SemanticDetection2D.msg`、`SemanticDetection2DArray.msg` | 给可选相机框/类别接入预留消息格式，不表示已有网络识别。 |

### 2.2 动态候选、聚类、跟踪和可视化

| 文件 | 做了什么 |
|---|---|
| `src/dynamic_detector.cpp` | 背景积累与前景体素比较、距离/高度范围裁剪；当前候选范围约6m。在固定世界坐标比较，避免直接把飞机运动导致的雷达坐标变化都当物体运动。关闭永久在线吸收，保留暂停后再运动对象的可检测性。 |
| `src/dynamic_cluster.cpp` | 候选点按空间聚类，跨帧关联成轨迹，用历史拟合速度；不只看一帧差异。0.5秒历史、0.2m位移、0.3m/s速度、方向线性度/拟合残差和连续4帧运动确认；近场有独立参数，仍需持续轨迹。稳定图0.35m邻域/支持比例0.60用于否决树叶/货架边缘等静态候选。显示位置与运动判断采用不同平滑，避免球更新迟缓。 |
| `src/dynamic_marker.cpp` | 轨迹ID对应显示ID；短寿命0.5秒、空对象/缺失ID删除、短队列，解决看起来“后期没更新/球残留”。 |
| `src/dynamic_predictor.cpp` | 恒速短时外推的辅助输出。风险判断自己用当前对象计算未来最近接近，不必须串在所有主节点之间。 |
| `src/semantic_fusion.cpp` | 预留相机框/投影/类别融合；默认关闭时只透传，不是已经部署LMNet或人体网络。 |
| `src/dynamic_tracker.cpp` | EXP遗留最近邻独立跟踪方案，现用CMake没有注册它；主跟踪已在dynamic_cluster中。 |
| `src/dynamic_kalman_tracker.cpp` | EXP遗留位置/速度卡尔曼跟踪方案，同样不是当前启用链路，也没有现用构建目标。 |
| `config/dynamic_detector.yaml` | 背景、体素、ROI和吸收行为。 |
| `config/dynamic_cluster.yaml` | 聚类/关联、普通与近场运动确认、速度/显示平滑、静态地图支持。 |

针对树叶误动，我们增加的不是“树叶语义识别”，而是较强的持续运动证据和稳定地图支持。静止的人可能是背景；其离开后是否清理由地图的新观测证据解决。运动确认、人体类别、逐点动态标签和碰撞风险是不同任务，当前没有逐点真值证明全部动态物体都识别正确。

## 3. 长期地图与局部新观测共同更新，解决历史人体“假墙”

核心是B`src/static_map_builder.cpp`和`config/static_map.yaml`。不是简单把每次扫描拼接成一个永不删除的PCD。

我们维护的是同一份哈希体素存储`cells_`及其不同视图：

| 输出视图 | 用处 |
|---|---|
| `/uav1/fastlio/cloud_map` | 持续累积但可撤销的全局导航地图；也是主演示显示和最终PCD的来源。 |
| `/uav1/stable_static_map` | 较多独立帧命中支持的稳定视图，供全局规划与动态误检否决；稳定不等于永远不删。 |
| 局部占用视图 | 机体约6m内、最近15帧有效占用，反映眼前实际情况。 |
| 局部自由空间视图 | 实际射线支持当前已空的位置，既用于地图撤销，也用于局部规划修正旧占用。 |

基本入图：按体素归并，每帧只加一次命中，通常全局至少2次、稳定至少8次、局部至少1次。设置体素/射线/容量上限与发布频率，防止无限增长；不因此称资源完全满足机载实时。

我们实际上保留了两种清理模式：

1. **普通example/仓库导航ours**：确认动态对象的当前位置/历史轨迹掩膜、回溯与自由空间清理；badcase关闭地图动态清理、自由清理和局部重规划，形成内部组合消融。轨迹圆柱直接删除有静态误删风险，不能与新探索模式混写。
2. **自主探索**：`evidence_only_clearing=true`。对象轨迹本身不授权圆柱删除，也不据它直接拒绝新点；用扫描时间配对位姿（误差≤0.08秒）和旋转后的雷达外参确定真实射线起点。只有旧点位于射线穿过的位置、离射线足够近（0.06m）、离回波端点有0.30m余量、当前回波及27邻域没有保护它时才记负证据。旧点至少0.5秒未再命中，普通3次/稳定6次新鲜负证据才能删；负证据间隔大于1秒重计，重新命中重置。

意义：人先静止时入图并不意味着永远固定；走开后新射线穿过原位置，可以撤销该历史占用。反过来“本帧没扫到”或“被货架挡住”不等于空闲。无法再观测的区域仍可能留鬼影；不能承诺全图零残影。射线算法、体素和多次证据是已有思想，当前属于工程实现/集成。

## 4. 从单纯躲避改成能绕过货架到达目标

### 4.1 指定目标规划

B`src/static_path_planner.cpp`接收稳定全局图、局部占用/自由空间、当前位姿和指定目标，按飞行高度切片做二维A*。改进内容不仅是改安全半径：

- 全局图先给参考路线，局部观测修正旧占用；全局旧图搜索失败时，也允许局部自由证据继续尝试，不再提前返回。
- 自由证据在三维里匹配旧原始占用，撤销后再做膨胀；当前局部占用最后写入。避免自由点把邻近墙体/其他高度误擦掉。
- 2D目标保留既有目标高度，避免连续点击使误差反复加入；仓库高度带独立设置，避免顶棚被投成平面假墙。
- 仓库失败可4→8→12m扩大搜索域，同一目标记住扩域，避免货架端部绕行被窗口裁掉。
- map固定栅格、旧路线全段安全检查、投影点重连；安全旧路只有新路长度改善至少25%才切换方向，新障碍使其不安全时立即作废。
- 每个前视直飞线段仍需验证，不能沿折线取一个很远的点然后穿过拐角。
- 控制器区分中间点0.03m与最终点0.25m容差，减少拐点提前HOLD。

相关文件：`config/path_planner.yaml`为通用参数；`config/warehouse_navigation.yaml`为仓库最后覆盖；`src/mavros_avoidance_controller.cpp`执行前视航点。它不是完整三维/动力学最优轨迹规划，也不保证任意指定目标必可达。

### 4.2 动静态即时风险与控制

| 文件 | 作用 |
|---|---|
| `src/collision_risk.cpp` + `config/dynamic_risk.yaml` | 相对位置/速度预测最近接近距离与时间；通常预测1.5秒，安全中心距离1.35m，紧急中心0.75m及更早的预测紧急条件，配目标保持、释放与限速。是中心距离，不自动按人体/飞机精确形状求碰撞。 |
| `src/static_collision_risk.cpp` + `config/static_risk.yaml` | 从新鲜当前扫描检查静态物体，邻域支持过滤孤点，减机体半径得到表面净空；即时检测不依赖长期图是否已经删干净。 |
| `src/avoidance_fusion.cpp` + `config/avoidance.yaml` | 两路等级/速度及来源时效融合，最大风险优先、双紧急加权和平滑；配置静态/动态权重0.60/0.40。 |
| `src/mavros_avoidance_controller.cpp` + `config/astra_mavros_controller.yaml` | 稳定地面→预发送→OFFBOARD/解锁→起飞→航点导航/风险保持；新鲜位姿、扫描、风险、航点守卫、悬停与模式丢失保护。 |
| `src/pose_stability.hpp`、`scripts/startup_checks.py` | 修复启动地面高度漂移、仿真时钟与本机时钟混用导致飞机不飞/反向指令；用有效连续窗口和实际空中驻留确认起飞。 |
| `src/system_watchdog.cpp` | 各关键数据更新频率/年龄与系统诊断，不以有球/无球替代节点是否工作。 |
| `src/goal_navigation.cpp`、`src/avoidance_command_bridge.cpp`、`config/command_bridge.yaml` | EXP/Prometheus可选分支，简单目标命令与避让指令桥；默认预览，不是当前MAVROS主演示必经节点。 |

仓库人工圆包络0.40m，规划膨胀0.80m已包含机体，通用仓库栅格0.10m再加半对角线约0.071m。静态机心到表面预警/紧急/解除约0.70/0.65/0.75m。探索格0.20m，有额外离散余量；不能把上述几种距离叫成同一个“安全半径”，也不能用改小机体尺寸强行穿狭缝。

## 5. 新增未知几何仓库自主探索、补扫和明确结束

这不是读取cangku货架坐标后按预设全仓航线飞。机载探索只用实际位姿、维护地图和自由空间；场景盒体由评估/人工行人侧使用，不传给Explorer。保留固定飞行高度，所以是未知几何的平面探索，不是任意楼层/三维无限黑箱。

| B下文件 | 作用/我们补足的内容 |
|---|---|
| `scripts/warehouse_explorer.py` | 探索状态机与航点唯一发布者；等待真实起飞确认，再找未知边界，选目标、移动、驻留观察、重选/黑名单、补扫、返回起点、驻留和保存；记录每种部分/失败终态。 |
| `scripts/exploration_grid.py` | 三态平面观测格、未知不可穿、机体/障碍膨胀、8邻域搜索与线段检查；初始12m半径计算窗口，靠观测接近边界按8m块扩张，新区域未知，20万格上限明确处理。 |
| `src/exploration_search.cpp` | 原生C++图搜索共享库，保留同可达性/角点规则，带贴边软代价；降低重复Python搜索开销。 |
| `scripts/exploration_native.py` | Python/C++接口与自动回退，无原生库仍可走Python，不能因此认定资源适合板端。 |
| `scripts/exploration_coverage.py` | 6m可见未知增益（隔墙不算）、前沿代表点、低增益窗口、当前实测表面3次扫描/2方位/0.75m基线质量和可达补扫视点。 |
| `scripts/warehouse_map_audit.py` | 退出后独立参考货架侧面支持审计，输出逐架不足及参考缺口；不能把缺口PCD拼回建图冒充扫描数据。 |
| `config/warehouse_exploration.yaml` | 探索窗口、前视1.2m/驻留2s、收敛/补扫/预算/返航、恢复和容量等参数。 |
| `launch/warehouse_exploration.launch` | 关闭通用A*，使用Explorer；加载保守证据清理与独立保存；当前限速0.30m/s（覆盖早期0.55），新鲜扫描/紧急悬停与授权退路保护。 |
| `scripts/run_warehouse_exploration.py` | 全系统启动、稳定地面等待、行人、记录状态、双坐标评估、结束导图及可选FreeDOM旁路，墙钟硬期限防止时钟停止时无限运行。 |
| `rviz/warehouse_exploration.rviz` | 前沿候选青色方点、地图、探索路线、自由/占用与任务状态展示。 |

我们遇到“货架旁挪得很慢/不动”后补了规划—控制一致的恢复：只能由Explorer在已观测空间找到净空单调增加的短退路，机心距支持障碍至少0.45m（0.40m圆包络+0.05m，不是正常规划半径），最多短距离推进到正常安全区域；控制器用新扫描、匹配目标、动态风险为0等二次授权。常规飞行仍按0.80m前向保护；没有安全方向不硬冲，连续30仿真秒无进度显式`STALLED_UNSAFE`。这是工程恢复条件，不提供未知遮挡中的安全证明。

结束是三个不同问题：

1. 在线可达探索收敛：90秒自由面积增量<1m²且新粗表面格<25，至少3个观察目标，转补扫；当前已测表面质量≥95%、无有用前沿/补扫及失败/容量问题才允许`observed_reachable_converged`。
2. 任务收尾：回到起点≤0.30m连续5秒并收到保存成功，才COMPLETE；预算用尽返航可以是“返回并保存成功”，但地图仍partial。
3. 全场景地图质量：独立参考货架侧面总体≥95%且每架≥90%是当前代理建议标准；上/下/背面和不可达表面不由它证明，`coverage_complete`仍不据返航强行置真。

## 6. 独立回环后端：做过、保留、主演示关闭

参考better_fastlio2、FAST_LIO_SAM的组织思路，另核对FAST_LIO_SLAM；没有把整个参考项目直接替换进来，也没有使用GTSAM实现当前后端。

| 文件 | 作用 |
|---|---|
| B`src/loop_closure_backend.cpp` | 距离/时间间隔筛候选，4关键帧子地图描述/高度归一化、Scan Context排序/门控、多偏航种子ICP、fitness/重叠率/最大修正/垂直分离约束、Huber稀疏SE(3)位姿图、优化路径和关键帧地图重建。 |
| B`config/loop_closure.yaml` | 关键帧、候选、描述/配准/图优化门限和容量参数。 |
| B`launch/loop_closure.launch` | 后端可选入口，普通装配默认关闭。 |
| B`launch/loop_closure_demo.launch`、`rviz/loop_closure.rviz` | 专项轨迹/约束/地图对比显示。 |
| B`test/loop_closure_synthetic.py`、`launch/loop_closure_synthetic_test.launch` | 合成人为漂移/回访验证后端链路。 |
| O`src/loop_closure_demo_flight.cpp`、对应launch | 固定方形回访任务，返航驻留和降落收尾。 |
| `scripts/run_sh/loop_closure_demo.sh` | 一键运行/录包/评估，检查插件和遗留节点。 |
| B`scripts/analyze_loop_closure_bag.py` | 在线原/优化里程计ATE、5秒RPE、闭合误差/轨迹；发表级最终Path不是当前自动报告的同一口径。 |
| B`LOOP_CLOSURE_CN.md`、专项world | 运行/方法/评价说明及测试环境。 |

当前后端没有接管正常飞控里程计，也没有把动态证据绑定到关键帧并确保回环重建人体不复生。历史真实运行出现误闭环、ATE变坏，因此它是试验成果/失败研究平台，不是目前已证明提高精度的功能。

## 7. FreeDOM外部地图基线：作者代码与我们适配分别归属

D包固定上游版本与MIT来源见`ASTRA_BASELINE_CN.md`。本轮解释来自当前本地源码/说明，没有新检索或新复现实验。

我们新增/适配：`config/astra_mid360.yaml`、`launch/astra_mid360_baseline.launch`、`ASTRA_BASELINE_CN.md`；`src/freenode.cpp`及对应头文件中ROS输入帧检查、短扫描队列、保存完成回执/计时日志；B的运行器增加`--freedom-baseline`、保存等待和各自坐标审计。没有把FreeDOM原方法叫成ours提出。

| 原作者文件/配套头 | 作用 |
|---|---|
| `src/node.cpp` | ROS可执行入口。 |
| `src/freenode.cpp` / `include/freedom/freenode.h` | 参数、点云/TF输入、线程/保存与可视化接线；含本项目接口修正。 |
| `src/freedom.cpp` / `freedom.h` | 原作者扫描清理、自由估计、地图细化、静态整合主流程。 |
| `src/mrmap.cpp` / `mrmap.h` | 多分辨率自由/静态体素与增量撤销。 |
| `src/scanmap.cpp` / `scanmap.h` | 当前扫描局部体素地图。 |
| `src/map.cpp` / `map.h` | 基础地图/空间索引管理。 |
| `src/depth_image.cpp` / `depth_image.h` | 深度投影/射线增强相关表示。 |
| `src/raycast.cpp` / `raycast.h` | 射线遍历相关实现。 |
| `src/visualization.cpp` / `visualization.h` | 原方法扫描/地图结果显示、静态点云发布。 |
| `common_types.h`、`utils.h`、`include/lidar/point_types.h` | 索引/通用类型、参数/数学工具、不同雷达点字段。 |
| `src/evaluate_tools/ground_truth_generate.cpp`、`static_map_evaluate.cpp` | 原作者离线参考生成与静态地图评估，存在不表示我们复现原论文分数。 |
| 其余原作者数据集config/launch/FOV图、CMake、package、README、LICENSE | 完整引入/保留原目录结构及依赖/许可，全集逐项列出。 |

输入`/cloud_registered_body`和扫描时刻`camera_init→body` TF；输出独立`/uav1/freedom/static_map`。它不是ours规划图的发布者，飞机由我们原Explorer/控制器飞。同一航线产生双图可比较地图，但不能说FreeDOM规划优于ours。当前关闭未校准的MID360射线增强，不是论文完整参数复现。

## 8. 仿真、飞行、行人、交互与一键演示

### 8.1 场景与雷达

| 文件 | 我们的作用 |
|---|---|
| `simulation/astra_gazebo_models/mid360/mid360.sdf` | 图样角度偏移15度，向下扫描更多，向上相应减少；PointCloud2类型匹配。不是把真实MID360物理视场变大。 |
| `simulation/sim_workspace/src/sensors/Mid360_simulation_plugin/livox_laser_simulation/src/livox_points_plugin.cpp` | 增加SDF角偏移参数，旋转原CSV射线方向，保留既有非重复扫描模拟框架。 |
| `simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch` | 修正world资源路径/不存在包查找，允许传入独立world。 |
| 同目录`astra_long_corridor.launch` | 长走廊PX4启动配置。 |
| `simulation/astra_gazebo_worlds/example.world` | 已有场景增加行人插件等适配。 |
| 同目录`RoboCup_sim/RoboCup_sim_pedsim.world` | 早期RoboCup与行人仿真场景。 |
| 同目录`fastlio_bench/long_corridor_degenerate.world` | 固定几何长走廊退化实验。 |
| 同目录`fastlio_bench/loop_closure_arena.world` | 不同墙/箱/柱/台阶地标的闭合飞行专项。 |
| 同目录`cangku.world` | 原有仓库，不是本系列新创；用`warehouse_scene.py`生成添加行人的运行副本，原件保留。 |

### 8.2 行人和飞行任务

行人：B`scripts/pedestrian_control.py`、`warehouse_pedestrian.py`、`exploration_pedestrian_walk.py`、`warehouse_scene.py`、`config/scenarios/astra_crossing_once.xml`、`launch/astra_pedestrians.launch`，分别负责自动横穿/单写者接管、仓库单人、探索先静后动、场景副本、轨迹定义和节点装配。W/S世界Y、A/D世界X，0.4秒断流停车；不能与无人机键盘轴约定混淆。

O包七个src文件全部解释如下：

| 文件 | 归属和职责 |
|---|---|
| `autoarming_control.cpp` | 原自动起飞+方形/圆形位置航线示例，非我们的避障算法。 |
| `keyboard_control.cpp` | 我们新增的无人机键盘位置控制。 |
| `position_control.cpp` | 原位置控制入口。 |
| `position_control_lib.cpp` | 原目标位置到MAVROS setpoint库；保留无频率sleep/参数读取调用等教学风险，不能当当前主飞控。 |
| `pub_origin.cpp` | 原地理原点发布示例。 |
| `long_corridor_baseline_flight.cpp` | 新增长走廊往返实验航线。 |
| `loop_closure_demo_flight.cpp` | 新增闭环方形航线/降落。 |

O`CMakeLists.txt`/`package.xml`注册新增节点与依赖，新增3个对应launch。七源文件是六个可执行程序加一个位置库，不是同时并行控制飞机的七条流程。

### 8.3 运行入口和生命周期

| `scripts/run_sh/`文件 | 用途 |
|---|---|
| `pc_example.sh` | 基础键盘建图；修复PX4/world路径、插件和消息类型等待，不依赖用户astra别名。 |
| `pc_example_tmux_backup.sh` | 保留旧tmux命令，旧别名/包路径可能过时。 |
| `pc_dynamic_avoidance.sh` | 旧多终端自动动态避障。 |
| `dynamic_comparison_demo.sh` | example内部badcase/ours，交给同一运行器。 |
| `warehouse_dynamic_demo.sh` | 单个可键盘行人的cangku导航。 |
| `warehouse_exploration_demo.sh` | 自主探索/补扫/返航/保存，可旁路FreeDOM。 |
| `fastlio_long_corridor_baseline.sh` | 走廊退化基线。 |
| `loop_closure_demo.sh` | 闭环专项。 |

B`scripts/run_dynamic_comparison.py`处理原生rosbag writer真正写盘/索引完成后才评估，隔离非关键报告失败，报告结束继续展示，重复Ctrl-C不会再打断Python清理；B`scripts/run_warehouse_exploration.py`处理仿真时钟、稳定地面与明确收尾。曾反复出现的Gazebo原生卸载异常与Python生命周期问题不同，不能因Python修复就宣称全部Gazebo崩溃消除。

## 9. 地图导出、量化效果和完整测试文件

B`scripts/save_navigation_map.py`直接保存维护话题有限XYZ，二进制PCD/JSON记来源、时刻、坐标、点数和SHA256，临时写后原子替换，拒绝空/陈旧扫描。服务`/demo/save_map`；普通ours`dynamic_demo_runs/`、探索`exploration_demo_runs/`。文件名final_map表示该次收尾快照，不保证全仓完整。badcase默认不一定装配保存服务，不应套用ours命令。

评估代码：B`scripts/analyze_dynamic_demo_bag.py`、`compare_dynamic_demo.py`、`analyze_loop_closure_bag.py`、`warehouse_map_audit.py`；F`scripts/fastlio_baseline_recorder.py`；根`analysis/plot_warehouse_run.py`。分别做动态任务指标/内部对照、在线回环误差、独立参考侧面、走廊前端误差、仓库实测路线展示。真值只用于评价，不给导航先验。

B`test/`逐文件：

| 文件 | 主要验证对象 |
|---|---|
| `test_demo_utilities.py` | 原样PCD、空地图拒绝、行人接管/停车/其他人固定。 |
| `test_runner_lifecycle.py` | writer收尾竞态、无效bag、重复Ctrl-C。 |
| `test_warehouse_scene.py` | 原仓库几何保留与行人边界。 |
| `test_exploration_grid.py` | 未知不穿、膨胀/当前占用、扩窗、角点、线段和安全恢复。 |
| `test_exploration_ros.py` | 探索消息/数据边界与断流夹具。 |
| `test_exploration_mission.py` | 探索/补扫/返航/驻留/保存/预算与失败终态。 |
| `test_exploration_coverage.py` | 多视角/增益、同输入搜索一致、参考侧面审计、PCD和容差。 |
| `test_startup_checks.py` | 仿真时钟初始化、稳定窗口、重复/失效数据、真实起飞确认及墙钟守卫。 |
| `test_pose_stability.cpp` | 独立C++位姿稳定窗口。 |
| `test_map_evidence_live.py` | 隔离真实C++地图节点，同输入静态保护/人体移除/射线/时效。 |
| `check_takeoff_controller_live.py` | 真实控制器+假飞控的起飞/断流/风险/授权退路/前向刹车，不连接实物。 |
| `check_warehouse_clearance.py` | 真实风险/A*节点，通道可行性、滞回、高度、融合/拐点。 |
| `check_planner_snapshot.py` | 用户停滞快照复现与扩大搜索回归，不控制飞机。 |
| `check_warehouse_multigoal.py` | 仿真多目标实飞，显式会发布目标，非只读。 |
| `check_warehouse_demo.py` | 仿真行人往返/动态风险/清图/保存。 |
| `check_live_demo.py` | 普通demo采样后暂停/再动与导图一致测试。 |
| `check_exploration_live.py` | 只读采集实际探索证据。 |
| `check_exploration_controls.py` | 暂停/恢复与保存快照联调。 |
| `check_rack_retention_live.py` | 只读多次已观测静态货架保留，分母不是全仓。 |
| `test_exploration_mission_live.py` | 独立真实Explorer+合成房间+运动学跟随器终态联调，不是PX4飞行。 |
| `benchmark_exploration_search.py` | Python/C++同输入搜索性能微测，不是整机性能。 |
| `synthetic_scene.py` | 动静场景点云/位姿夹具。 |
| `loop_closure_synthetic.py` | 人为漂移回访夹具。 |

B`scripts/validate_project.py`是静态结构检查；B`CMakeLists.txt`、`package.xml`、`.gitignore`负责构建/依赖/发布范围。历史已记录编译/单测/真实SITL等不同层次证据，但本次是清点说明，**没有重新启动飞行或把静态清点称作全部测试通过**。

## 10. LMNet思路展示、研究记录、证据合集和PPT工作

| 文件/文件组 | 作用 |
|---|---|
| `analysis/lmnet_style_visualize.py` | 从已有完整bag抽4帧，插值位姿+外参补偿，64×360球面量程、归一化残差及额外鸟瞰，保存NPZ/JSON。不推理LMNet/SalsaNext，也不把残差当动态真值。 |
| `analysis/LMNET_STYLE_VISUALIZATION_CN.md` | 输入、方法、图像与非重复采样/遮挡等限制。 |
| `analysis_outputs/lmnet_mid360_crossing/` | 三张图、数组与metadata实际产物；本轮全集逐文件保留。 |
| `docs/OURS_DEMO_TECHNICAL_CN.md` | ours技术、模块与证据边界。 |
| `docs/ours_demo_modules.dot/.svg/.png` | 可编辑关系源和展示图。 |
| `docs/组会汇报/2026-10-05/`两主文档 | 首轮完整素材/大纲。 |
| `docs/组会汇报/2026-10-07/`三个主文档 | 四重点源码/原论文/真实实验深化、逐页大纲与媒体出处。 |
| 同目录`prepare_materials.py` | 真实视频抽帧、实测报告/PCD复核、五图/逐架CSV。 |
| 同目录`validate_materials.py` | 数字/来源路径/文档/媒体/哈希一致性。 |
| 同目录`package_evidence.py` | 原始视频/bag/PCD等真实合集，逐文件复制/大小/哈希验证，不是仅清单。 |
| `docs/组会汇报/2026-10-07_完整证据素材包/` | 实际文件副本，主录像/原截图/图表/原实验分组；不是新实验，不宜普通Git推送大文件。 |
| `projects/astradrone_offboard_source_review_20261009.md` | 七个Offboard源码逐函数职责/限制。 |
| `projects/astradrone_groupmeeting_research_20261009.md/.facts.json` | 原论文/作者/厂家资料事实与来源归属，非部署或复现。 |
| `projects/astradrone_ppt_requirements_and_evidence_20261009.md` | 汇报需求、实际源码和真实证据定位。 |
| `docs/组会汇报/2026-10-09/`及子工程 | 42页PPT/PDF、逐页SVG/备注/来源/预览、设计与批注修订/旧版本备份。 |
| 其`validation/verify_revision.py`、`verify_later_revision.py` | 两次PPT修订的未改页、备注、原图来源和导出验证；非ROS算法测试。 |
| `docs/功能讲解/2026-10-10/` | 22页规则版PPT/PDF、文字/总图/讲稿及验证，保留旧版。 |
| 同目录`详细版_独立生成/content.py` | 57页更详细的中文正文/流程/补充讲稿定义。 |
| `build_deck.py` | 独立python-pptx生成原生编辑PPT及简图，不使用ppt-master。 |
| `verify_deck.py` | 实际PPTX/PDF文本、备注、排版与源码54引用核验。 |
| 其`逐页详细讲解与源码位置.md`及简图/预览/JSON | 可阅读/可追溯的完整规则说明，图不是新实验曲线。 |
| `AGENTS.md`、`交流记录/使用说明.txt` | 固定先读双记录、执行/验证、后记录的工作规则。 |
| `交流记录/交流与解决方案.txt`、`学术关联与论文写作记录.txt` | 需求/实际变化/失败/验证与学术归属持续记录。 |
| `交流记录/每日总结/2026-09-22.md`、`每周总结/2026-09-21至2026-09-27.md` | 当时日/周工作复盘。 |
| `交流记录/RA-L研究路线_2026-09-30.md` | 未来任务驱动占用核验研究计划，尚未完成的新机制不能计现有创新。 |

外部`/home/a/AstraDroneOpen_backups/`保存过完整/专项版本备份，是否某次已覆盖、包含哪些后续代码要按其README与对应记录判断，本次没有再次覆盖备份。Git推送也是工程管理，不等于算法验证。本次完整清单还列出第三方`install_files.txt`等安装副产物，明确不计新方法。

## 11. 目前真实主链关系

```text
MID360 + IMU
    │
    ▼
FAST-LIO2原估计/扫描输出
    │
    ▼
里程计桥 + 点云桥（统一map）
    ├─ 前景候选 → 聚类/轨迹/动态确认 → 动态风险 ─────┐
    ├─ 新鲜当前扫描 → 静态风险 ──→ 风险融合 ───────┤
    │                                              │即时保护
    └─ 可撤销全局/稳定/局部/自由空间视图             │
            ├─ 指定目标二维A* → 航点 ──────────────┤
            └─ 自主探索/补扫/返航 → 航点 ──────────┤二选一
                                                   ▼
                                            MAVROS控制器 → PX4
    维护cloud_map → RViz/PCD保存 → 离线审计/报告

旁路：FreeDOM独立地图；可选回环；辅助恒速预测；关闭的语义接口。
```

## 12. 已有实验支持了什么（本次重新读报告，不是重新飞行）

本次重新核对了以下原文件，数字只属于对应版本/航线，不外推当前所有场景：

| 原文件 | 能支持的结果 |
|---|---|
| [`dynamic_demo_runs/badcase_20260923_201211_metrics.json`](/home/a/AstraDroneOpen/dynamic_demo_runs/badcase_20260923_201211_metrics.json) | 一次内部badcase：到达61.698仿真秒、航程20.296m、旧位置稳定图128点。 |
| [`dynamic_demo_runs/ours_20260923_200322_metrics.json`](/home/a/AstraDroneOpen/dynamic_demo_runs/ours_20260923_200322_metrics.json) | 同设计的一次ours：37.377仿真秒、12.423m、旧位置稳定图0点。终点误差0.171m对比0.132m、最近人机中心0.965m对比0.979m，没有同时改善。组合开关+不同实际路线不能证明新LIO精度或统计显著性。 |
| [`exploration_demo_runs/warehouse_20261003_222021_result.json`](/home/a/AstraDroneOpen/exploration_demo_runs/warehouse_20261003_222021_result.json) | 97目标中93到达，返航并保存，home_error约0.123m；结束原因wall_budget_partial，exploration_complete和coverage_complete均false，不是全仓完整成功。 |
| [`warehouse_20261003_222021_coverage.json`](/home/a/AstraDroneOpen/exploration_demo_runs/warehouse_20261003_222021_coverage.json) | ours参考货架侧面9267/10192≈90.92%，最差单架29.59%。 |
| [`warehouse_20261003_222021_freedom_coverage.json`](/home/a/AstraDroneOpen/exploration_demo_runs/warehouse_20261003_222021_freedom_coverage.json) | FreeDOM旁路10091/10192≈99.01%，最差单架85.71%；均未通过总体95%且每架90%的联合要求。不是MOS精度、误删率、动态清除或总体方法排名。 |

这说明我们已经具备**“可运行的闭环工程、历史占用局部撤销、内部对照、长程探索返航与双地图审计”**的真实证据，同时也明确发现静态缺架、整体覆盖不足与评价边界。不能只挑局部0残影或总点数去证明所有问题解决。

## 13. 什么不能作为已完成成果来讲

- 没有替换FAST-LIO2核心滤波，也没有证据证明动态维护使前端定位普遍更准。
- LMNet/人体神经网络、R3LIVE、完整FUEL、DUFOMap等不能计作已部署；讨论/图像展示/文献阅读和实现不同。
- “两种地图”应解释成原估计树与外部导航存储，以及外部存储的多个视图，不说局部点直接重写了原ikd-Tree。
- 回环做过但真实误匹配未根除，主demo默认关闭。
- FreeDOM算法归原作者，当前旁路及保守配置不是完整论文重现，更不接管飞行。
- 返航/保存成功不等于全仓全部表面重建；固定高度仍会遮挡/不可达，参考侧面审计仍有缺架。
- 几何候选/运动确认/旧占用清理不是人体语义识别，也没有全场景零鬼影、零碰撞证明。
- 本次新增的是总览、全文件目录及核验记录；没有改动算法、再次编译/飞行、更新备份或推送GitHub。

可用于组会的一句话：**“我们以FAST-LIO2提供的连续位姿和注册扫描为基础，完成了动态观测、可撤销导航地图、全局与局部规划、近场安全控制、仓库自主探索及地图评价的闭环工程系统；并用内部消融、外部旁路基线和失败案例明确其效果及尚未解决的问题。”**
