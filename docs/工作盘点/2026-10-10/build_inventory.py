#!/usr/bin/env python3
"""Generate a provenance-aware, per-file inventory from local Git evidence.

Read-only against project sources. Output is limited to this report directory.
Git additions do not establish authorship. Imported and generated files remain
explicitly distinct from algorithm implementation.
"""
import collections
import csv
import hashlib
import json
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
BASE = '000b725'
BRIDGE = 'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/'
FAST = 'AstraDrone_ros1_ws/src/SLAM/FAST_LIO/'
FREEDOM = 'AstraDrone_ros1_ws/src/SLAM/freedom/'
OFFBOARD = 'AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/'

# Each executable/library/config/test has an explicit responsibility. Generic
# extension rules are reserved for copied assets, old PPT pages and outputs.
ROLES = {
 'fastlio_bridge.cpp':'将FAST-LIO里程计转换到统一map坐标；可选初始PX4对齐/视觉位姿输出；发布抽样、有长度上限的飞行轨迹。',
 'cloud_bridge.cpp':'把当前注册点云做与里程计一致的刚体坐标变换，发布统一坐标的registered_scan，不只是改frame名称。',
 'dynamic_detector.cpp':'体素降采样、启动背景积累和机体附近高度/距离裁剪；与背景对比生成前景候选，候选不等于动态真值。',
 'dynamic_cluster.cpp':'前景空间聚类、跨帧目标关联、位置历史和速度拟合；用位移/速度/方向一致性/连续帧确认运动，并以稳定地图支持否决静态误检。',
 'dynamic_tracker.cpp':'EXP遗留最近邻轨迹关联与速度估计；当前主包CMake未注册此目标，主链由dynamic_cluster负责跟踪。',
 'dynamic_kalman_tracker.cpp':'EXP遗留六维位置/速度卡尔曼跟踪方案；当前主包CMake未注册此目标，不是现有主链必经模块。',
 'dynamic_predictor.cpp':'按位置和速度做短时间恒速外推并发布预测对象；辅助侧路，当前风险节点自行计算相对运动预测。',
 'semantic_fusion.cpp':'可选相机检测框/投影接口，为几何对象附加类别；当前演示默认禁用识别，仅透传，不包含已部署人体网络。',
 'dynamic_marker.cpp':'把确认对象显示为带轨迹ID的球/标记；处理空数组删除和短生命周期，避免显示残留；球不是精确人体碰撞形状。',
 'collision_risk.cpp':'根据目标与无人机的相对位置/速度预测最近接近距离和时间，生成动态风险等级、避让速度及目标保持/释放滞回。',
 'static_collision_risk.cpp':'从当前新鲜扫描检测近场受支持静态障碍；扣除无人机圆包络，处理动态目标掩膜、风险进入/退出滞回。',
 'avoidance_fusion.cpp':'融合静态/动态风险等级与避让速度；来源有效期、优先级、紧急权重和平滑，不负责搜索路线。',
 'static_map_builder.cpp':'以哈希体素维护可撤销全局/稳定/滚动局部占用和当前自由空间；普通轨迹清理与探索专属时间配对射线证据清理分支。',
 'static_path_planner.cpp':'指定目标的高度切片二维A*；全局参考+局部观测修订，三维清旧占用后再膨胀，搜索扩域、路线滞回、安全重连和前视航点。',
 'mavros_avoidance_controller.cpp':'起飞状态机、稳定地面基准、预发送/OFFBOARD/解锁、导航速度与风险仲裁、扫描/位姿新鲜度、悬停和探索授权短退路复核。',
 'pose_stability.hpp':'C++地面位姿连续稳定窗口；拒绝无效/重复/断流数据，为控制器初始化提供基准门控。',
 'system_watchdog.cpp':'监测关键话题消息年龄/频率并发布诊断，区分合法空对象和感知断流；不直接控制飞机。',
 'avoidance_command_bridge.cpp':'Prometheus可选控制接口：把导航/融合避让转换为UAVCommand，默认预览、时效保护及陷阱退让；当前MAVROS主演示不使用。',
 'goal_navigation.cpp':'Prometheus可选简单目标比例导航，生成XY速度/Z位置命令；自身不做货架A*搜索，非当前主演示控制器。',
 'loop_closure_backend.cpp':'独立关键帧回环后端：空间/时间候选、多帧Scan Context、ICP与门限、Huber稀疏SE(3)位姿图、重建轨迹/地图及候选诊断；主链关闭。',
 'exploration_search.cpp':'C++共享库加速八邻域Dijkstra，禁止对角切角，支持贴边软代价；经Python包装调用，不是新增探索理论。',
 'dynamic_point_filter.hpp':'早期无神经网络几何时序过滤：静态/动态/未知三态、历史体素/残差/地图支持与聚类；控制增量入图，主演示关闭。',
 'laserMapping.cpp':'FAST-LIO原估计主程序；我们的改动接入几何过滤、三类诊断点云和增量入图门控，保留原迭代滤波/点面配准。',
 'laserMapping_color.cpp':'原有带图像着色的FAST-LIO程序；本轮系列改动同步接入几何入图过滤，彩色功能不是我们新实现的R3LIVE。',
 'mid360.yaml':'FAST-LIO MID360话题/外参/预处理/发布/保存参数；增加原始地图发布相关配置注释，仿真类型由launch覆盖。',
 'mapping_mid360.launch':'FAST-LIO MID360入口；支持rviz、sim_lidar以及dynamic_filter；区分真实Livox CustomMsg与Gazebo PointCloud2。',
 'mapping_mid360_color.launch':'原彩色FAST-LIO入口增加动态过滤开关/参数，保持独立于普通无彩色主演示。',
 'fastlio_baseline_recorder.py':'长走廊基线记录：与Gazebo真值首帧刚体对齐，输出位置/偏航误差和航程CSV/摘要；不是最终回环Path评测器。',
 'long_corridor_baseline.launch':'装配原FAST-LIO与走廊基线记录器，用于退化场景误差观测。',
 'long_corridor_baseline.md':'长走廊退化实验的场景、命令、记录和评价边界说明。',
 'keyboard_control.cpp':'新增无人机终端键盘控制：模式/解锁、相对起飞、XYZ位置增量、降落/退出；不能与自动控制器同时写setpoint。',
 'keyboard_control.launch':'无人机键盘控制节点及步长/起飞高度等参数入口。',
 'long_corridor_baseline_flight.cpp':'长走廊确定性起飞、往返、驻留/降落航线及跟踪误差保护，给退化实验固定飞行任务。',
 'long_corridor_baseline_flight.launch':'长走廊自动航线的高度、长度、速度及行为参数入口。',
 'loop_closure_demo_flight.cpp':'回环专用方形闭合航线、返航悬停、误差守卫及AUTO.LAND收尾，用于录制回访数据。',
 'loop_closure_demo_flight.launch':'回环方形航线的起飞高度、边长、速度、驻留和降落参数入口。',
 'autoarming_control.cpp':'原有自动起飞后正方形/圆形位置航线示例；不是当前避障控制器，本轮Git范围未修改。',
 'position_control.cpp':'原位置控制可执行入口，创建DroneControl；不是A*或动态避障实现。',
 'position_control_lib.cpp':'原DroneControl库，把目标位置转换成MAVROS位置setpoint；保留教学代码频率/参数读取等限制，非主控制链。',
 'pub_origin.cpp':'原地理原点发布示例；设置经纬度/高度原点，不是起飞或避障节点。',
 'position_control.h':'原DroneControl声明与成员接口；本轮Git范围未修改。',
 'warehouse_explorer.py':'自主探索主状态机：黑箱观测窗口、前沿评分、可达路线、观察/补扫/返航/驻留/保存、失败记忆、停滞与新鲜度守卫。',
 'exploration_grid.py':'未知/自由/占用平面栅格、机体/未知边缘膨胀、观测驱动扩窗、图搜索、前沿和安全线段/受约束短退路。',
 'exploration_native.py':'ctypes加载C++搜索库并转换数组；库不可用时由探索栅格回退Python实现。',
 'exploration_coverage.py':'低增益时间窗口、前沿代表点/可见未知增益、已测表面多视角证据与补扫候选；质量分母不包含从未看到的表面。',
 'warehouse_map_audit.py':'离线读取PCD、场景与坐标对齐，以货架暴露碰撞盒侧面样本做欧氏近邻支持审计并输出缺口参考PCD；不向导航注入真值。',
 'startup_checks.py':'Python启动地面稳定窗口、落地/采集/墙钟新鲜度和独立墙钟截止检查，解决仿真初始化高度/时间基准错误。',
 'run_dynamic_comparison.py':'example/cangku内部badcase与ours一键运行器：环境、进程、控制、行人、录包、指标、持续展示和退出前导图；修复writer竞态和重复Ctrl-C。',
 'run_warehouse_exploration.py':'仓库探索运行器：统一仿真时钟、地面稳定后启动SLAM、行人、状态收尾、PCD保存和离线审计；可并行FreeDOM，不替换规划地图。',
 'save_navigation_map.py':'把维护cloud_map有限XYZ原样导出二进制PCD，用临时文件原子替换并保存来源/时间/坐标/点数/哈希JSON；拒绝空/过期数据。',
 'pedestrian_control.py':'example行人唯一转发者与终端遥控：自动阶段后接管、选ID、WASD移动、断流停车、冻结其他行人。',
 'warehouse_pedestrian.py':'仓库单行人AgentStates/模型驱动，响应同一键盘命令，断流停车，利用场景碰撞盒约束人工行人不穿货架。',
 'warehouse_scene.py':'生成cangku运行副本，保留货架/围墙，去旧state并解析mesh路径，增加行人；抽取盒体供行人约束和评估，不给飞行规划真值。',
 'exploration_pedestrian_walk.py':'探索演示行人先静止后在近场往返移动，最后释放给键盘，制造已入图人体离开的测试条件。',
 'moving_obstacle.py':'旧演示移动Gazebo障碍模型脚本，通过模型状态服务按给定运动更新位置；不是点云检测算法。',
 'analyze_dynamic_demo_bag.py':'离线从bag统计到达时间/误差、真值航程、人机中心距离、风险时间、对齐ATE和离开旧位置ROI点数。',
 'compare_dynamic_demo.py':'合并badcase/ours报告并绘制同坐标真值轨迹、相对变化；组合消融不能归因为单一算法。',
 'analyze_loop_closure_bag.py':'回环bag的在线raw/optimized Odometry轨迹对齐ATE、固定时间RPE与闭合误差报告/图；最终历史Path还需另外评估。',
 'validate_project.py':'静态核对包、XML/YAML/Python、消息、CMake目标/路径和安全默认值；不是飞行测试。',
 'dynamic_detector.yaml':'启动背景、体素、距离/高度范围和在线背景吸收参数，减少自运动/树叶候选混淆。',
 'dynamic_cluster.yaml':'聚类/关联、轨迹长度、速度/位移/线性度/连续帧、静态地图支持、输出与运动平滑分离参数。',
 'dynamic_risk.yaml':'动态最近接近预测时域、安全/紧急中心距离、释放/目标保持、速度平滑和消息超时参数。',
 'loop_closure.yaml':'关键帧距离/角度、回环候选间隔、Scan Context、ICP重叠/修正/高度门限、图优化与重建地图参数。',
 'static_risk.yaml':'当前扫描的静态净空、机体半径、高度范围、邻域支持、动态掩膜及风险滞回参数。',
 'static_map.yaml':'哈希体素、命中阈值、稳定/局部视图、动态轨迹、自由证据撤销、发布频率和容量参数。',
 'path_planner.yaml':'通用二维A*的分辨率、膨胀、高度带、局部重规划、前视、搜索和容量参数。',
 'warehouse_navigation.yaml':'仅仓库最后加载的命名空间覆盖：0.40m机体、0.80m规划膨胀、0.10m通用规划格、高度带、扩域/换路及低速静态风险。',
 'warehouse_exploration.yaml':'探索0.20m栅格/黑箱扩窗/20万格、未知边缘/净空、前沿/补扫/收敛、恢复、任务预算和返航保存参数。',
 'astra_mavros_controller.yaml':'主MAVROS控制器起飞/速度/比例增益、目标与中间航点容差、风险/数据超时等基础参数。',
 'avoidance.yaml':'静动态风险融合来源、有效期、限速和紧急权重参数，不是世界或航线。',
 'command_bridge.yaml':'旧Prometheus指令桥预览/控制、导航/避让仲裁与陷阱恢复参数。',
 'mid360_gazebo.yaml':'旧通用MID360仿真/建图接线参数，需与现用mapping_mid360的sim_lidar区分。',
 'astra_crossing_once.xml':'PedSim两个行人单次横穿路径/场景定义，保证初期演示有可控运动。',
 'perception.launch':'装配里程计/点云桥、前景/聚类、辅助预测/语义/显示、维护地图与可选回环。',
 'astra_dynamic_avoidance.launch':'实际MAVROS主演示装配：感知+动静风险+融合+可选A*+飞控+watchdog/RViz；最后可加载仓库profile。',
 'warehouse_exploration.launch':'探索专属装配：关闭通用A*、单一Explorer航点写者、保守证据清理、0.30m/s/新鲜扫描/紧急保护及地图保存。',
 'astra_pedestrians.launch':'PedSim横穿、Gazebo行人接线和唯一遥控转发节点；不再启动第二个Gazebo。',
 'avoidance.launch':'旧Prometheus风险融合、简单目标与指令桥装配，默认只预览。',
 'complete_system.launch':'旧通用perception+Prometheus avoidance完整装配，非当前MAVROS主演示入口。',
 'mid360_mapping.launch':'旧通用MID360建图装配入口。',
 'mid360_perception.launch':'MID360建图与通用感知/可选回环的组合入口。',
 'loop_closure.launch':'可选独立后端，默认关闭，加载回环参数。',
 'loop_closure_demo.launch':'回环专项后端/显示/评测相关装配，与普通动态演示隔离。',
 'loop_closure_synthetic_test.launch':'确定性合成回访点云/位姿与回环节点的联调入口。',
 'synthetic_test.launch':'不启动Gazebo的合成动态场景与感知/风险链烟雾测试装配。',
 'DynamicObject.msg':'单目标ID、位姿、速度、尺寸、置信度、可选类别和预测标记消息。',
 'DynamicObjectArray.msg':'时间戳/坐标系统一的一组动态对象消息。',
 'SemanticDetection2D.msg':'可选图像检测框/类别/置信度消息，用于几何对象附加语义接口。',
 'SemanticDetection2DArray.msg':'带帧/时间的一组二维语义检测消息；不是网络权重。',
 'astra_dynamic_avoidance.rviz':'显示当前扫描、维护地图/稳定图/局部自由空间、动目标、规划路线和飞行轨迹。',
 'warehouse_exploration.rviz':'探索显示：地图/路径、候选视点、栅格状态和任务进度，青色方点不是激光点。',
 'loop_closure.rviz':'回环原轨迹/优化轨迹、约束与重建地图的专项显示。',
 'test_demo_utilities.py':'离线回归PCD逐点一致、空地图守卫、行人接管/断流停车/其他人固定。',
 'test_runner_lifecycle.py':'不启动飞控的writer落盘/有效bag/重复SIGINT退出竞态回归。',
 'test_warehouse_scene.py':'仓库副本保留原几何和人工行人边界碰撞约束回归。',
 'test_exploration_grid.py':'未知不可穿、占用优先、扩窗坐标/容量、角点/线段安全和净空单调恢复回归。',
 'test_exploration_ros.py':'探索ROS消息边界、当前观测/断流及状态相关离线夹具回归。',
 'test_exploration_mission.py':'探索/补扫/重探/返航/驻留/保存/预算部分结束与显式失败的状态机单测。',
 'test_exploration_coverage.py':'可见增益、表面多视角、原生/Python搜索一致、审计分母和PCD/欧氏容差回归。',
 'test_startup_checks.py':'地面稳定、重复/失效样本、仿真时钟顺序、墙钟期限和起飞确认新鲜度回归。',
 'test_pose_stability.cpp':'独立C++连续稳定窗口测试，不连接PX4。',
 'test_map_evidence_live.py':'隔离真实C++地图节点的同输入静态保护/人体清理/射线/时效回归；不是真机。',
 'check_takeoff_controller_live.py':'隔离真实编译控制器与假飞控，覆盖起飞、断流、风险停车、受授权退路/前向刹车等分支。',
 'check_warehouse_clearance.py':'隔离真实风险/规划节点，检查窄通道阈值/滞回、高度、路线稳定、自由证据及航点容差。',
 'check_planner_snapshot.py':'把实际停滞地图快照重放给隔离规划节点，检查扩域/绕行；不向飞行器发指令。',
 'check_warehouse_multigoal.py':'显式选用的SITL多目标闭环测试，会发布导航目标，仅用于仿真。',
 'check_warehouse_demo.py':'显式选用仓库行人往返/检测/风险/清图/地图导出的仿真回归。',
 'check_live_demo.py':'普通demo结束采样后，显式驱动行人暂停/再动并检查检测、残影与导出一致。',
 'check_exploration_live.py':'只读探索实测采集，真值仅用于分析，不注入规划。',
 'check_exploration_controls.py':'仿真暂停/恢复与保存地图逐点一致性联调。',
 'check_rack_retention_live.py':'只读把多次已经观测的货架参考体素与维护地图匹配；保留率不是全仓覆盖率。',
 'test_exploration_mission_live.py':'独立ROS探索进程+合成房间+运动学跟随器验证终态；不是PX4或完整仓库实验。',
 'benchmark_exploration_search.py':'同输入C++/Python搜索耗时与结果一致微测，不能解释为整机加速倍数。',
 'synthetic_scene.py':'确定性室内静态点云、移动目标和里程计夹具，测试消息链路。',
 'loop_closure_synthetic.py':'合成关键帧回访与人为累计漂移，验证后端接受约束/优化/出图，不证明真实精度改善。',
 'dynamic_comparison_demo.sh':'example badcase/ours单模式运行入口，调用同一个Python运行器。',
 'warehouse_dynamic_demo.sh':'cangku仓库ours/单键盘行人入口，包装comparison --scene cangku。',
 'warehouse_exploration_demo.sh':'仓库黑箱自主探索入口；可选FreeDOM、手动行人、无GUI与保持展示。',
 'pc_dynamic_avoidance.sh':'旧多终端自动SLAM/动态避障入口，补环境和消息/插件检查。',
 'pc_example.sh':'原基础demo改成键盘飞机/建图，修复launch绝对路径、PX4/雷达/PedSim资源及启动等待。',
 'pc_example_tmux_backup.sh':'保留旧tmux基础demo命令快照，含旧环境别名/launch假设，不建议作为现用主入口。',
 'fastlio_long_corridor_baseline.sh':'退化长走廊的一键SITL、FAST-LIO、航线与误差记录入口。',
 'loop_closure_demo.sh':'回环场景/飞行/录包/评测一键入口，包含插件路径和残留进程检查。',
 'mid360.sdf':'修改真实扫描图样方向偏移+15度、向下覆盖更多；改发布PointCloud2以匹配仿真FAST-LIO类型；不是增大物理型号FOV。',
 'livox_points_plugin.cpp':'雷达Gazebo插件增加vertical_angle_offset_deg读取和射线角偏移；保留原CSV采样，修复接口使用中的类型不兼容。',
 'astra_example.launch':'PX4/example仿真入口改成项目根路径world，避免不存在的env_map查找；可被独立demo传入运行world副本。',
 'astra_long_corridor.launch':'PX4长走廊world/初始飞机配置装配。',
 'example.world':'修改原example行人插件/场景接线，配合PedSim横穿，不等于新创全部环境模型。',
 'RoboCup_sim_pedsim.world':'早期RoboCup场景与PedSim行人支持的组合world，保留原资源归属。',
 'long_corridor_degenerate.world':'规则长走廊退化场景，测试几何约束不足时的FAST-LIO表现。',
 'loop_closure_arena.world':'回环专项非对称墙/箱/圆柱/台阶场景和闭合航线空间，仍不能保证绝不误回环。',
 'cangku.world':'原有52货架仓库；我们使用运行副本添加行人和执行探索，原world不是本系列新建，Git范围未修改。',
 'lmnet_style_visualize.py':'离线抽取雷达帧、位姿/外参补偿、球面量程/残差和额外鸟瞰图，输出NPZ/JSON；不运行LMNet网络。',
 'plot_warehouse_run.py':'读取已有bag真值轨迹和场景盒体，画仓库全局/通道飞行与行人路线，只做评估。',
 'ours_demo_modules.dot':'Graphviz模块关系源文件，区分感知、长期地图规划、即时安全和控制。',
 'prepare_materials.py':'读取真实报告/PCD/录像，抽帧并生成组会实测图、逐架CSV和来源索引，不生成算法性能。',
 'validate_materials.py':'核对组会数据数值、路径/链接、媒体尺寸、报告哈希和文档范围。',
 'package_evidence.py':'复制真实视频/bag/PCD/图表/报告为完整文件合集，逐文件验证大小和SHA256；清单只作导航。',
 'content.py':'独立57页功能详解PPT的页面正文、简单流程和详细讲稿内容定义。',
 'build_deck.py':'不使用ppt-master的python-pptx原生可编辑文字/方框/箭头生成器及讲稿/简图导出。',
 'verify_deck.py':'检查57页PPTX/XML/备注/正文/源码哈希、PDF实页文字几何和排版，不是算法测试。',
 'verify_delivery.py':'旧22页功能PPT的实际PPTX/备注/源码/渲染交付验证工具。',
 'verify_revision.py':'42页原PPT按批注修订后的来源、页数、备注、文件和未改页验证。',
 'verify_later_revision.py':'42页PPT后18页修订与前24页保持不变、来源与实际导出验证。',
 'build_contact_sheets.py':'把实际PDF渲染页组合为联系表用于目视审查；不是无人机程序。',
 'AGENTS.md':'持续工作规范：先完整读双记录，后解决/核验，最后追加记录，保护用户修改及科研证据边界。',
 '交流与解决方案.txt':'逐轮需求、方案、实际改动、验证、失败与待办的主交流记录。',
 '学术关联与论文写作记录.txt':'方法/相关工作/论文表述及已有方法、工程、候选创新、实验证据归属记录。',
 '使用说明.txt':'交流记录的使用、持续追加和历史缺失范围说明。',
 'RA-L研究路线_2026-09-30.md':'基于现有平台的任务驱动占用核验研究建议、基线/消融/评价计划，未当作已实现贡献。',
 'preprocess.cpp':'FAST-LIO原预处理/传感器解析；现用MARSIM把逐点时间curvature置0，Git范围未修改，属复用底座。',
 'IMU_Processing.hpp':'FAST-LIO原IMU初始化/传播/通常的逐点补偿；当前MARSIM跳过逐点反向补偿，Git范围未修改。',
 'use-ikfom.hpp':'FAST-LIO原状态定义、惯性传播函数/雅可比接口，Git范围未修改。',
 'ikd_Tree.cpp':'原增量KD树近邻/插入/删除实现，非本项目新地图算法。',
 'ikd_Tree.h':'原增量KD树数据结构与接口，Git范围未修改。',
}

FREE_ROLES = {
 'node.cpp':'原作者ROS可执行入口，实例化FreeNode。',
 'freenode.cpp':'ROS参数、扫描/TF、线程和保存桥；本项目增加输入帧校验、短队列、保存完成回执与收尾统计/减少逐帧日志。',
 'freedom.cpp':'原作者FreeDOM扫描移除、自由空间估计、地图移除与静态空间整合主流程。',
 'mrmap.cpp':'原作者多分辨率地图、自由体素证据/状态及地图细化。',
 'scanmap.cpp':'原作者单次扫描地图/局部扫描体素表示。',
 'map.cpp':'原作者基础地图数据结构与管理。',
 'depth_image.cpp':'原作者深度/量程图构造，用于可见性与可选射线增强。',
 'raycast.cpp':'原作者射线遍历/自由空间相关计算。',
 'visualization.cpp':'原作者扫描/地图动态结果和静态点云可视化发布。',
 'ground_truth_generate.cpp':'原作者评估工具：生成参考静态/动态标注数据，非无人机在线模块。',
 'static_map_evaluate.cpp':'原作者静态地图质量评价工具，未据其存在声称本地复现论文指标。',
 'astra_mid360.yaml':'我们新增MID360/body/TF配置、独立地图话题和保守关闭未标定射线增强。',
 'astra_mid360_baseline.launch':'我们新增独立FreeDOM启动与保存路径参数，旁路同扫描比较，不接管导航。',
 'ASTRA_BASELINE_CN.md':'我们新增适配/启动/坐标/保存/上游版本与实验边界说明。',
 'common_types.h':'原作者通用体素索引/坐标/点等类型。',
 'utils.h':'原作者公共数学/索引/参数工具接口。',
 'point_types.h':'原作者支持的雷达点类型与PCL字段定义。',
}

def git(*args):
    return subprocess.check_output(['git','-c','core.quotepath=false',*args],cwd=ROOT)

def zpaths(*args):
    return {p.decode('utf8') for p in git(*args).split(b'\0') if p}

def role(path):
    p = Path(path); name=p.name
    if path.startswith(FREEDOM):
        if name in FREE_ROLES: return FREE_ROLES[name]
        if p.suffix=='.rviz': return '原作者数据集/室内FreeDOM可视化配置；不是当前ours主演示配置。'
        if p.suffix=='.h' and p.stem+'.cpp' in FREE_ROLES:
            return FREE_ROLES[p.stem+'.cpp']+' 配套类声明/接口。'
        if '/config/' in path and p.suffix=='.yaml': return '原作者其他雷达/数据集配置，当前MID360入口不加载该文件。'
        if name in ('livox_fov.png','velodyne_fov.png'): return '原作者特定传感器视场掩膜，不能直接当MID360有效自由观测域。'
        if '/launch/' in path: return '原作者数据集运行/离线评价入口；不等于本项目仓库适配入口。'
    if name in ROLES: return ROLES[name]
    if name=='CMakeLists.txt': return '本目录构建目标、依赖、消息生成及安装清单；区分主包/原作者库，构建存在不代表launch启用。'
    if name=='package.xml': return 'ROS包名称、编译/运行依赖和消息元数据。'
    if name=='.gitignore': return '排除本目录构建/运行生成物等，避免将日志/大bag/PCD等误当源码发布。'
    if name=='LICENSE': return '原作者许可证及复用要求，不是算法实现。'
    if name=='install_files.txt': return '第三方安装文件路径记录/安装副产物；不是我们新增的SLAM算法，不能据Git变动认定研发贡献。'
    if name=='objects': return 'EXP遗留空占位文件，不实现感知/控制功能。'
    if '/Log/' in path: return 'FAST-LIO运行生成日志，本地修改/提交痕迹不是算法代码改动。'
    if '/修改前备份' in path: return 'PPT修订前的备份副本；保留旧版本，不算独立新模块。'
    if '/sources/' in path: return 'PPT制作工作区归档来源副本；原件职责见对应源码/实验文件，不重复计算贡献。'
    if '完整证据素材包' in path or '（复件）' in path: return '证据合集/复件中的实际来源副本或索引；原实验/录像/图表不等于新增算法，副本保留追溯。'
    if name in ('README.md','ASTRADRONE_INTEGRATION_CN.md','DYNAMIC_DEMO_CN.md','EXPLORATION_DEMO_CN.md','LOOP_CLOSURE_CN.md','NEXT_STEPS_CN.md','WAREHOUSE_DEMO_CN.md'):
        return {'README.md':'本目录总体用途、接口、构建和运行说明。','ASTRADRONE_INTEGRATION_CN.md':'EXP迁移到AstraDrone/PX4/MID360的整体接线、环境与验证说明。','DYNAMIC_DEMO_CN.md':'example badcase/ours完整启动、行人遥控、展示/评测/持续运行和导图说明。','EXPLORATION_DEMO_CN.md':'黑箱仓库探索、行人、结束/保存、覆盖评价、安全与失败限制说明。','LOOP_CLOSURE_CN.md':'可选后端/专项闭环运行、可视化/评测与误闭环限制说明。','NEXT_STEPS_CN.md':'后续工程/研究计划；计划不计为已实现。','WAREHOUSE_DEMO_CN.md':'cangku单行人导航的成组启动/键盘、参数和地图/测试边界说明。'}[name]
    if p.suffix in ('.mkv','.mp4','.bag','.pcd','.npz'): return '实验数据/离线点云/抽帧数组产物；用于复查和效果展示，不是在线算法代码。'
    if p.suffix in ('.png','.jpg','.jpeg','.svg'): return '流程/实测图、视频抽帧或PPT页面/预览；具体用途由同目录说明与文件名定位，不作为新性能实验。'
    if p.suffix in ('.pptx','.pdf','.zip'): return '汇报交付、PDF阅读版或资料归档；不参与SLAM/避障运行。'
    if p.suffix=='.csv': return '实验指标/逐架结果/素材清单等结构化表，作用以表头和来源索引为准。'
    if p.suffix=='.json': return '参数、实验/素材索引、来源事实/哈希或文档验收数据，作用以本文件字段和同目录说明为准。'
    if p.suffix in ('.md','.txt'): return '研究/实验/教学说明、逐页讲稿、总结或验证报告；以标题/同目录入口为准，不是在线程序。'
    return '辅助文件；仅确认路径和Git范围，未据文件名推断新的算法或原创贡献。'

def group(path):
    for prefix,label in [(BRIDGE,'02_动态导航主包'),(FAST,'01_FASTLIO改动与复用'),(OFFBOARD,'03_飞行任务示例'),(FREEDOM,'04_FreeDOM作者实现与适配'),('EXP-main/','05_EXP原包留档'),('simulation/','06_仿真与传感器'),('scripts/run_sh/','07_运行入口'),('analysis','08_离线分析'),('交流记录/','10_记录与研究计划'),('third_party/','11_安装副产物')]:
        if path.startswith(prefix): return label
    return '09_文档汇报与项目规范'

def provenance(path,status):
    if path.startswith('EXP-main/'): return '用户原EXP包迁移留档；不是每个文件都由本轮重新设计'
    if path.startswith(FREEDOM):
        if Path(path).name in ('astra_mid360.yaml','astra_mid360_baseline.launch','ASTRA_BASELINE_CN.md'): return '本项目新增外部基线适配'
        if Path(path).name in ('freenode.cpp','freenode.h'): return '原作者FreeDOM实现加本项目ROS/保存适配；以源码注释为证'
        return '引入原作者FreeDOM文件，保留MIT归属；Git新增不等于原创'
    if status=='原有参考': return '基线已存在且未计作本系列改动'
    if path.startswith(BRIDGE): return 'EXP迁移后集成/新增/改进；具体函数原创性不能仅靠Git判定'
    if '/Log/' in path or 'install_files.txt' in path: return '运行/安装生成物，不计算法贡献'
    if '/sources/' in path or '/修改前备份' in path or '完整证据素材包' in path or '（复件）' in path: return '归档/备份/证据副本，不重复计研发'
    if path.startswith(('docs/','projects/','analysis_outputs/','loop_closure_runs/')): return '本系列整理/生成的文档或实验材料；不是在线算法'
    return '本系列Git范围新增/修改，复用底层方法归原作者'

def main():
    assert git('merge-base','--is-ancestor',BASE,'HEAD')==b''
    base_paths=zpaths('diff','--name-only','-z',BASE,'HEAD')
    staged=zpaths('diff','--cached','--name-only','-z')
    dirty=zpaths('diff','--name-only','-z')
    untracked=zpaths('ls-files','--others','--exclude-standard','-z')
    reference={OFFBOARD+'src/'+n for n in ('autoarming_control.cpp','position_control.cpp','position_control_lib.cpp','pub_origin.cpp')}
    reference |= {FAST+'src/'+n for n in ('preprocess.cpp','IMU_Processing.hpp')}
    reference |= {FAST+'include/use-ikfom.hpp',FAST+'include/ikd-Tree/ikd_Tree.cpp',FAST+'include/ikd-Tree/ikd_Tree.h','simulation/astra_gazebo_worlds/cangku.world'}
    reference={p for p in reference if (ROOT/p).is_file()}
    paths=(base_paths|staged|dirty|untracked|reference)
    excluded={p for p in paths if p.startswith(str(OUT.relative_to(ROOT))+'/')}
    paths-=excluded
    rows=[]
    for path in sorted(paths):
        p=ROOT/path
        statuses=[]
        if path in base_paths: statuses.append('基线到HEAD变更')
        if path in staged: statuses.append('当前暂存')
        if path in dirty: statuses.append('当前未暂存')
        if path in untracked: statuses.append('当前未跟踪')
        if not statuses: statuses=['原有参考']
        status='；'.join(statuses)
        # Hash authored sources/configuration, not multi-gigabyte experiment
        # copies: this inventory is not a redundant full evidence-pack audit.
        digest=''
        if p.is_file() and p.suffix in ('.cpp','.hpp','.h','.py','.sh','.launch','.yaml','.world','.sdf','.xml','.msg') and p.stat().st_size<2000000:
            digest=hashlib.sha256(p.read_bytes()).hexdigest()
        rows.append(dict(分组=group(path),相对路径=path,状态=status,归属=provenance(path,status),作用=role(path),存在=p.is_file(),字节数=p.stat().st_size if p.is_file() else 0,源码SHA256=digest))
    with (OUT/'逐文件作用全集.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    report=['# 逐文件作用全集','',f'基线 `{BASE}` → 当前HEAD `{git("rev-parse","--short","HEAD").decode().strip()}`，再合并暂存、未暂存与未跟踪文件。','',
            '范围是本系列本地Git可追溯的全部路径，另加入少量原有底座供解释。已忽略的运行目录/外部PX4和PedSim不由Git完整覆盖。复制的图表、PPT页面、备份也逐项保留，但不重复计作新算法；对于这类产物按用途类别说明，不假装逐字审查每份副本。','',
            '阅读顺序：先读同目录《FASTLIO2基础上的工作总览与核心文件说明.md》，再用本表查漏。文件状态只表示Git范围，不表示本轮编译或运行通过。','']
    counts=collections.Counter(r['分组'] for r in rows)
    for g in sorted(counts):
        report += [f'## {g}（{counts[g]}个文件）','', '| 文件 | 作用 | 归属/状态 |','|---|---|---|']
        for r in rows:
            if r['分组']==g:
                report.append('| ['+r['相对路径']+'](<'+str(ROOT/r['相对路径'])+'>) | '+r['作用']+' | '+r['归属']+'；'+r['状态']+' |')
        report.append('')
    (OUT/'逐文件作用全集.md').write_text('\n'.join(report)+'\n',encoding='utf8')
    uncovered=[r['相对路径'] for r in rows if r['分组'] in ('01_FASTLIO改动与复用','02_动态导航主包','03_飞行任务示例','04_FreeDOM作者实现与适配','06_仿真与传感器','07_运行入口','08_离线分析') and r['作用'].startswith('辅助文件')]
    check=dict(baseline=BASE,head=git('rev-parse','HEAD').decode().strip(),total=len(rows),groups=dict(counts),git_head_changed=len(base_paths),missing=[r['相对路径'] for r in rows if not r['存在']],unclassified_core=uncovered,report_self_excluded=len(excluded),coverage_ok=not uncovered and base_paths.issubset({r['相对路径'] for r in rows}),validation_scope='文件范围/用途归类/存在性及源码哈希；没有新ROS编译、SITL或性能实验')
    (OUT/'文件盘点核验.json').write_text(json.dumps(check,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(check,ensure_ascii=False,indent=2))
    return 0 if check['coverage_ok'] and not check['missing'] else 1

if __name__=='__main__':
    raise SystemExit(main())
