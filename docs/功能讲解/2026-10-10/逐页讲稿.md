# 01_overview

这份讲解不是把所有代码都叫作避障，而是分清谁负责位置、谁维护地图、谁选路线、谁决定此刻怎样飞。整个过程持续循环：得到观测，更新判断，执行飞行，再从新视角获得下一轮观测。下面每个功能都有作用、规则和流程，先读模块页，再回到第二十一页总流程就容易串起来。

本稿根据二〇二六年十月十日当前工作区整理，流程图是机制示意，没有新增飞行、识别精度或性能实验。完整文字在/home/a/AstraDroneOpen/docs/功能讲解/2026-10-10/功能包作用与运行规则.md。具体源码主要在/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge。

---

# 02_packages

功能包可以理解为一个工具箱，内部文件则是不同工具。上次列出的动态检测、跟踪、风险判断、地图、路径和控制文件，大部分都在同一个fastlio_bridge包里，不是八个独立包。FAST_LIO负责估计位置并提供整理后的扫描；fastlio_bridge把它们接到动态环境的地图、规划和控制链；offboard保留键盘、指定位置和预设航线的独立飞行示例；仿真相关包提供仓库、无人机、雷达和行人。世界有真实布局，不等于探索器拿真实布局来选路。

独立飞行示例和主演示不能同时抢同一架无人机的指令。当前主链装配看/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch/astra_dynamic_avoidance.launch及launch/warehouse_exploration.launch。示例源码在/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src。

---

# 03_localization

定位不是只对惯性数据积分。惯性测量预测短时间内的运动，但误差会积累；激光扫描与已经存在的环境表面比较，为位置和方向提供修正约束。修正后的状态又用于整理扫描和更新匹配地图，下一帧继续预测和比较。输出位置、姿态以及当前扫描，其他模块才能在统一参考下判断变化。

当前主演示启用模拟雷达分支。模拟点没有真实逐点采集时刻，这个分支不执行与真实雷达相同的逐点去畸变，讲一般算法时不能说该仿真已经验证了这一能力。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp、src/IMU_Processing.hpp和src/preprocess.cpp。启动分支可看同包launch/mapping_mid360.launch及fastlio_bridge/scripts/run_warehouse_exploration.py中的sim_lidar参数。

---

# 04_alignment

如果定位和飞控采用不同原点或方向，同一障碍的位置就可能被理解错。衔接程序不只是改消息标签，而是实际旋转和平移位置及点云，让感知、地图、规划与控制说同一种位置语言。主演示初始化时借助飞控位置建立一次转换，之后每帧输出由激光定位驱动，不再持续拿飞控轨迹替代定位结果。

转换后的消息供下游使用，位置也按取样间隔记录已飞航迹，轨迹有最大长度避免无界增长。主演示不把借助飞控对齐后的结果又反馈给飞控，避免循环依赖。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/fastlio_bridge.cpp与src/cloud_bridge.cpp，装配在launch/perception.launch。

---

# 05_foreground

前景候选模块的作用是缩小后续分析范围。开头四十帧构成背景，后续新扫描按小空间格压缩，并只看无人机附近和指定高度范围。如果这个位置及附近格已经有背景支持，就不输出候选；与背景不同才交给跟踪模块。这一条件只能说明变化，不能直接证明物体正在移动。

无人机转到货架背后，新出现的静止表面也可能成为候选。当前关闭永久在线背景吸收，避免短暂停留的行人被不断吸收为永久背景；但是开头已存在且不动的人仍可能进入初始背景，因此不能叫人体识别器。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/dynamic_detector.cpp，参数在config/dynamic_detector.yaml，输入是当前扫描而不是累积全图。

---

# 06_tracking

先把距离较近的点组成目标，再把本帧目标与历史目标配对，保留同一个编号。配对检查合理位移，避免把两个不同物体串成一条移动轨迹。判断不仅看速度，还看一段时间的净位移、方向稳定性和轨迹是否符合连续运动。当前要求连续四次运动条件成立，单帧点增多不立即确认。

稳定地图的充分支持可以否决动态判断，减少树叶或货架边缘的静态误报；代价是可能压制被地图支持的真实运动，需要评价漏检。显示位置采用较快平滑，运动判定历史采用独立的保守平滑，因此球的位置更新和动态确认不是同一个速度。目标丢失超过允许帧数会删除。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/dynamic_cluster.cpp，参数在config/dynamic_cluster.yaml。

---

# 07_auxiliary

跟踪目标可以分成两条辅助支路。短时预测假定速度暂时不变，推算半秒后的目标位置，但转身、加速和停下都会让简单外推不准确。主碰撞判断直接读取当前位置和速度，自己计算双方的接近关系，不需要等辅助预测消息才工作。

类别接口默认关闭并原样转发目标。只有启用并收到有效图像框、相机参数及坐标转换时，才尝试把三维目标对应到图像类别框。这段程序自身不运行人体网络，消息名字包含类别也不能证明已经识别人。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/dynamic_predictor.cpp和src/semantic_fusion.cpp，开关在launch/perception.launch。

---

# 08_dynamic_risk

动态风险同时观察无人机和目标的位置、速度。把目标速度减去无人机速度之后，可以判断双方在接下来短时间内会不会接近、最近距离多大，以及最近接近发生在什么时候。高度关系也要检查，否则高度上明显错开的目标可能造成多余避让。

当前预测范围是一点五秒，普通水平安全距离是一点三五米。当前距离小于零点七五米，或者零点八秒内预计接近至零点四五米等紧急条件成立时，进入更高等级。保持和退出条件让边缘情况不反复切换。主要依据目标中心，不是完整人体形状的精确碰撞判断。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/collision_risk.cpp，参数在config/dynamic_risk.yaml。

---

# 09_static_risk

长期地图可帮助绕路，但此刻附近空间需要新扫描复查。静态风险先保留飞行高度附近的点，用邻域支持和稳健距离削弱孤立噪点，并在已跟踪动态目标周围作短时遮罩。这样动态目标交给动态风险处理，静态风险尽量关注近场环境净空。

当前仓库配置把自身水平半径零点四米与额外余量相加。普通安全余量零点三米，对应中心到障碍零点七米；紧急余量零点二五米，对应零点六五米；退出余量零点三五米，对应零点七五米。不同触发与退出条件减少反复切换。这是静态风险距离，不能直接替代规划网格的膨胀和未知空间检查。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/static_collision_risk.cpp，默认在config/static_risk.yaml，仓库覆盖在config/warehouse_navigation.yaml。

---

# 10_fusion

动态和静态两路都可能提出风险，融合模块让它们通过统一接口进入控制。最终风险等级取两者更高值，但方向建议不是永远从同一个来源取。双一级风险的速度建议求和，双二级按动态零点六、静态零点四加权；若方向抵消，优先静态安全方向。只有一路更紧急时采用对应策略。

融合后速度有限幅和平滑，当前融合速度上限零点八米每秒，这不是最终飞行一定达到的速度，控制器还有自己的上限。来源过期后不参与融合，但排除过期来源本身不是完整失效保护，控制器仍要判断必要消息是否新鲜。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/avoidance_fusion.cpp，参数在config/avoidance.yaml。

---

# 11_maps

地图维护在同一份空间格存储中积累占用与自由证据，再按用途输出不同视图。默认全局视图要求两次命中，稳定视图要求八次，局部视图保留近期附近占用，所以它们不是三套彼此无关的数据。全局用于长期选路，稳定支持可以帮助抑制感知误报，滚动局部图反映附近近期变化。

同一空间证据仍可被后续清空信息更新，不是次数多就永久锁死。这里发布的/uav1/fastlio/cloud_map由衔接包维护，不能说已经直接删除FAST-LIO内部用于匹配的树节点。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/static_map_builder.cpp，参数在config/static_map.yaml，启用及发布连接在launch/perception.launch。

---

# 12_ghost_removal

人离开之后，能否撤销旧位置的点，要看新扫描是否真的看穿那里。程序使用匹配时刻的无人机位置、姿态和雷达安装偏移计算射线起点，再检查沿途自由空间。没有扫到和被物体挡住都不是清空证据；新扫描端点及邻域受到保护，避免把当前真实表面误删。

当前普通占用需要连续三次、稳定占用六次清空证据，还检查旧点年龄和证据时间间隔。仓库探索采用自由证据删除，不按人体外形凭空抹点。地图更新后下一轮规划重新评估旧路线。静止的人仍是眼前真实障碍，安全规划不应该为了图好看忽略他。被长期遮挡的旧点也不能保证立即清掉。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/static_map_builder.cpp，探索覆盖参数在config/warehouse_exploration.yaml。

---

# 13_goal_planning

通用路径规划器的作用是去目标背后时会找绕行路线，而不是只远离障碍。它把飞行高度附近的三维占用投到空间网格，用网格搜索计算路线。先由稳定地图提供参考，再用有证据的自由空间撤销旧占用，补入当前局部障碍，最后扩大障碍安全边界并重新找路。清理应在原始占用上进行，不能随意擦掉膨胀安全区。

局部搜索范围失败时会从四米扩大到八米、十二米，不意味着未知格自动变安全。旧路线仍安全时，新路线要明显缩短，当前改善门槛约百分之二十五，才切换方向，减少抖动；旧路被挡则重新计算。只发布直线安全可见的下一航点，避免跨折线切角。这是固定飞行高度规划，不会主动升高越过货架。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/static_path_planner.cpp，参数在config/path_planner.yaml与config/warehouse_navigation.yaml；自主探索时关闭此程序。

---

# 14_exploration

探索器不是按预先写好的仓库货架路线飞行。开始时只有有限计算窗口，从实际扫描更新未知、自由和占用状态；靠近窗口边缘再扩展，新增格子仍然未知。从已观测安全区域寻找可到达的未知边界观察位置，比较能多看到多少信息和走过去的代价，决定下一站。新视角获得扫描后再循环。

未知区不能直接当成自由通道。长期没有进展的目标暂时降权，避免一直卡在同一货架。正常安全余量零点八米，起点受新障碍影响时才允许在已知空间中寻找净空不减的短退路，恢复下限零点四五米并不替代日常安全距离，控制器会再次检查扫描。当前启动文件覆盖控制速度上限为零点三米每秒。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/warehouse_explorer.py、scripts/exploration_grid.py、src/exploration_search.cpp与scripts/exploration_native.py。参数是config/warehouse_exploration.yaml，覆盖值还要看launch/warehouse_exploration.launch。

---

# 15_finish

探索进展很低并不立刻结束。当前九十秒窗口中新自由面积和新表面增长较少时进入检查，优先补扫已经发现但证据薄弱的表面。表面质量关注多次采集、至少两个观察方向和一定基线，而不是飞近一次就算看清。仍有可达收益就继续探索或补扫，无收益且满足质量等条件才表示已观测可达区域收敛。

达到预算上限也会返航，但标记为部分结果，不是假装完整。返航需要在起点零点三米内连续停留五秒，且确认保存后才完成；返航受阻或超时不算成功。当前已测表面百分之九十五门槛的分母只包含已经看到的表面，不包括从未看到的货架，因此不是全仓库完整率。独立覆盖验收看/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/warehouse_map_audit.py，状态与表面规则在scripts/warehouse_explorer.py及scripts/exploration_coverage.py。

---

# 16_control

控制器先确认地面位置稳定，持续发送设定值，申请控制模式和解锁，再起飞。起飞确认结合空中状态、相对地面抬升和稳定停留，不能只看错误的绝对高度值。进入任务后按航点误差计算水平速度，单独保持高度，并限制最终速度。

正常风险下跟随航点；一级预警默认停下等待，符合条件的静态恢复可以短退。普通导航二级风险可以执行融合建议，探索任务则开启紧急水平停等，仅在探索器授权并通过最新扫描检查时做短恢复。必要位置、风险或扫描过期时悬停，不盲目赶路；正常探索前进也检查前方净空。最后经MAVROS送给PX4，由底层飞控做姿态和电机控制，本模块不是全部飞控。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/mavros_avoidance_controller.cpp，参数在config/astra_mavros_controller.yaml以及仓库两个覆盖文件。

---

# 17_visualization

显示窗口把消息画出来，而不重新决定动作。当前扫描和维护地图是不同来源；已经飞过的轨迹与准备走的路线也不同。目标球表示跟踪中心，不是精确人体轮廓，更不是仿真中新生成的实体。探索中的青色方块是候选观察位置，不是雷达扫描的大方块。

标记依靠编号和短有效期更新，目标消失后不会永久留球，航迹也有最大长度。没有球可能是当前没有确认目标，不能凭显示认定程序停止；需要结合下一页消息监测。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/dynamic_marker.cpp、src/fastlio_bridge.cpp及scripts/warehouse_explorer.py，显示配置在rviz/astra_dynamic_avoidance.rviz和rviz/warehouse_exploration.rviz。

---

# 18_monitoring

运行监测观察定位、扫描、目标、风险及速度建议是否持续到达，统计消息数量、频率和最新消息距现在多久。先检查有没有持续更新，再看消息内容。如果目标列表持续更新但为空，可能只是当前没有目标；如果长时间没有任何更新，才应检查上游节点、连接和日志。

监测输出给人诊断，不直接接管飞行，真正的数据过期停车在控制器内。诊断正常也不能证明地图完整或没有碰撞，任务效果还要用对应证据评价。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/src/system_watchdog.cpp，装配和参数可查launch/astra_dynamic_avoidance.launch与launch/warehouse_exploration.launch。

---

# 19_map_save

地图保存器把实际维护话题中的结果带到离线，不按世界真值补点或裁掉人体来造一张好图。它检查地图和扫描接收时间，过期超过三秒或没有有效点时拒绝保存，不用坏数据覆盖上次有效结果。只保留有限坐标，先写临时文件并完成写入，再替换正式文件，减少中断留下半个地图的风险。

同时写来源说明，包括话题、参考系、采集时刻、点数和校验值；失败要给出原因。默认产物在/home/a/AstraDroneOpen/dynamic_demo_runs，运行前缀关联最后的_final_map.pcd。文件存在证明保存成功，不自动证明完整或者无鬼影，遮挡下的旧点需要重新可见才有清理证据。源码是/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/save_navigation_map.py，服务是/demo/save_map，最终位置由运行器和启动参数决定。

---

# 20_launch_examples

演示脚本准备环境并装配仿真、定位和所选任务，组织日志、录制、退出和地图保存。它们不是另一套避障算法。指定目标任务由通用规划器发航点，自主探索任务关闭通用规划器，由探索器发航点；两者共用感知、地图、风险和控制链。只保留一个航点指挥者才能避免相互覆盖。

MissionControl的offboard/src包含autoarming_control.cpp自动解锁、keyboard_control.cpp键盘飞行、position_control.cpp定点飞行、long_corridor_baseline_flight.cpp走廊航线、loop_closure_demo_flight.cpp回环航线、pub_origin.cpp原点发布以及position_control_lib.cpp公共控制库。这些是独立飞行示例，不是主避障控制器的并行步骤。行人键盘控制改变仿真人的位置，不是键盘飞无人机。当前入口在/home/a/AstraDroneOpen/scripts/run_sh/dynamic_comparison_demo.sh、warehouse_dynamic_demo.sh与warehouse_exploration_demo.sh；装配源码在/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch。

---

# 21_system_flow

总流程先由激光和惯性定位，再通过衔接形成统一位置与当前扫描。此后不是一条完全串行链，而是两路并行决策：长期路线由地图维护和选定任务规划产生，指定目标与探索二选一；即时安全从当前扫描和跟踪目标分别判断动静风险，再融合为统一等级和建议。两路在同一控制器汇合，经过当下检查后才送给飞控执行。

执行改变观察位置，新视角产生新扫描，下一轮又更新地图和判断。显示、监测、日志和地图保存是读取实际消息的旁路，不应画成必须等待显示完成才能飞行。探索器也直接消费当前观测，这张总图抽象了共享输入，没有表示它只能读一张累积全局图。实际装配可核对/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/launch/astra_dynamic_avoidance.launch、launch/perception.launch和launch/warehouse_exploration.launch。

---

# 22_summary

可以用三句话介绍当前工作：先把位置、扫描、地图和控制放到统一参考系；再持续维护空间证据，保留稳定结构，同时用新的可见证据重新检查旧占用，使局部变化影响路线；最后由目标导航或探索选下一站，由即时风险和控制决定此刻是否行动。这个协作过程比单纯说实现了避障更准确，也更容易与具体代码对应。

本稿只解释已经存在的工程机制。是否动态识别足够准、全仓库地图完整、完全无碰撞或者机载运行够快，都要对应实验，不能用流程图代替证明。具体作用、规则、参数和完整源码路径见/home/a/AstraDroneOpen/docs/功能讲解/2026-10-10/功能包作用与运行规则.md；本轮没有修改飞行程序，也没有新增实验或原创贡献主张。
