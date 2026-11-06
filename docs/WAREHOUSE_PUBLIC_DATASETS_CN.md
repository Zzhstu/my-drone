# 仓库/室内公开数据集选型（2026-11-03）

本次为官方数据说明和下载入口核查，未下载大数据、未接受访问协议、未训练公开数据。原无人机数据与模型未改。初轮按仓库场景选型；用户随后明确要求独立LiDAR点云、单目RGB、IMU，不依赖逐像素RGB-D深度。**当前优先级已更正为i2Nav-Robot、R3LIVE、FAST-LIVO2/LVBA**，原场景选型保留作背景。公开数据不能给本项目原仓库补出从未观测的真实纹理。

## 2026-11-03补充：按独立LiDAR＋RGB＋IMU筛选

“分文件保存”不是关键：RGB-D也能分开存。真正条件是RGB不附带完整逐像素实测深度，几何来自独立LiDAR回波，通过时间、内参和外参才能关联。我们的`recorded_topics.json`明确没有深度话题：`/lio_dataset/cloud`、`/cloud_registered`、`/livox/imu`、`/lio_dataset/rgb/compressed`等分别保存。

`integration/half_gs/depth_samples.py`生成的是实际扫描投影到图像的稀疏uv/z，不是相机深度。抽查本批中间一帧，960×720共691200像素，只有392个不同像素有最近扫描投影；单帧抽查不代表全集平均。训练时`integration/roomgs/train.py:depth_prior`也从累积实测点云做投影/z-buffer，在有值处监督，并未获得完整逐像素深度。

### A. i2Nav-Robot：优先同MID360硬件实验

[官方仓库](https://github.com/i2Nav-WHU/i2Nav-Robot)。官方ROS1话题表列有：

| 数据 | 官方话题 | 频率 |
|---|---|---:|
| MID360点云 | `/livox/mid360/points` | 10Hz |
| MID360内置IMU | `/livox/mid360/imu` | 200Hz |
| 左彩色图像 | `/avt_camera/left/image/compressed` | 10Hz |

相机为1600×1200彩色全局快门AVT双目，**本实验仅取左相机、不用右相机/视差/双目深度**，从而构成单目RGB＋独立MID360＋内置IMU。其他GNSS/轮速/Hesai/雷达不作该基线输入。官方有标定、ROS1包和下载入口，可优先核查`building00`/`building01`相关场景；不是仓库专项数据，视角覆盖尚未实查。

适配注意：作者的`mid360_imu`已从FLU改为FRD，不能直接套本无人机SDF外参；相机曝光起始时间、LiDAR自定义消息的实际包类型及相机到选定IMU的变换链需验证。官网全量约450GB，不下载全量；先选单序列或离线筛三话题。参考轨迹基准在ADIS16465中心，不直接当光学相机位姿。

### B. R3LIVE-Dataset：最明确的原始非RGB-D入口

[作者数据仓库](https://github.com/ziv-lin/r3live_dataset)。作者话题表是独立`/livox/lidar`、`/livox/imu`、`/camera/image_color/compressed`，分别10/200/15Hz，无完整深度图输入。LiDAR为Avia，**不是MID360**，但符合独立点云＋单目RGB＋IMU的采集原理。可先看官方标为室内的`hku_campus_seq_00`，不选故意退化序列做首个普通重建基线。

先从原始LiDAR/IMU估计轨迹与静态图，再以相机外参和图像时间求位姿、输入当前gsplat后端。不能直接复用我们的topic名和SDF标定，亦未保证新场景比仓库更清晰。

### C. FAST-LIVO2 / LVBA：最接近“图片＋PCD＋位姿”的离线入口

[FAST-LIVO2官方入口](https://github.com/hku-mars/FAST-LIVO2) / [Global-LVBA官方说明与下载](https://github.com/xuankuzcr/Global-LVBA#4-dataset-download)。后者明确其LVBA数据由原始FAST-LIVO2 rosbag生成，组织为`all_image/`、`all_pcd_body/`、`image_poses.txt`、`lidar_poses.txt`，另有可选的COLMAP/生成深度目录。

这是派生的估计点云/位姿集，不是独立新原始传感器基准；`depth/`为可选生成项，不代表深度相机数据。适合先测离线3DGS后端；需要完整前端对比时应使用原始FAST-LIVO2包。检查PCD为body局部坐标还是全局坐标、相机位姿方向/时间戳/内参后才能合并，不能直接叠所有局部PCD。

### 同类方法证据与其他候选

[GS-LIVM作者代码](https://github.com/xieyuser/GS-LIVM)明确输入图像、LiDAR和IMU，并提供R3LIVE/FAST-LIVO数据运行入口。这支持该模态组合可用于3DGS，不代表本机已部署或可不经适配直接跑MID360。官方论文[GS-LIVM: Real-Time Photo-Realistic LiDAR-Inertial-Visual Mapping with Gaussian Splatting](https://openaccess.thecvf.com/content/ICCV2025/papers/Xie_GS-LIVM_Real-Time_Photo-Realistic_LiDAR-Inertial-Visual_Mapping_with_Gaussian_Splatting_ICCV_2025_paper.pdf)的实验段通过索引核对；PDF直取403，未完整阅读/复现或移用性能结果。

其他已核官方README的候选：

- [M2TD](https://github.com/longlongzxl/M2TD)：手持/地面序列有MID360＋内置IMU＋Hikrobot单目RGB；UAV序列明确没有相机，不能因“无人机”而误选。隧道弱纹理，ROS2包，部分下载待发布，暂不优先清晰室内重建。
- [M3DGR](https://github.com/sjtuyinjie/M3DGR)：有MID360/内置IMU/普通RGB，也另含D435i深度。若接受仅禁止使用深度而不禁止数据包内存在深度，可仅取RGB/MID360/内置IMU；必须明确排除深度和其他IMU。`Dynamic01`官方约2.14GB/175秒，作动态候选；软件同步与rolling shutter是已知限制。若严格要求采集设备也不是RGB-D，相应不作首选。

当前推荐：**同硬件选i2Nav-Robot；最直接的原始独立模态选R3LIVE；最快验证离线图片＋PCD＋位姿后端选LVBA派生数据。**以上仅官方资料级筛选，未下载包验证、未编写新数据适配器、未运行新3DGS。

### 下载时的序列更正

进一步读取作者[逐序列JSON](https://github.com/i2Nav-WHU/i2Nav-Robot/blob/main/sequence_detail/i2nav_robot.json)后确认：`building00`为室外，不能仅凭building命名当作室内序列。当前优先`building01`（室内/室外，1369秒）；`parking01/02`为室内，但作者标有motion blur，不能承诺其图像更清晰。原建议building00只适用于同硬件测试。另可先选R3LIVE的`hku_campus_seq_00`（作者标为室内，202秒）做短序列验证。

i2Nav下载时除单序列ROS1 bag外，还要保留[calibration.yaml](https://github.com/i2Nav-WHU/i2Nav-Robot/blob/main/calibration/calibration.yaml)、[SHA256清单](https://github.com/i2Nav-WHU/i2Nav-Robot/blob/main/sha256sum/sha256sum.json)。下载网盘入口已从作者README核对；网盘页面无法由当前检索工具打开，不据此称失效，亦未验证登录后的文件或实际下载。

## 当前视角证据

从 `half_gs_dataset_runs/warehouse_half_20261103_capture/dataset/frames.json` 和冻结离线对齐重算相机世界坐标：高度10%/50%/90%分位约1.096/1.129/1.164m，最大约1.733m（起降最低帧不代表巡检高度）。光轴俯仰10%/50%/90%分位约-0.824/0.100/0.822度，垂直视场约54.875度。因此确实以低空、近水平视角为主。低空并不在几何上必然禁止拍高/低层，俯仰与站位能改变可见面，但遮挡/像素不足不能靠训练凭空补回；本轮仅选数据，不改相机或航线。

## 1. TorWIC-SLAM：真实仓库，首选

- 官方仓库：[TorWICDataset](https://github.com/Viky397/TorWICDataset#the-toronto-warehouse-incremental-change-slam-dataset)。作者README中的下载链接同时承载两个版本，须选 **SLAM** 版本。
- SLAM版：真实Clearpath仓库，20条轨迹/3种场景；两台Azure Kinect的RGB/深度、Ouster OS1-128三维点云、相机IMU、时间戳、内外参、LiDAR参考轨迹和独立扫描地图，含预测语义标签。以跨采集的半静态变化为主，不能直接等同连续行人运动基准。
- Mapping版：18条轨迹，D435i和Hokuyo二维激光，不能误当三维LiDAR版。
- 适配推断：可只使用一台相机RGB、三维LiDAR及相关IMU；忽略RGB-D深度是允许的实验设计，但需验证IMU频率/时间外参是否满足现有前端。用标定将LiDAR位姿变成光学相机位姿，转为现有帧索引、点云和Mask格式。
- 限制：地面机器人采集，不保证上下层/货架内面完整；参考位姿由扫描对齐地图得到。若直接用参考轨迹训练，只能说是已知位姿的3DGS实验，不能当本项目SLAM定位结果。场景变化期间不能将所有轨迹无条件合并为同一静态场景。
- 已核对官方README，未实取ROS包/标定文件；下载容量、具体视角覆盖、人物活动量和本机兼容性待样本核查。

## 2. ScanNet++：优先验证高质量室内3DGS

- [官方主页](https://scannetpp.mlsg.cit.tum.de/scannetpp/) / [数据文档](https://scannetpp.mlsg.cit.tum.de/scannetpp/documentation)。
- 提供室内DSLR图像、COLMAP相机/稀疏点、iPhone RGB-D及位姿等；官方已有3DGS示例。图像有33MP原始与较小分辨率版本。
- 适配推断：采用去畸变图像、对应内参/位姿和稀疏点，可测试当前gsplat后端；原数据集适配器要求的MID360轨迹链检查不能假装满足，须建立明确的公开数据入口，保持原无人机严格入口不变。
- 限制：非MID360采集、非专门仓库/动态去人数据；高质量图像不保证某一场景全表面覆盖。需注册并申请访问。默认整套数据约1.5TB，应只选单场景、单种图像版本；不要下载全套。
- 若采用扫描地图参与优化，应另报该输入条件，不用同一真值地图冒充独立几何评估。

## 3. Staer Warehouses v0.5：相似仿真仓库、多轨迹

- [作者数据卡](https://huggingface.co/datasets/staerrobotics/warehouses)。
- 数据卡列出Isaac Sim仓库15场景/147条轨迹，1920×1080双目RGB、深度、语义/实例图、IMU、逐帧相机位姿与内参。
- 适配推断：可取单目RGB+位姿测试3DGS，多条相同场景轨迹是选视角的候选；必须检查高度、俯仰和遮挡后才能说覆盖改善。
- 限制：数据卡未列MID360原始扫描，不能替代真实MID360前端验证；仿真深度/位姿须标为真值来源，不等同实机估计。未确认持续行人序列。官方列有轨迹覆盖、材质/光照等已知问题，不能称已完整。文件访问需登录接受条件，整库页面约123GB，应按单场景/轨迹取样。

## 4. 同硬件候选：MID360室内语义体素数据

- [Self-Collected LiDAR Dataset for Semantic Probabilistic Voxel Mapping](https://data.mendeley.com/datasets/8bt4kk8tmp/1)，DOI `10.17632/8bt4kk8tmp.1`。
- 发布者说明包含室内办公室的AVIA/MID360、RGB、IMU，ROS bag和离线文件、时间戳及标定，适合优先核查传感器格式迁移。
- 非仓库；只有首尾AprilTag对齐，没有连续全轨迹真值。当前只核实发布说明，文件列表/相机质量/实际下载内容未验证，暂不作为第一优先，亦不保证视角更全面。

## 另行排查但不优先

- [Fraunhofer Warehouse](https://mlps-lab.de/datasets/warehouse-dataset/)：真实仓库RGB、逐图位姿和标定；640×480、八相机，适合视觉定位或另建SfM/3DGS输入。未列MID360/IMU/三维扫描，不优先解决本轮清晰度与上下层问题。“horizontal/vertical trajectories”不能仅凭名称理解为低/高飞行轨迹。标注容量为分包5.9GB、货架测试3.7GB等，非商业条款见作者页。
- [TIERS Multi-Modal LiDAR Dataset](https://github.com/TIERS/multi_modal_lidar_dataset)：虽有MID360和IMU，官方室内话题表未列普通RGB相机；Ouster反射/量程图不是彩色RGB，不能只因同雷达就当彩色3DGS首选。
- [LiDAR Warehouse detection dataset](https://github.com/anavsgmbh/lidar-warehouse-dataset)：面向VLP-16点云目标检测；不能仅凭“仓库”名称认为拥有连续RGB/相机位姿。

## 建议的最小验证顺序（尚未执行）

1. 先取TorWIC-SLAM同一场景的一条短轨迹，检查标定、RGB、点云、IMU、参考位姿的实际格式和容量；不跨不同布局拼图。
2. 计算相机高度/俯仰分布，抽查上层/下层货架与两侧、遮挡后的观测。若仍缺视角，明示其只用于算法测试。
3. 参考位姿或估计位姿选择一种明确协议；后者需处理传感器话题/外参/时间，前者只评价后端。预测人体Mask与数据集原有预测语义标签分别说明，不冒充真值动态标注。
4. 单场景RGB按需选帧缩放，掩膜/内参同步调整；保留原帧索引，独立划分留出。初始点、轨迹、评价真值来源分开，避免数据泄漏。
5. 如果目标是排除“模拟画面/姿态误差”的影响，另选ScanNet++单场景，以官方输入和示例为参照测试相同后端。没有样本实测前，不承诺比现有仓库更完整或更清晰。
