# IMX577／10.5mm镜头的半仓库仿真参数

2026-11-04。来源为用户提供的摄像头规格截图。当前半仓库RGB采集默认使用此光学配置，适用于有/无行人半仓库。通用D435i原始资产及旧数据、旧模型保持原参数；20㎡旧入口未指定此profile时仍保留自己的原视场。

## 已生效的几何参数

| 项目 | 仿真值 | 口径 |
|---|---:|---|
| 水平视场 | 33° | 用户给定，作为针孔渲染主参数 |
| 垂直视场 | 25.05083396° | 960×720、方形像素，由水平视场推导；规格25°为近似值 |
| 对角视场 | 40.63584358° | 同一针孔模型推导；规格41°为近似值 |
| 图像尺寸 | 960×720 | 保留低流量模式，不是厂家实际输出模式声明 |
| 更新率 | 3Hz | 保留采集设置，非厂家帧率 |
| fx、fy | 1620.45284284px | width/[2tan(HFOV/2)] |
| cx、cy | 480、360px | 理想居中主点，非真实标定 |
| D | 全零 | 理想去畸变针孔仿真，不代表实机无畸变 |

原相机水平视场1.211rad（约69.39°）/960图像fx约693.40px，新镜头明显更窄。同距离下可见范围缩小：不能把原宽视场航线预检/覆盖率原样套到新采集。

## 截图参数如何使用

10.5mm定焦、F/3.6、CMOS IMX577、12.33M有效像素、1/2.3英寸光学格式、截图中的y′数值、TV畸变<-1%、相对照度>80%、温度范围均已保存到profile元数据。

Gazebo此处是理想针孔光学模拟。光圈/景深、渐晕、CMOS曝光读出/滚动快门和温度响应没有据此建立物理模型；TV单项指标不能转换成唯一k1/k2/p1/p2，故没有编造畸变系数。12.33M是有效像素数，未给出实际输出宽高/裁切/帧率，先保留960×720和3Hz控制存储与内存。真实输出模式确认后可调整尺寸，必须同步重算K。

相机壳体、质量、安装位姿和光学轴沿用现有仿真安装，仅作占位，不视为IMX577实际机械尺寸/质量/外参。原MID360/内置IMU不变，不增加深度相机。

## 文件和数据链

- 参数：`simulation/camera_profiles/imx577_10p5mm.json`。
- 推导：`integration/roomgs/camera_profiles.py`。
- 生成SDF及ROS相机插件：`integration/roomgs/lowflow_camera.py`，半仓库入口`integration/half_gs/scene.py:camera`显式选择profile。
- 新采集在私有目录生成`sensor_models/D435i/model.sdf`（保留现有容器/挂载名称）和`camera_profile.json`，真实ROS CameraInfo仍是导出K的来源；profile及生成参数加入源哈希/快照。
- 新规划的可见性使用同一profile；旧路线在新采集副本标记`visibility_preflight_requires_recheck`，不改写原路线。
- 复用冻结参考时仍保留原85633目标/原宽视场eligibility，审计改用实际新CameraInfo和冻结的capture profile；不重筛/缩小原目标分母。旧无profile数据仍执行旧K检查。全覆盖阈值不变。

本轮没有重新飞全仓、收集新重建数据或训练新3DGS。当前已训练模型和本地查看器仍读取各自旧的camera_input.json；更换仿真参数不会改变旧照片或给旧模型增加新观测。后续应按窄视场重新检查/补足采集视角。

## 验证

四项新测试通过：生成SDF/FOV与ROS K一致、原资产/安装/质量未变、legacy相机保留原模式、冻结旧目标与新采集光学分离并拒绝篡改。原静态场景四项回归也通过，合计8项。

实际隔离Gazebo场景已运行，无PX4/飞行。订阅真实Image/CameraInfo：960×720 RGB8、K与目标精确一致、D全零；两个已知彩色球投影误差约0.884/0.683px，均小于2px验收界限。结果见`evaluation/imx577_camera_20261104/runtime/validation.json`和`actual_rgb.png`。

首轮光学夹具检查曾把世界坐标轴红线与红球合并计算质心而失败；保留在`runtime_initial_mask_failure`，分离最大连通色块后复测通过。该失败属于测试目标提取问题，不冒充相机参数问题已被额外修复。

实现依据：[Gazebo官方相机插件说明](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera)，核对horizontal_fov、输出尺寸和CameraInfo/Image接口；没有从文档推断未给出的实机标定参数。
