# 独立 R3LIVE＋MID360＋单目图像仿真 demo

这个入口实际运行 hku-mars 的 R3LIVE 激光惯性与视觉惯性估计、彩色点云融合，不是给 FAST-LIO 的地图额外上色。相机深度图没有送入算法。源码固定在 `6143a38537f28cb36eb24e9bbe2e39c8f7967157`，原始来源：[hku-mars/r3live](https://github.com/hku-mars/r3live)，保留原许可证和作者署名。

这是 **Gazebo/PX4 SITL 的独立传感器建图演示**。R3LIVE里程计经机体原点补偿送入MAVROS外部视觉，PX4融合后由现有单一OFFBOARD控制器执行固定航线。当前MID360机型关闭GPS/磁罗盘，因此这一反馈是完整飞行所必需的。本 demo 不实现动态物体去除、回环或三角网格；输出为真实 RGB 点云 PCD。真实 MID360、机载性能和真机反馈飞行尚未验证。

## 启动

当前机器已具备现有项目的 ROS Noetic、PX4 SITL、Gazebo 相机/雷达插件和 Livox 消息包。独立构建：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/build_r3live_demo.sh
```

运行带 Gazebo、R3LIVE 控制窗和 RViz 的完整 demo：

```bash
bash scripts/run_sh/r3live_demo.sh --keep-open
```

流程：生成带彩色几何特征的静态房间 → 等待彩色图/CameraInfo → 启动 R3LIVE 与输入桥 → 确认激光/视觉输出 → 里程计接入MAVROS外部视觉 → 等待FCU地面位姿/航向稳定 → 约1.5m起飞 → 2m边长方形航线 → 返回起点 → AUTO.LAND/落地上锁 → 调用原生地图保存 → 验证/输出报告。`--keep-open` 在成功后保留本次界面，Ctrl-C清理本次进程。

只运行无界面传感器静态检查：

```bash
bash scripts/run_sh/r3live_demo.sh --headless --no-flight --stationary-seconds 12
```

无界面完整固定航线：

```bash
bash scripts/run_sh/r3live_demo.sh --headless
```

已有 ROS/PX4/Gazebo/FAST-LIO 演示运行时入口会拒绝启动，不主动杀掉用户进程。构建使用独立 `r3live_ws`、单线程 clang；不会重建原 `AstraDrone_ros1_ws`。可选 CGAL 离线网格工具默认不编译，当前机器不需要为了彩色点云 demo 安装 CGAL。

## 查看结果

每次保存在新的 `r3live_demo_runs/YYYYMMDD_HHMMSS/`，不覆盖旧航次：

- `rgb_pt.pcd`：R3LIVE 自身 `Global_map::save_to_pcd` 导出的真实 XYZ＋RGB 二进制地图，只包含达到颜色观测门槛的点。没有由外部真值/插值补点、后处理去动态或伪着色。
- `colored_map.png`：从这份 PCD 读取的3D/平面预览。
- `result.json`：是否成功、激光/相机消息数、真实 VIO 更新计数、最大视觉跟踪数、颜色种类、点数、地图 SHA、落地上锁确认、源/二进制指纹。
- `r3live.yaml`、`sensor_geometry.json`、`camera_input.json`：本次实际参数、从SDF计算的设备外参、真实收到的 CameraInfo/图像编码。
- `sensors.bag`：激光、IMU、彩色图/标定、估计输出和状态。用于检查/后续回放；当前入口未实现独立 bag 重放模式。采用原始频率图像录制，文件大小取决于航程与图像压缩率。
- `demo_*.log`、`ros_logs/`：本次组件日志。失败同样保存 `result.json`，不能把失败目录当成功结果。
- `cost_time_logger.log`、`lic_lio.log`：原生运行日志，不是独立真值精度评估。

运行时原生输出包括 `/aft_mapped_to_init`、`/camera_odom`、`/camera_path`、`/track_img`、`/RGB_map_0` 等。RViz 使用作者的 RGB 分块话题显示地图。`/r3live/vio_status` 是新增的只读诊断，报告真正进入视觉ESIKF/光度更新的帧；不是只看到图像话题就视作融合成功。

手动保存可在本次私有 ROS master 环境下调用 `/r3live/save_map`（std_srvs/Trigger），或按 R3LIVE 控制窗中的 S。入口会自动完成保存，通常无需手动操作。

## 输入与标定

```text
Gazebo MID360 PointCloud2 ── input_bridge ── /r3live/input/cloud ──┐
Gazebo MID360 IMU ───────────────────────── /livox/imu ─────────┤ R3LIVE LIO/VIO
Gazebo 彩色 Image ─────── input_bridge ── /r3live/input/image ──┘ → RGB地图
```

输入桥把实际点写入作者需要的 `PointXYZINormal`；仿真现有 XYZ 回波是瞬时采样，明确使用 `curvature=0`，与当前项目的 FAST-LIO sim 处理约定一致，没有伪造100ms逐点时间。真实扫描不能使用这个模式。

彩色图实际发布话题在启动后由 ROS 检查确定；本轮观测为 `/camera/color/image_raw` 与 `/camera/color/camera_info`。桥将 `rgb8` 转成 OpenCV 需要的 `bgr8`，保留原始时间戳并选取约10Hz输入，原始相机配置仍为30Hz。

标定来自真实收到的 CameraInfo 和**设备本身**的 SDF 安装关系，不使用房间模型或无人机世界位姿拟合。关键细节：

1. 当前雷达相对 IMU 平移为 `[0.011, 0.02329, 0.00588]` m，不能沿用上游写死的 Avia 平移。
2. R3LIVE 参数实际表示 camera→IMU 的安装变换，不能被变量名 `i2c` 误导而取反。
3. Gazebo 渲染相机沿 +X 前视，图像针孔模型使用 +Z 前视。相机插件的 frame_id 没有自动提供这一转换，因此在标定生成器中明确加入 optical 旋转。
4. 原生 LIO 只处理平移杠杆臂；生成器检查雷达/IMU轴对齐，不会无声忽略非单位旋转。

## 代码位置与上游适配

- `integration/r3live/run_demo.py`：私有 ROS/Gazebo 启动、实际输入检查、固定SITL飞行、落地保存与验收。
- `integration/r3live/input_bridge.py`：点字段/逐点时间与 BGR 图像转换。
- `integration/r3live/calibration.py`：SDF刚体变换与渲染/光学坐标转换。
- `integration/r3live/scene.py`：独立静态房间，不改变既有世界。
- `integration/r3live/evaluate.py`：读原生 RGB PCD、输出预览与文件摘要。
- `integration/r3live/test_inputs.py`：真实逐点时间、禁止真实扫描伪时序和光学坐标的三项回归。
- `integration/r3live/upstream.patch`：对固定作者源码的全部改动；构建脚本幂等检查补丁，不强制覆盖不同版本或已有修改。
- `third_party/r3live/`：完整作者源码与Git版本。

上游小修包括：重复模板定义的clang兼容修复；CGAL离线meshing可选；雷达平移可配置；可关闭OpenCV控制窗；在视觉线程与LIO/渲染同步后执行排队保存，不启动外部pcl_viewer；空彩色图明确报错；发布真实VIO处理结果；原生LIO里程计使用扫描测量时间而非发布时刻。保留作者核心状态估计、地图、视觉跟踪和着色算法。

## 验证范围

当前完成情况以 `evaluation/r3live_demo_20261031/REPORT.md` 和对应航次 `result.json` 为准。静态检查、完整飞行、GUI、彩色点云保存分别记录；失败记录保留。三项输入回归不替代完整运行。

这台机器运行的是 R3LIVE 本身；完整飞行由R3LIVE原生LIO输出经现有frame/lever-arm桥送入PX4外部视觉，桥不读取Gazebo真值或使用PX4初始对齐。没有运行FAST-LIO估计器。本机MAVROS原配置为`vision_pose.tf.listen=true`，demo仅在本次生成的`sitl.launch`末尾覆盖为false，让插件订阅位姿话题；系统配置未改。未验证真机反馈飞行、动态物体清理、闭环优化、地图完整率/几何真值精度、真实MID360点时间、单目滚动快门或机载实时性。单相机只有其视场内表面能获得颜色，不能将未着色方向说成不存在。

## 换成实物 MID360＋单目＋IMU

不能直接用本仿真默认参数飞行。需要另配相机内参、camera→IMU/LiDAR→IMU外参、硬件/软件时间同步、雷达消息格式/逐点时间与IMU单位。`input_bridge.py` 的 `livox_pointcloud2` 分支只接受确有 `offset_time` 纳秒字段的消息；缺字段会拒绝，不根据点序猜时间。若实物发布 driver2 `CustomMsg` 或其他时间字段，需要先增加相应消息适配并重放验证。

本demo没有真机启动入口；先录实物包做离线状态估计与颜色投影检查，再做飞控坐标/连续里程计接口和故障保护验证。

## 本轮实测交付

最终完整GUI航次为`r3live_demo_runs/20261031_133755/`：已在R3LIVE外部视觉反馈下完成起飞、方形航线、返回、落地上锁与自动保存；原生PCD为42475个RGB点，817次视觉ESIKF与光度更新均成功，输入0错误，整体入口退出0。

[完整验证报告](../evaluation/r3live_demo_20261031/REPORT.md) · [彩色地图预览](../r3live_demo_runs/20261031_133755/colored_map.png) · [实际PCD](../r3live_demo_runs/20261031_133755/rgb_pt.pcd)。上述为仿真工程验证，不是实物MID360或机载性能验证。
