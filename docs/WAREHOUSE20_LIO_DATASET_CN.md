# 小仓库 LiDAR–IMU＋单目 RGB 数据集

本入口只采集数据，不调用 R3LIVE，不执行 3DGS 训练。场景沿用净 5×4×3 m、一个原有货架的小仓库，无行人；飞行使用已知模型离线规划的巡检路线，不是未知环境自主探索。

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse20_lio_dataset.sh --headless
```

默认在 `lio_dataset_runs/warehouse20_日期_时间/` 新建目录，不覆盖既有实验。静态预检可加 `--no-flight --stationary-seconds 20`，预检不代表飞行或完整地图。

## 数据流与坐标

MID360 实际模拟点云和其内置 IMU → FAST-LIO → 同一 `camera_init` 坐标系下的静态点云与 IMU 轨迹。RGB 不参与定位；按每张图片的原始采集时间，对实际 LIO 轨迹作有时间包围的平移线性插值和旋转 SLERP，最大包围间隔 150 ms，禁止外推。

`T_A_B` 表示把 B 中坐标转换到 A。相机光学轴为 x 向右、y 向下、z 向前：

```text
T_map_optical(t) = T_map_imu(t) × inverse(T_body_imu) × T_body_optical
```

全部安装变换由本轮实际使用的无人机、MID360、RGB 相机 SDF 解析得到，保存于 `sensor_geometry.json`。不能用 MAVROS 机体位姿替代相机位姿。FAST-LIO `/Odometry` 的 child 名称虽然是 `body`，源码中实际位置/旋转是 IMU 状态。

当前模型解析结果：机体至 IMU 平移 `[-0.011, -0.02329, 0.12412] m`；IMU 至雷达测量原点平移 `[0.011, 0.02329, 0.00588] m`；IMU 至相机光学原点平移约 `[0.131026, 0.055790, -0.094120] m`。旋转矩阵必须一并使用，不能只加平移。原 D435i 模型在本入口只保留其 RGB 流，关闭深度/红外/相机 IMU；雷达内置 IMU 保持原始频率。模型安装参数只对该仿真成立，真机须重新标定。

## 数据目录

| 路径 | 内容 |
|---|---|
| `static_map.ply` / `.pcd` / `.npz` | 实际 FAST-LIO 配准点按 5 cm 体素保留一个实测点，XYZ、米，无颜色 |
| `dataset/images/*.jpg` | 640×480、5 Hz 源流中的关键帧，JPEG92，文件名为原测量纳秒戳 |
| `dataset/frames.json` / `.jsonl` | 每图 `stamp_ns`、光学 `c2w`、SHA256、插值两端时间及权重 |
| `trajectory_imu.jsonl` | 原始实际 FAST-LIO IMU 位姿和时间，可重算相机位姿 |
| `camera_input.json` | 实际 CameraInfo 内参、畸变、分辨率 |
| `sensor_geometry.json` | 从实际使用 SDF 解析的机体/IMU/相机外参 |
| `dataset/poses.csv` | 相机位置与四元数，四元数顺序 x y z w |
| `dataset/transforms.json` | NeRF/nerfstudio 常见格式；转换为 OpenGL 相机轴 |
| `dataset/sparse/0/*.txt` | COLMAP 文本格式已知相机内参、w2c 位姿及 LiDAR 初始点；无 SfM 特征轨迹 |
| `coordinate_conventions.json` | 各格式轴方向、矩阵方向、时间策略及点颜色说明 |
| `sensors.bag` | 降流后实际使用的 LiDAR、内置 IMU、单 JPEG、标定和估计输出，不记录深度或真值 |
| `dataset_validation.json` | 每图解码/哈希、轨迹外参重算、时间包围间隔与数据指纹验证 |
| `quality_filter.json` / `excluded_images/` | 同一时间戳出现不同RGB内容时，从训练索引排除并保留原图与原因 |
| `coverage.json` / `missing_surfaces.json` | 仅离线按冻结参考评估完整性；未通过时保留缺口 |
| `result.json` | 采集成功、真实飞行结果、正常落地/上锁、完整性及资源状态分别记录 |

COLMAP 文本中的点颜色 `128 128 128` 明确表示未知颜色的中性初始化，不能当成测得的 RGB；点的 `ERROR=0` 仅为格式占位，不是实测重投影误差。后续算法可利用图片学习外观或先投影着色；某些加载器需要将文本转换成二进制，或者要求额外轨迹字段。此数据导出不代表所有第三方 3DGS 项目已接入验证。

当前录包在定位就绪后启动，因此最早几张静止图像/状态可能早于 bag 的录制起点。这些图片和状态仍由采集器直接保存，但不能从该 bag 独立重放；`bag_validation.json` 分别列出袋内精确匹配和袋外启动数据，不将袋外记录冒充成已录包。离线 3DGS 输入是保存的点云、图片、位姿与标定，原始 bag 用于复核/重算。

## 验证和资源边界

每张图片可从保存的原始 LIO 轨迹与 SDF 外参重新计算出相机位姿，验证不使用 Gazebo 真值。场景 CAD 与单次冻结初始真值对齐只供离线覆盖审计，不进入定位、相机位姿导出或点云生成。

保留既有 4,407 个可观测目标、20 cm 几何支持、至少两图/40 cm 平移基线标准。`success` 表示本次飞行和数据采集通过，`complete_map` 单独表示上述覆盖标准通过；两者不能混用，更不代表连续表面、所有不可见内面或照片级质量。

`geometry_complete` 单独表示点云几何支持通过，`photo_complete` 表示照片多视角条件通过。数据导出会先扫描已关闭的 bag；同一测量时间戳对应不同 JPEG 内容的已选帧被移入 `excluded_images/`，不修改时间戳补造位姿。该处理是质量过滤，未声称修复 Gazebo 图像时间戳重复的根因。

RGB 源 5 Hz，LiDAR 取原始扫描的六分之一，主 IMU 频率不变。图像等待缓存 16，轨迹缓存 200，静态地图上限 100 万个体素；总采集预算 7 GiB、bag 6 GiB、进程树 RSS 6 GiB，并预留 20 GiB 磁盘和 1.5 GiB 可用 RAM，超限请求正常降落。原有 3DGS 实验不删除、不重新训练。

格式依据：[COLMAP 输出格式](https://colmap.github.io/format.html)、[nerfstudio 数据约定](https://docs.nerf.studio/quickstart/data_conventions.html)。当前仿真保留已有雷达视场设置，不能据此推断原装 MID360 的实际可见范围。

## 2026-11-02 本轮实际数据

目录：`lio_dataset_runs/warehouse20_20261102_capture/`。606/606 实际航点完成，正常落地上锁；最终 4,758 张有效图片、18,782 个原始 LIO 状态、80,900 个实测体素点。初始 4,760 关键帧从同一 bag 补入 2 张实际图片，再排除 4 个时间戳歧义帧，原始图像和 bag 均保留。全部图片哈希/解码和轨迹外参重算通过，最大 LIO 插值包围间隔 129 ms，禁止外推。

固定目标几何支持 4,407/4,407；照片多视角支持 4,397/4,407（99.773%），10 个货架样本仍不满足条件，见 `missing_surfaces.json`。因此几何完整性通过，严格联合完整性未通过；不将两者混称“完整3DGS”。本轮没有训练 3DGS。

数据约 1.68 GiB，记录到的采集进程树峰值 RSS 约 3.615 GiB。4,756 张有效图与 bag 精确字节匹配，另外 2 张是 bag 启动前采集器直接保存的静止图；录制区间内无缺图。独立格式往返及真实JPEG/1cm位姿篡改拒绝检查通过。最新入口又完成一轮 20 秒静态预检，验证独立参考资源、自动源码快照与时间戳歧义过滤链路；这不是第二轮完整飞行。
