# 资源受控的仓库重建

默认重建入口已使用低数据流配置。固定4407个表面目标、20cm地图支持、实际双视图和40cm平移基线、100%验收标准保持不变。静态资源预检通过不代表完成飞行或地图重建。

| 项目 | 当前配置 |
|---|---|
| 仿真相机源 | 私有RGB-only模型，640×480、5Hz；关闭不用的红外、深度及相机自身IMU传感器 |
| 相机几何 | 保留安装、FOV、机体质量和连接；显式沿用原RGB内参/零畸变 |
| RGB传输/存储 | JPEG质量92；R3LIVE读取同一JPEG解码结果，图像原始测量时间不变 |
| LiDAR输入 | 原传感器输出保留，适配器每6点取1点；静态实测约2914个有效点/帧 |
| 主IMU | 原MID360 IMU频率不变 |
| 采集缓存 | 16帧压缩图像，ROS写包缓冲32MB |
| 训练内存 | 图像逐张读取，深度先验LRU最多32帧，保留一个最新中间checkpoint |

原D435i资源不修改。每次运行在`sensor_models/D435i/`生成私有SDF，保留原来所有物理link、joint、质量和RGB图像几何，仅替换传感器发布方式。世界和货架不变。JPEG有损压缩及低频/低点数配置可能影响跟踪和细节，必须以本次实际定位和完整性结果判断。

录包保存全部**实际送入估计器**的压缩RGB、适配点云、原始主IMU、标定及必要定位/任务输出；不重复记录30Hz源RGB、原始稠密雷达、跟踪叠图。实时防撞仍读取原始雷达。这种包可以复核估计器输入，不能恢复未保存的原始稠密传感器数据。

## 容量预算

| 范围 | 应用预算 |
|---|---:|
| 采集目录 | 7GiB |
| 其中录包 | 6GiB |
| 训练产物预留 | 1GiB |
| 全航次含3DGS | 8GiB |
| 采集进程树RSS | 6GiB |
| 系统可用内存保留 | 1.5GiB |
| 磁盘空闲保留 | 20GiB |
| 训练显存 | 6GiB |

采集每5秒测量并写`resources.jsonl`和`resource_latest.json`；超过预算向唯一飞行控制器发送终止请求，正常降落并保存未通过结果，禁止训练。它是应用层采样检查，可能有短暂跨阈值增长，不是操作系统文件配额。训练另检查目录容量，图像与深度缓存不随总帧数无限增长。避免同时运行其他占满内存的训练任务。

先做静态预检：

```bash
bash scripts/run_sh/warehouse20_reconstruction_demo.sh --headless --resource-preflight
```

完整流程：

```bash
bash scripts/run_sh/warehouse20_reconstruction_demo.sh --headless
```

`--resource-preflight`只验证静态输入、配对、跟踪与资源；`result.json`中的地图`success`仍为false，另记录`resource_preflight_passed`，不能拿它启动训练。

## 已清理的数据

按2026-11-02要求，删除四次未通过实验的完整`sensors.bag`共164562424682字节（约153.26GiB），保留四个共415634851字节的关键事件片段。片段与原包选定窗口的每条消息内容、纳秒时间及话题多重集一致后才删除原包。净释放约152.87GiB。

所有已导出图像、PCD地图、位姿索引、诊断报告、源码快照和配置保留；原包删除后四次实验不能再完整原始重放。清单见`evaluation/warehouse20_20261102_resource/cleanup_manifest.json`。旧实验总目录约158GiB降至约5GiB，不能将操作系统可回收文件缓存当成同等大小的进程常驻内存。

第一静态低流量预检实测峰值进程树RSS约3.35GiB，输入错误0、153个精确配对、最多290跟踪点；35.2秒采样外推约2.11GiB/小时，仅限该短时静态测量，运动时容量须继续监测。RGB-only及后续飞行的实际结果见同目录报告，不预先声称质量或速度改善。

## 2026-11-02 已完成结果

完整数据包：`roomgs_runs/warehouse20_20261102_reconstruction`。采用已完成517站的原生58150点主图和四份补充来源49张真实图像，最终5435帧；最后一次低流量定向325站正常完成并落地上锁。其他辅助导航超时/报警原结果保留，**不是新的606站单次完整飞行成功**。主PCD未修改字节，配准只读取实测点云，没有以CAD或真值位姿求注册。

固定4407个目标的数据和学习后模型检查均100%，缺0。实际CUDA优化12000步，176986个高斯；680留出RGB视图PSNR29.6774dB。留出外观不参与颜色初始化/训练RGB损失，但前端几何与位姿来自整个采集，不是独立外部真值定位实验。标签、细边和部分地面仍有模糊；完整性仅指预声明20cm外露表面样本，非连续无限细节或遮挡内部。

整个数据包含图像/点云/模型约2.38GiB，低于8GiB预算；低流量采集进程树约3.5GiB，训练采样内核RSS高水位1.953GiB、PyTorch显存分配峰值291.625MiB。元数据/归档源码做独立副本，图像硬链接避免重复大拷贝；删去来源目录中的链接不删除包内图像。

模型：`gsplat/model.ply`、`model.pt`、`gaussians.npz`；原生图：`rgb_pt.pcd`；效果：`gsplat/interactive_before.png`、`room_six_views.png`、`walkthrough.mp4`；验收：`acceptance.json`。浏览器服务当前保留`http://127.0.0.1:8889`，交互使用SH0预览，图片/视频使用完整SH。

已实际核查PLY/NPZ均值/尺度/旋转/透明度逐点相同、参数有限，180帧/20fps/9秒视频，真实Chrome Connected与鼠标拖动前后像素变化55.84。11项控制/预算/纳秒JPEG恢复/完整性测试通过，位姿和图像篡改反例拒绝。

### 用现有数据离线复跑

目标目录必须不存在，避免覆盖实验：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse20_offline_bundle_demo.sh \
  roomgs_runs/warehouse20_20261101_201338 \
  roomgs_runs/warehouse20_reproduce_new \
  roomgs_runs/warehouse20_20261101_194006 \
  roomgs_runs/warehouse20_20261101_211603 \
  roomgs_runs/warehouse20_20261102_lowflow_full \
  roomgs_runs/warehouse20_20261102_targeted_aux
```

入口依次打包真实数据、实测点云配准/选择实际补充图像、验证来源、全目标审计，再训练。任何缺口都停止，不生成伪造观测。本次这些阶段已逐步实际完成；新的封装脚本做过语法检查，没有为验证包装再复制训练第二份模型。保存的包可独立重训：

```bash
bash scripts/run_sh/warehouse20_reconstruction_demo.sh --train-only roomgs_runs/warehouse20_20261102_reconstruction
```

已有模型会拒绝覆盖；需要新训练时使用新数据包目录。重新全场采集入口仍须看新航次的实测验收，不保证未知场景自主完整探索或真机安全。
