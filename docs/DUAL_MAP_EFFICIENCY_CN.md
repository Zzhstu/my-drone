# 双地图 Demo 的存储与仿真开销调整

2026-11-06，默认入口：

```bash
bash /home/a/AstraDroneOpen/scripts/run_sh/half_warehouse_dual_map_demo.sh --keep-open --keep-evidence-bags 0
```

本入口现在启用 `--efficient-demo --evidence-truth-hz 10 --evidence-map-hz 0.02`。

第二个行人（模型1，仓库右侧中部）默认在无人机首次超过0.9米高度后运动60仿真秒，然后停止。其速度仍为0.45m/s、往返路线仍为world x=1..5、y=-12。可用`--right-pedestrian-duration 180`显式恢复3分钟；旧三分钟专用入口的默认时长仍保留。

| 项目 | 调整前 | 调整后 |
|---|---:|---:|
| 录包 Gazebo 真值上限 | 30 Hz | 10 Hz |
| 录包 clock 上限 | 50 Hz | 10 Hz |
| 完整地图快照上限 | 0.05 Hz | 0.02 Hz，另保留最终快照 |
| 录包探索状态上限 | 1 Hz | 0.2 Hz |
| 录包鬼影历史上限 | 0.2 Hz | 0.05 Hz，另保留最终样本 |
| 路径/图版本诊断上限 | 1 Hz | 0.2 Hz |
| 状态历史 JSON | 每条消息 | 最多每2仿真秒一条，另存阶段变化及最终状态 |
| 注册点云录包 | 每帧，32字节点结构 | 每帧，12字节XYZ结构 |

注册点云不做空间或时间抽样，XYZ float32 位模式、点顺序、原头/时间戳保持；丢弃的是建图不用的强度/法线等字段及结构填充，不是原始传感器全字段归档。里程计、动态对象和 ACK 帧保持。作者 DUFO 与原维护引擎都以 XYZ 为几何输入。精简包仍不能重算完整原始 FAST-LIO 前端。

同时，在每航次 `<prefix>_models/` 生成私有 SDF：关闭未参与本 LiDAR 双地图任务的 D435i 彩色/双红外/深度和 FPV 共5路图像传感器及其视频流插件；保留模型质量/惯量、碰撞、关节和 D435i IMU。原 MID360、IMU、安装外参、物理1ms步长、控制和探索参数不变，原模型文件不修改，其他 RGB/3DGS Demo 不使用此开关。

文件 `<prefix>_efficiency.json` 记录私有模型和来源哈希，纳入航次 input_sha256；完整保存仍须通过同帧截止、点集/哈希与录包完整性校验。

同一个旧航次30仿真秒片段实测：121357147 → 66760584 bytes，减少44.9883%；290帧注册扫描全部保留，解码XYZ和消息头串流摘要一致。此数字只针对该固定片段，不能保证每航次同一压缩比例。

若需要旧显示/原记录条件，直接使用不带 `--efficient-demo` 的底层入口：

```bash
bash /home/a/AstraDroneOpen/scripts/run_sh/half_warehouse_two_pedestrians_three_minute_demo.sh --dual-map --max-wall-seconds 2400 --keep-open --keep-evidence-bags 0
```

历史实验未删除；`--keep-evidence-bags 0` 表示保留旧录包，会持续占空间。未显式传该参数时，沿用原有“新包验证通过后才保留最近一份”的策略，不删除地图和结果报告。

性能实测与资源竞争情况见 [本轮报告](../evaluation/dual_map_efficiency_20261106/REPORT.md)。录包大小、物理步进1000Hz、LiDAR扫描10Hz、Gazebo实时率是不同概念，不应仅根据文件大推断仿真变慢的根因。
