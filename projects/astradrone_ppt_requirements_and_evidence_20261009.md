# 本轮汇报需求与原始数据核对

日期：2026-10-09，Asia/Shanghai。此文件是 PPT 的来源材料，不是已经确认的页面设计，不代表 PPTX 已生成。

## 用户最新要求

制作可编辑、美观简洁的中文项目 PPT；在清晰排版的前提下包含较充分的解释文字和流程图，不以代码截图堆满页面。需要同时解释 `/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard/src` 的全部七个 C++ 文件及它们与当前主链的关系。

叙述顺序：

1. FAST-LIO2：原论文、原作者创新、原理、工作流程和项目使用方式。开场不把它定性为“只是一个前端”；回环及我们加入回环的失败经验放在后面的局限讨论。
2. 自己的工作：实际新增的模块、模块运行原理与规则、整体数据流、Offboard 控制源码、badcase/ours 效果。
3. 室内仓库：导航、动态避障、建图、黑箱自主探索；与已经迁移的 FreeDOM 地图基线比较，保留不足与实验条件。
4. LMNet：原工作的方法及输入，当前离线残差图的真实完成程度，后续边缘端在线人体去除与更一般动态去除的研究路线。明确“静止人体语义”与“正在运动的物体”不是同一个标签。

用户没有指定具体时长、页数、固定模板或是否委托全部设计判断。应由 PPT Master 的确认阶段确定，而不是假造确认。

## 已重新读取的原始数据

以下数值于本轮重新读取原始 JSON，不是重新飞行得到的新数据。旧图表和录屏只作与其来源范围匹配的证据。

### 9 月 23 日内部 badcase/ours 组合对照

原文件：

- `/home/a/AstraDroneOpen/dynamic_demo_runs/badcase_20260923_201211_metrics.json`
- `/home/a/AstraDroneOpen/dynamic_demo_runs/ours_20260923_200322_metrics.json`

| 指标 | badcase | ours | 解释边界 |
|---|---:|---:|---|
| 到达时间（仿真秒） | 61.698 | 37.377 | 单次组合对照，不是统计成功率 |
| 到目标后的 XY 航程（m） | 20.29634 | 12.42338 | Gazebo 轨迹来源，与探索航程口径不同 |
| 离开区域稳定地图点数 | 128 | 0 | 特定 ROI，不代表全图所有鬼影为零 |
| 终点 XY 误差（m） | 0.13222 | 0.17076 | 该指标没有改善，必须保留 |
| 最近人机中心 XY 距离（m） | 0.97905 | 0.96502 | 中心距离不是碰撞盒净空，也没有改善 |
| 到达判定 | true | true | 仅对应以上报告，不绑定 9 月 29 日视频 |

两者共享 FAST-LIO2 和控制链，关闭/开启的是工程模块组合；不能命名成“原版 FAST-LIO2 对比论文 SOTA”。

### 10 月 3 日仓库自主探索

原文件：`/home/a/AstraDroneOpen/exploration_demo_runs/warehouse_20261003_222021_result.json`。

- 任务终态 `returned_home_and_saved`、`state=COMPLETE`、`home_confirmed=true`，返航 XY 误差 0.12256m。
- 原始完成原因 `wall_budget_partial`；`coverage_complete=false` 且 `exploration_complete=false`。返回并保存不等于完整探索。
- 97 个目标中到达 93 个，`stalled_goals=3` 不是确切的重规划次数。
- 航程 547.88235m，仿真时长 2174.941s，墙钟时长 3833.265s；不得混用两种秒。
- 已知自由区域 1187m²；在线已观测表面的多视点质量比例 14684/15849=92.649%，不是全仓几何覆盖率。
- 最近窗口 Explorer tick P95 184.358ms，不是全系统延迟或机载算力结果。
- 终态 `map_points=284442` 与最后实际 PCD 的 284418 点是不同采样时刻，不应强行认为完全相等。

### 同轮 FreeDOM 地图对照

原文件：

- `/home/a/AstraDroneOpen/exploration_demo_runs/warehouse_20261003_222021_coverage.json`
- `/home/a/AstraDroneOpen/exploration_demo_runs/warehouse_20261003_222021_freedom_coverage.json`

| 货架侧面代理审计 | ours 维护图 | FreeDOM 旁路图 |
|---|---:|---:|
| 支持参考点 / 总参考点 | 9267 / 10192 | 10091 / 10192 |
| 侧面支持率 | 90.924% | 99.009% |
| 最低单架支持率 | 29.592% | 85.714% |
| 最终图点数 | 284418 | 756031 |
| 联合侧面验收 | false | false |

审计范围为碰撞盒暴露的货架侧面、指定高度带、参考采样步长 0.30m、近邻容差 0.25m；不是完整 visual mesh 或全三维地图。验收要求总支持至少 95%、每架至少 90%。点密度也不同，不能据此排名动态分割 IoU、静态误删或所有任务效果。FreeDOM 是同输入并行地图模块，没有接管该航次探索和控制。

## LMNet 风格离线材料的证据边界

重新读取 `/home/a/AstraDroneOpen/analysis_outputs/lmnet_mid360_crossing/metadata.json`；对应目录现有 `bev_frames.png`、`bev_residuals.png`、`range_residuals.png`。

- 原始包：`/home/a/AstraDroneOpen/dynamic_demo_runs/badcase_20260923_194722.bag`，不是上述成功对照的两个包。
- 四帧各 20000 点；展示投影 64×360，范围 30m，垂直投影范围 [-45°,45°]。
- 使用位姿将上一帧对齐到当前，再计算归一化残差 `abs(current_range-warped_previous_range)/current_range`。
- 这是离线原理图，不是 LMNet 网络推理、人体语义分割或动态点真值；非重复扫描、遮挡和位姿误差也会产生残差。
- 鸟瞰图是辅助显示，不能作为“LMNet 原网络直接输入 BEV”的依据。原网络方法由本轮独立研究文件中的原论文/作者来源确认。

## 素材入口及来源追溯

旧详细说明：`/home/a/AstraDroneOpen/docs/组会汇报/2026-10-07/01_四重点详细素材与实验依据.md`。

媒体出处：`/home/a/AstraDroneOpen/docs/组会汇报/2026-10-07/03_视频图表与证据使用说明.md`。

实际素材目录：`/home/a/AstraDroneOpen/docs/组会汇报/2026-10-07_完整证据素材包/`，不是仅有清单。必要素材按后续已确认的资源策略取用，不重复复制整套大视频/bag。

当前主线的在线语义模型、LMNet、SalsaNext 和回环不应画成已经启用的模块。未来人体语义去除只作用于长期背景地图时，当前人体仍应保留为实时避障障碍物；不能简单从全部地图和扫描中抹掉人体。

本轮仅做汇报来源核对，没有修改算法、控制代码、launch 或 world，没有编译 ROS 或运行新仿真。
