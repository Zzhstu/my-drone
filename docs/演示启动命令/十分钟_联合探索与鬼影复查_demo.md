# 十分钟联合探索与鬼影复查 demo

本轮新增独立实验入口，旧十分钟入口默认仍为原策略。新入口联合选择前沿、当前疑点和已清理历史区域；复查后可以继续探索。实际路线始终使用原始已观测安全图，假设清除的连接价值只参与评分。

在项目根目录运行：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/half_warehouse_joint_channel_ten_minute_demo.sh
```

无界面、保留离线评价证据：

```bash
bash scripts/run_sh/half_warehouse_joint_channel_ten_minute_demo.sh --headless --record-evidence
```

同退出目标的对照入口：

```bash
# B1：原策略，只放宽多视角退出目标到70%
bash scripts/run_sh/half_warehouse_online_loop_ten_minute_demo.sh --headless --record-evidence --quality-target 0.70

# B2：简单联合策略
bash scripts/run_sh/half_warehouse_online_loop_ten_minute_demo.sh --headless --record-evidence --mission-policy joint-simple

# B0：原策略、原85%退出目标
bash scripts/run_sh/half_warehouse_online_loop_ten_minute_demo.sh --headless --record-evidence
```

新策略结果在 `half_warehouse_joint_channel_runs/`，每航次独立前缀。`_result.json`记录返航/降落/保存，`_final_map.pcd`为在线维护图快照，`_final_map_ghost_history.json`保留嫌疑/清除历史，`_exploration.joint_events.jsonl`记录选择、取证质量和实际新证据。带`--record-evidence`还会保存录包，供独立定位/覆盖/地图审计。

真实ROS配置为 `AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/config/half_warehouse_joint_channel.yaml`。研究目录的`parameters_draft.yaml`是原设计规格，不能直接加载。70%是已观测表面多视角退出目标，不是全环境完整率；预算不足时仍会部分完成返航，不能以已保存地图判定覆盖目标达成。

保持前端20cm、维护体素15cm、规划20cm、速度上限0.60m/s、探索机心安全距离0.80m（另计网格量化保护）、固定相对高度1.20m以及600秒总预算/60秒储备。新策略不放宽占据清除证据，也不把历史疑点当作当前可通行空间。50ms评分预算采用协作式截止检查，不能保证抢占式硬实时。

首轮实施、负结果及后续修正见 `evaluation/joint_channel_20261024/`。本轮为SITL实验原型，完整动态真值误删/残留评价和多场景重复尚未完成，旧策略保留默认。无需同时运行另一套飞行控制器。

本轮最终结果：完整联合v2自动返航落地存图，434.004秒保守解锁预算、608.00㎡自由投影、定位RMSE0.949cm；旧70%策略313秒、586.12㎡。覆盖略增但未证实效率提升，多视角验收仍false。完整报告/对比图：`evaluation/joint_channel_20261024/IMPLEMENTATION_REPORT.md`。
