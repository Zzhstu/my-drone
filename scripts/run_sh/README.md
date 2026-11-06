# run_sh 目录说明

本目录包含若干通过 `tmux` 启动的脚本，方便一次性拉起演示、示例或录制所需的 ROS 节点。每个脚本都会先结束同名的 tmux 会话，然后按预设的分屏布局启动相关命令。概览如下：

## tmux 简介

`tmux` 是一款终端复用器，可在一个终端中创建多个会话、窗口和窗格，支持分屏、后台运行和会话恢复，适合管理多进程或长时间运行的任务。

## tmux 快速使用

### 创建与附着会话
- 新建会话：`tmux new -s <session_name>`
- 附着到已有会话：`tmux attach -t <session_name>` 或 `tmux a -t <session_name>`
- 列出会话：`tmux ls`
- 退出并关闭会话：在 tmux 中输入 `exit`，或在外部运行 `tmux kill-session -t <session_name>`

### 窗口与窗格操作（默认前缀为 `Ctrl+b`）
- 水平分屏：`Ctrl+b` 然后 `%`
- 垂直分屏：`Ctrl+b` 然后 `"`
- 切换窗格：`Ctrl+b` + 方向键
- 调整窗格大小：`Ctrl+b` 按住后再用方向键调整
- 创建新窗口：`Ctrl+b c`
- 切换窗口：`Ctrl+b n`（下一个）、`Ctrl+b p`（上一个）、`Ctrl+b <数字>`（跳转指定窗口）

### 会话管理
- 断开但保持运行：`Ctrl+b d`（detach），进程仍在后台运行。
- 重命名会话：`tmux rename-session -t <old> <new>`
- 复制模式：`Ctrl+b [` 进入滚屏/复制，`q` 退出。

## 运行脚本的小贴士

- 脚本默认会终止同名 tmux 会话并重新创建，运行前请确认对应会话中的任务可以安全结束。
- 如果需要在脚本基础上追加命令，可在脚本中相应窗格的 `tmux send-keys` 之后添加新的指令。
- 若想在附着时直接进入脚本创建的会话，可运行：`bash <script_name>.sh`，脚本末尾会自动执行 `tmux attach`。


## 右侧中部新增行人180秒实验

运行 `bash scripts/run_sh/half_warehouse_two_pedestrians_three_minute_demo.sh`。沿用当前补扫/入口识别版，保留原行人0，另在俯视图右侧中部world(3,-12)加入行人1，在world(1,-12)到(5,-12)之间往返。无人机高度首次超过0.9m后移动180秒仿真时间，随后停止。默认Gazebo/RViz开启，600/60秒预算保持；原单人入口不变。

2026-10-29起，这个双人入口显式传入 `--ghost-score-scale 1.3`，将活动鬼影verification视点的总效用提高30%；近家鬼影仍为0分，已清除历史回访、前沿、补面与安全限制保持。用 `bash scripts/run_sh/half_warehouse_two_pedestrians_three_minute_demo.sh --ghost-score-scale 1.0` 可恢复原倍率对照。通用运行器默认倍率仍为1.0。本次试验边界见[Dynablox关系与加分试验](../../evaluation/ghost_score_trial_20261029/DYNABLOX_COMPARISON.md)。

首次实验151542与负结果见 `evaluation/right_pedestrian_20261029/REPORT.md`：主体层历史残留代理清到0，但近地层仍有14个远参考点、人机最近中心距约0.645m，不能作为零鬼影/充分动态避让余量的保证。

## DUFOMap独立清理地图demo

安装可选依赖：`bash scripts/run_sh/install_dufomap_demo.sh`。启动（Gazebo和RViz）：`bash scripts/run_sh/half_warehouse_dufomap_demo.sh`。采用作者dufomap==1.1.1原生绑定，体素0.15m、测距膨胀0.20m、定位膨胀1格、2线程；已知激光外参旋转后加入机体位姿。原双人180秒、1.3复查倍率及600/60秒飞行预算保持。规划/避障/SLAM仍沿用当前项目，DUFOMap是独立地图支路，不给规划器提供清空后的障碍图。

输出在 `half_warehouse_dufomap_runs`，静态图以 `_dufomap_static_map.pcd` 结尾，同时保存被判动态剔除点、未过滤观测代表和参数/输入SHA。RViz默认绿色DUFOMap静态图，可切换橙色剔除点或原维护图。该支路使用连续 `map` 坐标，暂不施加回环图优化。详细结果与同输入回放见 [DUFOMap评测目录](../../evaluation/dufomap_20261029/)。原有入口默认不启用本支路。

## 原维护图 + DUFO背景图的同步双图demo

50平方米另一室内场景：`bash scripts/run_sh/office50_dual_map_demo.sh`。这是独立的7.2×7.0m封闭办公室，10件原尺寸GitHub家具网格与2个新箱体书柜，一名行人起飞后按仿真时间运动60秒再停止。家具来源为已固定提交的 `macc-n/gazebo_worlds` 小办公室；房间和物体位置重新布置，不声称上游原世界就是50m²。单独输出 `office50_dual_map_runs`，保留现有规划/双图/0.6m/s/15cm建图/20cm规划/600秒与60秒储备/近家5m鬼影0分参数；只改变场景和演员运动。默认Gazebo与RViz开启，compact录包，不清理仓库的旧结果。预检/实测见 `evaluation/office50_20261030`。人物使用同一站立网格平移/转向，没有骨骼步态动画。

最终184237实际GUI航次通过自主返航/慢降/双图导出：96sim秒，行人60sim秒，886同帧/19检查点；详见 [50㎡实验报告](../../evaluation/office50_20261030/REPORT.md)。新场景曾暴露同帧重复DUFO原生分类有少量边界差异，包装层已冻结每输入帧的一次结果供发布/保存共用，未改原生参数或放宽SHA校验。两轮负例保留。原5m近家零分覆盖小室内大部，本轮主动复查0秒、原图仍54疑似，且最小人机水平中心距.742m；跑通不代表全3D/零鬼影或充分动态安全。

启动新仿真（Gazebo/RViz）：`bash scripts/run_sh/half_warehouse_dual_map_demo.sh`。保留原双行人、180秒移动、1.3复查倍率、600/60秒预算及规划/避障参数；增加独立双图协调器，每50帧核对共同帧数、纳秒截止、坐标与图版本。原维护图白色，DUFO背景绿色；可选橙/紫表示15cm容差下几何分歧，**不是鬼影/自由标签**。两张图独立维护、不作交集或并集，DUFO不替换安全障碍图。

结果在 `half_warehouse_dual_map_runs`：`_final_map.pcd` 为原维护图，`_dufomap_static_map.pcd` 为DUFO背景，`_dual_map.json` 为共同截止/输入指纹/文件SHA清单。`_dual_map_pairs.jsonl` 是少量配对检查点，不录制整幅嵌套双图消息；录包沿用compact profile。本版本DUFO尚不支持回环历史重建：图修正版本大于0后拒绝联合配对/导出，仍保留各自独立结果与原飞行链路，不将改frame_id当作回环完成。

同数据验证（私有ROS master，无新飞行，不重算FAST-LIO前端）：

```bash
bash scripts/run_sh/dual_map_replay.sh \
  half_warehouse_dufomap_runs/warehouse_half_dufomap_20261030_114104 \
  --output-prefix half_warehouse_dual_map_runs/replay_$(date +%Y%m%d_%H%M%S) \
  --speed 2 --keep-open
```

默认显示RViz，结束后保留最终双图，Ctrl-C只关闭本次私有进程；使用 `--headless` 关闭显示，省略 `--keep-open` 则验证完自动退出。详细范围/实测结果见 [双图验证报告](../../evaluation/dual_map_20261030/REPORT.md)。新仿真入口已接入，固定轨迹重放不证明新航次路线、功耗或非零回环效果。
