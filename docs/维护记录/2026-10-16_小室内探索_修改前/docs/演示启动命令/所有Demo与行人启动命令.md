# 所有Demo与行人启动命令

核对日期：2026-10-14。范围是本项目已接入的PX4/Gazebo完整演示入口，以及同一演示的配置变体；单独节点launch、合成测试和保存/分析工具不是另一套完整飞行demo。

每个终端都先进入项目目录：

```bash
cd /home/a/AstraDroneOpen
```

同一时间运行一个完整demo。主启动脚本已经负责ROS、Gazebo、PX4、FAST-LIO与对应控制链，不需要另开第二个飞控控制器。

新增的 `docs/演示启动命令/行人键盘控制.sh` 只包装原有键盘客户端及ROS/PedSim环境，不生成行人、不替换算法。它补充了当前多工作区source顺序会丢失的PedSim消息路径。

## 1. 基础键盘飞行/建图

主脚本自动启动PedSim行人运动；没有本项目新增的行人键盘relay，不使用下面的行人键盘脚本。无人机在Offboard Control终端操作：T设起飞高度，W/S沿X，A/D沿Y，R/F调高度，L降落。无人机按键与行人按键的轴定义不同。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/pc_example.sh
```

## 2. 旧多终端动态避障

无人机自动起飞、使用自动目标。行人等待navigation_ready后开始自动横穿；横穿结束且/demo/pedestrian/ready为true后，键盘可接管0/1号。需要在RViz手动指定目标时，主命令改为AUTO_GOAL=false bash scripts/run_sh/pc_dynamic_avoidance.sh。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/pc_dynamic_avoidance.sh
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 3. example badcase内部对照

包含自动行人横穿，随后可键盘接管0/1号。关闭导航地图清理及局部恢复组合，但仍保留动态风险安全保护。默认录包/报告结束后继续展示；主终端Ctrl-C收尾。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/dynamic_comparison_demo.sh badcase
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 4. example ours内部对照

包含自动行人横穿，随后可键盘接管0/1号。使用可维护导航地图与局部路径修正。默认录包/报告结束后继续展示；主终端Ctrl-C收尾。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/dynamic_comparison_demo.sh ours
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 5. 仓库指定目标导航

自动起飞并进行指定目标导航；只存在0号行人。主脚本已启动仓库行人控制节点，但默认没有自动横穿；ready为true后可以键盘移动。RViz可用2D Nav Goal更改导航目标。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_dynamic_demo.sh
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 6. 仓库黑箱自主探索：默认自动行人

默认会等待无人机接近后让0号行人行走一轮。必须等/demo/pedestrian/walk_status为finished_keyboard_ready再用键盘；ready=true本身不足以证明自动脚本已经释放速度话题。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_exploration_demo.sh
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 7. 仓库黑箱自主探索：从开场手动行人

关闭默认的一轮自动行走。主脚本仍创建0号行人及控制节点；ready=true后可以直接使用第二终端的键盘。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_exploration_demo.sh --manual-pedestrian
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 8. 探索+FreeDOM旁路地图

仍是现有探索/控制链；FreeDOM并行建图，不替换导航地图和规划器。这里加manual-pedestrian便于开场键盘控制0号。如果去掉manual-pedestrian，则与默认探索一样，等自动行走结束再接管。FreeDOM节点需已经编译。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse_exploration_demo.sh --freedom-baseline --manual-pedestrian
```

另开真实终端，按上述时机接管行人：

```bash
cd /home/a/AstraDroneOpen
bash docs/演示启动命令/行人键盘控制.sh
```

## 9. 长走廊FAST-LIO基线

自动执行长走廊航线与基线记录；脚本不启动行人，也不提供对应行人键盘接口。结果由基线记录器保存到~/.ros/fastlio_baselines/。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/fastlio_long_corridor_baseline.sh
```

## 10. 回环专项演示

启动专用场景、FAST-LIO、回环后端、自动闭合航线和录包/分析；不启动行人。输出loop_closure_runs/。航线/录包结束不等于所有分窗口节点都关闭。历史误回环/精度负结果仍需如实保留。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/loop_closure_demo.sh
```

## 11. 历史tmux备份入口

这是旧备份，使用astra、qgc等外部别名及旧ROS环境假设，并且先结束同名pc_example tmux会话。当前演示优先使用第1项pc_example.sh；本轮仅检查语法，不承诺备份在当前环境可完整运行。

主终端：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/pc_example_tmux_backup.sh
```

## 行人键盘操作

| 按键 | 作用 |
|---|---|
| 0 / 1 | 选择行人；仓库只有0号 |
| W / S | 沿Gazebo world的+Y / -Y移动 |
| A / D | 沿Gazebo world的-X / +X移动 |
| 空格 | 停止 |
| Q / Ctrl-C | 退出键盘客户端并发送停止命令 |

键盘速度0.6m/s；持续按键产生重复命令维持移动，约0.4秒没有新命令后停车。退出键盘不结束主demo。仓库控制还有货架/墙体阻挡检查，不能用键盘让行人穿透货架。

不用新增包装脚本时，完整原始命令为：

```bash
cd /home/a/AstraDroneOpen
source /opt/ros/noetic/setup.bash
source /home/a/pedsim_ws/devel/setup.bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
export PYTHONPATH="/home/a/pedsim_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="/home/a/AstraDroneOpen/AstraDrone_ros1_ws/src:/home/a/pedsim_ws/src:${ROS_PACKAGE_PATH:-}"
rosrun fastlio_bridge pedestrian_control.py --keyboard
```

## 附加的仓库自动行走命令

需要手动开启项目已有的一轮自动行走时，先选第7项manual-pedestrian主命令，再另开终端执行：

```bash
cd /home/a/AstraDroneOpen
source /opt/ros/noetic/setup.bash
source /home/a/AstraDroneOpen/AstraDrone_ros1_ws/devel/setup.bash
rosrun fastlio_bridge exploration_pedestrian_walk.py
```

脚本等待无人机与0号行人的水平距离小于6米、无人机world高度大于0.9米，并连续满足8个仿真秒，随后控制行人沿左侧通道走一轮。结束后walk_status=finished_keyboard_ready；脚本继续保留状态发布，但不再发送行走速度。默认探索已经启动它，不要重复启动；运行期间不要同时使用键盘或另一份速度发布者。

需要直接发送固定速度时，先确保键盘和自动行走均已释放控制；在上述source后的终端运行：

```bash
rostopic pub -r 20 /demo/pedestrian/0/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.6, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

该命令持续沿world +Y移动；Ctrl-C后由0.4秒无命令保护停车。显式停止可发送：

```bash
rostopic pub -1 /demo/pedestrian/0/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 常用变体

仓库探索只关Gazebo窗口、保留RViz：

```bash
bash scripts/run_sh/warehouse_exploration_demo.sh --no-gazebo-gui
```

仓库探索两种GUI都关闭，但物理仿真仍运行：

```bash
bash scripts/run_sh/warehouse_exploration_demo.sh --headless
```

探索结束后保持窗口：

```bash
bash scripts/run_sh/warehouse_exploration_demo.sh --keep-open
```

两个观察目标后尝试返航，适合短演示，不等于完整仓库覆盖：

```bash
bash scripts/run_sh/warehouse_exploration_demo.sh --return-after-goals 2 --manual-pedestrian
```

仓库指定目标导航关闭两种GUI：

```bash
bash scripts/run_sh/warehouse_dynamic_demo.sh --headless
```

example对照关闭GUI：

```bash
bash scripts/run_sh/dynamic_comparison_demo.sh ours --headless
```

comparison/仓库导航的--auto-stop表示报告完成后自动结束；默认持续展示。--max-flight-seconds只限制录包/观察阶段，不代表整个demo停止。探索--test-seconds为测试强制时限，可能产生部分地图，不能当完整成功；--max-wall-seconds默认为4800秒的任务墙钟预算。--full-bag会增大录包体积，本表默认不加。

上述参数属于各自Python运行器；不要把--headless等参数套用到pc_example、pc_dynamic_avoidance、长走廊或回环多终端shell入口。

## 可选节点启动与完整demo的区别

`roslaunch fastlio_bridge astra_pedestrians.launch`只启动example的PedSim轨迹和relay，依赖已有世界/行人模型。当前第2/3/4项已自动启动它，不要重复启动。仓库依赖生成几何文件的warehouse_pedestrian.py，完整主入口已替用户传入，不应另启动example行人launch。

synthetic_test、loop_closure_synthetic_test等是组件合成检查；perception、avoidance、complete_system等launch是组合组件入口，不能替代上述完整场景/PX4编排脚本。

## 本轮核验范围

8个原有shell入口与新增行人包装脚本均进行bash -n检查；两个Python运行器的帮助/参数已核对；ROS/PedSim环境、消息导入及包定位通过只读检查。未启动任何Gazebo/PX4仿真、无人机或行人，也未重编译。

当前直接给shell包装入口传--help会被source的catkin环境脚本截获并导致usage作为shell文本执行；本轮改用完成source后直接运行Python运行器--help核验，不修改原启动脚本。
