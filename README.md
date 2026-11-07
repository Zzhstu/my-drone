# My Drone：动态环境无人机建图与探索

本项目基于 AstraDroneOpen，面向室内仓库等场景，集成 PX4/Gazebo 仿真、MID360 激光惯性定位、动态环境地图维护、自主探索和离线三维高斯建模（3DGS）。主要用于研究和工程验证：无人机在环境中定位、观察动态物体、维护供规划使用的地图，并保存可复查的实验结果。

当前公开仓库是从本机 `/home/a/AstraDroneOpen` 筛选的**源码快照**，不是整个工作目录的备份。实验数据、录包、已训练模型、部分权重、ROS 构建产物和本机环境不在仓库中；本机项目后续修改也不会自动同步到 GitHub。上传范围和可能缺失的资源见 [上传范围说明](UPLOAD_SCOPE_CN.md)。本仓库的启动链主要在已有 ROS 1/PX4/Gazebo SITL 环境中验证，**新机器直接克隆后的完整构建及真机飞行尚未验证**。

## 系统流程

```text
MID360 点云 + IMU ──> FAST-LIO 定位/点云 ──> 地图维护 ──> 探索、规划、避障及返航
                              │                 └──> DUFOMap 同输入双图对照
单目图像 + 相机参数/位姿 ──────┴──> 数据整理与可选动态 Mask ──> gsplat 离线训练/查看
```

## 已实现的功能与当前边界

- **无人机仿真与基本控制**：通过 PX4 SITL、Gazebo 和 ROS 1/MAVROS 组织无人机、雷达、场景和控制；可使用键盘或目标点开展演示。
- **激光惯性定位与点云建图**：接入 FAST-LIO/MID360 仿真数据，发布定位、轨迹和点云，并提供长走廊等测试场景。定位准确度受场景、标定和时间同步影响。
- **动态环境地图与路径规划**：集成动态目标检测/跟踪、历史占据维护、局部自由空间证据和路径重规划；支持目标导航及自主探索。动态残影、静态误删和安全余量仍需按场景验证。
- **自主探索与返航**：提供探索候选点、表面补扫、历史疑点复查、任务预算、返航与落地等演示入口。一次任务结束或地图保存成功不代表完整三维覆盖。
- **同航次双地图**：并行维护项目地图与原生 DUFOMap 静态背景图，支持按共同输入和截止时间保存、检查和展示。一次半仓库 SITL 航次完成返航及 4332 帧双图保存，但表面质量为 56.82%，未达到该次 70% 门槛；非零回环后的共同重建及重放一致性仍待验证。
- **回环探索**：提供关键帧、回环候选和地图版本相关的实验入口。已有负面案例，尚不能声称普遍改善定位或地图精度。
- **LiDAR 与单目辅助的离线 3DGS**：支持采集图像、位姿与点云，利用 gsplat 进行离线训练、导出和查看，并提供动态图像 Mask 接口。已有 G1 仿真基线完成训练，但固定目标严格完整性未通过；图像 Mask 不保证完全消除静止人体。
- **场景与传感器配置**：提供半仓库、小仓库、办公室、行人等仿真场景，以及 G1、MID360 参数适配入口。G1 仿真使用理想针孔、3 Hz 等近似，不能视为与真实设备完全等效。

FAST-LIO、DUFOMap、R3LIVE、gsplat 等方法及其第三方代码属于各自作者；本项目的工作主要是工程集成、配置适配及实验验证，不将已有算法表述为本项目原创。

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `AstraDrone_ros1_ws/src/` | ROS 1 定位、地图维护、规划、控制等功能包 |
| `simulation/` | Gazebo 世界、模型、PX4 启动文件和仿真工作空间 |
| `scripts/run_sh/` | 仿真、采集、训练、查看和重放脚本 |
| `integration/` | LiDAR/RGB 数据整理、半仓库采集及 gsplat 训练适配 |
| `third_party/` | 随快照提供的第三方源码；各组件许可证需分别核对 |
| `docs/` | 配置、实验方法和使用文档；部分历史文档链接指向未上传的实验目录 |

## 环境与使用前检查

以下命令以**本机已有环境**为前提：Ubuntu、ROS Noetic、Gazebo Classic、PX4 SITL、MAVROS、已编译的 ROS 工作空间及相应雷达/行人插件。双图演示还需要 DUFOMap 及其依赖；3DGS 需要匹配的 NVIDIA CUDA、PyTorch、gsplat 和 Python 环境。

仓库没有包含 `build/`、`devel/`、`.envs/`、实验数据及权重，克隆后需自行配置并编译依赖；具体版本、插件及外部资源还没有在干净机器上做完整安装验收。部分脚本仍引用 `/home/a/pedsim_ws` 等本机绝对路径，应先检查并按实际路径调整。部分 catkin `CMakeLists.txt` 是指向本机 ROS 安装的符号链接。运行图形化演示需要桌面环境，建议同一时间只启动一个完整飞行任务。脚本可能产生较大的新录包和实验输出。

```bash
# 在当前实际工作目录执行；若克隆到别处，请改为克隆目录
cd /home/a/AstraDroneOpen
source /opt/ros/noetic/setup.bash
```

## 常用启动命令

以下入口已对照项目中的脚本及现有演示文档检查；本次编写文档没有重新运行它们。每个航次是否成功需查看其实际日志与结果。

### 1. 只查看半仓库场景

```bash
GAZEBO_MODEL_PATH="$PWD/simulation/astra_gazebo_models:${GAZEBO_MODEL_PATH:-}" \
  gazebo --verbose "$PWD/simulation/astra_gazebo_worlds/cangku_half.world"
```

只打开场景，不启动无人机飞行、建图或训练。20 平方米单货架场景可运行 `bash scripts/run_sh/warehouse20_preview.sh`。

### 2. 基础飞行与 FAST-LIO 演示

```bash
bash scripts/run_sh/pc_example.sh
```

该脚本打开多个终端并启动基础仿真链。需要已编译的 PX4 和项目工作空间，并检查脚本内的 PedSim 路径。

### 3. 半仓库自主探索

```bash
bash scripts/run_sh/half_warehouse_exploration_demo.sh --keep-open
```

用于观察探索、建图、返航和落地流程；保留界面的具体条件以脚本实际行为为准。

### 4. 原维护图与 DUFOMap 双图

```bash
bash scripts/run_sh/half_warehouse_dual_map_demo.sh --keep-open --keep-evidence-bags 0
```

以相同航次输入并行维护两张图，并在满足检查条件后保存。`--keep-evidence-bags 0` 用于保留已有证据录包；本入口默认使用有限时任务设置。需要先安装并配置 DUFOMap，最终还应检查双图报告与覆盖质量，而非只看窗口。

### 5. G1 近似参数半仓库：采集与离线 3DGS

```bash
bash scripts/run_sh/static_half_warehouse_g1_gsplat_demo.sh --gui "g1_$(date +%Y%m%d_%H%M%S)"
```

此入口会实际采集、训练并尝试打开查看器，耗时和磁盘占用较大。需要 CUDA/gsplat 环境，并先按 [三高度采集说明](docs/STATIC_HALF_WAREHOUSE_HEIGHT_CN.md) 生成或准备 `half_gs_plans/warehouse_half_static_g1_20261104/inspection.json` 等航线文件；这些航线和旧模型没有随本仓库上传。不要覆盖既有数据集。

### 6. 查看已有 G1 模型（仅本地已具备模型时）

```bash
bash scripts/run_sh/open_static_half_warehouse_gsplat_desktop.sh \
  gsplat_runs/warehouse_half_static_g1_20261104
```

仓库未包含这个模型；只克隆仓库无法直接查看，需要使用本机保留的模型及对应环境。该命令不启动飞行或重新训练。

## 更多文档

- [图形化 Demo 与启动命令](docs/演示启动命令/2026-11-05_图形化Demo总览.md)：包含历史配置与实验状态，部分实验报告和数据未随仓库上传；当前具体参数以脚本为准。
- [G1 与 MID360 仿真适配](docs/G1_MID360_HALF_SIM_CN.md)
- [FAST-LIO 与 gsplat 数据和训练说明](docs/LIO_GSPLAT_CN.md)
- [半仓库 3DGS 采集与建模](docs/HALF_WAREHOUSE_GSPLAT_CN.md)
- [历史 README 存档](docs/README_历史版本.md)：保留原文供查阅，其中安装指引和能力描述可能已经过时。

## 后续工作

继续核对实际相机和雷达的标定、时空同步；改善弱覆盖区域与局部画质；在相同输入和预算下评估动态残影、误删与规划安全性；排查无限时长双图容量等待及 DUFO 重放差异。非零回环双图一致性、干净环境复现、真实设备续航及实机飞行均需要进一步验证。
