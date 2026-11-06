# RoomGS / Warehouse20 demo

状态：2026-11-01按用户要求删除上一轮全部RoomGS实验产物，包括最新航次的录包、图像、点云、模型、渲染、评测和相关下载缓存；源码、已编译节点、已安装环境保留。此前结果不能再从本地打开，历史摘要保留在交流记录。

当前新场景为[cangku 20㎡单货架仓库](WAREHOUSE20_CN.md)。本次已完成多航次真实数据严格验收、12000步实际CUDA训练、学习模型后审计及交互检查，结果为`roomgs_runs/warehouse20_20261102_reconstruction`。地图和模型同一4407目标缺口0，数据包约2.38GiB，细节仍有模糊。新仓库巡检路线/三角面验收已适配，低流量和容量预算、完成结果及复跑方式见[资源说明](ROOMGS_RESOURCES_CN.md)。当前入口默认warehouse20，原11×12m房间须显式`--scene room`。本页旧房间11418目标计数不用于新场景；新仓库预声明4407目标，实际通过状态以新报告为准。

## 范围与完整性定义

输入为现有仿真无人机的MID360点云、IMU和一幅RGB相机图像，不使用相机深度图。默认场景为净5×4×3m、仅一个原货架网格的仓库。它是已知模型离线规划的巡检航线，不是未知房间自主探索演示。

新仓库采集前固定4407个外部可观测表面目标，20cm表面单元/法向/部件各取实际网格代表点，独立安全相机能力域确定资格，不随已采数据改变。另1430个遮挡、贴地或域内不可观测目标保留在参考文件掩码中。每个必需目标同时满足：

1. 实际R3LIVE原生RGB地图存在20cm范围内的支持点。
2. 至少两幅实际采集关键帧可见，且相机位置有至少40cm平移间隔。
3. 图像视野、表面朝向和遮挡射线审计通过。

所有目标都满足才通过，required_fraction=1.0，missing_samples必须为0。不以99%等近似指标替代。采样网格和几何容差已在首次完整飞行前固定，失败不降低标准。它证明指定外露表面在该离散分辨率下的覆盖，不证明遮挡内部、未定义表面或无限连续细节。

已知模型用于离线目标/遮挡定义、补拍站点选择和完整性审计；飞行前单次冻结刚体坐标对齐仅供离线审计。R3LIVE、RGB位姿导出和训练损失均不读取网格或Gazebo世界位姿。飞行只读数字航点，局部防撞使用传感器标定变换后的原始雷达机体系扫描与相对指令，不读取网格。

## 构建和运行

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/build_roomgs_demo.sh
bash scripts/run_sh/roomgs_demo.sh
```

构建分别使用r3live_roomgs_ws与.envs/roomgs，不修改旧r3live_ws、原场景或系统CUDA。固定R3LIVE作者提交6143a385及本项目补丁；固定gsplat v1.5.3提交937e29912570c372bed6747a5c9bf85fed877bae。PyTorch2.7.1/cu128与CUDA12.8工具链用于RTX50系；本机旧Python3.8的Torch2.4/cu124实际CUDA操作报no kernel image，不能当作本后端可用环境。

默认顺序：真实PX4/Gazebo/R3LIVE采集 → 完成检查航点并落地上锁 → 原生地图保存 → 全部样本严格审计 → 离线gsplat训练 → 模型保存/留出视角/六向及新视角漫游 → 独立训练后几何审计。缺口或采集错误会返回非零并禁止训练。

无界面采集与训练：

```bash
bash scripts/run_sh/roomgs_demo.sh --headless
```

只采集并验收（仍须100%通过）：

```bash
bash scripts/run_sh/roomgs_demo.sh --headless --capture-only
```

对已经通过的航次训练：

```bash
bash scripts/run_sh/roomgs_demo.sh --train-only /绝对路径/roomgs_runs/航次目录 --steps 12000
```

已有模型不覆盖。不能用train-only绕过完整性或源文件/数据指纹检查。

训练完成后，打开交互高斯浏览器：

```bash
.envs/roomgs/bin/python integration/roomgs/viewer.py roomgs_runs/航次目录
# 浏览器访问 http://127.0.0.1:8889
```

浏览器使用学习后协方差、透明度与SH0颜色；保存的六向图片和漫游使用完整SH外观。旧大房间模型已删除；当前新20㎡模型已经训练，服务保留在本机8889端口。

## 输出

每次使用新的roomgs_runs/warehouse20_YYYYMMDD_HHMMSS目录：

- rgb_pt.pcd、colored_map.png：R3LIVE原生RGB地图与预览。
- sensors.bag：原始RGB/标定、点云、IMU、估计器与飞控输出，未录场景真值作为训练输入。
- dataset/images、frames.jsonl、frames.json：实际相机PNG与逐图原始测量时间、完整光学c2w位姿、图像SHA256。
- dataset/export_status.json：精确时间配对统计，过期未匹配数。
- audit_scene.json、audit_alignment.json：仅审计参考和冻结的初始对齐，不是估计器输入。
- coverage.json、coverage_samples.npz：每面、每样本的实际地图与多视角支持；失败另写missing_surfaces.json。
- result.json：采集/落地/原生导出/覆盖门槛与源代码冻结检查。
- acquisition_sources.json、acquisition_sources/：按内容哈希保存采集版本源码，后续离线工具更新不冒充采集版本；读取时复核快照。
- mesh_reference.npz、target_manifest.json：采集前固定的真实三角面目标、能力域及排除掩码/指纹；只用于离线规划和审计。
- gsplat/model.pt、model.ply、gaussians.npz：实际学习的三维位置、旋转/尺度、透明度和SH外观参数，不是把PCD改扩展名。
- gsplat/training.jsonl、provenance.json：优化过程、软件与输入指纹。
- gsplat/heldout_metrics.json、heldout_comparison.png：留出RGB视角的渲染对照。
- gsplat/initial_heldout_metrics.json：固定分散留出视点的优化前指标；与同一组视点的优化后指标比较，不混用不同视点集合。
- gsplat/room_six_views.png：在采集轨迹估计出的室内中心分别看四周、顶面和地面。
- gsplat/walkthrough.mp4：在相邻采集位姿之间插值的新视角渲染，不是输入图片录像。
- gsplat/model_coverage.json：训练完成后独立检查高斯均值对同一表面采样网格的支持，真值不进入训练。

留出帧不参与颜色初始化或训练RGB损失；前端轨迹和几何来自整段采集，因此不是完全独立外部真值定位实验。颜色初始化取训练图像投影，几何/深度先验取实际雷达地图，不读取仿真表面。

训练后独立审计检查透明度至少0.01的高斯均值对该航次预定义表面采样的支持；这不等于无限细节或逐像素完美。旧图像和指标产物已删除，不能作为当前新场景的验收结果。

## 原始来源

- R3LIVE：https://github.com/hku-mars/r3live
- gsplat：https://github.com/nerfstudio-project/gsplat
- gsplat DefaultStrategy遵循原3DGS的增密、分裂和剪枝；本工程增加ROS数据适配、LiDAR先验及验收/产物编排。没有集成FUEL、Global-LVBA或GS-SDF，不将调研候选说成已用代码。

本demo不是实物飞行入口，也不新增动态清理、回环、TSDF或mesh重建能力。旧evaluation/roomgs_20261101目录已删除；清理清单见docs/维护记录/2026-11-01_20平米单货架仓库/cleanup_manifest.json。
