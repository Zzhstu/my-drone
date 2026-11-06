# 适用于当前无人机数据的 GitHub 3DGS 项目

核实日期：2026-11-02。阅读范围为官方/作者 GitHub README、相关数据加载和掩膜损失代码、作者项目页；未安装或运行以下新框架，也未进行性能对比。上游 main 会变化，检查到的文件 blob SHA 另存于 `evaluation/3dgs_github_20261102/source_checks.json`。

当前输入：`lio_dataset_runs/warehouse20_20261102_capture/`，4,758 张640×480 RGB、80,900 个静态实测点、相机内参、同地图光学c2w位姿和原始时间戳，已导出COLMAP文本及OpenGL transforms。场景无行人，无动态Mask。固定20cm几何支持全通过，但10个货架样本未满足照片双视角/40cm基线条件，不将本轮数据称为已通过严格完整外观验收。

## 项目比较

| 项目 | 可以复用的功能 | 接入本批数据的工作 | 定位 |
|---|---|---|---|
| [Gaussian Splatting Lightning](https://github.com/yzslab/gaussian-splatting-lightning) | COLMAP/PLY初始点、外部动态物体Mask、训练/评估/Web查看器、uint8及有限图像缓存 | 优先用已导出的COLMAP文本；检查依赖版本，准备Mask命名与像素约定 | 功能最贴合“静态点云＋外部Mask＋3DGS”的候选 |
| [Nerfstudio / Splatfacto](https://github.com/nerfstudio-project/nerfstudio) | 完整3DGS训练和查看流程；已知相机transforms、PLY种子点、逐图Mask | 配置`ply_file_path`并启用点加载；让点云和相机接受同一归一化；配置逐图`mask_path`和缓存方式 | 完整框架候选 |
| [gsplat](https://github.com/nerfstudio-project/gsplat) | CUDA可微高斯光栅化、COLMAP示例训练器、已知点/位姿训练 | 复用现有适配器接入新的`static_map.*`/帧格式；为COLMAP数据补逐图动态Mask读取及相应监督 | 本机现有环境复用成本最低的候选 |
| [原版 3D Gaussian Splatting](https://github.com/graphdeco-inria/gaussian-splatting) | 原论文参考实现，COLMAP输入、训练、PLY输出和查看器 | 验证COLMAP文本/二进制加载，提供实测初始点；动态区域忽略需另行核实/适配，不能把alpha背景处理直接等同于动态掩膜监督 | 静态算法基线 |
| [SpotLessSplats](https://github.com/lilygoli/SpotLessSplats) | 使用Stable Diffusion特征与鲁棒优化忽略临时干扰，含SLS-agg/SLS-mlp/RobustFilter实验 | 特征提取、数据划分/文件命名、初始点与位姿加载适配、其CUDA分支环境验证 | 后续动态干扰研究候选 |
| [WildGaussians](https://github.com/jkulhanek/wild-gaussians) | DINO特征不确定性和外观建模，处理遮挡/光照外观变化 | NerfBaselines或其本地加载流程适配，额外特征/网络和依赖验证 | 后续复杂采集条件研究候选 |

这些是功能/集成范围判断，不是本机效果排名。外部Mask支持表示训练器能使用Mask，不表示自动完成人体分割。鲁棒学习方法也不能承诺所有动态物体、阴影或从未露出的背景都被正确处理。

## 1. Gaussian Splatting Lightning 的关键核实

- [README](https://github.com/yzslab/gaussian-splatting-lightning/blob/main/README.md)明确提供外部Mask、有限缓存、uint8图像及Web查看器。
- [COLMAP加载器](https://github.com/yzslab/gaussian-splatting-lightning/blob/main/internal/dataparsers/colmap_dataparser.py)有`points_from: sfm/random/ply`、`ply_file`、`mask_dir`，并能读`points3D.txt`。
- [训练指标实现](https://github.com/yzslab/gaussian-splatting-lightning/blob/main/internal/metrics/vanilla_metrics.py)会对预测图和监督图共同应用有效像素Mask。
- Mask单通道、黑色0表示忽略；图片`001.jpg`对应Mask文件`001.jpg.png`。这是该项目自己的命名规则。
- 可以限制缓存数，而不是把4,758张图全部解码驻留。具体缓存数量须实测，README给出的例子不能当成本机内存结果。
- **本批XYZ-only PLY不能直接套用其PLY加载分支**：[PLY读取器](https://github.com/yzslab/gaussian-splatting-lightning/blob/main/internal/utils/graphics_utils.py)要求red/green/blue字段。可先使用已导出的COLMAP `points3D.txt`，其中XYZ来自真实LiDAR，颜色128是明确标注的初始化占位，不是实测RGB；也可另行生成带初始化色的PLY。
- 仓库README仍给出PyTorch2.0.1/CUDA11.8示例，部分算法使用维护者修改的gsplat。当前主机为RTX5060Ti 16GB，需在独立环境验证兼容性。未用旧依赖覆盖当前环境。
- 仓库的SpotLessSplats集成明确未包含论文中的利用率裁剪和外观建模，不能视为原论文所有组件的完整复现。

候选配置选项（尚未执行；安装兼容性和实际CLI版本确认后再使用）：

```text
--data.path /home/a/AstraDroneOpen/lio_dataset_runs/warehouse20_20261102_capture/dataset
--data.parser Colmap
--data.parser.points_from sfm
--data.image_uint8 true
--data.train_max_num_images_to_cache 64
--data.async_caching true
--model.save_ply true
```

Mask准备好后再设置`--data.parser.mask_dir MASK_DIR`。当前无行人可以省略Mask参数建立静态基线；这里的64张缓存仅是初始候选设置，没有本机内存/质量实验结论。

## 2. Nerfstudio / Splatfacto 的关键核实

- [官方数据约定](https://docs.nerf.studio/quickstart/data_conventions.html#masks)支持每帧`mask_path`，同分辨率单通道黑白Mask，黑色忽略。
- [Nerfstudio数据加载器](https://github.com/nerfstudio-project/nerfstudio/blob/main/nerfstudio/data/dataparsers/nerfstudio_dataparser.py)的`load_3D_points`、`ply_file_path`以及`_load_3D_points`给点和相机应用一致的变换/尺度。
- [Splatfacto源码](https://github.com/nerfstudio-project/nerfstudio/blob/main/nerfstudio/models/splatfacto.py)支持`seed_points`及批次Mask训练。
- 当前`transforms.json`已有光学轴→OpenGL轴转换，但还需要明确点云路径/加载选项和缓存配置。不能只把文件放进目录就假设使用了LiDAR点云，也不能给相机和点云分别使用不同的归一化。

## 3. gsplat 与当前工作区

本机只读检查结果：RTX5060Ti（驱动显示16311MiB显存）、Torch2.7.1+cu128、CUDA12.8、gsplat1.5.3。版本检查仅证明安装/导入状态，不是本轮训练或性能验证。

[官方COLMAP训练示例](https://docs.gsplat.studio/main/examples/colmap.html)和[示例源码](https://github.com/nerfstudio-project/gsplat/blob/main/examples/simple_trainer.py)适合构建离线训练后端。当前上游main已经包含批次Mask的损失处理，但[COLMAP数据加载器](https://github.com/nerfstudio-project/gsplat/blob/main/examples/datasets/colmap.py)中的`mask_dict`主要处理每相机去畸变有效区域，不能据此认定已支持本项目的逐图动态Mask目录。当前本地旧训练适配器也尚无动态Mask接入。

因此，若优先复用现有可用环境，可在锁定gsplat1.5.3的前提下适配本批输入、逐图Mask及相应监督；不能把上游main的新接口当成本机1.5.3已经具备的接口。

## 4. 动态干扰研究与在线扩展

- [SpotLessSplats作者项目页](https://spotlesssplats.github.io/)明确链接到lilygoli仓库。论文为“SpotLessSplats: Ignoring Distractors in 3D Gaussian Splatting”，arXiv初稿2024，作者页列TOG2025；本轮未阅读全文或复现结果。
- [WildGaussians作者项目页](https://wild-gaussians.github.io/)列NeurIPS2024，作者代码是jkulhanek/wild-gaussians；核心是DINO不确定性与外观建模。不能与名称相似的其他Wild-GS仓库混同。
- 若后续目标改为LiDAR/IMU/相机在线联合建图，可看[GS-LIVM](https://github.com/xieyuser/GS-LIVM)。它有实际ROS/C++/CUDA源码和数据集入口；需要适配传感器话题、消息格式及标定，不是直接对现有PCD调用离线训练器。
- [LIV-GaussMap](https://github.com/sheng00125/LIV-GaussMap)主题虽贴合硬件，但本轮核实时其官方仓库根目录只有README和fig，未发现可直接部署的训练/建图源码，因此不列为当前实施首选。

## 当前接入顺序建议

先使用现有gsplat环境，或单独验证Lightning/Splatfacto环境，跑通当前无行人静态基线；确认采用真实LiDAR初始点、已知相机位姿和独立留出RGB，再接入外部动态Mask。SpotLessSplats/WildGaussians留作含行人等真实动态数据的对比方法。

本轮只完成项目检索、关键源码核实和方案比较。没有下载大仓库、安装依赖、启动训练或改写已有数据，不能将任何上游论文性能数字当作本机/本数据集成绩。
