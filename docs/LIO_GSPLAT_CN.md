# MID360/LIO＋单目 RGB 的离线 gsplat Demo

本入口使用现有 gsplat 1.5.3，在主机离线优化静态3DGS。输入为已经采集的实测静态点云、RGB、相机内参及测量时刻的光学相机位姿；外部逐图动态Mask是可选输入。当前数据没有行人，因此本轮正式结果是静态基线，不是动态去除效果实验。

清晰度改进与完整SH交互查看器见 [LIO_GSPLAT_DETAIL_CN.md](LIO_GSPLAT_DETAIL_CN.md)，新查看地址为8891；本页以下数值保留原版基线口径。

## 查看本轮结果

本机服务：**http://127.0.0.1:8890**。鼠标拖动旋转、滚轮调整观察距离；右侧“采集视点”配合“跳到所选视点”可切换到实际采集相机位置。

服务停止后可重启：

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/view_warehouse20_gsplat.sh
```

其他输出目录/端口：`bash scripts/run_sh/view_warehouse20_gsplat.sh OUTPUT_DIR 8891`。当前交互预览使用学习的SH0颜色；下面图片和视频由CUDA使用完整SH外观渲染，两种显示方式不应当逐像素比较。

结果目录：`gsplat_runs/warehouse20_lio_20261102_640/`。

| 文件 | 内容 |
|---|---|
| `gsplat/model.ply` | 真正学习后的3D高斯参数：位置、对数尺度、四元数、透明度及SH；不是普通点云改后缀 |
| `gsplat/model.pt` | 学习参数及相机K/渲染分辨率 |
| `gsplat/gaussians.npz` | 配套查看器使用的同一组学习参数 |
| `gsplat/room_six_views.png` | 从实测相机站位渲染的六个方向 |
| `gsplat/heldout_comparison.png` | 八个相同留出视点，左实际RGB、右模型渲染 |
| `gsplat/walkthrough.mp4` | 180帧/20fps的新插值视角快速预览，非原图播放 |
| `gsplat/heldout_metrics.json` | 全部595张留出RGB的指标 |
| `gsplat/model_coverage.json` | 独立训练后固定几何采样审计 |
| `gsplat/export_validation.json` | PLY/NPZ参数一致性、有限性和视频验证 |
| `gsplat/browser_validation.json` | 实际Chrome连接、WebGL渲染和鼠标拖动检查 |
| `source_dataset.json` | 原始数据位置、输入SHA及照片缺口说明 |
| `gsplat/training_sources.json` | 实际训练时的源码快照索引 |

查看模型只需本输出目录中的模型和帧位姿元数据；重训、输入复核及原始照片查看仍需要原数据集。输出不复制原始bag或全部RGB，避免重复占用磁盘。

## 重新训练

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/warehouse20_gsplat.sh
```

默认读取`lio_dataset_runs/warehouse20_20261102_capture/`，640×480、12,000步，在`gsplat_runs/warehouse20_lio_日期_时间/`新建结果，拒绝覆盖已有目录。包装脚本使用`.envs/roomgs`中的CUDA工具链及gsplat1.5.3，未升级或覆盖现有环境。

```bash
bash scripts/run_sh/warehouse20_gsplat.sh \
  --dataset /path/to/verified_lio_dataset \
  --output /path/to/new_output \
  --steps 12000 --width 640
```

输入为当前LIO采集器的完整验证格式，不是任意PCD目录：需要`static_map.npz`、`dataset/frames.json`及图片、`trajectory_imu.jsonl`、`camera_input.json`、`sensor_geometry.json`、验证/验收元数据。入口核对原始SHA，并从原LIO状态和模型外参重算每帧相机位姿。全部80,900个实测点直接初始化高斯，不读取CAD或Gazebo真值。

现有数据的几何检查通过，但10个货架样本照片双视角条件不足。默认按静态基线范围训练，报告保留缺口。若要求采集的严格联合完整性，增加`--require-complete-photos`；当前这批数据会明确拒绝，原有R3LIVE训练入口的严格门槛也没有被修改。

## 使用外部动态Mask

```bash
bash scripts/run_sh/warehouse20_gsplat.sh \
  --mask-dir /path/to/masks
```

- 每张RGB都必须有对应Mask；RGB文件`0000000001288000000.jpg`对应`0000000001288000000.jpg.png`。
- Mask为与原始RGB同分辨率的单通道二值PNG：**255/白色是有效静态区域，0/黑色是忽略区域**。如果分割网络输出“人=白色”，需先转换成上述有效区域约定。
- 指定Mask后，缺少文件、RGB多通道Mask或灰度非二值会报错，不会悄悄退回全白。全黑帧从训练/留出候选中排除。
- Mask使用最近邻缩放，默认在训练分辨率将忽略区域扩大3像素，可通过`--mask-dilate`调整，减少RGB缩放时的边界颜色混合。
- 训练图像着色初始化跳过忽略像素；L1只对有效像素归一化；SSIM只使用窗口内全部有效的中心；相关投影深度监督也忽略无效像素。没有把行人区域简单涂黑当作监督目标。
- 当前入口接收Mask，不包含人体分割模型。静态PCD若含动态鬼影仍需先清理；Mask不能恢复从未被拍到的背景。

## 资源与验证

图片每步懒加载，深度先验LRU最多32帧；CPU数值库2线程；PyTorch显存分配上限6GiB、进程RSS检查5GiB、输出预算2GiB及磁盘20GiB保留。高斯数量上限默认350,000，在复制/分裂提案阶段限制新增数。只保留一个最新中间参数检查点，当前不提供精确恢复优化器状态的断点续训接口。

CUDA Mask/高斯预算自检：

```bash
bash scripts/run_sh/warehouse20_gsplat.sh --self-test
```

CPU/CUDA损失、格式和严格照片门槛测试：

```bash
env -u PYTHONPATH .envs/roomgs/bin/python -m unittest discover \
  -s integration/lio_gs -p 'test_*.py' -v
```

独立训练后验证：

```bash
env -u PYTHONPATH .envs/roomgs/bin/python integration/lio_gs/verify.py \
  gsplat_runs/warehouse20_lio_20261102_640
```

## 本轮实测结果与边界

- 正式训练12,000步、350,000个高斯，4,163张训练图、595张留出RGB；训练和导出评估合计约253.64秒。
- 留出RGB PSNR **33.5589dB**、SSIM **0.897849**。同八视点优化前后PSNR **17.3959→32.1019dB**；八视点对照和595张集合不是同一统计口径。
- 训练后透明度≥0.01的338,771个高斯，对原固定4,407个可观测20cm目标全部提供20cm内支持。输入照片仍有10个目标不足，严格联合完整性仍为false；不等同连续表面、隐藏内面或所有细节完整。
- PLY/NPZ全部位置、尺度、四元数、透明度和SH逐项差0，参数有限；输入和训练源码指纹未变化。
- 记录的训练内存峰值约2.164GiB，PyTorch显存分配峰值565.98MiB；主要输出约0.30GiB，不包括另存的合成Mask兼容性试验。
- 本机CUDA自检发现HWC转置的channels-last池化反传会向忽略像素泄漏梯度；强制连续NCHW后CPU/CUDA回归均为0。实际gsplat毒化忽略区RGB的损失差0、参数梯度差2.33e-10，采用浮点原子归约容差；复制和分裂预算均实测通过。
- 外部Mask文件接口另用一个忽略矩形、一个全黑帧和其余全白Mask实跑100步，通过并排除全黑帧。该试验只是接口/优化测试，不是行人分割或动态去除效果验证。
- 货架、箱体和墙面可辨，部分标记边缘仍模糊。图像指标为同场景RGB外观留出，LiDAR地图与轨迹来自整次采集；不作为独立外部定位精度或新算法优势。

使用的开源后端：[gsplat](https://github.com/nerfstudio-project/gsplat)。本轮为该后端的本项目数据/Mask工程适配，不主张新3DGS算法。
