# gsplat 清晰度改进与同视角对照

本轮保留原640×480静态基线及原数据集，在同一批RGB、相机位姿、内参和外观训练/留出划分上继续优化。不重新采集、不使用留出RGB拟合，不添加生成式纹理。

## 查看

推荐使用[原生桌面查看器](LIO_GSPLAT_DESKTOP_CN.md)：`bash scripts/run_sh/open_warehouse20_gsplat_desktop.sh`。它不使用网页，旧网页服务目前已停止。以下保留网页方式的可选启动说明。

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/manage_warehouse20_gsplat_viewer.sh start
```

地址：**http://127.0.0.1:8891**。此后台入口由用户级systemd管理，关闭或重新打开VS Code不依赖原终端；没有设置开机/登录自动启动，重启电脑或用户会话结束后需要再次运行启动命令。`start`可重复执行，不会重复启动查看器。

```bash
bash scripts/run_sh/manage_warehouse20_gsplat_viewer.sh status
bash scripts/run_sh/manage_warehouse20_gsplat_viewer.sh logs
bash scripts/run_sh/manage_warehouse20_gsplat_viewer.sh restart
bash scripts/run_sh/manage_warehouse20_gsplat_viewer.sh stop
```

需要在前台观察启动输出时，可在停止后台服务后运行`bash scripts/run_sh/view_warehouse20_gsplat_hq.sh`，该前台进程需要保持终端运行。网页依赖本机查看器服务，模型文件保存在磁盘并不代表网页服务始终在线。旧SH0预览可另外运行`view_warehouse20_gsplat.sh`使用8890端口。

默认选用`gsplat_runs/warehouse20_lio_20261102_hq`的395789高斯版本。80万候选保留为实验数据，没有替换默认模型。

新窗口由CUDA计算完整三阶SH外观，PNG无损传输。鼠标拖动旋转、滚轮调整距离；点击“同视角切换版本”，在相机位置、方向和视场不变时比较原版和细节版。“采集视点”与“跳到所选视点”可返回真实拍摄位置。渲染宽度控制屏幕采样分辨率，不增加原照片已采集的细节。

与原版浏览器的SH0近似不同，新查看器直接使用训练参数和相同gsplat光栅化器，包含高阶SH视角相关外观。其相机采用viser已有的OpenCV约定，无重复翻转Y/Z；相机纵向视场和浏览器宽高比共同决定K，不拉伸原图。

## 重新训练

```bash
bash scripts/run_sh/warehouse20_gsplat_hq.sh --output /path/to/new_detail_output
```

拒绝覆盖已有输出。热启动自`gsplat_runs/warehouse20_lio_20261102_640/gsplat/model.pt`，核对原数据指纹及训练/留出索引。仅继承学习参数，重新建立优化器和增密状态，不是精确断点恢复。

- 固定4758图/4163训练/595外观留出，原始640×480，固定LIO相机位姿。
- 新增18000步；高斯上限800000，按实际梯度增密，不保证用满预算。
- 投影协方差下限eps2d由0.3调整为0.1，最小空间尺度由2mm调整为0.5mm。
- AbsGrad增密，阈值0.0008，8500步前允许增密。
- 均值学习率5e-5指数衰减至1e-6；三阶SH从热启动首步保留。
- 相邻RGB梯度误差权重0.15，与原有有效像素L1/SSIM、实测深度及几何先验联合优化。跨Mask边界的像素对不监督。
- 5GiB RSS检查、6GiB Torch分配上限、3GiB输出检查；单图懒读/深度LRU32/CPU2线程不变。

eps2d与absgrad语义参考[gsplat官方光栅化文档](https://docs.gsplat.studio/main/apis/rasterization.html)，实际运行接口以本机gsplat1.5.3为准。本轮是既有组件与超参数的工程改进；更长训练、增密、损失与渲染参数同时变化，不是逐项消融或算法优越证明。

首轮输出`warehouse20_lio_20261102_hq`，实际395789高斯，未用满上限。另有细节增密阶段：

```bash
bash scripts/run_sh/warehouse20_gsplat_hq_dense.sh --output /path/to/new_dense_output
```

默认从首轮模型继承参数，再优化8000步；增密阈值降低为0.0001、增密截止3500步、均值LR2e-5衰减至1e-6，其他损失/eps2d/最小尺度不变，高斯上限800000。可在命令末尾添加`--initialize-model /path/to/first_stage/gsplat/model.pt`改用自行训练的首轮。两次热启动都重建优化器，总优化步数应与各阶段新增步数区分。

## 比较与验证

`integration/lio_gs/compare.py`在同595张留出图、相同原始位姿/K、640×480与完整SH3下重新渲染两份模型，核对原版PSNR可复现后输出指标。

强边缘误差为相邻RGB差分与真实图像差分的绝对误差，阈值由真实RGB灰度梯度的90分位和0.02下限共同决定，逐图固定、两模型共用，越低越好。它衡量对已采集边缘的逼近，不以模型自身锐度变大作为正确性证据。

输出`evaluation/lio_gsplat_detail_20261102/comparison.json`、`full_comparison.png`与`detail_crops.png`。固定八个均匀留出视点；局部裁剪由真实RGB在32像素网格上最大梯度和选择，不按模型提升大小挑选。图顺序均为**原RGB／原版完整SH／细节版完整SH**；局部图仅最近邻放大两倍以显示像素，不作锐化或超分。

运行同图比较请使用统一CUDA工具链入口，避免直接启动Python缺少CPATH/编译器配置或并发触发CUDA缓存重编译：

```bash
bash scripts/run_sh/compare_warehouse20_gsplat.sh BASELINE_DIR DETAIL_DIR COMPARISON_DIR
```

首次CUDA缓存初始化应等其完成后再启动其他CUDA查看/比较进程；后续窗口使用同一套CC/CXX、CUDA include/lib和MAX_JOBS=1配置。

## 实测结果与版本选择

| 模型 | 高斯数 | 595图PSNR/dB | 595图SSIM | 强边缘误差 | 固定8图PSNR/dB | 输出约GiB |
|---|---:|---:|---:|---:|---:|---:|
| 原版 | 350000 | 33.558913 | 0.897849 | 0.02530047 | 32.101941 | 0.30 |
| 首轮细节版，默认 | 395789 | 35.021992 | 0.915586 | 0.02325261 | 33.422012 | 0.34 |
| 追加增密候选，未采纳 | 800000 | 35.158760 | 0.917510 | 0.02313136 | 33.277329 | 0.68 |

默认版相对原版595图PSNR提升1.4631dB、SSIM提升0.017736、强边缘误差降低8.094%。554/595图PSNR提高、534/595图强边缘误差降低，不能称每一视图都改善。约462.68秒完成本阶段训练/输出评估，训练进程实测HWM2.184GiB、Torch分配峰值643.73MiB；这些是该进程本次测量，不含桌面/查看器或编译器总资源。

追加8000步增密到80万后，全部595图均值稍高，但预先固定8图相对热启动模型退步0.1447dB，既有`verify.py`的同八图改进门槛未通过。因此未放宽门槛，也未以全量均值掩盖该退步；考虑翻倍的体积/高斯数，默认保留首轮版本。候选对照在`evaluation/lio_gsplat_detail_20261102/dense_comparison`，未采纳原因在候选目录`candidate_review.json`。

80万候选原进程在保存8000步最终PT/PLY/NPZ后收到SIGTERM（退出143，来源未定位）；新增`postprocess.py`检查最终步数、原始输入/源码/模型哈希后补完留出评估、六向图和180帧视频，没有再优化。恢复信息、资源范围及中断时间均写入其`training_result.json`。该候选不作为不中断训练的性能测量。原始导出参数检查/固定几何4407/4407通过，与八图质量门槛失败分别记录。

首轮和80万候选均完成真实Chrome连接、静止后截图、同相机A/B及鼠标拖动测试。测试浏览器强制软件WebGL，服务器执行CUDA完整SH；通过不代表GPU交互帧率基准。首轮热启动报告的旧`initial_gaussians`字段误记为80900实测点数，真实继承350000高斯，已以独立`metadata_correction.json`更正并修正未来训练器，保留原报告和实际训练源码快照。

原图仅640×480。近距离文字、从未拍清的小结构、始终遮挡的区域不能靠高斯增密凭空恢复；进一步提高真实细节需要更高分辨率采集并校验同步、标定和位姿。当前数据仍有10个照片覆盖样本不足，清晰度改进不代表严格完整外观验收通过。当前无行人，不新增真实动态去除结论。
