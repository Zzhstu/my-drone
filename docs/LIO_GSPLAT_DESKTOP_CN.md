# 原生桌面查看3DGS

当前已提供 **AstraDrone 3DGS Desktop** 原生窗口，复用本机gsplat CUDA渲染与GLFW/OpenGL显示，不使用浏览器、HTTP或逐帧PNG编码。默认读取已验收的395789高斯细节模型，完整三阶SH外观。

```bash
cd /home/a/AstraDroneOpen
bash scripts/run_sh/open_warehouse20_gsplat_desktop.sh
```

窗口独立于VS Code终端运行。重复启动不会重复加载模型；已运行时用Alt+Tab切换到窗口。关闭窗口或Esc退出。应用菜单可以搜索“3DGS 本地查看器”；桌面另有同名图标，如果GNOME提示未允许启动，可右键“允许启动”，或使用应用菜单/上面命令。当前环境的gio不支持设置metadata::trusted，未假称桌面图标已自动受信任。

| 操作 | 功能 |
|---|---|
| 鼠标左键拖动 | 改变看向 |
| 鼠标右键拖动 | 平移 |
| 滚轮 | 向前/向后移动 |
| W/S、A/D | 前后、左右移动 |
| Q/E | 降低/升高 |
| Shift | 加速 |
| Home | 回到初始采集视角 |
| 左/右方向键 | 在采集视点间切换 |
| F | 切换渲染宽度上限1280/960 |
| 加/减号 | 调整移动速度 |

普通启动默认1280×800窗口，开启垂直同步；最大化时保持实际窗口宽高比，渲染宽度最多1280，GPU放大显示。静止时保留最后一帧、按需重绘，降低空闲GPU负担。渲染分辨率提高不等于增加640×480源照片的真实细节。

前台启动和显式读取高斯PLY：

```bash
bash scripts/run_sh/view_warehouse20_gsplat_desktop.sh \
  --model gsplat_runs/warehouse20_lio_20261102_hq/gsplat/model.ply
```

本入口针对当前项目的相机元数据和三阶SH模型；读取原始log-scales/logit-opacity/quaternion/SH，不是把普通点云当成3DGS。当前PLY使用训练对应eps2d=0.1；其他模型可以通过`--eps2d`指定，且需提供匹配的相机元数据，不主张任意场景自动对齐。

## 实测与范围

- 本机RTX5060Ti、NVIDIA OpenGL，确认不是llvmpipe软件渲染。
- PT模型960×720/160帧自动相机运动，中位帧时间4.318ms、约231.58FPS；PLY模型1280×800/160帧，中位5.088ms、约196.55FPS。自检关闭垂直同步、预热10帧，包含CUDA读取和GL显示交换，排除加载/空闲；日常开启垂直同步约60FPS。这不是与SIBR或网页进行相同协议对照的性能结论，也不是最低帧率保证。
- PLY自检与既有完整SH渲染最大像素差1，直接GL显示与输入RGB最大像素差0；差异包含CPU正交位姿逆与原GPU矩阵逆的浮点数影响。
- 实际创建桌面窗口并查看截图。针对窗口发送过键盘/鼠标输入，画面发生变化；自动Home恢复比较未通过，同时诊断记录出现测试没有发送的W键事件，存在并发操作，不能据此声称输入回归全部通过或Home已定位故障。失败结果/截图保留；停止继续注入输入，保留窗口供用户操作。Home的回调实现设置原始相机位姿，未以放宽像素容差掩盖测试失败。
- 原模型/数据不变，没有重训或新增覆盖/动态去除证据。只在`.envs/roomgs`以`--no-deps`新增glfw2.9.0、PyOpenGL3.1.10，约3.5MB下载，未升级Torch/gsplat。

证据：`evaluation/lio_gsplat_desktop_20261103/`下的`native_pt/`、`native_ply/`、`interactive/`及输入检查日志。运行诊断`interactive/native_live.json`按需记录相机/分辨率/帧率，关闭应用后写完整运行报告。

已停止旧网页服务，释放其GPU/内存。如需诊断桌面服务：

```bash
systemctl --user status astradrone-gsplat-desktop.service
journalctl --user -u astradrone-gsplat-desktop.service -n 30 --no-pager
```

## 其他现成桌面工具

- [官方SIBR实时查看器](https://github.com/graphdeco-inria/gaussian-splatting#interactive-viewers)：原版3DGS提供的原生OpenGL查看器，另有安装/构建依赖。
- [Splatviz](https://github.com/Florian-Barthel/splatviz)：原生交互式查看/编辑器；本轮检查其当前README/pyproject，要求Python3.10及额外diff-gaussian-rasterization，与本机已有3.11环境不同。

本轮没有部署这两个框架；当前打开的工具是本项目复用gsplat的原生窗口适配，不能冒称SIBR或Splatviz运行结果。
