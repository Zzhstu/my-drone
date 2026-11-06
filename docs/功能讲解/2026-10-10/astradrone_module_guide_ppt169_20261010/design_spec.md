<!-- ppt-master-schema: design-spec/v1 -->
# 无人机功能与规则讲解 - Design Spec

## I. Project Information

| Item | Value |
| --- | --- |
| Project Name | astradrone_module_guide |
| Canvas Format | PPT 16:9, 1280 × 720 |
| Page Count | 22 |
| Primary Language | zh-CN |
| Target Audience | 项目作者、同组同学及不熟悉机器人代码的听众 |
| Communication Intent | 先讲模块作用，再解释判断规则，最后串联两个运行任务 |
| Desired Audience Outcome | 能解释每个模块为何存在、输入输出和关键限制，并找到对应源码 |
| Core Message / Ask / Action | 定位统一位置、地图保存并更新证据、规划选路、安全控制决定此刻怎么飞 |
| Delivery Context | 中文组会讲解与课后阅读 |
| Artifact Afterlife | 可编辑PPTX、PDF及详细中文文档 |
| Reading Mode | text |
| Content Strategy | 以当前源码为权威，正文使用通俗中文，完整文件路径放备注；不添加新实验结论 |
| Design Style | 自定义技术讲解手册：深蓝标题、浅色场、绿色处理链、橙色风险与限制 |
| AI Image Acquisition Path | not applicable |
| Generation Mode | continuous |
| Spec Refinement | disabled |
| Speaker Notes | enabled — 本轮主动策略：解释细节、当前参数和源码追溯 |
| Custom Animations | disabled — 工作流默认，方便编辑与打印 |
| Narration Audio | disabled — 工作流默认 |
| Created Date | 2026-10-10 |

委托状态：用户要求直接制作，沿用已授权的自主设计，无待审批选择。无模板引用。设计方向比较：①源码讲解手册（采用），模块段落与相邻机制图，方便逐项说明；②任务场景推演，按遇到行人的事件顺序铺陈，场景感较强但不便定位各模块；③职责地图，突出全局层级与模块边界，简洁但单模块规则文字不足。三者差别在叙事与信息层级，不仅是配色。本轮默认不做设计稿审阅暂停。

## II. Canvas Specification

| Property | Value |
| --- | --- |
| Format | PPT 16:9 |
| Dimensions | 1280 × 720 |
| viewBox | `0 0 1280 720` |
| Margins | 52 px，底部页脚保留 |
| Content Area | 标题后至底部，正文与关系图共享空间 |

## III. Visual Theme

### Theme Style

- **Mode**: custom
- **Mode Behavior**: 每个模块先给目的再讲证据和规则；通过流程中的分支解释限制；在总图中汇合长期规划与即时安全。
- **Visual style**: custom
- **Visual Style Behavior**: 浅色讲解纸面配深蓝标题，图中绿色代表正常处理、橙色代表限制与风险。单一流程采用连贯纵向阅读；多路输入、证据融合和两任务分支用跨区汇合。不同职责不强行画成完全串行链。无装饰照片或虚构图表。
- **Theme**: 中文技术讲解手册
- **Tone**: 清晰、克制、易读

### Color Scheme

| Role | HEX | Purpose |
| --- | --- | --- |
| Background | #F7F9FB | 全页纸面 |
| Secondary background | #EAF3F2 | 同一处理范围 |
| Primary | #16334A | 标题及总图核心 |
| Accent | #168B80 | 正常处理及更新 |
| Secondary accent | #D67B29 | 风险、限制、未完成状态 |
| Body text | #253746 | 正文 |
| Secondary text | #617382 | 标注与页脚 |
| Divider | #D3DEE5 | 边界与关系线 |

## IV. Typography System

### Font Plan

| Role | Character (Reference) | Primary | English if non-English | Fallback tail |
| --- | --- | --- | --- | --- |
| Title | 清晰无衬线 | Microsoft YaHei | Arial | sans-serif |
| Body | 可阅读无衬线 | Microsoft YaHei | Arial | sans-serif |

- **Title stack**: Microsoft YaHei, Arial, sans-serif
- **Body stack**: Microsoft YaHei, Arial, sans-serif

### Font Size Hierarchy

| Purpose | Anchor Size (px) |
| --- | ---: |
| Body | 24 |
| Title | 42 |
| Subtitle | 28 |
| Annotation | 18 |
| Footer | 16 |
| Display | 56 |

## V. Layout Principles

### Deck-wide Direction

- **Hierarchy direction**: 结论标题→作用与规则段落→输入输出关系→限制提醒。
- **Composition tendency**: 讲解页文字与流程并置；证据汇合、两任务对照与总图使用跨区关系布局。
- **Cross-page continuity**: 相同标题和页脚；图的结构由关系决定，不把每页强行变成相同卡片网格。
- **Spacing posture**: 开篇和收尾留白，模块页适度文字密度，总图节点简写。
- **Spacing anchors**: 页边52、块间24、栏间32、角半径12、正文行距36 px。

## VI. Icon Usage Specification

- **Primary bundled library**: none

| Icon Path | Suitable Scenarios |
| --- | --- |

## VIII. Image Resource List

| Filename | Dimensions | Ratio | Purpose | Type | Image pattern | Crop Policy | Acquire Via | Status | Reference | text_policy | page_role |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |

## IX. Content Outline

### Part 1: 位置与感知

#### Slide 01 - 看清环境，更新地图，安全飞行
- **Audience move**: 不清楚链路→理解本次讲解目的
- **Relationships**: 环境观测提供位置与空间证据，规划及安全共同影响飞行，新观测形成反馈
- **Title**: 看清环境，更新地图，安全飞行
- **Core message**: 本项目是多模块协作，不是一个避障文件完成所有事情。
- **Content**: 讲解作用、规则和模块连接；当前源码依据；不新增实验。

#### Slide 02 - 功能包与模块：谁负责哪件事
- **Audience move**: 把源码文件当包→分清工具箱和小工具
- **Relationships**: 定位包、衔接与避障包、飞行示例包及仿真包各自包含不同程序，主演示使用其中一条装配链
- **Title**: 功能包与模块：谁负责哪件事
- **Core message**: 多数避障模块同属fastlio_bridge。
- **Content**: 四类包；工具箱类比；当前主演示与独立飞行示例的区别。

#### Slide 03 - 定位建图：位置不只靠惯性积分
- **Audience move**: 只靠惯性积分→理解预测与环境校正
- **Relationships**: 惯性预测与新扫描匹配汇合为位置修正，修正后更新匹配地图并反馈
- **Title**: 定位建图：位置不只靠惯性积分
- **Core message**: 惯性负责短时预测，激光环境约束帮助修正。
- **Content**: 位置与姿态输出；匹配地图；当前模拟分支不宣称逐点补偿执行。

#### Slide 04 - 坐标衔接：让位置、点云与控制一致
- **Audience move**: 标签改名就对齐→理解数据转换与一次初始化
- **Relationships**: 初始飞控与定位建立转换，后续每帧定位和点云被转换并分别供下游及航迹记录
- **Title**: 坐标衔接：让位置、点云与控制一致
- **Core message**: 真正旋转平移数据，不只是改消息标签。
- **Content**: 一次初始对齐；之后激光定位独立输出；不回灌飞控避免循环依赖。

#### Slide 05 - 前景候选：先找与开头不一样的点
- **Audience move**: 新点就是动态→理解候选与确认分离
- **Relationships**: 当前扫描经筛选后与背景比较，分成已有背景与不同位置；后者才进入跟踪
- **Title**: 前景候选：先找与开头不一样的点
- **Core message**: 与背景不同，只是动态候选。
- **Content**: 开头40帧；近场与高度限制；背景不永久在线吸收；新货架表面也可能候选。

#### Slide 06 - 连续跟踪：多次证据才确认运动
- **Audience move**: 一次变化就报警→理解连续编号和证据
- **Relationships**: 候选组成目标并与历史配对，连续运动和稳定地图支持共同决定确认或继续观察
- **Title**: 连续跟踪：多次证据才确认运动
- **Core message**: 连续运动证据与静态支持共同约束误报。
- **Content**: 近点成组；编号；速度位移方向；4次确认；显示平滑和判定历史分开；丢失删除。

#### Slide 07 - 辅助接口：短时预测与类别转发
- **Audience move**: 话题名等于识别网络→理解两条辅助支路
- **Relationships**: 跟踪目标分别输入短时预测和类别转发，主碰撞判断直接使用目标当前位置及速度
- **Title**: 辅助接口：短时预测与类别转发
- **Core message**: 预测是辅助，类别接口默认未运行人体网络。
- **Content**: 恒速外推0.5秒；突然转向限制；类别接口默认关闭原样转发。

### Part 2: 风险、地图与选路

#### Slide 08 - 动态风险：判断物体会不会靠近
- **Audience move**: 只看当前距离→理解双方相对运动
- **Relationships**: 目标与无人机位置速度汇合，计算短时最近接近及高度关系，分配风险并输出方向
- **Title**: 动态风险：判断物体会不会靠近
- **Core message**: 既看现在，也看双方短时会不会靠近。
- **Content**: 1.5秒范围；1.35米水平中心距离；垂直关系；紧急条件；保持与退出防抖。

#### Slide 09 - 静态风险：最新扫描检查附近净空
- **Audience move**: 地图说可走就安全→理解当下扫描复查
- **Relationships**: 新扫描经高度及邻域支持过滤、动态目标遮罩，得到净空并比较尺寸加余量阈值
- **Title**: 静态风险：最新扫描检查附近净空
- **Core message**: 最新扫描负责最后一层近场空间检查。
- **Content**: 半径0.40米；仓库安全0.30米、紧急0.25米、退出0.35米；并非所有模块同一距离。

#### Slide 10 - 风险融合：统一等级与避让建议
- **Audience move**: 两路风险互相争抢→理解等级与速度分别融合
- **Relationships**: 动静风险汇合，取更高等级；速度按风险组合求和或加权，冲突时静态优先，过期来源排除
- **Title**: 风险融合：统一等级与避让建议
- **Core message**: 等级取较高者，方向并非永远取同一个来源。
- **Content**: 一级求和；双二级0.6/0.4；抵消静态优先；上限和平滑；控制器另负责失效保护。

#### Slide 11 - 地图维护：一份存储，多种用途
- **Audience move**: 有三张互不相关地图→理解共享空间格和视图
- **Relationships**: 当前扫描和位置更新同一存储，其不同证据门槛及范围派生全局、稳定和滚动局部视图
- **Title**: 地图维护：一份存储，多种用途
- **Core message**: 同一份证据存储，按用途输出不同地图。
- **Content**: 全局选路；稳定支持；局部反映新变化；不是FAST-LIO内部匹配树。

#### Slide 12 - 鬼影清理：有“已空”证据才撤销
- **Audience move**: 没看到就删除→理解可见自由证据
- **Relationships**: 匹配时间的雷达原点与射线产生自由空间证据，端点保护、遮挡和重复证据决定删除或保留
- **Title**: 鬼影清理：有“已空”证据才撤销
- **Core message**: 看穿旧位置才是清空证据，遮挡不是。
- **Content**: 正确原点和时间；连续3或6次门槛；探索仅证据删除；静止人体仍是现实障碍。

#### Slide 13 - 指定目标规划：地图变化后重新找路
- **Audience move**: 只会躲→理解长期选路与局部修正
- **Relationships**: 稳定地图及自由证据生成路线，局部占用修正后重规划，安全旧路线与明显改善比较后选择可见航点
- **Title**: 指定目标规划：地图变化后重新找路
- **Core message**: 先选绕行路线，再随局部变化重新评估。
- **Content**: 网格搜索；同飞行高度；4→8→12米局部尝试；安全旧路25%改善切换；不切角。

#### Slide 14 - 自主探索：从已知空间走向未知边界
- **Audience move**: 预知仓库路线→理解只基于观测前沿
- **Relationships**: 新扫描更新未知自由占用状态，在可达边界估计新观察收益，选安全航点，飞行产生新扫描反馈
- **Title**: 自主探索：从已知空间走向未知边界
- **Core message**: 从实际观测中选下一处能看见未知的位置。
- **Content**: 不读货架真值选路；收益与代价；未知不随便穿；失败降权；正常余量与恢复余量不同。

#### Slide 15 - 补扫与结束：返航不等于全图完整
- **Audience move**: 回来就完整→区分收敛、部分、保存
- **Relationships**: 探索低增益触发补扫检查，是否仍有可达收益与质量状态决定继续、收敛或部分返航，返航到家与保存才完成任务
- **Title**: 补扫与结束：返航不等于全图完整
- **Core message**: 任务状态与地图完整性必须分开说明。
- **Content**: 90秒低增益检查；3次2方向证据；已观测表面95%非全图；0.30米5秒返航；独立覆盖审计。

### Part 3: 执行、观察与整体协作

#### Slide 16 - 飞行控制：航点与风险共同决定速度
- **Audience move**: 规划路线自动就执行→理解控制器逐周期裁决
- **Relationships**: 起飞准备后航点和即时风险输入同一控制器，正常跟随、等待或紧急策略分支再经过扫描检查输出飞控指令
- **Title**: 飞行控制：航点与风险共同决定速度
- **Core message**: 规划说去哪，控制器决定现在能不能走。
- **Content**: 稳定地面和模式准备；起飞位移；风险分支；探索紧急策略与目标模式不同；过期悬停。

#### Slide 17 - 可视化：区分地图、路线和目标球
- **Audience move**: 所有标记都是点云→理解显示层含义
- **Relationships**: 各真实消息分别生成点云、路线、目标和候选标记，按各自时效删除或更新，显示不反向控制
- **Title**: 可视化：区分地图、路线和目标球
- **Core message**: 显示是观察窗口，不是另一个决策器。
- **Content**: 三类地图；已飞轨迹与规划路线；目标球；蓝色方块是候选标记；短寿命和有界轨迹。

#### Slide 18 - 运行监测：分清无目标与没数据
- **Audience move**: 没球就是程序停→理解新鲜度诊断
- **Relationships**: 消息时间、数量和频率汇合为诊断，空目标和超时分别报告；诊断供人观察而非代替控制
- **Title**: 运行监测：分清无目标与没数据
- **Core message**: 正常空结果与通信停滞不是一回事。
- **Content**: 统计消息数量与年龄；无目标正常；过期值得检查；安全动作由控制器执行。

#### Slide 19 - 点云保存：把真实维护结果带走
- **Audience move**: 输出文件就是干净完整→理解来源和保存检查
- **Relationships**: 实际维护地图经新鲜度和有效点检查，临时写入并替换最终文件，另写来源记录；失败返回原因
- **Title**: 点云保存：把真实维护结果带走
- **Core message**: 保存真实话题结果，不按世界真值裁剪“造好图”。
- **Content**: 新鲜度3秒；有限坐标；临时文件；PCD和来源；存在不代表全图完整。

#### Slide 20 - 演示与飞行示例：只保留一个指挥者
- **Audience move**: 所有示例同时启→理解装配与冲突
- **Relationships**: 演示脚本选择任务并启动装配；指定目标规划与自主探索互斥发布航点；独立键盘和预设飞行示例属于另一路
- **Title**: 演示与飞行示例：只保留一个指挥者
- **Core message**: 启动脚本管组织，飞行示例不要和主演示抢控制。
- **Content**: 仿真、启动、录制、保存；键盘、固定位置、走廊、回环预设航线；offboard源码区别。

#### Slide 21 - 总流程：长期路线与即时安全并行
- **Audience move**: 单串行避障链→掌握规划与即时安全双路汇合
- **Relationships**: 统一观测分成维护地图和感知风险，两种任务规划互斥提供航点，与融合风险汇入控制；执行反馈新观测；显示保存旁路消费消息
- **Title**: 总流程：长期路线与即时安全并行
- **Core message**: 地图决定长期路线，最新观测约束此刻动作。
- **Content**: 位置点云基础；地图规划主路；动态静态安全路；控制汇合；反馈；显示记录旁路。

#### Slide 22 - 如何讲清我们的工作
- **Audience move**: 笼统声称完美→清晰描述实现和证据边界
- **Relationships**: 输入、持续地图证据、任务航点与即时安全形成总结；实现事实与需要实验验证的主张对照
- **Title**: 如何讲清我们的工作
- **Core message**: 把已有定位与动态环境下地图、选路、安全执行协作讲明白。
- **Content**: 三段中文讲法；当前工程实现；未保证所有人体、全图完整和机载实时；配套源码文档。

## X. Speaker Notes Requirements

- **Generation**: enabled
- **Filename**: match each SVG filename under `notes/`
- **Content**: 作用与判断规则的口语解释；引用完整当前源码/config/launch位置；提醒当前状态和限制，无虚构性能数字。
- **Total duration**: 25–35分钟，可按模块节选
- **Notes style**: 中文通俗讲解
- **Presentation purpose**: 用模块作用和规则解释代码，最后连接整体闭环。
