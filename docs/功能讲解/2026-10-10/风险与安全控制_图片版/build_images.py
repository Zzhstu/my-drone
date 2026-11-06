#!/usr/bin/env python3
"""独立绘制讲解用 SVG/PNG；不生成 PPT、不调用 PPT 制作技能。

运行：QT_QPA_PLATFORM=offscreen python3 build_images.py
图中文字使用中文字体；参考 PPT 只用于取色和人工风格核对。
源码/配置哈希在生成前后比较，禁止把生成图片当作飞行验证。
"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import hashlib
import json
import math
import re
from pathlib import Path
from xml.sax.saxutils import escape
from zipfile import ZipFile, ZIP_DEFLATED

import yaml
from PIL import Image, ImageDraw, ImageFont
from PyQt5.QtCore import QByteArray, QRectF
from PyQt5.QtGui import QFont, QFontMetricsF, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer
from PyQt5.QtWidgets import QApplication

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
B = ROOT / 'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
REFERENCE = Path('/home/a/下载/fastlio2ppt.pptx')
FILES = [REFERENCE] + [B / p for p in [
    'src/collision_risk.cpp', 'src/static_collision_risk.cpp',
    'src/avoidance_fusion.cpp', 'src/mavros_avoidance_controller.cpp',
    'config/dynamic_risk.yaml', 'config/static_risk.yaml',
    'config/avoidance.yaml', 'config/astra_mavros_controller.yaml',
    'config/warehouse_navigation.yaml', 'config/warehouse_exploration.yaml',
    'launch/astra_dynamic_avoidance.launch', 'launch/warehouse_exploration.launch',
    'scripts/warehouse_explorer.py', 'scripts/exploration_grid.py',
]]
W, H = 1600, 900
NAVY, BLUE, TEAL = '#15334C', '#24618A', '#137B7E'
MUTED, BORDER, PALE = '#657585', '#D5E0E6', '#EFF5F8'
GREEN, AMBER, RED = '#177C73', '#A46E18', '#B94343'
APP = QApplication.instance() or QApplication([])
TEXT_CHECKS = []
PAGE_TEXT = {}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def font(size, bold=False):
    f = QFont('Noto Sans CJK SC')
    f.setPixelSize(round(size))
    f.setBold(bold)
    return f


def wrap(text, width, size=25, bold=False):
    metric = QFontMetricsF(font(size, bold))
    lines = []
    for para in str(text).split('\n'):
        line = ''
        for ch in para:
            if line and metric.horizontalAdvance(line + ch) > width:
                if ch in '，。；：！？、）》】%％' and len(line) > 1:
                    # 中文闭合标点不可单独出现在下一行行首。
                    lines.append(line[:-1].rstrip())
                    line = line[-1] + ch
                else:
                    lines.append(line.rstrip())
                    line = ch
            else:
                line += ch
        lines.append(line.rstrip())
    return lines


class Page:
    def __init__(self, number, name, title, subtitle, takeaway, sources):
        self.number, self.name, self.sources = number, name, sources
        self.parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">']
        self.rect(0, 0, W, H, '#FFFFFF', stroke='none')
        self.text(60, 40, title, 47, NAVY, True, width=1450)
        self.text(62, 105, subtitle, 24, MUTED, width=1460)
        self.line([(60, 151), (1540, 151)], BORDER, 1.5, arrow=False)
        self.rect(60, 774, 1480, 70, PALE, stroke='none')
        self.text(80, 792, '核心规则', 26, BLUE, True)
        self.text(210, 793, takeaway, 25, NAVY, width=1300)
        self.text(60, 863, '现有实现讲解 · 参数不是实验结果 · 源码对应见配套说明', 17, MUTED)
        self.text(1400, 862, f'{number:02d} / 10', 18, MUTED)

    def rect(self, x, y, w, h, fill='#FFFFFF', stroke=BORDER, radius=0, dashed=False):
        dash = ' stroke-dasharray="7 5"' if dashed else ''
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" fill="{fill}" stroke="{stroke}" stroke-width="1.5"{dash}/>')

    def line(self, points, color=BLUE, width=2, arrow=True, dashed=False):
        pts = ' '.join(f'{x},{y}' for x, y in points)
        dash = ' stroke-dasharray="7 5"' if dashed else ''
        self.parts.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}"{dash}/>')
        if arrow and len(points) > 1:
            # QtSvg 对 marker-end 支持有限，直接画三角形，保证 PNG 和 SVG 都有箭头。
            end_x, end_y = points[-1]
            for start_x, start_y in reversed(points[:-1]):
                length = math.hypot(end_x-start_x, end_y-start_y)
                if length > 0:
                    ux, uy = (end_x-start_x)/length, (end_y-start_y)/length
                    size = max(10, width*4)
                    back_x, back_y = end_x-size*ux, end_y-size*uy
                    half = size*0.48
                    triangle = [(end_x,end_y), (back_x-half*uy,back_y+half*ux),
                                (back_x+half*uy,back_y-half*ux)]
                    triangle_points = ' '.join(f'{x:.2f},{y:.2f}' for x,y in triangle)
                    self.parts.append(f'<polygon points="{triangle_points}" fill="{color}"/>')
                    break

    def circle(self, x, y, r, fill=PALE, color=BLUE, dashed=False):
        dash = ' stroke-dasharray="7 5"' if dashed else ''
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{color}" stroke-width="2"{dash}/>')

    def text(self, x, y, text, size=25, color=NAVY, bold=False, width=None, center=False, line_height=None):
        lines = wrap(text, width, size, bold) if width is not None else str(text).split('\n')
        lh = line_height or size * 1.42
        metric = QFontMetricsF(font(size, bold))
        for i, line in enumerate(lines):
            yy = y + i * lh
            actual = metric.horizontalAdvance(line)
            xx = x - actual / 2 if center else x
            ok = xx >= -0.1 and xx + actual <= W + 0.1 and yy >= 0 and yy + size <= H
            TEXT_CHECKS.append(dict(page=self.number, text=line, x=xx, y=yy,
                                    width=actual, size=size, in_page=ok))
            PAGE_TEXT.setdefault(self.number, []).append(line)
            self.parts.append(f'<text x="{x}" y="{yy + size}" font-family="Noto Sans CJK SC,Microsoft YaHei,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{"middle" if center else "start"}">{escape(line)}</text>')
        return len(lines) * lh

    def block(self, x, y, w, h, title, body, color=BLUE, size=25):
        self.rect(x, y, w, h, '#F6FAFC')
        self.rect(x, y, w, 69, color, stroke='none')
        heading_size = 27
        while len(wrap(title, w-36, heading_size, True)) > 1 and heading_size > 21:
            heading_size -= 1
        self.text(x+18, y+18, title, heading_size, '#FFFFFF', True, width=w-36)
        # 按字体真实宽度选最大的可容纳字号，不裁切，也不隐藏正文。
        while size > 21:
            needed = sum(len(wrap(t,w-46,size))*size*1.38 for t in body) + max(0,len(body)-1)*14
            if needed <= h-100:
                break
            size -= 1
        cursor = y + 85
        for text in body:
            hh = self.text(x+23, cursor, text, size, NAVY, width=w-46, line_height=size*1.38)
            cursor += hh + 14
        if cursor - 14 > y + h - 14:
            raise ValueError(f'page {self.number} block {title} overflow: {cursor} > {y+h}')

    def mini(self, x, y, w, h, title, detail='', color=BLUE):
        self.rect(x, y, w, h, '#F4F9FB', color)
        self.text(x+w/2, y+17, title, 27, color, True, width=w-30, center=True)
        if detail:
            self.text(x+w/2, y+58, detail, 22, MUTED, width=w-30, center=True)

    def finish(self):
        self.parts.append('</svg>')
        raw = '\n'.join(self.parts).encode('utf-8')
        svg = HERE/'SVG'/f'{self.number:02d}_{self.name}.svg'
        png = HERE/'PNG'/f'{self.number:02d}_{self.name}.png'
        svg.parent.mkdir(exist_ok=True)
        png.parent.mkdir(exist_ok=True)
        svg.write_bytes(raw)
        renderer = QSvgRenderer(QByteArray(raw))
        if not renderer.isValid():
            raise ValueError(f'Invalid SVG: {svg}')
        img = QImage(3840, 2160, QImage.Format_ARGB32)
        img.fill(0xFFFFFFFF)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.TextAntialiasing, True)
        renderer.render(painter, QRectF(0, 0, 3840, 2160))
        painter.end()
        if not img.save(str(png)):
            raise RuntimeError(f'Could not save {png}')
        return dict(number=self.number, name=self.name, svg=str(svg), png=str(png),
                    sources=self.sources, text=PAGE_TEXT[self.number])


def flow_page(number, name, title, subtitle, steps, takeaway, sources, extra=None):
    p = Page(number, name, title, subtitle, takeaway, sources)
    for i, (heading, lines) in enumerate(steps):
        x = 60 + i * 378
        p.block(x, 212, 346, 420, heading, lines, BLUE if i % 2 == 0 else TEAL, 25)
        if i < 3:
            p.line([(x+349, 425), (x+375, 425)])
    if extra:
        p.text(70, 666, extra, 26, NAVY, width=1450)
    return p


NOTES = [
('模块总览：规划与即时风险不是一回事', '''规划回答“应该沿哪条路到目标”，即时风险回答“现在还能不能执行这段速度”。动态风险接收已确认运动目标的位置、速度及无人机位姿；静态风险接收当前注册扫描和位姿，两条分支并行运行。融合器汇总等级和建议避让速度，控制器再结合航点、起飞状态和数据时效输出飞控速度。指定目标演示使用通用规划器；探索演示使用自主探索器作为唯一航点来源。
风险模块本身不负责地图清理，不负责全局绕行，也不直接发飞控指令。长期地图里的行人鬼影由地图维护和规划更新处理；即时安全判断尽量使用当前扫描，避免只依赖旧地图。动态识别不是人体语义识别：尚未确认运动的静止人，仍应通过当前扫描作为真实占用参与静态风险与规划。'''),
('动态风险：预测未来会不会靠近', '''无人机速度由相邻位姿差估计并平滑；目标速度来自上游跟踪。模块先看两者相对位置和相对速度，在最多1.50秒的预测窗口内寻找水平距离最小的时刻，并检查该时刻的高度差。普通预警需要正在接近、预测水平中心距离小于1.35米且预测高度差小于0.80米；近距离但正在远离的目标不必触发新的普通预警，原有锁定/解除保持可能仍延续。
紧急风险有两条入口：此刻水平中心距离小于0.75米且当前高度差小于0.80米；或者正在接近、最近接近发生在0.80秒以内、预测水平中心距离小于0.45米且预测高度差小于0.80米。当前近距离入口不要求正在接近，所以已经贴得太近，即使正在远离也可能被判紧急。
这些是中心距离门限，不是无人机外缘到人体外缘的准确间隙。目标尺寸没有自动和机体尺寸求和，预测假设短时速度近似不变；突然转向、加速、漏检或速度估计错误都会影响判断。图中相遇示意为规则解释，非实测轨迹。'''),
('动态风险流程与保持规则', '''对每个目标计算接近状态、预测距离、预测高度差和紧急条件；模块以距离危险程度和时间紧迫程度评分，紧急目标额外加分。已有威胁通常继续锁定同一编号；新出现的紧急目标可打断原锁定。不是把所有人的方向简单相加。
建议方向主要沿相对运动方向的侧向，选择更远离当前目标的一侧。同一目标锁定时避免左右方向瞬间翻转，换目标时尽量保持原速度所在半平面；随后平滑方向和建议速度。建议速度范围0.22到0.80米/秒，不等于无人机最终执行速度，控制器仍会限速或停车。
已有目标落在1.60米预测退出区内且仍接近时可以继续保持；短暂目标缺失保留最多0.30秒，解除判定另有0.30秒保持，紧急级还有0.25秒短保持。不能把不同计时器合并成“0.30秒后一定解除”。定时器每0.05秒检查一次，是设置周期而非本轮测得的实时性能。'''),
('静态距离：先计入机体，再看剩余间隙', '''静态分支先得到机体中心到受支持障碍表面的水平距离，再扣掉机体圆形半径。仓库配置半径0.40米，因此中心距表面0.70米时，外缘剩余间隙约0.30米。机体以固定水平圆近似，不是倾斜机体的精确三维包络。
仓库小于0.25米外缘间隙为紧急；小于0.30米为预警；已经在风险中时，间隙未达到0.35米仍保持一级预警。因此中心距表面分别为小于0.65米紧急、小于0.70米预警、达到0.75米才解除。恰好0.65米不满足严格小于的紧急条件，通常仍是一级；0.70到0.75米之间是否一级取决于之前是否激活。
普通example配置半径0.35米，外缘预警/紧急/解除间隙为0.55/0.30/0.60米，转换为中心到表面0.90/0.65/0.95米。仓库规划膨胀0.80米已含机体，不应再加0.40米；规划离散还会带来余量，所以不要用静态风险阈值直接判定所有窄缝可飞。'''),
('静态风险：从最新扫描中找到可靠障碍', '''当前输入是注册扫描，而非累计全局点云。先确认坐标系与位姿，然后去掉无效点，并筛选机身附近高度：相对高度低于-0.35米或高于+0.45米的点不参与；只看水平2.0米内且距离不小于0.05米的点。
已确认动态目标周围使用半径0.60米、上下各1.00米圆柱掩膜，使同一行人尽量不在静态与动态两边重复触发；空目标数组不立即清掩膜，最多保留0.60秒应对短漏检。这是近似规则，不是完美动静分割，掩膜也可能遮掉目标旁边的真实结构。
在剩余点中找最近原始点，再要求其三维0.30米邻域至少有4个支持点，以这些点的水平距离中位数代表障碍距离、均值代表位置。若支持不够，该帧测量无效，并不是继续寻找下一个可靠簇，更不能推断环境无障碍。有效测量扣除半径后决定等级，同时产生远离该障碍中心的建议速度。'''),
('风险融合：等级取严重者，方向按情形组合', '''每个来源的等级与速度消息都须在0.40秒接收时效内才作为新鲜来源。融合等级取动态和静态等级中的较高者。仅一侧二级时，建议方向取该侧；两侧均一级时，建议速度直接相加后限幅；两侧均二级时，按静态0.60、动态0.40组合。这两个权重只用于“双二级”，不是所有时刻都按六四比例。
双二级建议几乎相互抵消、组合速度小于0.05米/秒时，改选静态紧急方向，并输出对应来源说明。输出建议速度再平滑和限幅到0.80米/秒；等级为0时立即将过滤速度清零。
融合并不是最终动作裁决。控制器在一级时通常停车，探索在二级时通常也停车，故有“避让建议向量”不等于真的横移。过期来源在当前代码中会被当作0，融合器仍可持续输出新的聚合消息；因此聚合时间戳新鲜不能证明两条上游感知都有效。'''),
('安全控制：普通演示与探索演示分别说明', '''控制器先检查飞控连接、有效本地位姿、飞行模式和初始化状态；从地面稳定窗口重新固定起飞基准，发送预备指令后请求外部控制和解锁，确认实际在空中且高度稳定驻留后再允许导航。地面稳定窗口3秒、预发送3秒；实际空中高度在目标0.12米内驻留1秒。模式/解锁丢失或本地高度突然跳变时不自动反复解锁。
导航中，一级预警默认停止水平前进、优先让行；纯静态预警可在条件检查通过后低速退让。普通example一级静态退让上限0.15米/秒，仓库导航0.10米/秒；这不用于追赶目标。普通演示二级会采用融合避让建议，最终水平限幅0.65米/秒。
探索关闭旧式最近障碍斥力退让，导航上限0.30米/秒；二级默认水平零速度，并要求新鲜扫描。普通example导航上限0.35米/秒、仓库导航0.25米/秒。以上均为当前配置，不是实测刹停能力。'''),
('控制器流程：检查通过才执行航点', '''完成起飞之后，先验证航点和控制链时效。位姿接收超时0.35秒、融合等级或建议速度接收超时0.45秒时停止推进；探索另要求当前扫描采集时间不超过0.30秒且点云非空。航点超过1.50秒未更新时目标跟踪速度置零；这并不阻止独立的紧急避让分支执行。
零级风险按位置偏差生成追踪速度；探索还对沿速度方向未来0.50秒短段检查0.80米净空，任何相关当前点太近就把水平速度置零。一级停车/检查静态恢复；二级按普通或探索策略处理，最后限制水平与垂直速度并以30次/秒设置周期发送。
中间航点容差0.03米，明确最终目标容差0.25米。悬停到位后不提前退出风险检查，避免“已经到达”屏蔽后续危险。探索器自己判定最终到达，控制器收到的是中间航点。高度控制通常保留，但缺失目标/控制链数据时垂直也置零。'''),
('窄通道短退路：必须有授权和二次检查', '''探索器先在已观测自由空间内计算通往正常0.80米安全格的恢复路线，短命令不超过0.20米；必须在机体足迹已知、机心净空不低于0.45米且不继续变小的条件下前进。这是从不安全膨胀区恢复的例外，不是把正常规划净空降到0.45米。
控制器再检查授权≤0.75秒、恢复点与当前航点平面差≤0.03米、动态风险为0且新鲜、当前扫描≤0.30秒；恢复点距离须在0.01到0.28米范围，执行速度上限0.08米/秒。对未来0.50秒短段的5个样本检查扫描净空≥0.45米，且相对起点及前一样本不得减少超过0.005米。该数值容差是实现门控，不是动力学证明。
当前实际边界：guardedExplorerRecovery要求来源字符串恰为static_avoid；但avoidance_fusion在静态二级时发布static_emergency。这使正常装配的二级短退路被来源门控拒绝，即使代码在二级分支调用该函数。本次只记录和解释，未修复。一级纯静态可尝试授权短退路；二级讲解应写默认水平停车，不能无条件写“二级能恢复”。'''),
('规则小结：有效数据和安全证据不能省略', '''当前0/1/2为工程规则等级，并不代表形式化安全证书。动态风险接收超时会重置并输出0；静态点云失效、坐标不符、没有足够支持点，也可能输出0；融合器再把过期来源当0。因此“0”等级与“确实有有效观测且无危险”必须区分。探索的扫描门控能拦住部分扫描失效，但没有独立完整的上游有效标志和统一采集时间检查。
静态风险主要看机体高度带的近场点，动态预测依赖匀速近似；没有倾斜机体三维扫掠、目标准确尺寸求和、完整制动距离/延迟模型。普通与探索二级动作不同；起飞阶段在TAKEOFF分支先返回，未走导航阶段完整风险裁决，所以不能将后者描述成全阶段保护。空中零速度指令也不代表物理瞬间停住。
本套材料用当前源码/配置和参考PPT视觉样式制作，仅检查图文和文件，没有改代码，没有重新编译、飞行或碰撞验证。后续可验证来源有效性、二级恢复来源接口、实际刹停距离和接触净空，但这些不属于本次已完成工作。'''),
]


def generate():
    pages = []
    p = Page(1, '模块总览', '动态风险、静态风险与安全控制', '规划决定走哪条路；即时风险决定此刻还能不能继续执行。',
             '规划速度只是候选指令，必须经过即时风险与控制检查。', ['src/collision_risk.cpp', 'src/static_collision_risk.cpp', 'src/avoidance_fusion.cpp', 'src/mavros_avoidance_controller.cpp'])
    p.text(68, 181, '观测输入', 27, BLUE, True)
    p.text(425, 181, '并行判断', 27, BLUE, True)
    p.text(850, 181, '统一裁决与执行', 27, BLUE, True)
    p.mini(60, 248, 270, 130, '运动目标与位姿', '目标速度与自身运动')
    p.mini(410, 248, 345, 130, '动态风险', '预测会不会接近', TEAL)
    p.mini(60, 449, 270, 130, '当前扫描与位姿', '机身附近的障碍点')
    p.mini(410, 449, 345, 130, '静态风险', '判断机体还剩多少间隙')
    p.line([(332, 312), (402, 312)])
    p.line([(332, 514), (402, 514)])
    p.mini(848, 352, 285, 132, '风险融合', '等级、方向、风险来源', TEAL)
    p.line([(757, 312), (800, 312), (800, 388), (840, 388)])
    p.line([(757, 514), (800, 514), (800, 449), (840, 449)])
    p.mini(1212, 352, 328, 132, '安全控制', '跟踪／停车／有条件退让')
    p.line([(1136, 418), (1204, 418)])
    p.mini(848, 594, 285, 110, '规划或自主探索', '提供当前航点')
    p.line([(1136, 649), (1375, 649), (1375, 491)])
    p.text(86, 673, '长期地图负责路线参考，\n当前扫描负责即时保护。', 27, NAVY, width=660)
    pages.append(p.finish())

    p = Page(2, '动态风险原理', '动态风险：不只看现在近不近', '同时考虑运动方向、未来距离和高度差，提前判断行人横穿带来的威胁。',
             '正在靠近且预测会太近，才触发普通预警；已经太近可直接紧急。', ['src/collision_risk.cpp', 'config/dynamic_risk.yaml'])
    p.block(60, 205, 680, 342, '我们制定的判断规则', [
        '向前预测最多 1.50 秒，寻找这段时间内两者最接近的时刻。',
        '普通预警：正在接近，预测水平中心距离小于 1.35 米，预测高度差小于 0.80 米。',
        '正在远离且没有已有风险保持时，不因“看起来近”就新增普通预警。'], TEAL, 26)
    p.rect(795, 205, 745, 342, '#F6FAFC')
    p.text(822, 225, '规则示意：两条运动路线正在交叉', 27, BLUE, True)
    p.circle(917, 419, 30, '#E5F3F5', TEAL)
    p.text(917, 407, '机', 24, TEAL, True, center=True)
    p.line([(950, 419), (1255, 419)], BLUE, 3)
    p.circle(1146, 335, 27, '#FFF1D9', AMBER)
    p.text(1146, 323, '人', 24, AMBER, True, center=True)
    p.line([(1146, 366), (1146, 473)], BLUE, 3)
    p.circle(1146, 419, 57, 'none', RED, True)
    p.text(1255, 360, '预测接近区', 24, RED, True)
    p.text(828, 483, '先预测，再决定是否需要让行。', 25, NAVY)
    p.block(60, 578, 720, 160, '紧急入口一：已经太近', ['当前中心距离 < 0.75 米，且当前高度差 < 0.80 米。'], BLUE, 25)
    p.block(813, 578, 727, 160, '紧急入口二：即将快速逼近', ['0.80 秒内接近到 < 0.45 米，且预测高度差 < 0.80 米。'], TEAL, 25)
    pages.append(p.finish())

    p = flow_page(3, '动态风险流程', '动态风险的处理流程与防抖规则', '输入是已经确认的运动目标，不是把每个新出现的点都当成危险。', [
        ('1  接收目标与位姿', ['读取目标的位置和速度。', '估计无人机自身速度，并进行平滑。', '检查目标和位姿消息是否及时到达。']),
        ('2  预测接近程度', ['判断正在接近还是远离。', '计算未来最近距离、到达该时刻的时间和高度差。', '分别检查普通预警与两种紧急条件。']),
        ('3  选择主要威胁', ['危险程度与时间紧迫度共同决定优先级。', '尽量继续跟踪同一目标。', '新紧急目标可以打断原目标锁定。']),
        ('4  输出等级与建议', ['输出 0／1／2 级风险。', '建议向运动方向侧面让开，避免继续冲向交叉点。', '锁定方向、平滑建议速度，减少左右反复。'])],
        '目标锁定与解除保持，减少漏检或边界波动导致的反复变向。', ['src/collision_risk.cpp', 'config/dynamic_risk.yaml'],
        '退出区 1.60 米；短暂漏检保留 0.30 秒；解除保持 0.30 秒；紧急保持 0.25 秒。')
    pages.append(p.finish())

    p = Page(4, '静态距离与阈值', '静态风险：距离要先扣除机体尺寸', '以下以仓库配置说明，所有静态阈值都要区分“中心距离”和“外缘间隙”。',
             '仓库：中心到表面 0.70 米 ≈ 机体半径 0.40 米 + 剩余间隙 0.30 米。', ['src/static_collision_risk.cpp', 'config/static_risk.yaml', 'config/warehouse_navigation.yaml'])
    p.rect(60, 205, 790, 330, '#F6FAFC')
    p.circle(256, 358, 85, '#E6F2F4', TEAL)
    p.circle(256, 358, 5, TEAL, TEAL)
    p.text(256, 306, '机体圆形近似', 24, TEAL, True, center=True)
    p.rect(644, 263, 65, 184, '#DEEAF1', BLUE)
    p.text(677, 226, '障碍表面', 24, BLUE, True, center=True)
    p.line([(260, 411), (341, 411)], TEAL, 2)
    p.text(205, 460, '机体半径\n0.40 米', 24, TEAL, width=185)
    p.line([(345, 358), (638, 358)], BLUE, 2)
    p.text(486, 303, '外缘剩余间隙', 26, BLUE, True, center=True)
    p.text(486, 368, '预警阈值 0.30 米', 25, BLUE, center=True)
    p.block(900, 205, 640, 330, '为什么不能只看无人机中心', [
        '机心没有碰到墙，不代表旋翼有足够间隙。',
        '仓库机体半径设为 0.40 米；必须扣除后再判定风险。',
        '规划膨胀 0.80 米已包含机体，不要再重复加一次半径。'], TEAL, 26)
    for i, (title, body, color) in enumerate([
        ('二级：紧急', '中心到表面 < 0.65 米\n外缘间隙 < 0.25 米', RED),
        ('一级：预警', '中心到表面 < 0.70 米\n外缘间隙 < 0.30 米', AMBER),
        ('解除：留出缓冲', '中心到表面 ≥ 0.75 米\n外缘间隙 ≥ 0.35 米', GREEN)]):
        p.block(60+i*501, 575, 478, 163, title, [body], color, 25)
    pages.append(p.finish())

    p = flow_page(5, '静态风险流程', '静态风险：从当前扫描中找可靠障碍', '不直接用整张历史地图紧急刹车，而是检查机身附近的新扫描。', [
        ('1  筛选相关点', ['确认点云坐标和位姿。', '只保留相对机身高度 −0.35 到 +0.45 米的点。', '只检查水平 2.0 米内，去掉无效和过近点。']),
        ('2  避免重复判断', ['已确认运动目标由动态分支处理。', '静态分支屏蔽目标附近圆柱区域。', '短漏检保留掩膜，最长 0.60 秒。']),
        ('3  检查障碍支持', ['找到剩余点里的最近原始点。', '其 0.30 米邻域至少需要 4 个支持点。', '取支持点距离的中位数，降低孤立噪点影响。']),
        ('4  扣除半径并分级', ['从表面距离扣除机体半径，得到外缘间隙。', '按紧急、预警、解除缓冲确定等级。', '输出风险、净空和远离障碍的建议速度。'])],
        '静止人体仍可能是即时障碍；“静态风险”并不等于只判断永久建筑。', ['src/static_collision_risk.cpp', 'config/static_risk.yaml'],
        '支持不足会使测量无效，不等于环境安全；已有预警需到解除距离才退回零级。')
    pages.append(p.finish())

    p = Page(6, '风险融合规则', '风险融合：保留更严重的危险', '先统一风险等级，再按具体情形组合建议方向；并不是所有情况都按固定比例。',
             '危险等级不能被平均降低；融合建议也不能直接绕过控制器。', ['src/avoidance_fusion.cpp', 'config/avoidance.yaml'])
    p.mini(60, 205, 294, 140, '静态风险', '等级 + 建议速度')
    p.mini(60, 424, 294, 140, '动态风险', '等级 + 建议速度', TEAL)
    p.line([(357, 275), (399, 275), (399, 380), (434, 380)])
    p.line([(357, 494), (399, 494), (399, 420), (434, 420)])
    p.mini(443, 335, 300, 144, '等级取较高者', '任一二级，融合就是二级', TEAL)
    p.line([(746, 403), (784, 403)])
    rows = [
        ('一侧二级', '采用该侧的紧急建议方向'),
        ('两侧一级', '两个建议向量相加，再限制速度'),
        ('两侧二级', '仅此情形：静态 60% + 动态 40%'),
        ('双紧急几乎抵消', '组合速度 < 0.05 米/秒，改选静态方向'),
    ]
    for i, (label, rule) in enumerate(rows):
        y = 209+i*119
        p.rect(794, y, 746, 100, PALE)
        p.text(816, y+16, label, 26, TEAL if i%2 else BLUE, True)
        p.text(816, y+54, rule, 25, NAVY, width=710)
    p.rect(60, 642, 684, 95, '#F7FAFC')
    p.text(81, 658, '来源接收时效 0.40 秒；输出建议限速 0.80 米/秒。', 25, NAVY, width=638)
    p.text(815, 696, '最后输出：等级、建议速度、风险来源。', 26, BLUE, True)
    pages.append(p.finish())

    p = Page(7, '安全控制动作', '安全控制：普通演示与探索不同', '一级风险优先让行；二级风险怎样处理，必须结合当前演示配置解释。',
             '探索不盲目沿最近障碍斥力横移，避免躲开一侧货架却撞上另一侧。', ['src/mavros_avoidance_controller.cpp', 'config/astra_mavros_controller.yaml', 'config/warehouse_navigation.yaml', 'launch/warehouse_exploration.launch'])
    p.block(60, 210, 465, 440, '普通演示：跟踪与让行', [
        '零级：跟踪规划航点。',
        '一级：默认停止水平前进；纯静态预警可检查低速退让。',
        '二级：采用融合避让建议，再限制最终速度。',
        '普通场景导航上限 0.35 米/秒。'], BLUE, 26)
    p.block(560, 210, 465, 440, '仓库导航：更低速', [
        '同样保留一级停车让行。',
        '只在静态预警且退路检查通过时，慢速远离货架。',
        '导航上限 0.25 米/秒。',
        '静态退让 0.10 米/秒；不是向目标追赶。'], TEAL, 26)
    p.block(1060, 210, 480, 440, '自主探索：保守执行', [
        '零级：跟踪探索器航点，同时检查前方短段。',
        '一级：停车，只有授权短退路通过才恢复。',
        '二级：默认水平停车；当前正常来源下短退路会被拦住。',
        '导航上限 0.30 米/秒。'], BLUE, 26)
    p.text(75, 684, '共同规则：到位后仍检查风险；水平总限速 0.65 米/秒；不让两个控制器同时发飞控指令。', 27, NAVY, width=1450)
    pages.append(p.finish())

    p = Page(8, '控制器完整流程', '安全控制的完整执行流程', '先确认飞行状态与数据，再决定跟踪、让行或停止；风险检查不能被“已经到位”跳过。',
             '有效航点和新鲜数据只是执行前提，最终速度仍受风险规则约束。', ['src/mavros_avoidance_controller.cpp', 'config/astra_mavros_controller.yaml', 'launch/warehouse_exploration.launch'])
    steps = [('1  稳定初始化', '有效地面位姿\n重新确定起飞基准'), ('2  起飞确认', '预发指令、请求飞行\n实际离地并高度驻留'), ('3  导航数据检查', '航点、位姿、风险\n探索还需新鲜扫描'), ('4  风险动作裁决', '零级跟踪、一级让行\n二级按演示策略执行')]
    for i, (title, detail) in enumerate(steps):
        x=60+i*378
        p.mini(x, 229, 346, 151, title, detail, BLUE if i%2==0 else TEAL)
        if i<3:
            p.line([(x+349, 305), (x+375, 305)])
    p.line([(1378, 383), (1378, 480), (1135, 480)])
    p.mini(845, 422, 285, 128, '5  限速与发送', '高度／水平分别限幅', TEAL)
    p.line([(842, 485), (751, 485)])
    p.mini(430, 422, 315, 128, '6  下一周期', '继续追踪航点与检查风险')
    p.line([(430, 485), (40, 485), (40, 191), (988, 191), (988, 222)], dashed=True)
    p.text(70, 445, '下一周期继续检查', 22, MUTED)
    p.block(60, 591, 720, 145, '数据与命令的时效规则', ['位姿 0.35 秒；融合消息 0.45 秒；航点 1.50 秒。'], BLUE, 25)
    p.block(813, 591, 727, 145, '探索额外保护与到达容差', ['扫描 0.30 秒；中间点 0.03 米；终点 0.25 米。'], TEAL, 25)
    pages.append(p.finish())

    p = Page(9, '短退路与保护', '窄通道恢复：先有退路，再慢速执行', '短退路是特殊恢复措施，不是把正常规划安全距离降低。',
             '一级纯静态可以检查短退路；二级当前接口不一致，不能写成已能正常恢复。', ['src/mavros_avoidance_controller.cpp', 'scripts/warehouse_explorer.py', 'scripts/exploration_grid.py', 'config/warehouse_exploration.yaml', 'src/avoidance_fusion.cpp'])
    p.block(60, 207, 460, 402, '探索器：先确认有路', [
        '机体足迹必须在已观测空间内。',
        '净空沿恢复路线不继续减小，并返回正常安全区域。',
        '短命令最多 0.20 米；最低中心净空 0.45 米只用于恢复。'], BLUE, 26)
    p.block(570, 207, 460, 402, '控制器：重新检查', [
        '动态风险为零，授权与扫描都新鲜。',
        '恢复点与当前航点一致；再次检查短段净空。',
        '中心净空至少 0.45 米，恢复速度最多 0.08 米/秒。'], TEAL, 26)
    p.block(1080, 207, 460, 402, '3  低速执行或停车', [
        '通过：沿授权短退路慢速执行；不通过：水平停车。',
        '不沿单个障碍的斥力方向盲目横移。',
        '当前二级来源检查不一致，正常融合链路下短退路被拒绝。',
        '无安全方向或持续停滞，应明确记录部分任务失败。'], BLUE, 26)
    p.line([(523, 408), (563, 408)])
    p.line([(1033, 408), (1073, 408)])
    p.rect(60, 642, 1480, 95, '#FFF6E7', '#E8C892')
    p.text(84, 656, '正常通行依然保留 0.80 米规划净空。0.45 米只用于受约束恢复，不代表所有窄缝都能飞。\n零速度是控制指令，不意味着无人机物理上瞬间停住。', 25, NAVY, width=1410)
    pages.append(p.finish())

    p = Page(10, '规则小结与边界', '规则小结：安全依据不能省略', '我们实现的是可检查的工程保护层，不是零碰撞或所有工况有效的数学保证。',
             '零级风险不等于数据有效；可靠保护还需要感知时效、接口一致和真实刹停验证。', ['src/collision_risk.cpp', 'src/static_collision_risk.cpp', 'src/avoidance_fusion.cpp', 'src/mavros_avoidance_controller.cpp'])
    p.block(60, 211, 465, 485, '已经采用的机制', [
        '相对运动预测：提前发现行人接近。',
        '扣除机体半径：不只看机心距离。',
        '解除缓冲与方向保持：减少反复切换。',
        '规划与即时保护并行：路线可行仍要检查当前扫描。',
        '探索前向检查、限速与授权恢复。'], BLUE, 26)
    p.block(560, 211, 465, 485, '必须如实说明的限制', [
        '输入失效也可能输出零级，不能当作已经确认安全。',
        '二级短退路来源不一致，当前正常装配下会被拒绝。',
        '固定高度与圆形机体近似，不是完整三维碰撞模型。',
        '运动预测依赖短时速度稳定，漏检和突变会影响结果。'], TEAL, 26)
    p.block(1060, 211, 480, 485, '后续应验证的项目', [
        '各上游输入是否真的有效，而非仅聚合话题仍在发布。',
        '修正并联调二级恢复来源接口。',
        '测量真实刹停距离、感知延迟与旋翼净空。',
        '测试起飞、窄通道、漏检、遮挡和多目标情形。',
        '本轮只制作图片，以上不是新增实测。'], BLUE, 26)
    pages.append(p.finish())
    return pages


def verify_rules():
    dynamic=yaml.safe_load((B/'config/dynamic_risk.yaml').read_text())
    static=yaml.safe_load((B/'config/static_risk.yaml').read_text())
    warehouse=yaml.safe_load((B/'config/warehouse_navigation.yaml').read_text())
    fusion=yaml.safe_load((B/'config/avoidance.yaml').read_text())
    controller=yaml.safe_load((B/'config/astra_mavros_controller.yaml').read_text())
    expectations=[(dynamic,'prediction_horizon',1.5),(dynamic,'safe_radius',1.35),
                  (dynamic,'vertical_safe_distance',.8),(dynamic,'emergency_distance',.75),
                  (dynamic,'emergency_time_to_cpa',.8),(dynamic,'emergency_cpa_distance',.45),
                  (dynamic,'risk_exit_radius',1.6),(dynamic,'objects_timeout',.3),
                  (dynamic,'target_hold_time',.3),(dynamic,'risk_release_time',.3),
                  (dynamic,'emergency_hold_time',.25),
                  (static,'min_relative_z',-.35),(static,'max_relative_z',.45),
                  (static,'check_radius',2.),(static,'support_radius',.3),(static,'min_support_points',4),
                  (warehouse['static_collision_risk'],'uav_radius',.4),
                  (warehouse['static_collision_risk'],'safe_clearance',.3),
                  (warehouse['static_collision_risk'],'emergency_clearance',.25),
                  (warehouse['static_collision_risk'],'exit_clearance',.35),
                  (fusion,'static_emergency_weight',.6),(fusion,'dynamic_emergency_weight',.4),
                  (fusion,'source_timeout',.4),(controller,'risk_timeout',.45),
                  (controller,'odom_timeout',.35),(controller,'goal_tolerance',.25),
                  (controller,'waypoint_tolerance',.03),(controller,'publish_rate',30.)]
    for obj,key,value in expectations:
        assert math.isclose(obj[key],value), (key,obj[key],value)
    launch=(B/'launch/warehouse_exploration.launch').read_text()
    assert 'max_navigation_speed" value="0.30"' in launch
    assert 'hold_on_level2" value="true"' in launch
    assert 'require_fresh_scan" value="true"' in launch
    src=(B/'src/mavros_avoidance_controller.cpp').read_text()
    fus=(B/'src/avoidance_fusion.cpp').read_text()
    guard=src[src.index('bool guardedExplorerRecovery'):src.index('bool normalMotionClear')]
    assert 'avoidance_source_ != "static_avoid"' in guard
    assert 'source = "static_emergency"' in fus
    return len(expectations)+5


def main():
    before={str(p):sha(p) for p in FILES}
    checks=verify_rules()
    pages=generate()
    assert len(pages)==10
    assert all(t['in_page'] for t in TEXT_CHECKS), [t for t in TEXT_CHECKS if not t['in_page']]
    contact=Image.new('RGB',(1920,1540),'#E6EEF3')
    pen=ImageDraw.Draw(contact)
    ff=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',23)
    for i,page in enumerate(pages):
        im=Image.open(page['png']).convert('RGB')
        assert im.size==(3840,2160)
        im.thumbnail((920,518))
        # 缩略总览仅用于导航，不替代单页高清图。
        im.thumbnail((730,410))
        x=20+(i%3)*638; y=18+(i//3)*381
        im.thumbnail((618,348))
        contact.paste(im,(x,y))
        pen.text((x,y+350),f'{page["number"]:02d}  {page["name"]}',font=ff,fill=NAVY)
    contact.save(HERE/'00_全部图片预览.png')
    notes=['# 动态风险、静态风险与安全控制：图片与详细讲解','',
           '只交付图片和可编辑SVG，不生成PPT。参考文件：`/home/a/下载/fastlio2ppt.pptx`。',
           '图片白底、深蓝标题、蓝绿流程卡片；主图3840×2160、16:9。所有参数以本轮读取的配置和覆盖顺序为准。',
           '本轮无飞行源码修改、编译或仿真实测。所有示意是讲解，不是实验。','',
           '## 如何使用','',
           '- 直接按PNG目录的01到10顺序插入自己的PPT，或用图片浏览器全屏讲解。',
           '- SVG目录可用于后续改字、改节点和箭头；不需要本轮创建PPTX。',
           '- `风险与安全控制_高清图片合集.zip`包含10张高清PNG、10张SVG及详细讲解和来源核验。',
           '- 重生成：`QT_QPA_PLATFORM=offscreen python3 build_images.py`。','']
    for page,(title,body) in zip(pages,NOTES):
        notes += [f'## {page["number"]:02d}  {title}','',
                  f'[高清图片]({page["png"]}) · [可编辑流程图]({page["svg"]})','',body,'','源码与配置：','']
        for item in page['sources']:
            path=B/item
            notes.append(f'- [{item}]({path})')
        notes.append('')
    notes += ['## 本次新核对的实现边界','',
              '1. 二级恢复接口：控制器要求来源为`static_avoid`；融合器静态二级来源是`static_emergency`。双二级和动态二级同样不满足该检查。本轮不改代码，不能写“二级一定可以退让”。这是本轮代码静态复核发现，不是新SITL复现。',
              '2. 风险有效性：风险节点失效可能发布0，融合器过期来源亦降0；聚合话题继续发布时控制器的聚合新鲜度不足以确认各上游有效。探索新鲜扫描检查只补了一部分，目标输入和里程计仍有采集/接收时效差异。',
              '3. 起飞分支：起飞阶段先执行高度控制并返回，未进入导航阶段完整风险裁决；图中起飞确认是运行状态门控，不是全起飞阶段障碍保护证明。',
              '4. 历史记录的假飞控“二级恢复”检查不能自动证明正常融合器的来源组合也可通过，本材料以当前实际源码接线为准。',
              '5. 没有重新ROS编译、飞行、碰撞、动态检测精度、刹停距离或硬件实时性实验；图像检查只验证图文交付。','']
    (HERE/'详细讲解与源码对应.md').write_text('\n'.join(notes),encoding='utf-8')
    after={str(p):sha(p) for p in FILES}
    assert before==after,'source or reference changed while generating'
    (HERE/'内容与来源.json').write_text(json.dumps(dict(reference=str(REFERENCE),style=dict(
        background='white',title=NAVY,flow_blue=BLUE,flow_teal=TEAL,pale=PALE),
        sources_sha256=before,pages=pages),ensure_ascii=False,indent=2),encoding='utf-8')
    # XML解析、中文字体、文本几何、尺寸、源码参数、源哈希均在此完成检查。
    import xml.etree.ElementTree as ET
    for page in pages:
        ET.parse(page['svg'])
    report=dict(status='passed',pages=len(pages),png_size=[3840,2160],
                text_lines_checked=len(TEXT_CHECKS),parameter_and_interface_checks=checks,
                source_files_unchanged=len(FILES),svg_xml_valid=True,
                reference_sha256=before[str(REFERENCE)],pptx_created=False,
                validation_scope='来源/配置/图像格式/文字边界；非ROS编译或飞行安全实验',
                visual_review='待人工查看图片后另行记录')
    (HERE/'生成核验.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    with ZipFile(HERE/'风险与安全控制_高清图片合集.zip','w',ZIP_DEFLATED) as z:
        for sub in ['PNG','SVG']:
            for f in sorted((HERE/sub).iterdir()):
                z.write(f,f.relative_to(HERE))
        for name in ['00_全部图片预览.png','详细讲解与源码对应.md','内容与来源.json','生成核验.json','build_images.py']:
            z.write(HERE/name,name)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print('输出目录：'+str(HERE))


if __name__=='__main__':
    main()
