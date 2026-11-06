#!/usr/bin/env python3
"""Fill the user's original Word table, retaining page/table/paragraph formats."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED
import hashlib,json
from lxml import etree as E

ROOT=Path('/home/a/AstraDroneOpen')
BASE=ROOT/'docs/reports/weekly_20261031'
TEMPLATE=Path('/home/a/下载/2026.10.24-张志华-周报.docx')
OUTPUT=Path('/home/a/下载/2026.10.31-张志华-周报.docx')
if OUTPUT.exists():
    previous=BASE/'manifest.json'
    if not previous.exists() or hashlib.sha256(OUTPUT.read_bytes()).hexdigest()!=json.loads(previous.read_text())['sha256']:
        raise SystemExit('Output was externally edited; preserve it and choose a new filename')
W='http://schemas.openxmlformats.org/wordprocessingml/2006/main'
NS={'w':W}
q=lambda name:'{'+W+'}'+name
with ZipFile(TEMPLATE) as archive:
    originals={x.filename:archive.read(x.filename) for x in archive.infolist()}
doc=E.fromstring(originals['word/document.xml'])
body=doc.find('w:body',NS);table=body.find('w:tbl',NS);rows=table.findall('w:tr',NS)
cells=lambda row:rows[row].findall('w:tc',NS)
text_of=lambda p:''.join(p.xpath('.//w:t/text()',namespaces=NS))
heading_prototype=deepcopy(cells(1)[2].findall('w:p',NS)[0])
body_prototype=deepcopy(cells(1)[2].findall('w:p',NS)[1])
list_prototype=deepcopy(cells(0)[2].findall('w:p',NS)[0])

def para(text,prototype):
    p=E.Element(q('p'))
    pp=prototype.find('w:pPr',NS)
    if pp is not None:p.append(deepcopy(pp))
    if prototype is heading_prototype:
        pp=p.find('w:pPr',NS)
        if pp is None:pp=E.SubElement(p,q('pPr'))
        if pp.find('w:keepNext',NS) is None:
            pp.insert(1 if len(pp) and pp[0].tag==q('pStyle') else 0,E.Element(q('keepNext')))
    r=E.SubElement(p,q('r'));rp=prototype.find('w:r/w:rPr',NS)
    if rp is not None:r.append(deepcopy(rp))
    t=E.SubElement(r,q('t'));t.set('{http://www.w3.org/XML/1998/namespace}space','preserve');t.text=text
    return p

def fill(cell,items):
    for old in list(cell):
        if old.tag!=q('tcPr'):cell.remove(old)
    for text,kind in items:
        cell.append(para(text,heading_prototype if kind=='heading' else list_prototype if kind=='list' else body_prototype))

# Carry forward last week's actual next-week plan, without rewriting history.
previous_plan=[text_of(p) for p in cells(4)[1].findall('w:p',NS)]
fill(cells(0)[2],[(x,'list') for x in previous_plan])
completed=[
('1. 静态表面保护、未决点标色与缺测诊断',
 '针对静态货架误判，增加成熟垂直面及多帧支持筛选，仅暂停复查、不删占据。同一4508帧对照中，地图107720点及哈希不变，可行动候选3→1。导出红、紫红、橙色未决标记，分别检查货架背面与后方地面缺测。'),
('2. 探索路线、外走廊识别和降落改进',
 '加入重复/反向路段费用、未知收益和区域反馈，按实测证据抑制无效补扫；识别转角入口连续通过外走廊。三轮任务381/482/416秒，中位416秒，旧版470秒，仍有变慢负例。后续入口航次路线133.59米，但部分地面支持下降。一次AUTO.LAND慢降，三轮无超过25厘米的触地反弹。'),
('3. 双行人动态场景与主动复查评分',
 '新增右侧行人运动180秒后停止的场景，定位“已检测但未选中复查”的调度缺口。仅将活动鬼影复查效用乘1.3，完整实验中实际复查由0增至4次、其中2次有效；主体/近地轨迹残留代理由442/15降至0/2。同期地面支持率由97.41%降至96.70%，保留覆盖代价，未证明全面提升或逐点清除准确率。'),
('4. DUFOMap原生接入与同源双图维护',
 '接入作者DUFOMap 1.1.1，以同批注册点云、位姿和逐帧回执维护15厘米静态背景图；原地图继续承担原占据及历史证据。新增共同截止、坐标/图版本检查和联合可视化。10月30日17:50航次两图各处理5022帧、102检查点，509秒返航落地；原图111952点、DUFO图126359点。DUFO仍可能排除停止物体，非零回环后的历史重建尚未接入。'),
('5. 50平方米新室内跨场景验证',
 '基于GitHub家具生成50.4平方米办公室，布置12件静态物体和运动60秒的行人，导航参数保持原样。最终886帧、19共同检查点通过，96秒返航落地，现实整流程约4分02秒，已知自由面积44.04平方米。修复同帧重复分类引起的发布/保存不一致。原图仍有54个疑似体素；近家5米零分规则覆盖小屋大部，主动复查为0秒，最近人机水平中心距0.74米，余量偏紧。'),
('6. R3LIVE激光—单目—IMU彩色建图演示',
 '独立适配作者R3LIVE，完成相机内外参、光学坐标方向、仿真点时间和外部定位反馈接口。完整SITL航次接收820条LIO、819条相机里程计，817次真实视觉更新，正常降落上锁；原生RGB地图42475点、3468种颜色。只使用单目RGB，不用深度图。该演示采用预设飞行路线，尚未与自主探索/去动态双图合并，也未完成网格、3DGS或真机验证。')]
fill(cells(1)[2],[(x,k) for title,content in completed for x,k in [(title,'heading'),(content,'body')]])
learning=[
('1. DUFOMap与Dynablox的动态处理机制','heading'),
('阅读DUFOMap: Efficient Dynamic Awareness Mapping（IEEE RA-L，2024）及Dynablox: Real-time Detection of Diverse Dynamic Objects in Complex Environments（IEEE RA-L，2023）的主要方法与作者接口。前者依据历史可靠自由区域清理背景，后者利用TSDF时序证据识别动态物体。明确“静态背景干净”与“保留当前停止障碍”是不同目标，双图本身属于工程集成，尚不能作为原创贡献。','body'),
('论文：https://arxiv.org/abs/2403.01449；https://arxiv.org/abs/2304.10049','body'),
('2. 单目＋MID360适配和相关开源工作调研','heading'),
('梳理BTSA、M-detector、FreeDOM、Dynamic-LIO等方法的输入、动态处理层次及开源范围；区分已核对源码与本机实测。分析SLAM2的注册深度/mask要求：单目雷达可提供稀疏深度，但必须正确时空对齐，不能直接代替稠密RGB-D输入。学习R3LIVE（ICRA 2022）的LIO/VIO分工、标定和点云着色，已完成独立工程验证。','body'),
('源码：https://github.com/hku-mars/r3live；https://github.com/thisparticle/btsa','body'),
('3. 3DGS数据准备路线','heading'),
('初步梳理从R3LIVE导出图像、内参、完整相机位姿及XYZRGB初始化点，再进行离线3DGS训练的流程。发现现有相机里程计发布时刻不等于曝光时刻，已有轨迹文件还缺旋转；需新增测量时间配对和完整位姿导出。尚未训练3DGS，动态RGB还需要掩膜/观测筛选。','body')]
fill(cells(2)[2],learning)
others=[
 '整理地图、轨迹、分区域覆盖、动态残留代理与失败样本，保留录包和源码/参数指纹。经授权清理两个demo历史大包，释放约154.38GB；精简记录在同一30秒片段由698.29MB降至116.15MB，在线传感与控制频率保持。compact包省略原始雷达/IMU，不能用于完整前端重算。',
 '10月30日主项目通过368项回归及构建；独立R3LIVE另有3项输入/坐标回归通过，不将两工作空间检查简单当作飞行次数。多轮结果仍为部分建图；非零回环共同重建、逐点动态真值、机载资源/真实电池与实飞未完成。'
]
fill(cells(3)[2],[(x,'body') for x in others])
next_plan=[
 '1. 针对小场景5米近家零分覆盖过大的问题，研究按安全返航成本与实际观测收益安排复查，增加有界执行机会；保留当前障碍，优先改善人机避让余量。',
 '2. 补充移动后停止、遮挡与薄结构的同输入/同预算多轮对照，分别报告残留、静态误删代理、最差表面覆盖、有效复查比例和安全距离。',
 '3. 完成R3LIVE图像测量时刻、完整光学相机位姿和标定数据导出，先在静态小场景验证COLMAP格式，再启动离线3DGS与留出视角评价，不把PCD改格式等同训练。',
 '4. 推进DUFO非零回环历史一致性方案及与彩色建图的隔离接口；继续核实MID360、Orin NX 16GB、相机/系统/飞控与真实逐点时间，实机验证待硬件具备后开展。'
]
fill(cells(4)[1],[(x,'list') for x in next_plan])
header=body.findall('w:p',NS)[2]
replacement=para('汇报时间：2026.10.31（统计周期：10.26—10.31，周一至周六）',header)
body.replace(header,replacement)
originals['word/document.xml']=E.tostring(doc,xml_declaration=True,encoding='UTF-8',standalone=True)
# Clear stale application page/word counters; the PDF conversion supplies pagination.
if 'docProps/core.xml' in originals:
    core=E.fromstring(originals['docProps/core.xml'])
    for item in core:
        if E.QName(item).localname in ('created','modified'):
            item.text='2026-10-31T00:00:00Z'
    originals['docProps/core.xml']=E.tostring(core,xml_declaration=True,encoding='UTF-8',standalone=True)
with ZipFile(OUTPUT,'w',ZIP_DEFLATED) as z:
    for name,data in originals.items():z.writestr(name,data)
plain=['课题组周报','汇报人：张志华','汇报时间：2026.10.31','统计周期：2026.10.26—2026.10.31（周一至周六）','', '上周工作计划',*previous_plan,'','本周完成情况']
for h,t in completed:plain.extend([h,t])
plain.extend(['','学习情况',*[t for t,k in learning],'','其他工作进度',*others,'','下周工作计划',*next_plan])
(BASE/'周报正文.txt').write_text('\n\n'.join(plain)+'\n')
manifest=dict(template=str(TEMPLATE),template_sha256=hashlib.sha256(TEMPLATE.read_bytes()).hexdigest(),
 output=str(OUTPUT),sha256=hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),report_date='2026-10-31',
 period=['2026-10-26','2026-10-31'],xml_text_characters=len(''.join(doc.xpath('//w:t/text()',namespaces=NS))),
 preserved=['landscape A4','table grids/borders/merged cells','paragraph styles','font inheritance','five report sections'])
(BASE/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(manifest,ensure_ascii=False,indent=2))
