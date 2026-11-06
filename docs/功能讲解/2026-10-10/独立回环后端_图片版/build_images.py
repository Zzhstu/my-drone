#!/usr/bin/env python3
"""源码对应的四页图片；不生成 PPT，不启动 ROS，不修改算法。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import base64
import hashlib
import json
import math
import shutil
import sys
from pathlib import Path
from xml.sax.saxutils import escape
from zipfile import ZipFile, ZIP_DEFLATED
import yaml
from PIL import Image, ImageDraw, ImageFont
from PyQt5.QtCore import QByteArray, QRectF
from PyQt5.QtGui import QFont, QFontMetricsF, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer
from PyQt5.QtWidgets import QApplication

APP = QApplication.instance() or QApplication([])
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
B = ROOT / 'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
RESULT = ROOT / 'loop_closure_runs/loop_closure_20260923_182256'
REFERENCE = Path('/home/a/下载/fastlio2ppt.pptx')
NAVY = '#15334C'; BLUE = '#24618A'; GRAY = '#657585'
TEAL = '#137B7E'; RED = '#CB5263'; GOLD = '#D49B45'
COLORS = [('#EFF0FF','#B7C2EB'),('#FFF3E5','#E9C291'),
          ('#EDF8F0','#A6D2BD'),('#FFF0F5','#DFB3C9'),
          ('#EDF5FF','#A8C8E7'),('#F3EFFC','#C4B5E0')]
TEXTS = []; PAGES = []

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def font(size, bold=False):
    f = QFont('Noto Sans CJK SC'); f.setPixelSize(round(size)); f.setBold(bold)
    return f

class Page:
    def __init__(self, number, name, title, subtitle, footer):
        self.number=number; self.name=name; self.arrows=0; self.art=0
        self.parts=['<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="1920" height="1080" viewBox="0 0 1920 1080">']
        self.rect(0,0,1920,1080,'white','none')
        self.text(66,48,title,47,bold=True)
        self.text(68,119,subtitle,25,color=GRAY)
        self.line([(65,178),(1855,178)],'#DCE5EB',1,False)
        self.text(65,1028,footer,21,color=TEAL,bold=True,width=1640)
        self.text(1768,1030,'%02d / 04'%number,20,color=GRAY)

    def rect(self,x,y,w,h,fill='none',stroke=BLUE,sw=1.5,dashed=False):
        dash=' stroke-dasharray="8 6"' if dashed else ''
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{dash}/>')

    def poly(self,pts,fill,stroke='none',sw=1):
        pts=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        self.parts.append(f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def circle(self,x,y,r,fill,stroke='none',sw=1):
        self.parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def line(self,pts,color=BLUE,sw=2,arrow=True,dashed=False):
        coords=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        dash=' stroke-dasharray="7 5"' if dashed else ''
        self.parts.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{sw}"{dash}/>')
        if arrow:
            x,y=pts[-1]; a,b=pts[-2]; d=math.hypot(x-a,y-b)
            if d:
                ux,uy=(x-a)/d,(y-b)/d; k=max(8,3.5*sw)
                self.poly([(x,y),(x-k*ux-k*.4*uy,y-k*uy+k*.4*ux),
                           (x-k*ux+k*.4*uy,y-k*uy-k*.4*ux)],color)
                self.arrows+=1

    def text(self,x,y,t,size=23,color=NAVY,bold=False,width=None,center=False):
        fm=QFontMetricsF(font(size,bold)); lines=[]
        for para in t.split('\n'):
            line=''
            for ch in para:
                if width and line and fm.horizontalAdvance(line+ch)>width:
                    lines.append(line);line=ch
                else:line+=ch
            lines.append(line)
        for i,txt in enumerate(lines):
            yy=y+i*size*1.34; tw=fm.horizontalAdvance(txt); left=x-tw/2 if center else x
            TEXTS.append(dict(page=self.number,text=txt,x=left,y=yy,width=tw,height=size))
            anchor='middle' if center else 'start'
            self.parts.append(f'<text x="{x}" y="{yy+size}" font-family="Noto Sans CJK SC,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{anchor}">{escape(txt)}</text>')
        return len(lines)*size*1.34

    def card(self,x,y,w,h,title,body,color):
        fill,stroke=COLORS[color%6];self.rect(x,y,w,h,fill,stroke,2)
        th=self.text(x+20,y+15,title,27,bold=True,width=w-40)
        bh=self.text(x+20,y+24+th,body,21,width=w-40)
        return y+24+th+bh

    def image(self,path,x,y,w,h):
        encoded=base64.b64encode(path.read_bytes()).decode('ascii')
        self.parts.append(f'<image x="{x}" y="{y}" width="{w}" height="{h}" preserveAspectRatio="xMidYMid meet" xlink:href="data:image/png;base64,{encoded}"/>')

    def save(self):
        self.parts.append('</svg>'); raw='\n'.join(self.parts).encode('utf-8')
        for folder in ('SVG','PNG'):(HERE/folder).mkdir(exist_ok=True)
        name='%02d_%s'%(self.number,self.name)
        (HERE/'SVG'/f'{name}.svg').write_bytes(raw)
        renderer=QSvgRenderer(QByteArray(raw));assert renderer.isValid()
        im=QImage(3840,2160,QImage.Format_ARGB32);im.fill(0xFFFFFFFF)
        painter=QPainter(im);painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)
        renderer.render(painter,QRectF(0,0,3840,2160));painter.end()
        assert im.save(str(HERE/'PNG'/f'{name}.png'))
        PAGES.append(dict(number=self.number,name=self.name,arrows=self.arrows,illustrations=self.art))

def drone(p,x,y):
    for dx,dy in [(-23,-9),(23,-9),(-23,9),(23,9)]:
        p.line([(x,y),(x+dx,y+dy)],NAVY,3,False)
        p.circle(x+dx,y+dy,9,'#EAF4FC',BLUE)
    p.rect(x-10,y-7,20,14,BLUE,NAVY)
    p.circle(x,y,4,'#96DECE')

def graph(p,x,y,w,h,loop=True,corrected=False,labels=True):
    p.art+=1
    coordinates=[(.1,.7),(.14,.23),(.47,.12),(.85,.24),(.9,.71),(.54,.86),(.14 if corrected else .3,.75)]
    pts=[(x+a*w,y+b*h) for a,b in coordinates]
    p.line(pts,TEAL if corrected else BLUE,2.5,False)
    if loop:p.line([pts[0],pts[-1]],RED,3,False,True)
    for i,(a,b) in enumerate(pts):
        p.circle(a,b,7,TEAL if corrected else '#79AAD0',NAVY)
        if labels:p.text(a+10,b-12,str(i),16,color=GRAY)
    drone(p,pts[-1][0],pts[-1][1]-25)
    if labels:
        p.text(x+w*.5,y+h*.33,'关键帧',19,color=BLUE)
        p.text(x+w*.5,y+h*.53,'回访连接' if loop else '相邻位移',19,color=RED if loop else BLUE)

def frames(p,x,y,w,h):
    p.art+=1
    for i in range(4):
        xx=x+i*w*.2; yy=y+(3-i)*12
        p.rect(xx,yy,w*.29,h*.66,'#F8FBFE','#A6C2D7')
        for j in range(55):
            u=(j*37%97)/97;v=(j*29%89)/89
            p.circle(xx+10+u*w*.22,yy+9+v*h*.49,1.8,BLUE if i<3 else TEAL)
        p.text(xx+12,yy+h*.69,f'第{i+1}帧',17,color=GRAY)

def descriptor(p,x,y,w,h):
    p.art+=1;cx=x+w*.22;cy=y+h*.42;r=min(w*.21,h*.39)
    for k in (1,2,3):p.circle(cx,cy,r*k/3,'none','#96B7C8')
    for k in range(12):
        a=k*math.pi/6;p.line([(cx,cy),(cx+r*math.cos(a),cy+r*math.sin(a))],'#B4CBD6',1,False)
    for k in range(90):
        a=k*.77;rr=r*(.2+((k*23)%79)/100)
        p.circle(cx+rr*math.cos(a),cy+rr*math.sin(a),2,[BLUE,TEAL,GOLD][k%3])
    p.line([(x+w*.45,cy),(x+w*.56,cy)],BLUE,2)
    heat=['#EDF5FF','#BFD8E7','#78ADB5','#388B8A','#DFC077']
    for row in range(7):
        for col in range(12):
            p.rect(x+w*.6+col*w*.027,y+row*h*.095,w*.027-1,h*.095-1,heat[(row*3+col*7)%5],'none')
    p.text(cx,y+h*.83,'分环、分方向',18,color=GRAY,center=True)
    p.text(x+w*.76,y+h*.83,'局部高度摘要',18,color=GRAY,center=True)

def clouds(p,x,y,w,h,aligned=False):
    p.art+=1
    for j in range(110):
        a=(j%22)*w*.033;b=(j//22)*h*.11
        bend=math.sin(j%22*.24)*h*.17
        u=x+w*.06+a; v=y+h*.15+b+bend
        p.circle(u,v,2,BLUE)
        p.circle(u+(2 if aligned else w*.17),v+(1 if aligned else -h*.12),2,TEAL)
    p.circle(x+w*.08,y+h*.86,4,BLUE);p.text(x+w*.11,y+h*.8,'历史',18,color=GRAY)
    p.circle(x+w*.51,y+h*.86,4,TEAL);p.text(x+w*.54,y+h*.8,'当前',18,color=GRAY)

def gate(p,x,y,w,h):
    p.art+=1
    p.poly([(x+w*.45,y),(x+w*.78,y+h*.35),(x+w*.45,y+h*.7),(x+w*.12,y+h*.35)],'#FFF8E8',GOLD,2)
    p.text(x+w*.45,y+h*.19,'检查',23,bold=True,center=True)
    p.line([(x+w*.12,y+h*.35),(x,y+h*.35),(x,y+h*.86)],RED,2)
    p.line([(x+w*.78,y+h*.35),(x+w,y+h*.35),(x+w,y+h*.86)],TEAL,2)
    p.text(x,y+h*.88,'拒绝',18,color=RED,center=True)
    p.text(x+w,y+h*.88,'接纳',18,color=TEAL,center=True)

def map_scene(p,x,y,w,h):
    p.art+=1
    for rack in range(3):
        ox=x+rack*w*.28;oy=y+(rack%2)*h*.15
        for j in range(150):
            a=j%25; b=j//25
            if b in (0,2,5) or a in (0,24) or a%6<3:
                p.circle(ox+a*w*.008,oy+b*h*.075,1.7,[BLUE,TEAL,GOLD][b%3])
    p.line([(x,y+h*.82),(x+w*.7,y+h*.82),(x+w*.82,y+h*.53)],TEAL,3,False)
    drone(p,x+w*.82,y+h*.53)
    p.text(x+w*.45,y+h*.88,'按修正位置重新拼接',18,color=GRAY,center=True)

def architecture():
    p=Page(1,'独立回环后端_分区架构','独立回环后端：全局校正，连续控制分流',
           '利用重访地点约束历史轨迹；优化结果单独输出，不直接接管无人机飞行。',
           '当前主演示默认关闭回环；图为工程结构示意，不代表已证明精度提升。')
    for x,w,title in [(65,360,'输入与连续定位'),(485,950,'独立回环处理'),(1495,360,'结果输出')]:
        p.rect(x,245,w,650,'none',BLUE,2,True)
        p.rect(x+26,229,w-52,43,'white','none')
        p.text(x+w/2,232,title,29,bold=True,center=True)
    p.card(95,307,300,180,'连续位姿','无人机的位置与姿态\n保持原始定位链路',4)
    graph(p,114,430,246,44,False,labels=False)
    p.card(95,578,300,245,'当前注册点云','与位姿配对\n保存为机体局部点云',2)
    clouds(p,114,693,248,100,True)
    p.rect(515,295,890,300,'#FFF1BF','#9AAEBB',2,True)
    p.text(541,304,'寻找重访并验证几何',29,bold=True)
    p.card(540,368,235,169,'关键帧记忆','位移或转角触发\n连接相邻帧位移',0)
    p.card(819,368,235,169,'重访候选','时距与高度筛选\n多帧摘要再核验',1)
    p.card(1098,368,279,169,'点云对齐与门控','检查匹配质量与覆盖\n限制修正幅度',3)
    p.line([(395,393),(540,393)],BLUE,2)
    p.line([(395,675),(465,675),(465,522),(540,522)],BLUE,2)
    p.line([(775,445),(819,445)],BLUE,2)
    p.line([(1054,445),(1098,445)],BLUE,2)
    p.rect(515,635,890,230,'#E7F1E0','#9AAEBB',2,True)
    p.text(541,646,'历史轨迹修正与地图重建',29,bold=True)
    p.card(540,710,390,128,'位姿图优化','固定第一帧；相邻约束＋回访约束\n降低异常约束影响',2)
    p.card(1000,710,377,128,'地图重新拼接','每帧局部点云使用修正位置\n重新合并并降采样',2)
    p.line([(1238,537),(1238,613),(732,613),(732,710)],TEAL,2)
    p.line([(930,777),(1000,777)],TEAL,2)
    p.card(1525,307,300,229,'轨迹与回环连接','原始/修正轨迹并列\n输出连接线及诊断',4)
    graph(p,1547,442,243,62,True,labels=False)
    p.card(1525,635,300,229,'重建地图','单独的回环地图话题\n不覆盖导航维护地图',2)
    map_scene(p,1552,760,240,61)
    p.line([(930,747),(963,747),(963,620),(1457,620),(1457,443),(1525,443)],TEAL,2)
    p.line([(1377,781),(1525,781)],TEAL,2)
    p.rect(95,927,1730,63,'#EFF5F8','#A6C2D4')
    p.line([(395,455),(440,455),(440,912),(345,912),(345,927)],BLUE,2)
    p.text(124,942,'原始连续位姿 → 原有规划与安全控制；回环修正不回写原定位内部地图，也不直接用于飞控。',25,bold=True,width=1660)
    p.save()

def workflow():
    p=Page(2,'独立回环后端_七步图文流程','独立回环后端：从关键帧到修正地图',
           '参考七步循环图：上排从左向右，下排从右向左；每个环节配工程图示。',
           '示意图不是实验结果；循环由新关键帧触发，检测与地图发布采用独立定时任务。')
    upper=[(95,'1  保存关键帧','位姿与点云配对；移动1米\n或转动10度，保留一帧。',0),
           (540,'2  筛选历史地点','相隔20秒、25帧；距离8米内\n且高度差不超过1米。',1),
           (985,'3  比较环境摘要','最多聚合4帧；按方向和距离\n记录高度，并比较相似程度。',2),
           (1430,'4  对齐当前与历史点云','当前帧对历史小地图；\n尝试初始位置与不同朝向。',3)]
    for x,title,body,c in upper:p.card(x,254,395,320,title,body,c)
    frames(p,115,388,340,153)
    graph(p,560,391,345,158,True)
    descriptor(p,1005,393,340,154)
    clouds(p,1450,399,334,148,False)
    for x in (490,935,1380):p.line([(x,415),(x+50,415)],BLUE,2)
    lower=[(1295,'5  多道检查后接纳','匹配收敛、评分合格、重叠足够；\n位置或角度修正过大则拒绝。',4),
           (695,'6  修正整条关键帧轨迹','加入回访连接；固定第一帧，\n联合调整历史位置与姿态。',5),
           (95,'7  重建并发布结果','各帧点云按修正位置重新拼接；\n轨迹、地图与诊断单独输出。',2)]
    for x,title,body,c in lower:p.card(x,665,530,309,title,body,c)
    gate(p,1452,815,220,106)
    graph(p,765,803,360,135,True,True)
    map_scene(p,163,811,390,132)
    p.line([(1627,574),(1627,616),(1560,616),(1560,665)],BLUE,2)
    p.line([(1295,826),(1225,826)],BLUE,2)
    p.line([(695,826),(625,826)],BLUE,2)
    p.line([(95,837),(51,837),(51,211),(293,211),(293,254)],BLUE,2)
    p.text(348,197,'处理新的关键帧',23,color=BLUE)
    p.text(1194,592,'不合格：不加回环，等待后续帧',20,color=RED,width=550)
    p.save()

def rules():
    p=Page(3,'独立回环后端_接纳与保护规则','我们制定的规则：不是“看着像”就连接',
           '以下是当前默认参数，不是安全保证；重复结构仍可能通过检查，历史实验已出现误回环。',
           '回环修正的是位置关系，不自动去除人体鬼影；现有规则仍需更严格的重复结构验证。')
    specs=[(65,235,'01  关键帧与时间隔离','首帧初始化；之后移动1米或转10度存帧。\n点数至少250；回访须隔20秒和25帧。',0,'frames'),
           (675,235,'02  空间与高度先验','在8米范围内寻找历史地点，高差≤1米。\n最多保留15个空间候选，不强行连接近邻。',1,'graph'),
           (1285,235,'03  相似性分级放行','摘要分数≤0.22进入点云验证；\n距离≤2米时，≤0.55可进入几何兜底。',2,'descriptor'),
           (65,625,'04  点云验证与硬门限','对齐须收敛；评分≤0.30、重叠≥35%。\n位置修正≤4米、角度修正≤35度。',3,'clouds'),
           (675,625,'05  图优化与失败撤销','固定首帧；异常约束降权、限制每步改动。\n数值求解失败，移除本次新增回访连接。',4,'gate'),
           (1285,625,'06  限频、冷却与输出隔离','每2秒检查；接纳后冷却10个关键帧。\n地图定时检查5秒一次；不接管飞控。',5,'map')]
    for x,y,title,body,c,kind in specs:
        p.card(x,y,570,346,title,body,c)
        if kind=='frames':frames(p,x+60,y+158,435,144)
        elif kind=='graph':graph(p,x+90,y+151,375,155,True)
        elif kind=='descriptor':descriptor(p,x+55,y+167,445,142)
        elif kind=='clouds':clouds(p,x+80,y+166,405,133,True)
        elif kind=='gate':gate(p,x+175,y+176,220,102)
        else:map_scene(p,x+70,y+166,423,122)
        if kind=='clouds':p.text(x+38,y+308,'评分不是距离单位；低评分也不等于同一地点。',18,color=RED)
        if kind=='descriptor':p.text(x+38,y+308,'兜底仅允许尝试对齐，不直接认定回环。',18,color=TEAL)
    p.save()

def evidence():
    p=Page(4,'独立回环后端_历史真实结果','历史保存结果：能产生回环，不等于精度提高',
           '2026年9月23日闭环专用场景；沿用当时保存的原始轨迹图与评测报告，未重新运行实验。',
           '结论：保留独立后端作为研究模块；当前主演示默认关闭，不能宣称回环已改善精度。')
    p.rect(65,220,935,766,'white','#B7CCD9')
    p.image(Path(str(RESULT)+'_trajectory.png'),87,230,891,692)
    p.text(94,931,'黑色虚线：仿真真值   橙色：原始定位   绿色：修正定位',23,width=865)
    p.text(1040,232,'同一航次的在线位姿评测',30,bold=True)
    p.text(1040,284,'177.1秒 · 真值航程51.10米 · 58个关键帧',23,color=GRAY)
    metrics=[('整段三维位置误差','1.13 厘米 → 4.78 厘米','刚体对齐后的均方根误差；越小越好。'),
             ('5秒间隔的相对位移误差','0.97 厘米 → 2.89 厘米','相邻时间段的运动一致性也变差。'),
             ('接受回环 / 数值优化失败','2 条 / 0 次','数值求解成功，仍不能证明回访连接正确。')]
    for i,(title,value,desc) in enumerate(metrics):
        y=343+i*153;p.rect(1040,y,815,130,COLORS[i][0],COLORS[i][1],1.8)
        p.text(1064,y+12,title,24,bold=True)
        p.text(1064,y+44,value,32,color=RED if i<2 else TEAL,bold=True)
        p.text(1064,y+91,desc,21,color=GRAY,width=770)
    p.rect(1040,818,815,166,'#FFF3E5','#E9C291',1.8)
    p.text(1064,831,'为什么不能只看左侧轨迹图？',26,bold=True)
    p.text(1064,875,'俯视轨迹重合会掩盖高度方向的误差。\n这里评测在线输出，不是回环后最终整条轨迹；\n原报告与原图已附在“证据原件”目录。',22,width=760)
    p.save()

def source_facts():
    cfg=yaml.safe_load((B/'config/loop_closure.yaml').read_text())
    code=(B/'src/loop_closure_backend.cpp').read_text()
    values={'keyframe_distance':1.,'keyframe_angle_deg':10.,'min_keyframe_points':250,
            'min_loop_time':20.,'min_loop_keyframe_gap':25,'loop_search_radius':8.,
            'max_loop_vertical_separation':1.,'max_spatial_candidates':15,
            'descriptor_submap_keyframes':4,'scan_context_threshold':.22,
            'scan_context_geometric_threshold':.55,'geometric_fallback_radius':2.,
            'max_icp_candidates':5,'submap_neighbors':5,'icp_fitness_threshold':.30,
            'icp_min_overlap':.35,'max_loop_translation_correction':4.,
            'max_loop_rotation_correction_deg':35.,'loop_cooldown_keyframes':10,
            'loop_detection_rate':.5,'map_publish_rate':.2,'optimizer_iterations':8}
    for k,v in values.items():assert cfg[k]==v,(k,cfg[k],v)
    for s in ['Eigen::SimplicialLDLT','optimizer_huber_delta_ / norm',
              'edges_.pop_back();','poses[index] * perturbation(delta)',
              'keyframe.optimized_pose.matrix().cast<float>()','map_to_raw_ * raw_pose']:
        assert s in code,s
    launch=(B/'launch/loop_closure.launch').read_text()
    assert '<arg name="enabled" default="false"/>' in launch
    assert 'enable_loop_closure:=false' in (B/'scripts/run_dynamic_comparison.py').read_text()
    assert 'name="enable_loop_closure" value="false"' in (B/'launch/warehouse_exploration.launch').read_text()
    report=Path(str(RESULT)+'_report.txt').read_text()
    for s in ['0.0113 / 0.0081 / 0.0391','0.0478 / 0.0401 / 0.1204',
              '0.0097 m','0.0289 m','accepted loop constraints: 2','optimizer_failures: 0']:
        assert s in report,s
    sources=[B/'src/loop_closure_backend.cpp',B/'config/loop_closure.yaml',
             B/'launch/loop_closure.launch',B/'launch/warehouse_exploration.launch',
             B/'scripts/run_dynamic_comparison.py',B/'scripts/analyze_loop_closure_bag.py',
             Path(str(RESULT)+'_trajectory.png'),Path(str(RESULT)+'_report.txt'),REFERENCE]
    protected={str(p):sha(p) for p in sources}
    for folder in [HERE.parent/'可撤销地图维护_图文并茂补充',HERE.parent/'可撤销旧点地图维护_图片版']:
        for p in folder.rglob('*'):
            if p.is_file() and p.suffix in ('.svg','.png','.pptx'):protected[str(p)]=sha(p)
    return dict(config_rules=values,source_hashes=protected,figure_original=str(RESULT)+'_trajectory.png',
                report_original=str(RESULT)+'_report.txt',evaluation='historical online odometry, not final Path',
                validation_scope='source/material verification only; no ROS simulation, compile or new experiment')

def package():
    files=sorted(p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    with ZipFile(HERE.parent/'独立回环后端_图片与证据合集.zip','w',ZIP_DEFLATED) as z:
        for p in files:z.write(p,str(Path(HERE.name)/p.relative_to(HERE)))

def main():
    facts=source_facts()
    (HERE/'源码与数据核验.json').write_text(json.dumps(facts,ensure_ascii=False,indent=2))
    (HERE/'证据原件').mkdir(exist_ok=True)
    for suffix in ('_trajectory.png','_report.txt'):
        src=Path(str(RESULT)+suffix);shutil.copy2(src,HERE/'证据原件'/src.name)
    architecture();workflow();rules();evidence()
    (HERE/'版面核验信息.json').write_text(json.dumps(dict(pages=PAGES,texts=TEXTS),ensure_ascii=False,indent=2))
    overview=Image.new('RGB',(1920,1130),'#E9EEF2');draw=ImageDraw.Draw(overview)
    overview_font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',24)
    for i,p in enumerate(sorted((HERE/'PNG').glob('*.png'))):
        im=Image.open(p).convert('RGB');im.thumbnail((930,523))
        x=20+(i%2)*950;y=15+(i//2)*560;overview.paste(im,(x,y))
        draw.text((x,y+526),p.stem,font=overview_font,fill=NAVY)
    overview.save(HERE/'00_四页总预览.png')
    for p,digest in facts['source_hashes'].items():assert sha(Path(p))==digest,p
    package();print('已生成4张3840×2160 PNG、4张SVG和原图/报告副本。')

if __name__=='__main__':
    if '--pack' in sys.argv:package()
    else:main()
