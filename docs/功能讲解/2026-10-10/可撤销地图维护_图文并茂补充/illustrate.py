#!/usr/bin/env python3
"""六张可编辑工程图；绘制机制示意，不修改运行工程或制造实验数据。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import hashlib
import json
import math
from contextlib import contextmanager
from pathlib import Path
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET
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
OLD = HERE.parent / '可撤销旧点地图维护_图片版'
NAVY = '#17394F'; BLUE = '#28628B'; TEAL = '#1A837D'; RED = '#D04E58'
GRAY = '#687E8E'; ORANGE = '#D9933E'; CYAN = '#3686C5'
PALETTE = [('#EFF0FF','#B7C2EB'),('#FFF3E5','#E9C291'),
           ('#EDF8F0','#A6D2BD'),('#FFF0F5','#DFB3C9'),
           ('#EDF5FF','#A8C8E7'),('#F3EFFC','#C4B5E0')]
TEXTS = []; PAGES = []

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def font(size, bold=False):
    f = QFont('Noto Sans CJK SC'); f.setPixelSize(round(size)); f.setBold(bold)
    return f

def wrap(t, width, size, bold=False):
    fm=QFontMetricsF(font(size,bold)); out=[]
    for para in t.split('\n'):
        line=''
        for ch in para:
            if line and fm.horizontalAdvance(line+ch)>width:
                out.append(line);line=ch
            else:line+=ch
        out.append(line)
    return out

class Sheet:
    def __init__(self, n, name, title, subtitle, rule):
        self.n=n;self.name=name;self.arrows=0;self.illustrations=0
        self.transform=(0,0,1);self.bounds=[];self.parts=[]
        self.parts.append('<svg xmlns="http://www.w3.org/2000/svg" width="1920" height="1080" viewBox="0 0 1920 1080">')
        self.rect(0,0,1920,1080,'white','none')
        self.text(65,48,title,47,NAVY,True,width=1790)
        self.text(68,123,subtitle,26,GRAY,width=1780)
        self.line([(65,184),(1855,184)],'#DCE5EB',1,False)
        self.text(65,1032,rule,21,TEAL,True,width=1690)
        self.text(1764,1034,f'{n:02d} / 06',20,GRAY)

    def rect(self,x,y,w,h,fill='none',stroke=BLUE,sw=1.5,dashed=False):
        ds=' stroke-dasharray="7 5"' if dashed else ''
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{ds}/>')

    def polygon(self,pts,fill,stroke='none',sw=1.3):
        coords=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        self.parts.append(f'<polygon points="{coords}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def circle(self,x,y,r,fill,stroke='none',sw=1):
        self.parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def ellipse(self,x,y,rx,ry,fill,stroke='none',sw=1):
        self.parts.append(f'<ellipse cx="{x}" cy="{y}" rx="{rx}" ry="{ry}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def line(self,pts,color=BLUE,sw=2,arrow=True,dashed=False):
        ds=' stroke-dasharray="7 5"' if dashed else ''
        coords=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        self.parts.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="{sw}"{ds}/>')
        if arrow:
            x,y=pts[-1]
            for a,b in reversed(pts[:-1]):
                d=math.hypot(x-a,y-b)
                if d:
                    ux,uy=(x-a)/d,(y-b)/d;k=max(8,sw*3.5)
                    self.polygon([(x,y),(x-k*ux-.4*k*uy,y-k*uy+.4*k*ux),(x-k*ux+.4*k*uy,y-k*uy-.4*k*ux)],color)
                    self.arrows+=1;break

    def text(self,x,y,t,size=23,color=NAVY,bold=False,width=None,center=False,leading=1.35):
        lines=wrap(t,width,size,bold) if width else t.split('\n')
        fm=QFontMetricsF(font(size,bold)); tx,ty,s=self.transform
        for i,txt in enumerate(lines):
            yy=y+i*size*leading;tw=fm.horizontalAdvance(txt)
            left=x-tw/2 if center else x
            TEXTS.append(dict(page=self.n,text=txt,x=tx+left*s,y=ty+yy*s,
                width=tw*s,height=size*s,in_page=(tx+left*s>=0 and tx+(left+tw)*s<=1920 and ty+(yy+size)*s<=1080)))
            self.parts.append(f'<text x="{x}" y="{yy+size}" font-family="Noto Sans CJK SC,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{"middle" if center else "start"}">{escape(txt)}</text>')
        return len(lines)*size*leading

    @contextmanager
    def group(self,x,y,s=1):
        old=self.transform;tx,ty,ss=old
        self.transform=(tx+x*ss,ty+y*ss,ss*s)
        self.parts.append(f'<g transform="translate({x},{y}) scale({s})">')
        try:yield
        finally:self.parts.append('</g>');self.transform=old

    def card(self,x,y,w,h,title,body,color=0):
        fill,stroke=PALETTE[color%6]
        self.rect(x,y,w,h,fill,stroke,2)
        th=self.text(x+21,y+15,title,28,NAVY,True,width=w-42,leading=1.24)
        bh=self.text(x+21,y+23+th,body,22,NAVY,width=w-42,leading=1.35)
        self.bounds.append(dict(title=title,body_bottom=y+23+th+bh,art_top=y+142,
                                body_fits=y+23+th+bh<=y+144))
        return x+16,y+147,w-32,h-167

    def finish(self):
        self.parts.append('</svg>')
        raw='\n'.join(self.parts).encode('utf-8')
        for sub in ('SVG','PNG'): (HERE/sub).mkdir(exist_ok=True)
        basename=f'{self.n:02d}_{self.name}'
        sv=HERE/'SVG'/f'{basename}.svg'; pn=HERE/'PNG'/f'{basename}.png'
        sv.write_bytes(raw);renderer=QSvgRenderer(QByteArray(raw));assert renderer.isValid()
        im=QImage(3840,2160,QImage.Format_ARGB32);im.fill(0xFFFFFFFF)
        painter=QPainter(im);painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)
        renderer.render(painter,QRectF(0,0,3840,2160));painter.end();assert im.save(str(pn))
        PAGES.append(dict(number=self.n,name=self.name,png=str(pn.relative_to(HERE)),
                          svg=str(sv.relative_to(HERE)),arrows=self.arrows,
                          illustrations=self.illustrations,card_checks=self.bounds))

def cube(p,x,y,w=40,h=40,d=14,fill='#DAE9F0',stroke='#6F91A7'):
    p.polygon([(x,y),(x+w,y),(x+w,y+h),(x,y+h)],fill,stroke)
    p.polygon([(x,y),(x+d,y-d),(x+w+d,y-d),(x+w,y)],'#F7FBFD',stroke)
    p.polygon([(x+w,y),(x+w+d,y-d),(x+w+d,y+h-d),(x+w,y+h)],'#C3D9E5',stroke)

def drone(p,x,y,s=.65):
    with p.group(x,y,s):
        p.ellipse(0,17,40,8,'#DAE4EC')
        for u,v in [(-32,-12),(32,-12),(-32,12),(32,12)]:
            p.line([(0,0),(u,v)],NAVY,4,False)
            p.ellipse(u,v,16,5,'#E6F2FB',BLUE,2)
        p.polygon([(-13,-9),(11,-9),(17,1),(0,10),(-17,1)],'#4F8CB5',NAVY)
        p.line([(-8,7),(-13,20),(0,20)],NAVY,2,False)
        p.line([(8,7),(13,20),(0,20)],NAVY,2,False)
        p.circle(0,-4,4,'#9EE6E0',TEAL)

def person(p,x,y,s=1,color=ORANGE,ghost=False):
    with p.group(x,y,s):
        p.ellipse(0,3,15,4,'#E1E6E9')
        if not ghost:
            p.circle(0,-50,8,color)
            p.line([(-12,-26),(-8,-37),(7,-37),(14,-22)],color,5,False)
            p.line([(0,-35),(0,-15),(-8,0)],color,8,False)
            p.line([(1,-16),(10,0)],color,5,False)
        else:
            # 人体旧点也画成人形，而不是仅用一个红色柱子代替。
            for ix in range(-6,7,3):
                for yy in range(-56,-43,3):
                    if ix*ix+(yy+50)**2<=49:p.circle(ix,yy,1.4,color)
            for ix in range(-6,7,3):
                for yy in range(-38,-15,3):p.circle(ix,yy,1.5,color)
            for ax,ay,bx,by in [(-6,-35,-14,-21),(6,-35,14,-21),(0,-17,-8,0),(1,-17,10,0)]:
                for k in range(10):
                    xx=ax+(bx-ax)*k/9;yy=ay+(by-ay)*k/9
                    for dx in (-1.6,1.6):p.circle(xx+dx,yy,1.4,color)
            p.line([(-19,7),(19,7),(19,-61),(-19,-61),(-19,7)],color,1.3,False,True)

def shelf(p,x,y,w=90,h=105,d=25,points=False,color=TEAL):
    if not points:
        p.polygon([(x,y),(x+d,y-d),(x+w+d,y-d),(x+w,y)],'#DDEBE8','#799C98')
        for xx in (x,x+w):p.line([(xx,y),(xx,y+h)],'#53798B',3,False)
        for xx in (x+d,x+w+d):p.line([(xx,y-d),(xx,y+h-d)],'#81A79F',2,False)
        for z in (0,h*.34,h*.68,h):
            p.polygon([(x,y+z),(x+w,y+z),(x+w+d,y+z-d),(x+d,y+z-d)],'#CFE4DE','#70988F')
            if z<h:
                for j in (12,w*.56):
                    p.rect(x+j,y+z+10,w*.27,h*.22,'#D9C9AB','#B5A27F',1)
    else:
        hues=['#368A7B','#64A976','#A1BA66','#E3B453','#D78856']
        for level in range(22):
            for k in range(24):
                zz=level*h/21;u=k*w/23
                if level%7==0 or k in (0,23) or (k%6<4 and level%7<4):
                    for side in (0,1):
                        xx=x+u+side*d; yy=y+zz-side*d
                        p.circle(xx+.7*math.sin(k+level),yy,1.45,hues[min(4,level//5)])
        for z in (0,h*.34,h*.68,h):
            p.line([(x,y+z),(x+w,y+z),(x+w+d,y+z-d)],color,.7,False)

def warehouse(p,x,y,w,h,state='ghost',points=False,rays=False,local=False):
    """仓库货架/通道的抽象小场景；不是cangku完整几何或真实点云。"""
    p.illustrations+=1
    with p.group(x,y,min(w/360,h/210)):
        p.polygon([(7,151),(270,151),(351,85),(89,85)],'#F5F8FA','#CCDDE3')
        for k in range(8):
            a=7+k*37
            p.line([(a,151),(a+81,85)],'#E2EAF0',.8,False)
        for k in range(5):
            a=151-k*16.5
            p.line([(7+k*20,a),(270+k*20,a)],'#E2EAF0',.8,False)
        shelf(p,88,26,70,84,20,points)
        shelf(p,255,26,64,84,20,points)
        if local:
            p.ellipse(88,130,79,37,'none',CYAN,1.4)
        if state in ('ghost','moving','rays','kept'):
            person(p,181,140,1,RED,True)
        elif state=='standing':person(p,181,140,1,ORANGE,False)
        elif state=='clean':
            p.line([(159,145),(202,145),(215,134),(172,134),(159,145)],TEAL,1.5,False,True)
        if state in ('moving','rays','clean','kept'):
            person(p,244,132,.9,ORANGE)
            p.line([(195,151),(233,146)],ORANGE,1.5,True,True)
        if rays:
            for xx,yy in [(249,75),(264,91),(278,105)]:
                p.line([(58,129),(xx,yy)],'#6EBDB5',1.2,False)
                p.circle(xx,yy,3,CYAN)
            p.line([(70,128),(198,105)],TEAL,2,True)
        drone(p,53,125,.57)
        p.text(13,175,'无人机',17,BLUE)
        p.text(109,175,'货架 / 通道',17,GRAY)
        if state in ('ghost','moving','rays','kept'):p.text(245,175,'旧人体点',17,RED)
        elif state=='clean':p.text(255,175,'撤销后',17,TEAL)
        elif state=='standing':p.text(251,175,'当前人体',17,ORANGE)

def cloud_to_voxel(p,x,y,w,h):
    p.illustrations+=1
    with p.group(x,y,min(w/390,h/194)):
        shelf(p,15,18,90,95,18,True)
        person(p,133,131,.85,RED,True)
        p.line([(163,78),(205,78)],BLUE,2.5)
        for k,(a,b) in enumerate([(217,54),(257,54),(297,54),(237,101),(277,101)]):
            cube(p,a,b,28,28,10,'#E7EEF9' if k%2 else '#DFEEE5')
            p.circle(a+15,b+12,3,TEAL if k<3 else RED)
        p.text(28,157,'点云观测',18,NAVY)
        p.text(223,157,'格子 + 记录',18,NAVY)

def alignment(p,x,y,w,h):
    p.illustrations+=1
    with p.group(x,y,min(w/390,h/194)):
        drone(p,40,49,.6)
        p.text(13,76,'位姿',18,BLUE)
        p.line([(75,42),(149,42)],BLUE,2)
        p.rect(161,23,202,39,'white','#A6BAC9')
        p.text(176,30,'扫描时刻  t',19,NAVY)
        p.line([(173,108),(350,108)],'#A9B8C4',2,False)
        for a in (185,225,265,305,340):p.circle(a,108,4,'#AAC1D1')
        p.circle(265,108,7,CYAN);p.circle(277,79,7,ORANGE)
        p.line([(265,108),(277,79)],TEAL,2,False)
        p.text(151,132,'探索：时间差 ≤ 0.08 秒',17,TEAL)
        p.text(7,165,'同一坐标、同一时刻附近的观测',18,NAVY)

def voxel_update(p,x,y,w,h):
    p.illustrations+=1
    with p.group(x,y,min(w/390,h/194)):
        cube(p,33,43,69,67,27,'#EAF3FB')
        for a,b in [(47,60),(65,80),(82,67),(54,91),(94,93),(85,46)]:p.circle(a,b,3.7,CYAN)
        p.line([(142,82),(187,82)],BLUE,2.5)
        cube(p,220,43,69,67,27,'#DFEEE5')
        p.circle(254,75,5,TEAL)
        p.text(17,151,'同格很多点',18,NAVY)
        p.text(214,151,'本帧命中 +1',18,NAVY)
        p.text(143,5,'代表点更新',18,TEAL)

def ray_panel(p,x,y,w,h,kind='valid'):
    p.illustrations+=1
    with p.group(x,y,min(w/390,h/190)):
        p.rect(23,117,330,31,'#F4F8FA','#DAE5EB')
        drone(p,37,109,.48)
        cube(p,162,80,34,45,10,'#FBE2E6',RED)
        p.circle(179,99,4,RED)
        p.line([(58,100),(338,100)],'#B5C2CD',1,False,True)
        if kind=='occluded':
            p.rect(109,53,18,77,'#AAC6BB',TEAL)
            p.line([(58,100),(109,100)],TEAL,2.4,False)
            p.circle(109,100,5,CYAN)
            p.polygon([(127,60),(343,60),(343,130),(127,130)],'#E6E9ED')
            cube(p,162,80,34,45,10,'#FBE2E6',RED)
            p.circle(179,99,4,RED)
            p.text(143,18,'挡住后方 → 没有清空证据',18,GRAY)
        elif kind=='near':
            p.line([(58,100),(139,100)],TEAL,2.4,False)
            p.circle(205,100,5,CYAN)
            p.line([(143,140),(205,140)],ORANGE,1.7,False)
            p.text(150,153,'终点前 0.30 米保护段',17,ORANGE)
        elif kind=='off':
            p.line([(58,123),(332,123)],TEAL,2.2,False)
            p.circle(337,123,5,CYAN)
            p.line([(179,99),(179,123)],RED,1.4,False)
            p.text(101,19,'经过同一个格子 ≠ 穿过旧点',18,RED)
            p.text(115,153,'点线距离超过 6 厘米：不计',17,GRAY)
        elif kind=='current':
            for i in (-1,0,1):
                for j in (-1,0,1):p.rect(144+i*34,46+j*34,34,34,'none','#A8BBD2',1)
            p.line([(58,100),(194,93)],TEAL,2,False)
            p.circle(194,93,6,CYAN)
            p.rect(99,9,276,29,'#F7FBFD','none')
            p.text(119,18,'本格或相邻格仍有回波',18,BLUE)
            p.text(126,156,'保留当前真实结构',18,TEAL)
        else:
            p.line([(58,100),(281,100)],TEAL,2.4,False)
            for i in range(6):p.rect(73+i*35,92,28,16,'#D9EEE5','none')
            p.circle(179,99,4,RED)
            p.circle(338,100,5,CYAN)
            p.line([(286,140),(338,140)],ORANGE,1.7,False)
            p.text(286,119,'终点缓冲',12,ORANGE)
            p.text(132,18,'真实回波在旧点后方',18,TEAL)
            p.text(105,155,'绿色：回波前的已观测自由空间',16,TEAL)

def counters(p,x,y,w,h):
    p.illustrations+=1
    with p.group(x,y,min(w/550,h/220)):
        for row,limit in enumerate((3,6)):
            yy=20+row*78
            p.text(10,yy+3,'普通旧格' if row==0 else '稳定旧格',18,NAVY)
            for k in range(limit):
                p.rect(125+k*36,yy,29,28,'#DCEEE5',TEAL,1)
                p.text(140+k*36,yy+3,str(k+1),16,TEAL,True,center=True)
            end=125+limit*36
            p.line([(end+6,yy+14),(end+42,yy+14)],BLUE,2)
            p.text(end+52,yy+3,'撤销',19,RED,True)
        p.line([(129,176),(330,176)],'#B3C6D2',2,False)
        for u in (150,210,270):p.circle(u,176,4,TEAL)
        p.circle(324,176,6,CYAN)
        p.text(24,192,'再次命中：计数归零；超时再证据：重新累计',17,GRAY)

def views(p,x,y,w,h):
    p.illustrations+=1
    with p.group(x,y,min(w/550,h/220)):
        for j,(title,pts) in enumerate([('持续 ≥2',False),('稳定 ≥8',True),('局部 / 6米',True)]):
            xx=12+j*178
            p.rect(xx,15,164,141,'#F7FAFC','#BDCCD8')
            shelf(p,xx+20,37,82,66,18,pts)
            # 本页是删除后的输出，不再画成仍带人体旧点的地图。
            p.rect(xx+118,86,17,36,'none',TEAL,1,True)
            p.text(xx+82,170,title,18,NAVY,True,center=True)
        p.text(36,205,'旧格撤销后 → 各占用视图都不再输出该格',17,TEAL)

def main_workflow():
    p=Sheet(1,'七步图文循环总流程','可撤销旧点地图维护：七步循环工作流程',
            '沿用参考图的七步布局；在每一步放入仓库、点云、格子和观测变化。',
            '工程机制示意｜探索证据模式｜维护导航地图，不自动回写 FAST-LIO 内部匹配树')
    top=[(65+i*460,240,410,350) for i in range(4)]
    bottom=[(1295,651,560,348),(680,651,560,348),(65,651,560,348)]
    texts=[('1  初次建立地图记忆','注册扫描按15厘米格子聚合，\n记录代表点、命中和时间。'),
           ('2  配对扫描与位姿','确认坐标一致；探索模式\n等待时间相近的定位结果。'),
           ('3  写入当前真实占用','同一回调每格命中一次；\n再次看到便清零已空计数。'),
           ('4  用回波核实旧位置','从雷达位置向真实回波连线，\n只检查近处且终点前的空间。'),
           ('5  先通过误删保护','检查点线距离、当前邻格回波；\n遮挡或没扫到，不代表已经空了。'),
           ('6  累计证据并撤销旧格','旧格需满足时效与次数规则；\n普通3次，稳定旧格6次。'),
           ('7  发布更新后的地图','共享格子生成多种地图视图，\n供显示、规划、探索与保存使用。')]
    arts=[cloud_to_voxel,alignment,voxel_update,ray_panel,
          lambda p,x,y,w,h:ray_panel(p,x,y,w,h,'current'),counters,views]
    for i,(pos,tx,art) in enumerate(zip(top+bottom,texts,arts)):
        a=p.card(*pos,*tx,color=i);art(p,*a)
    for i in range(3):p.line([(top[i][0]+415,425),(top[i+1][0]-5,425)],BLUE,2.6)
    p.line([(1640,597),(1640,622),(1580,622),(1580,645)],BLUE,2.6)
    p.line([(1290,827),(1245,827)],BLUE,2.6);p.line([(675,827),(630,827)],BLUE,2.6)
    p.line([(60,827),(35,827),(35,215),(730,215),(730,235)],BLUE,2.5)
    p.text(79,191,'处理下一帧扫描；初次建记录后，从配对观测继续',19,BLUE)
    p.finish()

def person_sequence():
    p=Sheet(2,'人体离开与地图撤销对比','人离开以后，地图怎样把旧占用撤销？',
            '对比当前观测与历史点，而不是看到“人”就删；目前没有在这里运行人体识别网络。',
            '图中所有仓库与点云为机制示意，不是新增仿真结果或“人体全部清除”的保证。')
    states=['standing','moving','rays','clean']
    heads=['① 人先站着：能够进入地图','② 人已离开：旧点仍在原处','③ 再次观测：积累已空证据','④ 满足条件：撤销旧人体格']
    bodies=['静止并多次被扫到，人体点也会\n被当作当前占用，不会自动消失。',
            '橙色是现在的人；红色是历史点。\n只挡住新的动点，不能消除旧点。',
            '回波在旧位置后方，射线穿过旧格；\n还需通过邻格、距离和时效检查。',
            '删除的是满足条件的旧格记录；\n当前的人和真实货架仍须避让。']
    for k in range(4):
        x=65+k*460;a=p.card(x,235,410,496,heads[k],bodies[k],k)
        warehouse(p,a[0]+4,a[1]+18,a[2],240,states[k],True,k==2)
        p.text(x+23,667,['当前占用：存在','历史位置：仍占用','证据：逐次确认','旧位置：可被撤销'][k],23,RED if k<2 else TEAL,True,width=370)
        if k<3:p.line([(x+415,482),(x+455,482)],BLUE,3)
    p.rect(65,778,1790,205,'#F5F8FB','#BFD2DE')
    p.text(86,794,'不能把“没有看到”直接画成“已经不存在”',30,NAVY,True)
    p.text(90,847,'货架挡住 / 扫描稀疏 / 距离过远：本轮不增加撤销证据。人站着不动：仍是真实障碍。',25,NAVY,width=1700)
    p.text(90,898,'工程意义：允许地图纠正历史判断；局限：未再次观测的区域，旧点仍可能保留。',25,TEAL,width=1700)
    p.finish()

def protections():
    p=Sheet(3,'射线证据与四类误删保护','一条射线，怎样区别“已经空了”和“只是没看见”？',
            '探索模式：射线授权是有条件的；不能把整块局部区域、货架背后或稀疏缺点全部清空。',
            '参数来自当前仓库探索配置；示意并非静态保留率或清理精度实验。')
    a=p.card(65,229,570,360,'允许累计：穿过旧点到达后方','旧格距真实射线不超过6厘米；\n还须通过时效与邻格保护。',2)
    ray_panel(p,*a,'valid')
    a=p.card(707,229,570,360,'不能累计：货架把后方挡住','射线只到当前回波前方；\n遮挡后的旧点不能凭空删除。',3)
    ray_panel(p,*a,'occluded')
    a=p.card(1349,229,506,360,'不能累计：偏离旧点','即使穿过同一个15厘米格子，\n也不一定真的扫过代表点。',1)
    ray_panel(p,*a,'off')
    a=p.card(65,633,570,358,'保留：回波终点附近','终点前保留0.30米缓冲，\n避免删除真实表面。',1)
    ray_panel(p,*a,'near')
    a=p.card(707,633,570,358,'保留：附近仍有真实回波','本格及周围26邻格有当前点，\n该旧格不获撤销证据。',4)
    ray_panel(p,*a,'current')
    p.rect(1349,633,506,358,'#F0F5F8','#B7CCD9',2)
    p.text(1373,651,'时效、数量也必须过关',28,NAVY,True,width=458)
    lines=['扫描与位姿时间差 ≤ 0.08秒','最近命中后至少经过0.5秒','有效证据间隔超过1秒重新累计','同一回调多条射线最多计一次','普通旧格3次；稳定旧格6次','只检查近6米，不是清空全仓库']
    for j,t in enumerate(lines):
        p.circle(1381,717+j*40,4,TEAL)
        p.text(1398,699+j*40,t,21,NAVY,width=433)
    p.finish()

def storage_views():
    p=Sheet(4,'共享存储与多地图视图','一份可修改的地图记忆，生成不同用途的视图',
            '持久图、稳定图、局部图不是三套互不相关的历史；自由观测则来自最新一帧射线。',
            '局部图范围变小不等于全局记录被删；自由格也不等于所有格子都已获得持久删除授权。')
    a=p.card(65,230,490,430,'一份共享格子记录','代表位置、命中次数、最近时间、\n已空证据次数都保存在同一格里。',0)
    voxel_update(p,a[0],a[1]+18,a[2],205)
    p.text(87,605,'真正撤销：删除共享格记录',25,TEAL,True,width=445)
    p.line([(560,444),(606,444),(606,310),(649,310)],BLUE,2.4)
    p.line([(606,444),(606,727),(649,727)],BLUE,2.4)
    layouts=[(655,230,570,355,'持续地图：至少2次命中','长期保留已观测环境；显示和保存。',0,'ghost',False),
             (1285,230,570,355,'稳定视图：至少8次命中','反复出现的结构；普通全局规划。',2,'ghost',True),
             (655,631,570,360,'局部占用：近6米 / 近期','至少1次命中，近15帧且无已空计数。',4,'standing',True),
             (1285,631,570,360,'当前自由观测：射线经过','局部修路依据，比持久删除集合宽。',1,'rays',True)]
    for x,y,w,h,title,body,c,state,points in layouts:
        a=p.card(x,y,w,h,title,body,c)
        warehouse(p,a[0]+35,a[1]-12,a[2]-70,a[3]+20,state,points,c==1,c==4)
    p.rect(65,710,490,281,'#EFF7F1','#ACCFBA',2)
    p.text(88,729,'一次删除，多个视图改变',29,TEAL,True,width=440)
    p.text(88,785,'删除格子后，下次发布时\n持续、稳定、局部占用都不再\n从这个历史格子生成点。',24,NAVY,width=440)
    p.text(88,906,'自由观测不来自同一占用筛选。',21,GRAY,width=440)
    p.finish()

def clearing_modes():
    p=Sheet(5,'两种清理模式的图解','两条清理思路：普通演示与自主探索要分开讲',
            '普通演示可以利用确认目标的经过轨迹；探索模式关闭轨迹圆柱直接删除，只看真实观测证据。',
            '二者都是当前工程实现；图中圆柱范围不代表语义人体识别，保护规则也不能证明绝不误删。')
    p.rect(65,230,1790,351,'#FFF4ED','#E6BF98',2)
    p.text(87,245,'普通演示：确认运动 → 清理经过区域，同时阻止新动点重复入图',30,NAVY,True,width=1720)
    warehouse(p,88,311,520,252,'moving',True,False)
    p.line([(558,414),(610,414)],BLUE,3)
    with p.group(650,320,1):
        p.illustrations+=1
        for x in (48,108,168,228):
            p.rect(x-18,35,36,100,'#F6D9DF',RED,1,True)
            p.ellipse(x,35,18,7,'#FBE8EC',RED)
            p.ellipse(x,135,18,7,'#FBE8EC',RED)
        p.line([(48,153),(243,153)],RED,2,True,True)
        person(p,243,126,1,ORANGE)
        p.text(18,187,'清理轨迹附近的历史格子',20,RED)
    p.line([(978,414),(1040,414)],BLUE,3)
    warehouse(p,1056,302,426,255,'clean',True)
    p.text(1510,321,'历史轨迹保留20秒',23,NAVY,width=314)
    p.text(1510,365,'掩膜半径0.65米；\n实际清理还多一个格子。',22,NAVY,width=314)
    p.text(1510,438,'代价：错误运动目标\n可能误清邻近静态结构。',22,RED,width=314)
    p.rect(65,635,1790,363,'#EDF8F3','#A6CDB9',2)
    p.text(87,651,'自主探索：时间配对 → 射线与保护 → 多次确认 → 撤销旧格',30,NAVY,True,width=1720)
    warehouse(p,88,715,420,233,'rays',True,True)
    p.line([(498,816),(554,816)],BLUE,3)
    ray_panel(p,575,731,370,205,'current')
    p.line([(945,816),(1006,816)],BLUE,3)
    counters(p,1026,729,480,230)
    p.line([(1510,816),(1552,816)],BLUE,3)
    p.text(1567,745,'代价与边界',27,TEAL,True)
    p.text(1567,799,'更依赖再次扫描、位姿\n和有效时序；清理可能\n比直接轨迹删除更慢。',22,NAVY,width=269)
    p.text(600,959,'受到保护的格子保留；仅未被保护且满足条件的旧格累计撤销证据。',18,TEAL,width=940)
    p.finish()

def route(p,x,y,w,h,cleared=False):
    p.illustrations+=1
    with p.group(x,y,min(w/450,h/280)):
        p.rect(15,15,419,245,'#F8FBFC','#C9DCE5')
        for xx in (65,302):
            p.rect(xx-10,42,84,166,'none','#99B8AA',1,True)
            p.rect(xx,52,64,146,'#DCEBE3',TEAL,2)
            for yy in range(64,191,20):p.line([(xx+5,yy),(xx+59,yy)],'#A5C5B3',1,False)
        if not cleared:p.rect(140,124,151,34,'#FBE5E8','none')
        for i in range(11):
            for j in range(3):
                if cleared:p.circle(151+i*11,136+j*7,2,'#F8FBFC','#D7E4E8')
                else:p.circle(151+i*11,136+j*7,2.7,RED)
        p.text(146,174,'旧占用已撤销' if cleared else '历史点形成假墙',18,TEAL if cleared else RED)
        drone(p,214,225,.62)
        p.circle(214,35,7,ORANGE);p.text(231,22,'目标',18,NAVY)
        if cleared:p.line([(214,218),(214,157),(214,55)],TEAL,3)
        else:p.line([(214,218),(394,218),(394,28),(230,28)],BLUE,3)
        if cleared:
            person(p,391,172,.9,ORANGE)
            p.ellipse(391,153,25,37,'none',ORANGE,1.5)
        p.text(39,270,'保留货架占用与安全膨胀',18,GRAY)

def replanning():
    p=Sheet(6,'更新地图如何影响路径','地图纠正以后，路径也要重新审视',
            '普通两阶段规划的例子：全局提供方向，当前占用和自由观测共同修正局部；探索使用自己的规划器。',
            '路线仅示意；不等于本轮实测绕行距离、成功率或安全性；当前人体仍是实际障碍。')
    a=p.card(65,230,510,454,'更新前：假墙让路线绕远','规划读取历史占用，人体旧点\n可能被扩大为障碍，形成假墙。',3)
    route(p,a[0]+12,a[1]+2,a[2]-24,a[3]+3,False)
    p.line([(583,450),(640,450)],BLUE,3)
    a=p.card(655,230,610,454,'两条更新支路，不必完全串行','局部自由修路可先发生；\n持久删除仍须通过更严格的确认。',1)
    p.rect(681,416,557,80,'#F7FBFE','#A6C2D9',1.5)
    p.text(698,428,'当前自由观测 → 剔除相匹配的旧点',23,BLUE,True,width=520)
    p.text(698,459,'先修正原始三维点，再生成安全膨胀。',20,GRAY,width=520)
    p.rect(681,526,557,80,'#F1F9F2','#B0D1B7',1.5)
    p.text(698,538,'多次确认撤销 → 持续地图更新',23,TEAL,True,width=520)
    p.text(698,569,'真正删除共享记录；各占用视图随之更新。',20,GRAY,width=520)
    p.text(687,630,'当前局部占用最后叠加，不能被自由观测抵消。',21,NAVY,width=554)
    p.line([(1272,450),(1340,450)],BLUE,3)
    a=p.card(1349,230,506,454,'更新后：重新选择可行路线','通道内旧占用解除后，重新规划；\n真实货架、当前行人仍然需要避让。',2)
    route(p,a[0]+6,a[1]+2,a[2]-12,a[3]+3,True)
    p.rect(65,748,1790,249,'#F3F7FA','#B8CDD9',2)
    p.text(91,764,'地图、路线、安全保护：互相配合，但各自负责不同事情',31,NAVY,True,width=1710)
    for j,(title,body) in enumerate([('地图维护','纠正过去留下的失效占用。'),('路径规划','重新计算可走路线和中间目标。'),('即时保护','仍检查现在的障碍与输入有效性。')]):
        x=95+j*590
        p.text(x,830,title,27,TEAL,True)
        p.text(x,884,body,23,NAVY,width=540)
        if j<2:p.line([(x+545,872),(x+576,872)],BLUE,2)
    p.finish()

def verify_rules():
    cfg=yaml.safe_load((B/'config/static_map.yaml').read_text())
    expected={'voxel_size':.15,'minimum_static_hits':2,'stable_minimum_static_hits':8,
              'local_minimum_hits':1,'local_max_age_frames':15,'local_radius':6.,
              'free_space_miss_threshold':3,'free_space_endpoint_margin':.30,
              'free_space_max_range':6.,'dynamic_mask_radius':.65,'dynamic_history_seconds':20.}
    for k,v in expected.items():assert cfg[k]==v,(k,cfg[k],v)
    params={x.attrib['name']:x.attrib['value'] for x in ET.parse(B/'launch/warehouse_exploration.launch').getroot().findall('param')}
    more={'evidence_only_clearing':'true','stable_free_miss_threshold':'6',
          'free_evidence_max_gap':'1.0','free_evidence_min_age':'0.5',
          'free_ray_point_tolerance':'0.06','clearing_odom_max_dt':'0.08'}
    for k,v in more.items():assert params['/static_map_builder/'+k]==v
    src=(B/'src/static_map_builder.cpp').read_text()
    snippets=['if (evidence_only_clearing_) return;','cell.free_misses = 0;',
              'if (gap<=0 || gap>evidence_max_gap_) cell.free_misses=0;',
              'if (endpoints.count(key)) continue;','if (!protected_cells.count(key)) evidence.insert(key);',
              'cells_.erase(it);','local_map.push_back(p);',
              'if (publish_legacy_map_) legacy_map_pub_.publish(output);']
    for t in snippets:assert t in src,t
    planner=(B/'src/static_path_planner.cpp').read_text()
    assert planner.index('rasterize(primary_map, inflation_radius_, 1U, true);')<planner.index('rasterize(*overlay_map, inflation_radius_, 1U);')
    assert 'free_tree.nearestKSearch(point, 1, free_indices, free_distances)' in planner
    return len(expected)+len(more)+len(snippets)+2

def overview():
    im=Image.new('RGB',(1600,1440),'#E7EDF1');d=ImageDraw.Draw(im)
    f=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',23)
    for i,page in enumerate(PAGES):
        thumb=Image.open(HERE/page['png']).convert('RGB');thumb.thumbnail((772,434))
        x=14+(i%2)*800;y=12+(i//2)*478
        im.paste(thumb,(x,y));d.text((x+8,y+439),f'{i+1:02d}  {page["name"]}',font=f,fill=NAVY)
    im.save(HERE/'00_六张图文流程图预览.png')

def package():
    zip_path=HERE/'可撤销地图维护_六张图文流程图.zip'
    with ZipFile(zip_path,'w',ZIP_DEFLATED) as z:
        for file in sorted(HERE.rglob('*')):
            if file.is_file() and file!=zip_path and '__pycache__' not in file.parts:
                z.write(file,file.relative_to(HERE))
    with ZipFile(zip_path) as z:assert z.testzip() is None

def main():
    files=[B/'src/static_map_builder.cpp',B/'config/static_map.yaml',
           B/'launch/warehouse_exploration.launch',B/'src/static_path_planner.cpp',
           B/'scripts/warehouse_explorer.py',ROOT/'simulation/astra_gazebo_worlds/cangku.world',
           Path('/home/a/下载/fastlio2ppt.pptx')]
    sources={str(p):sha(p) for p in files}
    old_before={str(p.relative_to(OLD)):sha(p) for p in OLD.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    rules=verify_rules()
    for fn in [main_workflow,person_sequence,protections,storage_views,clearing_modes,replanning]:fn()
    assert all(t['in_page'] for t in TEXTS),[t for t in TEXTS if not t['in_page']]
    assert all(c['body_fits'] for p in PAGES for c in p['card_checks']),[c for p in PAGES for c in p['card_checks'] if not c['body_fits']]
    assert all(sha(Path(k))==v for k,v in sources.items())
    assert old_before=={str(p.relative_to(OLD)):sha(p) for p in OLD.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    overview()
    report={'pages':PAGES,'source_hashes':sources,'old_file_hashes':old_before,
            'text_checks':TEXTS,'source_rule_checks':rules,'pixel_size':[3840,2160],
            'kind':'工程机制示意；所有场景与点云为手绘矢量，不是实测结果',
            'old_preserved':True,'new_ppt_files':0}
    (HERE/'来源与生成核验.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    package()
    print(json.dumps({'pages':len(PAGES),'source_rules':rules,'text_lines':len(TEXTS),
         'scene_illustrations':sum(x['illustrations'] for x in PAGES),'arrows':sum(x['arrows'] for x in PAGES)},ensure_ascii=False))

if __name__=='__main__':main()
