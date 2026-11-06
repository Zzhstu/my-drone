#!/usr/bin/env python3
"""可编辑矢量图与高清位图。仅生成讲解材料，不启动ROS。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import base64
import math
from pathlib import Path
from xml.sax.saxutils import escape
from PyQt5.QtCore import QByteArray, QRectF
from PyQt5.QtGui import QFont, QFontMetricsF, QImage, QPainter
from PyQt5.QtSvg import QSvgRenderer
from PyQt5.QtWidgets import QApplication

APP = QApplication.instance() or QApplication([])
HERE = Path(__file__).resolve().parent
NAVY='#15334C'; BLUE='#24618A'; TEAL='#137B7E'; GRAY='#657585'
RED='#CB5263'; GOLD='#D49B45'; CYAN='#42A6CA'
PASTEL=[('#EFF0FF','#B7C2EB'),('#FFF3E5','#E9C291'),('#EDF8F0','#A6D2BD'),
        ('#FFF0F5','#DFB3C9'),('#EDF5FF','#A8C8E7'),('#F3EFFC','#C4B5E0')]
TEXTS=[]; PAGES=[]

def font(size,bold=False):
    f=QFont('Noto Sans CJK SC');f.setPixelSize(round(size));f.setBold(bold);return f

class Page:
    def __init__(self,n,name,title,subtitle,footer):
        self.n=n;self.name=name;self.arrows=0;self.illustrations=0
        self.parts=['<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="1920" height="1080" viewBox="0 0 1920 1080">']
        self.rect(0,0,1920,1080,'white','none')
        self.text(65,46,title,47,bold=True,width=1790)
        self.text(68,122,subtitle,25,color=GRAY,width=1780)
        self.line([(65,181),(1855,181)],'#DCE5EB',1,False)
        self.text(65,1027,footer,21,color=TEAL,bold=True,width=1680)
        self.text(1764,1030,f'{n:02d} / 07',20,color=GRAY)

    def rect(self,x,y,w,h,fill='none',stroke=BLUE,sw=1.5,dashed=False):
        ds=' stroke-dasharray="8 6"' if dashed else ''
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{ds}/>')

    def poly(self,pts,fill,stroke='none',sw=1):
        s=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        self.parts.append(f'<polygon points="{s}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def circle(self,x,y,r,fill,stroke='none',sw=1):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def line(self,pts,color=BLUE,sw=2,arrow=True,dashed=False):
        s=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        ds=' stroke-dasharray="7 5"' if dashed else ''
        self.parts.append(f'<polyline points="{s}" fill="none" stroke="{color}" stroke-width="{sw}"{ds}/>')
        if arrow:
            x,y=pts[-1];a,b=pts[-2];d=math.hypot(x-a,y-b)
            if d:
                ux,uy=(x-a)/d,(y-b)/d;k=max(8,sw*3.5)
                self.poly([(x,y),(x-k*ux-k*.4*uy,y-k*uy+k*.4*ux),(x-k*ux+k*.4*uy,y-k*uy-k*.4*ux)],color)
                self.arrows+=1

    def text(self,x,y,t,size=23,color=NAVY,bold=False,width=None,center=False):
        fm=QFontMetricsF(font(size,bold));lines=[]
        for para in t.split('\n'):
            part=''
            for ch in para:
                if width and part and fm.horizontalAdvance(part+ch)>width:
                    # 中文闭合标点不单独落在行首；只调整本目录的绘图副本。
                    if ch in '，。；：！？、）】》”' and len(part)>1:
                        lines.append(part[:-1]);part=part[-1]+ch
                    else:
                        lines.append(part);part=ch
                else:part+=ch
            lines.append(part)
        for i,txt in enumerate(lines):
            yy=y+i*size*1.34;tw=fm.horizontalAdvance(txt);left=x-tw/2 if center else x
            TEXTS.append(dict(page=self.n,text=txt,x=left,y=yy,width=tw,height=size))
            anchor='middle' if center else 'start'
            self.parts.append(f'<text x="{x}" y="{yy+size}" xml:space="preserve" font-family="Noto Sans CJK SC,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{anchor}">{escape(txt)}</text>')
        return len(lines)*size*1.34

    def card(self,x,y,w,h,title,body,color=0,size=22):
        fill,stroke=PASTEL[color%6];self.rect(x,y,w,h,fill,stroke,2)
        th=self.text(x+20,y+15,title,27,bold=True,width=w-40)
        return y+24+th+self.text(x+20,y+24+th,body,size,width=w-40)

    def image(self,path,x,y,w,h):
        raw=base64.b64encode(path.read_bytes()).decode()
        self.parts.append(f'<image x="{x}" y="{y}" width="{w}" height="{h}" preserveAspectRatio="xMidYMid meet" xlink:href="data:image/png;base64,{raw}"/>')

    def save(self):
        self.parts.append('</svg>');raw='\n'.join(self.parts).encode()
        for folder in ('SVG','PNG'):(HERE/folder).mkdir(exist_ok=True)
        basename=f'{self.n:02d}_{self.name}'
        (HERE/'SVG'/f'{basename}.svg').write_bytes(raw)
        renderer=QSvgRenderer(QByteArray(raw));assert renderer.isValid()
        im=QImage(3840,2160,QImage.Format_ARGB32);im.fill(0xFFFFFFFF)
        painter=QPainter(im);painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)
        renderer.render(painter,QRectF(0,0,3840,2160));painter.end()
        assert im.save(str(HERE/'PNG'/f'{basename}.png'))
        PAGES.append(dict(number=self.n,name=self.name,arrows=self.arrows,illustrations=self.illustrations))

def drone(p,x,y,s=1):
    for dx,dy in [(-26,-11),(26,-11),(-26,11),(26,11)]:
        p.line([(x,y),(x+dx*s,y+dy*s)],NAVY,3*s,False)
        p.circle(x+dx*s,y+dy*s,9*s,'#E5F0FA',BLUE)
    p.poly([(x-11*s,y-7*s),(x+11*s,y-7*s),(x+14*s,y+5*s),(x,y+11*s),(x-14*s,y+5*s)],BLUE,NAVY)
    p.circle(x,y-2*s,4*s,'#A6E3D6')

def shelf(p,x,y,w,h,pointcloud=False):
    d=w*.22
    if pointcloud:
        hues=[TEAL,'#55AC88','#87B783',GOLD,'#C67A51']
        for j in range(19):
            for k in range(21):
                if j%6==0 or k in (0,20) or (k%6<4 and j%6<4):
                    for side in (0,1):
                        p.circle(x+k*w/20+side*d,y+j*h/18-side*d,1.65,hues[j//4])
    else:
        p.poly([(x,y),(x+d,y-d),(x+w+d,y-d),(x+w,y)],'#DEEAE6','#8DA9AB')
        for a in (x,x+w):p.line([(a,y),(a,y+h)],'#65888B',3,False)
        for z in (0,h*.34,h*.68,h):
            p.poly([(x,y+z),(x+w,y+z),(x+w+d,y+z-d),(x+d,y+z-d)],'#D2E7E1','#789D98')
            if z<h:
                for q in (.13,.59):p.rect(x+q*w,y+z+8,w*.24,h*.23,'#D8C9AA','#B5A68A',1)

def scene(p,x,y,w,h,mode='observe'):
    """非比例机制示意；不冒充真实仓库、地图或航迹。"""
    p.illustrations+=1
    shelf(p,x+w*.16,y+h*.12,w*.20,h*.47,mode in ('map','save'))
    shelf(p,x+w*.69,y+h*.12,w*.18,h*.47,mode in ('map','save'))
    u=x+w*.22;v=y+h*.77
    if mode in ('observe','rescan'):
        p.poly([(u,v),(x+w*.61,y+h*.12),(x+w*.66,y+h*.68)],'#DFEEE7')
        for i in range(7):p.line([(u,v),(x+w*(.49+i*.025),y+h*(.23+i*.064))],'#67B5AA',1.2,False)
    drone(p,u,v,.70)
    if mode=='rescan':
        drone(p,x+w*.66,y+h*.75,.70)
        p.line([(u,v+32),(x+w*.65,v+32)],GOLD,2)
    if mode=='home':
        p.circle(u,v,38,'none',TEAL,2)
        p.line([(x+w*.80,y+h*.75),(x+w*.52,y+h*.9),(u,v)],TEAL,2.8)
    if mode=='save':
        p.rect(x+w*.51,y+h*.63,w*.19,h*.27,'#EDF5FF',BLUE)
        p.text(x+w*.535,y+h*.685,'地图',18,bold=True)
        p.line([(x+w*.43,y+h*.72),(x+w*.51,y+h*.72)],TEAL,2)

def grid(p,x,y,w,h,mode='frontier'):
    """抽象观测格，灰色只表示未知而不是预加载货架。"""
    p.illustrations+=1
    nx,ny=18,10;cw,ch=w/nx,h/ny
    for j in range(ny):
        for i in range(nx):
            unknown=i>10+(j%3)
            occupied=(i in (5,9) and 2<=j<=6)
            fill='#E4E9EE' if unknown else '#E4F3EE'
            if occupied:fill='#DFC6A0'
            p.rect(x+i*cw,y+j*ch,cw-1,ch-1,fill,'none')
    if mode=='initial':
        p.rect(x+w*.18,y+h*.12,w*.64,h*.75,'none',BLUE,2,True)
    else:
        for j in (1,4,7):p.circle(x+cw*(11+j%3),y+ch*(j+.5),6,CYAN,BLUE)
        path=[(x+cw*2,y+ch*8),(x+cw*8,y+ch*8),(x+cw*12,y+ch*8),(x+cw*12,y+ch*7.5)]
        if mode=='home':path=list(reversed(path))
        p.line(path,TEAL,3)
        drone(p,*path[0],.65)
        if mode=='blocked':
            p.circle(x+cw*8,y+ch*4,12,'none',RED,2)
            p.line([(x+cw*3,y+ch*4),(x+cw*7.5,y+ch*4)],RED,2,True,True)
        if mode=='expand':p.rect(x+w*.12,y+h*.07,w*.60,h*.80,'none',BLUE,2,True)

def evidence(p,x,y,w,h):
    p.illustrations+=1
    shelf(p,x+w*.43,y+h*.09,w*.18,h*.49,True)
    for u,v in [(x+w*.12,y+h*.7),(x+w*.76,y+h*.72)]:
        drone(p,u,v,.72)
        p.line([(u,v),(x+w*.52,y+h*.44)],TEAL,1.8)
    p.line([(x+w*.15,y+h*.85),(x+w*.77,y+h*.85)],GOLD,2)
    p.text(x+w*.48,y+h*.88,'不同位置、不同方向',18,color=GRAY,center=True)
