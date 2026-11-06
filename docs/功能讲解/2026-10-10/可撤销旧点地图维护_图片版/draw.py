"""中文矢量讲解图绘制；无PPT依赖，所有点云图均为机制示意。"""
import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import math
from pathlib import Path
from xml.sax.saxutils import escape
from PyQt5.QtCore import QByteArray,QRectF
from PyQt5.QtGui import QFont,QFontMetricsF,QImage,QPainter
from PyQt5.QtSvg import QSvgRenderer
from PyQt5.QtWidgets import QApplication

APP=QApplication.instance() or QApplication([])
NAVY='#15334C';BLUE='#24618A';TEAL='#137B7E';GRAY='#657585';RED='#B94343'
COLORS=[('#F0F0FF','#B3BFEA'),('#FFF3E7','#E6B988'),('#EEF8F1','#A6CCB7'),
        ('#FFF0F5','#DCB0C3'),('#EDF5FF','#A7C6E3'),('#F3EEFD','#C1B1E1')]
CHECKS=[]

def font(size,bold=False):
    f=QFont('Noto Sans CJK SC');f.setPixelSize(round(size));f.setBold(bold);return f

def wrap(text,width,size,bold=False):
    fm=QFontMetricsF(font(size,bold));out=[]
    for para in text.split('\n'):
        line=''
        for ch in para:
            if line and fm.horizontalAdvance(line+ch)>width:
                if ch in '，。；：！？、）》】%％' and len(line)>1:
                    out.append(line[:-1]);line=line[-1]+ch
                else:out.append(line);line=ch
            else:line+=ch
        out.append(line)
    return out

class Page:
    def __init__(self,n,name,title,subtitle,rule,sources):
        self.n=n;self.name=name;self.sources=sources;self.texts=[];self.arrow_count=0
        self.parts=['<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="900" viewBox="0 0 1600 900">']
        self.rect(0,0,1600,900,'#FFFFFF','none')
        self.text(60,34,title,46,NAVY,True,width=1480)
        self.text(63,101,subtitle,24,GRAY,width=1474)
        self.line([(60,151),(1540,151)],'#D5E0E6',1.2,False)
        self.rect(60,784,1480,62,'#EFF5F8','none')
        self.text(78,798,'核心规则',25,TEAL,True)
        self.text(210,798,rule,24,NAVY,width=1300)
        self.text(60,861,'现有源码规则 · 所有点云与场景小图为示意 · 非新增实验结果',17,GRAY)
        self.text(1400,861,f'{n:02d} / 16',18,GRAY)

    def rect(self,x,y,w,h,fill='#FFFFFF',stroke=BLUE,dashed=False):
        d=' stroke-dasharray="7 6"' if dashed else ''
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="1.6"{d}/>')

    def polygon(self,pts,fill,stroke='none'):
        s=' '.join(f'{x:.2f},{y:.2f}' for x,y in pts)
        self.parts.append(f'<polygon points="{s}" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')

    def circle(self,x,y,r,fill=TEAL,stroke='none'):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')

    def line(self,pts,color=BLUE,width=2,arrow=True,dashed=False):
        s=' '.join(f'{x},{y}' for x,y in pts);d=' stroke-dasharray="7 6"' if dashed else ''
        self.parts.append(f'<polyline points="{s}" fill="none" stroke="{color}" stroke-width="{width}"{d}/>')
        if arrow:
            x,y=pts[-1]
            for a,b in reversed(pts[:-1]):
                length=math.hypot(x-a,y-b)
                if length:
                    ux,uy=(x-a)/length,(y-b)/length;k=max(10,width*4)
                    self.polygon([(x,y),(x-k*ux-.48*k*uy,y-k*uy+.48*k*ux),
                                  (x-k*ux+.48*k*uy,y-k*uy-.48*k*ux)],color)
                    self.arrow_count+=1;break

    def text(self,x,y,t,size=24,color=NAVY,bold=False,width=None,center=False,line_height=1.40):
        lines=wrap(t,width,size,bold) if width else t.split('\n')
        fm=QFontMetricsF(font(size,bold))
        for i,line in enumerate(lines):
            yy=y+i*size*line_height;actual=fm.horizontalAdvance(line);left=x-actual/2 if center else x
            CHECKS.append(dict(page=self.n,text=line,x=left,y=yy,width=actual,size=size,
                               in_page=left>=0 and left+actual<=1600 and yy+size<=900))
            self.texts.append(line)
            self.parts.append(f'<text x="{x}" y="{yy+size}" font-family="Noto Sans CJK SC,Microsoft YaHei,sans-serif" font-size="{size}" font-weight="{700 if bold else 400}" fill="{color}" text-anchor="{"middle" if center else "start"}">{escape(line)}</text>')
        return len(lines)*size*line_height

    def box(self,x,y,w,h,title,body='',color=0,icon=None,size=24):
        fill,stroke=COLORS[color%len(COLORS)];self.rect(x,y,w,h,fill,stroke)
        compact=h<160
        padding=10 if compact else 16
        leading=1.27 if compact else 1.38
        ts=26 if compact or icon else 28
        size=min(size,22) if compact else size
        reserve=90 if icon else 12
        gap=5 if compact else 8
        # 标题与正文联合适配，不能仅缩正文后将标题挤出卡片。
        for _ in range(30):
            titleh=len(wrap(title,w-32,ts,True))*ts*leading
            bh=len(wrap(body,w-36,size))*size*leading if body else 0
            if padding+titleh+gap+bh+reserve<=h:break
            if size>21:size-=1
            elif ts>21:ts-=1
            else:raise ValueError(f'{self.n}: block overflow {title}')
        self.text(x+16,y+padding,title,ts,NAVY,True,width=w-32,line_height=leading)
        by=y+padding+titleh+gap
        if body:
            bh=self.text(x+18,by,body,size,NAVY,width=w-36,line_height=leading)
            if by+bh>y+h-reserve+1:raise ValueError(f'{self.n}: block overflow {title}')
        if icon:self.glyph(icon,x+22,y+h-82,w-44,70)

    def group(self,x,y,w,h,title,fill='#FFFFFF'):
        self.rect(x,y,w,h,fill,NAVY,True)
        tw=QFontMetricsF(font(27,True)).horizontalAdvance(title)
        self.rect(x+18,y-15,tw+24,36,'#FFFFFF','none')
        self.text(x+30,y-11,title,27,NAVY,True)

    def glyph(self,kind,x,y,w,h):
        # 可编辑示意：绿色为结构，红色为旧占用，蓝色为当前返回。
        if kind in ('points','ghost','clean','hit','voxel'):
            for i in range(12):
                for j in range(4):
                    self.circle(x+8+i*w/13,y+7+j*h/5,2.7,TEAL)
            if kind in ('ghost','clean'):
                for i in range(4):
                    for j in range(4):
                        px=x+w*.44+i*9;py=y+9+j*12
                        if kind=='ghost':self.circle(px,py,3.1,RED)
                        else:self.circle(px,py,3.1,'#FFFFFF','#D7E0E3')
            if kind=='voxel':
                for i in range(4):
                    for j in range(3):self.rect(x+w*.55+i*16,y+j*18,16,18,'none','#9BAAB6')
            if kind=='hit':
                for i in range(8):self.circle(x+w*.6+i*7,y+30,3,BLUE)
        elif kind=='ray':
            self.circle(x+10,y+h/2,8,BLUE)
            for j in range(3):self.line([(x+24,y+h/2),(x+w-15,y+15+j*23)],TEAL,1.8)
            for j in range(3):self.circle(x+w-9,y+15+j*23,4,BLUE)
        elif kind=='store':
            for i in range(3):
                self.rect(x+w*.24+i*12,y+i*12,w*.45,35,'#EDF5FF','#7B9AB4')
        elif kind=='clock':
            self.circle(x+w*.5,y+36,30,'#FFFFFF',BLUE)
            self.line([(x+w*.5,y+14),(x+w*.5,y+36),(x+w*.5+20,y+36)],BLUE,2,False)
        elif kind=='path':
            self.line([(x+10,y+58),(x+w*.35,y+58),(x+w*.35,y+15),(x+w*.75,y+15),(x+w-10,y+54)],TEAL,3)
        elif kind=='protect':
            self.rect(x+w*.38,y,70,68,'#E3F0E7',TEAL,True)
            for i in range(3):
                for j in range(3):self.circle(x+w*.38+12+i*22,y+12+j*22,3,BLUE)
        elif kind=='counter':
            for i in range(6):self.rect(x+8+i*w/6,y+22,w/6-9,28,COLORS[2][0],TEAL)

    def finish(self,out):
        self.parts.append('</svg>');raw='\n'.join(self.parts).encode()
        sv=out/'SVG'/f'{self.n:02d}_{self.name}.svg';pn=out/'PNG'/f'{self.n:02d}_{self.name}.png'
        sv.parent.mkdir(exist_ok=True);pn.parent.mkdir(exist_ok=True);sv.write_bytes(raw)
        renderer=QSvgRenderer(QByteArray(raw));assert renderer.isValid()
        im=QImage(3840,2160,QImage.Format_ARGB32);im.fill(0xFFFFFFFF)
        painter=QPainter(im);painter.setRenderHint(QPainter.Antialiasing);painter.setRenderHint(QPainter.TextAntialiasing)
        renderer.render(painter,QRectF(0,0,3840,2160));painter.end();assert im.save(str(pn))
        return dict(number=self.n,name=self.name,title=self.texts[0],text=self.texts,
                    sources=self.sources,svg=str(sv),png=str(pn),arrows=self.arrow_count)

def row(p,steps,y=214,h=410):
    count=len(steps);gap=38;w=(1480-gap*(count-1))/count
    for i,s in enumerate(steps):
        x=60+i*(w+gap);p.box(x,y,w,h,*s,color=i)
        if i<count-1:p.line([(x+w+3,y+h/2),(x+w+gap-3,y+h/2)])

def snake(p,steps):
    # 四个上排、三个下排；方向与用户提供的七步工作流程一致。
    top=[(60+i*378,230,346,218) for i in range(4)]
    bottom=[(1060,515,480,222),(560,515,460,222),(60,515,460,222)]
    for i,(s,pos) in enumerate(zip(steps,top+bottom)):
        p.box(*pos,*s,color=i,icon=['voxel','clock','hit','ray','protect','counter','store'][i],size=22)
    for i in range(3):p.line([(top[i][0]+349,339),(top[i+1][0]-3,339)])
    p.line([(1367,452),(1367,485),(1300,485),(1300,510)])
    p.line([(1055,626),(1023,626)]);p.line([(556,626),(523,626)])
    p.line([(58,626),(35,626),(35,189),(611,189),(611,224)])
    p.text(90,164,'处理下一帧；首次建立存储之后持续更新',22,BLUE)
