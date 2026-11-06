#!/usr/bin/env python3
"""原生矢量图文讲解：机制图不是实验图，历史图保留完整原件。"""
import json
import math
import zipfile
from PIL import Image, ImageDraw, ImageFont
from draw import Page, HERE, TEXTS, PAGES, NAVY, BLUE, TEAL, GRAY, RED, GOLD, CYAN, drone, shelf

E = HERE/'证据原件'
W = json.loads((E/'ours_cangku_20260927_121518_inflight_test.json').read_text())
X = json.loads((E/'ours_20260924_144552_manual_test.json').read_text())

def human(p,x,y,s=1,color=RED,cloud=False):
    if not cloud:
        p.circle(x,y-40*s,11*s,color)
        p.rect(x-11*s,y-24*s,22*s,34*s,color,'none')
        for a,b in [((-9,-18),(-22,8)),((9,-18),(22,8)),((-6,7),(-14,38)),((6,7),(14,38))]:
            p.line([(x+a[0]*s,y+a[1]*s),(x+b[0]*s,y+b[1]*s)],color,6*s,False)
    else:
        for j in range(19):
            for i in range(-5,6):
                width=2 if j<4 else (4 if j<13 else 3)
                if abs(i)<=width and (i+j)%2==0:
                    p.circle(x+i*4*s,y-48*s+j*4.6*s,2.5*s,color)

def dots(p,x,y,w,h,color=BLUE):
    for j in range(8):
        for i in range(11):
            if (i*3+j)%5:
                p.circle(x+i*w/10+math.sin(i+j)*2,y+j*h/7+math.cos(i*2-j)*2,2.8,color)

def label(p,x,y,t,color=GRAY,width=350):
    return p.text(x,y,t,20,color=color,width=width)

def illustration(p,x,y,w,h,kind):
    p.illustrations+=1
    if kind=='scan':
        shelf(p,x+w*.51,y+h*.22,w*.22,h*.48,True)
        human(p,x+w*.34,y+h*.42,.78,RED,True)
        drone(p,x+w*.15,y+h*.77,.75)
        for a in range(5):
            p.line([(x+w*.15,y+h*.77),(x+w*.48,y+h*(.2+.13*a))],TEAL,1.2,False)
    elif kind=='foreground':
        dots(p,x+12,y+25,w*.31,h*.52,GRAY)
        human(p,x+w*.79,y+h*.41,.92,GOLD,True)
        p.line([(x+w*.41,y+h*.47),(x+w*.61,y+h*.47)],BLUE,2.2)
        label(p,x+10,y+h*.80,'背景不重复输出',width=w*.48)
        label(p,x+w*.57,y+h*.80,'差异成为候选',GOLD,w*.43)
    elif kind=='cluster':
        for u,v,c in [(.2,.36,CYAN),(.62,.37,GOLD),(.78,.69,TEAL)]:
            dots(p,x+u*w-28,y+v*h-22,52,43,c)
            p.rect(x+u*w-35,y+v*h-29,70,60,'none',c,1.6,True)
        label(p,x+w*.1,y+h*.85,'邻近点连成一簇',width=w*.8)
    elif kind=='static':
        dots(p,x+15,y+25,w*.35,h*.51,TEAL)
        dots(p,x+w*.58,y+25,w*.33,h*.51,GOLD)
        p.line([(x+w*.35,y+h*.39),(x+w*.58,y+h*.39)],BLUE,2)
        p.circle(x+w*.76,y+h*.45,47,'none',TEAL,2)
        p.line([(x+w*.66,y+h*.26),(x+w*.86,y+h*.64)],RED,3,False)
        label(p,x+5,y+h*.81,'稳定地图支持 → 否决运动',width=w-10)
    elif kind=='match':
        for i in range(4):
            a=x+w*(.1+i*.23);b=y+h*(.65-.12*i)
            human(p,a,b,.44,GRAY if i<3 else RED)
            p.circle(a,b+30,5,TEAL)
            if i:p.line([(a-w*.23,b+h*.12+30),(a,b+30)],TEAL,2)
        label(p,x+5,y+h*.91,'预测位置附近，沿用同一编号',width=w-10)
    elif kind=='motion':
        good=[(x+w*.10,y+h*.55),(x+w*.35,y+h*.47),(x+w*.58,y+h*.40),(x+w*.84,y+h*.32)]
        jitter=[(x+w*.08,y+h*.82),(x+w*.22,y+h*.64),(x+w*.15,y+h*.89),(x+w*.28,y+h*.75)]
        p.line(good,TEAL,3)
        for a,b in good:p.circle(a,b,5,TEAL)
        p.line(jitter,RED,2.4,False)
        for a,b in jitter:p.circle(a,b,4,RED)
        label(p,x+5,y,'方向稳定，多帧一致',TEAL,w-10)
        label(p,x+w*.42,y+h*.72,'来回抖动不等于运动',RED,w*.57)
    elif kind=='display':
        p.rect(x+10,y+5,w*.42,h*.82,'#E7F1F7',BLUE)
        human(p,x+w*.2,y+h*.47,.72,RED,True)
        p.line([(x+w*.45,y+h*.40),(x+w*.57,y+h*.40)],BLUE,2.2)
        p.circle(x+w*.77,y+h*.40,26,RED,RED)
        p.circle(x+w*.74,y+h*.34,7,'#F6C5CF')
        label(p,x+4,y+h*.9,'确认点云',width=w*.42)
        label(p,x+w*.58,y+h*.9,'中心显示球',width=w*.4)

def architecture():
    p=Page(1,'模块架构','动态候选、聚类、跟踪与显示','先找与背景不同的点，再判断是否持续运动；输出同时服务显示、风险与地图维护。','机制示意｜运动检测不是人体识别，显示球不是碰撞包络。')
    for x,y,w,h in [(65,235,380,710),(490,225,940,745),(1475,235,380,710)]:
        p.rect(x,y,w,h,'none',BLUE,2,True)
    p.text(92,249,'输入与背景',30,bold=True)
    p.text(525,243,'候选提取与时序确认',30,bold=True)
    p.text(1510,249,'结果输出',30,bold=True)
    p.card(90,314,330,150,'当前帧扫描','用地图坐标下的本帧扫描；不是累计全局地图。',0)
    p.card(90,490,330,130,'无人机位姿','筛选周围6米及相对高度；近场规则按水平距离切换。',4)
    p.card(90,651,330,125,'稳定静态地图','给候选簇提供背景支持，抑制静态结构的假运动。',2)
    illustration(p,105,796,290,125,'scan')
    p.rect(515,305,890,210,'#FFF2C4',BLUE,1.6,True)
    p.card(537,328,360,160,'① 体素背景差异','初始40帧建立背景；未匹配背景的格子输出候选。',1)
    p.card(965,328,414,160,'② 邻近点聚成目标','0.55米连接邻近点；每簇4～1000点，计算中心。',0)
    p.line([(897,407),(965,407)],BLUE,2.5)
    p.rect(515,550,890,362,'#E8F3E8',BLUE,1.6,True)
    p.card(537,575,360,170,'③ 关联与运动历史','按预测位置匹配旧编号；多帧拟合速度，分别平滑位置和运动证据。',2)
    p.card(965,575,414,170,'④ 多条件确认','静态支持否决＋位移、速度、方向、拟合误差；连续4次合格才确认。',3)
    p.line([(1172,488),(1172,532),(713,532),(713,575)],BLUE,2.5)
    p.line([(897,652),(965,652)],BLUE,2.5)
    p.text(550,786,'跟踪状态：待确认 → 已确认 → 短时预测 → 丢失删除',24,color=TEAL,width=835)
    p.text(550,835,'跟踪由聚类节点完成，不另串接遗留跟踪器。',23,color=GRAY,width=815)
    for y,title,body,c in [(315,'目标中心与速度','编号、三维位置、速度、预测标志。',0),(505,'确认动态点云','输出本帧已确认簇中的点。',3),(695,'显示与下游使用','中心球供查看；对象供风险与地图模块使用。',2)]:
        p.card(1500,y,330,150,title,body,c)
    p.line([(420,395),(465,395),(465,405),(537,405)],BLUE,2.5)
    p.line([(420,554),(476,554),(476,455),(537,455)],BLUE,2)
    p.line([(420,712),(465,712),(465,690),(537,690)],TEAL,2)
    p.line([(1379,622),(1450,622),(1450,390),(1500,390)],BLUE,2.5)
    p.line([(1450,622),(1450,580),(1500,580)],RED,2)
    p.line([(1450,622),(1450,770),(1500,770)],TEAL,2)
    p.save()

def seven_steps():
    p=Page(2,'图文七步流程','动态感知工作流程','从当前点云到持续运动目标：参考四上三下的卡片布局，每一步配工程场景插图。','机制示意｜初始化背景只执行一次；聚类、关联、确认和显示逐帧更新。')
    top=[(70,255,412,315),(523,255,412,315),(976,255,412,315),(1429,255,412,315)]
    bottom=[(70,666,560,323),(675,666,560,323),(1280,666,560,323)]
    steps=[(top[0],'1　当前扫描与位姿','地图坐标下看本帧；筛选近场点，降低数据量。',0,'scan'),
           (top[1],'2　与初始背景比较','初始40帧形成背景；与背景不同的格子成为候选。',1,'foreground'),
           (top[2],'3　邻近点聚成簇','按0.55米邻近连接；太小或过大的点簇不参与。',2,'cluster'),
           (top[3],'4　检查静态支持','若多数点贴合稳定地图，本次运动判断不通过。',3,'static'),
           (bottom[2],'5　关联历史并估计运动','预测位置附近沿用旧编号；通过历史中心拟合速度。',4,'match'),
           (bottom[1],'6　连续运动才确认','多项条件合格4次才确认；持续3次不合格则取消动态状态。',5,'motion'),
           (bottom[0],'7　输出目标、点云与显示','观测目标显示红球；短时预测显示橙球；旧标记及时删除。',2,'display')]
    for (x,y,w,h),title,body,c,kind in steps:
        end=p.card(x,y,w,h,title,body,c,22)
        illustration(p,x+22,end+17,w-44,y+h-end-55,kind)
    for a,b in zip(top,top[1:]):p.line([(a[0]+a[2],412),(b[0],412)],BLUE,2.5)
    p.line([(1635,570),(1635,619),(1560,619),(1560,666)],BLUE,2.5)
    p.line([(1280,824),(1235,824)],BLUE,2.5)
    p.line([(675,824),(630,824)],BLUE,2.5)
    p.line([(70,827),(43,827),(43,216),(729,216),(729,255)],BLUE,2.5)
    p.text(242,188,'继续处理下一帧',20,color=BLUE)
    p.save()

def candidates():
    p=Page(3,'候选不等于动态','候选点不等于动态点','“新看到”与“真的在移动”要分开：雷达反复扫描、视角变化和无人机移动都可能带来候选。','当前规则｜不永久吸收在线前景；候选多一些，后级再以运动与静态支持筛选。')
    cols=[(70,'① 统一坐标，再比较背景',0),(675,'② 新出现的静态边缘',1),(1280,'③ 连续运动的目标',2)]
    bodies=['使用桥接后的当前扫描，静态物体理论上保持地图坐标不变；位姿误差仍会带来偏差。',
            '初始背景只覆盖当时看到的区域。新进入视野的货架或边缘，也可能成为候选。',
            '行人的候选点先聚成簇，再经历史运动确认；不是候选一出现就认定为人。']
    for (x,title,c),body in zip(cols,bodies):p.card(x,241,560,675,title,body,c,24)
    # 场景一：两个机位观察同一静态货架。
    shelf(p,330,494,135,203,True);drone(p,185,750,.94);drone(p,469,771,.94)
    p.line([(185,750),(368,620)],TEAL,2);p.line([(469,771),(368,620)],TEAL,2)
    p.text(105,814,'自身移动不直接等于物体移动',23,color=TEAL,width=490)
    # 场景二：灰色初始背景和新黄色回波。
    dots(p,714,490,185,218,GRAY);dots(p,990,490,160,218,GOLD)
    p.line([(917,602),(972,602)],BLUE,2.5)
    p.text(719,735,'初始背景',22,color=GRAY)
    p.text(997,735,'新增候选',22,color=GOLD)
    p.text(707,814,'静态货架也可能进入候选',23,color=GOLD,width=490)
    # 场景三：人体候选与确认轨迹。
    for i in range(3):
        human(p,1398+140*i,610,.95,GOLD if i<2 else RED,True)
    p.line([(1375,718),(1540,697),(1698,675)],TEAL,3)
    p.text(1320,768,'历史运动一致 → 才确认',25,color=TEAL,width=480)
    p.text(1320,819,'没有人体语义分类网络',23,color=GRAY,width=480)
    p.illustrations+=3
    p.rect(70,938,1770,66,'#EDF5FF','none')
    p.text(91,950,'检测窗口：无人机周围6米（三维距离）｜相对高度 −0.60～+1.20米｜体素边长0.25米',25,width=1710)
    p.save()

def confirmation():
    p=Page(4,'静态否决与运动确认','静态支持与连续运动确认','不只看中心有没有跳动：先排除稳定结构，再检查一段历史是否像连续运动。','规则边界｜慢速、急转、遮挡、背景混合或静态支持过强，都可能造成漏检。')
    p.card(70,238,640,345,'第一道：稳定静态地图否决','稳定地图至少30点才启用；簇内至少60%的点在0.35米内有背景支持，则本次运动判断不通过。',2,23)
    illustration(p,100,399,550,135,'static')
    p.card(70,615,640,357,'第二道：短历史的多项证据','最多保留12个历史中心；默认看水平移动，避免扫描点上下变化直接触发运动确认。',0,23)
    illustration(p,106,770,549,140,'motion')
    p.rect(760,238,1080,734,'#FFF7DE',BLUE,1.8,True)
    p.text(790,257,'每次判断必须同时满足下列条件',30,bold=True,width=1010)
    p.text(793,315,'条件',24,bold=True)
    p.text(1206,315,'一般区域',24,bold=True)
    p.text(1525,315,'近场＜3米',24,bold=True)
    rows=[('历史跨度足够','至少0.50秒','至少0.45秒'),('水平总位移足够','大于0.20米','大于0.18米'),('水平拟合速度足够','大于0.30米/秒','大于0.32米/秒'),('运动方向较一致','至少0.82','至少0.80'),('匀速拟合误差较小','不超过0.10米','不超过0.10米')]
    for i,(a,b,c) in enumerate(rows):
        y=364+i*66;p.rect(781,y-2,1035,60,'white' if i%2==0 else '#FFFBEF','none')
        p.text(797,y+8,a,23,width=370)
        p.text(1205,y+8,b,23,width=280)
        p.text(1519,y+8,c,23,width=288)
    p.text(794,711,'另须：拟合速度有效，且三维速度小于3米/秒。',24,color=GRAY,width=1000)
    p.line([(1185,758),(1185,788)],BLUE,2.5)
    p.card(795,792,450,143,'连续4次合格','才把该轨迹确认为动态目标。',3,23)
    p.card(1293,792,500,143,'连续3次不合格','已确认目标取消动态状态，不代表实体消失。',0,23)
    p.save()

def tracking_display():
    p=Page(5,'跟踪与显示规则','跟踪与显示：既要跟得上，也不要留下假球','位置输出快速跟随；判定历史单独平滑。观测、短时预测和丢失删除分别处理。','显示球直径固定0.3米｜不是实测尺寸；对象尺寸字段也只是0.60×0.60×1.20米默认值。')
    boxes=[(70,242,555,376),(660,242,555,376),(1250,242,590,376)]
    states=[('① 本帧重新观测到','在旧目标预测位置附近找最近中心；每个旧轨迹本帧最多匹配一次。',0),
            ('② 暂时没有观测到','仅对已确认目标外推位置，最多容忍4次丢失回调；不会凭空生成动态点云。',1),
            ('③ 持续丢失或取消确认','第5次丢失回调删除轨迹；未输出的旧显示编号显式删除。',2)]
    for (x,y,w,h),(t,b,c) in zip(boxes,states):p.card(x,y,w,h,t,b,c,24)
    human(p,244,489,.98,RED,True);p.circle(415,476,33,RED);p.circle(402,465,8,'#F6C7CC')
    p.line([(287,481),(364,481)],BLUE,2.5);label(p,138,561,'红球：本帧观测到的确认目标',RED,450)
    human(p,815,489,.98,GRAY,True);p.line([(850,479),(1020,479)],GOLD,2.5,True,True)
    p.circle(1053,479,33,'#FF9900');label(p,699,561,'橙球：短时外推，带预测标志',GOLD,480)
    p.circle(1488,479,37,'none',GRAY,2)
    p.line([(1458,449),(1518,509)],RED,3,False);p.line([(1458,509),(1518,449)],RED,3,False)
    label(p,1570,452,'旧编号删除',TEAL,220);label(p,1305,561,'标记0.5秒过期（按仿真时钟）',TEAL,490)
    p.illustrations+=3
    p.line([(625,438),(660,438)],BLUE,2.5)
    p.line([(1215,438),(1250,438)],BLUE,2.5)
    p.card(70,666,850,313,'跟踪更新规则','按“位置容差＋可行移动距离”缩放匹配门限；一般最多0.80米，近场最多1.00米。\n中心平滑权重0.65；运动历史与速度各0.20，分别兼顾跟随和抗抖动。',4,25)
    p.card(965,666,875,313,'显示更新规则','球的编号使用轨迹编号，而非数组顺序；球位置来自对象输出。\n聚类与显示输入队列各保留1条；每次消息更新球，未出现编号删除，0.5秒自动过期。',5,25)
    p.save()

def real_screens():
    p=Page(6,'仓库历史实景与显示','仓库历史实景与显示结果','复用2026年9月27日保存的原截图：完整保留画面，不把示意图冒充实验。','证据边界｜截图只展示一个时刻；检测持续性看原测试数据，不能由红色小区域推出识别精度。')
    p.rect(70,230,1180,737,'#F7F9FC','#C5D6E0')
    p.image(E/'ours_cangku_20260927_121518_rviz.png',80,244,1160,725)
    p.image(E/'ours_cangku_20260927_121518_gazebo.png',1290,241,545,288)
    p.text(1300,548,'同场次仓库仿真原画面',24,bold=True,width=540)
    p.card(1285,604,555,193,'图中如何分辨','绿点：维护导航地图\n黄点：局部障碍地图\n红点：已确认的动态候选簇点',2,23)
    p.card(1285,825,555,149,'不能混淆的内容','红点不是人体语义标签；中心球不是人体形状或安全半径。',3,23)
    p.illustrations+=2;p.save()

def real_counts():
    p=Page(7,'重复横穿的真实数据','暂停后再运动：历史回归数据','按保存的测试数据重绘；计数是匹配到行人附近的对象消息，不是逐点分割准确率。','历史功能证据｜本轮未重飞；不同场景和日期不作公平性能排名，当前参数变化须单独复测。')
    p.rect(70,230,1050,717,'#F7FAFC','#B4C9D6')
    p.text(102,250,'仓库：首次横穿与暂停后的反向横穿',29,bold=True,width=980)
    p.rect(115,313,22,22,TEAL,'none');p.text(148,307,'本帧观测到',22,color=TEAL)
    p.rect(407,313,22,22,GOLD,'none');p.text(440,307,'仅短时预测',22,color=GOLD)
    p.rect(703,313,22,22,'#E3E8ED','none');p.text(736,307,'未匹配到对象',22,color=GRAY)
    for y,key,title in [(405,'cross_first','首次横穿'),(572,'cross_repeat','暂停后反向横穿')]:
        r=W['phases'][key];obs=r['observed'];pred=r['detected']-obs;miss=r['messages']-r['detected']
        p.text(113,y,title,27,bold=True,width=900)
        x=116;full=900;yy=y+50
        for count,color in [(obs,TEAL),(pred,GOLD),(miss,'#E3E8ED')]:
            ww=full*count/r['messages']
            if ww:p.rect(x,yy,ww,44,color,'none')
            x+=ww
        p.text(115,y+102,f"观测 {obs}　预测 {pred}　未匹配 {miss}　共 {r['messages']} 条消息",24,width=925)
    p.text(110,774,'两阶段都有实际观测输出，但检测并不连续。',27,color=TEAL,bold=True,width=950)
    p.text(110,825,'附近0.9米的中心关联仅是评估代理；没有逐点真值，不能据此计算分割召回率。',23,color=GRAY,width=945)
    p.card(1165,230,675,341,'另一场景：9月24日重复横穿','首次 / 反向横穿实际观测：50 / 42 条\n匹配中心水平误差中位数：0.081 / 0.142 米\n消息时间戳年龄95分位：0.02695 / 0.026 秒\n来自9月24日示例场景，不是仓库数据。',0,24)
    p.card(1165,605,675,342,'这些结果说明什么','支持：初期展示后，目标暂停再运动仍有确认输出。\n不足：检测有间断；消息计数不是精确率。仿真时间戳年龄不是计算耗时或首次确认时延。',1,24)
    p.save()

def preview():
    files=sorted((HERE/'PNG').glob('*.png'))
    im=Image.new('RGB',(1920,1240),'#EEF2F5');d=ImageDraw.Draw(im)
    f=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',25)
    for i,path in enumerate(files):
        col,row=i%3,i//3;x=25+col*635;y=22+row*407
        thumb=Image.open(path).convert('RGB');thumb.thumbnail((610,344))
        im.paste(thumb,(x,y));d.text((x,y+354),path.stem,font=f,fill=NAVY)
    im.save(HERE/'00_七页总预览.png')

def package():
    out=HERE.parent/'动态候选聚类跟踪显示_图片与证据合集.zip'
    with zipfile.ZipFile(out,'w',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(HERE.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                z.write(path,str(path.relative_to(HERE.parent)))
    print('ZIP',out,out.stat().st_size)

if __name__=='__main__':
    architecture();seven_steps();candidates();confirmation();tracking_display();real_screens();real_counts()
    preview()
    (HERE/'版面核验信息.json').write_text(json.dumps(dict(pages=PAGES,texts=TEXTS),ensure_ascii=False,indent=2))
    print('已生成7张4K图片与7张可编辑矢量图；未启动ROS。')
    package()
