#!/usr/bin/env python3
"""独立生成16张地图维护讲解图片，不生成PPT，不修改飞行程序。"""
import ast,hashlib,json,math,re
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile,ZIP_DEFLATED
import yaml
from PIL import Image,ImageDraw,ImageFont
from draw import Page,row,snake,CHECKS,NAVY,BLUE,TEAL,GRAY,RED

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
B=ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
REF=Path('/home/a/下载/fastlio2ppt.pptx')
SOURCES=['src/static_map_builder.cpp','config/static_map.yaml','launch/perception.launch',
         'launch/astra_dynamic_avoidance.launch','launch/warehouse_exploration.launch',
         'src/static_path_planner.cpp','config/path_planner.yaml',
         'scripts/warehouse_explorer.py','scripts/exploration_grid.py',
         'scripts/save_navigation_map.py','scripts/run_dynamic_comparison.py']
FILES=[REF]+[B/s for s in SOURCES]
NOTES=[];PAGES=[]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def new(n,name,title,sub,rule,notes,sources=None):
    NOTES.append((title,notes));return Page(n,name,title,sub,rule,sources or SOURCES[:5])
def done(p):PAGES.append(p.finish(HERE))

def generate():
    p=new(1,'地图维护总架构','可撤销旧点：让地图能承认环境变化',
          '新扫描提供现在的事实；旧地图保存历史，但旧占用不是永久结论。',
          '维护导航地图，不自动回写FAST-LIO内部匹配树。',
          '模块接收map坐标系当前注册扫描和FAST-LIO位姿，维护一份哈希体素存储，再派生持续、稳定和局部占用视图。普通模式还接收确认运动目标；探索模式不以目标轨迹直接授权地图删除。地图更新在bridge内进行，不修改FAST-LIO状态估计器的内部ikd-Tree。当前注册扫描、自由观测和维护地图共同服务规划；风险保护仍检查当前障碍。')
    p.group(60,216,290,510,'输入与观测')
    p.box(82,264,246,114,'当前扫描','同一地图坐标',0)
    p.box(82,431,246,114,'位姿与时间','定位、射线起点',1)
    p.box(82,598,246,105,'运动目标','仅普通版清理',3,size=22)
    p.group(390,198,748,544,'地图维护核心')
    p.rect(415,250,695,207,'#FFF2BF',NAVY,True)
    p.text(432,263,'新点整理与观测证据',28,NAVY,True)
    p.box(438,314,278,117,'小格聚合','一帧同格算一次',0,size=23)
    p.box(777,314,302,117,'射线与保护','确认旧位置已空',2,size=23)
    p.line([(719,372),(773,372)])
    p.rect(415,500,695,205,'#E5F1E4',NAVY,True)
    p.text(432,512,'一份可修改的存储',28,TEAL,True)
    p.box(438,568,278,111,'跨帧命中','更新代表点与时间',2,size=22)
    p.box(777,568,302,111,'有条件撤销','删除失效旧格子',5,size=23)
    p.line([(580,434),(580,564)],TEAL)
    p.line([(930,434),(930,564)],TEAL)
    p.line([(773,627),(720,627)],TEAL)
    p.group(1170,216,370,510,'结果与使用')
    p.box(1193,265,324,114,'多种地图视图','共享更新、不同筛选',4)
    p.box(1193,432,324,114,'规划与探索','重新审视可走路线',2)
    p.box(1193,599,324,105,'显示与保存','保存真实维护快照',0,size=22)
    p.line([(332,321),(374,321),(374,372),(434,372)])
    p.line([(332,488),(377,488),(377,472),(1020,472),(1020,435)])
    p.line([(332,650),(363,650),(363,718),(987,718),(987,683)],GRAY,1.5,True,True)
    p.text(410,724,'仅普通版按轨迹清理；探索版不走这条删除支路',20,GRAY)
    p.line([(1114,620),(1152,620),(1152,322),(1189,322)],TEAL)
    p.line([(1355,383),(1355,428)]);p.line([(1355,550),(1355,595)])
    done(p)

    p=new(2,'鬼影问题与撤销动机','为什么只过滤新点还不够？',
          '人先站着、后来离开：留下的问题在旧地图，不只在当前这一帧。',
          '不再加入新的人体点，并不等于旧的人体点已经消失。',
          '当人在初始观测中静止，其返回点可多次入图。人移动以后，即使当前运动检测正确，旧位置仍可能在持续地图中形成假墙。该模块解决的是历史占用能否被撤销，而不是识别所有人体类别。各图红色点只是旧占用示意，不是实际分割结果。')
    row(p,[('1 人原本不动','反复扫到人体，旧位置进入地图。'),('2 人已经离开','当前扫描改变，历史点还留在原处。'),('3 新观测来核实','不是凭空删除，而是观察旧位置是否已经空了。'),('4 地图与路线更新','撤销满足条件的旧格子，再重新评估路线。')],h=420)
    for i,k in enumerate(['points','ghost','ray','clean']):p.glyph(k,85+i*378,555,300,74)
    p.text(76,690,'绿点：环境结构    红点：旧占用    空心点：已撤销的位置（示意）',25,GRAY)
    done(p)

    p=new(3,'小格子中的地图记忆','地图记住的不只是一个点',
          '把空间分成15厘米小格，每个格子保留代表位置和观测记录。',
          '同一输入帧的密集点不会把一个格子立刻变成稳定结构。',
          '体素边长0.15米。先将单次扫描同一格子的所有有效点求平均，再对cells_中的格子增加一次hits并更新last_frame与last_seen，free_misses重置为0。代表点使用随hits变化的更新权重；超过20次后仍继续以1/20权重更新，不是永久冻结。源码按处理回调增加帧计数，没有按唯一采集时间戳去重，因此“跨帧”不代表已严格验证消息都独立。')
    p.group(60,211,570,520,'当前一帧')
    p.box(90,263,510,180,'许多点落进同一小格','先求本帧代表位置，再更新一次命中。',0)
    p.glyph('voxel',120,475,440,135)
    p.line([(637,440),(696,440)])
    p.group(710,211,830,520,'格子的记录')
    for i,(a,b) in enumerate([('代表位置','随新观测逐步更新'),('命中次数','用于持续与稳定筛选'),('最近命中时间','判断是否刚被重新看到'),('已空证据次数','达到条件才授权撤销')]):
        x=738+(i%2)*402;y=266+(i//2)*220
        p.box(x,y,371,180,a,b,i)
    done(p)

    p=new(4,'七步循环更新流程','地图维护的七步循环工作流程',
          '沿用参考图的上排前进、下排返回结构；每来一帧，再检查一次。',
          '先写入当前占用，再判断哪些旧占用与新观测发生矛盾。',
          '探索版cloudCallback先检查坐标和时间配对；新格聚合后更新存储并清零其空证据，再运行updateEvidenceFreeSpace。有删除证据的旧格更新计数，达到3或6次门限后erase。最后容量限制及地图输出筛选。定时发布不是新观测，不能给旧格凭空增加证据。初始化随第一批合格扫描自然发生，不是原FAST-LIO ikd-Tree的初始化。')
    snake(p,[('1 建立小格存储','初始合格扫描建立记录'),('2 检查扫描与位姿','探索需时间配对'),('3 写入当前占用','更新命中、重置空证据'),('4 生成射线证据','近距离真实返回点'),('5 检查误删保护','当前回波、点线距离'),('6 累积并撤销旧格','满足次数与时效才删'),('7 输出多种地图','持续、稳定、局部视图')])
    done(p)

    p=new(5,'一份存储四种视图','不是四张互不相关的地图',
          '同一份存储生成三种占用视图；自由观测来自最新一帧射线。',
          '删除共享存储里的旧格后，下一次发布时各占用视图一起改变。',
          '持续地图minimum_static_hits=2；稳定视图stable_minimum_static_hits=8；局部视图local_minimum_hits=1，同时限制水平半径6米、last_frame不少于frame_index-15且free_misses为0。边界帧包含在内，不能机械等同精确1.5秒。局部仅是发布筛选，出局部半径不会自动删除持续图。自由视图latest_free_cells_是当前射线遍历集合，与cells_不同，且比严格持久删除候选范围宽。',
          ['src/static_map_builder.cpp','config/static_map.yaml'])
    p.group(60,219,450,506,'共享地图记忆')
    p.box(90,275,390,300,'一份可修改存储','记录命中、最近观测和空证据。\n删除不是只隐藏显示层。',2,icon='store')
    p.line([(513,428),(559,428)])
    p.group(580,219,960,506,'不同用途的输出')
    for i,(a,b) in enumerate([('持续地图：至少2次','显示与保存历史环境'),('稳定地图：至少8次','普通规划与静态支持'),('局部占用：至少1次','6米内、近15帧、无空证据'),('当前自由观测','为规划提供局部修正线索')]):
        x=608+(i%2)*460;y=267+(i//2)*219;p.box(x,y,430,185,a,b,i,size=22)
    done(p)

    p=new(6,'普通版运动轨迹清理','普通演示：沿确认运动轨迹回溯清理',
          '这是保留的普通ours规则，不要混同为探索版的证据删除。',
          '轨迹能够快速清旧点，但误跟踪可能连带擦掉邻近静态结构。',
          'enable_dynamic_map_clearing启用时，普通版记录确认目标最近20秒的中心轨迹，并沿当前速度向后回溯1.5秒，以0.15米采样步长检查。新点掩膜半径0.65米、半高1米；直接删除体素循环还加一格水平余量，并以离散格高度循环扩展，不能把实际删除区精确称为0.65米圆柱。该版本同时抑制轨迹附近新点入图并擦除旧格，没有按稳定命中次数保护货架。objectsCallback不将predicted样本记入轨迹；latest_objects其他遍历仍依赖上游传入真实确认对象。')
    row(p,[('1 确认运动目标','读取位置、速度和实际已走轨迹。'),('2 形成回溯区域','保留20秒轨迹，另向后回溯1.5秒。'),('3 不入图并擦旧点','屏蔽邻近新点，直接删除旧格。'),('4 保留模式边界','清理快，但假动态与轨迹范围可能误伤货架。')],h=430)
    for i,k in enumerate(['points','path','ghost','protect']):p.glyph(k,85+i*378,570,300,72)
    p.text(73,702,'探索版：不使用这条轨迹圆柱直接删除，也不按轨迹范围禁止新点入图。',25,TEAL)
    done(p)

    p=new(7,'探索版观测证据架构','探索演示：删除权交给新的观测证据',
          '“目标可能在运动”是线索；“旧位置被真实回波射线穿过”才进入删除检查。',
          '轨迹不是删除许可；没有足够的新观测就保留旧占用。',
          'evidence_only_clearing=true时clearDynamicVoxels直接返回，cloudCallback不再以运动掩膜拒绝新点。所有实际返回点都可以保护当前表面，包括运动目标范围内的返回。射线起点由扫描时间附近位姿与旋转后的雷达到惯导平移外参组成，范围只跟踪到回波前0.30米、最多6米。射线采样走过的小格只是候选，还要对旧代表点逐项检查。')
    p.group(60,218,330,510,'输入')
    p.box(83,274,284,150,'当前真实返回','包含当前人体返回',0)
    p.box(83,496,284,150,'配对位姿与外参','构造扫描射线起点',1)
    p.group(440,202,750,541,'证据与撤销')
    p.rect(465,251,698,188,'#FFF2BF',NAVY,True)
    p.box(488,286,290,118,'近场射线','只取回波前空间',1)
    p.box(837,286,300,118,'旧点矛盾检查','旧点确实靠近射线',3)
    p.line([(782,345),(833,345)])
    p.rect(465,486,698,216,'#E5F1E4',NAVY,True)
    p.box(488,539,290,133,'当前表面保护','邻格回波、命中重置',2)
    p.box(837,539,300,133,'有时效的次数确认','3次或6次后撤销',5)
    p.line([(987,408),(987,535)],TEAL);p.line([(782,605),(833,605)],TEAL)
    p.group(1230,218,310,510,'输出')
    p.box(1250,272,270,150,'修正后的地图','旧格可从多视图撤销',2)
    p.box(1250,499,270,148,'当前障碍仍有效','清图不替代即时避障',4)
    p.line([(370,349),(484,349)]);p.line([(370,570),(415,570),(415,449),(630,449),(630,408)])
    p.line([(1141,605),(1204,605),(1204,347),(1246,347)],TEAL)
    p.line([(1385,426),(1385,495)])
    done(p)

    p=new(8,'射线与遮挡示意','看不到，不等于这里已经空了',
          '只有回波前、确实经过旧点附近的射线，才能提供撤销线索。',
          '遮挡后方、偏离射线、没有有效返回：都不能凭缺点删除。',
          '本页为俯视机制示意。有效返回位于旧点之后，旧代表点投影必须在射线有效段内且点线距离不超过0.06米。若回波在旧点前方，则后方旧点被遮挡；若射线从旁边经过，虽可能落进同一15厘米格子，也不授权删该代表点。回波端点前0.30米留保护，不把测量表面直接当自由；最大6米截断限制有效删除范围。')
    p.group(60,215,1480,518,'三种观测必须区分')
    for i,(a,b) in enumerate([('1 支持已经空了','旧点在有效射线附近'),('2 前方存在遮挡','回波前就遇到实体'),('3 从旧点旁边扫过','同格不代表击穿旧点')]):
        x=82+i*492;p.box(x,260,450,410,a,b,i)
        yy=495;p.circle(x+45,yy,12,BLUE)
        end=x+396
        if i==0:
            p.line([(x+62,yy),(end,yy)],TEAL,3);p.circle(x+233,yy,7,RED);p.circle(end,yy,8,BLUE)
            p.text(x+45,550,'雷达',20,GRAY,center=True)
            p.text(x+233,550,'旧点',20,GRAY,center=True)
            p.text(end,550,'新回波',20,GRAY,center=True)
        elif i==1:
            p.line([(x+62,yy),(x+188,yy)],TEAL,3);p.rect(x+188,yy-34,14,68,'#D5E0E6',NAVY)
            p.line([(x+205,yy),(end,yy)],GRAY,2,False,True);p.circle(x+310,yy,7,RED)
            p.text(x+45,550,'雷达',20,GRAY,center=True)
            p.text(x+195,550,'当前障碍',20,GRAY,center=True)
            p.text(x+324,550,'遮挡后旧点',20,GRAY,center=True)
        else:
            p.line([(x+62,yy),(end,yy)],TEAL,3);p.circle(x+233,yy+45,7,RED);p.circle(end,yy,8,BLUE)
            p.line([(x+233,yy),(x+233,yy+40)],RED,1.5,False,True)
            p.text(x+90,571,'偏离太远，保留旧点',22,GRAY)
    done(p)

    p=new(9,'旧点删除判断流程','一个旧格子，到底能不能删除？',
          '先过几何与表面保护，再过时间与次数；不满足就保留。',
          '“当前没扫到”只是缺测，不是直接删除条件。',
          '探索版先检查有无配对位姿、有效时间与合法射线，再检查旧点位于回波前有效段且点线距离≤0.06米。当前返回端点同格跳过，27邻域有当前回波时保护旧格。距离最后命中不足0.5秒不累积删除计数。有效负证据之间间隔超过1秒或非正则先清零计数，然后按普通3次、已稳定6次判断。未观测帧不会自己加次数，也不自动清零；在下一次有效矛盾到来时才检查间隔。')
    p.box(70,235,330,153,'旧格与新射线','得到候选，不是立即删除',0)
    p.line([(404,311),(476,311)])
    p.polygon([(670,218),(850,311),(670,404),(490,311)],'#FFF2BF',BLUE)
    p.text(670,271,'几何和表面\n保护通过？',25,NAVY,True,width=220,center=True)
    p.line([(854,311),(935,311)])
    p.text(875,270,'是',22,TEAL)
    p.box(939,236,560,152,'检查最近命中与证据间隔','刚被扫到则不计数；旧证据过期则重算。',1)
    p.line([(1219,392),(1219,474)])
    p.polygon([(1219,478),(1398,568),(1219,658),(1040,568)],'#E5F1E4',TEAL)
    p.text(1219,537,'次数达到\n3次或6次？',25,NAVY,True,width=230,center=True)
    p.line([(1036,568),(944,568)]);p.text(969,525,'是',22,TEAL)
    p.box(555,493,385,150,'撤销共享存储旧格','下一次地图发布反映删除',2)
    p.box(70,497,330,150,'保留旧占用','缺证据、受保护或次数不足',4,size=22)
    p.line([(670,408),(670,451),(235,451),(235,493)],GRAY);p.text(548,411,'否：保留',22,GRAY)
    p.line([(1219,662),(1219,713),(235,713),(235,651)],GRAY);p.text(940,718,'否：继续等待',21,GRAY)
    done(p)

    p=new(10,'命中与空证据时间线','稳定点也能撤销，但要更多证据',
          '稳定只是更难被删除，不代表永久可信，也不是人体类别判定。',
          '命中就清零；过期证据不能无限拼凑成删除授权。',
          'hits≥8视为稳定，删除阈值为6次；不足8为普通格，删除阈值为3次。所有计数都需要当前射线几何矛盾、邻域保护通过以及距last_seen≥0.5秒。下方时间线为假设例子，非真实扫描日志：先命中再经过至少0.5秒等待，连续符合条件的6次负证据会删除稳定格。每次以采集stamp检查间隔，超过1秒清零再累计。不是“人离开0.6秒必删”，也不是必须严格相邻六帧；采样密度和遮挡会延长等待。')
    p.box(60,213,700,168,'普通旧格：3次有效空证据','少量观测形成的占用，较早进入撤销。',1)
    p.box(800,213,740,168,'稳定旧格：6次有效空证据','至少8次命中，更谨慎地撤销旧占用。',2)
    p.group(60,435,1480,277,'稳定格的假设更新序列')
    labels=['重新命中','等待满0.5秒','空证据1','空证据2','空证据3','空证据4','空证据5','空证据6']
    for i,label in enumerate(labels):
        x=81+i*181;p.box(x,492,159,114,label,'清零' if i==0 else ('暂不计数' if i==1 else ('删除' if i==7 else '保留')),i,size=22)
        if i<7:p.line([(x+162,549),(x+176,549)])
    p.text(87,640,'若中途再次命中 → 从零开始；若有效证据间隔超过1秒 → 下一次先清零。',25,TEAL)
    done(p)

    p=new(11,'避免误删静态结构','保护货架、薄结构与真实表面',
          '不因为旁边有人被误判运动，就把一整段货架擦掉。',
          '先保护当前返回，再要求真实射线矛盾，稳定格使用更高删除门限。',
          '探索模式解除轨迹直接擦除和轨迹入图拒绝，是针对历史静态误清风险的规则选择。当前返回点所在格及其27邻域可保护旧格；旧点必须靠近实际射线，不是射线穿过同格就删除；距最后命中0.5秒的等待和稳定6次阈值提供额外缓冲。保护不是完整概率模型或绝对保证；位姿误差、稀疏采样、表面边缘和遮挡依然可能影响结果。')
    row(p,[('1 轨迹不授权擦除','探索不沿目标圆柱直接删除货架。'),('2 当前表面优先','旧格周围27个邻格有回波就保护。'),('3 射线确实矛盾','旧代表点距离射线不超过6厘米。'),('4 稳定格更谨慎','最近命中有等待；稳定格需6次证据。')],h=425)
    for i,k in enumerate(['ghost','protect','ray','counter']):p.glyph(k,85+i*378,569,300,72)
    p.text(73,699,'这些是降低误删风险的工程规则，不是“所有静态表面永远不缺点”的保证。',25,GRAY)
    done(p)

    p=new(12,'人先静止后移动','人从静止到离开，地图如何变化？',
          '先承认当前占用，再核实旧位置消失；不需要一开始就知道“这是人”。',
          '静止的人仍是障碍；人离开后的旧占用才是要撤销的内容。',
          '本页讲探索证据模式。阶段一静止人体回波可以入图，多次命中后甚至进入稳定视图；阶段二人体移动时新位置仍是当前占用，不按轨迹直接从扫描中抹掉；阶段三重新看见旧位置后，只有有效穿越射线才能累积负证据；阶段四共享格删除后维护地图更新。没有再观测、旧位置被货架挡住或只见到附近空隙时，旧人体占用可能仍在。长期背景去人体与即时安全障碍是两个任务。')
    row(p,[('1 原地站立','当前回波占空间，可以进入稳定地图。'),('2 开始移动','新位置仍是障碍，风险保护继续工作。'),('3 回看旧位置','有效射线、多次已空证据，逐步核实。'),('4 撤销旧格','原位置不再长期挡路；现有人仍不能忽略。')],h=425)
    for i,k in enumerate(['points','ghost','ray','clean']):p.glyph(k,85+i*378,569,300,72)
    p.text(72,699,'旧位置被遮挡时：保留未确认占用；需要安全地换视角再观察，而不是强行擦除。',25,TEAL)
    done(p)

    p=new(13,'地图变化与路线更新','地图更新，要让路线也重新审视',
          '地图清得干净只是中间结果；最终目的是不再被历史假墙一直挡住。',
          '先在三维旧点上用自由证据修正，再膨胀障碍，最后叠加当前占用。',
          '普通规划器以稳定图作为历史主地图，在局部阶段用free点对原始三维占用近邻匹配，剔除匹配的旧点后再做二维高度带膨胀；当前local占用最后回填。不能对自由点在二维大范围擦除已膨胀障碍。持久删除等待3/6次，而局部自由话题能更早影响路径；latest_free_cells并不等于经过严格持久删除检查的evidence集合，不能声称两者同样保守。探索不用通用A*节点作为航点发布者，而由自身接收维护图、扫描与自由观测构造探索栅格。',
          ['src/static_map_builder.cpp','src/static_path_planner.cpp','scripts/warehouse_explorer.py','scripts/exploration_grid.py','launch/warehouse_exploration.launch'])
    p.group(60,212,510,510,'长期地图修正')
    p.box(88,263,454,174,'满足多次证据再删','共享存储删除旧占用',2,icon='clean')
    p.box(88,511,454,160,'持续与稳定地图更新','历史参考不再永远保留假墙',4)
    p.line([(315,441),(315,507)])
    p.group(630,212,910,510,'局部路线的及时修正')
    for i,(a,b) in enumerate([('旧点三维匹配','仅修正自由证据附近原点'),('剩余障碍膨胀','保留机体与安全间隙'),('当前占用回填','当前人、货架继续挡路')]):
        p.box(657+i*290,274,260,276,a,b,i)
        if i<2:p.line([(922+i*290,410),(943+i*290,410)])
    p.box(657,595,840,100,'重新检查路径或探索航点','探索与普通目标规划使用不同任务节点',2,size=22)
    p.line([(513,590),(611,590),(611,410),(653,410)],TEAL)
    p.line([(1358,554),(1358,591)])
    p.text(75,738,'局部自由观测可先修正路线，不必等持续地图完成删除；两者删除条件不能混同。',23,TEAL)
    done(p)

    p=new(14,'配对时间与更新时刻','时间错了，删除证据也会错',
          '刚收到消息，不代表它和当前扫描属于同一个时刻。',
          '定时重发旧图不是新观测；探索保留最后扫描采集时间。',
          '探索模式先要求扫描frame为map，寻找采集时间差不超过0.08秒的odom；找不到时只缓存最新一帧，等待配对，不使用上一帧位姿冒充当前。历史odom最多100条，无插值，仅取最近时间。有效四元数归一化后将配置extrinsic_T旋转到map，再构造射线。当前源码并非完备防重复/乱序扫描协议，图中只表述已有时间配对和输出时间戳规则；普通模式仍用最新位置加固定z偏移的近似起点。')
    row(p,[('1 扫描坐标一致','不是改个坐标名字就算对齐。'),('2 位姿时间接近','时间差最多0.08秒；否则等待最新扫描配对。'),('3 外参随姿态旋转','使用对应位姿构造雷达实际起点。'),('4 输出保留采集时间','定时重发旧图，不改称刚采集的新图。')],h=425)
    for i,k in enumerate(['points','clock','ray','store']):p.glyph(k,85+i*378,569,300,72)
    p.text(73,699,'规则限制：最近时间配对不是插值，不等于已经验证真实雷达逐点运动补偿。',25,GRAY)
    done(p)

    p=new(15,'容量与保存规则','持续地图也要有容量与保存规则',
          '地图删除有两种原因：观测确认失效，或者容量保护淘汰。',
          '保存的是当前维护图快照，不在导出时额外裁剪人体来制造“零鬼影”。',
          '存储上限500000格，超过后按last_frame淘汰较久未命中格，这不是动态识别或自由证据。每帧最多8000条抽样射线、200000个自由格，探索沿射线步长为0.075米（体素一半），不是static_map.yaml的普通射线0.15米参数。2Hz定时派生/发布视图，不是地图只以2Hz接收扫描。save_navigation_map写有限XYZ二进制PCD与元数据，非空才覆盖；地图或扫描墙钟接收停止超过3秒拒绝保存；各文件临时写/fsync/replace，PCD与JSON并非跨文件原子事务，校验和用于发现不一致。导出不额外删人体或真值裁剪。',
          ['src/static_map_builder.cpp','config/static_map.yaml','scripts/save_navigation_map.py','scripts/run_dynamic_comparison.py'])
    row(p,[('1 约束清理开销','最多8000条射线、20万个自由格。'),('2 限制地图容量','最多50万格；超限淘汰久未命中的格。'),('3 定时输出视图','设置每秒2次；重发不增加观测证据。'),('4 保存真实快照','有限点、来源、时间、点数和校验信息。')],h=425)
    for i,k in enumerate(['ray','store','clock','clean']):p.glyph(k,85+i*378,569,300,72)
    p.text(73,699,'地图缺点不一定都是误删：还可能是未扫到、遮挡、入图筛选或容量淘汰。',25,GRAY)
    done(p)

    p=new(16,'规则总览与适用边界','我们制定的规则：敢更新，也不随意删',
          '关注人离开后旧占用的恢复，同时保留货架与当前真实障碍。',
          '这是可追查的地图维护工程层，不是全图零鬼影或完整建图的保证。',
          '普通版轨迹圆柱清理与探索版观测证据模式都保留，讲解不能混用阈值和删除权限。探索版以真实回波前空间为依据，当前表面优先、稳定格更谨慎、证据有间隔限制，并区分即时路径修正与持续存储删除。剩余边界包括未再观测残影、几何与时间误差、当前自由视图不是严格删除授权集合、跨回调命中未按唯一时间戳去重、固定体素/非概率模型、不会自动修改FAST-LIO匹配树。建议后续成对评价旧人体残留、静态结构保留、删除延迟、路径恢复和资源，不能只看地图变空。')
    row(p,[('现有核心规则','同帧同格只记一次。\n共享存储派生多视图。\n探索不凭轨迹擦除。\n当前返回优先保护。'),('删除必须有条件','射线经过旧点附近。\n距命中至少0.5秒。\n证据间隔不超过1秒。\n普通3次、稳定6次。'),('必须保留的边界','未再观测不保证清除。\n当前人体仍是障碍。\n自由视图不是删除授权。\n不自动改内部匹配树。')],h=482)
    p.text(72,726,'建议成对检验：旧点残留 + 静态保留 + 恢复路线 + 处理开销；本轮没有新飞行实验。',24,TEAL)
    done(p)

def rules():
    cfg=yaml.safe_load((B/'config/static_map.yaml').read_text())
    expected=dict(voxel_size=.15,minimum_static_hits=2,stable_minimum_static_hits=8,
        local_minimum_hits=1,local_max_age_frames=15,local_radius=6.,free_space_miss_threshold=3,
        free_space_endpoint_margin=.30,free_space_max_range=6.,dynamic_history_seconds=20.,
        dynamic_mask_radius=.65,dynamic_mask_half_height=1.,backward_trail_seconds=1.5,
        trail_step=.15,max_free_rays=8000,max_free_voxels=200000,max_voxels=500000,publish_rate=2.)
    for k,v in expected.items():assert math.isclose(cfg[k],v),(k,cfg[k],v)
    launch=ET.parse(B/'launch/warehouse_exploration.launch').getroot()
    params={x.attrib['name']:x.attrib['value'] for x in launch.findall('param')}
    for k,v in dict(evidence_only_clearing='true',stable_free_miss_threshold='6',
        free_evidence_max_gap='1.0',free_evidence_min_age='0.5',free_ray_point_tolerance='0.06',
        clearing_odom_max_dt='0.08').items():assert params['/static_map_builder/'+k]==v
    src=(B/'src/static_map_builder.cpp').read_text()
    for snippet in ['if (evidence_only_clearing_) return;', 'const double step = voxel_size_ * 0.5;',
        'cell.free_misses = 0;', 'if (gap<=0 || gap>evidence_max_gap_) cell.free_misses=0;',
        'output.header.stamp = evidence_only_clearing_ ? last_header_.stamp : ros::Time::now();']:
        assert snippet in src,snippet
    return len(expected)+6+5

def main():
    before={str(x):sha(x) for x in FILES};count=rules();generate()
    assert len(PAGES)==16 and len(NOTES)==16
    assert all(x['in_page'] for x in CHECKS),[x for x in CHECKS if not x['in_page']]
    notes=['# 可撤销旧点的地图维护：详细讲解与源码对应','',
        '本次只生成16张高清PNG和可编辑SVG，没有生成PPT，没有改飞行程序。',
        '参考：`/home/a/下载/fastlio2ppt.pptx`及用户上传的分区架构/七步循环图；采用深蓝标题、白底、虚线分组、淡黄/淡绿核心区和多色流程框。',
        '所有点云小图、射线和时间线都是规则示意，不是新实验，不用于声称清理精度。','',
        '## 使用方法','',
        '- PNG目录01—16可全屏展示，也可自行插入PPT。每张3840×2160、16:9。',
        '- SVG目录可修改文字、框和箭头；压缩合集包含全部图片与讲解、来源和复验工具。',
        '- 重新生成：`QT_QPA_PLATFORM=offscreen python3 build_images.py`；独立检查：`python3 verify_images.py`。','']
    for p,(title,body) in zip(PAGES,NOTES):
        notes += [f'## {p["number"]:02d} {title}','',f'[高清图]({p["png"]}) · [可编辑图]({p["svg"]})','',body,'','源码与配置：','']
        notes += [f'- [{s}]({B/s})' for s in p['sources']];notes.append('')
    notes += ['## 话题与地图口径','',
        '| 内容 | 当前接口 | 口径 |','|---|---|---|',
        '|输入扫描|`/uav1/fastlio/registered_scan`|当前注册帧，不得反馈累计输出|',
        '|输入位姿|`/uav1/fastlio/odom`|map坐标位姿与采集时刻|',
        '|持续地图|`/uav1/fastlio/cloud_map`|共享存储中至少2次命中的格|',
        '|兼容地图|`/uav1/static_map`|持续地图别名，不是另一份独立存储|',
        '|稳定地图|`/uav1/stable_static_map`|至少8次命中，仍可被撤销|',
        '|局部占用|`/uav1/local_static_map`|水平6米、近15帧边界、至少1次、free_misses=0|',
        '|当前自由|`/uav1/local_free_space`|射线遍历集合，不等同严格持久删除许可|','',
        '## 必须避免的误讲','',
        '1. 不说“维护图直接修改FAST-LIO内部树”，它是bridge的独立导航地图。',
        '2. 不说“没点就删”或“所有空角度都是自由”，删除需要有效返回、几何和时序证据。',
        '3. 不说“稳定点永久不删”或“至少8次命中就是墙”，静止人体也能稳定。',
        '4. 不说“严格连续六帧必删”，代码按有效矛盾时间间隔检查，遮挡/未观测帧不自动计数。',
        '5. 不说“局部图过期等于全图删除”，局部是视图筛选；普通/稳定图来自共享存储。',
        '6. 不把latest_free_cells与evidence集合混同：后者更严格，局部路径可能在持久删除前使用前者。',
        '7. 不把定时器新接收称为新扫描，探索输出保留采集时间；当前回调也未完整按唯一时间戳去重。',
        '8. 不把原仿真注册点云称为本轮已验证真实雷达逐点去畸变。',
        '9. 不把保存PCD、局部旧ROI清空或二维可达路线当作整仓库完整/无碰撞/全图零鬼影。','',
        '本轮验证范围：源码与配置、图文排版、文件/链接/打包；未ROS编译、启动仿真、真机或清理性能测试。','']
    (HERE/'详细讲解与源码对应.md').write_text('\n'.join(notes))
    (HERE/'内容与来源.json').write_text(json.dumps(dict(pages=PAGES,sources_sha256=before,
        text_bounds=CHECKS,
        reference=str(REF),user_reference='两张会话附件：分区架构图和七步循环流程图；未保存附件副本',
        diagram_scope='纯矢量规则示意，无实验点云伪装'),ensure_ascii=False,indent=2))
    for p in PAGES:ET.parse(p['svg'])
    assert before=={str(x):sha(x) for x in FILES}
    report=dict(status='passed',pages=16,png_size=[3840,2160],text_lines=len(CHECKS),
        rule_checks=count,unchanged_sources=len(FILES),arrows=sum(p['arrows'] for p in PAGES),
        pptx_created=False,scope='图文来源核验，非ROS/飞行测试',visual_review='生成后另外检查并记录')
    (HERE/'生成核验.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    contact=Image.new('RGB',(1920,2370),'#E6EEF3');pen=ImageDraw.Draw(contact)
    ff=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',22)
    for i,p in enumerate(PAGES):
        with Image.open(p['png']) as im:
            im=im.convert('RGB');im.thumbnail((618,348));x=20+(i%3)*638;y=18+(i//3)*391
            contact.paste(im,(x,y));pen.text((x,y+350),f'{p["number"]:02d} {p["name"]}',font=ff,fill=NAVY)
    contact.save(HERE/'00_全部图片预览.png')
    for start in [0,8]:
        sheet=Image.new('RGB',(1920,1540),'#E6EEF3');d=ImageDraw.Draw(sheet)
        for j,p in enumerate(PAGES[start:start+8]):
            with Image.open(p['png']) as im:
                im=im.convert('RGB');im.thumbnail((618,348));x=20+(j%3)*638;y=18+(j//3)*391
                sheet.paste(im,(x,y));d.text((x,y+350),f'{p["number"]:02d} {p["name"]}',font=ff,fill=NAVY)
        sheet.save(HERE/f'预览_{start+1:02d}至{start+8:02d}.png')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
