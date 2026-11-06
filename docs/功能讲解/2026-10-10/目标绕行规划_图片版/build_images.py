#!/usr/bin/env python3
"""源码对应的目标绕行讲解。只生成图文与证据副本，不运行ROS。"""
import hashlib
import json
import shutil
import sys
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import yaml
from PIL import Image, ImageDraw, ImageFont
from draw import Page, TEXTS, PAGES, HERE, NAVY, BLUE, TEAL, GRAY, RED, GOLD, drone, shelf

ROOT=HERE.parents[3]
B=ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
ASSETS=HERE/'证据原件'
ARCHIVE=HERE.parent/'目标绕行规划_图片与证据合集.zip'
RECOVERY=ROOT/'dynamic_demo_runs/ours_cangku_20260927_133611_recovery_complete.json'
BAD=ROOT/'dynamic_demo_runs/badcase_20260923_201211_metrics.json'
OURS=ROOT/'dynamic_demo_runs/ours_20260923_200322_metrics.json'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def inspect_sources():
    general=yaml.safe_load((B/'config/path_planner.yaml').read_text())
    warehouse=yaml.safe_load((B/'config/warehouse_navigation.yaml').read_text())
    controller=yaml.safe_load((B/'config/astra_mavros_controller.yaml').read_text())
    checks=[]
    for obj,expected in [(general,dict(resolution=.25,inflation_radius=.95,planning_margin=4.,
            local_replan_to_final_goal=True,enable_local_replanning=True,local_free_clear_radius=.15,
            goal_tolerance=.30,replan_rate=2.,max_grid_cells=250000,goal_search_radius=1.50)),
        (warehouse['static_path_planner'],dict(max_planning_margin=12.,route_switch_improvement=.25,
            resolution=.10,inflation_radius=.80,lookahead_distance=.60,start_clearance_radius=.20,
            obstacle_min_relative_z=-.45,obstacle_max_relative_z=.55)),
        (warehouse['mavros_avoidance_controller'],dict(max_navigation_speed=.25)),
        (warehouse['static_collision_risk'],dict(uav_radius=.40,safe_clearance=.30,
            emergency_clearance=.25,exit_clearance=.35)),
        (controller,dict(goal_tolerance=.25,waypoint_tolerance=.03))]:
        for k,v in expected.items():assert obj[k]==v,(k,obj[k]);checks.append(k+'='+str(v))
    files=['src/static_path_planner.cpp','src/mavros_avoidance_controller.cpp',
        'src/static_map_builder.cpp','config/path_planner.yaml','config/warehouse_navigation.yaml',
        'config/astra_mavros_controller.yaml','launch/astra_dynamic_avoidance.launch',
        'launch/warehouse_exploration.launch','scripts/run_dynamic_comparison.py']
    for rel,fragments in {
        'src/static_path_planner.cpp':['grid.occupied.assign(static_cast<std::size_t>(cell_count), 0U)',
            'std::sqrt(0.5)*resolution_','free_tree.nearestKSearch','rasterize(primary_map, inflation_radius_, 1U, true)',
            'rasterize(*overlay_map, inflation_radius_, 1U)','margin * 2.0','active_search_margin_ = margin',
            'new_length >= old_length * (1.0-route_switch_improvement_)','lineIsFree(grid, retained[i-1], retained[i])',
            'if (!global_ok && !have_local_free_space_)','if (!global_ok && !enable_local_replanning_)',
            'if (lineIsFree(grid, path.front(), candidate))','nearestFreeCell(grid, gx, gy)',
            '(grid.isOccupied(cx + offset[0], cy)','publishPath({current}, stamp, local_path_pub_)'],
        'src/mavros_avoidance_controller.cpp':['(now - goal_time_).toSec() <= 1.5',
            '? waypoint_tolerance_ : goal_tolerance_','stop_on_level1_'],
        'launch/astra_dynamic_avoidance.launch':['Load last','config/warehouse_navigation.yaml'],
        'launch/warehouse_exploration.launch':['enable_path_planner" value="false']}.items():
        text=(B/rel).read_text()
        for fragment in fragments:assert fragment in text,fragment;checks.append(rel+': '+fragment)
    recovery=json.loads(RECOVERY.read_text());bad=json.loads(BAD.read_text());ours=json.loads(OURS.read_text())
    assert len(recovery)==2 and all(r['arrived'] for r in recovery)
    assert sum(len(r['samples']) for r in recovery)==658
    assert all(s['path_poses']>1 and s['risk']==0 for r in recovery for s in r['samples'])
    assert bad['vacated_start_stable_map_points']==128 and ours['vacated_start_stable_map_points']==0
    assert bad['goal_reached_odom'] and ours['goal_reached_odom']
    checks+=['recovery two arrivals, 658 samples','example original metrics, both arrived']
    (HERE/'证据原件').mkdir(exist_ok=True)
    sources=[*[B/f for f in files]]
    copies=[]
    for src in [RECOVERY,BAD,OURS,ASSETS/'C05_recovery_route.png',ROOT/'dynamic_demo_runs/badcase_vs_ours.png']:
        dst=HERE/'证据原件'/src.name
        if src.resolve()!=dst.resolve():shutil.copy2(src,dst)
        sources.append(src)
        copies.append(dict(source=str(src),copy=str(dst.relative_to(HERE)),sha256=sha(src)))
    sources += [p for p in (HERE.parent/'黑箱探索补扫返航_图片版').rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    return dict(checks=checks,source_hashes={str(p):sha(p) for p in sources},copies=copies,
        scope='source/image checks and historical reports; no new flight test'),recovery,bad,ours

def route_scene(p,x,y,w,h,mode='route'):
    """货架俯视示意；不是实际地图，不用于碰撞/性能计量。"""
    p.illustrations+=1
    rx=x+w*.47;ry=y+h*.19;rw=w*.12;rh=h*.52
    p.rect(rx-w*.035,ry-h*.07,rw+w*.07,rh+h*.14,'#E8DCE8','#D3BCCC',1,True)
    p.rect(rx,ry,rw,rh,'#DACBAD','#A7916C',1.5)
    for a in range(3):p.line([(rx,ry+rh*(a+1)/4),(rx+rw,ry+rh*(a+1)/4)],'#A7916C',1,False)
    start=(x+w*.16,y+h*.52);goal=(x+w*.83,y+h*.52)
    if mode=='blocked':
        p.line([start,(rx-w*.05,start[1])],RED,3,True,True)
        p.circle(rx-w*.10,start[1],12,'none',RED,2)
    elif mode=='lookahead':
        corner=(x+w*.33,y+h*.88)
        p.line([start,corner,(x+w*.71,y+h*.88),goal],TEAL,3)
        p.line([start,(x+w*.64,y+h*.88)],RED,2,True,True)
        p.circle(*corner,7,BLUE)
    else:
        p.line([start,(x+w*.31,y+h*.88),(x+w*.74,y+h*.88),goal],TEAL,3)
        if mode=='expand':
            p.rect(x+w*.23,y+h*.22,w*.58,h*.48,'none',RED,1.7,True)
            p.rect(x+w*.06,y+h*.05,w*.88,h*.91,'none',BLUE,1.7,True)
        if mode=='stable':p.line([start,(x+w*.30,y+h*.04),(x+w*.75,y+h*.04),goal],GOLD,2,True,True)
    drone(p,*start,.60 if w<500 else .85);p.circle(*goal,9,GOLD,NAVY,1)
    if h>175:
        p.text(start[0],start[1]+27,'当前位置',18,center=True)
        p.text(goal[0],goal[1]+27,'最终目标',18,center=True)

def points_scene(p,x,y,w,h):
    p.illustrations+=1;shelf(p,x+w*.08,y+h*.14,w*.26,h*.65,True)
    p.line([(x+w*.49,y+h*.50),(x+w*.66,y+h*.50)],BLUE,2.5)
    for i in range(8):
        for j in range(5):p.rect(x+w*.69+i*w*.030,y+h*.18+j*h*.115,w*.029,h*.11,
            '#DBC9AA' if i in (2,3) else '#E2F0F1','white',1)

def local_scene(p,x,y,w,h,cleared=False):
    p.illustrations+=1;shelf(p,x+w*.78,y+h*.03,w*.13,h*.62,True)
    if cleared:
        # 为空区域用浅绿色虚线框，不保留“绿色人体点”，避免误解为残影仍在。
        p.rect(x+w*.42,y+h*.26,w*.14,h*.41,'#E4F3EB',TEAL,1,True)
    else:
        for i in range(7):
            for j in range(9):
                u=x+w*.49+(i-3)*w*.019;v=y+h*.28+j*h*.045
                p.circle(u,v,2.3,RED)
    drone(p,x+w*.12,y+h*.68,.75)
    if cleared:
        for t in (0,.10,.20):p.line([(x+w*.12,y+h*.68),(x+w*.73,y+h*(.3+t))],TEAL,1.6)
        p.line([(x+w*.17,y+h*.82),(x+w*.68,y+h*.82)],TEAL,3)
    else:p.line([(x+w*.17,y+h*.82),(x+w*.37,y+h*.82),(x+w*.37,y+h*.06),(x+w*.69,y+h*.06)],GOLD,3)

def architecture():
    p=Page(1,'目标绕行模块架构','真正的目标绕行：先找到路，再安全地飞','不只在障碍前躲一下；始终把障碍后面的最终目标作为路线终点','机制示意｜目标规划与即时风险并行；当前是固定高度的平面路线，不是完整三维轨迹。')
    for x,w,t in [(75,380,'目标与环境输入'),(506,926,'地图感知的路线规划'),(1486,353,'执行与反馈')]:
        p.rect(x,249,w,695,'none',BLUE,2,True);p.text(x+w/2,215,t,29,bold=True,center=True)
    p.card(102,287,326,164,'最终目标 + 自身位置','同一地图坐标下确定起终点；\n二维选点不反复抬高高度。',0,20)
    drone(p,168,416,.72);p.circle(365,416,10,GOLD)
    p.card(102,479,326,180,'稳定的长期地图','保留已扫描的货架和墙体；\n为整段绕行提供结构信息。',2,20)
    shelf(p,187,625,70,23,True)
    p.card(102,689,326,205,'局部占用 + 为空证据','眼前仍有物体就补回障碍；\n观测到已空，才修正旧占用。',4,20)
    local_scene(p,132,814,264,68,True)
    p.rect(535,285,868,278,'#FFF1BF',BLUE,1.5,True);p.text(557,300,'全局路线：知道怎样绕到障碍后面',27,bold=True)
    p.card(558,358,365,170,'高度切片 + 安全膨胀','只投影飞行高度附近的点；\n留出机体和附加净空。',4,20)
    points_scene(p,610,497,250,25)
    p.card(977,358,402,170,'搜索到最终目标的路线','八方向搜索，不从角缝穿过；\n仓库无路时扩大搜索范围。',1,20)
    route_scene(p,1003,473,349,45);p.line([(923,437),(977,437)])
    p.rect(535,626,868,278,'#E9F3E7',BLUE,1.5,True);p.text(557,642,'局部修正：环境变了，整段路线也能变',27,bold=True)
    p.card(558,704,365,169,'修正旧占用，再搜索','先匹配三维为空证据；\n再膨胀并补回局部占用。',2,20)
    local_scene(p,591,838,300,25,True)
    p.card(977,704,402,169,'检查旧路与前视航点','旧路失效就换；仍有效则防抖；\n直飞到航点也必须不切角。',5,20)
    route_scene(p,1003,824,349,39,'lookahead');p.line([(923,780),(977,780)],TEAL)
    p.line([(1180,563),(1180,626)],TEAL);p.text(1201,583,'结合当前观测',19,color=TEAL)
    for pts in [[(455,362),(506,362)],[(455,561),(485,561),(485,479),(506,479)],[(455,791),(506,791)]]:p.line(pts,TEAL)
    p.card(1513,311,300,181,'安全控制器','跟踪前视航点并限制速度；\n即时风险可覆盖导航动作。',4,20);drone(p,1658,455,.90)
    p.card(1513,535,300,145,'到达后悬停','区分中间拐点与最终目标；\n不把过早停车当作到达。',2,20)
    p.card(1513,722,300,170,'路线与航迹显示','显示全局路线、修正路线、\n实际走过的航迹。',3,20)
    p.line([(1432,781),(1460,781),(1460,402),(1486,402)])
    p.line([(1813,412),(1870,412),(1870,982),(265,982),(265,944)],TEAL,2,True,True)
    p.text(635,946,'飞行产生新观测 → 更新地图 → 再检查路线',23,color=BLUE,bold=True);p.save()

def seven_steps():
    p=Page(2,'图文七步流程','目标绕行规划：七步图文流程','像参考图一样：四上三下，每一步都有仓库、点云或路线插图','机制示意｜同一目标持续更新；全局无路时，有局部为空证据仍可尝试修正路线。')
    upper=[(85,258,395,326),(535,258,395,326),(985,258,395,326),(1435,258,395,326)]
    content=[('1　接收目标与自身位置','保留最终目标；统一坐标；\n二维选点保持飞行高度。'),
        ('2　建立安全占用格','筛选飞行高度附近点云，\n膨胀障碍，给机体留余量。'),
        ('3　全局搜索绕行路线','搜索障碍两侧的连通通道；\n仓库无路时扩大搜索边界。'),
        ('4　结合局部观测修路','先纠正匹配的旧占用，\n再补回眼前障碍并重新搜索。')]
    for i,((x,y,w,h),(title,body)) in enumerate(zip(upper,content)):
        p.card(x,y,w,h,title,body,i)
        if i==1:points_scene(p,x+24,y+161,w-48,125)
        elif i==3:local_scene(p,x+24,y+155,w-48,140,True)
        else:route_scene(p,x+24,y+150,w-48,144,'expand' if i==2 else 'route')
        if i<3:p.line([(x+w,422),(upper[i+1][0],422)],BLUE,2.5)
    lower=[(1295,691,535,292),(695,691,535,292),(85,691,535,292)]
    content=[('5　判断是否需要换路线','旧路不安全立即换；仍可走时，\n明显缩短才换，避免两侧摇摆。','stable'),
        ('6　给出能直飞的前视点','检查机体到航点的整条线段；\n风险控制仍可停车或退让。','lookahead'),
        ('7　跟踪拐点，最终到达悬停','中间点与终点用不同容差；\n新目标会重新建立绕行路线。','route')]
    for i,((x,y,w,h),(title,body,mode)) in enumerate(zip(lower,content)):
        p.card(x,y,w,h,title,body,4+i);route_scene(p,x+40,y+142,w-80,123,mode)
        if i<2:p.line([(x,830),(lower[i+1][0]+lower[i+1][2],830)],BLUE,2.5)
    p.line([(1632,584),(1632,643),(1562,643),(1562,691)],BLUE,2.5)
    p.text(985,600,'默认重新搜索到最终目标，不只修眼前一小段',20,color=TEAL)
    p.line([(290,983),(290,1002),(52,1002),(52,221),(730,221),(730,258)],TEAL,2,True,True)
    p.text(792,191,'新观测或障碍变化：再次检查',19,color=TEAL);p.save()

def detour():
    p=Page(3,'搜索与扩域规则','为什么叫“真正的目标绕行”','近场避障回答“别撞上”；目标规划回答“从哪里绕过去才能到终点”','机制示意｜扩大的是搜索范围，不是把障碍擦掉，更不是放松安全距离。')
    for x,title,body,mode,color in [(75,'只盯眼前的障碍','前方挡住就停止或退让；\n不能单靠退一步找到障碍后面的通道。','blocked',3),
        (1000,'地图规划到最终目标','比较连通通道，先绕到货架端部；\n越过端部，再转向真正的终点。','route',2)]:
        p.card(x,224,845,478,title,body,color,24);route_scene(p,x+70,374,700,280,mode)
    for x,title,body,color in [(75,'规则一：搜索边界要够大','仓库从4米余量开始；失败后\n扩大到8米、12米，同目标保留\n已扩大的范围，避免来回缩放。',1),
        (690,'规则二：路径不能抄危险近路','八方向搜索禁止从障碍角缝穿越；\n简化路线时检查整条直线，\n不能只看终点格子是否空着。',4),
        (1305,'规则三：无路不能假装到达','远离终点时，稳定地图不足30点先保持；\n搜索最多25万格；仍无可行路\n就发布保持位置，等待重规划。',0)]:
        p.card(x,750,540,232,title,body,color,24)
    p.save()

def local_update():
    p=Page(4,'局部修图与通道恢复','旧地图说“堵着”，新观测说“已经空了”','先核对三维旧占用，再重新规划；眼前仍存在的物体必须保留','机制示意｜局部修路与持久地图撤销是不同操作；没有回波不等于有为空证据。')
    p.card(75,226,820,403,'之前：人体旧占用造成假墙','人已经离开，但历史点还挡住通道；\n全局路线可能绕远，甚至完全搜索失败。',3,24);local_scene(p,128,373,701,219)
    p.card(1005,226,840,403,'现在：局部观测确认，重新审视路线','匹配的旧点不再用于这轮障碍膨胀；\n重新计算到最终目标的路径，允许恢复通道。',2,24);local_scene(p,1060,373,720,219,True)
    p.line([(895,430),(1005,430)],TEAL,3)
    steps=[(75,'1　只撤销匹配的旧点','三维距离在0.15米内才匹配；\n不能用二维大圆直接擦掉附近墙。'),
        (690,'2　剩余障碍重新膨胀','先修正原始点，再留出安全余量；\n避免旧膨胀区被随意掏出假捷径。'),
        (1305,'3　当前局部占用最后回填','当前局部障碍优先，重新参与膨胀；\n不能只清旧点、不补回真实占用。')]
    for i,(x,t,b) in enumerate(steps):p.card(x,681,540,207,t,b,i+1,23)
    p.line([(615,786),(690,786)],TEAL);p.line([(1230,786),(1305,786)],TEAL)
    p.text(78,928,'全局无路 ≠ 直接结束：启用局部修路且已有为空证据时，仍尝试修正后的搜索。',27,color=BLUE,bold=True,width=1765);p.save()

def stability():
    p=Page(5,'稳定执行与安全边界','路线要稳定，也要随真实变化及时调整','仓库规则：保留有效旧路、检查实际直飞段；即时安全仍有最终决定权','机制示意｜这些规则是工程约束，不是任意未知场景、三维扫掠体或动态轨迹的安全证明。')
    p.card(75,225,840,337,'换路规则：不要每次扫描都左右摇摆','旧路线逐段仍可走：新路要缩短超过约25%才换；\n旧路遇到新占用：不受长度门限限制，立即作废。',2,24);route_scene(p,137,380,690,152,'stable')
    p.card(1000,225,845,337,'执行规则：不能隔着拐角追前视点','前视距离沿折线取点，但飞机实际直飞；\n直飞段被挡住，就先追最近的可见拐点。',4,24);route_scene(p,1070,380,690,152,'lookahead')
    p.card(75,609,540,315,'仓库规划参数','格子边长0.10米；障碍膨胀0.80米\n已含机体半径，另加约0.071米离散余量。\n高度带：目标高度下0.45米、上0.55米。\n前视0.60米；导航限速0.25米/秒。',1,23)
    p.card(690,609,540,315,'到达与时效规则','中间航点容差0.03米；最终控制容差0.25米。\n规划器距终点0.30米内仍发最终目标。\n航点超过1.5秒未更新，水平导航置零。\n风险动作优先，不是“有路就强行飞”。',0,23)
    p.card(1305,609,540,315,'当前实现的边界','只有占用格，未观测空白不严格禁行。\n起点有0.20米小范围清除例外。\n目标被占用时可改找邻近空格，\n不保证原目标到达；地图自身无年龄门控。',3,23)
    p.text(78,948,'此页为指定目标导航；自主探索另有“未知不能走”的栅格与任务规则。',26,color=BLUE,bold=True,width=1760);p.save()

def result_recovery(recovery):
    p=Page(6,'仓库真实绕行结果','真实仓库结果：恢复后完成去程与返程','复用当时保存的轨迹图；蓝线去程、橙线返程，均为地图坐标中的实际里程计航迹','历史实测｜这是保留原仿真和地图、重载后得到的两段恢复测试，不是冷启动全流程成功率。')
    p.image(HERE/'证据原件/C05_recovery_route.png',75,217,1110,735)
    p.card(1230,224,605,241,'去程：到达货架后方目标',
        f"目标（14.00，−0.15）米\n终点平面误差 {recovery[0]['error']:.3f} 米\n墙钟耗时 {recovery[0]['wall_seconds']:.1f} 秒；采样仿真跨度86.221秒",2,24)
    p.card(1230,488,605,241,'返程：沿可行通道返回',
        f"目标（−0.204，9.897）米\n终点平面误差 {recovery[1]['error']:.3f} 米\n墙钟耗时 {recovery[1]['wall_seconds']:.1f} 秒；采样仿真跨度98.886秒",4,24)
    p.card(1230,752,605,233,'这张图支持什么结论','658次采样无单点保持路径；两段均到达。\n最近点云表面约0.878 / 0.872米，\n不是碰撞真值；未主动移动行人，\n也没有验证全仓库、所有目标或真机。',1,23)
    p.text(95,959,'原图完整保留；来源编号与原始JSON见随附讲解文档。',20,color=GRAY);p.save()

def result_compare(bad,ours):
    p=Page(7,'内部对照真实结果','真实内部对照：清掉旧障碍，让路线少绕远','左图是原保存的实际真值航迹；右侧数值来自当时两份原始报告','历史实测｜各一次的组合消融：地图清理 + 局部重规划；不是单独证明新规划算法或统计优势。')
    p.image(HERE/'证据原件/badcase_vs_ours.png',70,216,790,790);x=917
    p.text(x,226,'两种配置',28,bold=True);p.text(1460,226,'原对照',28,color=GOLD,bold=True,center=True);p.text(1708,226,'完整方案',28,color=TEAL,bold=True,center=True)
    rows=[('到达时间（仿真秒）',f"{bad['time_to_goal_s']:.3f}",f"{ours['time_to_goal_s']:.3f}"),
        ('实际平面航程（米）',f"{bad['gazebo_route_length_after_goal_xy_m']:.3f}",f"{ours['gazebo_route_length_after_goal_xy_m']:.3f}"),
        ('旧位置区域稳定地图残留（点）','128','0'),
        ('终点平面误差（米）',f"{bad['final_goal_error_odom_xy_m']:.3f}",f"{ours['final_goal_error_odom_xy_m']:.3f}"),
        ('最近人机中心距离（米）',f"{bad['min_uav_pedestrian_center_xy_m']:.3f}",f"{ours['min_uav_pedestrian_center_xy_m']:.3f}")]
    for i,(label,a,b) in enumerate(rows):
        y=302+i*91;p.rect(x,y-5,920,77,'#F3F7FA' if i%2==0 else 'white','none')
        p.text(x+17,y,label,23,width=442);p.text(1460,y,a,27,bold=True,center=True);p.text(1708,y,b,27,bold=True,center=True,color=TEAL if i<3 else RED)
    p.card(x,783,920,199,'可以说的优势，与不能省略的代价','这次时间减少39.4%，航程减少38.8%，局部旧残留减少。\n但终点误差与最近中心距离没有改善；人的运动与航线不同。\n这是example场景，不能与上一页仓库恢复拼成同条件对比。',2,24);p.save()

def overview():
    out=Image.new('RGB',(1920,1250),'#E8EDF2');d=ImageDraw.Draw(out)
    f=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',25)
    for i,p in enumerate(sorted((HERE/'PNG').glob('*.png'))):
        col,row=i%3,i//3;im=Image.open(p).convert('RGB');im.thumbnail((610,344))
        x=22+col*640;y=22+row*410;out.paste(im,(x,y));d.text((x,y+350),p.stem,font=f,fill=NAVY)
    out.save(HERE/'00_七页总预览.png')

def pack():
    files=[p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    with ZipFile(ARCHIVE,'w',ZIP_DEFLATED) as z:
        for p in sorted(files):z.write(p,str(Path(HERE.name)/p.relative_to(HERE)))
    print('ZIP',len(files),ARCHIVE.stat().st_size)

def main():
    if '--pack-only' in sys.argv:pack();return
    facts,recovery,bad,ours=inspect_sources();(HERE/'源码与数据核验.json').write_text(json.dumps(facts,ensure_ascii=False,indent=2))
    architecture();seven_steps();detour();local_update();stability();result_recovery(recovery);result_compare(bad,ours)
    for t in TEXTS:assert t['x']>=0 and t['y']>=0 and t['x']+t['width']<=1920 and t['y']+t['height']<=1080,t
    (HERE/'版面核验信息.json').write_text(json.dumps(dict(texts=TEXTS,pages=PAGES),ensure_ascii=False,indent=2))
    overview();pack();print('PASS source',len(facts['checks']),'text',len(TEXTS),'illustrations',sum(p['illustrations'] for p in PAGES),'arrows',sum(p['arrows'] for p in PAGES))
if __name__=='__main__':main()
