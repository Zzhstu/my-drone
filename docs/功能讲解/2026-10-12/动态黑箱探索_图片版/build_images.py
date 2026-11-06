#!/usr/bin/env python3
"""八页原生矢量图文：只写本材料目录，原实验及运行代码保持不变。"""
import hashlib
import json
import shutil
import sys
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import yaml
from PIL import Image, ImageDraw, ImageFont
from draw import (HERE, Page, PAGES, TEXTS, NAVY, BLUE, TEAL, GRAY, RED, GOLD,
                  CYAN, drone, shelf, scene, grid, evidence)

ROOT=Path('/home/a/AstraDroneOpen')
B=ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
PREFIX=ROOT/'exploration_demo_runs/warehouse_20261003_222021'
ASSETS=HERE/'证据原件'
ARCHIVE=HERE.parent/'动态黑箱探索_八张图片与证据合集.zip'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def data(s): return json.loads(Path(str(PREFIX)+s).read_text())

def freeze():
    cfg=yaml.safe_load((B/'config/warehouse_exploration.yaml').read_text())
    expected={'initial_window_radius':12.,'growth_margin':4.,'growth_chunk':8.,
      'max_grid_cells':200000,'resolution':.2,'clearance':.8,'unknown_margin':.4,
      'frontier_sensor_range':6.,'frontier_min_gain_area':.5,'settle_seconds':2.,
      'progress_timeout':25.,'blacklist_seconds':60.,'max_target_attempts':2,
      'convergence_seconds':90.,'convergence_free_area':1.,'convergence_new_surfaces':25,
      'convergence_min_goals':3,'surface_voxel':.4,'surface_min_hits':3,
      'surface_min_views':2,'surface_view_baseline':.75,'surface_quality_target':.95,
      'inspection_revisit_seconds':20.,'inspection_budget_seconds':600.,
      'home_tolerance':.3,'home_settle_seconds':5.,'max_mission_seconds':3600.,
      'max_mission_wall_seconds':3600.,'return_timeout':1200.}
    checks=[]
    for k,v in expected.items():
        assert cfg[k]==v,(k,cfg[k],v); checks.append('有效配置 '+k)
    launch=(B/'launch/warehouse_exploration.launch').read_text()
    for f in ['max_navigation_speed" value="0.30','enable_path_planner" value="false',
              'enable_loop_closure" value="false','evidence_only_clearing" value="true',
              'stable_free_miss_threshold" value="6','free_ray_point_tolerance" value="0.06',
              'free_evidence_min_age" value="0.5','free_evidence_max_gap" value="1.0',
              'clearing_odom_max_dt" value="0.08','require_fresh_scan" value="true']:
        assert f in launch,f;checks.append('探索专属装配 '+f)
    code=(B/'scripts/warehouse_explorer.py').read_text()
    for f in ["('map','/uav1/fastlio/cloud_map'","('scan','/uav1/fastlio/registered_scan'",
              "('free','/uav1/local_free_space'",'self.grid.update_obstacles(points)',
              'np.concatenate([self.slice(self.cloud(\'map\',inputs)),self.slice(self.cloud(\'scan\',inputs))])',
              'if not self.remaining_inspection_count and self.useful_frontier_count:',
              'coverage_complete=False','not self.failed_visits','if self.save():']:
        assert f in code,f;checks.append('探索源码 '+f)
    maint=(B/'src/static_map_builder.cpp').read_text()
    for f in ['if (evidence_only_clearing_) return;','latest_free_cells_.insert(key);',
              'cell.free_misses = 0;','(relative-direction*along).norm() <= ray_point_tolerance_',
              'if (cell.free_misses>=threshold) { cells_.erase(it);','q.normalized() * lidar_offset_']:
        assert f in maint,f;checks.append('清图源码 '+f)
    run=data('_result.json');m=run['mission'];ours=data('_coverage.json');other=data('_freedom_coverage.json');meta=data('_final_map.json')
    assert run['termination']=='returned_home_and_saved' and m['finish_reason']=='wall_budget_partial'
    assert m['goals_reached']==93 and m['goals_selected']==97 and m['stalled_goals']==3
    assert not m['exploration_complete'] and not m['coverage_complete']
    assert meta['points']==284418 and sha(Path(str(PREFIX)+'_final_map.pcd'))==meta['sha256']
    assert (ours['supported_samples'],other['supported_samples'],ours['reference_samples'])==(9267,10091,10192)
    assert len(ours['racks'])==52 and not ours['shelf_side_acceptance'] and not other['shelf_side_acceptance']
    assert sum(r['coverage']<.9 for r in ours['racks'])==10 and sum(r['coverage']<.9 for r in other['racks'])==3
    checks+=['原任务返回保存但预算部分','原落盘PCD点数与哈希','原双覆盖及52货架']
    originals=[*[B/r for r in ['scripts/warehouse_explorer.py','scripts/exploration_grid.py',
      'scripts/exploration_coverage.py','scripts/exploration_native.py','src/exploration_search.cpp',
      'scripts/run_warehouse_exploration.py','scripts/warehouse_map_audit.py','src/static_map_builder.cpp',
      'src/mavros_avoidance_controller.cpp','config/warehouse_exploration.yaml','config/static_map.yaml',
      'launch/warehouse_exploration.launch','config/static_risk.yaml','launch/perception.launch']]]
    copies=[];(HERE/'证据原件').mkdir(exist_ok=True)
    for suffix in ['_result.json','_coverage.json','_freedom_coverage.json','_final_map.json',
                   '_alignment.json','_freedom_alignment.json','_scene.json']:
        src=Path(str(PREFIX)+suffix); dst=HERE/'证据原件'/src.name
        shutil.copy2(src,dst);originals.append(src)
        copies.append({'source':str(src),'copy':str(dst.relative_to(HERE)),'sha256':sha(src)})
    for name in ['C02_rack_support.png','C03_rack_support_world.png','C04_true_map_comparison.png']:
        src=ASSETS/name;dst=HERE/'证据原件'/name
        if src.resolve()!=dst.resolve():shutil.copy2(src,dst)
        originals.append(src)
        copies.append({'source':str(src),'copy':str(dst.relative_to(HERE)),'sha256':sha(src)})
    originals.extend(Path(str(PREFIX)+s) for s in ['_final_map.pcd','_freedom_static_map_point.pcd'])
    old=ROOT/'docs/功能讲解/2026-10-10/黑箱探索补扫返航_图片版'
    originals.extend(p for p in old.rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    facts={'checks':checks,'source_hashes':{str(p):sha(p) for p in originals},'copies':copies,
           'scope':'材料与历史证据核验，不是新飞行或新性能实验','experiment':str(PREFIX)}
    return facts,run,meta,ours,other

def person(p,x,y,s=1,ghost=False):
    c=RED if ghost else GOLD
    if ghost:
        for j in range(16):
            for i in range(7):
                if 2<=i<=4 or j<5:p.circle(x+(i-3)*5*s,y+j*5*s,1.8*s,c)
        p.rect(x-22*s,y-8*s,44*s,90*s,'none',RED,1.5,True)
    else:
        p.circle(x,y,8*s,c)
        p.line([(x,y+13*s),(x,y+50*s)],c,13*s,False)
        for dx in (-1,1):
            p.line([(x,y+21*s),(x+dx*15*s,y+37*s)],c,6*s,False)
            p.line([(x,y+49*s),(x+dx*12*s,y+78*s)],c,6*s,False)

def dots(p,x,y,w,h,gap=16,color=TEAL,hole=False):
    for j in range(int(h/gap)):
        for i in range(int(w/gap)):
            mid=(.35*w<i*gap<.60*w and .2*h<j*gap<.8*h)
            if hole and mid:p.circle(x+i*gap,y+j*gap,2.7,'white',GRAY,1)
            else:p.circle(x+i*gap,y+j*gap,2.7,RED if mid else color)

def dynamic_scene(p,x,y,w,h,phase):
    p.illustrations+=1
    scale=min(1.,h/160.)
    shelf(p,x+w*.05,y+h*.08,w*.19,h*.62,True)
    shelf(p,x+w*.71,y+h*.08,w*.19,h*.62,True)
    drone(p,x+w*.17,y+h*.84,.75*scale)
    if phase==0:person(p,x+w*.45,y+h*.16,.70*scale)
    elif phase==1:
        person(p,x+w*.45,y+h*.16,.70*scale,True);person(p,x+w*.65,y+h*.40,.70*scale)
        p.line([(x+w*.48,y+h*.22),(x+w*.64,y+h*.48)],GOLD,2)
    else:
        if phase==2:person(p,x+w*.45,y+h*.16,.70*scale,True)
        person(p,x+w*.72,y+h*.67,.65*scale)
        for t in (.25,.40,.55):p.line([(x+w*.17,y+h*.84),(x+w*.63,y+h*t)],TEAL,1.6,True)
        if phase==3:p.line([(x+w*.17,y+h*.84),(x+w*.42,y+h*.73),(x+w*.62,y+h*.25)],TEAL,3)

def architecture():
    p=Page(1,'模块架构','动态黑箱探索：边认识环境，边应对变化','布局未知；目标来自观测；人出现或离开后，地图与下一步决策继续更新',
      '机制示意｜固定高度二维探索；动态清图、即时安全与探索决策分工，不是学习网络。')
    for x,w,t in [(75,350,'观测与状态'),(480,910,'自主决策与地图维护'),(1445,395,'执行、反馈与结果')]:
        p.rect(x,245,w,705,'none',BLUE,2,True);p.text(x+w/2,205,t,27,bold=True,center=True)
    p.card(100,284,300,278,'当前位置 + 当前扫描','知道机体在哪里，\n看到附近真实障碍。',4)
    scene(p,117,412,262,127,'observe')
    p.card(100,594,300,312,'维护地图与自由观测','旧占用可被新观测反驳；\n没看到不等于已经空。',2)
    dynamic_scene(p,117,749,262,138,2)
    p.rect(508,277,854,301,'#FFF2C6',BLUE,1.5,True)
    p.text(532,295,'认识未知区域，选择值得去看的位置',28,bold=True)
    p.card(535,350,373,196,'观测格与安全通道','未知 / 自由 / 占用分开；\n只在已知安全空间搜索。',4,size=20)
    grid(p,555,462,333,64)
    p.card(953,350,385,196,'前沿选点与可达路径','能看见更多未知，且走得到；\n考虑路程、遮挡和失败记忆。',1,size=20)
    grid(p,977,462,335,64,'blocked')
    p.line([(908,445),(953,445)])
    p.rect(508,619,854,287,'#E9F5EC',BLUE,1.5,True)
    p.text(532,637,'环境变化 → 重新检查；增益下降 → 补扫 / 返航',27,bold=True)
    p.card(535,695,373,197,'撤销过时占用','真实射线 + 表面保护 + 多次证据；\n更新当前共享地图记忆。',2,size=20)
    dynamic_scene(p,562,849,314,34,3)
    p.card(953,695,385,197,'补扫与任务收尾','欠观测表面换角度；\n返航驻留、保存成功后结束。',5,size=20)
    scene(p,980,847,332,34,'home')
    p.line([(908,778),(953,778)],TEAL)
    p.line([(1142,578),(1142,619)],TEAL,2.5)
    p.text(745,583,'新观测改变下一轮选点与路线',19,color=TEAL)
    p.line([(425,390),(480,390)]);p.line([(425,766),(480,766)],TEAL)
    p.card(1470,284,345,268,'安全控制优先','先确认真实起飞与数据新鲜；\n当前人体仍是障碍，\n不能因清图而直接穿过去。',4,size=21)
    drone(p,1629,504,1.05);person(p,1743,450,.70)
    p.card(1470,598,345,142,'地图、航迹与状态','持续记录新的地图和动作，\n明确为什么继续、返航或失败。',2,size=21)
    p.card(1470,782,345,124,'结果验收独立','保存成功 ≠ 全仓完整；\n世界模型只用于离线评价。',3,size=21)
    p.line([(1390,390),(1445,390)]);p.line([(1390,826),(1445,826)],TEAL)
    p.line([(1790,552),(1875,552),(1875,980),(248,980),(248,950)],TEAL,2,True,True)
    p.text(656,955,'飞行得到新扫描 → 更新地图 → 调整目标与路径',22,color=BLUE,bold=True)
    p.save()

def lifecycle():
    p=Page(2,'图文七步流程','动态黑箱探索：七步图文循环','不是预设路线逐点执行；观察结果决定继续探索、换目标、补扫或返航',
      '机制示意｜动态点过滤不等于旧点撤销；探索结束与完整地图验收分别判断。')
    xs=[75,535,995,1455];w=385
    rows=[('1　起飞与初始化','确认真实起飞、数据新鲜；\n记住起点与飞行高度。','observe'),
          ('2　观察并更新地图','输入扫描与当前位置；\n旧占用接受新证据检查。','map'),
          ('3　选择未知边界视点','比较可见未知和到达代价；\n抑制已访问、失败位置。','frontier'),
          ('4　规划并安全执行','未知不能走，障碍留净空；\n新的人体可打断正常前进。','blocked')]
    for i,(x,(title,body,mode)) in enumerate(zip(xs,rows)):
        p.card(x,250,w,335,title,body,i,size=21)
        if i<2:scene(p,x+28,401,w-56,154,mode)
        else:grid(p,x+28,401,w-56,154,mode)
        if i<3:p.line([(x+w,418),(xs[i+1],418)],BLUE,2.5)
    lower=[(1320,'5　观察不足就换角度补扫','新增信息变少后检查已看表面；\n新通道出现时按条件恢复探索。','rescan'),
           (700,'6　按已知安全路径返航','观测收敛、预算或人工请求；\n无安全返航路不能假装完成。','home'),
           (75,'7　驻留确认、保存并结束','起点附近连续驻留5秒；\n收到保存成功回执才收尾。','save')]
    for i,(x,title,body,mode) in enumerate(lower):
        p.card(x,685,520,295,title,body,4+i,size=22)
        scene(p,x+34,818,451,139,mode)
        if i<2:p.line([(x,833),(lower[i+1][0]+520,833)],BLUE,2.5)
    p.line([(1647,585),(1647,635),(1580,635),(1580,685)],BLUE,2.5)
    p.text(96,613,'到达后观察2秒，再选下一处；路径受阻时重新检查，不沿原命令硬冲。',23,color=BLUE)
    p.line([(1647,250),(1647,212),(726,212),(726,250)],TEAL,2,True,True)
    p.text(793,186,'飞行中的新观测回到第2步',18,color=TEAL)
    p.line([(1495,980),(1495,998),(51,998),(51,648),(726,648),(726,585)],TEAL,2,True,True)
    p.text(1099,648,'补扫后出现新有效前沿 → 再探索',19,color=TEAL)
    p.save()

def blackbox():
    p=Page(3,'黑箱选点与安全规则','黑箱选点：不预知货架，也不把未知当通道','先确定能安全到达哪里，再比较去哪里能看见更多；几何未知不等于没有安全规则',
      '机制示意｜固定高度的八方向加权可达搜索；不是全三维轨迹或未来人体运动的时空规划。')
    p.card(75,230,855,536,'一步观测，生成下一处观察位置','灰：未知　浅绿：观测自由　棕：占用　蓝点：候选',4,size=22)
    grid(p,105,367,794,325,'blocked')
    p.text(107,711,'视点在已知安全边界附近，收益不能越过已测墙体。',23,color=BLUE)
    p.card(975,230,870,224,'规则一：工作窗口靠观测扩展','初始半径12米只是计算窗口，不是仓库轮廓。\n位姿或新鲜自由证据靠近边缘时扩展；新增区域仍未知。\n20万个格为容量上限，不加载货架位置或预设航线。',0)
    p.card(975,483,870,252,'规则二：先可达，再比较未知收益','按已知安全空间搜索；斜走不能切过障碍角。\n观察半径6米，遇占用遮挡停止计算墙后收益。\n近似可见未知少于0.5平方米的候选过滤；\n选点兼顾收益与到达路程，不保证全局最优覆盖。',1)
    p.card(75,800,855,182,'规则三：机体、安全距离和数据时效','规划格0.20米；正常机心净空0.80米已含机体，\n另计格子半对角线余量；当前速度上限0.30米/秒。\n未知空间不可直接通行，过期数据不能继续当新观测。',2,size=21)
    p.card(975,772,870,210,'规则四：不要反复去同一个失败位置','到达后观察2秒；目标25仿真秒无进展则记失败。\n失败区暂避60秒；同处失败两次后持续抑制。\n正常净空不足时只允许严格受约束短恢复，\n仍无进展会报告不安全停滞，不伪装为持续探索。',3,size=21)
    p.save()

def dynamic():
    p=Page(4,'动态变化与旧点撤销','动态环境：现在的人要避开，过去的占用要核实','探索不只选择新区域；当前扫描、可撤销地图和即时安全持续影响通路',
      '机制示意｜探索模式不凭运动轨迹直接擦除；本页不是动态分割精度或全图零鬼影实验。')
    titles=['1　人先站着，进入地图','2　人已离开，旧点仍在','3　射线提供新的空证据','4　撤销旧占用，重看路线']
    bodies=['反复观测会记住这个占用；\n“稳定”并不等于语义静态。',
            '仅过滤新运动点并不够；\n旧点可能仍像假墙挡路。',
            '旧位置被回波前射线穿过；\n当前真实表面仍优先保护。',
            '条件满足后删共享记录；\n新扫描和地图重建当前通道。']
    for i,x in enumerate([75,530,985,1440]):
        p.card(x,240,405,451,titles[i],bodies[i],i,size=21)
        dynamic_scene(p,x+24,381,354,204,i)
        dots(p,x+70,619,275,49,gap=17,hole=i==3)
        if i<3:p.line([(x+405,465),(x+455,465)],BLUE,2.5)
    p.card(75,738,1119,243,'撤销旧点：几何、当前表面与时间条件共同约束','扫描与位姿时间差不超过0.08秒；旧点确在回波前，离射线不超过6厘米。\n回波前留0.30米余量；当前3×3×3邻格有回波则保护。\n距最近命中至少0.5秒，有效空证据间隔不超过1秒；\n非稳定旧格3次、稳定旧格6次才撤销；重新命中则空证据清零。',2,size=22)
    p.card(1240,738,605,243,'当前人仍在：不会因清图而放行','探索用维护图 + 当前扫描重建占用；\n当前人体仍是障碍，未知也不能走。\n即时风险可覆盖导航，必要时保持。\n没有补扫候选且出现新有效前沿时，\n才从补扫按条件恢复探索。',4,size=21)
    p.save()

def rescan():
    p=Page(5,'换角度补扫','换角度补扫：从走过通道，到看清已见表面','同方向反复扫描不代表换了视角；地图点多，也不代表没有被遮挡的缺口',
      '机制示意｜95%仅是已看表面的在线质量；不能证明从未看见的表面已经完整。')
    p.card(75,230,855,482,'换一个可达方向，补充真实观察','补扫只从当前维护图的欠观测表面产生，不偷看世界文件。',1,size=22)
    evidence(p,110,374,781,228)
    p.text(178,653,'第一位置观察 → 沿安全通道换位置 → 再观察',25,color=TEAL,bold=True)
    p.card(975,230,870,222,'转入补扫，不是直接宣布完成','完整90仿真秒窗口：新自由面积少于1平方米，\n新增粗表面格少于25个，且至少到达3个观察目标；\n或者连续30仿真秒没有有用前沿。',0,size=22)
    p.card(975,482,870,252,'表面证据要来自不同时间和方向','0.40米粗表面格：至少3个不同采集时间戳，\n至少2个方向扇区；新增方向的观察位置\n距初次位置至少0.75米。重复发布同一帧不加证据。\n门限是工程规则，不保证真实表面绝不缺点。',2,size=22)
    p.card(75,766,855,215,'补扫位置仍然必须安全、可达','视线不能被已测占用挡住；到达不等于观测合格。\n证据仍不足时，20仿真秒冷却后允许重试。\n维护图中已撤销的旧占用，不继续当永久补扫对象。',4,size=22)
    p.card(975,772,870,209,'两种“95%”不能混用','在线质量：分母是当前已看到的表面格。\n离线覆盖：分母是声明范围内的参考货架侧面。\n从未扫到的整面，在线质量分母可能根本没有它。',3,size=22)
    p.save()

def ending():
    p=Page(6,'返航与结束判断','返航与结束：返回保存，和完整探索分开判断','没有有效新目标不等于全仓扫完；每次收尾必须保留真实原因',
      '机制示意｜默认完成后关闭本次仿真；可显式保留展示。不声称真机自动降落或完全建图。')
    p.card(75,232,852,251,'观测范围收敛：多项条件同时成立','完整增益窗口变小；已看表面质量至少95%；\n无有用前沿、无剩余补扫候选、无目标失败记录，\n观测格与表面证据没有容量不足。\n只能认证当前已观测可达范围，仍不等于全三维完整。',2,size=22)
    p.card(975,232,870,251,'部分结果：预算、限制或人工请求','探索3600仿真秒 / 3600真实秒预算；\n补扫600仿真秒预算；容量不足、视点耗尽、\n人工请求或显式目标数上限，均需写明结束原因。\n部分探索可以返航保存，但完整性不能改成成功。',1,size=22)
    p.line([(501,483),(501,516),(948,516)],BLUE,2.5,False)
    p.line([(1405,483),(1405,516),(948,516),(948,531),(263,531),(263,562)],BLUE,2.5)
    stages=[('按已知安全路返航','无路就报告受阻，\n不直接穿未知。'),('起点附近连续驻留','平面误差≤0.30米，\n持续5仿真秒。'),
            ('取得地图保存回执','失败则保持并重试，\n不是只看文件名。'),('任务结束并记录','返回、保存与完整性\n分别记录。')]
    for i,(x,(title,body)) in enumerate(zip([75,535,995,1455],stages)):
        p.card(x,562,390,217,title,body,4+i,size=21)
        if i==0:scene(p,x+37,679,318,73,'home')
        elif i==1:drone(p,x+96,723,.75);p.circle(x+96,723,43,'none',TEAL,2)
        elif i==2:p.rect(x+39,698,86,48,'#ECF7F0',TEAL);p.text(x+44,711,'已保存',20,color=TEAL,bold=True)
        else:p.text(x+25,702,'结束 ≠ 全图完整',27,color=RED,bold=True)
        if i<3:p.line([(x+390,645),(x+460,645)],BLUE,2.5)
    p.card(75,824,852,157,'连续驻留必须是真实有效状态','离开起点、暂停、过期输入或机位不安全，驻留重新计时。\n断流不继续发布新的探索航点；保存失败不能假报完成。',0,size=22)
    p.card(975,824,870,157,'异常有明确出口，保留部分地图','返航1200仿真秒超时；运行器4800真实秒硬保护。\n受阻、超时或人工中断不是成功探索，不能隐藏失败。',3,size=22)
    p.save()

def result_page(run,meta):
    m=run['mission']
    p=Page(7,'真实点云地图与任务结果','真实结果：长程飞行后，返回起点并保存地图','原保存点云俯视图；同一航次两种输出：左为维护图，右为独立旁路地图',
      '历史实验｜2026-10-03；预算到期、部分探索。旁路地图不负责选目标或控制飞机。')
    p.image(HERE/'证据原件/C04_true_map_comparison.png',73,211,1774,534)
    rows=[('观察目标到达',f"{m['goals_reached']} / {m['goals_selected']}",'不是全目标成功率基准'),
          ('累计平面航程',f"{m['travel_xy_m']:.2f} 米",'由位姿累计，非接触安全证明'),
          ('返航平面误差',f"{m['home_error_xy_m']:.3f} 米",'到起点误差，不是定位精度'),
          ('最终落盘维护图',f"{meta['points']:,} 点",'有限坐标快照，非完整性指标')]
    for i,(title,value,note) in enumerate(rows):
        x=75+i*451;p.card(x,768,415,134,title,'',i)
        p.text(x+21,817,value,34,color=TEAL,bold=True);p.text(x+21,870,note,18,color=GRAY,width=374)
    p.text(78,935,'返航保存：成功；停止原因：真实时间预算到期；探索完整：否；全三维完整：否。',25,color=RED,bold=True)
    p.text(78,978,'原图使用世界高度0.25—1.90米、各自初始刚体对齐；0.15米降采样仅用于显示。',21,color=GRAY)
    p.save()

def rack_map(p,racks,x,y,w,h,title):
    p.illustrations+=1;p.text(x+w/2,y,title,26,bold=True,center=True)
    p.rect(x,y+58,w,h-82,'#FCFDFE','#BCCDD8')
    xs=[r['center_world'][0] for r in racks];ys=[r['center_world'][1] for r in racks]
    lo,hi=min(xs)-2,max(xs)+2;bottom,top=min(ys)-2,max(ys)+2
    for idx,r in enumerate(racks):
        xx=x+25+(r['center_world'][0]-lo)/(hi-lo)*(w-50)
        yy=y+77+(top-r['center_world'][1])/(top-bottom)*(h-124)
        q=r['coverage'];c=TEAL if q>=.9 else '#B7CC68' if q>=.6 else GOLD if q>=.4 else RED
        p.rect(xx-8,yy-13,16,26,c,'white',1)
        if q<.9:p.text(xx+9,yy-15,str(idx+1),15,color=NAVY)
    p.text(x,y+h-19,'位置来自原报告；编号标出不足90%的货架',18,color=GRAY,width=w)

def coverage_page(run,ours,other):
    p=Page(8,'真实覆盖与当前不足','真实结果：有完整轮廓，仍不代表货架表面扫全','依据原报告位置与支持比例绘制；这不是预加载给规划器的世界地图',
      '历史实验｜两图均未通过总体≥95%、逐架≥90%的联合侧面验收；不作动态过滤性能排名。')
    rack_map(p,ours['racks'],85,219,835,299,'维护图：右侧货架仍有欠观测区域')
    rack_map(p,other['racks'],1000,219,835,299,'独立旁路图：缺口较少，仍有不足')
    p.text(579,523,'深绿：至少90%；浅绿、橙、红：不足90%',18,color=GRAY)
    rows=[('同范围的侧面代理指标','维护图','FreeDOM 旁路图'),
          ('参考侧面支持率',f"{ours['shelf_side_coverage']*100:.2f}%",f"{other['shelf_side_coverage']*100:.2f}%"),
          ('最差货架支持率',f"{ours['minimum_rack_coverage']*100:.2f}%",f"{other['minimum_rack_coverage']*100:.2f}%"),
          ('低于90%的货架','10 / 52','3 / 52'),('联合验收','未通过','未通过')]
    for i,row in enumerate(rows):
        for j,text in enumerate(row):
            x=[85,723,1280][j];w=[638,557,555][j];y=550+i*51
            p.rect(x,y,w,51,'#EAF3F8' if i==0 else '#F8FBFD' if i%2 else 'white','#D3E1EA',1)
            p.text(x+18,y+10,text,24,color=NAVY if i==0 or j==0 else RED if i==4 else TEAL,bold=i==0 or j>0)
    p.text(88,816,'范围：52个货架暴露碰撞盒侧面；高0.25—1.90米；0.30米采样，0.25米匹配容差。',21,color=GRAY)
    q=run['mission']['surface_quality']['observed_surface_quality_ratio']*100
    p.card(85,859,835,126,'在线已看质量，也不是全部表面覆盖',f'本次{q:.2f}%，分母只含已看表面；从未看见的面不在其中。',1,size=21)
    p.card(1000,859,835,126,'下一步应验证，而不是只看地图更满','同输入比较鬼影、静态保留、清除时延与安全；\n两图密度/规则不同，这一航次不能作综合性能排名。',3,size=21)
    p.save()

def pack():
    with ZipFile(ARCHIVE,'w',ZIP_DEFLATED) as z:
        for p in sorted(HERE.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                z.write(p,str(Path(HERE.name)/p.relative_to(HERE)))
    print('archive',str(ARCHIVE),ARCHIVE.stat().st_size)

def main():
    if '--pack-only' in sys.argv:pack();return
    facts,run,meta,ours,other=freeze()
    architecture();lifecycle();blackbox();dynamic();rescan();ending();result_page(run,meta);coverage_page(run,ours,other)
    assert len(PAGES)==8 and all(p['illustrations']>0 for p in PAGES if p['number']<=5)
    assert all(t['x']>=0 and t['y']>=0 and t['x']+t['width']<=1920 and t['y']+t['height']<=1080 for t in TEXTS),'text page bounds'
    (HERE/'源码与数据核验.json').write_text(json.dumps(facts,ensure_ascii=False,indent=2))
    (HERE/'版面核验信息.json').write_text(json.dumps({'pages':PAGES,'texts':TEXTS},ensure_ascii=False,indent=2))
    overview=Image.new('RGB',(1440,1770),'#E8EDF1');d=ImageDraw.Draw(overview)
    font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',20)
    for i,p in enumerate(sorted((HERE/'PNG').glob('*.png'))):
        with Image.open(p) as im:thumb=im.convert('RGB').resize((690,388))
        x=15+(i%2)*720;y=12+(i//2)*439;overview.paste(thumb,(x,y));d.text((x,y+393),p.stem,font=font,fill=NAVY)
    overview.save(HERE/'00_八页总预览.png')
    assert all(sha(p)==digest for p,digest in facts['source_hashes'].items())
    print(json.dumps({'pages':len(PAGES),'text_lines':len(TEXTS),'source_checks':len(facts['checks']),'originals_unchanged':True},ensure_ascii=False))

if __name__=='__main__':main()
