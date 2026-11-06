#!/usr/bin/env python3
"""七页源码对应材料；仅制图、复制证据，不更改运行工程。"""
import hashlib
import json
import shutil
import sys
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import yaml
from PIL import Image, ImageDraw, ImageFont
from draw import Page, TEXTS, PAGES, HERE, NAVY, BLUE, TEAL, GRAY, RED, GOLD, CYAN, scene, grid, evidence, drone

ROOT=HERE.parents[3]
B=ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
PREFIX=ROOT/'exploration_demo_runs/warehouse_20261003_222021'
ASSETS=HERE/'证据原件'
ARCHIVE=HERE.parent/'黑箱探索补扫返航_图片与证据合集.zip'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def data(suffix):return json.loads(Path(str(PREFIX)+suffix).read_text())

def source_check():
    config=yaml.safe_load((B/'config/warehouse_exploration.yaml').read_text())
    expected={'initial_window_radius':12.,'growth_margin':4.,'growth_chunk':8.,'max_grid_cells':200000,
      'resolution':.2,'clearance':.8,'recovery_minimum_clearance':.45,'unsafe_stall_seconds':30.,
      'convergence_seconds':90.,'convergence_free_area':1.,'convergence_new_surfaces':25,
      'convergence_min_goals':3,'surface_voxel':.4,'surface_min_hits':3,'surface_min_views':2,
      'surface_view_baseline':.75,'surface_quality_target':.95,'inspection_revisit_seconds':20.,
      'inspection_budget_seconds':600.,'home_tolerance':.3,'home_settle_seconds':5.,
      'max_mission_seconds':3600.,'max_mission_wall_seconds':3600.,'return_timeout':1200.,
      'frontier_min_gain_area':.5,'frontier_sensor_range':6.,'settle_seconds':2.,
      'progress_timeout':25.,'blacklist_seconds':60.,'max_target_attempts':2}
    checks=[]
    for k,v in expected.items():
        assert config[k]==v,(k,config[k]);checks.append(k)
    texts={p.name:p.read_text() for p in [B/'scripts/warehouse_explorer.py',B/'scripts/exploration_grid.py',
           B/'scripts/exploration_coverage.py',B/'scripts/run_warehouse_exploration.py',B/'launch/warehouse_exploration.launch']}
    for name,fragments in {
      'warehouse_explorer.py':["coverage_complete=False","'observed_reachable_converged'","'wall_budget_partial'","not self.failed_visits","if self.save():","'STALLED_UNSAFE'","scores = [gain[tuple(c)]/(2.+distances[tuple(c)])"],
      'exploration_grid.py':['grow_to_include','constant_values=False','math.sqrt(2)/2'],
      'exploration_coverage.py':['stamp <= self.last_stamp','np.linalg.norm(observer-record[2])>=self.baseline','keys = self.keys(current_map)','grid.occupied'],
      'run_warehouse_exploration.py':['default=4800.','returned_home_and_saved','SITL only: save then stop owned simulation processes; no physical landing claim'],
      'warehouse_exploration.launch':['max_navigation_speed" value="0.30','enable_path_planner" value="false','enable_loop_closure" value="false','evidence_only_clearing" value="true']}.items():
        for fragment in fragments:assert fragment in texts[name],fragment;checks.append(name+':'+fragment)
    run=data('_result.json');m=run['mission'];ours=data('_coverage.json');free=data('_freedom_coverage.json');meta=data('_final_map.json')
    assert m['goals_reached']==93 and m['goals_selected']==97
    assert run['termination']=='returned_home_and_saved' and m['finish_reason']=='wall_budget_partial'
    assert m['home_confirmed'] and not m['exploration_complete'] and not m['coverage_complete']
    assert meta['points']==284418 and sha(Path(str(PREFIX)+'_final_map.pcd'))==meta['sha256']
    assert ours['supported_samples']==9267 and free['supported_samples']==10091 and ours['reference_samples']==10192
    assert len(ours['racks'])==52 and not ours['shelf_side_acceptance'] and not free['shelf_side_acceptance']
    assert sum(r['coverage']<.9 for r in ours['racks'])==10 and sum(r['coverage']<.9 for r in free['racks'])==3
    checks+=['historical lifecycle and partial','PCD bytes match metadata','two original coverage reports']
    sources=[*[B/p for p in ['scripts/warehouse_explorer.py','scripts/exploration_grid.py','scripts/exploration_coverage.py',
        'scripts/exploration_native.py','src/exploration_search.cpp','scripts/run_warehouse_exploration.py',
        'scripts/warehouse_map_audit.py','config/warehouse_exploration.yaml','launch/warehouse_exploration.launch']]]
    copied=[];(HERE/'证据原件').mkdir(exist_ok=True)
    for suffix in ['_result.json','_coverage.json','_freedom_coverage.json','_final_map.json','_alignment.json','_freedom_alignment.json','_scene.json']:
        src=Path(str(PREFIX)+suffix);sources.append(src);dst=HERE/'证据原件'/src.name
        shutil.copy2(src,dst);copied.append(dict(source=str(src),copy=str(dst.relative_to(HERE)),sha256=sha(src)))
    for name in ['C02_rack_support.png','C03_rack_support_world.png','C04_true_map_comparison.png']:
        src=ASSETS/name;sources.append(src);dst=HERE/'证据原件'/name
        if src.resolve()!=dst.resolve():shutil.copy2(src,dst)
        copied.append(dict(source=str(src),copy=str(dst.relative_to(HERE)),sha256=sha(src)))
    # Keep all previous loop images and their generator unchanged.
    sources.extend(p for p in (HERE.parent/'独立回环后端_图片版').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    sources.extend(Path(str(PREFIX)+s) for s in ['_final_map.pcd','_freedom_static_map_point.pcd'])
    return dict(checks=checks,source_hashes={str(p):sha(p) for p in sources},copies=copied,
                experiment=str(PREFIX),validation_scope='materials and historical evidence, not a new ROS experiment'),run,ours,free,meta

def architecture():
    p=Page(1,'模块架构','黑箱自主探索：边飞边认识仓库','没有预设货架布局或飞行路线；由观测生成地图、目标和动作','机制示意｜固定高度的平面探索，不等于任意三维空间的完整扫描。')
    p.rect(75,249,380,695,'none',BLUE,2,True);p.text(265,218,'传感器与状态输入',29,bold=True,center=True)
    p.card(102,290,326,186,'位置与当前点云','回答：我在哪？\n眼前有什么物体？',4,size=20)
    scene(p,132,412,267,59,'observe')
    p.card(102,500,326,186,'可更新的维护地图','保留静态结构；旧占用\n须有新观测证据才能撤销。',2,size=20)
    scene(p,132,622,267,59,'map')
    p.card(102,710,326,184,'真实起飞与数据时效','确认已起飞、数据未过期；\n没有有效输入就先保持。',0)
    drone(p,164,849,.85);p.text(214,825,'先确认，再规划',20,color=BLUE)
    p.rect(506,249,926,695,'none',BLUE,2,True);p.text(969,218,'自主决策与任务管理',29,bold=True,center=True)
    p.rect(535,286,868,290,'#FFF1BF',BLUE,1.5,True)
    p.text(557,303,'认识环境 → 选择观察位置',28,bold=True)
    p.card(558,362,366,178,'观测格与安全通道','区分未知、自由与占用；\n只在已知安全空间走。',4,size=20)
    grid(p,580,485,324,43)
    p.card(977,362,402,178,'探索目标与路径','优先可看见更多未知的位置；\n计入路程和障碍遮挡。',1,size=20)
    scene(p,1003,487,349,42,'observe')
    p.line([(924,448),(977,448)])
    p.rect(535,635,868,269,'#E9F3E7',BLUE,1.5,True)
    p.text(557,651,'多角度补扫 → 返航 → 保存',28,bold=True)
    p.card(558,710,366,151,'已看表面的质量检查','观察太少就换位置补扫；\n发现新通道可恢复探索。',2)
    p.card(977,710,402,151,'任务结束与保存确认','回到起点，持续驻留5秒；\n地图保存成功后才结束。',5)
    p.line([(1180,576),(1180,635)],TEAL,2.5)
    p.text(1194,593,'增益变小 / 无有用前沿',19,color=TEAL)
    p.line([(924,787),(977,787)],TEAL,2.5)
    p.line([(588,635),(588,606),(557,606),(557,576)],TEAL,2,True,True)
    p.text(650,588,'新通道出现：再次探索',19,color=TEAL)
    p.line([(455,351),(506,351)])
    p.line([(455,587),(489,587),(489,543),(506,543)],TEAL)
    p.line([(455,802),(506,802)])
    p.rect(1486,249,353,695,'none',BLUE,2,True);p.text(1663,218,'执行与结果',29,bold=True,center=True)
    p.card(1513,310,300,188,'安全控制与反馈','执行观察航点；遇到真实\n障碍或过期数据先保持。',4)
    drone(p,1658,459,.95)
    p.card(1513,550,300,145,'轨迹、点云与状态','记录走过的路、维护地图\n和明确的结束原因。',2)
    p.card(1513,743,300,151,'独立地图验收','检查货架侧面缺口；\n真值不作为规划输入。',3)
    p.line([(1432,448),(1486,448)])
    p.line([(1432,787),(1460,787),(1460,619),(1486,619)],TEAL)
    p.line([(1813,409),(1870,409),(1870,981),(265,981),(265,944)],TEAL,2,True,True)
    p.text(640,948,'飞行产生新观测：持续改变下一轮决策',23,color=BLUE,bold=True)
    p.save()

def lifecycle():
    p=Page(2,'图文七步流程','探索、补扫、返航：七步图文流程','四上三下的主流程；遇到新未知区域或路径变化，会回到前面的步骤','机制示意｜补扫不是每到一个目标就执行；是否转入补扫由观测增益和候选状态决定。')
    upper=[(85,258,395,326),(535,258,395,326),(985,258,395,326),(1435,258,395,326)]
    contents=[('1　起飞与初始化','确认起飞和新鲜数据，\n记录起点及固定飞行高度。'),
      ('2　建立观测格','激光看到哪里才认识哪里；\n未知格不能直接当通道。'),
      ('3　选择未知边界视点','比较能看到的未知范围，\n同时考虑到达路程。'),
      ('4　安全规划并飞过去','路径考虑机体与净空；\n当前障碍变化时重新检查。')]
    for i,((x,y,w,h),(title,body)) in enumerate(zip(upper,contents)):
        p.card(x,y,w,h,title,body,i)
        if i==0:scene(p,x+24,y+151,w-48,144,'observe')
        else:grid(p,x+24,y+150,w-48,144,'blocked' if i==3 else 'initial' if i==1 else 'frontier')
        if i<3:p.line([(x+w,422),(upper[i+1][0],422)],BLUE,2.5)
    lower=[(1295,691,535,292),(695,691,535,292),(85,691,535,292)]
    for idx,((x,y,w,h),title,body,mode) in enumerate(zip(lower,
        ['5　增益变小，换角度补扫','6　按已知安全路径返航','7　驻留确认、保存并结束'],
        ['检查已有表面还缺哪些视角；\n新通道出现时可回到探索。','收敛、预算到期或人工请求返航；\n返航受阻不能当作成功。','起点附近连续5秒且保存成功；\n任务结束与地图完整分开记录。'],
        ['rescan','home','save'])):
        p.card(x,y,w,h,title,body,4+idx)
        scene(p,x+40,y+141,w-80,125,mode)
        if idx<2:p.line([(x,830),(lower[idx+1][0]+lower[idx+1][2],830)],BLUE,2.5)
    p.line([(1632,584),(1632,643),(1562,643),(1562,691)],BLUE,2.5)
    p.text(985,600,'正常推进：到达后观察2秒，再选目标',21,color=BLUE)
    p.text(1660,601,'增益小 / 无前沿',19,color=TEAL)
    p.line([(1523,258),(1523,220),(731,220),(731,258)],TEAL,2,True,True)
    p.text(811,190,'飞行获得新观测：更新观测格',19,color=TEAL)
    p.line([(1405,983),(1405,1001),(52,1001),(52,644),(731,644),(731,584)],TEAL,2,True,True)
    p.text(85,610,'补扫打开新区域：回到探索；路线受阻：另选目标',21,color=TEAL)
    p.save()

def blackbox():
    p=Page(3,'黑箱选点与安全规则','黑箱探索的规则：没有地图，也不盲目往前冲','“黑箱”指仓库几何未知，不是取消安全边界；飞行仍保持一个高度','机制示意｜当前人体即使不进入长期地图，当前扫描中仍作为障碍。')
    p.card(75,230,840,520,'从一次观测，到一个可到达的新视点','灰色：未知　绿色：已看自由空间　棕色：占用　蓝点：候选视点',4)
    grid(p,109,369,772,308,'blocked')
    p.text(107,704,'候选点在已知安全边界附近，不在未知障碍后面直接落点。',22,color=BLUE)
    p.card(962,230,881,228,'规则一：范围来自观测，而不是世界文件','初始计算窗口以地图原点为中心，半径12米。\n飞到边缘或发现新的自由空间后扩展；新增格保持未知。\n20万个格是资源上限，不代表仓库的真实轮廓。',0)
    p.card(962,483,881,241,'规则二：优先“看得多、走得到、路程合理”','先找安全可达的未知边界，再计算附近6米内的可见未知。\n射线被已测墙体挡住就停止，不把墙后未知算作可见收益。\n小于0.5平方米的收益候选过滤；到过的位置和失败位置受抑制。',1)
    p.card(75,785,840,194,'规则三：路线考虑机体和窄通道','0.20米格；正常机心净空0.80米，已含约0.40米机体半径，\n另加栅格半对角线余量；不沿障碍角斜切。未知也不可通行。',2)
    p.card(962,754,881,225,'规则四：卡住要换方案，不能无声永久悬停','目标25秒无进展或变得不可达：记录失败，暂避60秒再选点。\n同处达到两次失败会持续抑制。当前机位不安全时仅尝试\n受约束短恢复；30秒无0.20米进展则报告失败并保存部分图。',3)
    p.save()

def rescan():
    p=Page(4,'补扫与质量规则','补扫：从“走过通道”转向“看清货架”','点云看起来很多，不代表每一面都看清；同一个方向反复扫描也不等于多视角','机制示意｜95%是当前已看表面的观测质量，不包含从未看到的表面，不是全仓覆盖率。')
    p.card(75,232,840,453,'为什么要换位置：遮挡会留下缺口','同一侧扫描，只增加类似回波；走到另一个可达方向才可能补充信息。',1)
    evidence(p,121,393,741,213)
    p.text(495,625,'第一视点 → 绕到另一侧 → 再次观察',24,color=TEAL,bold=True,center=True)
    p.card(960,232,884,244,'什么时候进入补扫？','完整90秒观测窗口内：新自由面积少于1平方米，\n新增粗表面格少于25个，且窗口内至少到达3个观察目标。\n或者：连续30秒找不到有用前沿。低增益只触发补扫，不直接结束。',0)
    p.card(960,503,884,235,'一个表面格怎样才算“观察得够了”？','用0.40米的三维粗格计数，至少3次不同采集时刻。\n还需至少2个方向扇区；新增方向的位置距初次观察至少0.75米。\n重复发布同一帧不加次数；初次方向与后续方向共同构成证据。',2)
    p.card(75,726,840,249,'补扫目标怎么选？','只从当前维护地图的欠观测表面产生目标；\n换角度的位置必须安全、可达，并且未被已测障碍挡住。\n到达不等于观察合格；证据不足时，20秒冷却后可再次尝试。\n地图中已撤销的人体旧点，不再作为永久补扫对象。',4)
    p.card(960,773,884,202,'补扫之后仍可能继续探索','没有补扫候选却出现新有效前沿：重置增益窗口，继续探索。\n补扫预算600仿真秒耗尽，或可达视点用尽：返航并标为部分结果。',5)
    p.line([(917,879),(960,879)],TEAL,2.5)
    p.save()

def ending():
    p=Page(5,'返航与结束判断','返航和结束：需要确认，不是绕一圈就算完成','任务结束有两种口径：正常观测收敛，或带原因的部分结果返航','机制示意｜这里的返航是在起点附近悬停确认；不声称真机自动降落，也不保证全三维地图完整。')
    p.card(75,228,835,230,'正常收敛返航：要同时满足多项条件','完整增益窗口已经变小；已看表面质量至少95%；\n没有有用前沿、没有剩余补扫候选；没有目标失败记录，\n表面证据与观测格没有容量不足。只认证当前可观测、可达范围。',2)
    p.card(965,228,879,230,'部分结果返航：必须写明原因','探索3600仿真秒或3600真实秒预算到期；补扫预算到期；\n视点用尽仍未合格；容量不足；人工请求或显式目标上限。\n返回并保存可以成功，但“探索完整”不能因此改成成功。',1)
    p.line([(496,458),(496,495),(950,495),(950,530)],BLUE,2.5)
    p.line([(1403,458),(1403,495),(950,495)],BLUE,2.5,False)
    steps=[(75,'按已知安全路返航','新障碍出现时继续更新路；\n没有安全路则报告受阻。'),
           (535,'起点内连续驻留','平面误差不超过0.30米，\n持续保持5个仿真秒。'),
           (995,'保存成功回执','保存失败就保持并重试；\n不能只看文件名出现。'),
           (1455,'结束本次任务','记录终止原因和完整性；\n默认关闭本次仿真进程。')]
    for i,(x,title,body) in enumerate(steps):
        p.card(x,544,388,224,title,body,4+i)
        if i==0:scene(p,x+35,661,315,83,'home')
        elif i==1:
            p.circle(x+86,710,31,'none',TEAL,2);drone(p,x+86,710,.58)
            p.text(x+150,687,'0.30米 / 5秒',20,color=TEAL,bold=True)
        elif i==2:
            p.rect(x+38,694,80,43,'#FFFDF2',BLUE);p.text(x+48,700,'已保存',18,color=TEAL,bold=True)
            p.line([(x+130,715),(x+291,715)],TEAL,2.5)
        else:
            p.text(x+28,701,'任务成功 ≠ 地图完整',24,color=RED,bold=True)
        if i<3:p.line([(x+388,618),(steps[i+1][0],618)],BLUE,2.5)
    p.line([(950,530),(950,535),(270,535),(270,544)],BLUE,2)
    p.card(75,818,835,165,'连续性检查：离开或数据失效要重新计时','偏离起点、暂停、数据过期或机位不安全，都会重置驻留计时。\n缺少有效扫描时不继续发新探索航点，由控制器看门狗保持。',0)
    p.card(965,818,879,165,'异常保护：保存部分地图，不伪装成成功','返航超过1200仿真秒报告失败；运行器4800真实秒硬保护。\n显式保持展示模式可在完成后保留画面，不再进行探索。',3)
    p.save()

def result_page(run,meta):
    m=run['mission']
    p=Page(6,'真实地图与任务结果','真实实验：已完成长程探索与返航保存','复用原保存点云的俯视结果图；同一航次，左为我们的维护图，右为并行FreeDOM地图','真实结果｜2026-10-03长程仿真；预算到期返航，不是完整仓库建图成功。')
    p.image(HERE/'证据原件/C04_true_map_comparison.png',70,205,1780,548)
    metrics=[('观察目标到达',f"{m['goals_reached']} / {m['goals_selected']}",'选出的目标并非全部到达'),
       ('实际平面航程',f"{m['travel_xy_m']:.2f} 米",'里程计累计平面位移'),
       ('返航平面误差',f"{m['home_error_xy_m']:.3f} 米",'回到起点，不是定位精度'),
       ('最终落盘地图',f"{meta['points']:,} 点",'维护地图的有限坐标点')]
    for i,(title,value,note) in enumerate(metrics):
        x=75+i*447
        p.card(x,775,416,131,title,'',i)
        p.text(x+20,823,value,34,color=TEAL,bold=True)
        p.text(x+20,873,note,19,color=GRAY)
    p.text(78,933,'返航与保存：成功；停止原因：真实时间预算到期；探索完整：否；全三维完整：否。',25,color=RED,bold=True)
    p.text(78,976,'图示高度为世界坐标0.25—1.90米；仅显示降采样，不把地图点数当质量排名。',21,color=GRAY)
    p.save()

def coverage_map(p,racks,x,y,w,h,title):
    p.illustrations+=1
    p.text(x+w/2,y,title,26,bold=True,center=True)
    p.rect(x,y+62,w,h-99,'#FBFCFD','#B8CCD7')
    xx=[r['center_world'][0] for r in racks];yy=[r['center_world'][1] for r in racks]
    lo,hi=min(xx)-2,max(xx)+2;bottom,top=min(yy)-2,max(yy)+2
    for i,r in enumerate(racks):
        a=x+25+(r['center_world'][0]-lo)/(hi-lo)*(w-50)
        b=y+80+(top-r['center_world'][1])/(top-bottom)*(h-141)
        q=r['coverage'];c=TEAL if q>=.9 else '#B7CC68' if q>=.6 else GOLD if q>=.4 else RED
        p.rect(a-9,b-16,18,32,c,'white',1)
        if q<.9:p.text(a+10,b-17,str(i+1),16,color=NAVY)
    p.text(x,y+h-27,'真实货架中心位置；数字是未达标货架编号',19,color=GRAY,width=w)

def audit_page(ours,free,run):
    p=Page(7,'真实覆盖与缺口','真实实验：整体看起来完整，局部货架仍有缺口','下图依据原审计JSON的货架位置与比例绘制，不是示意布局；红、橙、浅绿表示不足90%','真实结果｜两幅地图都没有通过“总体至少95%、每架至少90%”的联合侧面验收。')
    coverage_map(p,ours['racks'],90,215,830,300,'我们的维护地图：右侧货架存在缺口')
    coverage_map(p,free['racks'],1000,215,830,300,'FreeDOM并行地图：缺口较少但仍存在')
    rows=[('同口径指标','我们的维护地图','FreeDOM并行地图'),
          ('参考侧面支持率',f"{100*ours['shelf_side_coverage']:.2f}%",f"{100*free['shelf_side_coverage']:.2f}%"),
          ('最差货架支持率',f"{100*ours['minimum_rack_coverage']:.2f}%",f"{100*free['minimum_rack_coverage']:.2f}%"),
          ('低于90%的货架','10 / 52','3 / 52'),('联合验收结果','未通过','未通过')]
    for i,row in enumerate(rows):
        y=551+i*53
        for j,t in enumerate(row):
            x=[90,725,1287][j];w=[635,562,543][j]
            p.rect(x,y,w,53,'#E9F2F7' if i==0 else '#F6FAFC' if i%2 else 'white','#D3E0E8',1)
            p.text(x+20,y+13,t,23,color=NAVY if i==0 else RED if i==4 else TEAL if j else NAVY,bold=i==0 or j>0)
    q=100*run['mission']['surface_quality']['observed_surface_quality_ratio']
    p.card(90,853,831,137,'在线“已看质量”与离线“参考覆盖”不同',f'在线质量{q:.2f}%只对当前已看表面计数，不能发现从未看到的整面。',1,size=21)
    p.card(1000,853,830,137,'我们仍需改进：视角覆盖与静态保留','这是一航次的代理几何指标；不能由缺口直接认定全部是误删，\n也不能由此判断人体去除精度。FreeDOM不参与本次飞行控制。',3,size=21)
    p.text(92,824,'评估：52个货架暴露侧面，参考高度0.25—1.90米，采样0.30米，匹配容差0.25米。',21,color=GRAY)
    p.save()

def pack():
    with ZipFile(ARCHIVE,'w',ZIP_DEFLATED) as z:
        for p in sorted(HERE.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts:
                z.write(p,str(Path(HERE.name)/p.relative_to(HERE)))
    print('archive',ARCHIVE,'bytes',ARCHIVE.stat().st_size)

def main():
    if '--pack-only' in sys.argv:pack();return
    facts,run,ours,free,meta=source_check()
    architecture();lifecycle();blackbox();rescan();ending();result_page(run,meta);audit_page(ours,free,run)
    assert all(t['x']>=0 and t['y']>=0 and t['x']+t['width']<=1920 and t['y']+t['height']<=1080 for t in TEXTS),'text bounds'
    (HERE/'源码与数据核验.json').write_text(json.dumps(facts,ensure_ascii=False,indent=2))
    (HERE/'版面核验信息.json').write_text(json.dumps(dict(texts=TEXTS,pages=PAGES),ensure_ascii=False,indent=2))
    overview=Image.new('RGB',(1440,4*435+50),'#E8EDF1');dr=ImageDraw.Draw(overview)
    f=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',20)
    for i,p in enumerate(sorted((HERE/'PNG').glob('*.png'))):
        with Image.open(p) as im:thumb=im.convert('RGB').resize((690,388))
        x=15+(i%2)*720;y=12+(i//2)*435;overview.paste(thumb,(x,y));dr.text((x,y+391),p.stem,font=f,fill=NAVY)
    overview.save(HERE/'00_七页总预览.png')
    assert all(sha(Path(p))==digest for p,digest in facts['source_hashes'].items())
    pack();print('pages',len(PAGES),'text lines',len(TEXTS),'source checks',len(facts['checks']))

if __name__=='__main__':main()
