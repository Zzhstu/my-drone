#!/usr/bin/env python3
"""独立图文与事实检查。不导入生成器，不启动ROS，不伪造运行结果。"""
import ast
import base64
import hashlib
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import yaml
from PIL import Image

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
B=ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
ARCHIVE=HERE.parent/'动态候选聚类跟踪显示_图片与证据合集.zip'
NS={'s':'http://www.w3.org/2000/svg'}
CHECKS=[]; RULES=[]

def check(ok,msg):
    if not ok: raise AssertionError(msg)
    CHECKS.append(msg)

def rule(ok,msg):
    check(ok,msg);RULES.append(msg)

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def files():
    return {str(Path(HERE.name)/p.relative_to(HERE)):p for p in HERE.rglob('*')
            if p.is_file() and '__pycache__' not in p.parts}

def archive(partial=False):
    inv=files()
    if partial:inv.pop(str(Path(HERE.name)/'独立核验.json'),None)
    with ZipFile(ARCHIVE) as z:
        check(z.testzip() is None,'ZIP CRC verified')
        names=set(z.namelist())
        if partial:names.discard(str(Path(HERE.name)/'独立核验.json'))
        check(names==set(inv),'ZIP complete actual file inventory')
        for name,p in inv.items():check(z.read(name)==p.read_bytes(),'ZIP exact bytes: '+name)
    return len(inv)

def source_rules():
    d=yaml.safe_load((B/'config/dynamic_detector.yaml').read_text())
    expected=dict(cloud_topic='/uav1/fastlio/registered_scan',odom_topic='/uav1/fastlio/odom',
                  output_topic='/uav1/foreground_points',world_frame='map',voxel_size=.25,
                  background_frames=40,background_neighbor_range=1,near_field_range=3.,
                  near_field_background_neighbor_range=1,enable_online_static_update=False,
                  min_relative_z=-.6,max_relative_z=1.2,max_range=6.)
    for k,v in expected.items():rule(d[k]==v,'detector loaded YAML '+k+'='+str(v))
    c=yaml.safe_load((B/'config/dynamic_cluster.yaml').read_text())
    expected=dict(cluster_tolerance=.55,min_cluster_size=4,max_cluster_size=1000,
                  track_match_distance=.8,near_track_match_distance=1.,
                  association_position_tolerance=.2,near_association_position_tolerance=.3,
                  max_association_speed=3.,horizontal_motion_only=True,velocity_history_size=12,
                  position_alpha=.65,motion_position_alpha=.2,velocity_alpha=.2,
                  motion_distance_threshold=.2,motion_speed_threshold=.3,motion_history_time=.5,
                  motion_linearity_threshold=.82,max_velocity_fit_residual=.1,
                  dynamic_confirm_frames=4,static_confirm_frames=3,max_velocity=3.,max_missed_frames=4,
                  near_field_range=3.,near_motion_history_time=.45,
                  near_motion_distance_threshold=.18,near_motion_speed_threshold=.32,
                  near_motion_linearity_threshold=.8,near_max_velocity_fit_residual=.1,
                  near_dynamic_confirm_frames=4,static_support_radius=.35,
                  static_support_ratio=.6,static_support_min_map_points=30)
    for k,v in expected.items():rule(c[k]==v,'cluster loaded YAML '+k+'='+str(v))
    det=(B/'src/dynamic_detector.cpp').read_text()
    cl=(B/'src/dynamic_cluster.cpp').read_text()
    mk=(B/'src/dynamic_marker.cpp').read_text()
    cm=(B/'CMakeLists.txt').read_text()
    launch=ET.parse(B/'launch/perception.launch').getroot()
    for name,yam in [('dynamic_detector','dynamic_detector.yaml'),('dynamic_cluster','dynamic_cluster.yaml')]:
        node=next(n for n in launch.findall('node') if n.get('name')==name)
        rule(any(r.get('file','').endswith(yam) for r in node.findall('rosparam')),'active node loads '+yam)
    rule('dynamic_cluster_node dynamic_cluster.cpp' in cm,'active cluster target compiled')
    rule('dynamic_tracker.cpp' not in cm and 'dynamic_kalman_tracker.cpp' not in cm,'legacy trackers not compiled')
    for s in ['background_building_voxels_.insert(', 'foreground.push_back(voxelCenter(key))',
              'if (!enable_online_static_update_)','normalizedFrame(msg->header.frame_id) !=']:
        rule(s in det,'detector implementation: '+s)
    for s in ['pcl::EuclideanClusterExtraction<PointT>', 'hasStableStaticSupport(*cloud, current_cluster)',
              'predictPosition(tracks_[i], msg->header.stamp)', 'max_association_speed_ * association_dt',
              'velocityFitResidual(track, fitted_velocity)', 'trajectoryLinearity(track)',
              'matched.push_back(true)', 'confirmed_dynamic_points.push_back(',
              'Track predicted_track = track;', 'publishObject(output, predicted_track, true);',
              'obj.size.x = 0.60;', 'obj.size.y = 0.60;', 'obj.size.z = 1.20;',
              'obj.semantic_class = 0;', 'std::max(0.0, std::min(1.0, track.linearity))']:
        rule(s in cl,'cluster implementation: '+s)
    moving=cl.split('bool moving =',1)[1].split('if (moving)',1)[0]
    for s in ['!stable_static_support','fitted_velocity_valid','history_dt >= required_history_time',
              'required_motion_distance','required_motion_speed','track.linearity >= required_linearity',
              'track.fit_residual <= allowed_fit_residual']:
        rule(s in moving,'conjunctive moving condition: '+s)
    for s in ['marker.id=obj.id','ros::Duration(0.5)', 'marker.scale.x=0.3;',
              'marker.scale.y=0.3;', 'marker.scale.z=0.3;', 'obj.predicted ? 0.6 : 0.0',
              'visualization_msgs::Marker::DELETE']:
        rule(s in mk,'display implementation: '+s)
    rule('/uav1/semantic_dynamic_objects'==yaml.safe_load((B/'config/dynamic_risk.yaml').read_text())['objects_topic'],
         'risk consumes semantic forwarding interface')
    rule('if (!semantic_fresh)' in (B/'src/semantic_fusion.cpp').read_text(),'semantic disabled forwarding exists')
    # 防止把图中讲解字当成已经实现的标记类型。
    rule('TEXT_VIEW_FACING' not in mk and 'Marker::ARROW' not in mk,'actual marker is sphere only')

def main():
    if '--archive-only' in sys.argv:
        print('PASS final archive',archive(),'actual files;',ARCHIVE.stat().st_size,'bytes');return
    facts=json.loads((HERE/'源码与数据核验.json').read_text())
    layout=json.loads((HERE/'版面核验信息.json').read_text())
    pngs=sorted((HERE/'PNG').glob('*.png'));svgs=sorted((HERE/'SVG').glob('*.svg'))
    check(len(pngs)==len(svgs)==7,'seven PNG and SVG images')
    check(not list(HERE.rglob('*.pptx')) and not list(HERE.rglob('*.ppt')),'no presentation generated')
    roots={}
    for n,(png,svg) in enumerate(zip(pngs,svgs),1):
        check(png.stem==svg.stem,'paired page '+str(n))
        with Image.open(png) as im:
            im.load();check(im.size==(3840,2160),'4K image '+str(n))
            check(im.convert('L').getextrema()[0]<80,'nonblank page '+str(n))
        r=ET.parse(svg).getroot();roots[n]=r
        check(r.get('viewBox')=='0 0 1920 1080','editable 16:9 canvas '+str(n))
        actual=r.findall('s:text',NS);meta=[t for t in layout['texts'] if t['page']==n]
        check([t.text or '' for t in actual]==[t['text'] for t in meta],'native text exact '+str(n))
        for t in meta:
            check(0<=t['x']<=t['x']+t['width']<=1920 and 0<=t['y']<=t['y']+t['height']<=1080,
                  'text bounds '+str(n)+': '+t['text'])
        for t in actual:check(float(t.get('font-size'))>=20,'font readable '+str(n))
        for el in r.findall('s:rect',NS)+r.findall('s:image',NS):
            x,y,w,h=[float(el.get(k)) for k in ('x','y','width','height')]
            check(0<=x<=x+w<=1920 and 0<=y<=y+h<=1080,'rect or image bounds '+str(n))
        for el in r.findall('s:circle',NS):
            x,y,rad=[float(el.get(k)) for k in ('cx','cy','r')]
            check(0<=x-rad and x+rad<=1920 and 0<=y-rad and y+rad<=1080,'point/icon bounds '+str(n))
        for el in r.findall('s:polyline',NS)+r.findall('s:polygon',NS):
            pts=[tuple(map(float,v.split(','))) for v in el.get('points').split()]
            check(all(0<=x<=1920 and 0<=y<=1080 for x,y in pts),'native line/arrow bounds '+str(n))
    check(sum('stroke-dasharray' in el.attrib for el in roots[1].findall('s:rect',NS))>=5,'dashed architecture partitions')
    titles=[t.text or '' for t in roots[2].findall('s:text',NS)]
    check(all(any(t.startswith(str(i)+'　') for t in titles) for i in range(1,8)),'seven numbered illustrated stages')
    page2=layout['pages'][1]
    check(page2['illustrations']==7 and page2['arrows']>=7,'seven scene illustrations and feedback loop')
    imgs=roots[6].findall('s:image',NS)
    check(len(imgs)==2,'two original historical images')
    for el,name in zip(imgs,['ours_cangku_20260927_121518_rviz.png','ours_cangku_20260927_121518_gazebo.png']):
        data=base64.b64decode(el.get('{http://www.w3.org/1999/xlink}href').split(',',1)[1])
        check(data==(HERE/'证据原件'/name).read_bytes(),'original screenshot embedded intact '+name)
        check(el.get('preserveAspectRatio')=='xMidYMid meet','original screenshot aspect '+name)
        x,y,w,h=[round(float(el.get(k))*2) for k in ('x','y','width','height')]
        with Image.open(pngs[5]) as im:
            pixels=list(im.convert('RGB').crop((x,y,x+w,y+h)).getdata())
        check(sum(max(p)-min(p)>30 for p in pixels)>2000,'actual color content rendered '+name)
        check(sum(max(p)<70 for p in pixels)>2000,'actual dark screenshot content rendered '+name)
    for src,h in facts['source_hashes'].items():check(sha(Path(src))==h,'protected original hash '+src)
    for row in facts['copies']:
        src=Path(row['source']);dst=HERE/row['copy']
        check(src.stat().st_size==dst.stat().st_size and sha(src)==sha(dst)==row['sha256'],'exact evidence copy '+row['copy'])
    for name in ('prepare_evidence.py','draw.py','build_images.py','verify_images.py'):
        ast.parse((HERE/name).read_text(),filename=name);check(True,'script syntax '+name)
    doc=(HERE/'讲解与实验出处.md').read_text()
    for target in re.findall(r'\]\(([^)]+)\)',doc):
        path=Path(target);path=path if path.is_absolute() else HERE/path
        # renderer links can contain a single source line suffix.
        path=Path(re.sub(r':\d+$','',str(path)))
        check(path.exists(),'actual document path '+target)
    for s in ['不是40次多数投票','不是卡尔曼状态估计','0.55米不限制整个簇的总直径',
              '不是校准', '上游完全停止回调时', 'confidence字段取轨迹方向一致性']:
        # 本文对“不是校准”的具体说法是“不是已校准”。
        term='不是已校准' if s=='不是校准' else s
        check(term in doc,'implementation caveat '+term)
    warehouse=json.loads((HERE/'证据原件/ours_cangku_20260927_121518_inflight_test.json').read_text())
    example=json.loads((HERE/'证据原件/ours_20260924_144552_manual_test.json').read_text())
    expected=[(66,40,40,0,26),(64,43,39,4,21)]
    for key,want in zip(('cross_first','cross_repeat'),expected):
        r=warehouse['phases'][key]
        got=(r['messages'],r['detected'],r['observed'],r['detected']-r['observed'],r['messages']-r['detected'])
        rule(got==want,'original warehouse phase counts '+key)
    text7='\n'.join(t['text'] for t in layout['texts'] if t['page']==7)
    rule('观测 40　预测 0　未匹配 26　共 66 条消息' in text7,'first phase rendered counts')
    rule('观测 39　预测 4　未匹配 21　共 64 条消息' in text7,'repeat phase rendered counts')
    for key,count,error,age in [('cross_first',50,.08051658749940833,.02695),('cross_repeat',42,.14226784847784524,.026)]:
        r=example['phases'][key]
        rule(r['observed']==count and abs(r['center_error_median']-error)<1e-10 and abs(r['age_p95_s']-age)<1e-10,
             'original example observed/error/age '+key)
        rule(f'{error:.3f}' in text7,'rounded original center error on image '+key)
    rule('不是逐点分割准确率' in '\n'.join(t['text'] for t in layout['texts']),'visual metric boundary')
    source_rules()
    n=archive(partial=True)
    report=dict(status='PASS',local_time=datetime.now().astimezone().isoformat(),
                checks_count=len(CHECKS),source_and_data_assertions=len(RULES),rules=RULES,
                checks=CHECKS,protected_files=len(facts['source_hashes']),pages=7,
                actual_zip_files_before_report=n,
                scope='only material verification; no new build, ROS flight, dynamic detection accuracy or resource test')
    (HERE/'独立核验.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('PASS',len(CHECKS),'independent checks;',len(RULES),'source/data assertions;',len(facts['source_hashes']),'protected files')

if __name__=='__main__':main()
