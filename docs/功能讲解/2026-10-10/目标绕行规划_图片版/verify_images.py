#!/usr/bin/env python3
"""独立验证实际交付文件；不导入绘图脚本，不启动或修改ROS。"""
import ast
import base64
import hashlib
import json
import re
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from zipfile import ZipFile
from PIL import Image

HERE = Path(__file__).resolve().parent
ARCHIVE = HERE.parent / '目标绕行规划_图片与证据合集.zip'
NS = {'s': 'http://www.w3.org/2000/svg'}
XLINK = '{http://www.w3.org/1999/xlink}href'
CHECKS = []

def check(condition, label):
    if not condition:
        raise AssertionError(label)
    CHECKS.append(label)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def inventory():
    return {str(Path(HERE.name) / p.relative_to(HERE)): p
            for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts}

def verify_archive(allow_report_missing=False):
    files = inventory()
    if allow_report_missing:
        files.pop(str(Path(HERE.name) / '独立核验.json'), None)
    with ZipFile(ARCHIVE) as z:
        check(z.testzip() is None, 'ZIP CRC')
        names = set(z.namelist())
        if allow_report_missing:
            names.discard(str(Path(HERE.name) / '独立核验.json'))
        check(names == set(files), 'ZIP actual file inventory')
        for name, p in sorted(files.items()):
            check(z.read(name) == p.read_bytes(), 'ZIP exact bytes: ' + name)
    return len(files)

def main():
    if '--archive-only' in sys.argv:
        n = verify_archive()
        print('PASS final archive', n, 'files;', ARCHIVE.stat().st_size, 'bytes')
        return
    facts = json.loads((HERE / '源码与数据核验.json').read_text())
    layout = json.loads((HERE / '版面核验信息.json').read_text())
    pngs = sorted((HERE / 'PNG').glob('*.png'))
    svgs = sorted((HERE / 'SVG').glob('*.svg'))
    check(len(pngs) == len(svgs) == 7, 'seven PNG and SVG pages')
    check(not list(HERE.rglob('*.pptx')), 'no PPTX generated')
    roots = {}
    for n, (png, svg) in enumerate(zip(pngs, svgs), 1):
        check(png.stem == svg.stem, f'page {n} paired names')
        with Image.open(png) as im:
            im.load()
            check(im.size == (3840, 2160), f'page {n} 4K dimensions')
            check(im.convert('L').getextrema()[0] < 80, f'page {n} nonblank pixels')
        root = ET.parse(svg).getroot()
        roots[n] = root
        check(root.get('viewBox') == '0 0 1920 1080', f'page {n} editable 16:9 canvas')
        text_nodes = root.findall('s:text', NS)
        page_lines = [t for t in layout['texts'] if t['page'] == n]
        check([t.text or '' for t in text_nodes] == [t['text'] for t in page_lines],
              f'page {n} editable text exactly matches metadata')
        for t in page_lines:
            check(0 <= t['x'] <= t['x'] + t['width'] <= 1920 and
                  0 <= t['y'] <= t['y'] + t['height'] <= 1080,
                  f'page {n} actual text bounds: ' + t['text'])
        for el in text_nodes:
            check(float(el.get('font-size')) >= 18, f'page {n} readable native font')
        for el in root.findall('s:rect', NS) + root.findall('s:image', NS):
            x, y, w, h = [float(el.get(k)) for k in ('x', 'y', 'width', 'height')]
            check(0 <= x <= x + w <= 1920 and 0 <= y <= y + h <= 1080,
                  f'page {n} rectangle/image bounds')
        for el in root.findall('s:circle', NS):
            x, y, r = [float(el.get(k)) for k in ('cx', 'cy', 'r')]
            check(0 <= x-r and x+r <= 1920 and 0 <= y-r and y+r <= 1080,
                  f'page {n} point/icon bounds')
        for el in root.findall('s:polyline', NS) + root.findall('s:polygon', NS):
            coords = [tuple(map(float, pair.split(','))) for pair in el.get('points').split()]
            check(all(0 <= x <= 1920 and 0 <= y <= 1080 for x, y in coords),
                  f'page {n} arrow/outline bounds')
    check(sum('stroke-dasharray' in r.attrib for r in roots[1].findall('s:rect', NS)) >= 5,
          'architecture dashed partitions')
    step_lines = [t.text or '' for t in roots[2].findall('s:text', NS)]
    check(all(any(t.startswith(str(i) + '　') for t in step_lines) for i in range(1, 8)),
          'seven illustrated step titles')
    p2 = next(p for p in layout['pages'] if p['number'] == 2)
    check(p2['illustrations'] == 7 and p2['arrows'] >= 7,
          'seven step illustrations with feedback arrows')
    for n, original, required in [
        (6, 'C05_recovery_route.png', [(31,119,180),(255,127,14)]),
        (7, 'badcase_vs_ours.png', [(230,126,0),(39,174,96)])]:
        nodes = roots[n].findall('s:image', NS)
        check(len(nodes) == 1, f'page {n} one whole original figure')
        embedded = base64.b64decode(nodes[0].get(XLINK).split(',', 1)[1])
        check(embedded == (HERE/'证据原件'/original).read_bytes(),
              f'page {n} original image exact embedded bytes')
        check(nodes[0].get('preserveAspectRatio') == 'xMidYMid meet',
              f'page {n} whole figure aspect kept')
        x,y,w,h=[round(float(nodes[0].get(k))*2) for k in ('x','y','width','height')]
        with Image.open(pngs[n-1]) as im:
            pixels=list(im.convert('RGB').crop((x,y,x+w,y+h)).getdata())
        for color in required:
            # 原图曲线颜色有抗锯齿；容许附近颜色，但必须在原图区域实际渲染。
            count=sum(all(abs(a-b)<28 for a,b in zip(pix,color)) for pix in pixels)
            check(count > 150, f'page {n} rendered curve pixels {color}: {count}')
    for src, expected in facts['source_hashes'].items():
        check(Path(src).is_file() and sha(Path(src)) == expected, 'protected source hash: ' + src)
    for item in facts['copies']:
        src, dst = Path(item['source']), HERE / item['copy']
        check(src.stat().st_size == dst.stat().st_size and sha(src) == sha(dst) == item['sha256'],
              'evidence copy exact: ' + item['copy'])
    for filename in ('draw.py', 'build_images.py', 'verify_images.py'):
        ast.parse((HERE / filename).read_text(), filename=filename)
        check(True, 'Python syntax: ' + filename)
    doc = (HERE / '讲解与实验出处.md').read_text()
    for target in re.findall(r'\]\(([^)]+)\)', doc):
        path = Path(target)
        if not path.is_absolute():
            path = HERE / path
        check(path.exists(), 'documentation link: ' + target)
    check('近终点分支' in doc and '未观测空间不严格禁行' in doc,
          'ordinary-planner and near-goal exceptions documented')
    recovery=json.loads((HERE/'证据原件/ours_cangku_20260927_133611_recovery_complete.json').read_text())
    check(len(recovery)==2 and all(r['arrived'] for r in recovery), 'two historical arrivals')
    check([len(r['samples']) for r in recovery]==[307,351], '658 recovery samples')
    check(all(s['risk']==0 and s['path_poses']>1 for r in recovery for s in r['samples']),
          'recovery samples have no single-pose hold and zero reported risk')
    texts_by_page={n:'\n'.join(t['text'] for t in layout['texts'] if t['page']==n)
                   for n in range(1,8)}
    for r in recovery:
        check(f"{r['error']:.3f}" in texts_by_page[6] and
              f"{r['wall_seconds']:.1f}" in texts_by_page[6],
              'recovery slide metrics from original report')
    bad=json.loads((HERE/'证据原件/badcase_20260923_201211_metrics.json').read_text())
    ours=json.loads((HERE/'证据原件/ours_20260923_200322_metrics.json').read_text())
    for obj in (bad,ours):
        check(obj['goal_reached_odom'], 'example historical goal reached')
        for key in ('time_to_goal_s','gazebo_route_length_after_goal_xy_m',
                    'final_goal_error_odom_xy_m','min_uav_pedestrian_center_xy_m'):
            check(f"{obj[key]:.3f}" in texts_by_page[7], 'example slide original metric: '+key)
    for key, percent in [('time_to_goal_s','39.4'),
                         ('gazebo_route_length_after_goal_xy_m','38.8')]:
        check(f"{(1-ours[key]/bad[key])*100:.1f}" == percent and percent+'%' in texts_by_page[7],
              'single-pair improvement recomputed: '+key)
    check(ours['final_goal_error_odom_xy_m'] > bad['final_goal_error_odom_xy_m'] and
          ours['min_uav_pedestrian_center_xy_m'] < bad['min_uav_pedestrian_center_xy_m'] and
          '没有改善' in texts_by_page[7], 'unimproved outcomes retained')
    file_count=verify_archive(allow_report_missing=True)
    report=dict(status='PASS',local_time=datetime.now().astimezone().isoformat(),
                checks_count=len(CHECKS),checks=CHECKS,
                protected_files=len(facts['source_hashes']),pages=7,
                zip_files_before_report=file_count,
                scope='material-only verification; no new ROS build, flight, safety or performance experiment')
    (HERE/'独立核验.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('PASS independent',len(CHECKS),'checks;',len(facts['source_hashes']),'protected files')

if __name__ == '__main__':
    main()
