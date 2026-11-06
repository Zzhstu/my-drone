#!/usr/bin/env python3
"""独立检查图文产物及原证据，不导入绘制脚本，不启动ROS。"""
import ast
import base64
import hashlib
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile
from PIL import Image, ImageChops

HERE = Path(__file__).resolve().parent
ARCHIVE = HERE.parent / '动态黑箱探索_八张图片与证据合集.zip'
CHECKS = []

def check(condition, label):
    if not condition:
        raise AssertionError(label)
    CHECKS.append(label)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load(name):
    return json.loads((HERE / name).read_text())

def archive_check():
    with ZipFile(ARCHIVE) as z:
        check(z.testzip() is None, '压缩包CRC')
        expected = {str(Path(HERE.name) / p.relative_to(HERE)): p
                    for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        check(set(z.namelist()) == set(expected), '全部文件实际打包，无遗漏或额外条目')
        for name, path in expected.items():
            check(hashlib.sha256(z.read(name)).hexdigest() == sha(path), '压缩包字节 ' + name)
    print(json.dumps({'archive': str(ARCHIVE), 'files': len(expected), 'checks': len(CHECKS),
                      'status': 'PASS'}, ensure_ascii=False))

def main():
    if '--archive-only' in sys.argv:
        archive_check()
        return
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PyQt5.QtCore import QByteArray, QRectF
    from PyQt5.QtGui import QImage, QPainter
    from PyQt5.QtSvg import QSvgRenderer
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    pngs = sorted((HERE/'PNG').glob('*.png'))
    svgs = sorted((HERE/'SVG').glob('*.svg'))
    check(len(pngs) == len(svgs) == 8, '八张PNG与八张SVG')
    check([p.stem for p in pngs] == [s.stem for s in svgs], 'PNG/SVG一一对应')
    check([p.stem[:2] for p in pngs] == ['%02d' % i for i in range(1, 9)], '连续八页')
    texts = {}
    ns = {'s': 'http://www.w3.org/2000/svg'}
    for i, (png, svg) in enumerate(zip(pngs, svgs), 1):
        raw = svg.read_bytes()
        root = ET.fromstring(raw)
        check(root.attrib['viewBox'] == '0 0 1920 1080', '%d矢量画布' % i)
        texts[i] = '\n'.join(t.text or '' for t in root.findall('.//s:text', ns))
        check('%02d / 08' % i in texts[i], '%d页码' % i)
        check(len(re.findall(r'[\u4e00-\u9fff]', texts[i])) > 100, '%d中文文字' % i)
        check(len(root.findall('.//s:rect', ns)) >= 5, '%d原生彩色模块' % i)
        if i <= 6:
            check(len(root.findall('.//s:circle', ns)) > 8, '%d图内无人机/点云/证据图形' % i)
            check(len(root.findall('.//s:polygon', ns)) > 0, '%d插图及箭头' % i)
        with Image.open(png) as im:
            im.load()
            check(im.size == (3840, 2160), '%d图片4K尺寸' % i)
            renderer = QSvgRenderer(QByteArray(raw))
            check(renderer.isValid(), '%d矢量可渲染' % i)
            output = QImage(3840, 2160, QImage.Format_ARGB32)
            output.fill(0xFFFFFFFF)
            painter = QPainter(output)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setRenderHint(QPainter.TextAntialiasing)
            renderer.render(painter, QRectF(0, 0, 3840, 2160))
            painter.end()
            output = output.convertToFormat(QImage.Format_RGBA8888)
            ptr = output.bits()
            ptr.setsize(output.byteCount())
            fresh = Image.frombytes('RGBA', (3840, 2160), bytes(ptr))
            check(ImageChops.difference(im.convert('RGB'), fresh.convert('RGB')).getbbox() is None,
                  '%d独立重渲染像素完全一致' % i)
        if i == 7:
            imgs = root.findall('.//s:image', ns)
            check(len(imgs) == 1, '07原结果图唯一完整嵌入')
            href = imgs[0].attrib['{http://www.w3.org/1999/xlink}href']
            original = (HERE/'证据原件/C04_true_map_comparison.png').read_bytes()
            check(base64.b64decode(href.split(',', 1)[1]) == original, '07嵌入原图逐字节一致')
    bounds = load('版面核验信息.json')
    for t in bounds['texts']:
        check(0 <= t['x'] <= t['x'] + t['width'] <= 1920 and
              0 <= t['y'] <= t['y'] + t['height'] <= 1080,
              '文字页内 %d %s' % (t['page'], t['text']))
        check(t['text'] in texts[t['page']], '文字实际写入 %d %s' % (t['page'], t['text']))
    facts = load('源码与数据核验.json')
    for path, digest in facts['source_hashes'].items():
        check(sha(path) == digest, '原件未改 ' + path)
    for item in facts['copies']:
        check(sha(HERE/item['copy']) == item['sha256'] == sha(item['source']),
              '证据副本 ' + item['copy'])
    check(len(facts['copies']) == 10, '十份实际原证据副本')
    prefix = Path(facts['experiment'])
    result = json.loads(Path(str(prefix)+'_result.json').read_text())
    check(result['termination'] == 'returned_home_and_saved', '真实返航保存')
    check(result['mission']['finish_reason'] == 'wall_budget_partial', '部分预算原因保留')
    check(not result['mission']['coverage_complete'] and not result['mission']['exploration_complete'],
          '没有冒充完整探索')
    check('93 / 97' in texts[7] and '284,418' in texts[7] and '部分' in texts[7], '07实际数值与结论')
    for suffix, missing, pct in [('_coverage.json', 10, '90.92%'),
                                  ('_freedom_coverage.json', 3, '99.01%')]:
        d = json.loads(Path(str(prefix)+suffix).read_text())
        check(len(d['racks']) == 52, suffix+'52架')
        check(sum(r['coverage'] < .9 for r in d['racks']) == missing, suffix+'缺口架数')
        check(abs(d['supported_samples']/d['reference_samples'] - d['shelf_side_coverage']) < 1e-12,
              suffix+'比例可复算')
        check(not d['shelf_side_acceptance'] and not d['full_3d_map_complete'], suffix+'未通过验收')
        check(pct in texts[8], suffix+'图中支持率')
    guide = (HERE/'讲解与实验出处.md').read_text()
    for target in re.findall(r'\]\(([^)]+)\)', guide):
        if not target.startswith(('http:', 'https:', '#')):
            check((Path(target) if target.startswith('/') else HERE/target).exists(), '讲解链接 ' + target)
    for name in ['draw.py', 'build_images.py', 'verify_images.py']:
        ast.parse((HERE/name).read_text())
        check(True, '脚本语法 ' + name)
    check(not list(HERE.rglob('*.pptx')), '未生成PPT')
    check((HERE/'00_八页总预览.png').exists(), '八页总预览')
    report = {'status':'PASS', 'checks':len(CHECKS), 'png_pages':8, 'svg_pages':8,
              'checks_detail':CHECKS, 'scope':'图片、文字、源码快照与历史证据；不是新飞行、资源或动态标注实验',
              'visual_review':'八页均人工查看大图；架构页调整文字插图距离后复看，08增加图例后复看'}
    (HERE/'独立核验.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'checks_detail'}, ensure_ascii=False))

if __name__ == '__main__':
    main()
