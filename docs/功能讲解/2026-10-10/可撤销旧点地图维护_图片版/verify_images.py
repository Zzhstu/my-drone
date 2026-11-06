#!/usr/bin/env python3
"""独立检查图片、矢量、内容来源、原文件保护及打包完整性，不启动ROS。"""
import ast
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from PIL import Image

HERE = Path(__file__).resolve().parent
PACKAGE = HERE / '可撤销旧点地图维护_16张图片与讲解.zip'
checks = []


def check(value, note):
    if not value:
        raise AssertionError(note)
    checks.append(note)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    meta = json.loads((HERE / '内容与来源.json').read_text())
    gen = json.loads((HERE / '生成核验.json').read_text())
    pages = meta['pages']
    check(len(pages) == 16, '恰好16张内容页')
    for kind, suffix in [('PNG', '.png'), ('SVG', '.svg')]:
        files = sorted((HERE / kind).glob('*' + suffix))
        check(len(files) == 16, kind + '恰好16份')
        check([int(p.name[:2]) for p in files] == list(range(1, 17)), kind + '编号01—16无缺失')
    check(not list(HERE.rglob('*.pptx')) and not list(HERE.rglob('*.ppt')), '未生成PPT/PPTX')
    for path, expected in meta['sources_sha256'].items():
        check(Path(path).is_file(), '来源存在：' + path)
        check(sha(Path(path)) == expected, '来源未改变：' + path)
    for page in pages:
        n = page['number']
        png, svg = Path(page['png']), Path(page['svg'])
        check(png.parent == HERE / 'PNG' and svg.parent == HERE / 'SVG', f'{n:02d}输出位于指定合集')
        with Image.open(png) as im:
            check(im.size == (3840, 2160) and im.format == 'PNG', f'{n:02d}高清PNG规格')
            im.verify()
        with Image.open(png) as im:
            im.load()
            check(im.convert('RGB').getextrema() != ((255, 255),) * 3, f'{n:02d}不是空白图片')
        root = ET.parse(svg).getroot()
        check(root.attrib['viewBox'] == '0 0 1600 900', f'{n:02d}矢量画布规格')
        elements = list(root.iter())
        texts = [e.text or '' for e in elements if e.tag.endswith('}text')]
        check(texts == page['text'], f'{n:02d}渲染文字与内容记录逐行一致')
        check(texts[0] == page['title'], f'{n:02d}标题正确')
        check(not any(e.tag.endswith('}image') for e in elements), f'{n:02d}全部图形可编辑而非嵌入截图')
        check(not any('href' in k for e in elements for k in e.attrib), f'{n:02d}无外部字体/图片资源依赖链接')
        triangles = 0
        for e in elements:
            tag = e.tag.rsplit('}', 1)[-1]
            if tag in ('polygon', 'polyline'):
                pts = [tuple(map(float, pair.split(','))) for pair in e.attrib['points'].split()]
                check(all(0 <= x <= 1600 and 0 <= y <= 900 for x, y in pts), f'{n:02d}{tag}几何在画布内')
                if tag == 'polygon' and len(pts) == 3:
                    triangles += 1
            if tag == 'rect':
                x, y, w, h = [float(e.attrib[k]) for k in ('x', 'y', 'width', 'height')]
                check(x >= 0 and y >= 0 and x + w <= 1600 and y + h <= 900, f'{n:02d}矩形在画布内')
        check(triangles == page['arrows'], f'{n:02d}箭头均有显式矢量箭头头部')
    check(all(b['in_page'] for b in meta['text_bounds']), '全部文字行在画布内')
    check(gen['text_lines'] == len(meta['text_bounds']), '文字行检查统计一致')
    check(gen['rule_checks'] == 29, '生成阶段29项源码/配置规则检查通过')
    check(gen['unchanged_sources'] == 12, '参考PPT和11份相关源码/配置未改变')
    check(gen['arrows'] == sum(p['arrows'] for p in pages), '全部页面箭头统计一致')
    notes = (HERE / '详细讲解与源码对应.md').read_text()
    for target in re.findall(r'\]\((/[^)]+)\)', notes):
        check(Path(target).exists(), '讲解文档链接存在：' + target)
    for word in ['27邻域', '0.08秒', '0.06米', 'latest_free_cells', '非真实扫描日志', '未ROS编译']:
        check(word in notes, '关键规则与验证边界在讲解中：' + word)
    for script in HERE.glob('*.py'):
        ast.parse(script.read_text(), filename=str(script))
        check(True, '脚本语法通过：' + script.name)
    for preview in ['00_全部图片预览.png', '预览_01至08.png', '预览_09至16.png']:
        with Image.open(HERE / preview) as im:
            im.verify()
        check(True, '预览可解码：' + preview)
    selected = sorted(p for p in HERE.rglob('*') if p.is_file()
                      and p.suffix in ('.png', '.svg', '.md', '.json', '.py')
                      and p.name != '独立核验.json')
    check(len(selected) >= 40, '真实图片、SVG、讲解和工具全部纳入合集')
    with ZipFile(PACKAGE, 'w', ZIP_DEFLATED) as z:
        for p in selected:
            z.write(p, str(Path(HERE.name) / p.relative_to(HERE)))
    with ZipFile(PACKAGE) as z:
        check(z.testzip() is None, '压缩包CRC校验通过')
        check(len(z.namelist()) == len(selected), '打包文件数与实际素材一致')
        for p in selected:
            name = str(Path(HERE.name) / p.relative_to(HERE))
            check(hashlib.sha256(z.read(name)).hexdigest() == sha(p), '打包字节一致：' + str(p.relative_to(HERE)))
    report = dict(status='passed', checks=len(checks), check_items=checks, pages=16,
                  package=str(PACKAGE), packed_files=len(selected), package_bytes=PACKAGE.stat().st_size,
                  scope='图文素材检查与打包验证；未运行ROS、仿真或真机',
                  source_unchanged=True, ppt_created=False)
    (HERE / '独立核验.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != 'check_items'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
