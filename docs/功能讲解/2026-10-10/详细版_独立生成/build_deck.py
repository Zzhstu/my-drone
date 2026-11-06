#!/usr/bin/env python3
"""独立制作原生PPTX、说明文档和简图；不调用任何技能工具。"""
import argparse
import hashlib
import html
import json
import re
import subprocess
import tempfile
from pathlib import Path

from PIL import ImageFont
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.xmlchemy import OxmlElement
from pptx.util import Inches, Pt

from content import SLIDES

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT / 'AstraDrone_ros1_ws/src/SLAM'
PREFIXES = {'bridge': BASE / 'fastlio_bridge', 'fast': BASE / 'FAST_LIO',
            'root': ROOT,
            'offboard': ROOT / 'AstraDrone_ros1_ws/src/MissionControl/astra_uavoffbard_frame/offboard'}
NAME = '无人机功能详解_简图详细版_2026-10-10'
FONT = 'Noto Sans CJK SC'
INK, MUTED, BLUE, RULE = '202A35', '647180', '275B88', 'CAD1D8'
SIZES = {'title': 28, 'heading': 20, 'body': 19.5, 'flow': 15, 'footer': 9}
FONT_PATH = subprocess.check_output(['fc-match', '-f', '%{file}', FONT], text=True).strip()
RECEIPT = []


def color(s):
    return RGBColor.from_string(s)


def font_runs(paragraph, size, bold=False, shade=INK):
    paragraph.font.name = FONT
    paragraph.font.size = Pt(size)
    paragraph.font.bold = bold
    paragraph.font.color.rgb = color(shade)
    for run in paragraph.runs:
        run.font.name = FONT
        run.font.size = Pt(size)
        run.font.bold = bold
        run.font.color.rgb = color(shade)
        rpr = run._r.get_or_add_rPr()
        for tag in ('a:ea', 'a:cs'):
            element = rpr.find('{http://schemas.openxmlformats.org/drawingml/2006/main}' + tag[2:])
            if element is None:
                element = OxmlElement(tag)
                rpr.append(element)
            element.set('typeface', FONT)


def measure_wrap(text, width, size):
    font = ImageFont.truetype(FONT_PATH, round(size * 2))
    max_width = width * 144 - 8
    lines, line = [], ''
    for ch in text:
        if ch == '\n':
            lines.append(line)
            line = ''
        elif line and font.getlength(line + ch) > max_width:
            lines.append(line)
            line = ch
        else:
            line += ch
    lines.append(line)
    return lines


def box(slide, text, x, y, w, h, size=19.5, bold=False, shade=INK,
        align=PP_ALIGN.LEFT, leading=None, tag='text'):
    shape = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    shape.name = tag
    tf = shape.text_frame
    tf.clear()
    tf.margin_left = tf.margin_right = Inches(0.015)
    tf.margin_top = tf.margin_bottom = Inches(0.015)
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.TOP
    for i, item in enumerate(text if isinstance(text, list) else [text]):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.text = item
        p.alignment = align
        p.space_before = Pt(0)
        p.space_after = Pt(10 if isinstance(text, list) else 0)
        p.line_spacing = Pt(leading or size * 1.25)
        font_runs(p, size, bold, shade)
    return shape


def line(slide, x1, y1, x2, y2, shade=RULE, arrow=False, width=1):
    shape = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,
                                       Inches(x1), Inches(y1), Inches(x2), Inches(y2))
    shape.line.color.rgb = color(shade)
    shape.line.width = Pt(width)
    theme_style = shape._element.find('{http://schemas.openxmlformats.org/presentationml/2006/main}style')
    if theme_style is not None:
        shape._element.remove(theme_style)
    if arrow:
        ln = shape._element.spPr.find('{http://schemas.openxmlformats.org/drawingml/2006/main}ln')
        end = OxmlElement('a:tailEnd')
        end.set('type', 'triangle')
        end.set('w', 'sm')
        end.set('len', 'sm')
        ln.append(end)
    return shape


def node(slide, label, x, y, w, h, size=15):
    shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y), Inches(w), Inches(h))
    shape.fill.background()
    shape.line.color.rgb = color('84929F')
    shape.line.width = Pt(1)
    theme_style = shape._element.find('{http://schemas.openxmlformats.org/presentationml/2006/main}style')
    if theme_style is not None:
        shape._element.remove(theme_style)
    shape.name = 'editable_flow_node'
    tf = shape.text_frame
    tf.clear()
    tf.margin_left = tf.margin_right = Inches(.06)
    tf.margin_top = tf.margin_bottom = Inches(.025)
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    p = tf.paragraphs[0]
    p.text = label
    p.alignment = PP_ALIGN.CENTER
    p.line_spacing = Pt(size * 1.2)
    p.space_after = Pt(0)
    font_runs(p, size)


def simple_flow(slide, labels):
    x, y, total, gap = .64, 6.20, 12.05, .32
    w = (total - gap * (len(labels) - 1)) / len(labels)
    for i, label in enumerate(labels):
        xx = x + i * (w + gap)
        node(slide, label, xx, y, w, .58)
        if i:
            line(slide, xx - gap + .02, y + .29, xx - .02, y + .29, shade=MUTED, arrow=True)


def full_system(slide):
    places = {
        'a': ('定位与当前扫描', 5.16, 1.48, 3.02, .68),
        'b': ('维护地图与自由证据', 1.25, 2.90, 3.16, .68),
        'c': ('最新扫描与运动目标', 8.92, 2.90, 3.16, .68),
        'd': ('指定目标规划 或 自主探索', 1.25, 4.13, 3.16, .68),
        'e': ('静态与动态风险融合', 8.92, 4.13, 3.16, .68),
        'f': ('安全控制：核验后发速度', 5.16, 5.04, 3.02, .68),
        'g': ('飞控执行，获得新扫描', 5.16, 6.10, 3.02, .68),
    }
    for label, x, y, w, h in places.values():
        node(slide, label, x, y, w, h, 18)
    for start, end in [('a', 'b'), ('a', 'c'), ('b', 'd'), ('c', 'e'), ('d', 'f'), ('e', 'f'), ('f', 'g')]:
        _, ax, ay, aw, ah = places[start]
        _, bx, by, bw, _ = places[end]
        line(slide, ax + aw / 2, ay + ah, bx + bw / 2, by, shade=MUTED, arrow=True, width=1.2)
    box(slide, '长期路线', 1.25, 2.40, 3.16, .32, 17, shade=BLUE)
    box(slide, '即时安全', 8.92, 2.40, 3.16, .32, 17, shade=BLUE, align=PP_ALIGN.RIGHT)
    return places


def path_for(source):
    prefix, rel = source.split(':', 1)
    path = PREFIXES[prefix] / rel
    if not path.is_file():
        raise FileNotFoundError(f'引用源文件不存在: {path}')
    return path


def write_flow_svg(labels, path):
    n = len(labels)
    w, gap = (1200 - 36 * (n - 1)) / n, 36
    elements = []
    for i, label in enumerate(labels):
        x = 40 + i * (w + gap)
        elements.append(f'<rect x="{x:.2f}" y="30" width="{w:.2f}" height="70" fill="white" stroke="#84929F"/>')
        if i:
            elements.append(f'<path d="M{x-gap+3:.2f} 65 H{x-3:.2f}" fill="none" stroke="#647180" marker-end="url(#arr)"/>')
        parts = measure_wrap(label, w / 96, 15)
        for j, part in enumerate(parts):
            yy = 65 - (len(parts) - 1) * 10 + j * 20 + 5
            elements.append(f'<text x="{x+w/2:.2f}" y="{yy:.2f}" text-anchor="middle" font-family="{FONT},sans-serif" font-size="18" fill="#202A35">{html.escape(part)}</text>')
    path.write_text('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 130"><defs>'
                    '<marker id="arr" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><path d="M0 0 L7 3.5 L0 7" fill="#647180"/></marker></defs>'
                    + ''.join(elements) + '</svg>', encoding='utf-8')


def write_system_svg(places, path):
    elements = ['<rect width="1280" height="720" fill="white"/>']
    scale = 96
    for label, x, y, w, h in places.values():
        elements.append(f'<rect x="{x*scale:.1f}" y="{y*scale:.1f}" width="{w*scale:.1f}" height="{h*scale:.1f}" fill="white" stroke="#84929F"/>')
        parts = measure_wrap(label, w, 18)
        for j, part in enumerate(parts):
            yy = (y + h / 2) * scale - (len(parts) - 1) * 13 + j * 26 + 7
            elements.append(f'<text x="{(x+w/2)*scale:.1f}" y="{yy:.1f}" text-anchor="middle" font-size="23" font-family="{FONT},sans-serif" fill="#202A35">{html.escape(part)}</text>')
    for start, end in [('a', 'b'), ('a', 'c'), ('b', 'd'), ('c', 'e'), ('d', 'f'), ('e', 'f'), ('f', 'g')]:
        _, ax, ay, aw, ah = places[start]
        _, bx, by, bw, _ = places[end]
        elements.append(f'<path d="M{(ax+aw/2)*scale:.1f} {(ay+ah)*scale:.1f} L{(bx+bw/2)*scale:.1f} {by*scale:.1f}" stroke="#647180" fill="none" marker-end="url(#arr)"/>')
    path.write_text('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 720"><defs><marker id="arr" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><path d="M0 0 L7 3.5 L0 7" fill="#647180"/></marker></defs>' + ''.join(elements) + '</svg>', encoding='utf-8')


def build():
    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(13.333333), Inches(7.5)
    prs.core_properties.title = '无人机建图、避障与自主探索：功能详细讲解'
    prs.core_properties.subject = '独立生成；简洁图示；实际源码规则与边界'
    prs.core_properties.author = 'AstraDroneOpen 项目讲解'
    diagram_dir = HERE / '简图'
    diagram_dir.mkdir(exist_ok=True)
    data = []
    for item in SLIDES:
        data.append(item)
        if item.get('kind') == 'system':
            data.append(dict(item, title='系统总关系图（可直接编辑）', kind='diagram'))
    source_receipts = {}
    manual = ['# 无人机功能详解：简图详细版', '',
              '本版独立制作，不使用ppt-master。白底、深色文字、少量蓝色标题；文字和流程节点均为原生可编辑对象。', '',
              '依据2026-10-10当前源码、配置、启动接线与既有项目记录。规则例子不是新实测；旧PPT及飞行程序保持不变。', '']
    visible_characters = 0
    for i, item in enumerate(data, 1):
        slide = prs.slides.add_slide(prs.slide_layouts[6])
        slide.background.fill.solid()
        slide.background.fill.fore_color.rgb = color('FFFFFF')
        box(slide, item['chapter'], .65, .28, 10, .28, 10.5, shade=BLUE, tag='chapter')
        box(slide, item['title'], .65, .71, 12.05, .67, SIZES['title'], True, tag='title')
        line(slide, .65, 1.42, 12.69, 1.42)
        if item.get('kind') == 'diagram':
            positions = full_system(slide)
            write_system_svg(positions, HERE / '系统总关系图_简洁.svg')
        else:
            for x, heading, paragraphs in [(.65, item['left_title'], item['left']),
                                           (6.90, item['right_title'], item['right'])]:
                box(slide, heading, x, 1.65, 5.78, .42, 20, True, BLUE, tag='section_heading')
                # 字号固定；内容多时必须拆页/修改内容，而不是自动缩小。
                lines = [len(measure_wrap(t, 5.78, 19.5)) for t in paragraphs]
                estimate_pt = sum(lines) * 26 + (len(paragraphs) - 1) * 10 + 5
                if estimate_pt > 4.00 * 72:
                    raise ValueError(f'第{i}页 {heading} 文字超限: {estimate_pt:.1f}pt，需拆页或编辑正文')
                box(slide, paragraphs, x, 2.22, 5.78, 3.86, 19.5, leading=26, tag='body')
                RECEIPT.append(dict(page=i, heading=heading, lines=lines, estimate_pt=estimate_pt))
            line(slide, 6.655, 1.75, 6.655, 5.94, 'E3E7EB', width=.65)
            simple_flow(slide, item['flow'])
            write_flow_svg(item['flow'], diagram_dir / f'{i:02d}_流程.svg')
        line(slide, .65, 7.03, 12.69, 7.03)
        files = [path_for(s) for s in item['sources']]
        footer = '源码与完整规则见讲稿  |  ' + '、'.join(p.name for p in files[:2])
        box(slide, footer, .65, 7.14, 11.4, .19, 9, shade=MUTED, tag='footer')
        box(slide, f'{i:02d} / {len(data):02d}', 12.02, 7.13, .67, .22, 9, shade=MUTED, align=PP_ALIGN.RIGHT)
        for path in files:
            source_receipts[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
        notes = ('本页用途：' + item['title'] + '\n\n' + item['left_title'] + '\n' + '\n\n'.join(item['left'])
                 + '\n\n' + item['right_title'] + '\n' + '\n\n'.join(item['right'])
                 + '\n\n进一步解释与提醒：\n' + item['notes']
                 + '\n\n简单流程：\n' + ' → '.join(item['flow'])
                 + '\n\n实际文件位置：\n' + '\n'.join(str(p) for p in files)
                 + '\n\n验证边界：本页是源码规则讲解；本轮未编译、启动ROS或进行新的飞行效果实验。')
        slide.notes_slide.notes_text_frame.text = notes
        manual.extend([f'## 第{i:02d}页｜{item["title"]}', '', f'### {item["left_title"]}', '',
                       '\n\n'.join(item['left']), '', f'### {item["right_title"]}', '',
                       '\n\n'.join(item['right']), '', '### 进一步解释与提醒', '', item['notes'], '',
                       '### 简单流程', '', ' → '.join(item['flow']), '', '### 源码与参数位置', ''])
        manual.extend(f'- [{p.name}]({p})' for p in files)
        manual.extend(['', '---', ''])
        visible_characters += sum(len(t) for t in item['left'] + item['right'])
    prs.save(HERE / (NAME + '.pptx'))
    (HERE / '逐页详细讲解与源码位置.md').write_text('\n'.join(manual), encoding='utf-8')
    (HERE / '内容与来源.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    (HERE / '来源文件哈希.json').write_text(json.dumps(source_receipts, ensure_ascii=False, indent=2), encoding='utf-8')
    (HERE / '排版预检.json').write_text(json.dumps(dict(pages=len(data), font=FONT,
        body_font_points=19.5, visible_body_characters=visible_characters,
        sources=len(source_receipts), estimates=RECEIPT), ensure_ascii=False, indent=2), encoding='utf-8')
    return len(data)


def convert_pdf():
    with tempfile.TemporaryDirectory(prefix='astradrone_ppt_lo_') as directory:
        profile = Path(directory).as_uri()
        result = subprocess.run(['libreoffice', '-env:UserInstallation=' + profile,
                                 '--headless', '--convert-to', 'pdf', '--outdir', str(HERE),
                                 str(HERE / (NAME + '.pptx'))], capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise RuntimeError(result.stderr or result.stdout)
        print(result.stdout.strip())
    if not (HERE / (NAME + '.pdf')).is_file():
        raise RuntimeError('LibreOffice没有生成PDF')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--pdf', action='store_true')
    args = parser.parse_args()
    count = build()
    print(json.dumps(dict(slides=count, pptx=str(HERE / (NAME + '.pptx'))), ensure_ascii=False))
    if args.pdf:
        convert_pdf()
