#!/usr/bin/env python3
"""独立文件、实际PDF文字和页内几何验证，生成全部页面预览。"""
import hashlib
import json
import math
import posixpath
import re
import unicodedata
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

import pymupdf as fitz
from PIL import Image, ImageDraw, ImageFont
from pptx import Presentation
from pptx.enum.shapes import MSO_SHAPE_TYPE

from build_deck import FONT_PATH, HERE, NAME, ROOT


def compact(text):
    return re.sub(r'\s+', '', unicodedata.normalize('NFKC', text)).replace('\u00ad', '').replace('\u200b', '')


def verify():
    pptx, pdf = HERE / (NAME + '.pptx'), HERE / (NAME + '.pdf')
    data = json.loads((HERE / '内容与来源.json').read_text(encoding='utf-8'))
    checks, errors = [], []

    def check(label, condition, detail=''):
        checks.append(dict(label=label, passed=bool(condition), detail=detail))
        if not condition:
            errors.append(label + ': ' + detail)

    with zipfile.ZipFile(pptx) as archive:
        names = set(archive.namelist())
        check('PPTX ZIP完整性', archive.testzip() is None)
        for name in names:
            if name.endswith(('.xml', '.rels')):
                ET.fromstring(archive.read(name))
            if name.endswith('.rels'):
                base = '' if name == '_rels/.rels' else name.split('/_rels/')[0]
                for rel in ET.fromstring(archive.read(name)):
                    if rel.attrib.get('TargetMode') == 'External':
                        continue
                    target = posixpath.normpath(posixpath.join(base, rel.attrib['Target'])).lstrip('/')
                    check('关系目标存在:' + target, target in names)
        slides = [n for n in names if re.fullmatch(r'ppt/slides/slide\d+\.xml', n)]
        notes = [n for n in names if re.fullmatch(r'ppt/notesSlides/notesSlide\d+\.xml', n)]
        check('PPTX页数', len(slides) == len(data), str(len(slides)))
        check('每页都有讲稿', len(notes) == len(data), str(len(notes)))
        check('没有用整页图片代替编辑对象', not any(n.startswith('ppt/media/') for n in names))
    prs = Presentation(pptx)
    native_shapes = 0
    for i, slide in enumerate(prs.slides, 1):
        native_shapes += len(slide.shapes)
        text = compact('\n'.join(s.text for s in slide.shapes if s.has_text_frame))
        expected = data[i - 1]
        check(f'第{i}页标题', compact(expected['title']) in text)
        if expected.get('kind') != 'diagram':
            for j, paragraph in enumerate(expected['left'] + expected['right'], 1):
                check(f'第{i}页原生正文{j}', compact(paragraph) in text)
        check(f'第{i}页补充讲稿', compact(expected['notes']) in compact(slide.notes_slide.notes_text_frame.text))
        for j, shape in enumerate(slide.shapes):
            if shape.shape_type == MSO_SHAPE_TYPE.LINE:
                continue
            check(f'第{i}页对象{j}边界', shape.left >= 0 and shape.top >= 0
                  and shape.left + shape.width <= prs.slide_width + 2
                  and shape.top + shape.height <= prs.slide_height + 2)
    doc = fitz.open(pdf)
    check('实际导出PDF页数', len(doc) == len(data), str(len(doc)))
    previews = HERE / '验证预览'
    previews.mkdir(exist_ok=True)
    page_results = []
    for i, page in enumerate(doc, 1):
        actual = page.get_text()
        norm = compact(actual)
        check(f'第{i}页PDF无替代字符', '\ufffd' not in actual)
        check(f'第{i}页PDF标题完整', compact(data[i - 1]['title']) in norm)
        if data[i - 1].get('kind') != 'diagram':
            for j, paragraph in enumerate(data[i - 1]['left'] + data[i - 1]['right'], 1):
                check(f'第{i}页PDF正文完整{j}', compact(paragraph) in norm, paragraph[:30])
        bottom = 0.0
        for block in page.get_text('dict')['blocks']:
            for ln in block.get('lines', []):
                for span in ln['spans']:
                    if abs(span['size'] - 19.5) < .2:
                        bottom = max(bottom, span['bbox'][3])
        check(f'第{i}页正文未碰流程图', bottom <= 6.14 * 72, f'bottom={bottom:.2f}pt')
        pixmap = page.get_pixmap(matrix=fitz.Matrix(1.1, 1.1), alpha=False)
        pixmap.save(str(previews / f'page_{i:02d}.png'))
        if data[i - 1].get('kind') == 'diagram':
            pixmap.save(str(HERE / '系统总关系图_简洁.png'))
        page_results.append(dict(page=i, title=data[i - 1]['title'], text_chars=len(norm),
                                 body_bottom_pt=bottom, image=f'page_{i:02d}.png'))
    font = ImageFont.truetype(FONT_PATH, 19)
    for start in range(0, len(doc), 6):
        sheet = Image.new('RGB', (1280, 1150), '#EBEEF1')
        drawer = ImageDraw.Draw(sheet)
        for offset in range(min(6, len(doc) - start)):
            index = start + offset + 1
            x, y = (offset % 2) * 640 + 10, (offset // 2) * 380 + 10
            thumbnail = Image.open(previews / f'page_{index:02d}.png')
            thumbnail.thumbnail((620, 349))
            sheet.paste(thumbnail, (x, y))
            drawer.text((x + 6, y + 353), f'{index:02d}  {data[index - 1]["title"][:26]}', font=font, fill='#33414D')
        sheet.save(previews / f'联系表_{start+1:02d}_{min(start+6,len(doc)):02d}.jpg', quality=93)
    receipts = json.loads((HERE / '来源文件哈希.json').read_text(encoding='utf-8'))
    for relative, digest in receipts.items():
        path = ROOT / relative
        check('当前引用源码未变化:' + relative,
              path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest)
    for path in (HERE / '简图').glob('*.svg'):
        ET.parse(path)
    ET.parse(HERE / '系统总关系图_简洁.svg')
    report = dict(passed=not errors, pages=len(doc), checks=len(checks), errors=errors,
                  native_shapes=native_shapes, image_only_slides=0,
                  presentation_sha256=hashlib.sha256(pptx.read_bytes()).hexdigest(),
                  pdf_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(),
                  source_files=len(receipts), page_results=page_results, details=checks,
                  validation_scope='结构、备注、源码路径、实际PDF文字与页内布局；非ROS/飞行验证')
    (HERE / '验证报告.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('passed','pages','checks','native_shapes','source_files','errors')}, ensure_ascii=False))
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    verify()
