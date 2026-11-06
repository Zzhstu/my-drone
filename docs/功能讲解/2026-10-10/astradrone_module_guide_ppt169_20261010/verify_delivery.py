#!/usr/bin/env python3
"""Read the exported deck, validate links, and render its actual PDF for review."""
import hashlib
import json
import posixpath
import re
import subprocess
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from PIL import Image, ImageDraw


def main():
    project = Path(__file__).resolve().parent
    pptx = sorted((project / 'exports').glob('astradrone_module_guide_20261010_*.pptx'))[-1]
    pdf = pptx.with_suffix('.pdf')
    out = project / 'validation/rendered'
    out.mkdir(parents=True, exist_ok=True)
    ns = {'p': 'http://schemas.openxmlformats.org/presentationml/2006/main',
          'a': 'http://schemas.openxmlformats.org/drawingml/2006/main',
          'r': 'http://schemas.openxmlformats.org/package/2006/relationships'}
    with zipfile.ZipFile(pptx) as z:
        assert z.testzip() is None, 'ZIP corruption'
        names = set(z.namelist())
        slides = sorted((n for n in names if re.fullmatch(r'ppt/slides/slide\d+\.xml', n)),
                        key=lambda n: int(re.search(r'(\d+)\.xml', n).group(1)))
        notes = sorted(n for n in names if re.fullmatch(r'ppt/notesSlides/notesSlide\d+\.xml', n))
        assert len(slides) == len(notes) == 22
        xml_count = rel_count = native_count = 0
        texts = []
        for name in names:
            if name.endswith(('.xml', '.rels')):
                root = ET.fromstring(z.read(name))
                xml_count += 1
                if name.endswith('.rels'):
                    base = posixpath.dirname(posixpath.dirname(name))
                    for rel in root:
                        if rel.get('TargetMode') == 'External':
                            continue
                        target = rel.get('Target', '')
                        resolved = (target.lstrip('/') if target.startswith('/') else
                                    posixpath.normpath(posixpath.join(base, target)))
                        assert resolved in names, (name, resolved)
                        rel_count += 1
        for index, name in enumerate(slides, 1):
            root = ET.fromstring(z.read(name))
            native_count += len(root.findall('.//p:sp', ns))
            assert not root.findall('.//p:pic', ns), 'Page should be native editable shapes'
            page_text = ''.join(t.text or '' for t in root.findall('.//a:t', ns))
            assert page_text.strip(), name
            texts.append(page_text)
            rel_name = f'ppt/slides/_rels/slide{index}.xml.rels'
            rel_root = ET.fromstring(z.read(rel_name))
            note_targets = [rel.get('Target') for rel in rel_root
                            if rel.get('Type', '').endswith('/notesSlide')]
            assert note_targets == [f'../notesSlides/notesSlide{index}.xml'], note_targets
            note_root = ET.fromstring(z.read(f'ppt/notesSlides/notesSlide{index}.xml'))
            note_text = ''.join(t.text or '' for t in note_root.findall('.//a:t', ns))
            assert len(note_text) > 100
        assert '长期路线与即时安全并行' in texts[20]
        assert '自身半径0.40米' in texts[8]
    info = subprocess.check_output(['pdfinfo', str(pdf)], text=True)
    assert re.search(r'^Pages:\s+22$', info, re.M)
    subprocess.run(['pdftoppm', '-png', '-scale-to', '1280', str(pdf), str(out / 'page')],
                   check=True, timeout=60)
    pages = sorted(out.glob('page-*.png'), key=lambda p: int(p.stem.split('-')[-1]))
    assert len(pages) == 22
    sheets = []
    for start in range(0, len(pages), 6):
        sheet = Image.new('RGB', (1328, 1176), '#D3DEE5')
        draw = ImageDraw.Draw(sheet)
        for k, page in enumerate(pages[start:start + 6]):
            im = Image.open(page).convert('RGB')
            im.thumbnail((640, 360), Image.Resampling.LANCZOS)
            x, y = 16 + (k % 2) * 656, 28 + (k // 2) * 388
            sheet.paste(im, (x, y))
            draw.text((x, y - 18), f'PAGE {start + k + 1:02d}', fill='#16334A')
        target = out / f'contact-{start // 6 + 1}.png'
        sheet.save(target)
        sheets.append(str(target.relative_to(project)))
    report = dict(slides=22, notes=22, native_editable_shapes=native_count,
                  parsed_xml_parts=xml_count, verified_internal_relationships=rel_count,
                  pdf_pages=22, rendered_pages=len(pages), contact_sheets=sheets,
                  pptx_bytes=pptx.stat().st_size, pdf_bytes=pdf.stat().st_size,
                  pptx_sha256=hashlib.sha256(pptx.read_bytes()).hexdigest(),
                  pdf_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(),
                  scope='Document integrity and actual exported PDF rendering only; no ROS or flight test')
    (project / 'validation/delivery_checks.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
