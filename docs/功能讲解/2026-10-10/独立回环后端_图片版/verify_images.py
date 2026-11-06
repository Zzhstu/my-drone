#!/usr/bin/env python3
"""独立核验图片、中文字边界、源码/历史原件与压缩文件。"""
import ast
import base64
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile
from PIL import Image, ImageChops, ImageStat

HERE=Path(__file__).resolve().parent
checks=[]

def check(ok,label):
    checks.append(dict(check=label,passed=bool(ok)))
    if not ok:raise AssertionError(label)

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    metadata=json.loads((HERE/'版面核验信息.json').read_text())
    facts=json.loads((HERE/'源码与数据核验.json').read_text())
    pngs=sorted((HERE/'PNG').glob('*.png'));svgs=sorted((HERE/'SVG').glob('*.svg'))
    check(len(pngs)==len(svgs)==4,'4 PNG and 4 SVG')
    check(not list(HERE.rglob('*.pptx')),'no PPTX generated')
    for p in pngs:
        with Image.open(p) as im:
            im.load();check(im.size==(3840,2160),p.name+' dimensions')
            check(max(ImageStat.Stat(im.convert('RGB')).var)>300,p.name+' nonempty rendering')
    namespace={'s':'http://www.w3.org/2000/svg'}
    for p in svgs:
        root=ET.parse(p).getroot();check(root.get('viewBox')=='0 0 1920 1080',p.name+' vector canvas')
        nodes=root.findall('.//s:text',namespace)
        check(len(nodes)>=18,p.name+' editable text')
        for node in nodes:
            check(float(node.get('font-size'))>=16,p.name+' readable text '+str(node.text))
        for node in root.findall('.//s:rect',namespace):
            x,y,w,h=[float(node.get(k)) for k in ('x','y','width','height')]
            check(x>=0 and y>=0 and x+w<=1920 and y+h<=1080,p.name+' rectangle in canvas')
        page=int(p.name[:2]);expected=[t['text'] for t in metadata['texts'] if t['page']==page]
        check([n.text or '' for n in nodes]==expected,p.name+' exact text inventory')
    for t in metadata['texts']:
        check(t['x']>=0 and t['y']>=0 and t['x']+t['width']<=1920 and t['y']+t['height']<=1080,
              str(t['page'])+' font bounds '+t['text'])
    for page in metadata['pages']:
        check(page['illustrations']>=6 if page['number']==2 else page['illustrations']>=4 if page['number'] in (1,3) else page['illustrations']==0,
              str(page['number'])+' diagram count appropriate to page')
    for p,digest in facts['source_hashes'].items():check(sha(Path(p))==digest,'protected original '+p)
    for key in ('figure_original','report_original'):
        src=Path(facts[key]);dst=HERE/'证据原件'/src.name
        check(src.read_bytes()==dst.read_bytes(),'unchanged evidence copy '+src.name)
    image_node=ET.parse(svgs[3]).getroot().find('s:image',namespace)
    uri=image_node.get('{http://www.w3.org/1999/xlink}href')
    check(base64.b64decode(uri.split(',',1)[1])==Path(facts['figure_original']).read_bytes(),'embedded original plot bytes')
    with Image.open(pngs[3]) as im:
        # The embedded chart region must not have rendered as a blank box.
        chart=im.crop((180,480,1900,1790)).convert('RGB')
        check(sum(1 for r,g,b in chart.resize((400,300)).getdata() if g>r+25 and g>b+15)>500,
              'actual raster chart contains green trajectory')
    for p in (HERE/'build_images.py',HERE/'verify_images.py'):ast.parse(p.read_text());check(True,p.name+' syntax')
    archive=HERE.parent/'独立回环后端_图片与证据合集.zip'
    with ZipFile(archive) as z:
        check(z.testzip() is None,'ZIP CRC')
        for p in HERE.rglob('*'):
            if not p.is_file() or '__pycache__' in p.parts or p.name=='独立核验.json':continue
            check(z.read(str(Path(HERE.name)/p.relative_to(HERE)))==p.read_bytes(),'ZIP exact '+str(p.relative_to(HERE)))
    result=dict(scope='material-only checks; no algorithm experiment',count=len(checks),passed=all(c['passed'] for c in checks),checks=checks)
    (HERE/'独立核验.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print('PASS: %d independent checks'%len(checks))

if __name__=='__main__':main()
