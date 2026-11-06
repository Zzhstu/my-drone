#!/usr/bin/env python3
"""独立检查图文文件、证据副本、原图内嵌、原件保护和实际ZIP。"""
import ast
import base64
import hashlib
import json
import re
import sys
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET
from PIL import Image, ImageStat

HERE=Path(__file__).resolve().parent
ARCHIVE=HERE.parent/'黑箱探索补扫返航_图片与证据合集.zip'
checks=[]
def check(ok,label):
    checks.append(dict(check=label,passed=bool(ok)))
    if not ok:raise AssertionError(label)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def archive_checks(include_report=False):
    with ZipFile(ARCHIVE) as z:
        check(z.testzip() is None,'ZIP CRC')
        files=[p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and
               (include_report or p.name!='独立核验.json')]
        for p in files:check(z.read(str(Path(HERE.name)/p.relative_to(HERE)))==p.read_bytes(),'archive exact '+str(p.relative_to(HERE)))
        if include_report:check(len(files)==len(z.namelist()),'complete final archive inventory')

def main():
    if '--archive-only' in sys.argv:
        archive_checks(True);print('PASS final archive:',len(checks),'checks');return
    facts=json.loads((HERE/'源码与数据核验.json').read_text())
    meta=json.loads((HERE/'版面核验信息.json').read_text())
    pngs=sorted((HERE/'PNG').glob('*.png'));svgs=sorted((HERE/'SVG').glob('*.svg'))
    check(len(pngs)==len(svgs)==7,'seven PNG + seven SVG')
    check(not list(HERE.rglob('*.pptx')),'no PPTX')
    ns={'s':'http://www.w3.org/2000/svg'}
    for path in pngs:
        with Image.open(path) as im:
            im.load();check(im.size==(3840,2160),path.name+' dimensions')
            check(max(ImageStat.Stat(im.convert('RGB')).var)>250,path.name+' nonempty content')
    for path in svgs:
        root=ET.parse(path).getroot();n=int(path.name[:2])
        check(root.get('viewBox')=='0 0 1920 1080',path.name+' canvas')
        nodes=root.findall('.//s:text',ns)
        check(len(nodes)>=20,path.name+' editable text')
        check([node.text or '' for node in nodes]==[t['text'] for t in meta['texts'] if t['page']==n],path.name+' text inventory')
        for node in nodes:check(float(node.get('font-size'))>=16,path.name+' font size '+str(node.text))
        for node in root.findall('.//s:rect',ns):
            x,y,w,h=[float(node.get(k)) for k in ('x','y','width','height')]
            check(min(x,y,w,h)>=0 and x+w<=1920 and y+h<=1080,path.name+' rectangle bounds')
        if n==1:check(len([r for r in root.findall('.//s:rect',ns) if r.get('stroke-dasharray')])>=5,'partitioned architecture')
        if n==2:
            titles=[node.text or '' for node in nodes]
            check(all(any(t.startswith(str(i)) for t in titles) for i in range(1,8)),'seven-step snake content')
            check(any(node.get('stroke-dasharray') for node in root.findall('.//s:polyline',ns)),'explicit feedback loop')
        if n==6:
            imnode=root.find('s:image',ns);uri=imnode.get('{http://www.w3.org/1999/xlink}href')
            check(base64.b64decode(uri.split(',',1)[1])==(HERE/'证据原件/C04_true_map_comparison.png').read_bytes(),'embedded genuine original map')
        if n==7:
            check(len([r for r in root.findall('.//s:rect',ns) if r.get('width')=='18' and r.get('height')=='32'])==104,'52 genuine rack positions in each map')
    for t in meta['texts']:
        check(t['x']>=0 and t['y']>=0 and t['x']+t['width']<=1920 and t['y']+t['height']<=1080,str(t['page'])+' glyph bounds '+t['text'])
    # Body text should be inside a colored card, not under the chart/illustration.
    # Pixel-box dimensions + actual visual inspection complement this check.
    for page in meta['pages']:
        if page['number'] in (1,2,3,4,5):check(page['illustrations']>0,'illustrations on mechanism page '+str(page['number']))
        if page['number']==2:check(page['illustrations']==7,'one illustration per snake card')
    for src,digest in facts['source_hashes'].items():check(sha(Path(src))==digest,'protected source '+src)
    for copy in facts['copies']:
        check(sha(HERE/copy['copy'])==copy['sha256'],'evidence copy '+copy['copy'])
    readme=(HERE/'讲解与实验出处.md').read_text()
    for path in re.findall(r'\]\(([^)]+)\)',readme):
        if not path.startswith('http'):check((HERE/path).exists(),'documentation link '+path)
    for p in HERE.glob('*.py'):ast.parse(p.read_text());check(True,p.name+' syntax')
    # Genuine PCD plot must be visibly rendered (not just present in SVG).
    with Image.open(pngs[5]) as im:
        pixels=im.crop((140,410,3700,1506)).resize((640,200)).convert('RGB').getdata()
        check(sum(1 for r,g,b in pixels if g>r+15 and g>b+10)>600,'actual plotted map colored returns')
    archive_checks()
    report=dict(scope='image/source/evidence only; no new flight or completeness test',count=len(checks),passed=True,checks=checks)
    (HERE/'独立核验.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print('PASS:',len(checks),'independent checks')

if __name__=='__main__':main()
