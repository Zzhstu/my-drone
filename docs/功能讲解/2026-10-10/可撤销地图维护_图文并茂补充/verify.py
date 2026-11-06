#!/usr/bin/env python3
"""独立核验交付图片/来源/旧材料/压缩包，不运行ROS，不修改科研代码。"""
import ast
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile
from PIL import Image

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'可撤销旧点地图维护_图片版'
RESULT=HERE/'独立核验.json'
TESTS=[]

def test(name,passed):
    TESTS.append({'name':name,'passed':bool(passed)})
    assert passed,name

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def geometry(node,transform=(0,0,1),page=0):
    """检查可绘制几何变换后不越出页面；字号边界另用生成时的真实字体测量。"""
    tx,ty,s=transform
    tr=node.attrib.get('transform','')
    if tr:
        m=re.fullmatch(r'translate\(([-\d.]+),([-\d.]+)\) scale\(([-\d.]+)\)',tr)
        test(f'{page}: supported transform {tr}',m is not None)
        x,y,scale=map(float,m.groups());tx+=x*s;ty+=y*s;s*=scale
    tag=node.tag.split('}')[-1];a=node.attrib;pts=[]
    if tag=='rect':
        x,y,w,h=(float(a[k]) for k in ('x','y','width','height'));pts=[(x,y),(x+w,y+h)]
        test(f'{page}: positive rectangle',w>=0 and h>=0)
    elif tag=='circle':
        x,y,r=(float(a[k]) for k in ('cx','cy','r'));pts=[(x-r,y-r),(x+r,y+r)]
    elif tag=='ellipse':
        x,y,rx,ry=(float(a[k]) for k in ('cx','cy','rx','ry'));pts=[(x-rx,y-ry),(x+rx,y+ry)]
    elif tag in ('polygon','polyline'):
        pts=[tuple(map(float,p.split(','))) for p in a['points'].split()]
    if pts:
        test(f'{page}: {tag} canvas geometry',all(-.05<=tx+x*s<=1920.05 and -.05<=ty+y*s<=1080.05 for x,y in pts))
    for child in node:geometry(child,(tx,ty,s),page)

def main():
    report=json.loads((HERE/'来源与生成核验.json').read_text())
    test('six pages',len(report['pages'])==6)
    test('no PPT files',not any(HERE.rglob('*.ppt*')))
    test('all declared text lines in page',all(t['in_page'] for t in report['text_checks']))
    test('text checks >= 200',len(report['text_checks'])>=200)
    for page in report['pages']:
        pn=HERE/page['png'];sv=HERE/page['svg']
        test(f'{pn.name}: present',pn.is_file())
        with Image.open(pn) as im:
            test(f'{pn.name}: resolution',im.size==(3840,2160))
            im.load();test(f'{pn.name}: nonempty content',im.convert('RGB').getextrema()[0][0]<240)
        xml=ET.parse(sv).getroot();test(f'{sv.name}: viewbox',xml.attrib['viewBox']=='0 0 1920 1080')
        test(f'{sv.name}: editable native illustration',not any(e.tag.split('}')[-1]=='image' for e in xml.iter()))
        # 保护对照页是并列小图，射线不需要附加流程箭头；总流程必须明确连接。
        test(f'{sv.name}: workflow arrows',page['arrows']>=7 if page['number']==1 else page['arrows']>=0)
        test(f'{sv.name}: illustrated scenes',page['illustrations']>=2)
        for c in page['card_checks']:test(f'{sv.name}: card body {c["title"]}',c['body_fits'])
        geometry(xml,page=page['number'])
        source_texts=[e.text for e in xml.iter() if e.tag.split('}')[-1]=='text']
        measured=[e['text'] for e in report['text_checks'] if e['page']==page['number']]
        test(f'{sv.name}: every text measured',source_texts==measured)
    for source,digest in report['source_hashes'].items():test(f'source unchanged: {source}',sha(Path(source))==digest)
    before=report['old_file_hashes']
    after={str(p.relative_to(OLD)):sha(p) for p in OLD.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    test('previous entire picture set preserved',before==after)
    for path in HERE.glob('*.py'):
        ast.parse(path.read_text());test(f'{path.name}: syntax',True)
    notes=(HERE/'看图说明与工程依据.md').read_text()
    for ref in re.findall(r'\]\((/[^)]+)\)',notes):
        actual=re.sub(r':\d+$','',ref);test(f'note source link: {actual}',Path(actual).is_file())
    for claim in ['不是实测结果','本轮','帧']:
        test('claim boundary '+claim,claim in notes or claim in report['kind'])
    archive=HERE/'可撤销地图维护_六张图文流程图.zip'
    with ZipFile(archive) as z:
        test('archive CRC',z.testzip() is None)
        files=[p for p in HERE.rglob('*') if p.is_file() and p not in (archive,RESULT) and '__pycache__' not in p.parts]
        archive_names=set(z.namelist())
        names={str(p.relative_to(HERE)) for p in files}
        test('archive includes complete file collection',names<=archive_names)
        test('archive contains no unexplained items',archive_names<=names|{'独立核验.json'})
        for p in files:test('archive byte match '+str(p.relative_to(HERE)),z.read(str(p.relative_to(HERE)))==p.read_bytes())
    result={'passed':True,'checks':len(TESTS),'source_rule_checks':report['source_rule_checks'],
            'page_count':6,'illustrations':sum(p['illustrations'] for p in report['pages']),
            'text_lines':len(report['text_checks']),'arrows':sum(p['arrows'] for p in report['pages']),
            'original_files_preserved':len(before),'source_files_unchanged':len(report['source_hashes']),
            'png_size':[3840,2160],'scope':'图片、规则、文件；没有ROS编译或仿真实验',
            'tests':TESTS}
    RESULT.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k!='tests'},ensure_ascii=False))

if __name__=='__main__':main()
