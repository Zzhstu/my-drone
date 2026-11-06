#!/usr/bin/env python3
"""独立复验交付格式、源码未变、链接和SVG文字；非飞行验证。"""
import ast
import hashlib
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile
from PIL import Image

HERE=Path(__file__).resolve().parent
checks=[]


def check(name, value):
    checks.append(dict(check=name,passed=bool(value)))


def main():
    data=json.loads((HERE/'内容与来源.json').read_text())
    gen=json.loads((HERE/'生成核验.json').read_text())
    pages=data['pages']
    check('10页', len(pages)==10)
    check('无PPTX', not list(HERE.rglob('*.pptx')))
    check('生成阶段文字边界', gen['status']=='passed')
    check('参考风格配色', data['style']['title']=='#15334C' and data['style']['flow_teal']=='#137B7E')
    for path,expected in data['sources_sha256'].items():
        p=Path(path)
        check('来源存在 '+path,p.is_file())
        check('来源未改 '+path,hashlib.sha256(p.read_bytes()).hexdigest()==expected)
    ns={'s':'http://www.w3.org/2000/svg'}
    for page in pages:
        svg=Path(page['svg']);png=Path(page['png'])
        x=ET.parse(svg).getroot()
        check('SVG画布 '+svg.name,x.attrib['viewBox']=='0 0 1600 900')
        txt=[n.text or '' for n in x.findall('s:text',ns)]
        check('SVG正文对应 '+svg.name,txt==page['text'])
        check('中文标点行首 '+svg.name,all(not t.startswith(('。','，','；')) for t in txt))
        check('不含外链图像 '+svg.name,not x.findall('s:image',ns))
        with Image.open(png) as im:
            im.verify()
        with Image.open(png) as im:
            check('PNG高清 '+png.name,im.size==(3840,2160))
    notes=(HERE/'详细讲解与源码对应.md').read_text()
    check('10节详细讲解',len(re.findall(r'^## \d\d ',notes,re.M))==10)
    for link in re.findall(r'\]\((/[^)]+)\)',notes):
        check('正文链接 '+link,Path(link).is_file())
    check('明确二级接口边界','static_avoid' in notes and 'static_emergency' in notes)
    check('明确失效零级','零级与“确实' in notes or '“0”等级与“确实' in notes)
    check('未扩大安全结论','没有重新ROS编译、飞行、碰撞' in notes)
    for name in ['build_images.py','verify_images.py']:
        ast.parse((HERE/name).read_text())
        check('Python语法 '+name,True)
    with ZipFile(HERE/'风险与安全控制_高清图片合集.zip') as z:
        check('压缩包CRC',z.testzip() is None)
        for page in pages:
            for kind in ['svg','png']:
                p=Path(page[kind]);rel=str(p.relative_to(HERE))
                check('压缩包逐字节 '+rel,z.read(rel)==p.read_bytes())
        for name in ['详细讲解与源码对应.md','内容与来源.json','生成核验.json','build_images.py']:
            check('压缩包文本同步 '+name,z.read(name)==(HERE/name).read_bytes())
    report=dict(status='passed' if all(c['passed'] for c in checks) else 'failed',
                total=len(checks),passed=sum(c['passed'] for c in checks),
                failed=[c for c in checks if not c['passed']],
                checks=checks,scope='图文/来源/链接/打包验证；未ROS编译、飞行或碰撞测试')
    (HERE/'独立复验结果.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='checks'},ensure_ascii=False,indent=2))
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
