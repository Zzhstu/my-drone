#!/usr/bin/env python3
"""Verify report scope, source hashes and local links; never run flight nodes."""
import ast
import csv
import hashlib
import json
import re
import subprocess
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]

def main():
    with (HERE/'逐文件作用全集.csv').open(encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    indexed={r['相对路径'] for r in rows}
    checks=[]
    def check(name,condition): checks.append(dict(check=name,passed=bool(condition)))
    check('无重复文件',len(indexed)==len(rows))
    changed={p.decode() for p in subprocess.check_output(['git','diff','--name-only','-z','000b725','HEAD'],cwd=ROOT).split(b'\0') if p}
    check('基线到HEAD全部文件纳入',changed<=indexed)
    for directory in ('AstraDrone_ros1_ws/src/SLAM/fastlio_bridge','AstraDrone_ros1_ws/src/SLAM/freedom','EXP-main'):
        files={p.decode() for p in subprocess.check_output(['rg','--files','--hidden',directory],cwd=ROOT).splitlines() if p}
        check(directory+' 全部未忽略源文件纳入',files<=indexed)
    for r in rows:
        path=ROOT/r['相对路径']
        check(r['相对路径']+'存在且作用非空',path.is_file() and bool(r['作用']))
        if r['源码SHA256']:
            check(r['相对路径']+'源码哈希',hashlib.sha256(path.read_bytes()).hexdigest()==r['源码SHA256'])
    for path in HERE.glob('*.md'):
        text=path.read_text(encoding='utf8')
        check(path.name+'围栏配对',text.count('```')%2==0)
        for match in re.finditer(r'\]\((?:<([^>]+)>|([^\)]+))\)',text):
            link=match.group(1) or match.group(2)
            if not link.startswith(('http://','https://','#')):
                check(path.name+'链接 '+link,(HERE/link).exists())
    for path in HERE.glob('*.py'):
        ast.parse(path.read_text(encoding='utf8'),filename=str(path))
        check(path.name+'Python解析',True)
    result=dict(checks_total=len(checks),passed=sum(c['passed'] for c in checks),failed=[c for c in checks if not c['passed']],scope='只核对盘点路径、源码哈希及材料格式；不编译ROS、不运行飞行/算法实验')
    (HERE/'文档验证结果.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return int(bool(result['failed']))

if __name__=='__main__': raise SystemExit(main())
