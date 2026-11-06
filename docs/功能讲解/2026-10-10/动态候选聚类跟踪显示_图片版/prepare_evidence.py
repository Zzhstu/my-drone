#!/usr/bin/env python3
"""冻结源码/旧材料校验和，复制真实证据；不启动ROS或改动科研实现。"""
import hashlib
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BRIDGE = ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    target = HERE/'源码与数据核验.json'
    if target.exists():
        print('保留已冻结的来源校验和；重新生成图片不会刷新基线。')
        return
    sources = [p for folder in ('src','config','launch','rviz','msg','test')
               for p in (BRIDGE/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    sources += [BRIDGE/'CMakeLists.txt', Path('/home/a/下载/fastlio2ppt.pptx')]
    old = HERE.parent/'目标绕行规划_图片版'
    sources += [p for p in old.rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    sources += [ROOT/'AstraDrone_ros1_ws/src/SLAM/FAST_LIO/src/laserMapping.cpp']
    evidence = ['ours_20260924_144552_manual_test.json',
                'ours_cangku_20260927_121518_inflight_test.json',
                'ours_cangku_20260927_121518_rviz.png',
                'ours_cangku_20260927_121518_gazebo.png']
    copies=[]
    for name in evidence:
        src=ROOT/'dynamic_demo_runs'/name
        dst=HERE/'证据原件'/name
        dst.parent.mkdir(exist_ok=True)
        shutil.copyfile(src,dst)
        copies.append(dict(source=str(src),copy=str(dst.relative_to(HERE)),sha256=sha(src)))
    obj=dict(source_hashes={str(p):sha(p) for p in sources},copies=copies,
             scope='材料制作；原始测试是历史仿真，不是本轮新运行或当前版本效果')
    target.write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf8')
    print('冻结',len(sources),'个文件；复制',len(copies),'个证据原件')

if __name__=='__main__': main()
