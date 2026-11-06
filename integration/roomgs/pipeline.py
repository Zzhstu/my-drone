#!/usr/bin/env python3
"""Sequential capture -> 100% finite-resolution map gate -> offline gsplat."""
import argparse
from pathlib import Path
from datetime import datetime
import json
import os
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]


def validate_capture(output):
    capture=json.loads((output/'result.json').read_text())
    coverage=json.loads((output/'coverage.json').read_text())
    if (not capture['success'] or not capture.get('source_unchanged') or not coverage['passed']
            or coverage['complete_fraction']!=1.0 or coverage['required_fraction']!=1.0
            or coverage['missing_samples']!=0 or coverage['total_samples']<=0):
        raise RuntimeError('地图未达100%预定义采样完整性，禁止进入3DGS训练；检查coverage.json和缺口清单')
    return capture,coverage


def run(args):
    output=Path(args.train_only) if args.train_only else Path(args.run_dir or ROOT/'roomgs_runs'/(args.scene+'_'+datetime.now().strftime('%Y%m%d_%H%M%S')))
    if not args.train_only:
        cmd=[sys.executable,str(ROOT/'integration/roomgs/capture.py'),'--output-dir',str(output)]
        cmd+=['--scene',args.scene]
        if args.headless:cmd.append('--headless')
        if args.resource_preflight:cmd+=['--no-flight','--resource-preflight','--stationary-seconds','30']
        if args.inspection_plan:cmd+=['--inspection-plan',args.inspection_plan]
        if args.auxiliary_capture:cmd.append('--auxiliary-capture')
        captured=subprocess.run(cmd)
        if args.resource_preflight or args.auxiliary_capture:
            captured.check_returncode()
            return
        if captured.returncode:
            # Never relax acceptance. Only real observations already in this
            # completed flight's bag may repair keyframe thinning.
            from recover_pairs import recover
            recover(output)
    capture,coverage=validate_capture(output)
    if args.capture_only:return
    env=os.environ.copy();env.pop('PYTHONPATH',None)
    cuda=ROOT/'.envs/roomgs'
    env.update(CUDA_HOME=str(cuda),PATH=str(cuda/'bin')+':'+env.get('PATH',''),
               CPATH=str(cuda/'targets/x86_64-linux/include'),
               LIBRARY_PATH=str(cuda/'lib'),MAX_JOBS='1',TORCH_CUDA_ARCH_LIST='12.0',
               CXX='/usr/bin/g++',CC='/usr/bin/gcc')
    subprocess.run([str(cuda/'bin/python'),str(ROOT/'integration/roomgs/train.py'),
                    '--capture',str(output),'--steps',str(args.steps)],env=env,check=True)
    print('完整地图与离线3DGS已保存：'+str(output),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--capture-only',action='store_true')
    p.add_argument('--resource-preflight',action='store_true')
    p.add_argument('--auxiliary-capture',action='store_true')
    p.add_argument('--inspection-plan')
    p.add_argument('--train-only')
    p.add_argument('--run-dir')
    p.add_argument('--steps',type=int,default=12000)
    p.add_argument('--scene',choices=['warehouse20','room'],default='warehouse20')
    run(p.parse_args())
