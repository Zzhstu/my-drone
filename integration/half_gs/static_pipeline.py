"""Actor-free measured capture -> fixed audit -> explicit partial offline GS."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time
ROOT=Path(__file__).resolve().parents[2]


def read(p):return json.loads(p.read_text())
def write(p,d):p.write_text(json.dumps(d,indent=2)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def run(args):
    dataset=Path(args.dataset).resolve();output=Path(args.output).resolve()
    logs=dataset/'postprocess_logs';logs.mkdir(exist_ok=True)
    status=dict(completed=False,stage='waiting_capture',dataset=str(dataset),output=str(output),actors=False)
    def stage(name,command):
        status['stage']=name;write(dataset/'pipeline_status.json',status);print('STAGE '+name,flush=True)
        with (logs/(name+'.log')).open('w') as f:subprocess.run(command,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
    gpu=lambda script,*rest:['bash',str(ROOT/'scripts/run_sh/half_gs_python.sh'),str(ROOT/script),*map(str,rest)]
    write(dataset/'pipeline_status.json',status)
    try:
        until=time.monotonic()+args.wait_seconds
        while True:
            try:result=read(dataset/'result.json') if (dataset/'result.json').exists() else {}
            except json.JSONDecodeError:time.sleep(.2);continue
            if result.get('error'):raise RuntimeError('Capture failed: '+result['error'])
            if result.get('success') and result.get('landed') and result.get('disarmed'):break
            if not args.wait or time.monotonic()>until:raise RuntimeError('Capture not completed normally')
            time.sleep(3)
        while subprocess.run(['systemctl','--user','is-active','--quiet',args.capture_unit]).returncode==0:time.sleep(2)
        if result['pedestrians'] or not result.get('actor_free_verified') or read(dataset/'scene_scene.json')['actors']:
            raise ValueError('Not a verified actor-free capture')
        if not result['source_unchanged'] or not read(dataset/'dataset_validation.json')['passed']:raise ValueError('Capture integrity failed')
        stage('fixed_coverage_audit',gpu('integration/half_gs/audit.py',dataset))
        acceptance=read(dataset/'acceptance.json')
        if not acceptance['dataset_ready']:raise ValueError('Invalid measured dataset')
        if not acceptance['joint_complete'] and not args.partial_baseline:raise ValueError('Strict completeness not passed')
        if args.partial_baseline and acceptance.get('baseline_training_authorized') is not True:raise ValueError('Missing baseline authorization')
        command=['bash',str(ROOT/'scripts/run_sh/warehouse20_gsplat.sh'),'--dataset',str(dataset),'--output',str(output),
            '--steps',str(args.steps),'--width',str(args.width),'--max-gaussians','800000','--output-gib','3',
            '--eps2d','.1','--min-scale','.001','--edge-weight','.15','--means-lr-final','1e-6',
            '--grow-grad','.0008','--refine-until','8500','--absgrad']
        command.append('--allow-partial-map' if args.partial_baseline else '--require-complete-photos')
        stage('train_gsplat',command)
        stage('verify_trained_exports',gpu('integration/lio_gs/verify.py',output))
        training=read(output/'gsplat/training_result.json');exports=read(output/'gsplat/export_validation.json')
        receipt=dict(completed=True,dataset=str(dataset),output=str(output),actors=False,mask_source='Implicit all-valid actor-free capture',
            truth_geometry_used_for_training=False,strict_joint_complete=acceptance['joint_complete'],
            input_coverage=read(dataset/'coverage.json'),training=training,export_validation=exports,
            scope='Actual static multi-height SITL capture and offline GS; planned and measured coverage kept separate; not real-flight safety or fully observed hidden surfaces')
        write(output/'acceptance.json',receipt);write(dataset/'pipeline_result.json',receipt)
        status.update(completed=True,stage='completed',model_sha256=exports['model_sha256']);write(dataset/'pipeline_status.json',status)
        print(json.dumps(status,indent=2),flush=True)
    except BaseException as e:
        status.update(completed=False,error=str(e));write(dataset/'pipeline_status.json',status);raise


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--output',required=True)
    p.add_argument('--wait',action='store_true');p.add_argument('--partial-baseline',action='store_true')
    p.add_argument('--wait-seconds',type=float,default=14400);p.add_argument('--steps',type=int,default=16000);p.add_argument('--width',type=int,default=960)
    p.add_argument('--capture-unit',default='astradrone-static-half-gs-capture.service');run(p.parse_args())
