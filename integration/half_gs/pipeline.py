"""Measured half-warehouse capture -> learned masks -> explicit partial GS baseline."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[2]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')


def run(args):
    dataset=Path(args.dataset).resolve();output=Path(args.output).resolve();logs=dataset/'postprocess_logs';logs.mkdir(exist_ok=True)
    status=dict(completed=False,stage='waiting_capture',dataset=str(dataset),output=str(output),partial_baseline=args.partial_baseline)
    write(dataset/'pipeline_status.json',status)
    def stage(name,command):
        status['stage']=name;write(dataset/'pipeline_status.json',status);print('STAGE '+name,flush=True)
        with (logs/(name+'.log')).open('w') as stream:subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True)
    gpu=lambda script,*rest:['bash',str(ROOT/'scripts/run_sh/half_gs_python.sh'),str(ROOT/script),*map(str,rest)]
    ros=lambda script,*rest:['/usr/bin/bash','-ec','source /opt/ros/noetic/setup.bash; exec /usr/bin/python3 "$@"','half-gs-post',str(ROOT/script),*map(str,rest)]
    try:
        until=time.monotonic()+args.wait_seconds
        while True:
            try:result=read(dataset/'result.json') if (dataset/'result.json').exists() else {}
            except json.JSONDecodeError:time.sleep(.2);continue
            if result.get('success') and result.get('landed') and result.get('disarmed'):break
            if result.get('error'):raise RuntimeError('Capture failed; no model trained: '+result['error'])
            if not args.wait or time.monotonic()>until:raise RuntimeError('Capture has not completed normally')
            time.sleep(3)
        while subprocess.run(['systemctl','--user','is-active','--quiet','astradrone-half-gs-capture.service']).returncode==0:time.sleep(2)
        if not result['source_unchanged'] or not read(dataset/'dataset_validation.json')['passed']:raise ValueError('Capture integrity not verified')
        plan=read(dataset/'inspection_plan.json')
        if args.partial_baseline and plan.get('partial_baseline_authorized') is not True:raise ValueError('Missing recorded partial-baseline authorization')
        scripts=['masks.py','depth_samples.py','semantic_points.py','audit.py','pipeline.py']
        archive=dataset/'postprocess_sources';archive.mkdir(exist_ok=True);source_index={}
        for name in scripts:
            path=ROOT/'integration/half_gs'/name;digest=sha(path);copy=archive/(digest[:16]+'__'+name);shutil.copyfile(path,copy)
            source_index[str(path)]=dict(sha256=digest,snapshot=str(copy.relative_to(dataset)))
        write(dataset/'postprocess_sources.json',source_index)
        if not (dataset/'mask_manifest.json').exists():
            for folder in ['dataset/masks','dataset/person_core','mask_preview']:
                p=dataset/folder
                if p.exists():p.rename(p.with_name(p.name+'_incomplete_'+str(time.time_ns())))
            stage('learned_masks',gpu('integration/half_gs/masks.py',dataset))
        masks=read(dataset/'mask_manifest.json')
        if masks['simulator_truth_used'] or masks['person_positive_images']==0:raise ValueError('No real learned person masks were verified')
        frames=read(dataset/'dataset/frames.json')
        if len(masks['frames'])!=len(frames):raise ValueError('Learned masks do not cover every curated RGB')
        for row,frame in zip(masks['frames'],frames):
            if row['image_sha256']!=frame['sha256'] or sha(dataset/'dataset/masks'/row['mask'])!=row['mask_sha256']:
                raise ValueError('Image/mask provenance checksum changed')
        if not (dataset/'lidar_depth_manifest.json').exists():
            folder=dataset/'dataset/lidar_depth'
            if folder.exists():folder.rename(folder.with_name('lidar_depth_incomplete_'+str(time.time_ns())))
            stage('measured_lidar_depth',ros('integration/half_gs/depth_samples.py',dataset))
        if not (dataset/'semantic_filter.json').exists():
            if (dataset/'pre_semantic_static_map.npz').exists():
                for name in ['static_map.npz','static_map.ply','static_map.pcd','dataset_validation.json']:
                    original=dataset/('pre_semantic_'+name)
                    if original.exists():shutil.copyfile(original,dataset/name)
            stage('semantic_point_votes',gpu('integration/half_gs/semantic_points.py',dataset))
        stage('revalidate_dataset',ros('integration/lio_dataset/export.py',dataset))
        stage('fixed_coverage_audit',gpu('integration/half_gs/audit.py',dataset))
        acceptance=read(dataset/'acceptance.json')
        if not acceptance['dataset_ready']:raise ValueError('Dataset integrity failed after preprocessing')
        if not args.partial_baseline and not acceptance['joint_complete']:raise ValueError('Strict coverage gate not passed; no training')
        if output.exists():raise FileExistsError('Preserving existing model run; choose a fresh output: '+str(output))
        command=['bash',str(ROOT/'scripts/run_sh/warehouse20_gsplat.sh'),'--dataset',str(dataset),
            '--mask-dir',str(dataset/'dataset/masks'),'--output',str(output),'--steps',str(args.steps),'--width','960',
            '--max-gaussians','800000','--output-gib','3','--eps2d','.1','--min-scale','.001',
            '--edge-weight','.15','--means-lr-final','1e-6','--grow-grad','.0008','--absgrad']
        if args.partial_baseline:command.append('--allow-partial-map')
        else:command.append('--require-complete-photos')
        stage('train_gsplat',command)
        stage('verify_trained_exports',gpu('integration/lio_gs/verify.py',output))
        if not all(sha(path)==row['sha256'] for path,row in source_index.items()):raise ValueError('Postprocessing sources changed during pipeline')
        training=read(output/'gsplat/training_result.json');validation=read(output/'gsplat/export_validation.json')
        receipt=dict(completed=True,scene='Original half warehouse visual layout, private matching mesh collisions and two pedestrians',
            dataset=str(dataset),output=str(output),mask_source=masks['source'],mask_truth_used=False,
            person_positive_images=masks['person_positive_images'],images=masks['images'],
            semantic_removed_points=read(dataset/'semantic_filter.json')['removed_points'],
            input_coverage=read(dataset/'coverage.json'),strict_joint_complete=acceptance['joint_complete'],
            partial_baseline_authorized=bool(args.partial_baseline),training=training,export_validation=validation,
            scope='Actual SITL major-region capture + image-model masks + static measured cloud + offline GS; incomplete coverage kept, not autonomous unknown-scene or real-flight validation')
        write(output/'acceptance.json',receipt);write(dataset/'pipeline_result.json',receipt)
        status.update(completed=True,stage='completed',model_ply_sha256=validation['model_sha256']);write(dataset/'pipeline_status.json',status)
        (output/'README.md').write_text('# 双行人半仓库离线3DGS基线\n\n数据集：'+str(dataset)+'\n\n图像模型Mask、实测静态点云与LIO位姿；完整性缺口见acceptance.json，不称100%完整。\n\n模型：gsplat/model.pt、model.ply、gaussians.npz；实际视频/六向图/留出对照在gsplat目录。\n')
        print(json.dumps(status,indent=2),flush=True)
    except BaseException as error:
        status.update(error=str(error),completed=False);write(dataset/'pipeline_status.json',status);raise


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--output',required=True);p.add_argument('--wait',action='store_true')
    p.add_argument('--wait-seconds',type=float,default=14400);p.add_argument('--partial-baseline',action='store_true');p.add_argument('--steps',type=int,default=12000)
    run(p.parse_args())
