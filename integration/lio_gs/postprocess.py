"""Recover exports/evaluation from a fully saved final model, without optimizing."""
import argparse
import json
import os
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
import numpy as np
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation,Slerp
import torch
import imageio.v2 as imageio
from integration.lio_gs.dataset import load,sha
from integration.lio_gs.losses import image_metrics
from integration.lio_gs.rendering import render


def read(p):return json.loads(Path(p).read_text())
def write(p,v):Path(p).write_text(json.dumps(v,indent=2)+'\n')


def run(output):
    output=Path(output).resolve();folder=output/'gsplat';config=read(output/'config.json')
    if (folder/'training_result.json').exists():raise ValueError('Existing final report will not be overwritten')
    history=[json.loads(s) for s in (folder/'training.jsonl').read_text().splitlines()];last=history[-1]
    model=torch.load(folder/'model.pt',map_location='cpu',weights_only=False)
    if last['step']!=config['steps'] or model['steps']!=config['steps']:raise ValueError('Final optimization was not completed')
    model_hash=sha(folder/'model.pt');started=time.monotonic();torch.set_num_threads(2)
    torch.cuda.set_per_process_memory_fraction(.25)
    data=load(config['dataset'],config['width'],config['mask_dir'],config['mask_dilate'])
    source=read(output/'source_dataset.json')
    if data['fingerprints']!=source['input_sha256']:raise ValueError('Measured input changed')
    split=read(folder/'split.json')
    if split['test_indices']!=data['test'].tolist() or split['train_indices']!=data['train'].tolist():raise ValueError('Split changed')
    params={n:v.cuda() for n,v in model['params'].items()};eps=model.get('eps2d',.3)
    k=torch.tensor(data['K'],device='cuda');poses=torch.tensor(data['poses'],device='cuda');height,width=data['height'],data['width']
    selected=set(data['test'][np.linspace(0,len(data['test'])-1,8).astype(int)].tolist());metrics=[];comparisons=[]
    with torch.no_grad():
        for idx in data['test']:
            i=int(idx);rgb,alpha,_=render(params,poses[i],k,width,height,eps2d=eps);rgb=rgb.clamp(0,1)
            target=torch.tensor(data['images'][i],device='cuda',dtype=torch.float32)/255;mask=torch.tensor(data['masks'][i],device='cuda')
            metrics.append(dict(index=i,mean_alpha=float(alpha.mean()),**image_metrics(rgb,target,mask)))
            if i in selected:comparisons.append(np.concatenate([data['images'][i],(rgb.cpu().numpy()*255).astype(np.uint8)],1))
        warm=read(folder/'warm_start.json');parent=Path(warm['path']).parent
        cameras=read(parent/'six_view_cameras.json');pictures=[]
        for row in cameras:
            rgb,_,_=render(params,torch.tensor(row['c2w'],device='cuda',dtype=torch.float32),k,width,height,eps2d=eps)
            picture=Image.fromarray((rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8));ImageDraw.Draw(picture).text((12,12),row['name'],fill='white')
            picture.save(folder/(row['name']+'.png'));pictures.append(np.array(picture))
        write(folder/'six_view_cameras.json',cameras)
        Image.fromarray(np.concatenate([np.concatenate(pictures[:3],1),np.concatenate(pictures[3:],1)],0)).save(folder/'room_six_views.png')
        with imageio.get_writer(str(folder/'walkthrough.mp4'),fps=20,macro_block_size=1) as video:
            for i in np.linspace(0,len(data['poses'])-2,180).astype(int):
                pose=np.eye(4,dtype=np.float32);pose[:3,3]=data['poses'][[i,i+1],:3,3].mean(0)
                pose[:3,:3]=Slerp([0,1],Rotation.from_matrix(data['poses'][[i,i+1],:3,:3]))([.5]).as_matrix()[0]
                rgb,_,_=render(params,torch.tensor(pose,device='cuda'),k,width,height,eps2d=eps)
                video.append_data((rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8))
    heldout=dict(count=len(metrics),psnr_mean=float(np.mean([r['psnr'] for r in metrics])),ssim_mean=float(np.mean([r['ssim'] for r in metrics])),
        frames=metrics,scope='Same-capture appearance holdout; unchanged fixed original LIO poses and measured geometry')
    write(folder/'heldout_metrics.json',heldout);Image.fromarray(np.concatenate(comparisons)).save(folder/'heldout_comparison.png')
    snapshots=read(folder/'training_sources.json');source_unchanged=all(sha(p)==row['sha256'] and sha(folder/row['snapshot'])==row['sha256'] for p,row in snapshots.items())
    input_unchanged=all(sha(p)==h for p,h in source['input_sha256'].items());seed_unchanged=sha(warm['path'])==warm['sha256']
    initial=read(folder/'initial_heldout_metrics.json')
    status={s.split(':')[0]:s.split(':')[1] for s in Path('/proc/self/status').read_text().splitlines() if ':' in s}
    peak=max(last['kernel_peak_rss_gib'],int(status['VmHWM'].split()[0])/2**20)
    report=dict(trained=True,postprocess_resumed=True,original_process_exit_code=143,
        recovery_scope='Final 8000-step PT/PLY/NPZ already saved; only heldout rendering, six views, video and report resumed; no extra training',
        wall_seconds=time.time()-((folder/'training.jsonl').stat().st_mtime-last['wall_seconds']),
        wall_seconds_scope='Elapsed including interrupted interval and recovery; reconstructed from log mtime, not uninterrupted performance',
        optimization_logged_seconds=last['wall_seconds'],resumed_postprocess_seconds=time.monotonic()-started,
        output=str(output),steps=config['steps'],gaussians=len(params['means']),initial_gaussians=warm['gaussians'],initial_measured_points=len(data['xyz']),
        source_unchanged=source_unchanged,input_unchanged=input_unchanged,seed_unchanged=seed_unchanged,
        train_images=len(data['train']),heldout_images=len(metrics),heldout_psnr=heldout['psnr_mean'],heldout_ssim=heldout['ssim_mean'],
        same_eight_initial_psnr=initial['psnr_mean'],same_eight_final_psnr=float(np.mean([r['psnr'] for r in metrics if r['index'] in selected])),
        photo_missing_samples=10,strict_joint_complete=False,mask_enabled=False,video_frames=180,model_sha256=sha(folder/'model.ply'),
        eps2d=eps,warm_start=True,kernel_peak_rss_gib=peak,
        resource_scope='Maximum of logged training samples and resumed postprocess HWM; terminated-process export HWM unavailable',
        gpu_peak_mib=max(last['gpu_peak_mib'],torch.cuda.max_memory_allocated()/2**20),
        output_gib=sum(p.stat().st_size for p in output.rglob('*') if p.is_file())/2**30)
    if not all([input_unchanged,source_unchanged,seed_unchanged,sha(folder/'model.pt')==model_hash]):raise ValueError('Recovery source/model/input changed')
    write(folder/'postprocess_resume_sources.json',{str(p):sha(p) for p in [Path(__file__),ROOT/'integration/lio_gs/rendering.py',ROOT/'integration/lio_gs/losses.py',ROOT/'scripts/run_sh/postprocess_warehouse20_gsplat.sh']})
    write(folder/'training_result.json',report);print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output');run(p.parse_args().output)
