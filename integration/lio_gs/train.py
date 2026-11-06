#!/usr/bin/env python3
"""Measured LiDAR-initialized, optionally masked, offline gsplat 1.5.3 baseline."""
import argparse
from collections import OrderedDict
from datetime import datetime
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
if __name__=='__main__' and not os.environ.get('ROOMGS_CUDA_LIB_READY'):
    env=os.environ.copy();prefix=Path(sys.executable).parent.parent
    env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','');env['ROOMGS_CUDA_LIB_READY']='1'
    os.execve(sys.executable,[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]],env)
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'integration/roomgs'))
import numpy as np
from PIL import Image,ImageDraw
from scipy.spatial.transform import Rotation,Slerp
from scipy.spatial import cKDTree
import torch
import torch.nn.functional as F
import gsplat
from gsplat import export_splats
from gsplat.strategy import DefaultStrategy
from integration.roomgs import train as base
from integration.lio_gs.dataset import load,initial_colours,sha
from integration.lio_gs.losses import photometric,image_metrics,edge_loss
from integration.lio_gs.rendering import render as render_gaussians


def write(path,value):Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')


class BudgetStrategy(DefaultStrategy):
    def __init__(self,maximum,**kwargs):super().__init__(**kwargs);self.maximum=maximum

    @torch.no_grad()
    def _grow_gs(self,params,optimizers,state,step):
        available=self.maximum-len(params['means'])
        if available<=0:return 0,0
        gradients=state['grad2d']/state['count'].clamp_min(1)
        if int((gradients>self.grow_grad2d).sum())>available:
            keep=torch.zeros_like(gradients);keep[torch.topk(gradients,available).indices]=1
            state['grad2d'].mul_(keep)
        return super()._grow_gs(params,optimizers,state,step)


def resource_check(folder,args):
    fields={line.split(':')[0]:line.split(':')[1].strip() for line in Path('/proc/self/status').read_text().splitlines() if ':' in line}
    rss=int(fields['VmRSS'].split()[0])*1024;hwm=int(fields['VmHWM'].split()[0])*1024
    available=next(int(s.split()[1])*1024 for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:'))
    size=sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())
    if rss>args.ram_gib*2**30 or available<1.0*2**30:raise RuntimeError('RAM budget/reserve reached; original input retained')
    if size>args.output_gib*2**30 or shutil.disk_usage(folder).free<20*2**30:raise RuntimeError('Output storage budget/reserve reached')
    return dict(rss_gib=rss/2**30,kernel_peak_rss_gib=hwm/2**30,available_ram_gib=available/2**30,
        output_gib=size/2**30,gpu_peak_mib=torch.cuda.max_memory_allocated()/2**20)


def snapshot(folder):
    sources=[Path(__file__),ROOT/'integration/lio_gs/dataset.py',ROOT/'integration/lio_gs/losses.py',ROOT/'integration/lio_gs/rendering.py',
        ROOT/'integration/lio_dataset/poses.py',ROOT/'integration/roomgs/train.py',ROOT/'integration/roomgs/image_sequence.py',
        Path(inspect.getfile(DefaultStrategy)),Path(inspect.getfile(gsplat.rasterization)),ROOT/'scripts/run_sh/warehouse20_gsplat.sh']
    snapshots=folder/'training_sources';snapshots.mkdir();manifest={}
    for path in sources:
        digest=sha(path);destination=snapshots/(digest[:16]+'__'+path.name);shutil.copyfile(path,destination)
        manifest[str(path)]=dict(sha256=digest,snapshot=str(destination.relative_to(folder)))
    write(folder/'training_sources.json',manifest);return manifest


def run(args):
    if gsplat.__version__!='1.5.3':raise RuntimeError('This adapter is validated against gsplat 1.5.3')
    if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
    if args.steps<100 or args.width<32 or not 0<=args.mask_dilate<=10:raise ValueError('Invalid training configuration')
    if min(args.eps2d,args.min_scale,args.means_lr,args.grow_grad)<=0 or args.edge_weight<0 or (args.means_lr_final is not None and args.means_lr_final<=0):
        raise ValueError('Invalid detail configuration')
    if any(value is not None and (not math.isfinite(value) or value<=0) for value in (args.color_lr,args.sh_lr)):
        raise ValueError('Appearance learning rates must be finite and positive')
    output=Path(args.output).resolve() if args.output else ROOT/'gsplat_runs'/datetime.now().strftime('warehouse20_lio_%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False);folder=output/'gsplat';folder.mkdir();(output/'dataset').mkdir()
    write(output/'config.json',vars(args));sources=snapshot(folder)
    torch.set_num_threads(2);torch.manual_seed(19);np.random.seed(19)
    torch.cuda.set_per_process_memory_fraction(min(1.,args.gpu_memory_gib*2**30/torch.cuda.get_device_properties(0).total_memory))
    start=time.monotonic();report=dict(trained=False,output=str(output),scope='Offline static-background gsplat baseline; no dynamic-scene performance claim')
    params=None
    try:
        data=load(args.dataset,args.width,args.mask_dir,args.mask_dilate,args.require_complete_photos,args.allow_partial_map)
        if len(data['xyz'])>args.max_gaussians:raise ValueError('Gaussian budget smaller than measured initialization cloud')
        images,poses,k,xyz,train,test=[data[n] for n in ('images','poses','K','xyz','train','test')]
        height,width=data['height'],data['width'];masks=data['masks']
        shutil.copyfile(data['source']/'dataset/frames.json',output/'dataset/frames.json')
        shutil.copyfile(data['source']/'camera_input.json',output/'camera_input.json')
        write(output/'source_dataset.json',dict(path=str(data['source']),input_sha256=data['fingerprints'],
            photo_missing_samples=data['acceptance']['multiview_missing_samples'],joint_complete=data['acceptance']['joint_complete'],
            truth_geometry_used_for_training=False,geometry_source='Actual static_map.npz measured points; no CAD filling',
            partial_baseline_authorized=bool(args.allow_partial_map and data['acceptance'].get('baseline_training_authorized')),
            input_geometry_complete=data['acceptance']['geometry_complete'],mask_source=data['acceptance'].get('mask_source','implicit all-valid/static scene')))
        write(folder/'mask_manifest.json',dict(enabled=any(row['path'] for row in data['mask_manifest']),
            convention='255 valid static pixels, 0 ignored pixels',dilate_ignored_pixels=args.mask_dilate,
            no_mask_policy='Implicit all-valid; current scene contains no pedestrians',frames=data['mask_manifest'],
            excluded_all_black=data['excluded_all_black']))
        write(folder/'split.json',dict(train_indices=train.tolist(),test_indices=test.tolist(),
            policy='Every eighth eligible actual image held out; RGB appearance initialization uses train split only',
            scope='LIO poses and LiDAR map use the acquisition; this is RGB appearance holdout, not independent pose/geometry truth'))
        warm=bool(args.initialize_model)
        if warm:
            seed_path=Path(args.initialize_model).resolve();seed_root=seed_path.parent.parent
            seed_source=json.loads((seed_root/'source_dataset.json').read_text())
            seed_split=json.loads((seed_path.parent/'split.json').read_text())
            if seed_source['input_sha256']!=data['fingerprints'] or seed_split['train_indices']!=train.tolist() or seed_split['test_indices']!=test.tolist():
                raise ValueError('Warm start requires identical measured inputs, masks and appearance split')
            seed=torch.load(seed_path,map_location='cpu',weights_only=False)
            params=torch.nn.ParameterDict({n:torch.nn.Parameter(v.to('cuda')) for n,v in seed['params'].items()})
            tree=cKDTree(xyz);colored=None;seed_hash=sha(seed_path)
            write(folder/'warm_start.json',dict(path=str(seed_path),sha256=seed_hash,gaussians=len(params['means']),
                policy='Learned parameters only; fresh optimizer and density state, not exact checkpoint resume'))
        else:
            color,colored=initial_colours(data);params,tree=base.init_splats(xyz,color,'cuda')
        if len(params['means'])>args.max_gaussians:raise ValueError('Initialization exceeds Gaussian budget')
        initial_gaussian_count=len(params['means'])
        def render(params,pose,k,width,height,degree=3,depth=False):
            return render_gaussians(params,pose,k,width,height,degree,depth,args.eps2d,args.absgrad and torch.is_grad_enabled())
        initial_means=params['means'].detach().clone();tk=torch.tensor(k,device='cuda');tp=torch.tensor(poses,device='cuda')
        multiplier=.4 if warm else 1.
        rates=dict(means=args.means_lr,scales=5e-3*multiplier,quats=1e-3*multiplier,
            opacities=.03*multiplier,sh0=.0025*multiplier,shN=.000125*multiplier)
        if args.color_lr is not None:rates['sh0']=args.color_lr
        if args.sh_lr is not None:rates['shN']=args.sh_lr
        optimizers={n:torch.optim.Adam([p],lr=rates[n],eps=1e-15) for n,p in params.items()}
        strategy=BudgetStrategy(args.max_gaussians,refine_start_iter=100 if warm else 500,refine_stop_iter=min(args.refine_until,args.steps-1000),
            refine_every=100,reset_every=10000,prune_opa=.001,prune_scale3d=.15,grow_grad2d=args.grow_grad,absgrad=args.absgrad,verbose=False)
        strategy.check_sanity(params,optimizers);state=strategy.initialize_state(scene_scale=6.)
        selected=set(test[np.linspace(0,len(test)-1,min(8,len(test))).astype(int)].tolist());initial=[]
        with torch.no_grad():
            for i in sorted(selected):
                pred,_,_=render(params,tp[i],tk,width,height,degree=3 if warm else 0)
                target=torch.tensor(images[i],device='cuda',dtype=torch.float32)/255
                mask=torch.tensor(masks[i],device='cuda');initial.append(dict(index=int(i),**image_metrics(pred.clamp(0,1),target,mask)))
        write(folder/'initial_heldout_metrics.json',dict(frames=initial,psnr_mean=float(np.mean([r['psnr'] for r in initial]))))
        write(folder/'provenance.json',dict(gsplat=gsplat.__version__,torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(0),
            actual_initial_points=len(xyz),initial_points_with_training_rgb=colored,pose_source='FAST-LIO + model optical extrinsic at original RGB stamps',
            optimizer_inputs='Measured cloud, train RGB/masks, calibrated camera poses/K only',truth_geometry_used_for_training=False,
            photometric='Valid-pixel L1; SSIM only wholly valid in-image windows; no black RGB targets',depth='Projected measured LiDAR z-buffer; masked pixels excluded'))
        depths=OrderedDict();anchors=None;order=[];rng=np.random.RandomState(91);history=[]
        log=(folder/'training.jsonl').open('w',buffering=1)
        for step in range(args.steps):
            if not order:order=rng.permutation(train).tolist()
            i=int(order.pop());target=torch.tensor(images[i],device='cuda',dtype=torch.float32)/255
            final_lr=args.means_lr_final if args.means_lr_final is not None else args.means_lr
            optimizers['means'].param_groups[0]['lr']=args.means_lr*(final_lr/args.means_lr)**(step/max(1,args.steps-1))
            mask=torch.tensor(masks[i],device='cuda');degree=3 if warm else min(3,step//1800)
            prediction,alpha,meta=render(params,tp[i],tk,width,height,degree,depth=True)
            strategy.step_pre_backward(params,optimizers,state,step,meta)
            l1,structural=photometric(prediction[...,:3],target,mask)
            if i not in depths:depths[i]=base.depth_prior(xyz,poses[i],k,width,height)
            depth=depths.pop(i);depths[i]=depth
            while len(depths)>32:depths.popitem(last=False)
            prior=torch.tensor(depth,device='cuda');valid=torch.isfinite(prior)&mask
            dl=(torch.abs(prediction[...,3][valid]-prior[valid])/prior[valid].clamp_min(.4)).mean() if bool(valid.any()) else prediction.sum()*0
            if anchors is None or len(anchors)!=len(params['means']) or step%100==0:
                _,nearest=tree.query(params['means'].detach().cpu().numpy(),workers=2);anchors=torch.tensor(xyz[nearest],device='cuda')
            geometry_loss=F.smooth_l1_loss(params['means'],anchors,beta=.04)
            edges=edge_loss(prediction[...,:3],target,mask) if args.edge_weight else prediction.sum()*0
            loss=.8*l1+.2*structural+.05*dl+.10*geometry_loss+args.edge_weight*edges
            if not torch.isfinite(loss):raise RuntimeError('Non-finite training loss')
            loss.backward()
            for optimizer in optimizers.values():optimizer.step();optimizer.zero_grad(set_to_none=True)
            with torch.no_grad():params['scales'].clamp_(math.log(args.min_scale),math.log(.18));params['opacities'].clamp_(-10,10)
            strategy.step_post_backward(params,optimizers,state,step,meta,packed=False)
            if len(params['means'])>args.max_gaussians:raise RuntimeError('Gaussian count budget invariant failed')
            if step%100==0 or step==args.steps-1:
                resources=resource_check(output,args)
                row=dict(step=step+1,loss=float(loss.detach()),l1=float(l1.detach()),depth=float(dl.detach()),edge=float(edges.detach()),
                    gaussians=len(params['means']),wall_seconds=time.monotonic()-start,**resources)
                history.append(row);log.write(json.dumps(row)+'\n');print(json.dumps(row),flush=True)
            if step and step%3000==0:
                torch.save(dict(step=step,params={n:p.detach().cpu() for n,p in params.items()}),folder/'checkpoint_latest.tmp')
                os.replace(folder/'checkpoint_latest.tmp',folder/'checkpoint_latest.pt')
        log.close();torch.cuda.synchronize();learned={n:p.detach().cpu() for n,p in params.items()}
        torch.save(dict(params=learned,K=k,width=width,height=height,steps=args.steps,eps2d=args.eps2d),folder/'model.pt')
        export_splats(**{n:params[n].detach() for n in ('means','scales','quats','opacities','sh0','shN')},format='ply',save_to=str(folder/'model.ply'))
        np.savez_compressed(folder/'gaussians.npz',**{n:p.numpy() for n,p in learned.items()})
        metrics=[];comparisons=[]
        with torch.no_grad():
            for i in test:
                pred,alpha,_=render(params,tp[i],tk,width,height);target=torch.tensor(images[i],device='cuda',dtype=torch.float32)/255
                mask=torch.tensor(masks[i],device='cuda');rgb=pred.clamp(0,1)
                metrics.append(dict(index=int(i),mean_alpha=float(alpha.mean()),**image_metrics(rgb,target,mask)))
                if int(i) in selected:comparisons.append(np.concatenate([images[i],(rgb.cpu().numpy()*255).astype(np.uint8)],axis=1))
        heldout=dict(count=len(metrics),psnr_mean=float(np.mean([r['psnr'] for r in metrics])),
            ssim_mean=float(np.mean([r['ssim'] for r in metrics if r['ssim'] is not None])),frames=metrics,
            scope='Held-out RGB appearance only; original LIO/geometry use the whole capture; masks applied to evaluation')
        write(folder/'heldout_metrics.json',heldout);Image.fromarray(np.concatenate(comparisons,axis=0)).save(folder/'heldout_comparison.png')
        positions=poses[:,:3,3];middle=np.median(positions,axis=0);clearance=tree.query(positions,workers=2)[0]
        score=np.linalg.norm(positions-middle,axis=1);score[clearance<.35]=np.inf
        center_id=int(np.argmin(score)) if np.isfinite(score).any() else int(np.argmax(clearance));center=positions[center_id]
        pictures=[];cameras=[]
        with torch.no_grad():
            for name,direction in [('east',[1,0,0]),('north',[0,1,0]),('west',[-1,0,0]),('south',[0,-1,0]),('ceiling',[0,0,1]),('floor',[0,0,-1])]:
                pose=base.look_at(center,direction);rgb,alpha,_=render(params,torch.tensor(pose,device='cuda',dtype=torch.float32),tk,width,height)
                picture=Image.fromarray((rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8));ImageDraw.Draw(picture).text((12,12),name,fill='white')
                picture.save(folder/(name+'.png'));pictures.append(np.asarray(picture));cameras.append(dict(name=name,c2w=pose.tolist()))
        Image.fromarray(np.concatenate([np.concatenate(pictures[:3],1),np.concatenate(pictures[3:],1)],0)).save(folder/'room_six_views.png')
        write(folder/'six_view_cameras.json',cameras)
        import imageio.v2 as imageio
        video=imageio.get_writer(str(folder/'walkthrough.mp4'),fps=20,macro_block_size=1)
        with torch.no_grad():
            for i in np.linspace(0,len(poses)-2,180).astype(int):
                pose=np.eye(4);pose[:3,3]=(poses[i,:3,3]+poses[i+1,:3,3])/2
                pose[:3,:3]=Slerp([0,1],Rotation.from_matrix(poses[[i,i+1],:3,:3]))([.5]).as_matrix()[0]
                rgb,_,_=render(params,torch.tensor(pose,device='cuda',dtype=torch.float32),tk,width,height)
                video.append_data((rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8))
        video.close()
        input_unchanged=all(sha(p)==h for p,h in data['fingerprints'].items())
        source_unchanged=all(sha(p)==row['sha256'] for p,row in sources.items())
        seed_unchanged=not warm or sha(seed_path)==seed_hash
        report.update(trained=True,steps=args.steps,gaussians=len(params['means']),initial_gaussians=initial_gaussian_count,
            source_unchanged=source_unchanged,input_unchanged=input_unchanged,seed_unchanged=seed_unchanged,wall_seconds=time.monotonic()-start,
            train_images=len(train),heldout_images=len(test),initial_measured_points=len(xyz),heldout_psnr=heldout['psnr_mean'],heldout_ssim=heldout['ssim_mean'],
            same_eight_initial_psnr=float(np.mean([r['psnr'] for r in initial])),
            same_eight_final_psnr=float(np.mean([r['psnr'] for r in metrics if r['index'] in selected])),
            photo_missing_samples=data['acceptance']['multiview_missing_samples'],strict_joint_complete=False,
            mask_enabled=any(r['path'] for r in data['mask_manifest']),video_frames=180,
            model_sha256=sha(folder/'model.ply'),eps2d=args.eps2d,warm_start=warm,**resource_check(output,args))
        report['input_geometry_complete']=data['acceptance']['geometry_complete']
        report['partial_baseline_authorized']=bool(args.allow_partial_map and data['acceptance'].get('baseline_training_authorized'))
        report['mask_source']=data['acceptance'].get('mask_source','implicit all-valid/static scene')
        write(folder/'training_result.json',report)
        if not input_unchanged or not source_unchanged or not seed_unchanged:raise RuntimeError('Inputs, seed or training source changed')
        print(json.dumps(report,indent=2),flush=True)
    except BaseException as error:
        write(folder/'failure.json',dict(error=str(error),trained=report['trained'],wall_seconds=time.monotonic()-start))
        raise
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',default=str(ROOT/'lio_dataset_runs/warehouse20_20261102_capture'))
    parser.add_argument('--output');parser.add_argument('--steps',type=int,default=12000)
    parser.add_argument('--width',type=int,default=640);parser.add_argument('--mask-dir')
    parser.add_argument('--mask-dilate',type=int,default=3);parser.add_argument('--require-complete-photos',action='store_true')
    parser.add_argument('--allow-partial-map',action='store_true',help='Explicit baseline only; also requires recorded dataset authorization; never marks coverage complete')
    parser.add_argument('--gpu-memory-gib',type=float,default=6.);parser.add_argument('--ram-gib',type=float,default=5.)
    parser.add_argument('--output-gib',type=float,default=2.);parser.add_argument('--max-gaussians',type=int,default=350000)
    parser.add_argument('--initialize-model');parser.add_argument('--eps2d',type=float,default=.3)
    parser.add_argument('--min-scale',type=float,default=.002);parser.add_argument('--edge-weight',type=float,default=0.)
    parser.add_argument('--means-lr',type=float,default=1.6e-4);parser.add_argument('--means-lr-final',type=float)
    parser.add_argument('--color-lr',type=float,help='Explicit SH0 learning rate; default preserves cold/warm policy')
    parser.add_argument('--sh-lr',type=float,help='Explicit higher-SH learning rate; default preserves cold/warm policy')
    parser.add_argument('--refine-until',type=int,default=6500);parser.add_argument('--grow-grad',type=float,default=.0003)
    parser.add_argument('--absgrad',action='store_true')
    run(parser.parse_args())
