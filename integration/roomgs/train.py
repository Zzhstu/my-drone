#!/usr/bin/env python3
"""Offline, LiDAR-initialized gsplat training. No scene mesh or truth is loaded.

gsplat v1.5.3 provides the differentiable rasterizer, density control and export.
This adapter supplies the measured R3LIVE RGB/c2w dataset and LiDAR priors.
"""
import argparse
from pathlib import Path
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from collections import OrderedDict
if __name__=='__main__' and not os.environ.get('ROOMGS_CUDA_LIB_READY'):
    # Use the isolated CUDA/toolchain runtime before any compiled module loads.
    env=os.environ.copy();prefix=Path(sys.executable).parent.parent
    env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    env['ROOMGS_CUDA_LIB_READY']='1'
    os.execve(sys.executable,[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]],env)
import numpy as np
import fcntl
_backend_lock=(Path(sys.executable).resolve().parents[1]/'.gsplat_backend.lock').open('a')
fcntl.flock(_backend_lock,fcntl.LOCK_EX)
from PIL import Image,ImageDraw
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation,Slerp
import torch
from torch import nn
import torch.nn.functional as F
from gsplat import rasterization,export_splats
from gsplat.strategy import DefaultStrategy
from geometry import read_pcd
from provenance import verify_sources
from image_sequence import CapturedImages

ROOT=Path(__file__).resolve().parents[2]
C0=.28209479177387814


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')


def similarity(a,b):
    a=a.permute(2,0,1)[None];b=b.permute(2,0,1)[None]
    mu_a=F.avg_pool2d(a,11,1,5);mu_b=F.avg_pool2d(b,11,1,5)
    va=F.avg_pool2d(a*a,11,1,5)-mu_a.square()
    vb=F.avg_pool2d(b*b,11,1,5)-mu_b.square()
    cov=F.avg_pool2d(a*b,11,1,5)-mu_a*mu_b
    return (((2*mu_a*mu_b+.01**2)*(2*cov+.03**2))/
            ((mu_a.square()+mu_b.square()+.01**2)*(va+vb+.03**2))).mean()


def look_at(center,direction):
    forward=np.array(direction,dtype=float);forward/=np.linalg.norm(forward)
    up=np.array([0,0,1.]) if abs(forward[2])<.9 else np.array([0,1.,0])
    right=np.cross(forward,up);right/=np.linalg.norm(right)
    down=np.cross(forward,right)
    pose=np.eye(4);pose[:3,:3]=np.column_stack([right,down,forward]);pose[:3,3]=center
    return pose


def load_capture(output,width):
    coverage=json.loads((output/'coverage.json').read_text())
    capture=json.loads((output/'result.json').read_text())
    if not (capture['success'] and capture['source_unchanged'] and coverage['passed']
            and coverage['complete_fraction']==1. and coverage['missing_samples']==0
            and coverage['required_fraction']==1.):
        raise RuntimeError('Incomplete map: training prohibited')
    # Hash only the actual training inputs. Offline audit geometry is not read.
    for name in ('rgb_pt.pcd','dataset/frames.json','camera_input.json'):
        if sha(output/name)!=coverage['input_sha256'][name]:raise RuntimeError('Training input changed: '+name)
    verify_sources(output,capture)
    if (output/'source_sessions.json').exists():
        from multisession import verify_bundle
        verify_bundle(output)
    rows=json.loads((output/'dataset/frames.json').read_text())
    info=json.loads((output/'camera_input.json').read_text())
    if any(abs(d)>1e-8 for d in info['D']):raise ValueError('Undistort real images and update K before training')
    height=round(info['height']*width/info['width'])
    k=np.array(info['K'],dtype=np.float32).reshape(3,3)
    k[0]*=width/info['width'];k[1]*=height/info['height']
    images=[];poses=[];stamps=[]
    for row in rows:
        path=output/'dataset'/row['file_path']
        if sha(path)!=row['sha256']:raise RuntimeError('Image checksum mismatch: '+str(path))
        images.append(path);pose=np.array(row['c2w'],dtype=np.float32)
        if not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-4):raise ValueError('Invalid rotation')
        poses.append(pose);stamps.append(row['stamp_ns'])
    if len(rows)<100:raise RuntimeError('Too few inspection views')
    xyz,rgb=read_pcd(output/'rgb_pt.pcd')
    # Deterministic measured-point voxel selection, no geometry interpolation.
    _,indices=np.unique(np.floor(xyz/.055).astype(np.int64),axis=0,return_index=True)
    xyz=xyz[np.sort(indices)].astype(np.float32)
    train=np.flatnonzero(np.arange(len(rows))%8!=0);test=np.flatnonzero(np.arange(len(rows))%8==0)
    return CapturedImages(images,width,height),np.stack(poses),k,xyz,train,test,stamps


def initial_colours(xyz,images,poses,k,train):
    """Use training images only; held-out appearance is not used for initialization."""
    height,width=images.shape[1:3]
    sums=np.zeros((len(xyz),3),np.float64);counts=np.zeros(len(xyz),np.int32)
    for i in train[::max(1,len(train)//240)]:
        cam=(xyz-poses[i,:3,3])@poses[i,:3,:3]
        z=cam[:,2];uv=cam[:,:2]/np.maximum(z[:,None],.001)
        uv=uv@np.diag([k[0,0],k[1,1]])+np.array([k[0,2],k[1,2]])
        good=(z>.4)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
        ids=np.flatnonzero(good);pix=uv[ids].astype(int);flat=pix[:,1]*width+pix[:,0]
        closest=np.full(height*width,np.inf,dtype=np.float32)
        np.minimum.at(closest,flat,z[ids]);visible=z[ids]<=closest[flat]+.10
        ids=ids[visible];pix=pix[visible]
        sums[ids]+=images[i][pix[:,1],pix[:,0]]/255.;counts[ids]+=1
    color=np.full((len(xyz),3),.5,dtype=np.float32)
    good=counts>0;color[good]=sums[good]/counts[good,None]
    return np.clip(color,.001,.999),int(good.sum())


def init_splats(xyz,color,device):
    tree=cKDTree(xyz);dist,idx=tree.query(xyz,k=12)
    offsets=xyz[idx]-xyz[:,None]
    covariance=np.einsum('nki,nkj->nij',offsets,offsets)/12
    _,rotation=np.linalg.eigh(covariance)
    rotation[:,:,0]*=np.linalg.det(rotation)[:,None]
    quats=Rotation.from_matrix(rotation).as_quat()[:,[3,0,1,2]].astype(np.float32)
    tangent=np.clip(np.sqrt(np.mean(dist[:,1:4]**2,axis=1)),.018,.12)
    scales=np.column_stack([np.minimum(tangent*.15,.008),tangent,tangent]).astype(np.float32)
    values=dict(means=xyz,scales=np.log(scales),quats=quats,
                opacities=np.full(len(xyz),math.log(.4/.6),np.float32),
                sh0=((color-.5)/C0)[:,None,:],shN=np.zeros((len(xyz),15,3),np.float32))
    return nn.ParameterDict({name:nn.Parameter(torch.tensor(a,device=device)) for name,a in values.items()}),tree


def render(params,c2w,k,width,height,degree=3,depth=False):
    rgb,alpha,meta=rasterization(means=params['means'],quats=F.normalize(params['quats'],dim=-1),
        scales=params['scales'].exp(),opacities=params['opacities'].sigmoid(),
        colors=torch.cat([params['sh0'],params['shN']],dim=1),
        viewmats=torch.linalg.inv(c2w)[None],Ks=k[None],width=width,height=height,
        near_plane=.15,far_plane=30.,sh_degree=degree,packed=False,
        render_mode='RGB+ED' if depth else 'RGB')
    return rgb[0],alpha[0],meta


def depth_prior(xyz,pose,k,width,height):
    cam=(xyz-pose[:3,3])@pose[:3,:3];z=cam[:,2]
    uv=cam[:,:2]/np.maximum(z[:,None],.001)
    uv=uv@np.diag([k[0,0],k[1,1]])+np.array([k[0,2],k[1,2]])
    good=(z>.4)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    pix=uv[good].astype(int);closest=np.full(width*height,np.inf,np.float32)
    np.minimum.at(closest,pix[:,1]*width+pix[:,0],z[good])
    return closest.reshape(height,width)


def run(args):
    output=Path(args.capture).resolve();folder=output/'gsplat';folder.mkdir(exist_ok=True)
    if (folder/'model.pt').exists():raise RuntimeError('Model already exists; do not overwrite an experiment')
    if not torch.cuda.is_available():raise RuntimeError('CUDA unavailable')
    fixed_input_bytes=sum(p.stat().st_size for p in output.rglob('*') if p.is_file() and folder not in p.parents)
    def storage_check(extra=0):
        size=fixed_input_bytes+sum(p.stat().st_size for p in folder.rglob('*') if p.is_file())+extra
        if size>args.max_run_gib*2**30:raise RuntimeError('Run storage budget exceeded; training stopped without deleting inputs')
        return size
    storage_check()
    torch.set_num_threads(2);torch.manual_seed(19);np.random.seed(19)
    total_gpu=torch.cuda.get_device_properties(0).total_memory
    torch.cuda.set_per_process_memory_fraction(min(1.,args.gpu_memory_gib*2**30/total_gpu),0)
    device='cuda';start=time.monotonic()
    images,poses,k,xyz,train,test,stamps=load_capture(output,args.width)
    write(folder/'split.json',dict(train_indices=train.tolist(),test_indices=test.tolist(),
                                  policy='Every eighth exported frame held out; init appearance from training RGB only'))
    colour,coloured=initial_colours(xyz,images,poses,k,train)
    params,tree=init_splats(xyz,colour,device);initial_mean=params['means'].detach().clone()
    rates=dict(means=1.6e-4,scales=5e-3,quats=1e-3,opacities=.03,sh0=.0025,shN=.000125)
    optimizers={name:torch.optim.Adam([param],lr=rates[name],eps=1e-15) for name,param in params.items()}
    strategy=DefaultStrategy(refine_start_iter=500,refine_stop_iter=min(5500,args.steps-1000),
        refine_every=100,reset_every=10000,prune_opa=.001,prune_scale3d=.15,
        grow_grad2d=.0003,verbose=True)
    strategy.check_sanity(params,optimizers);state=strategy.initialize_state(scene_scale=6.)
    tk=torch.tensor(k,device=device);tp=torch.tensor(poses,device=device)
    height,width=images.shape[1:3];depths=OrderedDict();history=[];anchors=None
    comparison_indices=set(test[np.linspace(0,len(test)-1,min(8,len(test))).astype(int)].tolist())
    initial_metrics=[]
    with torch.no_grad():
        for index in sorted(comparison_indices):
            prediction,_,_=render(params,tp[index],tk,width,height,degree=0)
            target=torch.tensor(images[index],device=device,dtype=torch.float32)/255.
            initial_metrics.append(dict(index=int(index),psnr=float(-10*torch.log10(F.mse_loss(prediction,target)))))
    write(folder/'initial_heldout_metrics.json',dict(frames=initial_metrics,
        psnr_mean=float(np.mean([m['psnr'] for m in initial_metrics])),scope='Same eight distributed held-out views, before optimization'))
    log=(folder/'training.jsonl').open('w',buffering=1)
    source_files=[Path(__file__),ROOT/'integration/roomgs/geometry.py',ROOT/'integration/roomgs/audit_model.py',ROOT/'integration/roomgs/image_sequence.py']
    if (output/'source_sessions.json').exists():source_files += [ROOT/'integration/roomgs/multisession.py',ROOT/'integration/roomgs/registration.py']
    source_hashes={str(p):sha(p) for p in source_files}
    write(folder/'provenance.json',dict(gsplat_revision='937e29912570c372bed6747a5c9bf85fed877bae',
        torch=torch.__version__,cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(0),
        input_map_sha256=sha(output/'rgb_pt.pcd'),input_frames_sha256=sha(output/'dataset/frames.json'),
        source_sha256=source_hashes,steps=args.steps,width=width,height=height,
        method='gsplat differentiable SH 3DGS + DefaultStrategy; observed LiDAR depth/nearest-surface priors',
        truth_geometry_used_for_training=False,initial_points=len(xyz),initial_colour_points=coloured))
    rng=np.random.RandomState(91);last_count=len(params['means']);order=[]
    for step in range(args.steps):
        if not order:order=rng.permutation(train).tolist()
        index=order.pop()
        target=torch.tensor(images[index],device=device,dtype=torch.float32)/255.
        degree=min(3,step//1500)
        prediction,alpha,meta=render(params,tp[index],tk,width,height,degree,depth=True)
        strategy.step_pre_backward(params,optimizers,state,step,meta)
        rgb=prediction[...,:3]
        l1=F.l1_loss(rgb,target);structural=1-similarity(rgb,target)
        if index not in depths:depths[index]=depth_prior(xyz,poses[index],k,width,height)
        depth=depths.pop(index);depths[index]=depth
        while len(depths)>32:depths.popitem(last=False)
        prior=torch.tensor(depth,device=device)
        valid=torch.isfinite(prior)
        depth_loss=(torch.abs(prediction[...,3][valid]-prior[valid])/prior[valid].clamp_min(.4)).mean()
        if anchors is None or len(anchors)!=len(params['means']) or step%100==0:
            _,nearest=tree.query(params['means'].detach().cpu().numpy(),workers=4)
            anchors=torch.tensor(xyz[nearest],device=device)
        lidar_loss=F.smooth_l1_loss(params['means'],anchors,beta=.04)
        loss=.8*l1+.2*structural+.05*depth_loss+.10*lidar_loss
        if not torch.isfinite(loss):raise RuntimeError('Non-finite training loss')
        loss.backward()
        for optimizer in optimizers.values():optimizer.step();optimizer.zero_grad(set_to_none=True)
        with torch.no_grad():
            params['scales'].clamp_(math.log(.002),math.log(.18))
            params['opacities'].clamp_(-10,10)
        strategy.step_post_backward(params,optimizers,state,step,meta,packed=False)
        # Bound density by stopping further refinement, without deleting geometry.
        if len(params['means'])>450000:strategy.refine_stop_iter=step
        if step%100==0 or step==args.steps-1:
            storage_check()
            row=dict(step=step+1,loss=float(loss.detach()),l1=float(l1.detach()),
                depth=float(depth_loss.detach()),gaussians=len(params['means']),
                wall_seconds=time.monotonic()-start,gpu_peak_mib=torch.cuda.max_memory_allocated()/2**20)
            history.append(row);log.write(json.dumps(row)+'\n');print(json.dumps(row),flush=True)
        if step and step%3000==0:
            storage_check(sum(p.numel()*p.element_size() for p in params.values())+65536)
            torch.save(dict(step=step,params={n:p.detach().cpu() for n,p in params.items()}),folder/'checkpoint_latest.tmp')
            os.replace(folder/'checkpoint_latest.tmp',folder/'checkpoint_latest.pt')
    log.close()
    torch.cuda.synchronize()
    # Export actual learned Gaussian parameters, not a renamed RGB PCD.
    learned={name:p.detach().cpu() for name,p in params.items()}
    torch.save(dict(params=learned,K=k,width=width,height=height,steps=args.steps),folder/'model.pt')
    export_splats(**{name:params[name].detach() for name in ('means','scales','quats','opacities','sh0','shN')},
                  format='ply',save_to=str(folder/'model.ply'))
    np.savez_compressed(folder/'gaussians.npz',**{name:p.numpy() for name,p in learned.items()})
    metrics=[];comparisons=[]
    with torch.no_grad():
        for i in test:
            prediction,alpha,_=render(params,tp[i],tk,width,height)
            target=torch.tensor(images[i],device=device,dtype=torch.float32)/255.
            rgb=prediction.clamp(0,1);mse=F.mse_loss(rgb,target)
            metrics.append(dict(index=int(i),psnr=float(-10*torch.log10(mse)),
                                ssim=float(similarity(rgb,target)),mean_alpha=float(alpha.mean())))
            if int(i) in comparison_indices:
                rendered=(rgb.cpu().numpy()*255).astype(np.uint8)
                comparisons.append(np.concatenate([images[i],rendered],axis=1))
    write(folder/'heldout_metrics.json',dict(count=len(metrics),psnr_mean=float(np.mean([m['psnr'] for m in metrics])),
        ssim_mean=float(np.mean([m['ssim'] for m in metrics])),frames=metrics,
        scope='held-out RGB; calibrated poses/map geometry were estimated from the whole acquisition; no external ground-truth geometry metric'))
    Image.fromarray(np.concatenate(comparisons,axis=0)).save(folder/'heldout_comparison.png')
    # Use a measured camera station, since the geometric room midpoint may
    # lie inside the central rack. Clearance uses measured LiDAR only.
    positions=poses[:,:3,3];middle=np.median(positions,axis=0)
    middle[:2]=(positions.min(axis=0)[:2]+positions.max(axis=0)[:2])/2
    clearance=tree.query(positions,workers=2)[0]
    score=np.linalg.norm(positions[:,:2]-middle[:2],axis=1)+.5*np.abs(positions[:,2]-middle[2])
    score[clearance<.35]=np.inf
    center_index=int(np.argmin(score)) if np.isfinite(score).any() else int(np.argmax(clearance))
    center=positions[center_index].copy()
    write(folder/'render_center.json',dict(actual_capture_index=center_index,center=center.tolist(),
        measured_map_clearance_m=float(clearance[center_index]),
        policy='Actual measured camera centre near trajectory midpoint with >=35cm observed-map clearance when available; no CAD'))
    six=[];six_meta=[]
    with torch.no_grad():
        for name,direction in [('east',[1,0,0]),('north',[0,1,0]),('west',[-1,0,0]),
                               ('south',[0,-1,0]),('ceiling',[0,0,1]),('floor',[0,0,-1])]:
            pose=look_at(center,direction)
            rgb,alpha,_=render(params,torch.tensor(pose,device=device,dtype=torch.float32),tk,width,height)
            picture=Image.fromarray((rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8))
            ImageDraw.Draw(picture).text((12,12),name,fill='white');picture.save(folder/(name+'.png'))
            six.append(np.asarray(picture));six_meta.append(dict(name=name,c2w=pose.tolist(),
                                    alpha_over_95_fraction=float((alpha>.95).float().mean())))
    Image.fromarray(np.concatenate([np.concatenate(six[:3],axis=1),np.concatenate(six[3:],axis=1)],axis=0)).save(folder/'room_six_views.png')
    write(folder/'six_view_cameras.json',six_meta)
    import imageio.v2 as imageio
    video=imageio.get_writer(str(folder/'walkthrough.mp4'),fps=20,macro_block_size=1)
    selected=np.linspace(0,len(poses)-2,180).astype(int)
    with torch.no_grad():
        for j,i in enumerate(selected):
            # New intermediate pose, not an input image replay.
            pose=np.eye(4);pose[:3,3]=(poses[i,:3,3]+poses[i+1,:3,3])/2
            pose[:3,:3]=Slerp([0,1],Rotation.from_matrix(poses[[i,i+1],:3,:3]))([.5]).as_matrix()[0]
            rgb,_,_=render(params,torch.tensor(pose,device=device,dtype=torch.float32),tk,width,height)
            video.append_data((rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8))
    video.close()
    unchanged=all(sha(name)==digest for name,digest in source_hashes.items())
    report=dict(trained=True,steps=args.steps,gaussians=len(params['means']),
        source_unchanged=unchanged,wall_seconds=time.monotonic()-start,
        peak_gpu_mib=torch.cuda.max_memory_allocated()/2**20,model_sha256=sha(folder/'model.ply'),
        initial_l1=history[0]['l1'],final_l1=history[-1]['l1'],heldout_views=len(test),
        train_view_count=len(train),heldout_psnr_mean=float(np.mean([m['psnr'] for m in metrics])),
        comparison_initial_psnr=float(np.mean([m['psnr'] for m in initial_metrics])),
        comparison_final_psnr=float(np.mean([m['psnr'] for m in metrics if m['index'] in comparison_indices])),
        scope='Actual offline CUDA gsplat training after the 100% acquisition map gate')
    write(folder/'training_result.json',report)
    report.update(storage_bytes=storage_check(),storage_budget_gib=args.max_run_gib,
                  depth_cache_limit_frames=32,image_loading='single-frame lazy JPEG/PNG decode',gpu_memory_budget_gib=args.gpu_memory_gib)
    write(folder/'training_result.json',report)
    if not unchanged:raise RuntimeError('Training source changed during execution')
    print(json.dumps(report,indent=2),flush=True)
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',required=True)
    parser.add_argument('--steps',type=int,default=12000)
    parser.add_argument('--width',type=int,default=480)
    parser.add_argument('--gpu-memory-gib',type=float,default=6.)
    parser.add_argument('--max-run-gib',type=float,default=8.)
    args=parser.parse_args()
    capture=run(args)
    # Separate, post-training auditor. Its reference data never enters the loss.
    subprocess.run([sys.executable,str(ROOT/'integration/roomgs/audit_model.py'),'--capture',str(capture)],check=True)
