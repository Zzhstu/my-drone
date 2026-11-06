"""Real CUDA rasterization + gradient validation before scene training."""
import json
import fcntl
import sys
from pathlib import Path
_backend_lock=(Path(sys.executable).resolve().parents[1]/'.gsplat_backend.lock').open('a')
fcntl.flock(_backend_lock,fcntl.LOCK_EX)
import torch
from torch import nn
from gsplat import rasterization

torch.manual_seed(19)
means=nn.Parameter(torch.tensor([[0.,0.,2.],[.15,0,2.1]],device='cuda'))
scale=nn.Parameter(torch.full((2,3),.08,device='cuda'))
quat=nn.Parameter(torch.tensor([[1.,0,0,0],[1.,0,0,0]],device='cuda'))
alpha=nn.Parameter(torch.full((2,),.7,device='cuda'))
color=nn.Parameter(torch.tensor([[.8,.1,.2],[.1,.8,.2]],device='cuda'))
k=torch.tensor([[[30.,0,16],[0,30,16],[0,0,1]]],device='cuda')
rgb,a,meta=rasterization(means,quat,scale,alpha,color,
    torch.eye(4,device='cuda')[None],k,32,32,render_mode='RGB+ED',packed=False)
loss=rgb[...,:3].square().mean()+a.mean()*.1
loss.backward();torch.cuda.synchronize()
params=[means,scale,quat,alpha,color]
assert torch.isfinite(rgb).all() and float(a.max())>0
assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in params)
print(json.dumps(dict(success=True,torch=torch.__version__,cuda=torch.version.cuda,
    gpu=torch.cuda.get_device_name(),rgb_shape=list(rgb.shape),loss=float(loss),
    max_alpha=float(a.max()),mean_gradient_norm=float(means.grad.norm()))),flush=True)
