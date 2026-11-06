"""Pinned gsplat 1.5.3 rendering shared by detail training and full-SH viewer."""
import torch
import torch.nn.functional as F
from gsplat import rasterization


def render(params,c2w,k,width,height,degree=3,depth=False,eps2d=.3,absgrad=False):
    rgb,alpha,meta=rasterization(means=params['means'],quats=F.normalize(params['quats'],dim=-1),
        scales=params['scales'].exp(),opacities=params['opacities'].sigmoid(),
        colors=torch.cat([params['sh0'],params['shN']],dim=1),
        viewmats=torch.linalg.inv(c2w)[None],Ks=k[None],width=width,height=height,
        near_plane=.15,far_plane=30.,sh_degree=degree,packed=False,eps2d=eps2d,absgrad=absgrad,
        render_mode='RGB+ED' if depth else 'RGB')
    return rgb[0],alpha[0],meta
