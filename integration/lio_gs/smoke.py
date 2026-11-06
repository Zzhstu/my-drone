"""Actual gsplat CUDA backward masking and bounded duplicate/split checks."""
import json
from pathlib import Path
import sys
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'integration/roomgs'))
from integration.lio_gs.train import BudgetStrategy,base
from integration.lio_gs.dataset import load
from integration.lio_gs.losses import photometric,edge_loss
from integration.lio_gs.rendering import render


def main(output):
    torch.set_num_threads(2);torch.manual_seed(19)
    data=load(ROOT/'lio_dataset_runs/warehouse20_20261102_capture',128)
    xyz=data['xyz'][::4];params,_=base.init_splats(xyz,np.full_like(xyz,.5),'cuda')
    i=int(data['train'][len(data['train'])//2]);pose=torch.tensor(data['poses'][i],device='cuda');k=torch.tensor(data['K'],device='cuda')
    prediction,_,_=base.render(params,pose,k,data['width'],data['height'],degree=0)
    target=torch.tensor(data['images'][i],device='cuda',dtype=torch.float32)/255
    mask=torch.ones(data['height'],data['width'],dtype=torch.bool,device='cuda');mask[30:65,40:90]=False
    l1,ssim=photometric(prediction,target,mask);loss=.8*l1+.2*ssim
    pixel_grad=torch.autograd.grad(loss,prediction,retain_graph=True)[0]
    grad=torch.autograd.grad(loss,params['means'],retain_graph=True)[0]
    changed=target.clone();changed[~mask]=37
    a,b=photometric(prediction,changed,mask);other=.8*a+.2*b
    other_grad=torch.autograd.grad(other,params['means'])[0]
    difference=float((grad-other_grad).abs().max())
    print(json.dumps(dict(loss_difference=abs(float(loss-other)),gradient_difference=difference,
        gradient_max=float(grad.abs().max()),finite=bool(torch.isfinite(grad).all()),
        ignored_pixel_gradient=float(pixel_grad[~mask].abs().max()))),flush=True)
    # CUDA rasterization backward uses atomic reductions, so repeated calls
    # are compared with float32 tolerances rather than bitwise equality.
    assert torch.allclose(grad,other_grad,atol=1e-6,rtol=1e-4) and torch.isfinite(grad).all()
    assert float(pixel_grad[~mask].abs().max())==0 and float(grad.abs().max())>0
    ordinary,_,_=render(params,pose,k,data['width'],data['height'],degree=0)
    assert torch.equal(ordinary,prediction)
    detail,_,meta=render(params,pose,k,data['width'],data['height'],degree=0,eps2d=.1,absgrad=True)
    opts={n:torch.optim.Adam([v],lr=.001) for n,v in params.items()}
    detail_strategy=BudgetStrategy(25000,absgrad=True)
    detail_state=detail_strategy.initialize_state(scene_scale=6.)
    detail_strategy.step_pre_backward(params,opts,detail_state,1,meta)
    ea=edge_loss(detail,target,mask);eb=edge_loss(detail,changed,mask)
    assert float(ea)==float(eb)
    ea.backward();detail_strategy.step_post_backward(params,opts,detail_state,1,meta,packed=False)
    assert torch.isfinite(meta['means2d'].absgrad).all() and float(meta['means2d'].absgrad.abs().max())>0
    cases=[]
    for large in [False,True]:
        p,_=base.init_splats(xyz[:100],np.full((100,3),.5,np.float32),'cuda')
        with torch.no_grad():p['scales'].fill_(float(np.log(.1 if large else .01)))
        opts={n:torch.optim.Adam([v],lr=.001) for n,v in p.items()}
        state=dict(scene_scale=6.,grad2d=torch.ones(100,device='cuda'),count=torch.ones(100,device='cuda'),radii=None)
        strategy=BudgetStrategy(105);duplicates,splits=strategy._grow_gs(p,opts,state,1000)
        assert len(p['means'])==105
        assert strategy._grow_gs(p,opts,state,1000)==(0,0)
        cases.append(dict(large=large,duplicates=duplicates,splits=splits,final_count=len(p['means'])))
    report=dict(passed=True,backend='Actual gsplat 1.5.3 CUDA rasterization/backward',detail_absgrad_backward=True,default_render_bitwise_equal=True,
        poisoned_mask_rgb_loss_difference=abs(float(loss-other)),parameter_gradient_max_difference=float((grad-other_grad).abs().max()),
        ignored_pixel_gradient_max=float(pixel_grad[~mask].abs().max()),duplicate_split_budget_tests=cases,
        verified_images=len(data['rows']),verified_measured_points=len(data['xyz']),
        scope='Synthetic ignored region on actual RGB/pose/cloud; confirms masking and gradients, not dynamic-person removal quality')
    Path(output).parent.mkdir(parents=True,exist_ok=True);Path(output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)


if __name__=='__main__':main(sys.argv[1])
