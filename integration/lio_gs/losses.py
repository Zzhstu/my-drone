"""Valid-pixel supervision; dynamic RGB never becomes a black training target."""
import torch
import torch.nn.functional as F


def ssim_map(a,b):
    # Explicit NCHW avoids a locally reproduced CUDA channels-last pooling
    # backward error (ignored pixels received gradients on this Torch/GPU).
    a=a.permute(2,0,1)[None].contiguous();b=b.permute(2,0,1)[None].contiguous()
    ma=F.avg_pool2d(a,11,1,5);mb=F.avg_pool2d(b,11,1,5)
    va=F.avg_pool2d(a*a,11,1,5)-ma.square();vb=F.avg_pool2d(b*b,11,1,5)-mb.square()
    cov=F.avg_pool2d(a*b,11,1,5)-ma*mb
    return (((2*ma*mb+.01**2)*(2*cov+.03**2))/((ma.square()+mb.square()+.01**2)*(va+vb+.03**2))).mean(1)[0]


def valid_windows(mask):
    """Only SSIM windows whose in-image pixels are all valid may supervise."""
    m=mask.to(dtype=torch.float32)[None,None]
    support=F.avg_pool2d(torch.ones_like(m),11,1,5)
    return (F.avg_pool2d(m,11,1,5)>=support-1e-7)[0,0]


def photometric(prediction,target,mask):
    if not bool(mask.any()):raise ValueError('No valid image pixels')
    l1=(prediction[mask]-target[mask]).abs().mean()
    windows=valid_windows(mask);scores=ssim_map(prediction,target)
    structural=1-scores[windows].mean() if bool(windows.any()) else prediction.sum()*0
    return l1,structural


def image_metrics(prediction,target,mask):
    mse=((prediction[mask]-target[mask])**2).mean()
    windows=valid_windows(mask);score=ssim_map(prediction,target)
    return dict(psnr=float(-10*torch.log10(mse.clamp_min(1e-12))),
        ssim=float(score[windows].mean()) if bool(windows.any()) else None,
        valid_pixels=int(mask.sum()),ssim_windows=int(windows.sum()))


def edge_loss(prediction,target,mask):
    """Match measured RGB differences, excluding pairs crossing a mask edge."""
    horizontal=mask[:,1:]&mask[:,:-1];vertical=mask[1:]&mask[:-1]
    dx=(prediction[:,1:]-prediction[:,:-1])-(target[:,1:]-target[:,:-1])
    dy=(prediction[1:]-prediction[:-1])-(target[1:]-target[:-1])
    zero=prediction.sum()*0
    return .5*((dx[horizontal].abs().mean() if bool(horizontal.any()) else zero)+
               (dy[vertical].abs().mean() if bool(vertical.any()) else zero))
