"""Same-input, same-pose held-out appearance and measured-edge comparison."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import cv2
from PIL import Image,ImageDraw
import torch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from integration.lio_gs.dataset import load,sha
from integration.lio_gs.losses import image_metrics
from integration.lio_gs.rendering import render


def detected_markers(image,detector,min_side=20):
    corners,ids,_=detector.detectMarkers(cv2.cvtColor(image,cv2.COLOR_RGB2GRAY))
    result=[]
    for polygon,identity in zip(corners,ids.flatten() if ids is not None else []):
        polygon=polygon.reshape(4,2)
        if np.linalg.norm(polygon-np.roll(polygon,1,axis=0),axis=1).min()>=min_side:
            result.append((int(identity),polygon))
    return result


def marker_retention(reference,detected):
    # The warehouse repeats the same ID. Match positions as well as IDs,
    # one-to-one, without inventing identities for unreadable render patches.
    candidates=[]
    for i,(identity,polygon) in enumerate(reference):
        diagonal=np.linalg.norm(polygon[0]-polygon[2])
        for j,(other,points) in enumerate(detected):
            if identity!=other:continue
            error=float(np.sqrt(np.mean(np.sum((polygon-points)**2,axis=1))))
            if error<=.20*diagonal:candidates.append((error,i,j))
    used_reference=set();used_detected=set();errors=[]
    for error,i,j in sorted(candidates):
        if i in used_reference or j in used_detected:continue
        used_reference.add(i);used_detected.add(j);errors.append(error)
    return dict(reference_markers=len(reference),matched_markers=len(errors),
                matched_corner_rms_sum_px=float(sum(errors)))


def measured_edges(prediction,target,mask):
    valid=mask[:-1,:-1]&mask[:-1,1:]&mask[1:,:-1]
    dx=target[:-1,1:]-target[:-1,:-1];dy=target[1:,:-1]-target[:-1,:-1]
    luminance=torch.tensor([.2126,.7152,.0722],device=target.device)
    magnitude=((dx*luminance).sum(-1).square()+(dy*luminance).sum(-1).square()).sqrt()
    threshold=max(.02,float(torch.quantile(magnitude[valid],.9)))
    strong=valid&(magnitude>=threshold)
    error=.5*((prediction[:-1,1:]-prediction[:-1,:-1]-dx).abs().mean(-1)+
               (prediction[1:,:-1]-prediction[:-1,:-1]-dy).abs().mean(-1))
    return dict(edge_l1=float(error[valid].mean()),
        strong_edge_l1=float(error[strong].mean()) if bool(strong.any()) else None,
        strong_edge_pixels=int(strong.sum()),strong_edge_threshold=threshold)


def run(args):
    torch.set_num_threads(2);torch.cuda.set_per_process_memory_fraction(.25)
    roots=[Path(args.baseline).resolve(),Path(args.detail).resolve()]
    configs=[json.loads((r/'config.json').read_text()) for r in roots]
    sources=[json.loads((r/'source_dataset.json').read_text()) for r in roots]
    splits=[json.loads((r/'gsplat/split.json').read_text()) for r in roots]
    if sources[0]['input_sha256']!=sources[1]['input_sha256'] or splits[0]!=splits[1]:raise ValueError('Comparison inputs/split differ')
    if configs[0]['width']!=configs[1]['width']:raise ValueError('Comparison resolutions differ')
    partial=bool(configs[0].get('allow_partial_map') and configs[1].get('allow_partial_map'))
    data=load(sources[0]['path'],configs[0]['width'],configs[0]['mask_dir'],configs[0]['mask_dilate'],allow_partial_geometry=partial)
    if data['fingerprints']!=sources[0]['input_sha256']:raise ValueError('Comparison data changed')
    hashes=[sha(r/'gsplat/model.pt') for r in roots]
    checkpoints=[torch.load(r/'gsplat/model.pt',map_location='cpu',weights_only=False) for r in roots]
    parameters=[{n:v.cuda() for n,v in c['params'].items()} for c in checkpoints]
    selected=set(data['test'][np.linspace(0,len(data['test'])-1,8).astype(int)].tolist())
    metrics=[[],[]];full=[];crops=[];crop_choices=[]
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)) if args.markers else None
    k=torch.tensor(data['K'],device='cuda');poses=torch.tensor(data['poses'],device='cuda')
    header_height=36
    def panel(images,label):
        joined=Image.fromarray(np.concatenate(images,axis=1));panel=Image.new('RGB',(joined.width,joined.height+header_height),'white')
        panel.paste(joined,(0,header_height));ImageDraw.Draw(panel).text((8,10),label,fill='black');return np.array(panel)
    with torch.no_grad():
        for index in data['test']:
            i=int(index);image=data['images'][i];target=torch.tensor(image,device='cuda',dtype=torch.float32)/255
            mask=torch.tensor(data['masks'][i],device='cuda');pictures=[image]
            reference=detected_markers(image,detector) if detector else None
            for j,params in enumerate(parameters):
                rgb,_,_=render(params,poses[i],k,data['width'],data['height'],eps2d=checkpoints[j].get('eps2d',.3));rgb=rgb.clamp(0,1)
                row=dict(index=i,**image_metrics(rgb,target,mask),**measured_edges(rgb,target,mask))
                picture=(rgb.cpu().numpy()*255).astype(np.uint8) if detector or i in selected else None
                if detector:row['markers']=marker_retention(reference,detected_markers(picture,detector))
                metrics[j].append(row)
                if i in selected:pictures.append(picture)
            if i in selected:
                full.append(panel(pictures,'Frame %d | Captured RGB / Baseline full SH / Detail full SH'%i))
                gray=image.astype(float).mean(-1);gradient=np.zeros_like(gray)
                gradient[:-1,:-1]=np.abs(gray[:-1,1:]-gray[:-1,:-1])+np.abs(gray[1:,:-1]-gray[:-1,:-1])
                # Select using measured RGB only; never pick the largest model gain.
                size=128;choices=[(gradient[y:y+size,x:x+size].sum(),x,y)
                    for y in range(0,image.shape[0]-size+1,32) for x in range(0,image.shape[1]-size+1,32)]
                _,x,y=max(choices);crop_choices.append(dict(index=i,x=x,y=y,size=size,selection='Maximum measured RGB gradient sum on fixed 32px grid'))
                tiles=[np.array(Image.fromarray(p[y:y+size,x:x+size]).resize((256,256),Image.Resampling.NEAREST)) for p in pictures]
                crops.append(panel(tiles,'Frame %d crop (%d,%d), 2x pixels | RGB / Baseline / Detail'%(i,x,y)))
    summary=[]
    for rows in metrics:
        mean={key:float(np.mean([r[key] for r in rows if r[key] is not None])) for key in ('psnr','ssim','edge_l1','strong_edge_l1')}
        if detector:
            counts={key:sum(r['markers'][key] for r in rows) for key in ('reference_markers','matched_markers','matched_corner_rms_sum_px')}
            counts['retention_fraction']=counts['matched_markers']/max(1,counts['reference_markers'])
            counts['mean_matched_corner_rms_px']=counts['matched_corner_rms_sum_px']/max(1,counts['matched_markers'])
            mean['markers']=counts
        summary.append(mean)
    cached=json.loads((roots[0]/'gsplat/heldout_metrics.json').read_text())
    if abs(summary[0]['psnr']-cached['psnr_mean'])>1e-4:raise ValueError('Baseline independent re-render does not reproduce recorded PSNR')
    if not all(sha(r/'gsplat/model.pt')==h for r,h in zip(roots,hashes)):raise ValueError('Comparison model changed')
    destination=Path(args.output);destination.mkdir(parents=True,exist_ok=True)
    Image.fromarray(np.concatenate(full)).save(destination/'full_comparison.png')
    Image.fromarray(np.concatenate(crops)).save(destination/'detail_crops.png')
    report=dict(count=len(data['test']),width=data['width'],height=data['height'],baseline=summary[0],detail=summary[1],
        psnr_delta=summary[1]['psnr']-summary[0]['psnr'],ssim_delta=summary[1]['ssim']-summary[0]['ssim'],
        strong_edge_error_reduction_fraction=1-summary[1]['strong_edge_l1']/summary[0]['strong_edge_l1'],
        frame_improvements=dict(psnr=sum(b['psnr']>a['psnr'] for a,b in zip(*metrics)),
            strong_edges=sum(b['strong_edge_l1']<a['strong_edge_l1'] for a,b in zip(*metrics) if a['strong_edge_l1'] is not None)),
        selected_indices=sorted(selected),crop_choices=crop_choices,model_paths=[str(r) for r in roots],model_pt_sha256=hashes,
        source_sha256={str(Path(__file__)):sha(__file__),str(ROOT/'integration/lio_gs/rendering.py'):sha(ROOT/'integration/lio_gs/rendering.py')},
        protocol='%d identical held-out RGB, original fixed LIO poses/K, native resolution, full SH3, each trained eps2d; no heldout fitting or image sharpening'%len(data['test']),
        edge_protocol='Mean valid adjacent RGB gradient absolute error; strong edges threshold=max(0.02, measured-image gradient 90th percentile); lower is better',
        marker_protocol='Supplemental only: fixed OpenCV DICT_4X4_50, minimum side 20px; match same ID and original-image corners within 0.20 diagonal, one-to-one. Repeated observations, not unique physical boxes; not pose or map truth.' if detector else None,
        scope='Same-capture appearance holdout; geometry/poses use complete acquisition, not external truth accuracy; added compute/configuration confounded, not an algorithm ablation',
        frames={'baseline':metrics[0],'detail':metrics[1]})
    (destination/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('baseline');p.add_argument('detail');p.add_argument('output')
    p.add_argument('--markers',action='store_true',help='Supplemental repeated ArUco observation retention; only for the marked static warehouse')
    run(p.parse_args())
