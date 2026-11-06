"""Image-only learned person masks; no simulator poses/labels are imported."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import torch
from ultralytics import YOLO,__version__,settings
ROOT=Path(__file__).resolve().parents[2]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(args):
    root=Path(args.dataset).resolve();folder=root/'dataset';rows=json.loads((folder/'frames.json').read_text())
    destination=folder/'masks';core=folder/'person_core';destination.mkdir(exist_ok=False);core.mkdir()
    preview=root/'mask_preview';preview.mkdir();torch.set_num_threads(2)
    settings.update({'sync':False})
    model=YOLO(args.weights);weight_hash=sha(args.weights);records=[];started=time.monotonic()
    for i,row in enumerate(rows):
        path=folder/row['file_path']
        if sha(path)!=row['sha256']:raise ValueError('RGB changed during image-only mask generation')
        result=model.predict(str(path),classes=[0],conf=.15,imgsz=960,device=0,retina_masks=True,verbose=False)[0]
        h,w=result.orig_shape;ignore=np.zeros((h,w),np.uint8);strong=np.zeros_like(ignore)
        boxes=result.boxes.xyxy.cpu().numpy();confidence=result.boxes.conf.cpu().numpy()
        if len(boxes) and result.masks is None:raise ValueError('Person boxes exist without segmentation masks')
        if result.masks is not None:
            for mask,score,box in zip(result.masks.data.cpu().numpy(),confidence,boxes):
                bitmap=(mask>.5).astype(np.uint8)*255
                if score>=.35:strong|=bitmap
                ignore|=bitmap
                x0,y0,x1,y1=box;pad=12
                ignore[max(0,int(y0)-pad):min(h,int(np.ceil(y1))+pad),max(0,int(x0)-pad):min(w,int(np.ceil(x1))+pad)]=255
        valid=255-cv2.dilate(ignore,np.ones((7,7),np.uint8));name=path.name+'.png'
        cv2.imwrite(str(destination/name),valid);cv2.imwrite(str(core/name),strong)
        records.append(dict(index=i,image=row['file_path'],image_sha256=row['sha256'],mask=name,mask_sha256=sha(destination/name),
            core_sha256=sha(core/name),persons=len(boxes),confidence=confidence.tolist(),boxes=boxes.tolist(),ignored_fraction=float((valid==0).mean())))
        if len(boxes) and len(list(preview.glob('*.jpg')))<12:result.save(str(preview/(path.stem+'.jpg')))
        if i%100==0:print('Image masks',i,len(rows),'person-positive',sum(r['persons']>0 for r in records),flush=True)
    report=dict(images=len(records),person_positive_images=sum(r['persons']>0 for r in records),persons_total=sum(r['persons'] for r in records),
        model='YOLO11s-seg',ultralytics=__version__,weights=str(Path(args.weights).resolve()),weight_sha256=weight_hash,
        source='Image-only COCO person class 0, conf>=0.15; instance union + 12px bbox expansion + 3px dilation; white=valid, black=ignored',
        core_source='Original predicted person silhouettes with confidence>=0.35; used only for conservative measured-point votes',
        simulator_truth_used=False,scope='Learned detections, not ground-truth precision/recall; false positives/missed distant people and shadows remain possible',
        elapsed_seconds=time.monotonic()-started,source_sha256=sha(__file__),frames=records)
    (root/'mask_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='frames'},indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset');p.add_argument('--weights',default=str(ROOT/'integration/half_gs/weights/yolo11s-seg.pt'))
    run(p.parse_args())
