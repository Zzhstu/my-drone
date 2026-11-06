"""Conservative image-model votes on measured DUFOMap points; no truth input."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time
import sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from integration.lio_gs.dataset import Masks


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def run(root):
    root=Path(root);frames=json.loads((root/'dataset/frames.json').read_text());info=json.loads((root/'camera_input.json').read_text())
    k=np.array(info['K']).reshape(3,3);width,height=info['width'],info['height'];xyz=np.load(root/'static_map.npz')['xyz']
    mask_info=json.loads((root/'mask_manifest.json').read_text());depth_info=json.loads((root/'lidar_depth_manifest.json').read_text())['records']
    mask_paths=[root/'dataset/masks'/(Path(row['file_path']).name+'.png') for row in frames]
    valid_masks=Masks(mask_paths,width,height,3)
    eligible=np.array([i for i in range(len(frames)) if valid_masks[i].any()]);heldout=set(eligible[np.arange(len(eligible))%8==0].tolist())
    people=np.zeros(len(xyz),np.int32);static=np.zeros(len(xyz),np.int32);used=[];started=time.monotonic()
    for i,row in enumerate(frames):
        if i in heldout or not depth_info[i]['available'] or not valid_masks[i].any():continue
        pose=np.array(row['c2w']);camera=(xyz-pose[:3,3])@pose[:3,:3];z=camera[:,2]
        uv=camera[:,:2]/np.maximum(z[:,None],.001);uv=uv*np.array([k[0,0],k[1,1]])+k[:2,2]
        good=(z>.15)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
        ids=np.flatnonzero(good);pix=uv[ids].astype(int)
        data=np.load(root/'dataset/lidar_depth'/depth_info[i]['path']);depth=np.full((height,width),np.inf,np.float32)
        samples=data['uv'];np.minimum.at(depth,(samples[:,1],samples[:,0]),data['z'])
        depth=cv2.erode(depth,np.ones((11,11),np.uint8))
        visible=np.abs(z[ids]-depth[pix[:,1],pix[:,0]])<=.18
        ids=ids[visible];pix=pix[visible]
        name=Path(row['file_path']).name+'.png';core=cv2.imread(str(root/'dataset/person_core'/name),cv2.IMREAD_GRAYSCALE)
        valid=valid_masks[i]
        if core is None:raise ValueError('Missing learned mask')
        people[ids]+=core[pix[:,1],pix[:,0]]>127;static[ids]+=valid[pix[:,1],pix[:,0]];used.append(i)
        if i%200==1:print('Semantic point votes',i,len(frames),flush=True)
    remove=(people>=2)&(static<2);clean=xyz[~remove]
    # Preserve raw measured products and hashes before publishing the curated map.
    previous={}
    for name in ['static_map.npz','static_map.ply','static_map.pcd','dataset_validation.json']:
        p=root/name
        if p.exists():
            previous[name]=sha(p);shutil.copyfile(p,p.with_name('pre_semantic_'+p.name))
    np.savez_compressed(root/'static_map.npz',xyz=clean)
    np.savez_compressed(root/'semantic_point_votes.npz',xyz=xyz,person_votes=people,static_votes=static,removed=remove)
    for suffix in ['ply','pcd']:
        if suffix=='ply':header='ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n'%len(clean)
        else:header='VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n'%(len(clean),len(clean))
        (root/('static_map.'+suffix)).write_bytes(header.encode()+clean.astype('<f4').tobytes())
    report=dict(input_points=len(xyz),removed_points=int(remove.sum()),retained_points=len(clean),used_training_indices=used,
        heldout_indices=sorted(heldout),
        original_input_sha256=previous,final_map_sha256=sha(root/'static_map.npz'),
        policy='Only actual measured DUFOMap points; >=2 strong predicted person-silhouette observations and <2 valid-static observations; same-frame measured LiDAR depth agrees within18cm; held-out image indices excluded',
        truth_used=False,mask_weights_sha256=mask_info['weight_sha256'],
        scope='Image-model semantic curation, not ground-truth dynamic accuracy; false positives/misses remain possible; dropped raw points preserved',
        source_sha256=sha(__file__),wall_seconds=time.monotonic()-started)
    (root/'semantic_filter.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='used_training_indices'},indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');run(p.parse_args().dataset)
