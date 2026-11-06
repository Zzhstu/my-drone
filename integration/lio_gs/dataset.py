"""Strict measured LiDAR/RGB input, bounded decode, no CAD or truth imports."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image,ImageFilter
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from integration.lio_dataset.poses import PoseBuffer
from integration.roomgs.image_sequence import CapturedImages


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Masks:
    def __init__(self,paths,width,height,dilate=3):
        self.paths=paths;self.width=width;self.height=height;self.dilate=dilate
        self.white=np.ones((height,width),dtype=bool)

    def __getitem__(self,index):
        if self.paths[index] is None:return self.white
        with Image.open(self.paths[index]) as source:
            mask=source.convert('L').resize((self.width,self.height),Image.Resampling.NEAREST)
            if self.dilate:
                invalid=Image.fromarray(255-np.asarray(mask))
                mask=Image.fromarray(255-np.asarray(invalid.filter(ImageFilter.MaxFilter(2*self.dilate+1))))
            return np.asarray(mask)>0


def load(source,width,mask_dir=None,dilate=3,require_complete_photos=False,allow_partial_geometry=False):
    source=Path(source).resolve();validation=json.loads((source/'dataset_validation.json').read_text())
    acceptance=json.loads((source/'acceptance.json').read_text())
    partial_allowed=bool(allow_partial_geometry and acceptance.get('baseline_training_authorized') is True)
    if not acceptance['dataset_ready'] or (not acceptance['geometry_complete'] and not partial_allowed) or not validation['passed']:
        raise ValueError('A verified, geometrically complete measured dataset is required')
    if require_complete_photos and not acceptance['joint_complete']:
        raise ValueError('Strict photo completeness has not passed; inputs remain unchanged')
    fingerprints={}
    for name,expected in validation['input_sha256'].items():
        path=source/name
        if sha(path)!=expected:raise ValueError('Input changed: '+name)
        fingerprints[str(path)]=expected
    index=json.loads((source/'source_archive.json').read_text()) if (source/'source_archive.json').exists() else {}
    for name,row in index.items():
        if sha(source/row['archive'])!=row['sha256']:raise ValueError('Acquisition source archive changed: '+name)
    rows=json.loads((source/'dataset/frames.json').read_text());info=json.loads((source/'camera_input.json').read_text())
    if any(abs(v)>1e-8 for v in info['D']):raise ValueError('Undistort images and masks together before training')
    height=round(info['height']*width/info['width']);k=np.array(info['K'],dtype=np.float32).reshape(3,3)
    k[0]*=width/info['width'];k[1]*=height/info['height']
    geometry=json.loads((source/'sensor_geometry.json').read_text())
    extrinsic=np.linalg.inv(np.array(geometry['body_from_imu']))@np.array(geometry['body_from_camera'])
    states=[json.loads(s) for s in (source/'trajectory_imu.jsonl').read_text().splitlines()]
    buffer=PoseBuffer(extrinsic,capacity=len(states)+1)
    for row in states:buffer.add(row['stamp_ns'],row['map_from_imu'])
    images=[];poses=[];mask_paths=[];manifest=[];last=-1
    for i,row in enumerate(rows):
        path=source/'dataset'/row['file_path']
        if sha(path)!=row['sha256']:raise ValueError('RGB changed: '+str(path))
        stamp=row['stamp_ns'];solved=buffer.camera(stamp)
        if stamp<=last or solved is None:raise ValueError('Invalid or unbracketed RGB stamp')
        pose,association=solved
        if not np.allclose(pose,np.array(row['c2w']),atol=1e-9,rtol=0) or association!=row['pose_association']:
            raise ValueError('Camera pose no longer matches original LIO/model extrinsic')
        last=stamp;images.append(path);poses.append(pose);fingerprints[str(path)]=row['sha256']
        if mask_dir is not None:
            relative=Path(row['file_path']).relative_to('images')
            mask_path=Path(mask_dir).resolve()/(str(relative)+'.png')
        else:mask_path=source/'dataset'/row['mask_path'] if row.get('mask_path') else None
        if mask_path is not None:
            if not mask_path.is_file():raise ValueError('Missing mask: '+str(mask_path))
            with Image.open(mask_path) as m:
                values=np.asarray(m)
                if values.ndim!=2 or m.size!=(info['width'],info['height']):raise ValueError('Mask must be single channel at original RGB dimensions')
                if values.dtype!=bool and not np.isin(values,[0,255]).all():raise ValueError('Mask must be binary PNG: 0 ignore, 255 valid')
            digest=sha(mask_path);fingerprints[str(mask_path)]=digest
        else:digest=None
        mask_paths.append(mask_path);manifest.append(dict(index=i,path=str(mask_path) if mask_path else None,sha256=digest))
    if any(p is not None for p in mask_paths) and any(p is None for p in mask_paths):raise ValueError('Every RGB frame must have an explicit mask when masking is enabled')
    masks=Masks(mask_paths,width,height,dilate)
    valid_counts=np.array([int(masks[i].sum()) for i in range(len(rows))]) if any(p is not None for p in mask_paths) else np.full(len(rows),width*height)
    eligible=np.flatnonzero(valid_counts>0)
    if len(eligible)<100:raise ValueError('Too few images with valid supervision')
    train=eligible[np.arange(len(eligible))%8!=0];test=eligible[np.arange(len(eligible))%8==0]
    xyz=np.load(source/'static_map.npz')['xyz'].astype(np.float32)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or len(xyz)<100 or not np.isfinite(xyz).all():raise ValueError('Invalid measured static cloud')
    return dict(source=source,rows=rows,images=CapturedImages(images,width,height),poses=np.array(poses,dtype=np.float32),K=k,
        xyz=xyz,train=train,test=test,masks=masks,mask_manifest=manifest,fingerprints=fingerprints,acceptance=acceptance,
        valid_pixels=valid_counts,excluded_all_black=np.flatnonzero(valid_counts==0).tolist(),width=width,height=height)


def initial_colours(data):
    xyz=data['xyz'];poses=data['poses'];k=data['K'];height=data['height'];width=data['width'];train=data['train']
    sums=np.zeros((len(xyz),3),np.float64);counts=np.zeros(len(xyz),np.int32)
    for i in train[::max(1,len(train)//280)]:
        cam=(xyz-poses[i,:3,3])@poses[i,:3,:3];z=cam[:,2]
        uv=cam[:,:2]/np.maximum(z[:,None],.001);uv=uv@np.diag([k[0,0],k[1,1]])+k[:2,2]
        good=(z>.4)&(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
        ids=np.flatnonzero(good);pix=uv[ids].astype(int);flat=pix[:,1]*width+pix[:,0]
        nearest=np.full(width*height,np.inf);np.minimum.at(nearest,flat,z[ids])
        valid=(z[ids]<=nearest[flat]+.10)&data['masks'][i][pix[:,1],pix[:,0]]
        ids=ids[valid];pix=pix[valid]
        sums[ids]+=data['images'][i][pix[:,1],pix[:,0]]/255.;counts[ids]+=1
    color=np.full((len(xyz),3),.5,dtype=np.float32);good=counts>0;color[good]=sums[good]/counts[good,None]
    return np.clip(color,.001,.999),int(good.sum())
