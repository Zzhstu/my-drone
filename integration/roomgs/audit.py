"""Strict finite-resolution completeness audit. This module is OFFLINE only."""
from pathlib import Path
import json
import math
import hashlib
import numpy as np
from geometry import read_pcd,transform


def targets(scene,step=.20):
    lo,hi=np.array(scene['bounds'])
    points=[];normals=[];labels=[]
    def face(lower,upper,axis,value,normal,label):
        axes=[a for a in range(3) if a!=axis]
        us=[np.arange(lower[a]+step/2,upper[a],step) for a in axes]
        aa,bb=np.meshgrid(*us,indexing='ij')
        p=np.zeros((aa.size,3));p[:,axis]=value;p[:,axes[0]]=aa.ravel();p[:,axes[1]]=bb.ravel()
        # Exclude physically buried samples (e.g. floor under a solid box).
        exposed=np.ones(len(p),dtype=bool)
        for box in scene['boxes']:
            low=np.array(box['minimum']);high=np.array(box['maximum'])
            probe=p+np.array(normal)*.005
            exposed&=~((probe>low+1e-5)&(probe<high-1e-5)).all(axis=1)
        p=p[exposed]
        points.extend(p);normals.extend([normal]*len(p));labels.extend([label]*len(p))
    for axis,negative,positive in [(0,'back','front'),(1,'right','left'),(2,'floor','ceiling')]:
        n=np.zeros(3);n[axis]=1
        face(lo,hi,axis,lo[axis],n.tolist(),negative)
        face(lo,hi,axis,hi[axis],(-n).tolist(),positive)
    for box in scene['boxes']:
        if not box['name'].startswith('landmark_'):continue
        low=np.array(box['minimum']);high=np.array(box['maximum'])
        for axis in range(3):
            n=np.zeros(3);n[axis]=1
            if axis!=2:face(low,high,axis,low[axis],(-n).tolist(),box['name']+'_'+str(axis)+'-')
            face(low,high,axis,high[axis],n.tolist(),box['name']+'_'+str(axis)+'+')
    # Wall samples buried behind the old decorative tiles are replaced by the
    # exposed front tile face at the same y/z, at the declared 20cm resolution.
    p=np.array(points);n=np.array(normals);labels=np.array(labels)
    for box in scene['boxes']:
        if not box['name'].startswith('tile_'):continue
        low=np.array(box['minimum']);high=np.array(box['maximum'])
        on=(labels=='front')&(p[:,1]>=low[1])&(p[:,1]<=high[1])&(p[:,2]>=low[2])&(p[:,2]<=high[2])
        p[on,0]=low[0]
    return p,n,labels


def visible(points,normals,c2w,k,width,height,boxes):
    center=c2w[:3,3];relative=points-center
    cam=relative@c2w[:3,:3]
    depth=cam[:,2];den=np.maximum(depth,.001)
    uv=cam[:,:2]/den[:,None]@np.diag([k[0,0],k[1,1]])+np.array([k[0,2],k[1,2]])
    facing=np.sum(normals*(-relative),axis=1)/np.maximum(np.linalg.norm(relative,axis=1),1e-6)
    good=(depth>.5)&(depth<18)&(facing>math.cos(math.radians(82)))
    good&=(uv[:,0]>=8)&(uv[:,0]<width-8)&(uv[:,1]>=8)&(uv[:,1]<height-8)
    ids=np.flatnonzero(good)
    endpoint=points[ids]+normals[ids]*.005
    ray=endpoint-center
    with np.errstate(divide='ignore',invalid='ignore'):
        for box in boxes:
            t0=(np.array(box['minimum'])-center)/ray
            t1=(np.array(box['maximum'])-center)/ray
            near=np.minimum(t0,t1).max(axis=1);far=np.maximum(t0,t1).min(axis=1)
            blocked=(far>=np.maximum(near,0))&(near<.999)&(far>1e-4)
            good[ids[blocked]]=False
    return good


def audit(output):
    output=Path(output)
    scene=json.loads((output/'audit_scene.json').read_text())
    frames=json.loads((output/'dataset/frames.json').read_text())
    alignment=np.array(json.loads((output/'audit_alignment.json').read_text())['world_from_estimator'])
    info=json.loads((output/'camera_input.json').read_text());k=np.array(info['K']).reshape(3,3)
    p,n,labels=targets(scene)
    first=np.full((len(p),3),np.nan);views=np.zeros(len(p),dtype=int);baseline=np.zeros(len(p),bool)
    for frame in frames:
        pose=alignment@np.array(frame['c2w'])
        seen=visible(p,n,pose,k,info['width'],info['height'],scene['boxes'])
        new=seen&(views==0);first[new]=pose[:3,3]
        ids=np.flatnonzero(seen)
        baseline[ids]|=np.linalg.norm(first[ids]-pose[:3,3],axis=1)>=.4
        views+=seen
    xyz,_=read_pcd(output/'rgb_pt.pcd');xyz=transform(xyz,alignment)
    distance=nearest_distances(xyz,p)
    geometry=distance<=.20
    photos=(views>=2)&baseline
    complete=geometry&photos
    groups={}
    for label in sorted(set(labels)):
        mask=labels==label
        groups[label]=dict(total=int(mask.sum()),geometry=int(geometry[mask].sum()),
                           multiview=int(photos[mask].sum()),complete=int(complete[mask].sum()))
    report=dict(passed=bool(complete.all()),required_fraction=1.0,step_m=.20,distance_tolerance_m=.20,
                minimum_views=2,minimum_translation_baseline_m=.4,total_samples=len(p),
                geometry_fraction=float(geometry.mean()),multiview_fraction=float(photos.mean()),
                complete_fraction=float(complete.mean()),missing_samples=int((~complete).sum()),groups=groups,
                scope='100% of predefined exposed structural/furniture surface samples at 20cm, not continuous or occluded interiors',
                truth_usage='offline audit only; map and training inputs are measured R3LIVE outputs')
    report['input_sha256']={name:hashlib.sha256((output/name).read_bytes()).hexdigest()
                           for name in ('rgb_pt.pcd','dataset/frames.json','camera_input.json','audit_scene.json','audit_alignment.json')}
    np.savez_compressed(output/'coverage_samples.npz',points=p,normals=n,labels=labels,
                        nearest_map_distance=distance,views=views,baseline=baseline,complete=complete)
    (output/'coverage.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['passed']:
        (output/'missing_surfaces.json').write_text(json.dumps([dict(xyz=a.tolist(),face=str(b),distance=float(c),views=int(d))
             for a,b,c,d in zip(p[~complete],labels[~complete],distance[~complete],views[~complete])],indent=2)+'\n')
    return report


def nearest_distances(xyz,points):
    """Measured support only; exact rejection checks prevent approximate holes."""
    try:
        from scipy.spatial import cKDTree
        return cKDTree(xyz).query(points,k=1)[0]
    except ImportError:
        import cv2
        # OpenCV 3.x and 4.x expose different Python FLANN bindings.
        matcher=cv2.FlannBasedMatcher(dict(algorithm=1,trees=8),dict(checks=256))
        matches=matcher.knnMatch(np.ascontiguousarray(points,dtype=np.float32),
                                 np.ascontiguousarray(xyz,dtype=np.float32),k=1)
        distance=np.array([row[0].distance for row in matches])
        # A nearby actual point certifies support. An approximate miss is
        # checked against every original measured point before rejecting it.
        for i in np.flatnonzero(distance>.20):
            distance[i]=np.sqrt(np.min(np.sum((xyz-points[i])**2,axis=1)))
        return distance
