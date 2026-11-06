"""Offline rigid point-to-plane registration of measured native PCDs only.

For evaluating possible reuse of retained measured image sessions. No CAD,
Gazebo pose, frozen audit alignment, scale fit or synthetic points are read.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'integration/roomgs'))
from geometry import read_pcd,transform


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def voxel(p,step=.08):
    _,ids=np.unique(np.floor(p/step).astype(int),axis=0,return_index=True)
    return p[np.sort(ids)]


def register(source,target,destination):
    a=voxel(read_pcd(source)[0]);b=voxel(read_pcd(target)[0]);tree=cKDTree(b)
    _,neighbors=tree.query(b,k=16,workers=2)
    centered=b[neighbors]-b[neighbors].mean(axis=1,keepdims=True)
    covariance=np.einsum('nki,nkj->nij',centered,centered)/16
    _,vectors=np.linalg.eigh(covariance);normals=vectors[:,:,0]
    heldout=np.arange(len(a))%5==0;train=a[~heldout];matrix=np.eye(4);history=[]
    for iteration in range(70):
        p=transform(train,matrix);distance,ids=tree.query(p,workers=2)
        keep=distance<max(.08,.20-.004*iteration)
        p=p[keep];q=b[ids[keep]];n=normals[ids[keep]]
        residual=np.sum(n*(q-p),axis=1);weight=np.sqrt(np.minimum(1.,.025/np.maximum(abs(residual),1e-9)))
        system=np.column_stack([np.cross(p,n),n])
        update,_,rank,_=np.linalg.lstsq(system*weight[:,None],residual*weight,rcond=None)
        if rank!=6:raise RuntimeError('Measured registration is degenerate')
        update[:3]*=min(1.,.10/max(np.linalg.norm(update[:3]),1e-12))
        update[3:]*=min(1.,.10/max(np.linalg.norm(update[3:]),1e-12))
        delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec(update[:3]).as_matrix();delta[:3,3]=update[3:]
        matrix=delta@matrix
        history.append(dict(iteration=iteration,pairs=len(p),plane_rms_m=float(np.sqrt(np.mean(residual**2))),update_norm=float(np.linalg.norm(update))))
        if np.linalg.norm(update)<1e-7:break
    distance,_=tree.query(transform(a[heldout],matrix),workers=2)
    overlap=float((distance<=.08).mean());median=float(np.median(distance))
    angle=float(Rotation.from_matrix(matrix[:3,:3]).magnitude());translation=float(np.linalg.norm(matrix[:3,3]))
    passed=bool(overlap>=.85 and median<=.05 and angle<.26 and translation<.5 and history[-1]['plane_rms_m']<.04)
    report=dict(passed=passed,source=str(Path(source).resolve()),target=str(Path(target).resolve()),
        target_from_source=matrix.tolist(),source_sha256=sha(source),target_sha256=sha(target),
        registration_source_sha256=sha(__file__),source_points=len(a),target_points=len(b),
        heldout_points=int(heldout.sum()),heldout_overlap_within_8cm=overlap,heldout_median_nn_m=median,
        rotation_rad=angle,translation_m=translation,history=history,truth_or_CAD_used=False,scale_fit=False,
        scope='Internal consistency of rigid measured-map registration with near-identity initialization; not external pose accuracy or complete-map acceptance')
    Path(destination).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('history','target_from_source')},indent=2),flush=True)
    if not passed:raise RuntimeError('Measured registration failed internal consistency; do not reuse old poses')
    return matrix


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source');p.add_argument('target');p.add_argument('output')
    a=p.parse_args();register(a.source,a.target,a.output)
