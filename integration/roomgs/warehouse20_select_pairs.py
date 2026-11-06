"""Offline selection of additional *measured* native frames for fixed targets.

This is used only if keyframe thinning missed evidence that exists in the bag.
An unavailable observation remains unavailable; this tool cannot close it.
"""
from pathlib import Path
import argparse,json
import numpy as np
from warehouse20_audit import intersector,view_mask,has_translation_baseline,write,sha


def run(output):
    output=Path(output);data=np.load(output/'mesh_reference.npz')
    definition=json.loads((output/'target_manifest.json').read_text())
    if sha(output/'mesh_reference.npz')!=definition['reference_sha256']:raise RuntimeError('Reference changed')
    p,n=[data[k] for k in ('points','normals')];ray=intersector(data['triangles'])
    align=np.array(json.loads((output/'audit_alignment.json').read_text())['world_from_estimator'])
    info=json.loads((output/'camera_input.json').read_text());k=np.array(info['K']).reshape(3,3)
    rows=json.loads((output/'dataset/frames.json').read_text());native=json.loads((output/'dataset/native_optical_poses.json').read_text())
    centers=[];masks=[]
    for row in rows:
        pose=align@np.array(row['c2w']);centers.append(pose[:3,3]);masks.append(view_mask(ray,p,n,pose,k,info['width'],info['height']))
    centers=np.array(centers);masks=np.array(masks)
    complete=np.array([has_translation_baseline(centers[masks[:,i]]) for i in range(len(p))])
    missing=np.flatnonzero(~complete);old={r['stamp_ns'] for r in rows};candidates=[r for r in native if r['stamp_ns'] not in old]
    visibility=[];positions=[]
    for i,row in enumerate(candidates):
        pose=align@np.array(row['c2w']);positions.append(pose[:3,3])
        visibility.append(view_mask(ray,p[missing],n[missing],pose,k,info['width'],info['height']))
        if i%1000==0:print('Measured candidate',i,len(candidates),flush=True)
    visibility=np.array(visibility);positions=np.array(positions);selected=set();unavailable=[]
    for j,target in enumerate(missing):
        existing=centers[masks[:,target]];ids=np.flatnonzero(visibility[:,j]);found=False
        if len(existing):
            for idx in ids:
                if np.linalg.norm(existing-positions[idx],axis=1).max()>=.4-1e-9:
                    selected.add(int(idx));found=True;break
        if not found:
            for idx in ids:
                far=ids[np.linalg.norm(positions[ids]-positions[idx],axis=1)>=.4-1e-9]
                if len(far):selected.update([int(idx),int(far[0])]);found=True;break
        if not found:unavailable.append(int(target))
    report=dict(scope='Only measured bag poses; RGB extraction still requires exact matching messages',
        original_photo_missing=len(missing),unavailable_targets=unavailable,
        additional_stamp_ns=[candidates[i]['stamp_ns'] for i in sorted(selected)],
        source_sha256=sha(__file__),reference_sha256=sha(output/'mesh_reference.npz'))
    write(output/'additional_pair_selection.json',report);print('Missing',len(missing),'unavailable',len(unavailable),'additional frames',len(selected),flush=True)
    if unavailable:raise RuntimeError('Actual bag still lacks observations; new acquisition required, do not alter targets')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('capture');run(Path(parser.parse_args().capture))
