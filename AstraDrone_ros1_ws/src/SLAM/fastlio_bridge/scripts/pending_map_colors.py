#!/usr/bin/env python3
"""Color existing measured points by unresolved ledger state, never add points.

Other map points are NOT certified static or ghost-free. A suspended structural
candidate remains unresolved. Cleared history without occupancy is not drawn.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


LEGEND = {
    0: dict(name='other_map_points_not_certified_static', rgb=[155,175,190]),
    1: dict(name='unresolved_with_post_departure_evidence', rgb=[255,55,55]),
    2: dict(name='unresolved_surface_revisit_suspended', rgb=[255,175,25]),
    3: dict(name='unresolved_without_post_departure_evidence', rgb=[225,55,235]),
    4: dict(name='ledger_unavailable', rgb=[125,105,220]),
}


def label_points(points, history):
    xyz=np.ascontiguousarray(points,dtype='<f4').reshape(-1,3)
    if not np.isfinite(xyz).all(): raise ValueError('nonfinite map points')
    labels=np.full(len(xyz),4 if history is None else 0,dtype='<u4')
    unmatched=[]; pending=[]; collisions=[]
    if history is not None:
        if history.get('version')!=1 or not isinstance(history.get('records'),list):
            raise ValueError('invalid ghost ledger')
        # Exact float32 mean coordinates match the exact published PCL point.
        # Re-flooring a rounded PCD point can choose a neighboring voxel at a
        # cell boundary; nearest-neighbor coloring can paint an unrelated wall.
        evidence={}
        for r in history['records']:
            if r.get('state') not in ('suspected','evicted_unresolved'): continue
            position=r.get('position')
            if not isinstance(position,(list,tuple)) or len(position)!=3 or not np.isfinite(position).all():
                raise ValueError('unresolved ledger record lacks a finite position')
            if r.get('state')=='evicted_unresolved' and not r.get('occupied_now',False): continue
            code=2 if r.get('static_surface_suppressed') else (
                1 if any(r.get(k,0)>0 for k in ('free_evidence_frames',
                    'positive_endpoint_frames','post_departure_hit_frames')) else 3)
            key=tuple(np.asarray(position,dtype='<f4'))
            if key in evidence:
                old_code,old_key=evidence[key]
                collisions.append(dict(position=list(map(float,key)),keys=[old_key,r.get('key')]))
                # Distinct voxel means can collapse to one float32 coordinate.
                # Keep both map points and conservatively show the stronger
                # unresolved state, rather than failing the final map save.
                severity={2:0,1:1,3:2}
                code=max((code,old_code),key=lambda value:severity[value])
            evidence[key]=(code,r.get('key'))
            pending.append(dict(key=r.get('key'),position=position,label=code))
        found=set()
        for index,p in enumerate(xyz):
            key=tuple(p)
            if key in evidence:
                labels[index]=evidence[key][0];found.add(key)
        unmatched=[dict(key=k,position=list(map(float,p))) for p,(_,k) in evidence.items() if p not in found]
    colors=np.asarray([LEGEND[i]['rgb'] for i in range(5)],dtype=np.uint32)[labels]
    packed=(colors[:,0]<<16)|(colors[:,1]<<8)|colors[:,2]
    summary=dict(points=len(xyz),counts={str(i):int(np.count_nonzero(labels==i)) for i in LEGEND},
        legend={str(k):v for k,v in LEGEND.items()},unmatched_unresolved_records=unmatched,
        unresolved_records=pending,coordinate_collision_records=collisions,
        scope='Labels describe unresolved ghost history, not ground-truth moving objects. '
              'Only existing measured XYZ are colored; gray does not mean certified ghost-free.')
    return xyz,packed,labels,summary


def colored_payload(points,history):
    xyz,rgb,labels,summary=label_points(points,history)
    rows=np.empty(len(xyz),dtype=[('xyz','<f4',(3,)),('rgb','<u4'),('label','<u4')])
    rows['xyz']=xyz;rows['rgb']=rgb;rows['label']=labels
    header=('# .PCD v0.7 - measured map with unresolved ghost labels\nVERSION 0.7\n'
        'FIELDS x y z rgb label\nSIZE 4 4 4 4 4\nTYPE F F F F U\nCOUNT 1 1 1 1 1\n'
        'WIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {0}\nDATA binary\n').format(len(xyz))
    payload=header.encode('ascii')+rows.tobytes()
    summary['sha256']=hashlib.sha256(payload).hexdigest()
    summary['fields']=['x','y','z','rgb','label']
    return payload,summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd',type=Path)
    parser.add_argument('--history',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    from warehouse_map_audit import read_xyz_pcd
    if args.output.resolve()==args.pcd.resolve(): raise ValueError('refuse to replace source XYZ PCD')
    history=json.loads(args.history.read_text())
    payload,summary=colored_payload(read_xyz_pcd(args.pcd),history)
    summary.update(source_xyz=str(args.pcd.resolve()),source_xyz_sha256=hashlib.sha256(args.pcd.read_bytes()).hexdigest(),
        source_history=str(args.history.resolve()),frame_id=history.get('frame_id'),map_stamp=history.get('map_stamp'))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_bytes(payload)
    args.output.with_suffix('.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary['counts']))


if __name__=='__main__':main()
