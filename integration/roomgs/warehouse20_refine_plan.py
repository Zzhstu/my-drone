"""Known-model full inspection with measured-failure-directed local sweeps.

The same independently frozen targets are retained. No new sensor samples or
poses are synthesized; all planned stations still require actual acquisition.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from warehouse20_audit import sha,write,intersector,view_mask
from warehouse20_plan import body_pose,verify
from warehouse20_mission import SPAWN


def safe(p):
    return (np.all(np.abs(p[:2]) <= [2.04,1.54]) and .405<=p[2]<=2.495
            and not (abs(p[0])<2.23 and abs(p[1])<.75 and p[2]<2.24))


def annotate_sweeps(reference,failed,destination):
    data=np.load(reference/'mesh_reference.npz');count=len(data['points'])
    masks=np.unpackbits(data['capability_visibility'],axis=1)[:,:count].astype(bool)
    centers=data['capability_camera_poses'][:,:3,3];poses=data['capability_body_views']
    critical=np.flatnonzero(~np.load(failed/'coverage_samples.npz')['complete'])
    witnesses=set()
    for column in critical:
        ids=np.flatnonzero(masks[:,column]);xyz=centers[ids]
        distances=np.linalg.norm(xyz[:,None]-xyz[None,:],axis=-1)
        a,b=np.unravel_index(np.argmax(distances),distances.shape)
        witnesses.update([int(ids[a]),int(ids[b])])
    keys=set()
    for idx in witnesses:
        p=poses[idx];side=np.array([-math.sin(p[3]),math.cos(p[3]),0.])
        for offset in [np.zeros(3),np.array([0.,0.,-.045]),np.array([0.,0.,.045]),-.045*side,.045*side]:
            q=p.copy();q[:3]+=offset
            if safe(q[:3]):keys.add(tuple(np.round(q,9)))
    plan=json.loads(destination.read_text());indices=[]
    for i,p in enumerate(plan['waypoints']):
        q=np.array(p);q[:3]+=SPAWN
        if tuple(np.round(q,9)) in keys:indices.append(i)
    plan.update(sweep_station_indices=indices,sweep_dwell_seconds=1.5,
        sweep_annotation_source_sha256=sha(__file__))
    write(destination,plan)
    print('Extended dwell stations:',len(indices),flush=True)


def design(reference,failed,destination):
    data=np.load(reference/'mesh_reference.npz')
    definition=json.loads((reference/'target_manifest.json').read_text())
    count=len(data['points']);masks=np.unpackbits(data['capability_visibility'],axis=1)[:,:count].astype(bool)
    poses=data['capability_body_views'];centers=data['capability_camera_poses'][:,:3,3]
    first=np.full((count,3),np.nan);complete=np.zeros(count,bool);selected=[]
    available=np.ones(len(masks),bool)
    far=np.zeros_like(masks)
    for iteration in range(700):
        unseen=~np.isfinite(first[:,0])
        score=(masks & unseen[None]).sum(axis=1)+2*(masks & far & (~complete)[None]).sum(axis=1)
        score=score.astype(float)
        score[~available]=-1;idx=int(np.argmax(score))
        if score[idx]<=0:break
        selected.append(idx);available[idx]=False;seen=masks[idx]
        new=seen&unseen;first[new]=centers[idx]
        if new.any():far[:,new]=np.linalg.norm(centers[:,None,:]-first[None,new,:],axis=-1)>=.50
        complete|=seen&far[idx]
        if iteration%25==0:print('Full inspection stations',len(selected),'remaining',int((~complete).sum()),flush=True)
        if complete.all():break
    # The eligibility lattice guarantees >=40cm, not necessarily 50cm.
    for column in np.flatnonzero(~complete):
        ids=np.flatnonzero(masks[:,column]);xyz=centers[ids]
        distances=np.linalg.norm(xyz[:,None]-xyz[None,:],axis=-1)
        a,b=np.unravel_index(np.argmax(distances),distances.shape)
        if distances[a,b]<.4-1e-9:raise RuntimeError('Frozen target lost a witness')
        for idx in (int(ids[a]),int(ids[b])):
            if idx not in selected:selected.append(idx)
    stations=[p.copy() for p in poses[selected]]
    old=np.load(failed/'coverage_samples.npz');critical=np.flatnonzero(~old['complete'])
    witnesses=set()
    for column in critical:
        ids=np.flatnonzero(masks[:,column]);xyz=centers[ids]
        distances=np.linalg.norm(xyz[:,None]-xyz[None,:],axis=-1)
        a,b=np.unravel_index(np.argmax(distances),distances.shape)
        witnesses.update([int(ids[a]),int(ids[b])])
    # Short cross sweeps around exact capability witnesses provide measured
    # visibility near thin box/board occlusion boundaries. Keep the centre too.
    for idx in sorted(witnesses):
        item=poses[idx].copy();sideways=np.array([-math.sin(item[3]),math.cos(item[3]),0.])
        for offset in [np.zeros(3),np.array([0.,0.,-.045]),np.array([0.,0.,.045]),-.045*sideways,.045*sideways]:
            candidate=item.copy();candidate[:3]+=offset
            if safe(candidate[:3]):stations.append(candidate)
    # Group nearby positions; turn at the station using the shortest yaw arc.
    unique={tuple(np.round(item,9)):item for item in stations};stations=list(unique.values())
    route=[];current=np.array([-1.3,-1.2,1.]);angle=math.pi/2
    def add(p,yaw):
        nonlocal current,angle
        route.append([*(np.array(p)-SPAWN),float(yaw)]);current=np.array(p);angle=yaw
    add(current,angle)
    for side in [-1,1,0]:
        group=[p.copy() for p in stations if ((p[2]>=2.24) if side==0 else (p[2]<2.24 and np.sign(p[1])==side))]
        if not group:continue
        add([*current[:2],2.35],angle)
        safe_y=side*1.2 if side else -1.2
        add([-1.3,safe_y,2.35],angle)
        while group:
            score=[np.linalg.norm(p[:3]-current)+.1*abs((p[3]-angle+math.pi)%(2*math.pi)-math.pi) for p in group]
            p=group.pop(int(np.argmin(score)));add(p[:3],p[3])
    add([*current[:2],2.35],angle);add([-1.3,-1.2,2.35],math.pi/2);add([-1.3,-1.2,1.],math.pi/2)
    xyz=np.array(route)[:,:3]+SPAWN
    for a,b in zip(xyz[:-1],xyz[1:]):
        for t in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(a-b)/.03)))):
            if not safe(a*(1-t)+b*t):raise RuntimeError('Unsafe route segment')
    plan=dict(scene='warehouse20',world_sha256=definition['world_sha256'],mesh_sha256=definition['mesh_sha256'],
        reference_sha256=definition['reference_sha256'],body_from_camera=data['body_from_camera'].tolist(),
        source='OFFLINE known-mesh full view selection; failed measured targets add local station sweeps; not autonomous exploration',
        spawn_world=SPAWN.tolist(),waypoints=route,required_samples=count,ideal_missing_samples=0,
        path_length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
        supplementary_views=len(stations),base_waypoints=1,measured_failure_targets=critical.tolist(),
        measured_failure_report_sha256=sha(failed/'coverage.json'),
        source_sha256={str(Path(__file__).resolve()):sha(__file__)})
    write(destination,plan)
    annotate_sweeps(reference,failed,destination)
    print('Full stations',len(selected),'critical witnesses',len(witnesses),'route',len(route),'metres',plan['path_length_m'],flush=True)
    verify(reference,destination)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--failed',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--annotate-existing',action='store_true')
    a=p.parse_args()
    if a.annotate_existing:annotate_sweeps(a.reference,a.failed,a.output);verify(a.reference,a.output)
    else:design(a.reference,a.failed,a.output)
