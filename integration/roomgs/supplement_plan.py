"""Offline gap-only view design, retaining the full immutable audit target set."""
import argparse,json,math
from pathlib import Path
import numpy as np
from warehouse20_audit import intersector,view_mask,has_translation_baseline,sha,write
from warehouse20_mission import SPAWN
from warehouse20_plan import body_pose


def safe(p):
    return bool(np.all(np.abs(p[:2])<=[1.950001,1.450001]) and .449999<=p[2]<=2.450001
        and not (abs(p[0])<2.23 and abs(p[1])<.75 and p[2]<2.24))


def design(bundle,destination):
    data=np.load(bundle/'mesh_reference.npz');definition=json.loads((bundle/'target_manifest.json').read_text())
    state=np.load(bundle/'coverage_samples.npz');missing=np.flatnonzero(~state['complete'])
    if not len(missing):raise RuntimeError('No acquisition gap remains')
    p=data['points'][missing];n=data['normals'][missing];ray=intersector(data['triangles'])
    info=json.loads((bundle/'camera_input.json').read_text());k=np.array(info['K']).reshape(3,3)
    align=np.array(json.loads((bundle/'audit_alignment.json').read_text())['world_from_estimator'])
    rows=json.loads((bundle/'dataset/frames.json').read_text());poses=[align@np.array(r['c2w']) for r in rows]
    seen=np.array([view_mask(ray,p,n,x,k,info['width'],info['height']) for x in poses]);centers=np.array([x[:3,3] for x in poses])
    observed=[centers[seen[:,j]].copy() for j in range(len(missing))]
    packed=data['capability_visibility'];visibility=((packed[:,missing//8]>>(7-missing%8))&1).astype(bool)
    cameras=data['capability_camera_poses'];body=data['capability_body_views'];selected=[];complete=np.zeros(len(missing),bool)
    available=np.ones(len(body),bool)
    for _ in range(100):
        far=np.zeros_like(visibility);unseen=np.array([len(x)==0 for x in observed])
        for j,x in enumerate(observed):
            if len(x):far[:,j]=(np.linalg.norm(cameras[:,None,:3,3]-x[None],axis=-1)>=.50).any(axis=1)
        score=3*(visibility&far&(~complete)[None]).sum(1)+(visibility&unseen[None]).sum(1)
        score=score.astype(float);score[~available]=-1;idx=int(np.argmax(score))
        if score[idx]<=0:break
        selected.append(idx);available[idx]=False
        for j in np.flatnonzero(visibility[idx]):
            observed[j]=np.vstack([observed[j],cameras[idx,:3,3]])
            complete[j]=has_translation_baseline(observed[j])
        if complete.all():break
    for j in np.flatnonzero(~complete):
        ids=np.flatnonzero(visibility[:,j]);c=cameras[ids,:3,3]
        distance=np.linalg.norm(c[:,None]-c[None,:],axis=-1);a,b=np.unravel_index(np.argmax(distance),distance.shape)
        if distance[a,b]<.4-1e-9:raise RuntimeError('Frozen witness pair unavailable')
        for idx in [int(ids[a]),int(ids[b])]:
            if idx not in selected:selected.append(idx)
    nodes=[];offsets=[-.09,-.045,0.,.045,.09] if len(selected)<=28 else [-.06,0.,.06]
    for idx in selected:
        b=body[idx];lateral=np.array([-math.sin(b[3]),math.cos(b[3]),0.]);forward=np.array([math.cos(b[3]),math.sin(b[3]),0.])
        for u in offsets:
            for z in offsets:
                q=b.copy();q[:3]+=u*lateral+[0.,0.,z]
                if safe(q[:3]):nodes.append(q)
        for v in [-.06,.06]:
            q=b.copy();q[:3]+=v*forward
            if safe(q[:3]):nodes.append(q)
    nodes=list({tuple(np.round(p,9)):p for p in nodes}.values());route=[];sweeps=[]
    current=np.array([-1.3,-1.2,1.]);heading=math.pi/2
    def add(x,a,sweep=False):
        nonlocal current,heading
        if sweep:sweeps.append(len(route))
        route.append([*(np.array(x)-SPAWN),float(a)]);current=np.array(x);heading=a
    add(current,heading)
    for side in [-1,1,0]:
        group=[p.copy() for p in nodes if ((p[2]>=2.24) if side==0 else (p[2]<2.24 and np.sign(p[1])==side))]
        if not group:continue
        add([*current[:2],2.35],heading);add([-1.3,side*1.2 if side else -1.2,2.35],heading)
        while group:
            scores=[np.linalg.norm(p[:3]-current)+.15*abs((p[3]-heading+math.pi)%(2*math.pi)-math.pi) for p in group]
            station=group.pop(int(np.argmin(scores)));add(station[:3],station[3],True)
    add([*current[:2],2.35],heading);add([-1.3,-1.2,2.35],math.pi/2);add([-1.3,-1.2,1.],math.pi/2)
    xyz=np.array(route)[:,:3]+SPAWN
    for a,b in zip(xyz[:-1],xyz[1:]):
        for t in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/.03)))):
            if not safe(a*(1-t)+b*t):raise RuntimeError('Unsafe auxiliary route segment')
    new_centers=[];new_masks=[];optical=data['body_from_camera']
    for item in route:
        pose=body_pose(np.array(item[:3])+SPAWN,item[3])@optical
        new_centers.append(pose[:3,3]);new_masks.append(view_mask(ray,p,n,pose,k,info['width'],info['height']))
    total_centers=np.vstack([centers,new_centers]);total_seen=np.vstack([seen,new_masks])
    supported=np.array([has_translation_baseline(total_centers[total_seen[:,j]]) for j in range(len(missing))])
    if not supported.all():raise RuntimeError('Ideal additional views still lack frozen target support')
    result=dict(scene='warehouse20',world_sha256=definition['world_sha256'],mesh_sha256=definition['mesh_sha256'],
        reference_sha256=definition['reference_sha256'],body_from_camera=optical.tolist(),spawn_world=SPAWN.tolist(),
        source='Known-model gap-only offline plan; no fabricated measurements or reduced target set',
        waypoints=route,path_length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
        required_samples=len(data['points']),targeted_missing_ids=missing.tolist(),selected_capability_views=len(selected),
        ideal_combined_missing_samples=0,combined_preflight_scope='Actual old poses plus ideal proposed stations, NOT acquired coverage',
        sweep_station_indices=sweeps,position_tolerance=.10,yaw_tolerance=.06,dwell_seconds=.8,sweep_dwell_seconds=2.,
        integral_feedback=True,source_frames_sha256=sha(bundle/'dataset/frames.json'),source_sha256=sha(__file__))
    write(destination,result)
    print(json.dumps({k:v for k,v in result.items() if k not in ['waypoints','sweep_station_indices','body_from_camera']},indent=2),flush=True)
    print('Auxiliary waypoints:',len(route),'sweep stations',len(sweeps),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();design(a.bundle,a.output)
