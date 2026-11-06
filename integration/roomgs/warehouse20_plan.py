"""OFFLINE known-model inspection design, not unknown-room exploration.

The fixed capability reference is created independently of this route. Geometry
is used for preflight view selection only; flight gets numeric waypoints, and
neither R3LIVE nor gsplat receives this mesh or simulator poses.
"""
from pathlib import Path
import argparse
import json
import math
import sys
import numpy as np
from warehouse20_audit import ROOT,intersector,view_mask,sha,write,has_translation_baseline
from warehouse20_mission import SPAWN,waypoints,ideal_camera_poses
sys.path.insert(0,str(ROOT))
from integration.r3live.calibration import read_geometry


def body_pose(xyz,yaw):
    c,s=math.cos(yaw),math.sin(yaw);pose=np.eye(4)
    pose[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];pose[:3,3]=xyz;return pose


def verify(reference,destination):
    plan=json.loads(destination.read_text());data=np.load(reference/'mesh_reference.npz')
    p,n=[data[k] for k in ('points','normals')];ray=intersector(data['triangles'])
    optical=np.array(plan['body_from_camera']);k=np.array([[462.26636,0,320],[0,462.26636,240],[0,0,1.]])
    centers=[];masks=[]
    for item in plan['waypoints']:
        pose=body_pose(np.array(item[:3])+SPAWN,item[3])@optical
        centers.append(pose[:3,3]);masks.append(view_mask(ray,p,n,pose,k,640,480))
    centers=np.array(centers);masks=np.array(masks);views=masks.sum(0)
    baseline=np.array([has_translation_baseline(centers[masks[:,i]]) for i in range(len(p))])
    complete=(views>=2)&baseline
    result=dict(scope='Independent fixed waypoint preflight, not acquired measurements',passed=bool(complete.all()),
        total_samples=len(p),missing_samples=int((~complete).sum()),fraction=float(complete.mean()),
        minimum_views=int(views.min()),waypoints=len(masks),path_length_m=plan['path_length_m'],
        plan_sha256=sha(destination),reference_sha256=sha(reference/'mesh_reference.npz'))
    print(json.dumps(result,indent=2),flush=True)
    if not result['passed']:raise RuntimeError('Planned station views are incomplete')
    return result


def design(reference,destination):
    definition=json.loads((reference/'target_manifest.json').read_text());data=np.load(reference/'mesh_reference.npz')
    p,n=[data[k] for k in ('points','normals')];ray=intersector(data['triangles'])
    vehicle=Path('/home/a/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/iris_mid360.sdf')
    g=read_geometry(vehicle,ROOT/'simulation/astra_gazebo_models/mid360/mid360.sdf',ROOT/'simulation/astra_gazebo_models/D435i/model.sdf')
    optical=np.array(g['body_from_camera']);k=np.array([[462.26636,0,320],[0,462.26636,240],[0,0,1.]])
    poses=[body_pose(np.array(item[:3])+SPAWN,item[3])@optical for item in waypoints()]
    views=np.zeros(len(p),int);first=np.full(p.shape,np.nan);baseline=np.zeros(len(p),bool)
    base_masks=[];base_centers=[]
    for pose in poses:
        seen=view_mask(ray,p,n,pose,k,640,480);new=seen&(views==0);first[new]=pose[:3,3];views+=seen;ids=np.flatnonzero(seen)
        baseline[ids]|=np.linalg.norm(first[ids]-pose[:3,3],axis=1)>=.4
        base_masks.append(seen);base_centers.append(pose[:3,3])
    base_masks=np.array(base_masks);base_centers=np.array(base_centers)
    for idx in np.flatnonzero((views>=2)&~baseline):
        baseline[idx]=has_translation_baseline(base_centers[base_masks[:,idx]])
    missing=np.flatnonzero(~((views>=2)&baseline));print('Base missing',len(missing),flush=True)
    points=p[missing];normals=n[missing];initial=first[missing].copy();covered=np.zeros(len(missing),bool)
    masks=np.unpackbits(data['capability_visibility'],axis=1)[:,:len(p)][:,missing].astype(bool)
    camera=data['capability_camera_poses'][:,:3,3];candidates=data['capability_body_views']
    far=np.linalg.norm(camera[:,None,:]-initial[None],axis=-1)>=.45
    selected=[];available=np.ones(len(masks),bool)
    for _ in range(500):
        unseen=~np.isfinite(initial[:,0])
        gain=(masks&far)@(~covered).astype(float)+.5*(masks@unseen.astype(float))
        gain[~available]=-1;idx=int(np.argmax(gain))
        if gain[idx]<=0:break
        available[idx]=False;selected.append(idx);seen=masks[idx]
        new=seen&unseen;initial[new]=camera[idx]
        if new.any():far[:,new]=np.linalg.norm(camera[:,None,:]-initial[None,new,:],axis=-1)>=.45
        covered|=seen&far[idx]
        if len(selected)%10==0:print('Selected',len(selected),'remaining',int((~covered).sum()),flush=True)
        if covered.all():break
    for column in np.flatnonzero(~covered):
        ids=np.flatnonzero(masks[:,column]);xyz=camera[ids]
        # Exhaustive witness pair only for the few first-anchor corner cases.
        distances=np.linalg.norm(xyz[:,None]-xyz[None,:],axis=-1)
        a,b=np.unravel_index(np.argmax(distances),distances.shape)
        if distances[a,b]<.4-1e-9:raise RuntimeError('No real 40cm witness pair for frozen target %d'%int(missing[column]))
        for chosen in (int(ids[a]),int(ids[b])):
            if chosen not in selected:selected.append(chosen)
        covered[column]=True
    extra=candidates[selected];route=waypoints()
    # Nearest-neighbour inspection within each side, with a high transfer
    # through the central gap. Every low waypoint is outside rack inflation.
    current=np.array(route[-1][:3])+SPAWN
    def add(xyz,yaw):route.append([*(np.array(xyz)-SPAWN),float(yaw)])
    for side in [-1,1,0]:
        group=[x.copy() for x in extra if ((x[2]>=2.28) if side==0 else (x[2]<2.28 and np.sign(x[1])==side))]
        if not group:continue
        add([current[0],current[1],2.35],route[-1][3])
        safe_y=side*1.2 if side else -1.2
        add([-1.3,safe_y,2.35],route[-1][3]);current=np.array([-1.3,safe_y,2.35])
        while group:
            scores=[np.linalg.norm(x[:3]-current) for x in group];i=int(np.argmin(scores));point=group.pop(i)
            add(point[:3],point[3]);current=point[:3]
    add([current[0],current[1],2.35],route[-1][3]);add([-1.3,-1.2,2.35],math.pi/2);add([-1.3,-1.2,1.],math.pi/2)
    # Independent continuous path guard against structure/rack AABB. This
    # conservative guard is only preflight; measured scans guard actual flight.
    xyz=np.array(route)[:,:3]+SPAWN
    for a,b in zip(xyz[:-1],xyz[1:]):
        for u in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b-a)/.04)))):
            point=a*(1-u)+b*u
            if np.any(np.abs(point[:2])>np.array([2.08,1.58])) or not .40<=point[2]<=2.5:
                raise RuntimeError('Path violates structural body envelope')
            if abs(point[0])<2.23 and abs(point[1])<.75 and point[2]<2.24:
                raise RuntimeError('Path crosses inflated rack')
    result=dict(scene='warehouse20',world_sha256=definition['world_sha256'],mesh_sha256=definition['mesh_sha256'],
        reference_sha256=definition['reference_sha256'],body_from_camera=optical.tolist(),
        source='OFFLINE known-mesh preflight design; not autonomous unknown-world exploration',
        spawn_world=SPAWN.tolist(),waypoints=route,supplementary_views=len(selected),
        base_waypoints=len(waypoints()),required_samples=len(p),ideal_missing_samples=0,
        path_length_m=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum()),
        source_sha256={str(Path(__file__).resolve()):sha(__file__),str(Path(__file__).with_name('warehouse20_mission.py').resolve()):sha(Path(__file__).with_name('warehouse20_mission.py'))})
    destination.parent.mkdir(parents=True,exist_ok=True);write(destination,result);print('Plan saved',destination,'views',len(selected),'waypoints',len(route),'metres',result['path_length_m'],flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--reference',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--verify',action='store_true')
    args=parser.parse_args()
    if args.verify:verify(args.reference,args.output)
    else:design(args.reference,args.output)
