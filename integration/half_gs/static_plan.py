"""Known-scene three-height inspection; fixed camera, unchanged audit targets."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from integration.half_gs.scene import prepare_scene
from integration.half_gs.plan import navigation
from integration.roomgs.camera_profiles import profile_projection, G1_PROFILE
from integration.roomgs.warehouse20_audit import intersector,view_mask,has_translation_baseline


def write(p,d):p.write_text(json.dumps(d,indent=2)+'\n')


def build(args):
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    class SceneArgs:static=True;no_flight=True;plan=None;reference_dir=args.reference
    _,meta,_=prepare_scene(output,SceneArgs())
    _,_,_,connected,home,index,path,xy=navigation(meta,actor_keepouts=False,continuous_clearance=True)
    nodes=list(sorted(connected));positions=np.array([xy(n) for n in nodes])
    spawn=np.array(meta['spawn_world']);groups=[]
    for box in meta['boxes']:
        if not box['name'].startswith('shelf'):continue
        center=np.array(box['center'][:2]);angle=box['yaw']
        tangent=np.array([math.cos(angle),math.sin(angle)])
        for side in [-1,1]:
            normal=side*np.array([-math.sin(angle),math.cos(angle)])
            candidates=[]
            for offset in [-1.,1.]:
                desired=center+normal*(box['size'][1]/2+1.35)+tangent*offset
                valid=(positions-center)@normal>=1.05
                score=np.linalg.norm(positions-desired,axis=1);score[~valid]=np.inf
                selected=int(np.argmin(score))
                if not np.isfinite(score[selected]) or score[selected]>1.4:raise ValueError('No safely connected station for '+box['name'])
                node=nodes[selected];position=positions[selected]
                heading=math.atan2(*(center-position)[::-1])
                candidates.append((node,heading))
            if np.linalg.norm(xy(candidates[0][0])-xy(candidates[1][0]))<.8:raise ValueError('Insufficient face translation baseline')
            groups.append(dict(rack=box['name'],side=side,stations=candidates))
    route=[];observation_indices=[];visits=[];current_node=home;current_z=1.2;current_yaw=0.
    def add(position,z,yaw,observe=False):
        route.append([float(position[0]-spawn[0]),float(position[1]-spawn[1]),float(z-spawn[2]),float(yaw)])
        if observe:observation_indices.append(len(route)-1)
    add(spawn[:2],1.2,0.,True)
    def move(node):
        nonlocal current_node
        way=path(current_node,node);anchors=[]
        for i in range(1,len(way)-1):
            if tuple(np.subtract(way[i],way[i-1]))!=tuple(np.subtract(way[i+1],way[i])):anchors.append(way[i])
        if len(way)>1:anchors.append(way[-1])
        for n in anchors:add(xy(n),current_z,current_yaw)
        current_node=node
    pending=groups.copy()
    while pending:
        options=[]
        for i,g in enumerate(pending):
            for order in [0,1]:
                node,heading=g['stations'][order];way=path(current_node,node)
                distance=sum(np.linalg.norm(xy(a)-xy(b)) for a,b in zip(way,way[1:]))
                options.append((distance+.2*abs(math.atan2(math.sin(heading-current_yaw),math.cos(heading-current_yaw))),i,order))
        _,choice,first=min(options);group=pending.pop(choice)
        low,mid,high=.65,1.2,1.90
        heights=[low,mid,high] if abs(current_z-low)<abs(current_z-high) else [high,mid,low]
        order=[first,1-first]
        indices=[]
        for level,z in enumerate(heights):
            for station in (order if level%2==0 else order[::-1]):
                node,heading=group['stations'][station];move(node)
                add(xy(node),z,heading,True);indices.append(len(route)-1);current_z=z;current_yaw=heading
        visits.append(dict(rack=group['rack'],side=group['side'],indices=indices))
    # Four outer viewpoints, each reached through the same connected free grid.
    bounds=meta['bounds_world'];middle=np.array([(bounds[0]+bounds[2])/2,(bounds[1]+bounds[3])/2])
    for corner in [[bounds[0]+1.,bounds[1]+1.],[bounds[2]-1.,bounds[1]+1.],
                   [bounds[2]-1.,bounds[3]-1.],[bounds[0]+1.,bounds[3]-1.]]:
        node=nodes[int(np.argmin(np.linalg.norm(positions-corner,axis=1)))];move(node)
        heading=math.atan2(*(middle-xy(node))[::-1]);add(xy(node),1.2,heading,True);current_z=1.2;current_yaw=heading
    move(home);add(spawn[:2],1.2,0.)
    distance=sum(np.linalg.norm(np.array(a[:3])-b[:3]) for a,b in zip(route,route[1:]))
    plan=dict(waypoints=route,observation_indices=observation_indices,face_visits=visits,waypoint_count=len(route),
        route_length_m=float(distance),body_height_bands_m=[.65,1.2,1.90],dwell_seconds=.6,
        geometry_clearance_m=.8,minimum_ceiling_origin_gap_m=.55,partial_baseline_authorized=True,
        actor_free=True,scope='Known static half-warehouse, fixed monocular camera, two translated viewpoints per rack face per height; no gimbal/vehicle pitch commands; not unknown-scene exploration or real-flight safety validation',
        camera_policy='G1 centered pinhole approximation, 1280x720; mount retained; independent sparse LiDAR',
        camera_projection=profile_projection(1280,720,G1_PROFILE),
        reference_policy='Previous frozen surface target set unchanged; actor removal checked separately; unobservable faces not declared complete')
    write(output/'inspection.json',plan)
    reference=np.load(output/'mesh_reference.npz');p,n,labels=[reference[k] for k in ['points','normals','labels']]
    ray=intersector(reference['triangles']);extrinsic=reference['body_from_camera'];definition=json.loads((output/'target_manifest.json').read_text())
    k=np.array(plan['camera_projection']['K']);count=np.zeros(len(p),np.int32);first_position=np.full(p.shape,np.nan);baseline=np.zeros(len(p),bool)
    samples=[route[0]]
    for a,b in zip(route,route[1:]):
        a,b=np.array(a),np.array(b);turn=math.atan2(math.sin(b[3]-a[3]),math.cos(b[3]-a[3]))
        steps=max(1,int(math.ceil(np.linalg.norm(b[:3]-a[:3])/.30)),int(math.ceil(abs(turn)/.20)))
        for amount in np.arange(1,steps+1)/steps:
            q=a+(b-a)*amount;q[3]=a[3]+turn*amount;samples.append(q)
    packed=[];centers=[]
    for j,(x,y,z,heading) in enumerate(samples):
        c,s=math.cos(heading),math.sin(heading);t=np.eye(4);t[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];t[:3,3]=np.array([x,y,z])+spawn
        camera=t@extrinsic;seen=view_mask(ray,p,n,camera,k,1280,720);new=seen&(count==0);first_position[new]=camera[:3,3]
        count+=seen;ids=np.flatnonzero(seen);baseline[ids]|=np.linalg.norm(first_position[ids]-camera[:3,3],axis=1)>=.4
        packed.append(np.packbits(seen));centers.append(camera[:3,3])
        if j%200==0:print('Static planned views',j,len(samples),flush=True)
    packed=np.array(packed);centers=np.array(centers)
    for i in np.flatnonzero((count>=2)&~baseline):
        visible=((packed[:,i//8]>>(7-i%8))&1).astype(bool);baseline[i]=has_translation_baseline(centers[visible])
    photos=(count>=2)&baseline;rack=np.char.startswith(labels.astype(str),'shelf')
    report=dict(ideal_photo_fraction=float(photos.mean()),targets=len(p),missing=int((~photos).sum()),sampled_trajectory_views=len(samples),
        bands={},scope='Planned fixed-camera station/path visibility at <=30cm/0.2rad interpolation. Ideal pose interpolation is not executed controller timing. No measured LiDAR map or actual flight evidence; do not call capture complete.')
    for name,band in [('lower',p[:,2]<.6),('middle',(p[:,2]>=.6)&(p[:,2]<1.25)),('upper',p[:,2]>=1.25)]:
        chosen=rack&band;report['bands'][name]=dict(targets=int(chosen.sum()),supported=int(photos[chosen].sum()),fraction=float(photos[chosen].mean()))
    write(output/'planned_coverage.json',report)
    np.savez_compressed(output/'planned_coverage_samples.npz',points=p,labels=labels,photo_supported=photos,views=count)
    print(json.dumps(dict(waypoints=len(route),observations=len(observation_indices),path_m=distance,preflight=report),indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--reference',default=str(ROOT/'half_gs_plans/warehouse_half_20261103'))
    build(p.parse_args())
