"""Freeze real-mesh targets and a known-scene inspection plan before capture."""
import argparse
import heapq
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'integration/roomgs'))
sys.path.insert(0,str(ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'))
from integration.roomgs.warehouse20_audit import raw_reference,intersector,view_mask,sha
from integration.r3live.calibration import read_geometry
from warehouse_scene import blocked
from integration.half_gs.scene import prepare_scene


def save(path,data):Path(path).write_text(json.dumps(data,indent=2)+'\n')


def navigation(meta,actor_keepouts=True,continuous_clearance=False):
    bounds=meta['bounds_world'];step=.25
    xs=np.arange(bounds[0]+.9,bounds[2]-.8,step);ys=np.arange(bounds[1]+.9,bounds[3]-.8,step)
    def permitted(x,y):
        if actor_keepouts and -8.8<x<-7.2 and -7.8<y<-.2:return False
        if actor_keepouts and .2<x<5.8 and -12.9<y<-11.1:return False
        return not blocked(x,y,meta['boxes'],.80)
    free=np.array([[permitted(x,y) for x in xs] for y in ys])
    def index(xy):return (int(np.argmin(abs(ys-xy[1]))),int(np.argmin(abs(xs-xy[0]))))
    home=index(meta['spawn_world'][:2]);pending=[home];component={home}
    edge_cache={}
    def neighbours(a):
        y,x=a
        for dy,dx in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
            b=(y+dy,x+dx)
            if not (0<=b[0]<len(ys) and 0<=b[1]<len(xs) and free[b]):continue
            if dx and dy and not (free[y,b[1]] and free[b[0],x]):continue
            if continuous_clearance:
                key=tuple(sorted([a,b]))
                if key not in edge_cache:
                    from warehouse_scene import blocked_segment
                    # 5mm sampling allowance keeps the declared 0.80m bound
                    # when collinear grid edges are compressed into a segment.
                    edge_cache[key]=not blocked_segment(xs[x],ys[y],xs[b[1]],ys[b[0]],meta['boxes'],.805)
                if not edge_cache[key]:continue
            yield b,step*math.hypot(dx,dy)
    for a in pending:
        for b,_ in neighbours(a):
            if b not in component:component.add(b);pending.append(b)
    def route(a,b):
        if a==b:return [a]
        queue=[(0.,a)];cost={a:0.};parent={}
        while queue:
            _,u=heapq.heappop(queue)
            if u==b:break
            for v,d in neighbours(u):
                distance=cost[u]+d
                if distance<cost.get(v,float('inf')):
                    cost[v]=distance;parent[v]=u
                    heapq.heappush(queue,(distance+step*math.hypot(v[0]-b[0],v[1]-b[1]),v))
        if b not in parent:raise ValueError('Disconnected inspection station')
        result=[b]
        while result[-1]!=a:result.append(parent[result[-1]])
        return result[::-1]
    xy=lambda node:np.array([xs[node[1]],ys[node[0]]])
    return xs,ys,free,component,home,index,route,xy


def build(output):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    class Args:no_flight=True;plan=None;reference_dir=None
    world,meta,_=prepare_scene(output,Args())
    p,n,labels,local_tri,small=raw_reference();pick=np.char.startswith(labels.astype(str),'rack_')
    p,n,labels=p[pick],n[pick],labels[pick]
    import xml.etree.ElementTree as ET
    from integration.r3live.calibration import pose as matrix
    points=[];normals=[];names=[];triangles=[]
    for model in ET.parse(world).findall('.//world/model'):
        if not model.get('name','').startswith('shelf'):continue
        link=model.find('link');visual=link.find('visual');t=matrix(model.findtext('pose'))@matrix(link.findtext('pose'))@matrix(visual.findtext('pose'))
        points.append(p@t[:3,:3].T+t[:3,3]);normals.append(n@t[:3,:3].T)
        names.extend([model.get('name')]*len(p));triangles.append(local_tri@t[:3,:3].T+t[:3,3])
    for box in meta['boxes']:
        if not box['name'].startswith('Untitled'):continue
        yaw=box['yaw'];c,s=math.cos(yaw),math.sin(yaw);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        center=np.array(box['center']);size=np.array(box['size'])
        mesh=trimesh.creation.box(extents=size);triangles.append(mesh.triangles@r.T+center)
        for axis in range(3):
            others=[i for i in range(3) if i!=axis]
            aa,bb=np.meshgrid(*[np.arange(-size[i]/2+.1,size[i]/2,.2) for i in others])
            for sign in [-1,1]:
                q=np.zeros((aa.size,3));q[:,axis]=sign*size[axis]/2;q[:,others[0]]=aa.ravel();q[:,others[1]]=bb.ravel();q=q@r.T+center
                keep=(q[:,2]>.05)&(q[:,2]<2.49);q=q[keep]
                nn=np.zeros(3);nn[axis]=sign;nn=nn@r.T
                points.append(q);normals.append(np.tile(nn,(len(q),1)));names.extend([box['name']]*len(q))
    bounds=meta['bounds_world']
    xx,yy=np.meshgrid(np.arange(bounds[0]+.2,bounds[2]-.1,.2),np.arange(bounds[1]+.2,bounds[3]-.1,.2))
    for z,name,normal in [(0.,'floor',[0,0,1]),(2.5,'ceiling',[0,0,-1])]:
        points.append(np.column_stack([xx.ravel(),yy.ravel(),np.full(xx.size,z)]));normals.append(np.tile(normal,(xx.size,1)));names.extend([name]*xx.size)
    # Ceiling and ground are included in the independent occlusion mesh.
    for z in [-.05,2.55]:
        mesh=trimesh.creation.box(extents=[bounds[2]-bounds[0],bounds[3]-bounds[1],.1]);mesh.apply_translation([(bounds[0]+bounds[2])/2,(bounds[1]+bounds[3])/2,z]);triangles.append(mesh.triangles)
    points=np.concatenate(points);normals=np.concatenate(normals);names=np.array(names);triangles=np.concatenate(triangles)
    ray=intersector(triangles)
    xs,ys,free,connected,home,index,path,xy=navigation(meta)
    nodes=[(y,x) for y in range(0,len(ys),4) for x in range(0,len(xs),4) if (y,x) in connected]
    vehicle=Path('/home/a/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/iris_mid360.sdf')
    g=read_geometry(vehicle,ROOT/'simulation/astra_gazebo_models/mid360/mid360.sdf',ROOT/'simulation/astra_gazebo_models/D435i/model.sdf')
    from integration.roomgs.camera_profiles import profile_projection, G1_PROFILE
    optics=profile_projection(1280,720,G1_PROFILE)
    extrinsic=np.array(g['body_from_camera']);k=np.array(optics['K'])
    masks=[];cameras=[];bodies=[];body_nodes=[];views=np.zeros(len(points),np.int32);first=np.full(points.shape,np.nan);baseline=np.zeros(len(points),bool)
    for j,node in enumerate(nodes):
        x,y=xy(node)
        for z in [.65,1.2,1.8]:
            for yaw in np.arange(8)*math.pi/4:
                c,s=math.cos(yaw),math.sin(yaw);t=np.eye(4);t[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];t[:3,3]=[x,y,z];cam=t@extrinsic
                seen=view_mask(ray,points,normals,cam,k,1280,720)
                new=seen&(views==0);first[new]=cam[:3,3];views+=seen
                ids=np.flatnonzero(seen);baseline[ids]|=np.linalg.norm(first[ids]-cam[:3,3],axis=1)>=.4
                masks.append(np.packbits(seen));cameras.append(cam);bodies.append([x,y,z,yaw]);body_nodes.append(node)
        if j%20==0:print('Capability',j,len(nodes),'targets',len(points),flush=True)
    eligible=(views>=2)&baseline
    packed=np.array(masks);cameras=np.array(cameras);bodies=np.array(bodies)
    reference=output/'mesh_reference.npz'
    np.savez_compressed(reference,points=points[eligible],normals=normals[eligible],labels=names[eligible],triangles=triangles,
        all_points=points,all_normals=normals,all_labels=names,eligible=eligible,capability_camera_poses=cameras,
        capability_body_views=bodies,capability_visibility=packed,body_from_camera=extrinsic)
    manifest=dict(scene='warehouse_half_dynamic',required_fraction=1.,step_m=.2,distance_tolerance_m=.2,
        minimum_views=2,minimum_baseline_m=.4,required_samples=int(eligible.sum()),raw_surface_samples=len(points),
        unobservable_samples=int((~eligible).sum()),eligibility_frozen_before_capture=True,selected_flight_route_used_for_eligibility=False,
        scope='Predetermined externally observable real rack mesh and inner structural 20cm samples, from connected 0.80m-clearance camera lattice; fixed actor patrol keepout envelopes; not hidden/continuous completeness',
        world_sha256=sha(world),reference_sha256=sha(reference),mesh_sha256=sha(ROOT/small['rack']['original_mesh']),
        camera_K=k.tolist(),camera_projection=optics,body_from_camera=extrinsic.tolist(),capability_count=len(cameras))
    save(output/'target_manifest.json',manifest)
    target=int.from_bytes(np.packbits(eligible).tobytes(),'little');visibility=[int.from_bytes(row.tobytes(),'little')&target for row in packed]
    second=[0]*len(visibility);unseen=target;done=0;selected=[];current=np.array([-6.,-4.,1.2,math.pi]);available=np.ones(len(bodies),bool)
    while done!=target:
        benefit=np.array([2*(v&unseen).bit_count()+(v&~done&second[i]).bit_count() if available[i] else 0 for i,v in enumerate(visibility)])
        cost=1+np.linalg.norm(bodies[:,:3]-current[:3],axis=1)*.35+np.abs(np.arctan2(np.sin(bodies[:,3]-current[3]),np.cos(bodies[:,3]-current[3])))*.4
        choice=int(np.argmax(benefit/cost))
        if benefit[choice]==0:raise RuntimeError('Frozen required target cannot be covered by selected views; do not shrink denominator')
        v=visibility[choice];done|=v&second[choice];unseen&=~v;selected.append(choice);available[choice]=False;current=bodies[choice]
        distant=np.linalg.norm(cameras[:,:3,3]-cameras[choice,:3,3],axis=1)>=.4
        for i in np.flatnonzero(distant):second[i]|=v
        if len(selected)%25==0:print('Selected',len(selected),'covered',done.bit_count(),'/',target.bit_count(),flush=True)
    route=[];spawn=np.array(meta['spawn_world']);current_node=home;current_z=1.2;current_yaw=math.pi
    def add(position,z,yaw):route.append([float(position[0]-spawn[0]),float(position[1]-spawn[1]),float(z-spawn[2]),float(yaw)])
    for _ in range(12):add(spawn[:2],1.2,math.pi)
    for choice in selected:
        destination=body_nodes[choice];way=path(current_node,destination)
        # Collapsing collinear grid edges preserves the validated path.
        anchors=[way[0]]
        for i in range(1,len(way)-1):
            if tuple(np.subtract(way[i],way[i-1]))!=tuple(np.subtract(way[i+1],way[i])):anchors.append(way[i])
        if len(way)>1:anchors.append(way[-1])
        for node in anchors[1:]:add(xy(node),current_z,current_yaw)
        x,y,z,yaw=bodies[choice];add([x,y],z,yaw);current_node=destination;current_z=z;current_yaw=yaw
    for node in path(current_node,home)[1::4]:add(xy(node),current_z,current_yaw)
    add(spawn[:2],1.2,0.)
    distance=sum(np.linalg.norm(np.array(a[:3])-b[:3]) for a,b in zip(route,route[1:]))
    plan=dict(waypoints=route,selected_capability_indices=selected,ideal_required_samples=target.bit_count(),ideal_complete=True,
        waypoint_count=len(route),path_length_m=float(distance),spawn_world=spawn.tolist(),**{k:manifest[k] for k in ['world_sha256','mesh_sha256','reference_sha256']},
        policy='Known-scene fixed inspection; A* uses static geometry and declared actor patrol keepouts, not an unknown-scene autonomous exploration result',
        geometry_clearance_m=.8,navigation_speed_mps=.6,mask_source='Learned image model; no simulator mask labels')
    save(output/'inspection.json',plan);print(json.dumps({k:v for k,v in plan.items() if k not in ['waypoints','selected_capability_indices']},indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('output');build(p.parse_args().output)
