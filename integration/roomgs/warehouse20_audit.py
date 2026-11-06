"""Offline mesh surface reference and strict audit; never imported by SLAM/GS."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from trimesh.ray.ray_pyembree import RayMeshIntersector
from scipy.spatial import cKDTree
from scipy.spatial.distance import pdist
from geometry import read_pcd,transform

ROOT=Path(__file__).resolve().parents[2]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2)+'\n')


def triangles_from_dae(path,scale):
    root=ET.parse(path).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    triangles=[];labels=[]
    for node in root.findall('c:library_visual_scenes/c:visual_scene/c:node',ns):
        matrix=np.fromstring(node.find('c:matrix',ns).text,sep=' ').reshape(4,4)
        for instance in node.findall('c:instance_geometry',ns):
            mesh=root.find("c:library_geometries/c:geometry[@id='%s']/c:mesh"%instance.get('url')[1:],ns)
            source=mesh.find('c:vertices/c:input',ns).get('source')[1:]
            xyz=np.fromstring(mesh.find("c:source[@id='%s']/c:float_array"%source,ns).text,sep=' ').reshape(-1,3)
            xyz=(xyz@matrix[:3,:3].T+matrix[:3,3])*scale
            for faces in mesh.findall('c:triangles',ns):
                inputs=faces.findall('c:input',ns);stride=max(int(i.get('offset')) for i in inputs)+1
                offset=next(int(i.get('offset')) for i in inputs if i.get('semantic')=='VERTEX')
                indices=np.fromstring(faces.find('c:p',ns).text,sep=' ',dtype=int).reshape(-1,stride)[:,offset]
                tri=xyz[indices.reshape(-1,3)];triangles.extend(tri)
                labels.extend(['rack_'+node.get('name')]*len(tri))
    return np.array(triangles),np.array(labels)


def raw_reference():
    meta=json.loads((ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.json').read_text())
    mesh=ROOT/meta['rack']['original_mesh'];tri,labels=triangles_from_dae(mesh,np.array(meta['rack']['scale']))
    p=[];n=[];names=[]
    # Triangle-lattice samples have maximum edge spacing 20cm; small details
    # retain at least their triangle centroid. No bounding-box replacement.
    for t,label in zip(tri,labels):
        normal=np.cross(t[1]-t[0],t[2]-t[0]);length=np.linalg.norm(normal)
        if length<1e-12:continue
        normal/=length;count=max(1,int(np.ceil(max(np.linalg.norm(t[i]-t[j]) for i,j in [(0,1),(1,2),(2,0)])/.20)))
        for i in range(count):
            for j in range(count-i):
                u=(i+1/3)/count;v=(j+1/3)/count
                p.append(t[0]+u*(t[1]-t[0])+v*(t[2]-t[0]));n.append(normal);names.append(label)
    # Six structural inner faces, retaining the predeclared 20cm grid.
    bounds=np.array([[-2.5,-2,0],[2.5,2,3.]])
    for axis,negative,positive in [(0,'west','east'),(1,'south','north'),(2,'floor','ceiling')]:
        axes=[a for a in range(3) if a!=axis]
        aa,bb=np.meshgrid(*[np.arange(bounds[0,a]+.10,bounds[1,a],.20) for a in axes],indexing='ij')
        for end,label in [(0,negative),(1,positive)]:
            vertices=np.zeros((aa.size,3));vertices[:,axis]=bounds[end,axis]
            vertices[:,axes[0]]=aa.ravel();vertices[:,axes[1]]=bb.ravel()
            normal=np.zeros(3);normal[axis]=1 if end==0 else -1
            p.extend(vertices);n.extend([normal]*len(vertices));names.extend([label]*len(vertices))
    points=np.array(p);normals=np.array(n);names=np.array(names)
    # One representative per 20cm surface cell, normal and component. Long,
    # thin triangulated rods otherwise receive thousands of redundant samples
    # inside a single nominal cell. Selection never consults visibility/data.
    keys=np.column_stack([np.floor(points/.20),np.round(normals,2),
                          np.unique(names,return_inverse=True)[1]])
    center=(np.floor(points/.20)+.5)*.20
    order=np.argsort(np.linalg.norm(points-center,axis=1),kind='stable')
    _,selected=np.unique(keys[order],axis=0,return_index=True)
    ids=np.sort(order[selected])
    return points[ids],normals[ids],names[ids],tri,meta


def intersector(tri):
    mesh=trimesh.Trimesh(vertices=tri.reshape(-1,3),faces=np.arange(tri.size//3).reshape(-1,3),process=False)
    return RayMeshIntersector(mesh)


def unobstructed(ray,p,n,origins):
    origins=np.broadcast_to(origins,p.shape).copy();endpoint=p+n*.003
    direction=endpoint-origins;distance=np.linalg.norm(direction,axis=1)
    hits,ids,_=ray.intersects_location(origins,direction,multiple_hits=False)
    seen=np.ones(len(p),bool)
    if len(ids):seen[ids]=np.linalg.norm(hits-origins[ids],axis=1)>=distance[ids]-.01
    return seen


def view_mask(ray,p,n,pose,k,width,height):
    relative=p-pose[:3,3];camera=relative@pose[:3,:3]
    depth=camera[:,2];uv=camera[:,:2]/np.maximum(depth[:,None],.001)
    uv=uv@np.diag([k[0,0],k[1,1]])+np.array([k[0,2],k[1,2]])
    facing=np.sum(n*(-relative),axis=1)/np.maximum(np.linalg.norm(relative,axis=1),1e-8)
    seen=(depth>.5)&(depth<18)&(facing>math.cos(math.radians(82)))
    seen&=(uv[:,0]>=8)&(uv[:,0]<width-8)&(uv[:,1]>=8)&(uv[:,1]<height-8)
    ids=np.flatnonzero(seen)
    if len(ids):seen[ids]&=unobstructed(ray,p[ids],n[ids],pose[:3,3])
    return seen


def has_translation_baseline(centers):
    """Any observed pair, not only distances to the first (possibly middle) view."""
    if len(centers)<2:return False
    extent=np.ptp(centers,axis=0)
    # Nanometre tolerance only compensates round trips between world/home
    # coordinates. It does not relax the physical 40cm criterion.
    threshold=.4-1e-9
    if extent.max()>=threshold:return True
    if np.linalg.norm(extent)<threshold:return False
    return bool(np.max(pdist(centers))>=threshold)


def prepare(output):
    output=Path(output);p,n,labels,tri,meta=raw_reference();ray=intersector(tri)
    # Immutable capability envelope, independent of the selected flight route
    # and all acquired images. No missed observation can remove a target later.
    # Vehicle centre has >=0.45m structural clearance and stays outside the
    # rack's expanded AABB below its top; overhead transfer plane is 2.35m.
    first=np.full(p.shape,np.nan);baseline=np.zeros(len(p),bool);views=np.zeros(len(p),int)
    positions=[]
    for x in np.linspace(-1.95,1.95,17):
        for y in np.linspace(-1.45,1.45,13):
            for z in np.r_[np.arange(.45,2.06,.2),2.35,2.45]:
                if abs(x)<2.26 and abs(y)<.80 and z<2.28:continue
                positions.append([x,y,z])
    sys.path.insert(0,str(ROOT))
    from integration.r3live.calibration import read_geometry
    vehicle=Path(os.environ.get('PX4_DIR','/home/a/PX4-Autopilot'))/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/iris_mid360.sdf'
    geometry=read_geometry(vehicle,ROOT/'simulation/astra_gazebo_models/mid360/mid360.sdf',ROOT/'simulation/astra_gazebo_models/D435i/model.sdf')
    optical=np.array(geometry['body_from_camera']);k=np.array([[462.26636,0,320],[0,462.26636,240],[0,0,1.]])
    masks=[];camera_poses=[];body_views=[]
    for index,body in enumerate(np.array(positions)):
        for yaw in np.arange(16)*math.pi/8:
            c,s=math.cos(yaw),math.sin(yaw);body_matrix=np.eye(4)
            body_matrix[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];body_matrix[:3,3]=body
            pose=body_matrix@optical;seen=view_mask(ray,p,n,pose,k,640,480)
            new=seen&(views==0);first[new]=pose[:3,3];views+=seen
            ids=np.flatnonzero(seen);baseline[ids]|=np.linalg.norm(first[ids]-pose[:3,3],axis=1)>=.4
            masks.append(seen);camera_poses.append(pose);body_views.append([*body,float(yaw)])
        if index%200==0:print('Capability station',index,len(positions),flush=True)
    masks=np.array(masks);camera_poses=np.array(camera_poses)
    for idx in np.flatnonzero((views>=2)&~baseline):
        baseline[idx]=has_translation_baseline(camera_poses[masks[:,idx],:3,3])
    eligible=(views>=2)&baseline
    path=output/'mesh_reference.npz'
    np.savez_compressed(path,points=p[eligible],normals=n[eligible],labels=labels[eligible],triangles=tri,
        all_points=p,all_normals=n,all_labels=labels,eligible=eligible,capability_positions=np.array(positions),
        capability_camera_poses=camera_poses,capability_body_views=np.array(body_views),
        capability_visibility=np.packbits(masks[:,eligible],axis=1),body_from_camera=optical)
    report=dict(scene='warehouse20',required_fraction=1.,step_m=.20,distance_tolerance_m=.20,
        raw_surface_samples=len(p),required_samples=int(eligible.sum()),unobservable_samples=int((~eligible).sum()),
        capability_body_positions=len(positions),capability_yaw='16 yaw directions per station; actual camera extrinsics and intrinsics',
        sample_policy='One actual triangle sample per 20cm voxel, normal and component, nearest voxel centre; selected before visibility',
        camera_K=k.tolist(),body_from_camera=optical.tolist(),minimum_views=2,minimum_baseline_m=.4,
        scope='All predetermined externally observable structural and real rack triangle samples; excludes buried/occluded interiors and surfaces not visible from the declared safe camera capability lattice',
        eligibility_frozen_before_capture=True,selected_flight_route_used_for_eligibility=False,
        world_sha256=sha(ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.world'),
        mesh_sha256=sha(ROOT/meta['rack']['original_mesh']),reference_sha256=sha(path))
    write(output/'target_manifest.json',report);return report


def audit(output,map_filename='rgb_pt.pcd',estimator='R3LIVE'):
    output=Path(output);definition=json.loads((output/'target_manifest.json').read_text())
    if sha(output/'mesh_reference.npz')!=definition['reference_sha256']:raise RuntimeError('Preflight mesh targets changed')
    data=np.load(output/'mesh_reference.npz');p,n,labels=[data[k] for k in ('points','normals','labels')]
    ray=intersector(data['triangles']);alignment=np.array(json.loads((output/'audit_alignment.json').read_text())['world_from_estimator'])
    frames=json.loads((output/'dataset/frames.json').read_text());info=json.loads((output/'camera_input.json').read_text());k=np.array(info['K']).reshape(3,3)
    views=np.zeros(len(p),int);baseline=np.zeros(len(p),bool);first=np.full(p.shape,np.nan);seen_masks=[];centers=[]
    for row in frames:
        pose=alignment@np.array(row['c2w']);seen=view_mask(ray,p,n,pose,k,info['width'],info['height'])
        new=seen&(views==0);first[new]=pose[:3,3];views+=seen;ids=np.flatnonzero(seen)
        baseline[ids]|=np.linalg.norm(first[ids]-pose[:3,3],axis=1)>=.4
        seen_masks.append(seen);centers.append(pose[:3,3])
    seen_masks=np.array(seen_masks);centers=np.array(centers)
    for idx in np.flatnonzero((views>=2)&~baseline):
        baseline[idx]=has_translation_baseline(centers[seen_masks[:,idx]])
    if map_filename.endswith('.npz'):
        xyz=np.load(output/map_filename)['xyz']
    else:
        xyz,_=read_pcd(output/map_filename)
    distance=cKDTree(transform(xyz,alignment)).query(p)[0]
    geometry=distance<=.20;photos=(views>=2)&baseline;complete=geometry&photos
    report=dict(passed=bool(complete.all()),required_fraction=1.,total_samples=len(p),missing_samples=int((~complete).sum()),
        geometry_fraction=float(geometry.mean()),multiview_fraction=float(photos.mean()),complete_fraction=float(complete.mean()),
        step_m=.20,distance_tolerance_m=.20,minimum_views=2,minimum_translation_baseline_m=.4,
        scope=definition['scope'],truth_usage='Mesh and single frozen initial alignment used OFFLINE only; actual '+estimator+' outputs supply dataset',
        groups={str(label):dict(total=int((labels==label).sum()),geometry=int(geometry[labels==label].sum()),multiview=int(photos[labels==label].sum()),complete=int(complete[labels==label].sum())) for label in sorted(set(labels))},
        input_sha256={name:sha(output/name) for name in (map_filename,'dataset/frames.json','camera_input.json','target_manifest.json','mesh_reference.npz','audit_alignment.json')})
    np.savez_compressed(output/'coverage_samples.npz',points=p,normals=n,labels=labels,nearest_map_distance=distance,views=views,baseline=baseline,complete=complete)
    write(output/'coverage.json',report)
    write(output/'missing_surfaces.json',[dict(xyz=a.tolist(),normal=b.tolist(),face=str(c),distance=float(d),views=int(e),baseline=bool(f)) for a,b,c,d,e,f in zip(p[~complete],n[~complete],labels[~complete],distance[~complete],views[~complete],baseline[~complete])])
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--prepare');parser.add_argument('--capture')
    args=parser.parse_args();print(json.dumps(prepare(args.prepare) if args.prepare else audit(args.capture),indent=2),flush=True)
