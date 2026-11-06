#!/usr/bin/env python3
"""50.4m2 room with pinned GitHub furniture; scene truth never enters planning.

Furniture meshes/collisions retain native scale; only top-level poses change.
COLLADA bounds are conservative actor-guard/footprint bounds, not mesh surfaces.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def transform_pose(text):
    p=np.fromstring(text or '0 0 0 0 0 0',sep=' ')
    if len(p)!=6 or not np.isfinite(p).all() or np.any(np.abs(p[3:5])>1e-7):
        raise ValueError('expected upright finite SDF pose')
    c,s=math.cos(p[5]),math.sin(p[5]);t=np.eye(4)
    t[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];t[:3,3]=p[:3]
    return t


def mesh_vertices(path):
    root=ET.parse(path).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    if root.findtext('c:asset/c:up_axis',namespaces=ns)!='Z_UP':
        raise ValueError('unreviewed COLLADA up axis')
    geometry={}
    for g in root.findall('c:library_geometries/c:geometry',ns):
        mesh=g.find('c:mesh',ns);position=mesh.find("c:vertices/c:input[@semantic='POSITION']",ns)
        src=mesh.find("c:source[@id='%s']/c:float_array"%position.get('source').lstrip('#'),ns)
        geometry[g.get('id')]=np.fromstring(src.text,sep=' ').reshape(-1,3)
    output=[]
    def walk(node,parent):
        matrix=parent.copy()
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag=='matrix':matrix=matrix@np.fromstring(child.text,sep=' ').reshape(4,4)
            elif tag in ('translate','rotate','scale'):raise ValueError('unreviewed COLLADA transform')
        for instance in node.findall('c:instance_geometry',ns):
            xyz=geometry[instance.get('url').lstrip('#')]
            output.append(xyz@matrix[:3,:3].T+matrix[:3,3])
        for child in node.findall('c:node',ns):walk(child,matrix)
    scene_id=root.find('c:scene/c:instance_visual_scene',ns).get('url').lstrip('#')
    scene=root.find("c:library_visual_scenes/c:visual_scene[@id='%s']"%scene_id,ns)
    for node in scene.findall('c:node',ns):walk(node,np.eye(4))
    unit=float(root.find('c:asset/c:unit',ns).get('meter','1'))
    points=np.vstack(output)*unit
    if not np.isfinite(points).all():raise ValueError('nonfinite mesh')
    return points


def model_bounds(model,assets):
    points=[]
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            transform=transform_pose(link.findtext('pose'))@transform_pose(collision.findtext('pose'))
            geometry=collision.find('geometry');mesh=geometry.find('mesh')
            if mesh is not None:
                relative=mesh.findtext('uri')[len('model://'):]
                xyz=mesh_vertices(assets/relative)
                xyz=xyz*np.fromstring(mesh.findtext('scale','1 1 1'),sep=' ')
            else:
                size=np.fromstring(geometry.findtext('box/size'),sep=' ')/2
                xyz=np.array([[x,y,z] for x in [-size[0],size[0]] for y in [-size[1],size[1]] for z in [-size[2],size[2]]])
            points.append(xyz@transform[:3,:3].T+transform[:3,3])
    xyz=np.vstack(points);lo=xyz.min(axis=0);hi=xyz.max(axis=0)
    if np.any(hi-lo>3.5) or hi[2]-lo[2]<.1:raise ValueError('implausible furniture bounds')
    return lo,hi


def add_box(world,name,center,size,color):
    model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,[*center,0,0,0]))
    link=ET.SubElement(model,'link',name='body')
    for tag in ['collision','visual']:
        item=ET.SubElement(link,tag,name=tag);g=ET.SubElement(item,'geometry')
        ET.SubElement(ET.SubElement(g,'box'),'size').text=' '.join(map(str,size))
        if tag=='visual':
            material=ET.SubElement(item,'material')
            for channel in ['ambient','diffuse']:ET.SubElement(material,channel).text=color
    return dict(name=name+'/body',center=list(center),size=list(size),yaw=0.)


def prepare(root,prefix):
    root=Path(root);folder=root/'simulation/astra_gazebo_worlds/small_office';assets=folder/'assets'
    provenance=json.loads((folder/'SOURCE.json').read_text());source=folder/'office_small.original.world'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=provenance['original_sha256']:
        raise ValueError('pinned office source changed')
    for row in provenance['assets']:
        if hashlib.sha256((folder/row['path']).read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('pinned furniture mesh changed')
    original=ET.parse(source).getroot().find('world')
    sdf=ET.Element('sdf',version='1.6');world=ET.SubElement(sdf,'world',name='default')
    for tag in ['light','gravity','magnetic_field','atmosphere','physics']:
        if original.find(tag) is not None:world.append(copy.deepcopy(original.find(tag)))
    physics=world.find('physics')
    for tag,value in [('max_step_size','0.001'),('real_time_factor','1'),('real_time_update_rate','1000')]:
        el=physics.find(tag)
        if el is None:el=ET.SubElement(physics,tag)
        el.text=value
    ground=copy.deepcopy(original.find("model[@name='ground_plane']"));world.append(ground)
    for el in ground.findall('.//geometry/plane/size'):el.text='7.6 7.4'
    boxes=[]
    for name,c,size in [('wall_west',[-3.7,0,1.5],[.2,7.4,3]),('wall_east',[3.7,0,1.5],[.2,7.4,3]),
                        ('wall_south',[0,-3.6,1.5],[7.2,.2,3]),('wall_north',[0,3.6,1.5],[7.2,.2,3])]:
        boxes.append(add_box(world,name,c,size,'0.8 0.82 0.85 1'))
    add_box(world,'roof',[0,0,3.1],[7.6,7.4,.2],'0.85 0.86 0.88 1')
    ET.SubElement(world.find("model[@name='roof']/link/visual"),'transparency').text='0.95'
    layout=[('desk_brown','desk_nw',-2.3,2.65,0),('desk_brown','desk_se',2.3,-2.75,0),
        ('file_cabinet_small','cabinet_sw',-2.95,-2.65,0),('file_cabinet_large','cabinet_ne',2.8,3.,0),
        ('file_cabinet_large','cabinet_west',-1.,3.,0),('file_cabinet_small','cabinet_east',3.05,0.,0),
        ('sofa_set_2','sofa_north',.5,2.95,0),('office_chair','chair_nw',-2.3,1.55,0),
        ('office_chair','chair_se',2.4,-1.5,math.pi),('coffee_table_set_1','coffee_table',-2.95,-.05,0)]
    furniture=[]
    for source_name,name,x,y,yaw in layout:
        model=copy.deepcopy(original.find("model[@name='%s']"%source_name));lo,hi=model_bounds(model,assets)
        size=hi-lo;c=(lo+hi)/2;rot=transform_pose('0 0 0 0 0 '+str(yaw))[:3,:3]
        target=np.array([x,y,-lo[2]])-rot@np.array([c[0],c[1],0.])
        model.set('name',name);model.find('pose').text=' '.join(map(str,[*target,0,0,yaw]))
        flag=model.find('static')
        if flag is None:flag=ET.SubElement(model,'static')
        flag.text='true'
        for uri in model.findall('.//geometry/mesh/uri'):uri.text=(assets/uri.text[len('model://'):]).resolve().as_uri()
        for visual in model.findall('.//visual'):
            old=visual.find('material')
            if old is not None:visual.remove(old)
            material=ET.SubElement(visual,'material')
            color='0.46 0.28 0.13 1' if 'desk' in name or 'table' in name else '0.28 0.42 0.52 1'
            for channel in ['ambient','diffuse']:ET.SubElement(material,channel).text=color
        world.append(model)
        box=dict(name=name+'/bounds',center=[x,y,float(size[2]/2)],size=size.tolist(),yaw=yaw)
        boxes.append(box);furniture.append(dict(name=name,source_model=source_name,**{k:box[k] for k in ['center','size','yaw']}))
        extent=np.abs(rot[:2,:2])@size[:2]/2
        if x-extent[0]<-3.6 or x+extent[0]>3.6 or y-extent[1]<-3.5 or y+extent[1]>3.5:
            raise ValueError('Furniture crosses room boundary: '+name)
    for name,c,size in [('bookcase_west',[-3.15,1.25,.9],[.35,1.4,1.8]),
                        ('bookcase_east',[3.15,1.4,.9],[.35,1.2,1.8])]:
        boxes.append(add_box(world,name,c,size,'0.38 0.22 0.1 1'))
        furniture.append(dict(name=name,source_model='new primitive bookcase',center=c,size=size,yaw=0.))
    person=ET.SubElement(world,'include');ET.SubElement(person,'uri').text='model://person_standing'
    ET.SubElement(person,'name').text='1';ET.SubElement(person,'pose').text='1.1 -0.2 0 0 0 0'
    gui=ET.SubElement(world,'gui');camera=ET.SubElement(gui,'camera',name='user_camera')
    ET.SubElement(camera,'pose').text='9 -10 10 0 0.7 2.25'
    spec=dict(model_name='1',spawn_world=[1.1,-.2,0],waypoints_world_xy=[[1.1,1.2],[1.1,-.2]],
        duration_sim_seconds=60.,speed_mps=.45,trigger='first observed UAV body height above 0.9m',
        scope='Actor-only controller/ground truth; not UAV input')
    from warehouse_scene import blocked,blocked_segment
    spawn=[-1.2,-1.8,.06]
    if blocked(*spawn[:2],boxes,.95):raise ValueError('Office UAV spawn lacks existing safety clearance')
    for a,b in zip(spec['waypoints_world_xy'],spec['waypoints_world_xy'][::-1]):
        if blocked_segment(*a,*b,boxes,.35):raise ValueError('Office pedestrian path obstructed')
    path=Path(str(prefix)+'_office50.world');path.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(sdf).write(path,encoding='utf-8',xml_declaration=True)
    info=dict(repository=provenance['repository'],commit=provenance['commit'],license='GPL-3.0; original file retained in small_office/LICENSE',
        source=str(source),source_sha256=provenance['original_sha256'],generated_world=str(path),
        scene_type='office-50',layout_revision=2,nominal_size_m=[7.2,7.0],nominal_area_m2=50.4,bounds_world=[-3.6,-3.5,3.6,3.5],
        spawn_world=spawn,boxes=boxes,furniture=furniture,static_furniture_count=len(furniture),
        pedestrian_model='person_standing',pedestrian_world=None,extra_pedestrian=spec,
        note='New 50.4m2 sealed room using unscaled upstream furniture meshes/collisions at new poses. '
             'Furniture enclosing boxes are evaluator/actor guards only, not mesh surface truth or UAV inputs. '
             'One translated/turned standing person (no skeletal gait), moves 60 sim seconds after takeoff then stops.')
    Path(str(prefix)+'_scene.json').write_text(json.dumps(info,indent=2)+'\n')
    return path


def evaluate(prefix,points,alignment):
    from dual_map_contract import geometry_disagreement
    r=np.asarray(alignment['map_from_world_rotation']);t=np.asarray(alignment['map_from_world_translation'])
    world=(points-t)@r;scene=json.loads(Path(str(prefix)+'_scene.json').read_text());targets=[]
    for axis,coordinate,lo,hi in [(0,-3.6,-3.5,3.5),(0,3.6,-3.5,3.5),(1,-3.5,-3.6,3.6),(1,3.5,-3.6,3.6)]:
        q,z=np.meshgrid(np.arange(lo+.1,hi,.2),np.arange(.25,2.21,.2));a=np.zeros((q.size,3))
        a[:,axis]=coordinate;a[:,1-axis]=q.ravel();a[:,2]=z.ravel();targets.append(a)
    ref=np.vstack(targets);missing,_,meta=geometry_disagreement(ref,world,.25,max_points=500000)
    report=dict(scope='New room inward walls z.25..2.20m only; no furniture-mesh surface/full3D claim',
        reference_samples=len(ref),supported_samples=len(ref)-len(missing),inward_wall_support_ratio=1-len(missing)/len(ref),
        euclidean_tolerance_m=.25,sampling_step_m=.2,full_3d_map_complete=False,map_points=len(points))
    Path(str(prefix)+'_wall_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report,missing@r.T+t
