"""Build a 20m² derivative of cangku with one original textured rack.

This is scene preparation only. Metadata is offline geometry, never a SLAM pose
or an exploration input. Original warehouse worlds and rack assets stay intact.
"""
from pathlib import Path
import argparse
import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
INNER_SIZE=(5.,4.)
HEIGHT=3.
THICKNESS=.12


def rack_vertices(mesh,scale):
    """Evaluate this asset's COLLADA instance transforms, not raw source bounds."""
    root=ET.parse(mesh).getroot();ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    points=[]
    for node in root.findall('c:library_visual_scenes/c:visual_scene/c:node',ns):
        matrix=np.fromstring(node.find('c:matrix',ns).text,sep=' ').reshape(4,4)
        for instance in node.findall('c:instance_geometry',ns):
            geometry=root.find("c:library_geometries/c:geometry[@id='%s']"%instance.get('url')[1:],ns)
            source=geometry.find('c:mesh/c:vertices/c:input',ns).get('source')[1:]
            data=geometry.find("c:mesh/c:source[@id='%s']/c:float_array"%source,ns)
            xyz=np.fromstring(data.text,sep=' ').reshape(-1,3)
            points.append((xyz@matrix[:3,:3].T+matrix[:3,3])*scale)
    if not points:raise ValueError('Rack mesh has no instantiated geometry')
    return np.concatenate(points)


def generate(output=None):
    output=Path(output or ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.world').resolve()
    source=ROOT/'simulation/astra_gazebo_worlds/cangku.world'
    original=ET.parse(source).getroot().find('world')
    sdf=ET.Element('sdf',version='1.7');world=ET.SubElement(sdf,'world',name='cangku_20sqm')
    for name in ('light','gravity','magnetic_field','atmosphere','physics','scene','spherical_coordinates'):
        for element in original.findall(name):world.append(copy.deepcopy(element))
    ambient=world.find('scene/ambient')
    if ambient is not None:ambient.text='.65 .65 .65 1'
    gui=ET.SubElement(world,'gui',fullscreen='0');camera=ET.SubElement(gui,'camera',name='user_camera')
    ET.SubElement(camera,'pose').text='-2.2 -1.65 2.6 0 0.50 0.65'
    ET.SubElement(camera,'view_controller').text='orbit'
    shell=ET.SubElement(world,'model',name='warehouse_shell');ET.SubElement(shell,'static').text='true'
    wall_material=original.find("model[@name='Untitled']/link[@name='Wall_0']/visual/material")
    def slab(name,center,size,material=None):
        link=ET.SubElement(shell,'link',name=name)
        ET.SubElement(link,'pose').text=' '.join(map(str,[*center,0,0,0]))
        for kind in ('collision','visual'):
            element=ET.SubElement(link,kind,name=name+'_'+kind)
            geometry=ET.SubElement(element,'geometry');box=ET.SubElement(geometry,'box')
            ET.SubElement(box,'size').text=' '.join(map(str,size))
            if kind=='visual':
                if material is not None:element.append(copy.deepcopy(material))
                else:
                    mat=ET.SubElement(element,'material')
                    ET.SubElement(mat,'ambient').text='.65 .65 .65 1'
                    ET.SubElement(mat,'diffuse').text='.75 .75 .75 1'
    x,y=INNER_SIZE;t=THICKNESS
    slab('floor',[0,0,-t/2],[x+2*t,y+2*t,t])
    slab('ceiling',[0,0,HEIGHT+t/2],[x+2*t,y+2*t,t])
    for sign,name in [(-1,'west'),(1,'east')]:
        slab(name,[sign*(x+t)/2,0,HEIGHT/2],[t,y+2*t,HEIGHT],wall_material)
    for sign,name in [(-1,'south'),(1,'north')]:
        slab(name,[0,sign*(y+t)/2,HEIGHT/2],[x,t,HEIGHT],wall_material)
    rack=copy.deepcopy(original.find("model[@name='shelf_aruco']"))
    if rack is None:raise ValueError('Original cangku rack missing')
    rack.find('pose').text='0 0 0 0 0 0'
    link=rack.find('link');link.find('pose').text='0 0 0 0 0 0'
    visual=link.find('visual');mesh=visual.find('geometry/mesh')
    asset=(source.parent/mesh.findtext('uri')).resolve()
    if not asset.is_file():raise FileNotFoundError(asset)
    uri='model://Shelf_with_ARUco_boxes/meshes/'+asset.name
    mesh.find('uri').text=uri
    # A solid bounding box would make ray measurements fill the open shelves.
    # Static triangle collision instead uses exactly the visible original mesh.
    collision=link.find('collision');collision.find('pose').text=visual.findtext('pose')
    collision.remove(collision.find('geometry'));collision.append(copy.deepcopy(visual.find('geometry')))
    world.append(rack)
    scale=np.fromstring(mesh.findtext('scale'),sep=' ');vertices=rack_vertices(asset,scale)
    low,high=vertices.min(axis=0),vertices.max(axis=0)
    textures=[]
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    for image in ET.parse(asset).findall('.//c:library_images//c:init_from',ns):
        path=asset.parent/image.text
        if not path.is_file():raise FileNotFoundError(path)
        textures.append(str(path.relative_to(ROOT)))
    output.parent.mkdir(parents=True,exist_ok=True)
    pretty=minidom.parseString(ET.tostring(sdf,encoding='utf-8')).toprettyxml(indent='  ')
    output.write_text('\n'.join(line for line in pretty.splitlines() if line.strip())+'\n')
    info=dict(scene='cangku_20sqm',source_world=str(source.relative_to(ROOT)),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        world=str(output.relative_to(ROOT)) if ROOT in output.parents else str(output),
        interior_size_m=[x,y,HEIGHT],interior_floor_area_m2=x*y,
        area_definition='Inside wall faces; not free area after rack footprint',
        bounds_world=[[-x/2,-y/2,0],[x/2,y/2,HEIGHT]],
        shelf_count=1,other_furniture_count=0,actors=0,
        rack=dict(model='shelf_aruco',original_mesh=str(asset.relative_to(ROOT)),scale=scale.tolist(),
            minimum=low.tolist(),maximum=high.tolist(),size_m=(high-low).tolist(),
            collision='Same static triangle mesh as visual; open shelf gaps retained',textures=textures),
        rack_end_to_wall_m=[float(x/2+low[0]),float(x/2-high[0])],
        clear_area_minus_mesh_aabb_m2=float(x*y-(high[0]-low[0])*(high[1]-low[1])),
        suggested_spawn_world=[-1.3,-1.2,.06],
        scope='Scene preparation only; no mapping/flight/3DGS result. Rack AABB is not a completeness reference surface.',
        next_task='Adapt inspection route and mesh-surface audit to this world before R3LIVE/3DGS; old 11x12m route must not be reused')
    output.with_suffix('.json').write_text(json.dumps(info,ensure_ascii=False,indent=2)+'\n')
    return info


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path)
    args=parser.parse_args();print(json.dumps(generate(args.output),ensure_ascii=False,indent=2))
