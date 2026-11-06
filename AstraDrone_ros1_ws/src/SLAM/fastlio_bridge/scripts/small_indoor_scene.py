#!/usr/bin/env python3
"""Prepare the pinned GitHub office; geometry is never an Explorer input."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(root, prefix):
    folder = Path(root)/'simulation/astra_gazebo_worlds/small_office'
    provenance = json.loads((folder/'SOURCE.json').read_text())
    source = folder/'office_small.original.world'
    if hashlib.sha256(source.read_bytes()).hexdigest() != provenance['original_sha256']:
        raise ValueError('Pinned office source changed; review SOURCE.json first')
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    keep = set(provenance['kept_model_names'])
    for child in list(world):
        if child.tag == 'state' or (child.tag == 'model' and child.get('name') not in keep):
            world.remove(child)
    for model in world.findall('model'):
        flag = model.find('static')
        if flag is None:
            flag = ET.SubElement(model, 'static')
        flag.text = 'true'
    for uri in world.findall('.//geometry/mesh/uri'):
        relative = uri.text.removeprefix('model://') if hasattr(str, 'removeprefix') else uri.text[len('model://'):]
        asset = folder/'assets'/relative
        if not asset.is_file():
            raise FileNotFoundError(str(asset))
        uri.text = asset.resolve().as_uri()
    # Geometry and poses stay unchanged. No remote material/model lookup.
    for model in world.findall('model'):
        colour = ('0.80 0.82 0.85 1' if model.get('name') == 'office_small_1' else
                  '0.46 0.28 0.13 1' if model.get('name').startswith('desk') else
                  '0.30 0.42 0.52 1')
        for visual in model.findall('.//visual'):
            material = visual.find('material')
            if material is not None:
                visual.remove(material)
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = colour
            ET.SubElement(material, 'diffuse').text = colour
    physics = world.find('physics')
    for tag, value in [('max_step_size','0.001'),('real_time_update_rate','1000'),('real_time_factor','1')]:
        element = physics.find(tag)
        if element is None:
            element = ET.SubElement(physics, tag)
        element.text = value
    camera = world.find('gui/camera/pose')
    if camera is not None:
        camera.text = '11 -13 12 0 0.65 2.25'
    generated = Path(str(prefix)+'_office.world')
    tree.write(str(generated), encoding='utf-8', xml_declaration=True)
    info = dict(repository=provenance['repository'], commit=provenance['commit'],
                source=str(source), generated_world=str(generated),
                source_sha256=provenance['original_sha256'],
                kept_model_names=sorted(keep), spawn_world=[0.,0.,.06],
                note='Static furnished ~10x10m office. Original selected collision geometry/poses; '
                     'tabletop clutter omitted. No scene geometry is passed to Explorer.')
    Path(str(prefix)+'_scene.json').write_text(json.dumps(info,indent=2)+'\n')
    return generated


def interior_wall_samples(source, step=.20, lower=.25, upper=2.20):
    """Evaluator-only samples of the original room's inward-facing wall boxes.

    Excludes floor, furniture, outward faces and window/door edges. Not full
    visual-mesh completeness and not a sensor/target supplied to the explorer.
    """
    import math
    import numpy as np
    world = ET.parse(source).getroot().find('world')
    outer = world.find("model[@name='office_small_1']")
    room = outer.find("model[@name='office_small']")
    def pose(el):
        return np.asarray([float(v) for v in el.findtext('pose','0 0 0 0 0 0').split()])
    base = pose(outer)+pose(room)
    samples, planes = [], []
    for link in room.findall('link'):
        p = pose(link)
        c,s = math.cos(p[5]),math.sin(p[5])
        for collision in link.findall('collision'):
            size = np.fromstring(collision.findtext('geometry/box/size'),sep=' ')
            q = pose(collision)
            lo,hi = max(lower,q[2]-size[2]/2),min(upper,q[2]+size[2]/2)
            if hi<=lo:
                continue
            # Walls run along local x; use the local y face pointing inward.
            normal_world = np.asarray([-s,c])
            center_world = base[:2]+p[:2]+np.asarray([c*q[0]-s*q[1],s*q[0]+c*q[1]])
            sign = -1. if np.dot(normal_world,center_world-base[:2])>0 else 1.
            planes.append((center_world+normal_world*sign*size[1]/2,normal_world*sign))
            x = np.linspace(q[0]-size[0]/2,q[0]+size[0]/2,max(2,int(math.ceil(size[0]/step))+1))
            z = np.linspace(lo,hi,max(2,int(math.ceil((hi-lo)/step))+1))
            xx,zz = np.meshgrid(x,z)
            yy = q[1]+sign*size[1]/2
            samples.append(np.column_stack((base[0]+p[0]+c*xx.ravel()-s*yy,
                                             base[1]+p[1]+s*xx.ravel()+c*yy,
                                             base[2]+p[2]+zz.ravel())))
    points = np.vstack(samples)
    # Clip corners behind the adjoining wall; these are not interior surfaces.
    for center, normal in planes:
        points = points[((points[:,:2]-center)@normal)>=-1e-5]
    return np.unique(np.round(points,6),axis=0)
