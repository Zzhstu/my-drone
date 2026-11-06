#!/usr/bin/env python3
"""Crop the original warehouse's long axis to half area; never scale racks."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from warehouse_scene import collision_boxes, pose, blocked

SOURCE_SHA256 = '2ea8635e9f1cf6e50e7cb7238eb98e4abcec2e2523683a5304a1307b39736f11'


def generate(root, output, metadata):
    root, output, metadata = Path(root), Path(output), Path(metadata)
    source = root/'simulation/astra_gazebo_worlds/cangku.world'
    if hashlib.sha256(source.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError('Original cangku changed; review crop geometry before regenerating')
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    for state in world.findall('state'):
        world.remove(state)
    shell = world.find("model[@name='Untitled']")
    roof = shell.find("link[@name='Floor_0']")
    original = [float(v) for v in roof.findtext('collision/geometry/box/size').split()]
    length, width = original[:2]
    shift = length/4
    roof_pose = pose(roof)
    shell_pose = pose(shell)
    left = shell_pose[0]+roof_pose[0]-length/2
    bottom = shell_pose[1]+roof_pose[1]-width/2
    bounds = [left, bottom, left+length/2, bottom+width]
    # Resize both visual and collision boxes, retaining height and wall thickness.
    for name in ('Floor_0', 'Wall_1', 'Wall_3'):
        link = shell.find("link[@name='%s']" % name)
        p = pose(link);p[0] -= shift
        link.find('pose').text = ' '.join(format(v,'.12g') for v in p)
        for size in link.findall('./*/geometry/box/size'):
            values = [float(v) for v in size.text.split()]
            values[0] -= length/2
            size.text = ' '.join(format(v,'.12g') for v in values)
    right = shell.find("link[@name='Wall_2']")
    p = pose(right);p[0] -= length/2
    right.find('pose').text = ' '.join(format(v,'.12g') for v in p)
    kept, omitted = [], []
    for model in list(world.findall('model')):
        if not model.get('name','').startswith('shelf'):
            continue
        box = collision_boxes(_model_world(model))[0]
        xy = corners(box)
        # Keep whole racks in the retained left half, with no shifts or scaling.
        if (xy[:,0].min() >= bounds[0]+.15 and xy[:,0].max() <= bounds[2]-.15 and
                xy[:,1].min() >= bounds[1]+.15 and xy[:,1].max() <= bounds[3]-.15):
            kept.append(model.get('name'))
        else:
            omitted.append(model.get('name'));world.remove(model)
    for uri in world.findall('.//geometry/mesh/uri'):
        if uri.text.startswith('../'):
            path = (source.parent/uri.text).resolve()
            if not path.is_file():
                raise FileNotFoundError(str(path))
            # Portable checked-in world when output is beside the original.
            import os
            uri.text = os.path.relpath(path, output.parent)
    camera = world.find('gui/camera/pose')
    if camera is not None:
        camera.text = '-28 -31 34 0 0.68 0.75'
    boxes = collision_boxes(world)
    if blocked(-6,-4,boxes,.95):
        raise ValueError('Retained original spawn is obstructed')
    output.parent.mkdir(parents=True,exist_ok=True)
    tree.write(str(output),encoding='utf-8',xml_declaration=True)
    info = dict(source=str(source),source_sha256=SOURCE_SHA256,generated_world=str(output),
                original_nominal_size_m=[length,width],nominal_size_m=[length/2,width],
                original_nominal_area_m2=length*width,nominal_area_m2=length*width/2,
                area_ratio=.5,bounds_world=bounds,spawn_world=[-6.,-4.,.06],
                kept_shelves=kept,omitted_shelves=omitted,boxes=boxes,
                note='Nominal area is roof XY rectangle, not navigable free area. '
                     'Retained rack poses/dimensions, left entrance, wall thickness and roof height unchanged. '
                     'Roof/long walls shortened and right wall moved; no uniform scaling. '
                     'Scene/evaluation geometry only; not an Explorer input.')
    metadata.write_text(json.dumps(info,indent=2)+'\n')
    return output


def _model_world(model):
    world = ET.Element('world');world.append(model)
    return world


def corners(box):
    import math
    x,y = np.asarray(box['size'][:2])/2
    local = np.array([[-x,-y],[x,-y],[x,y],[-x,y]])
    c,s = math.cos(box['yaw']),math.sin(box['yaw'])
    return local@np.array([[c,s],[-s,c]])+box['center'][:2]


def prepare(root, prefix, extra_pedestrian=False, extra_pedestrian_duration=180.):
    if not np.isfinite(extra_pedestrian_duration) or extra_pedestrian_duration<=0:
        raise ValueError('Extra pedestrian duration must be finite and positive')
    world = generate(root,Path(str(prefix)+'_cangku_half.world'),Path(str(prefix)+'_scene.json'))
    tree = ET.parse(world)
    for uri in tree.findall('.//geometry/mesh/uri'):
        uri.text = (world.parent/uri.text).resolve().as_uri()
    sdf_world = tree.getroot().find('world')
    # Match the existing cangku demo: insert one standing Gazebo person only
    # in this generated run copy; the checked-in half-world stays geometry-only.
    if any(node.findtext('name') == '0' for node in sdf_world.findall('include')):
        raise ValueError('Generated half warehouse already contains pedestrian model 0')
    # Match the exploration-walk setup used by the full cangku demo: near the
    # UAV start so the actor is observed while stationary, then walk the left aisle.
    pedestrian_xy = (-8.0, -4.0)
    boxes = collision_boxes(sdf_world)
    if blocked(*pedestrian_xy, boxes, .35):
        raise ValueError('Half-warehouse pedestrian spawn is obstructed')
    person = ET.SubElement(sdf_world, 'include')
    ET.SubElement(person, 'uri').text = 'model://person_standing'
    ET.SubElement(person, 'name').text = '0'
    ET.SubElement(person, 'pose').text = '-8 -4 0 0 0 0'
    extra_spec=None
    if extra_pedestrian:
        from warehouse_scene import blocked_segment
        extra_spec=dict(model_name='1',spawn_world=[3.,-12.,0.],
            waypoints_world_xy=[[1.,-12.],[5.,-12.]],
            duration_sim_seconds=float(extra_pedestrian_duration),speed_mps=.45,
            trigger='first observed UAV body height above 0.9m',
            display_region='right middle in x=-world_y, y=world_x view',
            scope='Simulator actor control/evaluator only; never planner input')
        route=[extra_spec['spawn_world'][:2]]+extra_spec['waypoints_world_xy']
        for a,b in zip(route,route[1:]+route[:1]):
            if blocked_segment(*a,*b,boxes,.35):raise ValueError('Extra pedestrian route obstructed')
        person=ET.SubElement(sdf_world,'include')
        ET.SubElement(person,'uri').text='model://person_standing'
        ET.SubElement(person,'name').text='1'
        ET.SubElement(person,'pose').text='3 -12 0 0 0 0'
    tree.write(str(world),encoding='utf-8',xml_declaration=True)
    metadata_path = Path(str(prefix)+'_scene.json')
    metadata = json.loads(metadata_path.read_text(encoding='utf8'))
    metadata['pedestrian_model'] = 'person_standing'
    metadata['pedestrian_world'] = [-8.0, -4.0, 0.0]
    if extra_spec is not None:metadata['extra_pedestrian']=extra_spec
    metadata['note'] += ' A single standing pedestrian is inserted in the generated runtime copy only; '
    metadata['note'] += 'its collision-box guard controls the actor, not UAV perception or planning.'
    if extra_spec is not None:
        metadata['note'] += (' Extra actor 1 patrols the right-middle aisle for '
                             '{:g} simulation seconds after takeoff.').format(extra_pedestrian_duration)
    metadata_path.write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf8')
    return world
