"""Private half-warehouse run copy; original world and assets stay intact."""
import json
import hashlib
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'))
from half_warehouse_scene import prepare
from integration.roomgs.lowflow_camera import make_rgb_camera
from integration.roomgs.camera_profiles import G1_PROFILE


def camera(source,models):
    return make_rgb_camera(source,models,width=1280,height=720,hz=3,profile=G1_PROFILE)


def prepare_scene(output,args):
    world=prepare(ROOT,output/'scene',extra_pedestrian=True)
    scene=json.loads((output/'scene_scene.json').read_text());tree=ET.parse(world)
    static=bool(getattr(args,'static',False))
    if static:
        parent=tree.getroot().find('world')
        for node in list(parent):
            if node.tag=='include' and node.findtext('name') in ['0','1']:parent.remove(node)
    mesh_assets=set()
    for model in tree.findall('.//world/model'):
        if not model.get('name','').startswith('shelf'):continue
        for link in model.findall('link'):
            visual=link.find('visual');collision=link.find('collision')
            if visual is None or collision is None:continue
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            # Original half-world uses solid rack boxes, unlike its open visual
            # mesh. Correct only sensor collision in this private run copy.
            import copy
            collision.find('pose').text=visual.findtext('pose','0 0 0 0 0 0')
            collision.remove(collision.find('geometry'));collision.append(copy.deepcopy(visual.find('geometry')))
            mesh_assets.add(Path(mesh.findtext('uri').removeprefix('file://')) if hasattr(str,'removeprefix') else Path(mesh.findtext('uri').replace('file://','',1)))
    tree.write(str(world),encoding='utf-8',xml_declaration=True)
    scene.update(scene='warehouse_half_static_gs' if static else 'warehouse_half_dynamic_gs',actors=[] if static else ['0','1'],collision_policy='Private run rack collisions use same original visual mesh; visual poses/scales/textures and original world files unchanged',
        image_masks='Implicit all-valid in actor-free static scene' if static else 'Image detection/segmentation model only; simulator truth reserved for independent audit',
        mesh_assets=[str(p) for p in sorted(mesh_assets)])
    if static:
        for key in ['pedestrian_model','pedestrian_world','extra_pedestrian']:scene.pop(key,None)
        scene['note']='Original half warehouse with matching mesh collisions; all person includes removed in private copy. No actor controller is launched.'
    (output/'scene_scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    if args.plan:
        plan=json.loads(Path(args.plan).read_text())
    elif args.no_flight:plan=dict(waypoints=[],scope='Stationary sensor smoke only')
    else:raise ValueError('Provide an independently validated half-warehouse inspection plan')
    if args.reference_dir:
        for name in ['mesh_reference.npz','target_manifest.json']:shutil.copyfile(Path(args.reference_dir)/name,output/name)
        if static:
            reference=Path(args.reference_dir)/'scene_cangku_half.world'
            # Changing actors must not silently change the audited warehouse.
            def structure(path):
                root=ET.parse(path).getroot();parent=root.find('world')
                for node in list(parent):
                    if node.tag=='include' and node.findtext('name') in ['0','1']:parent.remove(node)
                def canonical(e):return (e.tag,tuple(sorted(e.attrib.items())),(e.text or '').strip(),tuple(canonical(c) for c in e))
                return canonical(root)
            if structure(reference)!=structure(world):raise ValueError('Static world differs from frozen reference beyond actor removal')
            path=output/'target_manifest.json';manifest=json.loads(path.read_text())
            manifest.update(parent_world_sha256=manifest.get('parent_world_sha256',manifest['world_sha256']),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                actor_free_structural_match=True,scene='warehouse_half_static',
                scope=manifest['scope']+'; unchanged target set reused after verified actor-only removal; no claim about previously excluded hidden faces')
            path.write_text(json.dumps(manifest,indent=2)+'\n')
    return world,scene,plan
