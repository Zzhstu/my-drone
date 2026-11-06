"""Private camera-free SDFs for LiDAR-only demonstrations; stock models intact."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare_camera_free_vehicle(root, px4, prefix):
    root,px4,prefix=Path(root),Path(px4),Path(prefix)
    models=px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
    source=models/'iris_mid360/iris_mid360.sdf'
    tree=ET.parse(str(source));vehicle=tree.getroot().find('model')
    directory=Path(str(prefix)+'_models');directory.mkdir(exist_ok=False)
    fingerprints={str(source):hashlib.sha256(source.read_bytes()).hexdigest()}
    disabled=[]
    for include in vehicle.findall('include'):
        uri=include.find('uri');name=(uri.text or '').replace('model://','')
        if name not in ('D435i','fpv_cam'):continue
        path=(root/'simulation/astra_gazebo_models/D435i/model.sdf' if name=='D435i'
              else models/'fpv_cam/fpv_cam.sdf')
        fingerprints[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        camera=ET.parse(str(path))
        for parent in camera.iter():
            for node in list(parent):
                if node.tag=='sensor' and node.get('type') in ('camera','depth','multicamera'):
                    disabled.append(name+'/'+node.get('name',''));parent.remove(node)
                elif node.tag=='plugin' and node.get('filename')=='librealsense_gazebo_plugin.so':
                    parent.remove(node)
        # Relative mesh references must still resolve against the original model.
        for mesh in camera.findall('.//mesh/uri'):
            if not (mesh.text or '').startswith(('model://','file://','/')):
                mesh.text=(path.parent/mesh.text).resolve().as_uri()
        modeldir=directory/name;modeldir.mkdir()
        output=modeldir/'model.sdf';camera.write(str(output),encoding='utf-8',xml_declaration=True)
        (modeldir/'model.config').write_text('<model><name>'+name+'</name><version>1.0</version>'
            '<sdf version="1.5">model.sdf</sdf></model>')
        # file URI keeps the nested model names and existing joints unchanged.
        uri.text=modeldir.resolve().as_uri()
    output=directory/'iris_mid360.sdf';tree.write(str(output),encoding='utf-8',xml_declaration=True)
    generated={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
               for p in directory.rglob('*') if p.is_file()}
    report=dict(vehicle_sdf=str(output.resolve()),disabled_image_sensors=disabled,
        input_sha256={**fingerprints,**generated},
        scope='Private runtime model copies; camera rendering/stream plugins removed. '
              'Body/camera inertias, collisions, joints, MID360, IMU and physics timestep unchanged.')
    Path(str(prefix)+'_efficiency.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
