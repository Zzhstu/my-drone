"""Private RGB-only camera variant with a coherent rendered FOV and ROS K."""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from integration.roomgs.camera_profiles import projection,profile_projection


def make_rgb_camera(source,models,*,width=None,height=None,hz=5,profile=None):
    source=Path(source);folder=Path(models)/'D435i';folder.mkdir(parents=True)
    tree=ET.parse(source);model=tree.getroot().find('model');removed=[]
    for link in model.findall('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('name')!='color':
                removed.append(sensor.get('name'));link.remove(sensor)
    for plugin in list(model.findall('plugin')):model.remove(plugin)
    color=model.find("link[@name='camera_color_frame']/sensor[@name='color']")
    width=int(width if width is not None else color.findtext('camera/image/width'))
    height=int(height if height is not None else color.findtext('camera/image/height'))
    optics=profile_projection(width,height,profile) if profile else projection(width,height,float(color.findtext('camera/horizontal_fov')))
    color.find('camera/horizontal_fov').text=repr(optics['horizontal_fov_rad'])
    color.find('camera/image/width').text=str(width);color.find('camera/image/height').text=str(height)
    color.find('update_rate').text=str(hz)
    always=color.find('always_on')
    if always is None:always=ET.SubElement(color,'always_on')
    always.text='true'
    plugin=ET.SubElement(color,'plugin',name='roomgs_rgb_only',filename='libgazebo_ros_camera.so')
    for name,value in dict(robotNamespace='/',cameraName='camera/color',imageTopicName='image_raw',
        cameraInfoTopicName='camera_info',frameName='camera_color_frame',updateRate='0',
        focalLength=repr(optics['K'][0][0]),Cx=str(width/2),Cy=str(height/2),CxPrime=str(width/2),
        distortionK1='0',distortionK2='0',distortionK3='0',distortionT1='0',distortionT2='0').items():
        ET.SubElement(plugin,name).text=value
    destination=folder/'model.sdf';tree.write(str(destination),encoding='utf-8',xml_declaration=True)
    shutil.copyfile(source.parent/'model.config',folder/'model.config')
    (folder/'meshes').symlink_to(source.parent/'meshes',target_is_directory=True)
    (folder/'policy.json').write_text(json.dumps(dict(original=str(source),
        original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),removed_sensors=removed,
        retained='RGB only; physical links/joints/masses/mounts unchanged',
        width=width,height=height,hz=hz,optics=optics,
        plugin='libgazebo_ros_camera.so',raw_camera_imu_used=False,main_lidar_imu_unchanged=True),indent=2)+'\n')
    return destination
