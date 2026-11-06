"""Known vehicle calibration from selected SDFs, never environment geometry."""
import hashlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def translation(element):
    pose=np.asarray([float(x) for x in element.findtext('pose','0 0 0 0 0 0').split()])
    if pose.shape!=(6,) or not np.isfinite(pose).all() or np.any(np.abs(pose[3:])>1e-9):
        raise ValueError('Simulation profile requires aligned body/IMU/laser axes')
    return pose[:3]


def model_file(name, search_paths):
    for directory in search_paths:
        directory=Path(directory)/name
        config=directory/'model.config'
        if config.is_file():
            sdf=ET.parse(str(config)).getroot().findtext('sdf')
            path=directory/sdf if sdf else directory/(name+'.sdf')
            if path.is_file():return path.resolve()
    raise ValueError('Gazebo model not found: '+name)


def sensor_geometry(vehicle_path, sensor_path):
    vehicle=ET.parse(str(vehicle_path)).getroot().find('model')
    mount=next(x for x in vehicle.findall('include') if x.findtext('uri')=='model://mid360')
    sensor=ET.parse(str(sensor_path)).getroot().find('model')
    link=sensor.find("link[@name='lidar_link']")
    common=translation(vehicle)+translation(mount)+translation(sensor)+translation(link)
    imu=common+translation(link.find("sensor[@type='imu']"))
    laser=common+translation(link.find("sensor[@type='ray']"))
    return dict(imu_origin_in_body=imu.tolist(),laser_origin_in_body=laser.tolist(),
                lidar_in_imu_translation=(laser-imu).tolist(),
                lidar_in_imu_rotation=np.eye(3).ravel().tolist(),
                source_sha256={str(Path(p).resolve()):hashlib.sha256(Path(p).read_bytes()).hexdigest()
                               for p in (vehicle_path,sensor_path)},
                note='Known vehicle SDF calibration; no scene/person truth, no fitted map offset.')


def load_sim_geometry(root, px4):
    root,px4=Path(root),Path(px4)
    model_root=px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
    paths=[p for p in os.environ.get('GAZEBO_MODEL_PATH','').split(':') if p]
    paths += [model_root,root/'simulation/astra_gazebo_models']
    geometry=sensor_geometry(model_root/'iris_mid360/iris_mid360.sdf',model_file('mid360',paths))
    config=root/'AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360_sim.yaml'
    profile=yaml.safe_load(config.read_text())
    parameters=profile['mapping']
    for key,expected in [('extrinsic_T',geometry['lidar_in_imu_translation']),
                         ('extrinsic_R',geometry['lidar_in_imu_rotation'])]:
        if not np.allclose(parameters[key],expected,atol=1e-9,rtol=0):
            raise ValueError('Simulated sensor SDF no longer matches '+str(config)+': '+key)
    for node,key,expected in [('fastlio_bridge','imu_origin_in_body',geometry['imu_origin_in_body']),
                              ('fastlio_cloud_bridge','imu_origin_in_body',geometry['imu_origin_in_body']),
                              ('static_map_builder','lidar_origin_in_odom',geometry['laser_origin_in_body'])]:
        if not np.allclose(profile[node][key],expected,atol=1e-9,rtol=0):
            raise ValueError('Simulated sensor SDF no longer matches '+node+'/'+key)
    return geometry
