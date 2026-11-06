"""Read sensor mounting geometry, never scene truth, for the R3LIVE simulation."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np


def pose(text):
    x, y, z, roll, pitch, yaw = map(float, (text or '0 0 0 0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                  [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


def read_geometry(vehicle, lidar, camera):
    v, l, c = [ET.parse(str(p)).getroot().find('model') for p in (vehicle, lidar, camera)]
    mounts = {e.findtext('uri'): pose(e.findtext('pose')) for e in v.findall('include')}
    ll = l.find("link[@name='lidar_link']")
    b_l = pose(v.findtext('pose')) @ mounts['model://mid360'] @ pose(l.findtext('pose')) @ pose(ll.findtext('pose'))
    b_i = b_l @ pose(ll.find("sensor[@type='imu']").findtext('pose'))
    b_laser = b_l @ pose(ll.find("sensor[@type='ray']").findtext('pose'))
    color = c.find("link[@name='camera_color_frame']")
    # Gazebo rendering cameras look along +X; Image pinhole coordinates use
    # +Z forward, +X right, +Y down. The plugin labels images camera_color_frame
    # but does not supply this optical rotation in a TF. Apply it explicitly.
    sensor_from_optical = np.eye(4)
    sensor_from_optical[:3, :3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    b_c = pose(v.findtext('pose')) @ mounts['model://D435i'] @ pose(c.findtext('pose')) @ pose(color.findtext('pose')) @ pose(color.find('sensor').findtext('pose')) @ sensor_from_optical
    i_c, i_l = np.linalg.inv(b_i) @ b_c, np.linalg.inv(b_i) @ b_laser
    if not np.allclose(i_l[:3, :3], np.eye(3), atol=1e-8):
        raise ValueError('R3LIVE upstream LIO requires aligned LiDAR/IMU axes; cannot silently ignore rotation')
    return dict(lidar_in_imu_translation=i_l[:3, 3].tolist(),
                camera_in_imu_rotation=i_c[:3, :3].ravel().tolist(),
                camera_in_imu_translation=i_c[:3, 3].tolist(),
                body_from_camera=b_c.tolist(), body_from_imu=b_i.tolist())
