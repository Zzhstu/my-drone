import sys
from pathlib import Path
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from sim_sensor_geometry import sensor_geometry,translation,load_sim_geometry
import xml.etree.ElementTree as ET


class SimSensorGeometryTest(unittest.TestCase):
    def test_actual_vehicle_calibration_matches_sim_profile(self):
        root=Path(__file__).resolve().parents[5]
        geometry=load_sim_geometry(root,Path.home()/'PX4-Autopilot')
        np.testing.assert_allclose(geometry['imu_origin_in_body'],[-.011,-.02329,.12412],atol=1e-12)
        np.testing.assert_allclose(geometry['lidar_in_imu_translation'],[.011,.02329,.00588],atol=1e-12)

    def test_sensor_pose_difference_not_reused_as_extrinsic(self):
        vehicle='<sdf><model><include><uri>model://mid360</uri><pose>0 0 .08 0 0 0</pose></include></model></sdf>'
        sensor='''<sdf><model><link name="lidar_link"><sensor type="imu"><pose>-.011 -.02329 .04412 0 0 0</pose>
          </sensor><sensor type="ray"><pose>0 0 .05 0 0 0</pose></sensor></link></model></sdf>'''
        with tempfile.TemporaryDirectory() as folder:
            v=Path(folder)/'vehicle.sdf';s=Path(folder)/'sensor.sdf';v.write_text(vehicle);s.write_text(sensor)
            geometry=sensor_geometry(v,s)
        np.testing.assert_allclose(np.array(geometry['imu_origin_in_body'])+
                                   geometry['lidar_in_imu_translation'],geometry['laser_origin_in_body'])

    def test_rotated_mount_requires_full_rotation_support(self):
        with self.assertRaises(ValueError):translation(ET.fromstring('<x><pose>0 0 0 0 .1 0</pose></x>'))


if __name__=='__main__':unittest.main()
