import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from demo_efficiency import prepare_camera_free_vehicle
from evidence_recording import packed_xyz, recording_policy
from experiment_evidence import EVIDENCE_TOPICS


class EfficiencyTest(unittest.TestCase):
    def test_packed_xyz_preserves_coordinate_bytes_with_padding_and_endianness(self):
        from sensor_msgs.msg import PointCloud2,PointField
        for endian in ('<','>'):
            data=bytearray(2*72)
            expected=bytearray()
            for row in range(2):
                for col in range(2):
                    xyz=struct.pack(endian+'3I',0x80000000,0x7fc01234,0x3f800000+col+row)
                    base=row*72+col*32
                    data[base+4:base+16]=xyz;expected.extend(xyz)
            cloud=PointCloud2(height=2,width=2,is_bigendian=endian=='>',point_step=32,row_step=72,
                fields=[PointField(name=n,offset=4+4*i,datatype=7,count=1)
                        for i,n in enumerate(('x','y','z'))],data=bytes(data),is_dense=False)
            cloud.header.seq=123
            result=packed_xyz(cloud)
            self.assertEqual(result.data,bytes(expected));self.assertEqual(result.header.seq,123)
            self.assertEqual(result.row_step,24);self.assertEqual(result.is_bigendian,cloud.is_bigendian)
            self.assertFalse(result.is_dense)

    def test_light_recording_keeps_every_mapping_input_and_frozen_checkpoint(self):
        p=recording_policy(EVIDENCE_TOPICS,'compact',10.,.02,light=True)
        for topic in ('/uav1/fastlio/registered_scan','/uav1/fastlio/odom',
                      '/uav1/semantic_dynamic_objects','/uav1/loop_slam/input_ack'):
            self.assertIn(topic,p['topics']);self.assertNotIn(topic,p['max_hz'])
        self.assertTrue(p['pack_registered_scan_xyz'])
        self.assertIn('/uav1/loop_slam/snapshot',p['final_sample_topics'])
        self.assertFalse(recording_policy(EVIDENCE_TOPICS,'full',light=True)['pack_registered_scan_xyz'])

    def test_private_model_preserves_original_inertias_collisions_and_nonimage_sensors(self):
        root=Path(__file__).resolve().parents[5]
        # Resolve repository root without depending on this machine's checkout name.
        root=next(p for p in Path(__file__).resolve().parents if (p/'simulation/astra_gazebo_models').is_dir())
        px4=Path.home()/'PX4-Autopilot'
        with tempfile.TemporaryDirectory() as td:
            report=prepare_camera_free_vehicle(root,px4,Path(td)/'run')
            self.assertEqual(len(report['disabled_image_sensors']),5)
            directory=Path(report['vehicle_sdf']).parent
            for name,original in [('D435i',root/'simulation/astra_gazebo_models/D435i/model.sdf'),
                ('fpv_cam',px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/fpv_cam/fpv_cam.sdf')]:
                before=ET.parse(str(original));after=ET.parse(str(directory/name/'model.sdf'))
                for tag in ('inertial','collision','joint'):
                    self.assertEqual([ET.tostring(x) for x in before.findall('.//'+tag)],
                                     [ET.tostring(x) for x in after.findall('.//'+tag)])
                self.assertFalse(any(x.get('type') in ('camera','depth') for x in after.findall('.//sensor')))
                if name=='D435i':self.assertTrue(after.findall('.//sensor[@type="imu"]'))


if __name__=='__main__':unittest.main()
