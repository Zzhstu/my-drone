"""Geometry/time invariants that matter for a LiDAR-camera sensor adapter."""
import unittest
from pathlib import Path
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from input_bridge import convert_cloud
from calibration import read_geometry


class Inputs(unittest.TestCase):
    def test_true_nanosecond_offsets_are_sorted_and_preserved(self):
        data=np.array([(1.,0.,0.,20.,90000000),(2.,0.,0.,30.,10000000)],
                      dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('intensity','<f4'),('offset_time','<u4')])
        msg=PointCloud2(height=1,width=2,point_step=20,row_step=40,data=data.tobytes())
        msg.fields=[PointField(name=n,offset=i*4,datatype=6 if n=='offset_time' else 7,count=1)
                    for i,n in enumerate(data.dtype.names)]
        msg.header.stamp.secs=42
        out=convert_cloud(msg,'livox_pointcloud2',stride=1)
        pts=np.frombuffer(out.data,dtype='<f4').reshape(-1,8)
        np.testing.assert_allclose(pts[:,7],[10,90])
        np.testing.assert_allclose(pts[:,0],[2,1])
        self.assertEqual(out.header.stamp.secs,42)

    def test_real_scan_cannot_silently_invent_time(self):
        msg=PointCloud2(height=1,width=1,point_step=12,row_step=12,
                        data=np.array([1.,0.,0.],dtype='<f4').tobytes())
        msg.fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')]
        with self.assertRaises(ValueError):convert_cloud(msg,'livox_pointcloud2')
        self.assertEqual(np.frombuffer(convert_cloud(msg,'sim',stride=1).data,dtype='<f4')[7],0)

    def test_sim_optical_z_is_body_forward_and_lidar_lever_arm_matches_sdf(self):
        root=Path(__file__).resolve().parents[2]
        g=read_geometry(root/'simulation/px4_sim_files/px4_iris_sdf/iris_mid360/iris_mid360.sdf',
                        root/'simulation/astra_gazebo_models/mid360/mid360.sdf',
                        root/'simulation/astra_gazebo_models/D435i/model.sdf')
        np.testing.assert_allclose(g['lidar_in_imu_translation'],[.011,.02329,.00588],atol=1e-8)
        transform=np.array(g['body_from_camera'])
        np.testing.assert_allclose(transform[:3,2],[1,0,0],atol=.004)
        np.testing.assert_allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-8)


if __name__=='__main__':unittest.main()
