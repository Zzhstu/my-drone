import unittest
import numpy as np
from poses import PoseBuffer,VoxelMap,quaternion,rotation


class TestMeasurementPoses(unittest.TestCase):
    def test_slerp_rotates_camera_lever_arm_after_pose_interpolation(self):
        ext=np.eye(4);ext[0,3]=1
        a=np.eye(4);b=np.eye(4);b[0,3]=2;b[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        poses=PoseBuffer(ext);poses.add(1000000000,a);poses.add(1100000000,b)
        camera,meta=poses.camera(1050000000)
        np.testing.assert_allclose(camera[:3,3],[1+2**-.5,2**-.5,0],atol=1e-12)
        self.assertEqual(meta['alpha'],.5)
        self.assertIsNone(poses.camera(999999999));self.assertIsNone(poses.camera(1100000001))

    def test_large_gap_and_backward_clock_are_rejected(self):
        p=PoseBuffer(np.eye(4));p.add(0,np.eye(4));p.add(200000000,np.eye(4))
        self.assertIsNone(p.camera(100000000))
        with self.assertRaises(ValueError):p.add(100000000,np.eye(4))

    def test_quaternion_half_turn_and_arbitrary_axes_round_trip(self):
        for q in [[1,0,0,0],[0,1,0,0],[0,0,1,0],[.3,-.5,.7,.1],[0,0,0,1]]:
            r=rotation(q);np.testing.assert_allclose(rotation(quaternion(r)),r,atol=1e-12)

    def test_voxels_keep_only_actual_measured_points_and_bound_memory(self):
        v=VoxelMap(.1,2);a=np.array([[.01,.01,.01],[.02,.02,.02],[.11,0,0]])
        v.add(a);self.assertEqual(len(v.array()),2)
        self.assertTrue(all(any(np.allclose(p,q) for q in a) for p in v.array()))
        with self.assertRaises(RuntimeError):v.add(np.array([[.3,0,0]]))


if __name__=='__main__':unittest.main()
