import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ground_relative_altitude import GroundRelativeAltitude


class GroundRelativeAltitudeTest(unittest.TestCase):
    def test_perpendicular_clearance_is_constant_in_tilted_map(self):
        n=GroundRelativeAltitude();plane=np.array([-.025,.035,-.08])
        self.assertTrue(n.initialize(plane,[0,0,1.2]))
        xy=np.array([[0,0],[20,-16],[-10,14],[15,10]])
        z=n.z(xy,None);scale=np.sqrt(1+plane[:2]@plane[:2])
        np.testing.assert_allclose((z-xy@plane[:2]-plane[2])/scale,n.normal_height)
        self.assertAlmostEqual(n.z([0,0],None),1.2)
        self.assertGreater(np.ptp(z),1.)

    def test_coordinate_reset_requires_new_measurement_preserves_height(self):
        n=GroundRelativeAltitude();n.initialize([.02,0,0],[0,0,1.2]);h=n.normal_height
        n.reset_coordinates();self.assertIsNone(n.plane);self.assertEqual(n.z([1,2],7),7)
        self.assertTrue(n.initialize([0,.03,-.1],[5,5,1.1]))
        self.assertEqual(n.normal_height,h)

    def test_missing_or_invalid_ground_never_authorizes_height_field(self):
        n=GroundRelativeAltitude()
        for plane,pos in [(None,[0,0,1.2]),([1,0,0],[0,0,1.2]),
                          ([0,0,np.nan],[0,0,1.2]),([0,0,1],[0,0,1.2])]:
            self.assertFalse(n.initialize(plane,pos));self.assertIsNone(n.plane)
        self.assertEqual(n.z([1,2],1.2),1.2)

    def test_explorer_obstacles_use_the_same_height_as_goals(self):
        # A ceiling at the far corner must remain outside the body band at
        # constant real height; the old fixed-map plane incorrectly includes it.
        from warehouse_explorer import Explorer
        e=Explorer.__new__(Explorer);e.altitude=1.2
        e.p=lambda key,default=None: {'ground_relative_navigation':True}.get(key,default)
        e.ground_navigation=GroundRelativeAltitude();e.ground_navigation.initialize([-.025,.035,0],[0,0,1.2])
        xy=np.array([[20.,-10.]]);z=float(e.navigation_z(xy)[0])
        points=np.array([[*xy[0],z],[*xy[0],z+.5],[*xy[0],z+1.0]])
        np.testing.assert_allclose(e.slice(points),points[:2])
        with patch('warehouse_explorer.rospy.Time.now',return_value=__import__('rospy').Time(1)):
            self.assertAlmostEqual(e.pose(xy[0]).pose.position.z,z)


if __name__=='__main__':unittest.main()
