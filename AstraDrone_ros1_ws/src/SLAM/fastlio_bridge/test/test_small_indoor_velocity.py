import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from small_indoor_velocity import rotate_xy,yaw,fresh


class MapLocalVelocityTest(unittest.TestCase):
    def test_ninety_degree_map_offset_rotates_velocity(self):
        x,y=rotate_xy(.30,0.,math.pi/2,0.)
        self.assertAlmostEqual(x,0.,places=12)
        self.assertAlmostEqual(y,-.30,places=12)
        x,y=rotate_xy(0.,.30,math.pi/2,0.)
        self.assertAlmostEqual(x,.30,places=12)
        self.assertAlmostEqual(y,0.,places=12)

    def test_same_frame_and_arbitrary_heading_preserve_speed(self):
        self.assertEqual(rotate_xy(.2,-.1,.4,.4),(.2,-.1))
        for angle in [-3.,-1.,0.,1.,3.]:
            x,y=rotate_xy(.2,-.1,angle,.7)
            self.assertAlmostEqual(math.hypot(x,y),math.hypot(.2,.1))

    def test_sign_equivalent_quaternions_produce_same_yaw(self):
        a=NS(x=0.,y=0.,z=math.sin(.4),w=math.cos(.4))
        b=NS(x=-a.x,y=-a.y,z=-a.z,w=-a.w)
        self.assertAlmostEqual(yaw(a),yaw(b))
        self.assertAlmostEqual(yaw(a),.8)

    def test_invalid_orientation_and_vectors_are_rejected(self):
        with self.assertRaises(ValueError):yaw(NS(x=0.,y=0.,z=0.,w=0.))
        with self.assertRaises(ValueError):rotate_xy(float('nan'),0.,0.,0.)

    def test_acquisition_and_wall_age_both_required(self):
        msg=NS(header=NS(stamp=NS(to_sec=lambda:10.)))
        self.assertTrue(fresh(10.1,[(msg,20.)],20.1))
        self.assertFalse(fresh(10.5,[(msg,20.)],20.1))
        self.assertFalse(fresh(9.9,[(msg,20.)],20.1))
        self.assertFalse(fresh(10.1,[(msg,20.)],21.1))
        self.assertFalse(fresh(10.1,[None],20.1))
