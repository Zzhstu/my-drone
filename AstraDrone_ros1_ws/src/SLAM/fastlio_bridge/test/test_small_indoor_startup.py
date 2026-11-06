import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from small_indoor_startup import HeadingWindow


def q(degrees):
    return (0.,0.,math.sin(math.radians(degrees)/2),math.cos(math.radians(degrees)/2))


class HeadingReadinessTest(unittest.TestCase):
    def test_early_stable_heading_does_not_bypass_fcu_readiness(self):
        w=HeadingWindow()
        for stamp in range(8):
            self.assertFalse(w.update(stamp,q(90),ready=False))

    def test_yaw_reset_requires_a_new_full_dwell(self):
        w=HeadingWindow()
        for t in [1.,2.,3.]:
            self.assertFalse(w.update(t,q(90)))
        for t in [3.1,4.,5.,6.]:
            self.assertFalse(w.update(t,q(0)))
        self.assertTrue(w.update(6.2,q(.1)))

    def test_duplicate_or_reverse_time_cannot_manufacture_dwell(self):
        w=HeadingWindow()
        for _ in range(20):
            self.assertFalse(w.update(1.,q(0)))
        self.assertFalse(w.update(.5,q(0)))
        self.assertFalse(w.update(2.,q(0)))

    def test_quaternion_sign_is_same_attitude_and_loss_resets(self):
        w=HeadingWindow()
        self.assertFalse(w.update(1.,q(0)))
        self.assertTrue(w.update(4.1,tuple(-v for v in q(0))))
        self.assertFalse(w.update(4.2,q(0),ready=False))
        self.assertFalse(w.update(5.,q(0)))
        self.assertFalse(w.update(6.,(float('nan'),0.,0.,1.)))
