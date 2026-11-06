import math
import unittest
import numpy as np
from tracking import relative_setpoint,NativeIntegrator


class TrackingTests(unittest.TestCase):
    def test_stationary_bias_rejected_without_unbounded_windup(self):
        integrator=NativeIntegrator();error=.05;dt=.05
        for _ in range(1000):
            correction=integrator.update([error,0.,0.],dt,True)
            error+=(.05-error-correction[0])*dt
        self.assertLess(abs(error),.001)
        self.assertLessEqual(np.linalg.norm(integrator.correction),.060000001)
        previous=integrator.correction.copy()
        for _ in range(100):integrator.update([2.,0.,0.],dt,True)
        np.testing.assert_array_equal(previous,integrator.correction)

    def test_estimator_origin_and_height_bias_do_not_shift_requested_native_view(self):
        native=np.eye(4);native[:3,3]=[1.,2.,.7]
        fcu=native.copy();fcu[:3,3]+=[.2,-.1,.08]
        command,_=relative_setpoint(fcu,native,[1.,2.,.7],0.)
        np.testing.assert_allclose(command,fcu[:3,3])
        command,_=relative_setpoint(fcu,native,[1.,2.,1.],0.)
        np.testing.assert_allclose(command-fcu[:3,3],[0.,0.,.3])

    def test_rotated_estimator_axes_preserve_body_relative_command(self):
        native=np.eye(4);fcu=np.eye(4)
        fcu[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        command,heading=relative_setpoint(fcu,native,[.5,0.,0.],-.1)
        np.testing.assert_allclose(command,[0.,.5,0.],atol=1e-12)
        self.assertAlmostEqual(heading,math.pi/2-.1)


if __name__=='__main__':unittest.main()
