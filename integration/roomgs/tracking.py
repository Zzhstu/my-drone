"""Relative measured-pose feedback; no scene or simulator state."""
import math
import numpy as np


def relative_setpoint(fcu,native,target,native_heading):
    rotation=fcu[:3,:3]@native[:3,:3].T
    position=fcu[:3,3]+rotation@(np.asarray(target)-native[:3,3])
    fcu_yaw=math.atan2(fcu[1,0],fcu[0,0])
    native_yaw=math.atan2(native[1,0],native[0,0])
    error=(native_heading-native_yaw+math.pi)%(2*math.pi)-math.pi
    return position,fcu_yaw+error


class NativeIntegrator:
    """Small, gated correction for stationary observer/controller bias."""
    def __init__(self):self.correction=np.zeros(3)

    def update(self,error,dt,at_commanded_station):
        error=np.asarray(error)
        if at_commanded_station and np.linalg.norm(error)<.15:
            self.correction+=.25*error*max(0.,min(float(dt),.15))
            norm=np.linalg.norm(self.correction)
            if norm>.06:self.correction*=.06/norm
        return self.correction.copy()
