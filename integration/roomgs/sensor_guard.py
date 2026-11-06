"""Body-frame obstacle tube; inputs are measured points and relative motion."""
import numpy as np


def count_in_tube(points,delta,radius=.38):
    u=np.clip(points@delta/max(float(delta@delta),1e-8),0,1)
    return int((np.linalg.norm(points-u[:,None]*delta,axis=1)<radius).sum())
