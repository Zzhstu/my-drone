#!/usr/bin/env python3
"""Measured planar ground height for navigation only; never fills the map."""
import numpy as np


class GroundRelativeAltitude:
    def __init__(self):
        self.plane = None
        self.normal_height = None

    def reset_coordinates(self):
        # Distance to the ground survives a rigid graph/frame correction.
        self.plane = None

    def initialize(self, plane, position):
        if plane is None:
            return False
        plane = np.asarray(plane, dtype=float)
        position = np.asarray(position, dtype=float)
        if (plane.shape != (3,) or position.shape != (3,) or
                not np.isfinite(plane).all() or not np.isfinite(position).all() or
                np.linalg.norm(plane[:2]) > np.tan(np.deg2rad(15.))):
            return False
        scale = np.sqrt(1. + plane[:2] @ plane[:2])
        height = (position[2] - position[:2] @ plane[:2] - plane[2]) / scale
        if self.normal_height is None:
            if not .8 <= height <= 2.:
                return False
            self.normal_height = float(height)
        self.plane = plane.copy()
        return True

    def z(self, xy, fallback):
        if self.plane is None:
            return fallback
        xy = np.asarray(xy, dtype=float)
        return (xy[..., :2] @ self.plane[:2] + self.plane[2] +
                self.normal_height * np.sqrt(1. + self.plane[:2] @ self.plane[:2]))

    def summary(self):
        return dict(ready=self.plane is not None,
                    plane=None if self.plane is None else self.plane.tolist(),
                    normal_height_m=self.normal_height,
                    scope='Measured ground-relative navigation height; no map point insertion')
