#!/usr/bin/env python3
"""Read-only same-cutoff dual-map contract. Absence is never free-space evidence."""
from dataclasses import dataclass
import hashlib
import math
import itertools
import numpy as np


@dataclass(frozen=True)
class MapState:
    stamp_ns: int
    processed_frames: int
    input_frames: int
    frame_id: str
    graph_revision: int = 0
    requested_revision: int = 0
    resolution: float = .15
    healthy: bool = True
    rebuilding: bool = False
    capacity_limited: bool = False
    frozen: bool = False

    @property
    def key(self):
        return self.graph_revision, self.processed_frames, self.stamp_ns


def validate_pair(current, background, current_from_background):
    if not current.healthy or not background.healthy:
        raise ValueError('unhealthy map source')
    if current.capacity_limited or background.capacity_limited:
        raise ValueError('map capacity limited')
    if current.rebuilding or background.rebuilding:
        raise ValueError('map rebuild in progress')
    if current.frame_id != 'loop_map' or background.frame_id != 'map':
        raise ValueError('unexpected source coordinate frames')
    if current.processed_frames < 1 or current.stamp_ns <= 0:
        raise ValueError('empty map boundary')
    if current.key != background.key:
        raise ValueError('different graph/frame count/cutoff')
    if current.graph_revision != 0 or current.requested_revision != 0 or background.requested_revision != 0:
        raise ValueError('DUFOMap graph rebuild is not integrated; nonzero graph revision refused')
    if not math.isfinite(current.resolution) or not math.isfinite(background.resolution) or current.resolution <= 0:
        raise ValueError('invalid map resolution')
    if abs(current.resolution-background.resolution) > 1e-9:
        raise ValueError('different map resolutions')
    transform = np.asarray(current_from_background)
    if transform.shape != (4, 4) or not np.isfinite(transform).all() or not np.allclose(transform, np.eye(4), atol=1e-7, rtol=0):
        raise ValueError('local/global transform is not identity at graph revision zero')
    for state in (current, background):
        if state.input_frames < state.processed_frames:
            raise ValueError('processed frames exceed received frames')
        if state.frozen and state.input_frames != state.processed_frames:
            raise ValueError('frozen map has pending inputs')
    return dict(processed_frames=current.processed_frames, stamp_ns=current.stamp_ns,
                graph_revision=0, resolution_m=current.resolution,
                frozen=bool(current.frozen and background.frozen),
                common_frame='loop_map', identity_transform_verified=True,
                observations_are_independent=False, planner_input=False)


def point_set_sha256(points):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('nonfinite point set')
    indexes = np.lexsort((points[:, 2], points[:, 1], points[:, 0]))
    return hashlib.sha256(np.ascontiguousarray(points[indexes]).tobytes()).hexdigest()


def geometry_disagreement(current, background, tolerance=.15, max_points=350000):
    """Geometric differences only; do not label these points ghosts or free space."""
    current, background = [np.asarray(x, dtype=np.float32).reshape(-1, 3) for x in (current, background)]
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('invalid geometric comparison tolerance')
    if not all(np.isfinite(x).all() for x in (current, background)):
        raise ValueError('nonfinite geometry comparison input')
    if len(current)+len(background) > max_points:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.float32), dict(computed=False, reason='comparison_capacity')
    def unmatched(a, b):
        if not len(a) or not len(b):
            return a.copy()
        # Exact radius membership using vectorized bounded-cell joins. ROS's
        # Python does not require SciPy. Retain all representatives per cell.
        ak = np.floor(a.astype(float)/tolerance).astype(np.int64)
        bk = np.floor(b.astype(float)/tolerance).astype(np.int64)
        low = np.minimum(ak.min(axis=0),bk.min(axis=0))-2
        high = np.maximum(ak.max(axis=0),bk.max(axis=0))+2
        shape = high-low+1
        if math.prod(int(x) for x in shape) > np.iinfo(np.int64).max:
            raise ValueError('geometry extent exceeds integer spatial index')
        stride = np.array([int(shape[1])*int(shape[2]),int(shape[2]),1],np.int64)
        ai=(ak-low)@stride; bi=(bk-low)@stride
        order=np.argsort(bi); ids=bi[order]; targets=b[order].astype(float)
        matched=np.zeros(len(a),bool)
        for delta in itertools.product((-1,0,1),repeat=3):
            index=np.flatnonzero(~matched)
            if not len(index): break
            keys=ai[index]+np.asarray(delta,dtype=np.int64)@stride
            left=np.searchsorted(ids,keys,'left'); right=np.searchsorted(ids,keys,'right')
            counts=right-left
            for offset in range(int(counts.max()) if len(counts) else 0):
                active=(counts>offset)&~matched[index]
                chosen=index[active]
                distance2=np.sum((a[chosen].astype(float)-targets[left[active]+offset])**2,axis=1)
                matched[chosen[distance2<=tolerance*tolerance]]=True
        return a[~matched]
    ca, ba = unmatched(current, background), unmatched(background, current)
    return ca, ba, dict(computed=True, tolerance_m=tolerance, current_only_points=len(ca),
                       background_only_points=len(ba),
                       scope='Nearest-point disagreement only; sampling, occupancy changes, ghosts and pose error may all contribute. No occupancy or free-space decision.')
