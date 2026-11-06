#!/usr/bin/env python3
"""Nearby viewpoint coverage, explicitly NOT observed surface completeness."""
from collections import deque
import math
import numpy as np


def geometry_bound_sweep_gain(sweep, gain, channels, mask, mass, scale=16.):
    """Boost existing real sensing requests, without inventing new viewpoints."""
    missing=np.zeros(mask.shape,bool)
    for name,layer in channels.items():
        if name in ('missing_floor','missing_boundary'):missing |= layer
    bonus=sweep & missing & mask
    result=mass.copy();result[bonus]+=gain[bonus]*scale
    return result,int(bonus.sum())


def sweep_candidates(grid, distances, visited, focus=None, radius=2., minimum_area=.5):
    eligible=grid.safe & np.isfinite(distances) & ~visited
    if focus is not None:
        eligible &= focus
    # Tiny quantization pockets must not consume the whole flight budget.
    remaining=eligible.copy();mask=np.zeros(grid.shape,bool);gain=np.zeros(grid.shape,float)
    areas=[]
    minimum=max(1,int(math.ceil(minimum_area/grid.resolution**2)))
    for y,x in np.argwhere(eligible):
        if not remaining[y,x]:continue
        queue=deque([(y,x)]);remaining[y,x]=False;component=[]
        while queue:
            row,col=queue.popleft();component.append((row,col))
            for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
                if 0<=r<mask.shape[0] and 0<=c<mask.shape[1] and remaining[r,c]:
                    remaining[r,c]=False;queue.append((r,c))
        if len(component)>=minimum:
            indices=tuple(np.asarray(component).T);mask[indices]=True
            area=len(component)*grid.resolution**2;gain[indices]=area;areas.append(area)
    # Normalize GLOBAL missing-view component area before joint score clipping.
    # A local 2m disk gave every pocket a saturated reward=1, so many cheap
    # tiny near-home pockets could starve an entire distant aisle forever.
    if areas:gain/=sum(areas)
    return mask,gain,dict(remaining_reachable_viewpoint_area_m2=float(mask.sum()*grid.resolution**2),
        reached_viewpoint_neighborhood_area_m2=float((visited&grid.safe).sum()*grid.resolution**2),
        remaining_components=len(areas),largest_component_areas_m2=sorted(areas,reverse=True)[:8],
        scope='Actual close-range viewpoint visitation only; never marks surfaces, occupancy or unknown cells observed')


class ScanDwell:
    def __init__(self, now, seconds, minimum_scans=8):
        self.started=float(now);self.seconds=float(seconds);self.minimum_scans=int(minimum_scans)
        self.continuous=None;self.scans=0;self.last_stamp=-float('inf')

    def update(self, now, scan_stamp, close):
        if not close:
            self.continuous=None;self.scans=0;self.last_stamp=scan_stamp
        else:
            if self.continuous is None:self.continuous=float(now)
            if scan_stamp>=self.continuous and scan_stamp>self.last_stamp:
                self.scans+=1;self.last_stamp=scan_stamp
        complete=(self.continuous is not None and now-self.continuous>=self.seconds and
                  self.scans>=self.minimum_scans)
        expired=now-self.started>=self.seconds+3.
        return bool(complete),bool(expired)
