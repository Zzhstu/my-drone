#!/usr/bin/env python3
"""Sensor-only progress/observed-surface evidence, not ground-truth coverage.

Missing surfaces which have NEVER returned a ray cannot enter this denominator.
Fixed-altitude navigation cannot certify roof/interior/occluded mesh completeness.
"""
from collections import deque
import math
import time
import numpy as np


class ObservedMapAcceptance:
    """Optional demo exit using fresh sensor evidence; never full coverage."""
    def __init__(self, target=.90, window=60., hold=10.,minimum_seconds=180.):
        if not (.5<=target<=1. and math.isfinite(window) and window>=1. and
                math.isfinite(hold) and hold>=1. and math.isfinite(minimum_seconds) and minimum_seconds>=0.):
            raise ValueError('Invalid observed-map acceptance thresholds')
        self.target,self.window,self.hold = target,window,hold
        self.minimum_seconds=minimum_seconds
        self.reset()

    def reset(self):
        self.samples = deque()
        self.last_stamp = self.since = None
        self.report = dict(accepted=False,quality_target=self.target,
                           window_seconds=self.window,hold_seconds=self.hold)

    def update(self, now, stamp, known_count, resolution, goals, mission_seconds,
               surface, capacity_limited=False):
        # A timer receiving duplicate scans cannot certify a plateau or dwell.
        if self.last_stamp is not None and stamp<=self.last_stamp:
            return False
        if self.last_stamp is not None and stamp-self.last_stamp>1.5:
            self.reset()
        self.last_stamp = stamp
        self.samples.append((now,known_count))
        while len(self.samples)>1 and self.samples[1][0]<=now-self.window:
            self.samples.popleft()
        area = known_count*resolution**2
        gain = max(0,known_count-self.samples[0][1])*resolution**2
        threshold = max(2.,.01*area)
        ready = now-self.samples[0][0]>=self.window
        quality = surface.get('observed_surface_quality_ratio')
        eligible = bool(ready and gain<threshold and goals>=6 and mission_seconds>=self.minimum_seconds
                        and surface.get('observed_surface_voxels',0)>=1000
                        and quality is not None and math.isfinite(quality) and quality>=self.target
                        and not capacity_limited and not surface.get('surface_evidence_capacity_limited',False))
        self.since = (now if self.since is None else self.since) if eligible else None
        held = now-self.since if self.since is not None else 0.
        self.report.update(accepted=bool(eligible and held>=self.hold),
            window_ready=ready,new_known_area_m2=gain,area_threshold_m2=threshold,
            observed_surface_quality_ratio=quality,eligible=eligible,held_seconds=held)
        return self.report['accepted']


class ProgressWindow:
    def __init__(self, seconds=90., free_threshold=1., surface_threshold=25):
        self.seconds = seconds
        self.free_threshold = free_threshold
        self.surface_threshold = surface_threshold
        self.samples = deque()

    def update(self, now, known_count, surface_count, goals):
        # Inputs are monotonic evidence counters, not noisy raw map point count.
        self.samples.append((now,known_count,surface_count,goals))
        while len(self.samples)>1 and self.samples[1][0] <= now-self.seconds:
            self.samples.popleft()

    def reset(self):
        self.samples.clear()

    def summary(self, resolution):
        if not self.samples:
            return dict(window_ready=False,new_free_area_m2=0.,new_surface_voxels=0,
                        observation_goals=0,low_gain=False)
        first,last = self.samples[0],self.samples[-1]
        area = max(0,last[1]-first[1])*resolution**2
        surfaces = max(0,last[2]-first[2])
        ready = last[0]-first[0] >= self.seconds
        return dict(window_ready=ready,new_free_area_m2=area,new_surface_voxels=surfaces,
                    observation_goals=max(0,last[3]-first[3]),
                    low_gain=bool(ready and area<self.free_threshold and surfaces<self.surface_threshold))



def suppress_disk(mask, cell, radius):
    """Visit-memory suppression in a small patch, not a whole-grid allocation."""
    n = int(math.ceil(radius))
    y,x = cell
    y0,y1 = max(0,y-n),min(mask.shape[0],y+n+1)
    x0,x1 = max(0,x-n),min(mask.shape[1],x+n+1)
    yy,xx = np.ogrid[y0:y1,x0:x1]
    patch = mask[y0:y1,x0:x1]
    patch[(yy-y)**2+(xx-x)**2 < radius**2] = False


def representative_cells(mask, gain, distances, resolution, spacing=1.):
    """One best representative per metre bin; avoid scoring every cyan pixel."""
    cells = np.argwhere(mask)
    if not len(cells):
        return cells
    n = max(1,int(round(spacing/resolution)))
    bins = cells//n
    rank = gain[mask]/(2.+distances[mask])
    order = np.argsort(-rank,kind='stable')
    _,indexes = np.unique(bins[order],axis=0,return_index=True)
    return cells[order[np.sort(indexes)]]


def visible_unknown_gain(grid, cells, sensor_range=6., rays=96, return_claims=False):
    """Approximate horizontal visibility, stopped by RAW occupied cells.

    Gain may enter unknown but never passes a measured wall. Densely sample
    each ray to prevent skipping a one-cell wall. This is a scoring heuristic,
    NOT a traversability test or 3-D visibility certificate.
    """
    if not len(cells):
        return (np.empty(0),[]) if return_claims else np.empty(0)
    angles = np.arange(rays)*2*math.pi/rays
    steps = np.arange(grid.resolution*.5,sensor_range+1e-6,grid.resolution*.5)
    offsets = np.stack((np.sin(angles)[:,None]*steps,
                        np.cos(angles)[:,None]*steps),axis=-1)/grid.resolution
    offsets = np.floor(offsets+.5).astype(np.int32)
    gain = [];claims=[]
    for cell in cells:
        points = offsets+cell
        y,x = points[...,0],points[...,1]
        valid = (y>=0)&(y<grid.shape[0])&(x>=0)&(x<grid.shape[1])
        yc,xc = np.clip(y,0,grid.shape[0]-1),np.clip(x,0,grid.shape[1]-1)
        blocked = ~valid | grid.occupied[yc,xc]
        visible = (np.cumsum(blocked,axis=1)==0)&~grid.known[yc,xc]
        indexes = yc[visible]*grid.shape[1]+xc[visible]
        unique=np.unique(indexes)
        gain.append(len(unique)*grid.resolution**2)
        if return_claims:
            from exploration_region_memory import BenefitClaim
            iy,ix=unique//grid.shape[1],unique%grid.shape[1]
            xy=np.c_[grid.x0+(ix+.5)*grid.resolution,grid.y0+(iy+.5)*grid.resolution]
            bins=np.floor(xy/2.4).astype(int);rows=[]
            if len(bins):
                regions,counts=np.unique(bins,axis=0,return_counts=True)
                for region,count in sorted(zip(regions,counts),key=lambda pair:-pair[1])[:24]:
                    q=xy[np.all(bins==region,axis=1)]
                    support=frozenset(map(tuple,np.floor(q/grid.resolution).astype(int)))
                    rows.append(BenefitClaim(('space',int(region[0]),int(region[1]),-1,-1),
                        tuple(np.r_[q.mean(axis=0),0.]),float(count)*grid.resolution**2,support=support))
            claims.append(tuple(rows))
    return (np.asarray(gain),claims) if return_claims else np.asarray(gain)


def raw_line_visible(grid, a, b, endpoint_margin=0.):
    """Only interior cells block a ray; its endpoint may be a surface."""
    count = max(2,int(math.ceil(math.dist(a,b)/grid.resolution*4)))
    t = np.linspace(0.,1.,count,endpoint=False)
    if endpoint_margin:
        t = t[(1.-t)*math.dist(a,b)>endpoint_margin]
    x = np.floor((a[0]+t*(b[0]-a[0])-grid.x0)/grid.resolution).astype(np.int32)
    y = np.floor((a[1]+t*(b[1]-a[1])-grid.y0)/grid.resolution).astype(np.int32)
    end = grid.cell(b)
    interior = (y!=end[0]) | (x!=end[1])
    y,x = y[interior],x[interior]
    if ((y<0)|(y>=grid.shape[0])|(x<0)|(x>=grid.shape[1])).any():
        return False
    return not grid.occupied[y,x].any()


class SurfaceEvidence:
    def __init__(self, voxel=.4, minimum_hits=3, minimum_views=2,
                 baseline=.75, maximum=150000):
        self.voxel,self.minimum_hits,self.minimum_views = voxel,minimum_hits,minimum_views
        self.baseline,self.maximum = baseline,maximum
        self.records = {}
        self.ever = set()
        self.capacity_limited = False
        self.last_stamp = None
        self.scan_count=0

    def keys(self, points):
        if not len(points):
            return np.empty((0,3),dtype=np.int64)
        return np.unique(np.floor(points/self.voxel).astype(np.int64),axis=0)

    def observe(self, points, observer, stamp, max_range=6.):
        # Re-publishing the same scan does not add evidence. Out-of-order scans
        # cannot turn a parked scanner into independent observations either.
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.scan_count+=1
        points = np.asarray(points)
        observer = np.asarray(observer)
        points = points[np.linalg.norm(points-observer,axis=1)<=max_range]
        for k in self.keys(points):
            key = tuple(k)
            center = (k+.5)*self.voxel
            angle = math.atan2(observer[1]-center[1],observer[0]-center[0])
            bit = 1 << (int((angle+math.pi)/(2*math.pi)*8)%8)
            record = self.records.get(key)
            if record is None:
                if len(self.records)>=self.maximum or len(self.ever)>=self.maximum:
                    self.capacity_limited = True
                    continue
                record = [0,bit,observer.copy()]
                self.records[key] = record
                self.ever.add(key)
            record[0] += 1
            if np.linalg.norm(observer-record[2])>=self.baseline:
                record[1] |= bit

    def complete(self, key):
        record = self.records.get(tuple(key))
        return bool(record and record[0]>=self.minimum_hits and
                    bin(record[1]).count('1')>=self.minimum_views)

    def deficits(self, current_map):
        # Restrict to the CURRENT maintained map. Cleared people must not
        # survive as permanent inspection targets in this evidence sidecar.
        keys = self.keys(current_map)
        done = np.asarray([self.complete(k) for k in keys],dtype=bool)
        self.current_keys=keys;self.current_qualified_keys=keys[done]
        self.current_points=np.asarray(current_map)
        return (keys[~done]+.5)*self.voxel,dict(
            observed_surface_voxels=len(keys),qualified_surface_voxels=int(done.sum()),
            observed_surface_quality_ratio=float(done.mean()) if len(keys) else None,
            surface_evidence_capacity_limited=self.capacity_limited)

    def inspection_candidates(self, grid, distances, deficits, minimum_distance=1.2,
                              limit=180, range_m=3., arrival_tolerance=.3,
                              altitude=None,min_elevation=-22.2,max_elevation=37.2,
                              robust_quota=False,view_altitude=None):
        """Observe deficient measured surfaces from another safe aisle view.

        Bounded coarse representatives; candidates do NOT contain truth shelf
        locations. Surface ray is checked against measured occupancy only.
        """
        mask = np.zeros(grid.shape,dtype=bool)
        gain = np.zeros(grid.shape,dtype=float)
        self.last_measured_inspection_claims={}
        self.last_inspection_report=dict(groups_available=0,groups_tried=0,
            contributing_groups=0,trial_limit=0,candidate_cells=0,bounded_search_exhausted=False)
        if not math.isfinite(arrival_tolerance) or arrival_tolerance<0:
            raise ValueError('Arrival tolerance must be finite and nonnegative')
        if not len(deficits):
            return mask,gain
        xy_cells = np.asarray([grid.cell(p[:2]) for p in deficits])
        _,index,counts = np.unique(xy_cells//max(1,int(.8/grid.resolution)),axis=0,
                                  return_index=True,return_counts=True)
        # Keep the deficit mass when coarse representatives are selected.
        # A bin with 50 missing voxels is more useful than one isolated voxel.
        # This remains a heuristic score; actual scans alone certify quality.
        # A dense but unreachable/occluded group must not consume the quota
        # before its viewpoints are checked. Bound attempts separately from
        # the number of groups which actually contribute a usable view.
        trial_limit=min(len(counts),max(32,8*limit) if robust_quota else limit)
        selected = np.argsort(-counts,kind='stable')[:trial_limit]
        contributing=0;tried=0
        for representative in selected:
            tried+=1;contributed=False
            surface = deficits[index[representative]]
            record = self.records.get(tuple(np.floor(surface/self.voxel).astype(np.int64)))
            # Sector centres, not boundaries: grid quantization and the .30m
            # arrival tolerance must not turn a proposed "new" direction back
            # into the old sector. Score the ACTUAL rasterized waypoint below.
            for angle in (np.arange(8)+.5)*2*math.pi/8:
                for radius in (1.5,2.5,range_m):
                    xy = surface[:2]+radius*np.asarray([math.cos(angle),math.sin(angle)])
                    cell = grid.cell(xy)
                    if not grid.inside(cell) or not grid.safe[cell] or not np.isfinite(distances[cell]):
                        continue
                    # A coarse voxel centre can lie just behind the actual
                    # first return. Only scoring uses a quantization margin;
                    # path/footprint safety ALWAYS keeps full obstacle inflation.
                    if distances[cell]<minimum_distance or not raw_line_visible(
                            grid,grid.xy(cell),surface[:2],self.voxel*math.sqrt(2)/2):
                        continue
                    view_xy=grid.xy(cell)
                    if altitude is not None:
                        view_z=altitude if view_altitude is None else float(view_altitude(view_xy))
                        horizontal=math.dist(view_xy,surface[:2])
                        low=math.degrees(math.atan2(surface[2]-self.voxel/2-view_z,horizontal))
                        high=math.degrees(math.atan2(surface[2]+self.voxel/2-view_z,horizontal))
                        if low<min_elevation or high>max_elevation: continue
                    if record and math.dist(view_xy,record[2][:2])<self.baseline+arrival_tolerance:
                        continue  # even an arrival at the tolerance edge needs an independent baseline
                    direction = math.atan2(view_xy[1]-surface[1],view_xy[0]-surface[0])
                    angle_margin = math.asin(min(1.,arrival_tolerance/max(
                        math.dist(view_xy,surface[:2]),1e-6)))
                    bits = 0
                    for angle in (direction-angle_margin,direction,direction+angle_margin):
                        bits |= 1 << int(((angle+math.pi)%(2*math.pi))/(2*math.pi)*8)
                    if angle_margin>=math.pi/4:
                        bits = 255  # excessive arrival uncertainty cannot promise a new sector
                    if record and record[0]>=self.minimum_hits and record[1]&bits:
                        continue
                    mask[cell] = True
                    gain[cell] += int(counts[representative])
                    from exploration_region_memory import surface_claim
                    claim=surface_claim('measured',surface,mass=int(counts[representative]),critical=False)
                    self.last_measured_inspection_claims.setdefault(cell,set()).add(claim)
                    contributed=True
            contributing+=int(contributed)
            if contributing>=limit:break
        self.last_inspection_report=dict(groups_available=len(counts),groups_tried=tried,
            contributing_groups=contributing,trial_limit=trial_limit,
            candidate_cells=int(mask.sum()),bounded_search_exhausted=bool(
                contributing<limit and tried==trial_limit and trial_limit<len(counts)))
        return mask,gain


    @staticmethod
    def fit_ground_plane(points, altitude, bin_size=.8):
        """Robust lower-envelope plane in the CURRENT map frame.

        A plane extrapolation creates sensing requests only. It is never
        inserted into the occupied/free map or counted as observed coverage.
        """
        points=np.asarray(points,dtype=float).reshape((-1,3))
        points=points[np.isfinite(points).all(axis=1)]
        points=points[(points[:,2]<altitude-.55)&(points[:,2]>altitude-3.)]
        if len(points)<30:return None
        bins=np.floor(points[:,:2]/bin_size).astype(np.int32)
        order=np.argsort(points[:,2],kind='stable')
        _,idx=np.unique(bins[order],axis=0,return_index=True)
        low=points[order[idx]]
        if len(low)<25 or min(np.ptp(low[:,:2],axis=0))<2.:return None
        # Stable cap: RANSAC must not depend on scan ordering / random state.
        low=low[::max(1,int(math.ceil(len(low)/2000.)))]
        xy=np.column_stack((low[:,:2],np.ones(len(low))))
        rng=np.random.RandomState(1701);best=None
        for _ in range(80):
            idx=rng.choice(len(low),3,replace=False)
            if abs(np.linalg.det(xy[idx]))<1e-7:continue
            coef=np.linalg.solve(xy[idx],low[idx,2])
            if np.linalg.norm(coef[:2])>math.tan(math.radians(15.)):continue
            inlier=np.abs(xy@coef-low[:,2])<.07
            rank=(int(inlier.sum()),-float(np.median(xy@coef)))
            if best is None or rank>best[0]:best=(rank,inlier,coef)
        if best is None or best[0][0]<max(25,.40*len(low)):return None
        coef=np.linalg.lstsq(xy[best[1]],low[best[1],2],rcond=None)[0]
        # Reject a horizontal shelf deck: at least 90% of measured low
        # samples must be at/above the proposed supporting floor plane.
        if np.mean(low[:,2]<xy@coef-.12)>.10:return None
        return coef

    @staticmethod
    def measured_ceiling(points, altitude):
        """Infer an indoor sensing focus from actual overhead returns only."""
        points=np.asarray(points,dtype=float).reshape(-1,3)
        upper=points.copy();upper[:,2]*=-1
        fitted=SurfaceEvidence.fit_ground_plane(upper,-altitude)
        if fitted is None:return None,np.zeros(len(points),bool)
        plane=-fitted;on=np.abs(points[:,2]-(points[:,:2]@plane[:2]+plane[2]))<=.10
        bins=np.unique(np.floor(points[on,:2]/.8).astype(np.int32),axis=0)
        if len(bins):
            span=np.ptp(bins,axis=0)+1
            if min(span)*.8>=6. and len(bins)/float(np.prod(span))>=.70:return plane,on
        return None,np.zeros(len(points),bool)

    def missing_geometry_candidates(self,grid,distances,current_map,altitude,
            limit=32,range_m=6.,arrival_tolerance=.3,
            min_elevation=-22.2,max_elevation=37.2,lidar_height=.13,view_altitude=None,task_claims_enabled=False):
        """Views for missing floor and never-returned structural boundaries.

        Only measured lower-envelope geometry / measured occupied silhouettes
        are used. Hypothesized floor / silhouette samples are requests, NOT
        map points and NOT quality evidence. A view must lie in the original
        inflated reachable grid and its sightline must clear measured 3D
        occupied voxels and the actual downward sensor cone.
        """
        from ghost_revisit import occupied_voxel_keys,visible_target_count
        from exploration_grid import expand
        began=time.monotonic()
        mask=np.zeros(grid.shape,bool);gain=np.zeros(grid.shape,float)
        self.last_missing_geometry_masks={name:np.zeros(grid.shape,bool) for name in ('floor','boundary')}
        self.last_missing_geometry_regions={name:{} for name in ('floor','boundary')}
        self.last_missing_geometry_claims={name:{} for name in ('floor','boundary')}
        report=dict(floor_plane=None,missing_floor_bins=0,unseen_boundary_groups=0,
            targets_tried=0,visible_floor_targets=0,visible_boundary_targets=0,
            candidate_cells=0,sensing_hypotheses_only=True,generation_ms=0.,
            rejected_fov=0,rejected_3d_occlusion=0,rejected_unsafe_or_unreachable=0,
            deferred_floor_outside_observed_roof=0,
            already_measured_boundary_targets=0,ceiling_connected_groups_skipped=0,targets=[])
        self.last_missing_geometry_report=report
        points=np.asarray(current_map,dtype=float).reshape((-1,3))
        points=points[np.isfinite(points).all(axis=1)]
        plane=self.fit_ground_plane(points,altitude)
        if plane is None:
            report['generation_ms']=(time.monotonic()-began)*1000.
            return mask,gain
        report['floor_plane']=[float(v) for v in plane]
        from exploration_region_memory import MeasuredSurfaceSupport
        face_support=MeasuredSurfaceSupport(getattr(self,'current_points',points),plane) if task_claims_enabled else None
        heights=points[:,2]-(points[:,0]*plane[0]+points[:,1]*plane[1]+plane[2])
        up=np.r_[-plane[:2],1.];up/=np.linalg.norm(up)
        measured_boundary=occupied_voxel_keys(points[heights>.06],.15)
        ground=points[np.abs(heights)<=.10]
        ground_seen=expand(grid.raster(ground),(.15 if task_claims_enabled else .35)/grid.resolution)
        # The target need not fit the UAV footprint; its aerial column needs
        # positive free evidence. This includes floor next to shelves / walls.
        missing=grid.known & ~grid.occupied & ~ground_seen
        coarse=max(1,int(round(.8/grid.resolution)))
        floor_targets=[];entries=[]
        cells=np.argwhere(missing)
        if len(cells):
            bins=cells//coarse
            _,idx,counts=np.unique(bins,axis=0,return_index=True,return_counts=True)
            report['missing_floor_bins']=len(idx)
            # Keep the complete bounded request list. A rotating scheduler
            # below limits actual ray trials instead of permanently dropping
            # the same far rows before visibility is considered.
            for k in np.argsort(-counts,kind='stable'):
                cell=tuple(cells[idx[k]]);x,y=grid.xy(cell)
                z=plane[0]*x+plane[1]*y+plane[2]
                floor_targets.append(('floor',np.array([x,y,z]),float(counts[k]),None))
        # A multi-height measured silhouette can define an inspection need
        # even when the opposite face has never produced a return. Unlike
        # deficits(), this does NOT require a measured voxel on that face.
        structural_mask=(heights>.3)&(heights<3.)
        # Detect a spatially continuous measured overhead plane. Projecting a
        # low warehouse ceiling into the 2D silhouette would mark every aisle
        # occupied and suppress ALL unseen back-face requests. Keep these
        # exact ceiling returns in occupied3d below for physical LOS blocking.
        ceiling,on_ceiling=self.measured_ceiling(points,altitude)
        indoor_target_mask=(expand(grid.raster(points[on_ceiling]),.8/grid.resolution)
            if task_claims_enabled and ceiling is not None else None)
        component_labels=None;components={}
        if task_claims_enabled:
            from exploration_region_memory import floor_deficit_components
            interior_missing=missing if indoor_target_mask is None else missing&indoor_target_mask
            component_labels,components=floor_deficit_components(interior_missing,grid.resolution)
        report['floor_missing_components']=len(components)
        report['floor_largest_missing_component_m2']=max([v['area_m2'] for v in components.values()]+[0.])
        report['ceiling_plane']=None;report['excluded_ceiling_returns']=0
        if ceiling is not None:
            report['ceiling_plane']=ceiling.tolist()
            report['excluded_ceiling_returns']=int(np.count_nonzero(on_ceiling&structural_mask))
            structural_mask &= ~on_ceiling
        structural=points[structural_mask]
        if len(structural):
            xy=np.floor(structural[:,:2]/.8).astype(np.int32)
            unique,inverse,counts=np.unique(xy,axis=0,return_inverse=True,return_counts=True)
            low=np.full(len(unique),np.inf);high=np.full(len(unique),-np.inf)
            np.minimum.at(low,inverse,structural[:,2]);np.maximum.at(high,inverse,structural[:,2])
            occupied=grid.raster(structural)
            entries=[]
            order=np.argsort(inverse,kind='stable')
            chunks=np.split(order,np.cumsum(counts)[:-1])
            tall=np.flatnonzero((high-low>.65)&(counts>=8))
            # A returned shelf top is allowed to anchor its never-seen back
            # boundary if a measured adjacent vertical side establishes the
            # structure. Requiring a tall column ON the missing back face
            # would recreate the old never-returned-surface blind spot.
            eligible=(high-low>.65)&(counts>=8)
            for k in np.flatnonzero((counts>=8)&~eligible):
                if len(tall) and np.min(np.linalg.norm(unique[tall]-unique[k],axis=1))<=2.3:
                    eligible[k]=True
            if report['ceiling_plane'] is not None:
                roof=np.asarray(report['ceiling_plane'])
                centers=(unique+.5)*.8
                reaches_roof=high>=centers@roof[:2]+roof[2]-.25
                report['ceiling_connected_groups_skipped']=int(np.count_nonzero(eligible&reaches_roof))
                # Indoor boundary walls are already handled by frontiers and
                # measured-surface evidence. Do not spend the short inspection
                # budget chasing their exterior backs through door openings.
                # This changes sensing requests only; all wall points still
                # block both navigation and measured 3D line of sight.
                eligible &= ~reaches_roof
            for k in np.flatnonzero(eligible):
                # Use actual measured XY extremum per cardinal side, rather
                # than pretending the entire bin is a solid rectangular box.
                group=structural[chunks[k]]
                for axis,sign in ((0,1),(0,-1),(1,1),(1,-1)):
                    anchor=group[np.argmax(sign*group[:,axis]),:2]
                    normal=np.zeros(2);normal[axis]=sign
                    outside=grid.cell(anchor+normal*.65)
                    if not grid.inside(outside) or not grid.known[outside] or occupied[outside]:continue
                    xyz=np.array([*anchor,altitude])
                    # Angular evidence on the top of a column cannot certify
                    # that its low back face exists in the point cloud. The
                    # per-height measured-return check below owns this task.
                    entries.append(('boundary',xyz,min(16.,float(counts[k])/4.),normal))
            report['unseen_boundary_groups']=len(entries)
        # One outward representative per direction in each spatial bin. A
        # wall/front with many points must not erase the less sampled back.
        boundary_bins={}
        for request in entries:
            _,target,_,normal=request
            region=tuple(np.floor(target[:2]/2.4).astype(int))
            direction=tuple(normal.astype(int));key=(region,direction)
            old=boundary_bins.get(key)
            center=(np.asarray(region)+.5)*2.4
            rank=(float(np.dot(target[:2],normal)),-float(np.linalg.norm(target[:2]-center)))
            if old is None or rank>old[0]:boundary_bins[key]=(rank,request)
        by_region={}
        for (region,_),(_,request) in boundary_bins.items():
            kind,target,mass,normal=request
            floor_z=plane[0]*target[0]+plane[1]*target[1]+plane[2]
            # A single target at map flight altitude can fall on the shelf's
            # top in the tilted map frame, leaving its low back face blind.
            # Request both low and middle face bands, never synthetic returns.
            heights_to_inspect=np.unique([min(altitude,floor_z+.10),min(altitude,floor_z+1.0)])
            for height in heights_to_inspect:
                point=target.copy();point[2]=height
                key=np.floor(point/.15).astype(int)
                if any(tuple(key+(dx,dy,0)) in measured_boundary
                        for dx in (-1,0,1) for dy in (-1,0,1)):
                    report['already_measured_boundary_targets']+=1;continue
                by_region.setdefault(region,[]).append((kind,point,mass,normal))
        queues={'floor':[[r] for r in floor_targets],
                'boundary':[by_region[k] for k in sorted(by_region)]}
        cursors=getattr(self,'missing_geometry_cursors',{'floor':0,'boundary':0})
        self.missing_geometry_cursors=cursors
        report['spatial_groups_available']={k:len(v) for k,v in queues.items()}
        report['spatial_groups_tried']={k:0 for k in queues}
        report['spatial_groups_contributed']={k:0 for k in queues}
        report['target_trials_cap_per_kind']=max(32,4*limit)
        occupied3d=occupied_voxel_keys(points,.20)
        view_cells=np.argwhere(grid.safe & np.isfinite(distances))
        view_xy=np.array([grid.xy(cell) for cell in view_cells],dtype=float).reshape(-1,2)
        view_cost=distances[tuple(view_cells.T)]
        view_z=(np.full(len(view_xy),altitude) if view_altitude is None else
                np.broadcast_to(view_altitude(view_xy),(len(view_xy),)))
        view_xyz=np.column_stack((view_xy,view_z))+lidar_height*up
        def inspect(request):
            kind,target,mass,normal=request
            # Optional floor refinement stays under actually observed indoor
            # structure. Unknown rooms remain available to frontier exploration.
            if kind=='floor' and indoor_target_mask is not None:
                cell=grid.cell(target[:2])
                if not grid.inside(cell) or not indoor_target_mask[cell]:
                    report['deferred_floor_outside_observed_roof']+=1
                    return False
            report['targets_tried']+=1;contributed=False
            floor_z=plane[0]*target[0]+plane[1]*target[1]+plane[2]
            # Enumerate the ACTUAL safe lattice. Sparse polar rings can
            # jump over an entire narrow aisle after safety inflation.
            delta=view_xy-target[:2]
            radial=np.linalg.norm(delta,axis=1)
            possible=(radial>=1.5)&(radial<=range_m)&(view_cost>=1.2)
            if normal is not None:
                possible &= delta@normal>=.15*radial
            ids=np.flatnonzero(possible)
            if len(ids):
                ray=target-view_xyz[ids]
                vertical=ray@up
                horizontal=np.sqrt(np.maximum(0.,np.sum(ray*ray,axis=1)-vertical**2))
                uncertainty=.08+arrival_tolerance*np.linalg.norm(plane[:2])
                angles=np.array([np.degrees(np.arctan2(vertical+dz,
                    np.maximum(.01,horizontal+dr))) for dz in (-uncertainty,uncertainty)
                    for dr in (-arrival_tolerance,arrival_tolerance)])
                cone=(angles.min(axis=0)>=min_elevation+2.)&(angles.max(axis=0)<=max_elevation-2.)
                report['rejected_fov']+=int(np.count_nonzero(~cone))
                ids=ids[cone]
            # Keep at most 24 visible cells per request, with shortest actual
            # route cost first. Occluded cells never consume that quota.
            ids=ids[np.argsort(view_cost[ids],kind='stable')]
            accepted=0
            for index in ids:
                cell=tuple(view_cells[index]);view=view_xyz[index]
                if not visible_target_count(view,[target],occupied3d,.20,
                        -90.,90.,endpoint_margin=.22):
                    report['rejected_3d_occlusion']+=1;continue
                mask[cell]=True;gain[cell]+=mass;contributed=True
                self.last_missing_geometry_masks[kind][cell]=True
                from exploration_region_memory import surface_claim
                seen,count=face_support.assess(kind,target,() if normal is None else normal) if face_support is not None else (False,0)
                origin=float(target[:2]@plane[:2]+plane[2])
                component=components.get(int(component_labels[grid.cell(target[:2])]),{}) if kind=='floor' and component_labels is not None else {}
                critical=(component.get('area_m2',0.)>=.5 and component.get('core_cells',0)>0) if kind=='floor' and task_claims_enabled else not seen
                claim=surface_claim(kind,target,() if normal is None else normal,mass,
                    height_origin=origin,critical=critical,support_voxels=count,missing_area_m2=component.get('area_m2',0.))
                self.last_missing_geometry_regions[kind].setdefault(cell,set()).add(
                    claim.key if task_claims_enabled else (kind,*map(int,np.floor(target[:2]/2.4))))
                self.last_missing_geometry_claims[kind].setdefault(cell,set()).add(claim)
                accepted+=1
                if accepted>=24:break
            if contributed:report['visible_'+kind+'_targets']+=1
            report['targets'].append(dict(kind=kind,xyz=target.tolist(),contributed=contributed))
            return contributed
        for kind,groups in queues.items():
            if not groups:continue
            # Coprime stepping spreads consecutive attempts across the map;
            # persistent cursor visits every bin, independent of map size.
            stride=max(1,int(math.sqrt(len(groups))))
            while math.gcd(stride,len(groups))!=1:stride+=1
            before=report['targets_tried'];tried=0
            for offset in range(len(groups)):
                if report['targets_tried']-before>=report['target_trials_cap_per_kind']:break
                index=((cursors.get(kind,0)+offset)*stride)%len(groups)
                group=groups[index];contributed=False
                for request in group:
                    if report['targets_tried']-before>=report['target_trials_cap_per_kind']:break
                    contributed=inspect(request) or contributed
                tried+=1
                report['spatial_groups_contributed'][kind]+=int(contributed)
                if report['spatial_groups_contributed'][kind]>=max(1,limit//2):break
            cursors[kind]=(cursors.get(kind,0)+tried)%len(groups)
            report['spatial_groups_tried'][kind]=tried
        report['next_spatial_cursor']={k:int(v) for k,v in cursors.items()}
        report['candidate_cells']=int(mask.sum())
        report['candidate_cells_by_kind']={k:int(v.sum()) for k,v in self.last_missing_geometry_masks.items()}
        report['generation_ms']=(time.monotonic()-began)*1000.
        return mask,gain
