#!/usr/bin/env python3
"""Choose observed-safe LiDAR views of suspected old dynamic returns.

The input points are hypotheses from the maintained map, not collision-free
space. This module never changes occupancy or relaxes the navigation guard.
"""
import math
import numpy as np


def pending_diagnostics(history,phase,selection=None,limit=128):
    """Explain unresolved records without certifying them or using scene truth."""
    result=[]
    for r in history.get('records',[]):
        if r.get('state')!='suspected':continue
        point=r.get('position',[])
        if len(point)!=3:continue
        group=str(tuple(np.floor(np.asarray(point[:2])/.8).astype(int)))
        if r.get('positive_endpoint_frames',0):reason='positive_returns_need_multiview_baseline'
        elif r.get('free_evidence_frames',0):reason='insufficient_consistent_free_rays'
        elif r.get('post_departure_hit_frames',0):reason='occupied_reconfirmation_needs_more_views'
        elif r.get('protected_endpoint_frames',0):reason='protected_by_measured_endpoint'
        elif r.get('off_point_ray_frames',0):reason='rays_cross_voxel_but_miss_stored_point'
        else:reason='no_point_aligned_ray_evidence'
        result.append(dict(key=r.get('key'),position=point,group=group,phase=phase,
            evidence_reason=reason,free_evidence_frames=r.get('free_evidence_frames',0),
            post_departure_hit_frames=r.get('post_departure_hit_frames',0),
            positive_endpoint_frames=r.get('positive_endpoint_frames',0),
            positive_endpoint_baseline=r.get('positive_endpoint_baseline',0.),
            view_selection=(selection or {}).get('groups',{}).get(group,{})))
        if len(result)>=limit:break
    return dict(pending_total=sum(r.get('state')=='suspected' for r in history.get('records',[])),
                records=result,truncated=len(result)<sum(r.get('state')=='suspected' for r in history.get('records',[])))


def suspect_groups(points, altitude, height_below=.45, height_above=.55,
                   group_size=.8, limit=16, priorities=None):
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] >= altitude-height_below) &
                    (points[:, 2] <= altitude+height_above)]
    groups = {}
    for p in points:
        key = tuple(np.floor(p[:2]/group_size).astype(int))
        groups.setdefault(key, []).append(p[:2])
    return [(key, np.mean(values, axis=0), len(values))
            for key, values in sorted(groups.items(),
                key=lambda item: -(priorities.get(item[0],len(item[1]))
                                   if priorities is not None else len(item[1])))[:limit]]


def history_group_priorities(history, altitude, height_below=.45,
                             height_above=.55, group_size=.8):
    """Aggregate only actionable, still-suspected 3-D voxel evidence.

    Cleared history has a separate, lower-priority review path. It is never
    sent back as a navigation obstacle or treated as current occupancy.
    """
    priorities = {}
    for record in history.get('records', []):
        if record.get('state') != 'suspected' or not record.get('actionable'):
            continue
        point = record.get('position', ())
        if len(point) != 3 or not all(math.isfinite(float(v)) for v in point):
            continue
        if not altitude-height_below <= float(point[2]) <= altitude+height_above:
            continue
        weight = float(record.get('priority', 0.))
        if not math.isfinite(weight) or weight <= 0.:
            continue
        key = tuple(np.floor(np.asarray(point[:2], dtype=float)/group_size).astype(int))
        priorities[key] = priorities.get(key, 0.) + weight
    return priorities


def cleared_history_groups(history, altitude, height_below=.45,
                           height_above=.55, group_size=.8, limit=16):
    """Return historical cleared locations as optional observation targets.

    A cleared record is eligible only while the same cell remains unoccupied.
    Reaching a viewpoint is an inspection attempt, not proof of free 3-D space.
    """
    groups = {}
    for record in history.get('records', []):
        if (record.get('state') != 'cleared' or
                record.get('occupied_now') is not False):
            continue
        episodes = record.get('cleared_episodes', 0)
        if not isinstance(episodes, (int, float)) or not math.isfinite(episodes) or episodes < 1:
            continue
        point = record.get('position', ())
        if len(point) != 3 or not all(math.isfinite(float(v)) for v in point):
            continue
        if not altitude-height_below <= float(point[2]) <= altitude+height_above:
            continue
        key = tuple(np.floor(np.asarray(point[:2], dtype=float)/group_size).astype(int))
        center, weight = groups.setdefault(key, ([], 0.))
        center.append(np.asarray(point[:2], dtype=float))
        # A cleared location is a hint to review, never a live ghost claim.
        groups[key] = (center, weight + .25*(1. + .25*min(episodes-1, 3)))
    ordered = sorted(groups.items(), key=lambda item: (-item[1][1], item[0]))[:limit]
    views = [(key, np.mean(points, axis=0), len(points))
             for key, (points, _) in ordered]
    priorities = {key: weight for key, (_, weight) in ordered}
    return views, priorities


def observed_los(grid, a, b, endpoint_margin=.30):
    """Require known, unobstructed interior; endpoint may be occupied or free."""
    length = math.dist(a, b)
    if length <= endpoint_margin:
        return False
    count = max(2, int(math.ceil(length/(grid.resolution*.4))))
    for t in np.linspace(0., 1.-endpoint_margin/length, count, endpoint=False):
        cell = grid.cell((a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])))
        if not grid.inside(cell) or not grid.known[cell] or grid.occupied[cell]:
            return False
    return True


def occupied_voxel_keys(points, voxel_size):
    """Current map support for a 3-D occlusion proxy; absence is not free space."""
    if not math.isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError('Invalid map voxel size')
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    return {tuple(key) for key in np.floor(
        points[np.isfinite(points).all(axis=1)] / voxel_size).astype(np.int32)}


def _segment_intersects_box(start, end, lower, upper):
    """Closed segment versus closed AABB; touching a voxel is a collision."""
    t_low, t_high = 0., 1.
    for axis in range(3):
        delta = end[axis]-start[axis]
        if abs(delta) < 1e-12:
            if start[axis] < lower[axis] or start[axis] > upper[axis]:
                return False
            continue
        near = (lower[axis]-start[axis])/delta
        far = (upper[axis]-start[axis])/delta
        t_low = max(t_low, min(near, far))
        t_high = min(t_high, max(near, far))
        if t_low > t_high:
            return False
    return True


def certify_3d_swept_route(route, free_voxels, occupied_voxels, voxel_size,
                           body_radius, body_half_height, free_stamp,
                           occupied_stamp, now, max_age, max_checked=200000):
    """Conservatively certify a full body-box sweep in a measured 3-D grid.

    Every voxel touched by each segment expanded by the horizontal rotor
    radius and vertical half-height must have explicit recent free evidence.
    Occupied evidence wins over free evidence. Unknown, stale, or oversized
    searches fail closed. The box over-approximates a round rotor footprint.
    This voxel-model certificate is not a controller authorization or proof of
    continuous physical clearance between LiDAR rays.
    """
    route = np.asarray(route, dtype=float)
    if route.ndim != 2 or route.shape[1] != 3 or len(route) < 2 or not np.isfinite(route).all():
        raise ValueError('3-D route must contain at least two finite XYZ points')
    for name, value, minimum in (
            ('voxel_size', voxel_size, 0.), ('body_radius', body_radius, 0.),
            ('body_half_height', body_half_height, 0.), ('max_age', max_age, 0.)):
        if not math.isfinite(value) or value <= minimum:
            raise ValueError('%s must be finite and positive' % name)
    if not isinstance(max_checked, int) or max_checked < 1:
        raise ValueError('max_checked must be a positive integer')
    if not math.isfinite(now):
        raise ValueError('now must be finite')
    for name, stamp in (('free', free_stamp), ('occupied', occupied_stamp)):
        if stamp is None or not math.isfinite(stamp) or not 0. <= now-stamp <= max_age:
            return dict(certified=False, reason='stale_%s_evidence' % name,
                        checked_voxels=0, first_blocked_voxel=None)
    free = set(free_voxels)
    occupied = set(occupied_voxels)
    inflation = np.array([body_radius, body_radius, body_half_height])
    checked = set()
    for start, end in zip(route[:-1], route[1:]):
        low = np.floor((np.minimum(start, end)-inflation)/voxel_size).astype(int)-1
        high = np.floor((np.maximum(start, end)+inflation)/voxel_size).astype(int)
        # Limit the bounding search before allocating or iterating a huge grid.
        volume = int(np.prod((high-low+1).astype(object)))
        if volume > max_checked:
            return dict(certified=False, reason='resource_limit',
                        checked_voxels=len(checked), first_blocked_voxel=None)
        for ix in range(low[0], high[0]+1):
            for iy in range(low[1], high[1]+1):
                for iz in range(low[2], high[2]+1):
                    key = (ix, iy, iz)
                    if key in checked:
                        continue
                    voxel_min = np.array(key, dtype=float)*voxel_size-inflation
                    voxel_max = voxel_min+voxel_size+2.*inflation
                    if not _segment_intersects_box(start, end, voxel_min, voxel_max):
                        continue
                    checked.add(key)
                    if len(checked) > max_checked:
                        return dict(certified=False, reason='resource_limit',
                                    checked_voxels=len(checked), first_blocked_voxel=None)
                    if key in occupied:
                        return dict(certified=False, reason='occupied',
                                    checked_voxels=len(checked), first_blocked_voxel=key)
                    if key not in free:
                        return dict(certified=False, reason='unknown',
                                    checked_voxels=len(checked), first_blocked_voxel=key)
    return dict(certified=True, reason='known_free_voxel_sweep',
                checked_voxels=len(checked), first_blocked_voxel=None)


def validate_height_offsets(offsets):
    """Bound read-only height probes; no value here authorizes vertical flight."""
    if not isinstance(offsets, (list, tuple)) or not 1 <= len(offsets) <= 6:
        raise ValueError('ghost_vertical_view_offsets_m must contain 1..6 offsets')
    values = []
    for offset in offsets:
        if not isinstance(offset, (int, float)) or not math.isfinite(offset) or not .05 <= abs(offset) <= 2.:
            raise ValueError('ghost vertical offsets must be finite and 0.05..2 m from flight height')
        values.append(float(offset))
    if len(set(values)) != len(values):
        raise ValueError('ghost vertical offsets must be distinct')
    return tuple(values)


def visible_target_count(view, targets, occupied, voxel_size=.15,
                         min_elevation_deg=-22.2, max_elevation_deg=37.2,
                         endpoint_margin=.30):
    """Count targets inside modeled LiDAR elevation with no mapped blocker.

    This is a planning proxy, not proof that a sparse/nonrepeating scan actually
    crossed a target.  The map builder alone decides free/occupied evidence.
    """
    view = np.asarray(view, dtype=float)
    count = 0
    for target in np.asarray(targets, dtype=float).reshape((-1, 3)):
        delta = target-view
        horizontal = float(np.linalg.norm(delta[:2]))
        distance = float(np.linalg.norm(delta))
        if distance <= endpoint_margin or horizontal <= 1e-6:
            continue
        elevation = math.degrees(math.atan2(delta[2], horizontal))
        if not min_elevation_deg <= elevation <= max_elevation_deg:
            continue
        direction = delta/distance
        blocked = False
        for length in np.arange(voxel_size*.5, distance-endpoint_margin,
                                voxel_size*.5):
            key = tuple(np.floor((view+direction*length)/voxel_size).astype(np.int32))
            if key in occupied:
                blocked = True
                break
        if not blocked:
            count += 1
    return count


def select_revisit_view(grid, distances, groups, position, visits,
                        minimum_range=1.2, maximum_range=3.0,
                        min_view_change=.7, max_attempts=2,
                        priorities=None, speed_mps=.45, dwell_seconds=2.,
                        remaining_seconds=None, return_distances=None,
                        reserve_seconds=0., target_points=None,
                        occupied_voxels=None, sensor_altitude=None,
                        voxel_size=.15, min_elevation_deg=-22.2,
                        max_elevation_deg=37.2, phase_remaining_seconds=None,
                        return_speed_mps=None, diagnostics=None):
    """Return (safe cell, group key), or (None, None) if no view is reachable.

    Every candidate uses the existing fully inflated safe grid and finite path
    search distance. Line of sight is checked on raw occupancy and known cells.
    A second attempt must use a distinct view, never an arrival loop. Optional
    3-D occlusion checks reject views with no currently visible target voxel.
    """
    if not math.isfinite(speed_mps) or speed_mps <= 0 or dwell_seconds < 0 or reserve_seconds < 0:
        raise ValueError('Invalid revisit time estimate')
    return_speed_mps = speed_mps if return_speed_mps is None else return_speed_mps
    if not math.isfinite(return_speed_mps) or return_speed_mps<=0:
        raise ValueError('Invalid return time estimate')
    if diagnostics is not None:
        diagnostics.clear();diagnostics.update(groups={},selected_group=None)
    def reject(key,reason):
        if diagnostics is not None:
            counts=diagnostics['groups'].setdefault(str(key),{})
            counts[reason]=counts.get(reason,0)+1
    best = None
    radius = int(math.ceil(maximum_range/grid.resolution))
    for key, center, count in groups:
        previous = visits.get(key, [])
        attempt_limit = max_attempts.get(key, 0) if isinstance(max_attempts, dict) else max_attempts
        if len(previous) >= attempt_limit:
            reject(key,'attempts_exhausted')
            continue
        cy, cx = grid.cell(center)
        for y in range(max(0, cy-radius), min(grid.shape[0], cy+radius+1)):
            for x in range(max(0, cx-radius), min(grid.shape[1], cx+radius+1)):
                cell = (y, x)
                if not grid.safe[cell] or not np.isfinite(distances[cell]):
                    reject(key,'unsafe_or_unreachable')
                    continue
                outbound_time = float(distances[cell])/speed_mps
                return_time = 0.
                if return_distances is not None:
                    if not np.isfinite(return_distances[cell]):
                        continue
                    return_time = float(return_distances[cell])/return_speed_mps
                if (phase_remaining_seconds is not None and
                        outbound_time+dwell_seconds+reserve_seconds>phase_remaining_seconds):
                    reject(key,'revisit_phase_budget')
                    continue
                if (remaining_seconds is not None and
                        outbound_time+dwell_seconds+return_time+reserve_seconds > remaining_seconds):
                    reject(key,'flight_return_budget')
                    continue
                xy = grid.xy(cell)
                range_m = math.dist(xy, center)
                if not minimum_range <= range_m <= maximum_range:
                    reject(key,'sensor_range')
                    continue
                if math.dist(xy, position) < min_view_change:
                    reject(key,'view_baseline')
                    continue
                if any(math.dist(xy, old) < min_view_change for old in previous):
                    reject(key,'repeated_view')
                    continue
                if not observed_los(grid, xy, center):
                    reject(key,'blocked_or_unknown_2d_los')
                    continue
                if target_points is not None and occupied_voxels is not None:
                    targets = target_points.get(key, ())
                    if not len(targets) or sensor_altitude is None or not visible_target_count(
                            (xy[0],xy[1],sensor_altitude), targets,
                            occupied_voxels, voxel_size,
                            min_elevation_deg, max_elevation_deg):
                        reject(key,'occluded_or_outside_vertical_fov')
                        continue
                reject(key,'eligible_views')
                value = float(priorities.get(key, count)) if priorities is not None else float(count)
                if value <= 0 or not math.isfinite(value):
                    continue
                # The grid's search cost already includes clearance penalties.
                # Return cost is a planning estimate, not a flight-time promise.
                score = value/(1.+outbound_time+dwell_seconds+return_time)
                if best is None or score > best[0]:
                    best = (score, cell, key)
    if best is not None and diagnostics is not None:
        diagnostics['selected_group']=str(best[2])
    return (best[1], best[2]) if best is not None else (None, None)


def diagnose_alternative_height_views(grid, distances, groups, position,
                                      target_points, occupied_voxels,
                                      free_voxels, current_altitude, offsets,
                                      free_stamp, occupied_stamp, now,
                                      max_age, voxel_size=.15,
                                      body_radius=.45, body_half_height=.55,
                                      minimum_range=1.2, maximum_range=3.,
                                      min_elevation_deg=-22.2,
                                      max_elevation_deg=37.2, priorities=None):
    """Report geometry-only alternate-height views, never executable goals.

    Candidate XY cells are reachable in the *current* flight-height grid.
    Their proposed Z values are tested for a vertical voxel sweep at that XY,
    but this does not certify an approach route at the new altitude, controller
    behavior, or collision-free travel in continuously unknown space.
    """
    offsets = validate_height_offsets(offsets)
    views = []
    for offset in offsets:
        altitude = current_altitude+offset
        if not math.isfinite(altitude):
            continue
        cell, key = select_revisit_view(
            grid, distances, groups, position, {},
            minimum_range=minimum_range, maximum_range=maximum_range,
            min_view_change=0., max_attempts=1, priorities=priorities,
            target_points=target_points, occupied_voxels=occupied_voxels,
            sensor_altitude=altitude, voxel_size=voxel_size,
            min_elevation_deg=min_elevation_deg,
            max_elevation_deg=max_elevation_deg)
        if cell is None:
            continue
        xy = grid.xy(cell)
        targets = target_points.get(key, ())
        current_visible = visible_target_count(
            (xy[0],xy[1],current_altitude), targets, occupied_voxels,
            voxel_size,min_elevation_deg,max_elevation_deg)
        alternate_visible = visible_target_count(
            (xy[0],xy[1],altitude), targets, occupied_voxels,
            voxel_size,min_elevation_deg,max_elevation_deg)
        vertical = certify_3d_swept_route(
            [(xy[0],xy[1],current_altitude),(xy[0],xy[1],altitude)],
            free_voxels,occupied_voxels,voxel_size,body_radius,
            body_half_height,free_stamp,occupied_stamp,now,max_age)
        views.append(dict(group=list(key),view_xy=list(xy),
                          proposed_altitude_m=altitude,
                          current_altitude_visible_targets=current_visible,
                          proposed_altitude_visible_targets=alternate_visible,
                          proxy_target_gain=alternate_visible-current_visible,
                          vertical_sweep=vertical,
                          flight_authorized=False,
                          reason='no_certified_full_3d_route_or_controller_integration'))
    views.sort(key=lambda item:(-item['proxy_target_gain'],
                                -item['proposed_altitude_visible_targets'],
                                item['proposed_altitude_m']))
    return views
