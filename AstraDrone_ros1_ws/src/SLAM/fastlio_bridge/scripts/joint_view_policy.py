#!/usr/bin/env python3
"""Bounded observation selection; consumes measured maps, never simulator truth.

All hypothetical maps are private arrays used to rank *observations*. Returned
viewpoints always belong to the original inflated safe grid. Quality scores
are uncalibrated heuristics, not probabilities or clearance certificates.
"""
from dataclasses import dataclass, field
import hashlib
import math
import time
import numpy as np
from exploration_grid import expand
from exploration_native import search as native_search
from ghost_revisit import observed_los, visible_target_count
from exploration_route_memory import RouteMemory, RouteQuery, reversal_cost
from corridor_progress import CorridorProgress
from exploration_region_memory import RegionMemory
from exploration_coverage import visible_unknown_gain


DEFAULTS = dict(
    joint_selection_enabled=True, joint_strategy='channel_verification',
    joint_decision_hz=1., joint_frontier_candidate_limit=24,
    joint_ghost_group_limit=12, joint_candidate_azimuth_samples=16,
    joint_view_ranges_m=[1.5, 2.5, 4.], joint_retained_views_per_group=8,
    joint_total_candidate_limit=48, joint_region_size_m=2.5,
    joint_region_limit=256, joint_topology_update_seconds=5.,
    joint_lookahead_depth=2, joint_lookahead_branching=6,
    joint_lookahead_discount=.7, joint_weight_area=.5,
    joint_weight_verify=.3, joint_weight_connectivity=.2,
    joint_area_scale_m2=10., joint_verify_group_scale=3.,
    joint_connectivity_area_scale_m2=20., joint_min_evidence_quality=.15,
    joint_history_review_scale=.25, joint_score_epsilon_seconds=2.,
    joint_target_min_hold_seconds=3., joint_switch_improvement_ratio=.2,
    joint_ghost_action_budget_seconds=90., joint_ghost_cooldown_seconds=30.,
    joint_min_fresh_view_scans=8, joint_max_view_seconds=6.,
    joint_low_gain_windows=2, joint_extra_compute_budget_ms=50.,
    joint_replan_on_safety_invalidation=True,
    joint_surface_inspection_enabled=False, joint_unseen_geometry_enabled=False,
    joint_surface_candidate_limit=6,
    joint_viewpoint_sweep_enabled=False,joint_surface_dwell_seconds=1.,
    joint_bind_sweep_to_geometry=False,joint_stop_low_gain_surface=False,
    joint_surface_low_gain_limit=2,joint_surface_min_new_voxels=8,
    joint_surface_frontier_gate_m2=0.,
    joint_defer_home_review=False,joint_home_review_radius_m=5.,
    joint_home_ghost_reward_scale=1.,
    joint_return_review_seconds=15.,joint_return_review_max_detour_m=2.,
    joint_surface_gain_scale=80., joint_missing_geometry_gain_scale=16.,
    joint_measured_refinement_scale=.15,
    joint_indoor_structure_focus=False,joint_outside_ceiling_gain_scale=.02,
    joint_surface_start_seconds=180.,
    joint_surface_action_budget_seconds=60.,
    joint_route_memory_enabled=False, joint_route_tile_size_m=.8,
    joint_route_repeat_weight=.6, joint_route_reverse_seconds=6.,
    joint_route_region_size_m=6., joint_route_region_switch_seconds=6.,
    joint_unknown_gain_multiplier=1.,joint_corridor_progress_enabled=False,
    joint_corridor_task_seconds=100.,joint_corridor_step_m=4.5,
    joint_corridor_max_width_m=4.5,joint_corridor_entry_enabled=False,
    joint_region_feedback_enabled=False,joint_region_feedback_cooldown_s=60.,
    joint_region_low_yield_limit=2,joint_tail_min_area_per_extra_s=.10,
    joint_tail_min_extra_s=5.,joint_active_ghost_gain_multiplier=1.)


def parameters(getter):
    values = {k: getter(k, v) for k, v in DEFAULTS.items()}
    for k, default in DEFAULTS.items():
        v = values[k]
        if isinstance(default, bool):
            if type(v) is not bool:
                raise ValueError(k + ' must be boolean')
        elif isinstance(default, str):
            if v not in ('simple', 'channel_verification'):
                raise ValueError('Unknown joint strategy')
        elif isinstance(default, list):
            if not isinstance(v, (list, tuple)) or not 1 <= len(v) <= 8 or not all(
                    isinstance(x, (int, float)) and math.isfinite(x) and 1.2 <= x <= 4.5 for x in v):
                raise ValueError(k + ' must contain finite ranges in [1.2,4.5]')
        elif isinstance(default, int):
            if type(v) is not int or not 1 <= v <= 1000:
                raise ValueError(k + ' must be an integer in [1,1000]')
        elif not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
            raise ValueError(k + ' must be finite and nonnegative')
    for k in ('joint_decision_hz', 'joint_region_size_m', 'joint_area_scale_m2',
              'joint_verify_group_scale', 'joint_connectivity_area_scale_m2',
              'joint_score_epsilon_seconds', 'joint_max_view_seconds',
              'joint_extra_compute_budget_ms', 'joint_topology_update_seconds',
              'joint_surface_gain_scale','joint_missing_geometry_gain_scale','joint_surface_dwell_seconds',
              'joint_route_tile_size_m','joint_route_region_size_m','joint_unknown_gain_multiplier',
              'joint_corridor_task_seconds','joint_corridor_step_m','joint_corridor_max_width_m',
              'joint_region_feedback_cooldown_s','joint_tail_min_extra_s',
              'joint_active_ghost_gain_multiplier'):
        if values[k] <= 0:
            raise ValueError(k + ' must be positive')
    for k in ('joint_lookahead_discount', 'joint_min_evidence_quality', 'joint_history_review_scale',
              'joint_measured_refinement_scale','joint_outside_ceiling_gain_scale',
              'joint_home_ghost_reward_scale'):
        if not 0 <= values[k] <= 1:
            raise ValueError(k + ' must be in [0,1]')
    if values['joint_lookahead_depth'] > 2 or values['joint_lookahead_branching'] > 8:
        raise ValueError('Joint lookahead is bounded to depth 2 / branching 8')
    if not values['joint_replan_on_safety_invalidation']:
        raise ValueError('Safety invalidation must override target hold')
    if not math.isclose(sum(values[k] for k in (
            'joint_weight_area', 'joint_weight_verify', 'joint_weight_connectivity')), 1., abs_tol=1e-9):
        raise ValueError('Joint utility weights must sum to one')
    return values


def reserved_home_review_seconds(getter):
    """Zero home ghost utility disables its optional return observation reserve."""
    if not getter('joint_defer_home_review',False) or getter('joint_home_ghost_reward_scale',1.)==0.:
        return 0.
    return getter('joint_return_review_seconds',15.)


@dataclass
class View:
    cell: tuple
    xy: tuple
    kind: str
    area_m2: float = 0.
    group: tuple = None
    episode: tuple = None
    priority: float = 0.
    quality: float = 0.
    connectivity_m2: float = 0.
    travel_s: float = 0.
    home_s: float = 0.
    dwell_s: float = 0.
    score: float = 0.
    rewards: frozenset = field(default_factory=frozenset)
    inspection_source: str = 'measured'
    inspection_regions: tuple = ()
    repeated_path_m: float = 0.
    repeat_cost_s: float = 0.
    reversal_cost_s: float = 0.
    region_switch_cost_s: float = 0.
    ghost_reward_scale: float = 1.
    corridor_task: bool = False
    benefit_claims: tuple = ()
    feedback_scale: float = 1.
    extra_return_seconds: float = 0.
    extra_area_rate: float = 0.

    @property
    def route_cost_s(self):
        return self.repeat_cost_s+self.reversal_cost_s+self.region_switch_cost_s

    def summary(self):
        return dict(xy=list(self.xy), kind=self.kind,inspection_source=self.inspection_source,
                    inspection_regions=[list(r) for r in self.inspection_regions],
                    group=[int(x) for x in self.group] if self.group is not None else None,
                    episode=[int(x) if isinstance(x, (int, np.integer)) else x for x in self.episode]
                    if self.episode is not None else None,
                    area_m2=self.area_m2, quality=self.quality,
                    priority=self.priority, connectivity_proxy_m2=self.connectivity_m2,
                    travel_s=self.travel_s, home_s=self.home_s,
                    dwell_s=self.dwell_s, score=self.score,
                    repeated_path_m=self.repeated_path_m,repeat_cost_s=self.repeat_cost_s,
                    reversal_cost_s=self.reversal_cost_s,region_switch_cost_s=self.region_switch_cost_s,
                    ghost_reward_scale=self.ghost_reward_scale,corridor_task=self.corridor_task,
                    feedback_scale=self.feedback_scale,extra_return_seconds=self.extra_return_seconds,
                    extra_area_rate=self.extra_area_rate,benefit_claims=[c.summary() for c in self.benefit_claims[:24]])


class JointViewPolicy:
    def __init__(self, getter=lambda k, v: v, clock=time.monotonic):
        self.p = parameters(getter)
        self.clock = clock
        self.stats = {}  # keyed by group and observed episode, never by arrival
        self.used_ghost_seconds = 0.
        self.used_surface_seconds = 0.
        self.last_decision = -float('inf')
        self.last_selection_stamp = -float('inf')
        self.selected = None
        self.report = dict(enabled=True, decisions=0, candidates=0)
        self.topology_cache = None
        self.inspection_region_visits = {}
        self.surface_low_gain_count=0
        self.surface_stopped=False
        self.route_memory=RouteMemory(self.p['joint_route_tile_size_m'])
        self.active_route_region=None
        self.region_memory=RegionMemory(cooldown=self.p['joint_region_feedback_cooldown_s'],
            low_limit=self.p['joint_region_low_yield_limit'])
        self.corridor=CorridorProgress(self.p['joint_corridor_task_seconds'],
            self.p['joint_corridor_step_m'],self.p['joint_corridor_max_width_m'],
            self.p['joint_corridor_entry_enabled'])

    def invalidate_coordinates(self):
        # Flight clock, action budget, attempts and outcome diagnostics survive.
        self.selected = None
        self.last_decision = -float('inf')
        self.topology_cache = None
        self.inspection_region_visits.clear()
        self.surface_low_gain_count=0;self.surface_stopped=False
        self.route_memory.reset_coordinates();self.active_route_region=None
        self.corridor.reset();self.region_memory.reset_coordinates()

    def observe_position(self, position):
        if self.p['joint_route_memory_enabled']:
            self.route_memory.observe(position)
        if self.p['joint_corridor_progress_enabled']:
            self.corridor.observe(position)

    def route_region(self, xy):
        return tuple(math.floor(float(v)/self.p['joint_route_region_size_m']) for v in xy)

    def diverse(self, rows, key, limit):
        if not self.p['joint_route_memory_enabled']:
            return rows[:limit]
        first=[];rest=[];seen=set()
        for row in rows:
            region=key(row)
            if region in seen:rest.append(row)
            else:first.append(row);seen.add(region)
        return (first+rest)[:limit]

    def observe_surface_gain(self, new_voxels):
        """Measured new surface voxels across travel AND dwell, not body visits."""
        if not self.p['joint_stop_low_gain_surface']:return
        self.surface_low_gain_count=(self.surface_low_gain_count+1
            if new_voxels<self.p['joint_surface_min_new_voxels'] else 0)
        stopped=self.surface_low_gain_count>=self.p['joint_surface_low_gain_limit']
        self.surface_stopped=(self.surface_stopped or stopped) if self.p['joint_region_feedback_enabled'] else stopped

    def complete_inspection(self, view):
        """Charge sensing regions after actual arrival, never after selection."""
        for region in view.inspection_regions:
            key=tuple(region)
            if key in self.inspection_region_visits or len(self.inspection_region_visits)<self.p['joint_region_limit']:
                self.inspection_region_visits[key]=self.inspection_region_visits.get(key,0)+1

    def charge(self, seconds):
        if math.isfinite(seconds) and seconds > 0:
            self.used_ghost_seconds += seconds

    def complete_view(self, view, stamp, effective, measured=True):
        if view.episode is None:
            return
        effective = bool(measured and effective)
        stats = self.stats.setdefault(view.episode, dict(views=[], measured=0, effective=0, cooldown=-float('inf')))
        stats['views'].append(view.xy)
        stats['measured'] += int(measured)
        stats['effective'] += int(measured and effective)
        if not effective:
            stats['cooldown'] = stamp + self.p['joint_ghost_cooldown_seconds']
        self.selected = None
        self.last_decision = -float('inf')

    def _region(self, grid, cell):
        x, y = grid.xy(cell)
        size = self.p['joint_region_size_m']
        return math.floor(x / size), math.floor(y / size)

    def _value(self, view):
        area = min(1., max(0., view.area_m2) / self.p['joint_area_scale_m2'])*self.p['joint_unknown_gain_multiplier']
        quality = view.quality if self.p['joint_strategy'] != 'simple' else 1.
        verification = min(1., max(0., view.priority * quality) / self.p['joint_verify_group_scale'])
        if view.kind=='surface_inspection':
            scale=self.p['joint_missing_geometry_gain_scale'] if view.inspection_source.startswith('missing_') else self.p['joint_surface_gain_scale']
            verification=min(1.,max(0.,view.priority)/scale)
            if self.p['joint_unseen_geometry_enabled']:
                if view.inspection_source=='measured':
                    verification*=self.p['joint_measured_refinement_scale']
                if view.inspection_regions:
                    verification*=float(np.mean([1./(1.+self.inspection_region_visits.get(tuple(r),0))**2
                        for r in view.inspection_regions]))
        if view.kind == 'history_review':
            verification *= self.p['joint_history_review_scale']
        connection = (min(1., view.connectivity_m2 / self.p['joint_connectivity_area_scale_m2']) * quality
                      if self.p['joint_strategy'] != 'simple' else 0.)
        utility=(self.p['joint_weight_area'] * area + self.p['joint_weight_verify'] * verification +
                 self.p['joint_weight_connectivity'] * connection)
        # Zero the entire ghost action, including any incidental area or
        # connectivity reward. Independent frontier/surface views keep theirs.
        utility*=view.feedback_scale
        if view.kind=='verification' and view.group is not None:
            utility*=self.p['joint_active_ghost_gain_multiplier']
        return utility*view.ghost_reward_scale if view.group is not None else utility

    def _score(self, view):
        return self._value(view) / (self.p['joint_score_epsilon_seconds'] +
                                   view.travel_s + view.dwell_s + .35 * view.home_s+view.route_cost_s)

    def _connection(self, grid, distances, cell, target_points, scan_cells, now, deadline):
        """Private optimistic occupancy removal; measured-known mask never grows."""
        if self.clock() >= deadline or not len(target_points):
            return 0.
        signature = (grid.shape, grid.x0, grid.y0,
                     hashlib.sha256(np.packbits(grid.safe).tobytes()).digest())
        cache = self.topology_cache
        if cache is None or cache[0] != signature or now-cache[1] >= self.p['joint_topology_update_seconds']:
            coords = np.argwhere(grid.safe)
            # Exact unique tiles; never sample away an exceeded resource cap.
            xy = np.column_stack((grid.x0+(coords[:, 1]+.5)*grid.resolution,
                                  grid.y0+(coords[:, 0]+.5)*grid.resolution))
            regions = np.unique(np.floor(xy/self.p['joint_region_size_m']).astype(np.int32), axis=0)
            self.topology_cache = (signature, now, len(regions) > self.p['joint_region_limit'])
        # select() resets per-decision diagnostics. Keep the capacity veto in
        # the cache itself so a later cache hit cannot clear the safety limit.
        self.report['region_capacity_limited'] = self.topology_cache[2]
        if self.topology_cache[2]:
            return 0.
        removable = grid.raster(target_points) & grid.occupied & ~scan_cells
        if not removable.any():
            return 0.
        hypothetical = grid.body_known & ~expand(grid.occupied & ~removable, grid.clearance)
        # This mask is NEVER assigned to grid.safe or used to issue a route.
        result = native_search(hypothetical, cell, grid.resolution)
        if result is None or self.clock() >= deadline:
            return 0.
        reached = np.isfinite(result[0])
        return float(np.count_nonzero(reached & grid.safe & ~np.isfinite(distances)) * grid.resolution**2)

    def select(self, grid, position, distances, home_distances, frontier_cells, gains,
               records, points, scan_points, altitude, now, budget, ghost_fresh=True,
               pending_age=20., max_attempts=2, view_change=.7, dwell=4.,
               range_min=1.2, range_max=4.5, voxel_size=.15,
               min_elevation=-22.2, max_elevation=37.2, cleared_group_limit=2,
               inspection_cells=None, inspection_gains=None,inspection_sources=None,inspection_regions=None,
               view_altitude=None,home_xy=None,return_phase=False,
               surface_allowed=True,return_review_remaining_s=None,route_parent=None,indoor_weights=None,
               corridor_cells=None,corridor_gains=None,corridor_wall_mask=None,inspection_claims=None,frontier_claim_cache=None):
        if budget is None or home_distances is None:
            self.report.update(reason='missing_return_budget', candidates=0)
            return None
        began = self.clock()
        # Leave time for a final non-preemptible native search/index step and
        # JSON diagnostics. This is a cooperative budget, not hard real time.
        allowance = self.p['joint_extra_compute_budget_ms']/1000.
        deadline = began + max(.001, allowance-min(.010, allowance*.25))
        old = self.selected
        route_enabled=self.p['joint_route_memory_enabled'] and not return_phase and route_parent is not None
        query=(RouteQuery(grid,route_parent,grid.cell(position),self.route_memory) if route_enabled else None)
        candidates = []
        feedback=self.p['joint_region_feedback_enabled'] and not return_phase
        tail_mode=bool(feedback and self.surface_stopped)
        claim_cache={} if frontier_claim_cache is None else dict(frontier_claim_cache)
        region_rejections=[]
        claim_rows={}
        if inspection_claims is not None and inspection_cells is not None:
            sources=inspection_sources if inspection_sources is not None else ['measured']*len(inspection_cells)
            for cell,source,claims in zip(inspection_cells,sources,inspection_claims):
                claim_rows.setdefault((tuple(cell),source),set()).update(claims)
        def current_frontier(cell):
            if cell not in claim_cache:
                claim_cache[cell]=visible_unknown_gain(grid,[cell],return_claims=True)
            values,claims=claim_cache[cell]
            return float(values[0]),claims[0]
        corridor_cell=None
        corridor_rows=(list(zip(corridor_cells,corridor_gains))
                       if corridor_cells is not None and corridor_gains is not None else [])
        if self.p['joint_corridor_progress_enabled'] and not return_phase:
            cc=np.asarray([r[0] for r in corridor_rows],int).reshape(-1,2) if corridor_rows else frontier_cells
            cg=np.asarray([r[1] for r in corridor_rows]) if corridor_rows else gains
            corridor_cell=self.corridor.choose(grid,position,distances,route_parent,home_distances,
                cc,cg,self.route_memory,budget,now,current=old,indoor_weights=indoor_weights,wall_mask=corridor_wall_mask)
        self.report.update(decisions=self.report.get('decisions', 0)+1,
                           timed_out=False, rejected={}, region_capacity_limited=False,surface_candidates=[],
                           deferred_home_groups=[],return_phase=bool(return_phase),
                           home_ghost_policy=dict(reward_scale=self.p['joint_home_ghost_reward_scale'],
                               radius_m=self.p['joint_home_review_radius_m'],zeroed_records=0,
                               zeroed_samples=[]),
                           surface_low_gain_count=self.surface_low_gain_count,surface_stopped=self.surface_stopped,
                           tail_mode=tail_mode,eligible_frontier_count=0,eligible_surface_count=0,region_feedback=self.region_memory.report(),
                           region_rejections=region_rejections)
        if self.p['joint_corridor_progress_enabled'] and not return_phase and self.corridor.report.get('hold'):
            self.selected=None;self.last_decision=now
            self.report.update(candidates=0,selected=None,top_candidates=[],
                corridor_progress=self.corridor.report,compute_ms=(self.clock()-began)*1000.)
            return None
        def reject(reason):
            r = self.report['rejected']; r[reason] = r.get(reason, 0)+1
        def make(cell, kind, area=0., group=None, episode=None, priority=0., quality=0.,source='measured',regions=()):
            if not grid.inside(cell) or not grid.safe[cell] or not np.isfinite(distances[cell]):
                return None
            view_dwell=(dwell if group else (self.p['joint_surface_dwell_seconds']
                if kind=='surface_inspection' else 1.))
            if return_phase:
                if (return_review_remaining_s is not None and
                        float(budget.travel_seconds(distances[cell]))+view_dwell>return_review_remaining_s):
                    reject('return_review_action_time');return None
                direct=home_distances[grid.cell(position)]
                detour=float(distances[cell]+home_distances[cell]-direct)
                if not math.isfinite(detour) or detour>self.p['joint_return_review_max_detour_m']:
                    reject('return_review_detour');return None
            if not bool(budget.feasible(now, distances[cell], home_distances[cell], view_dwell,
                                       spend_review=return_phase)):
                reject('return_budget'); return None
            view = View(tuple(cell), grid.xy(cell), kind, float(area), group, episode,
                        float(priority), float(quality),
                        travel_s=float(budget.travel_seconds(distances[cell])),
                        home_s=float(budget.travel_seconds(home_distances[cell])),
                        dwell_s=float(view_dwell),inspection_source=source)
            view.inspection_regions=tuple(sorted(set(tuple(r) for r in regions)))
            if feedback and group is None:
                if kind=='frontier':
                    fresh_area,claims=current_frontier(tuple(cell))
                    # Old-target retention must not resurrect stale area claims.
                    if tail_mode:view.area_m2=min(view.area_m2,fresh_area)
                    view.benefit_claims=claims
                else:view.benefit_claims=tuple(claim_rows.get((tuple(cell),source),
                    old.benefit_claims if old is not None and old.xy==view.xy else ()))
                if (self.surface_stopped and kind=='surface_inspection' and view.benefit_claims
                        and not any(c.critical for c in view.benefit_claims)):
                    reject('observed_geometry_refinement_stopped');return None
                scale,states=self.region_memory.assess(view.benefit_claims,now,view.xy)
                view.feedback_scale=scale
                if scale<=0.:
                    reject('region_cooling');region_rejections.append(dict(xy=list(view.xy),reason='region_cooling',states=states[:8]));return None
                direct=home_distances[grid.cell(position)]
                extra=view.travel_s+view.dwell_s+view.home_s-float(budget.travel_seconds(direct))
                if not math.isfinite(extra):reject('missing_fresh_return_cost');return None
                view.extra_return_seconds=max(0.,extra)
                view.extra_area_rate=view.area_m2*scale/max(self.p['joint_tail_min_extra_s'],view.extra_return_seconds)
                if kind=='frontier' and not (corridor_cell is not None and tuple(cell)==tuple(corridor_cell)):
                    if tail_mode and view.area_m2*scale<max(5.,self.p['joint_surface_frontier_gate_m2']) and view.extra_area_rate<self.p['joint_tail_min_area_per_extra_s']:
                        reject('tail_low_incremental_value');region_rejections.append(dict(xy=list(view.xy),
                            reason='tail_low_incremental_value',area=view.area_m2,scale=scale,
                            extra_seconds=view.extra_return_seconds,rate=view.extra_area_rate));return None

            view.corridor_task=bool(corridor_cell is not None and tuple(cell)==tuple(corridor_cell) and kind=='frontier')
            if group is not None and home_xy is not None:
                center=np.mean(groups[group]['points'],axis=0)[:2]
                if math.dist(center,home_xy)<=self.p['joint_home_review_radius_m']:
                    view.ghost_reward_scale=self.p['joint_home_ghost_reward_scale']
            if query is not None:
                repeat,length,direction,_=query.cost(cell)
                view.repeated_path_m=repeat
                view.repeat_cost_s=float(budget.travel_seconds(repeat))*self.p['joint_route_repeat_weight']
                view.reversal_cost_s=(reversal_cost(self.route_memory.heading,direction,
                    self.p['joint_route_reverse_seconds']) if length>=.7 else 0.)
            # Frontier rewards are grouped to avoid double-counting the same
            # local patch in the two-step route. Ghost rewards use episode ID.
            tokens = [('area', self._region(grid, cell))] if area > 0 else []
            if episode is not None:
                tokens.append(('verification', episode))
            view.rewards = frozenset(tokens)
            view.score = self._score(view)
            return view
        ordered = sorted(zip(frontier_cells, gains),
                         key=lambda pair: -float(pair[1])/(2.+float(distances[tuple(pair[0])])))
        frontier_rows=self.diverse(ordered,lambda row:self.route_region(grid.xy(tuple(row[0]))),
                                   self.p['joint_frontier_candidate_limit'])
        if corridor_cell is not None:
            preferred=next((row for row in corridor_rows+ordered if tuple(row[0])==tuple(corridor_cell)),None)
            if preferred is not None:
                frontier_rows=[preferred]+[r for r in frontier_rows if tuple(r[0])!=tuple(corridor_cell)]
                frontier_rows=frontier_rows[:self.p['joint_frontier_candidate_limit']]
        for cell, gain in ([] if return_phase else frontier_rows):
            item = make(tuple(cell), 'frontier', gain)
            if item is not None:
                candidates.append(item)
        surface_cells={}
        elapsed=budget.elapsed(now)
        surface_remaining=self.p['joint_surface_action_budget_seconds']-self.used_surface_seconds
        if (surface_allowed and not return_phase and (not self.surface_stopped or feedback) and self.p['joint_surface_inspection_enabled'] and elapsed is not None and
                elapsed>=self.p['joint_surface_start_seconds'] and surface_remaining>0 and
                inspection_cells is not None and inspection_gains is not None):
            sources=(['measured']*len(inspection_cells) if inspection_sources is None else inspection_sources)
            if len(sources)!=len(inspection_cells):raise ValueError('Inspection source count differs')
            regions=([()]*len(inspection_cells) if inspection_regions is None else inspection_regions)
            if len(regions)!=len(inspection_cells):raise ValueError('Inspection region count differs')
            def inspection_rank(row):
                cell,gain,source,physical_regions=row;cell=tuple(cell)
                if not self.p['joint_unseen_geometry_enabled']:
                    return -float(gain)/(2.+float(distances[cell]))
                view=View(cell,grid.xy(cell),'surface_inspection',priority=float(gain),quality=1.,
                    inspection_source=source,inspection_regions=tuple(physical_regions))
                cost=(self.p['joint_score_epsilon_seconds']+self.p['joint_surface_dwell_seconds']+
                    float(budget.travel_seconds(distances[cell]))+
                    .35*float(budget.travel_seconds(home_distances[cell])))
                return -self._value(view)/cost
            # Apply physical-region debt BEFORE the six-slot truncation.
            # Scoring only retained views would still starve a distant patch
            # behind many cheap views of the same already-visited region.
            ranked=sorted(zip(inspection_cells,inspection_gains,sources,regions),key=inspection_rank)
            # Reserve opportunities for BOTH never-returned floor and back
            # boundary requests. A dense already-seen front cannot consume
            # all six slots before the rear requirements are even evaluated.
            buckets={name:[] for name in ('missing_sweep','missing_floor','missing_boundary','measured')}
            for row in ranked:buckets.get(str(row[2]),buckets['measured']).append(row)
            ordered=[]
            for i in range(min(32,max([len(v) for v in buckets.values()]+[0]))):
                for source in buckets:
                    if i<len(buckets[source]):ordered.append(buckets[source][i])
            retained=0;seen=set()
            for cell,gain,source,regions in ordered:
                cell=tuple(cell)
                if cell in seen:continue
                if self.surface_stopped and not str(source).startswith('missing_'):continue
                item=make(cell,'surface_inspection',priority=float(gain),quality=1.,source=str(source),regions=regions)
                if item is not None and item.travel_s+item.dwell_s<=surface_remaining:
                    item.rewards=(frozenset(('surface',*r) for r in item.inspection_regions)
                        if item.inspection_regions else frozenset([('surface',self._region(grid,cell))]))
                    candidates.append(item);surface_cells[cell]=float(gain)
                    seen.add(cell);retained+=1
                    if retained>=self.p['joint_surface_candidate_limit']:break
            self.report['surface_candidates']=[v.summary() for v in candidates if v.kind=='surface_inspection']
        groups = {}
        if ghost_fresh and self.used_ghost_seconds < self.p['joint_ghost_action_budget_seconds']:
            eligible=[];positions=[]
            for record in records:
                xyz = record.get('position', [])
                if len(xyz) != 3 or not np.isfinite(xyz).all():
                    continue
                active = (record.get('state') == 'suspected' and record.get('actionable') is True and
                          now-float(record.get('last_dynamic', now)) >= pending_age)
                historical = (record.get('state') == 'cleared' and record.get('occupied_now') is False and
                              record.get('cleared_episodes', 0) > 0)
                if not active and not historical:
                    continue
                if (home_xy is not None and self.p['joint_home_ghost_reward_scale']==0. and
                        math.dist(xyz[:2],home_xy)<=self.p['joint_home_review_radius_m']):
                    # Filter per voxel BEFORE grouping, even if a group straddles
                    # the radius. No ledger/occupancy/evidence is modified.
                    hp=self.report['home_ghost_policy'];hp['zeroed_records']+=1
                    if len(hp['zeroed_samples'])<16:
                        hp['zeroed_samples'].append(dict(position=[float(v) for v in xyz],score=0.))
                    reject('home_ghost_score_zero');continue
                if (return_phase and home_xy is not None and math.dist(xyz[:2],home_xy)>
                        self.p['joint_home_review_radius_m']+1.2):continue
                eligible.append((record,active));positions.append(xyz)
            # navigation_z reads a ROS parameter; calling it for every one of
            # hundreds of records exhausted the 50ms budget before any ray
            # could be tested. Evaluate the height field in ONE vector call.
            positions=np.asarray(positions,dtype=float).reshape(-1,3)
            heights=(np.full(len(positions),altitude) if view_altitude is None or not len(positions)
                else np.broadcast_to(view_altitude(positions[:,:2]),(len(positions),)))
            for (record,active),xyz,local_altitude in zip(eligible,positions,heights):
                if not local_altitude-1.25 <= xyz[2] <= local_altitude+.85:continue
                key = tuple(np.floor(np.asarray(xyz[:2])/.8).astype(int))
                entry = groups.setdefault(key, dict(points=[], priority=0., episode=0, active=False,
                                                     boundary=True))
                entry['points'].append(xyz)
                entry['active'] |= active
                entry['boundary'] &= bool(record.get('static_boundary_candidate', False))
                entry['episode'] = max(entry['episode'], int(record.get('suspected_episodes', 1)))
                entry['priority'] += (max(0., float(record.get('priority', 1.))) if active else
                                      (1.+.25*min(3, record.get('cleared_episodes', 1)-1)))
        # Historical review has bounded groups and a post-normalization scale;
        # dense old point counts must not saturate away its lower priority.
        historical = sorted((k for k in groups if not groups[k]['active']),
                            key=lambda k: -groups[k]['priority'])
        for key in historical[cleared_group_limit:]:
            del groups[key]
        if old is not None:
            valid_ghost = False
            if old.group in groups:
                g = groups[old.group]
                current_episode = (*old.group, g['episode'], 'active' if g['active'] else 'historical')
                valid_ghost = (current_episode == old.episode and
                    self.used_ghost_seconds < self.p['joint_ghost_action_budget_seconds'])
            rotating_missing=(self.p['joint_unseen_geometry_enabled'] and
                old.kind=='surface_inspection' and old.inspection_source.startswith('missing_'))
            valid_surface=(old.kind!='surface_inspection' or old.cell in surface_cells or rotating_missing)
            if self.surface_stopped and old.kind=='surface_inspection' and not (feedback and old.inspection_source.startswith('missing_')):valid_surface=False
            if not surface_allowed and old.kind=='surface_inspection':valid_surface=False
            if return_phase and old.group is None:valid_surface=False
            if (old.group is None or valid_ghost) and valid_surface:
                item = make(grid.cell(old.xy), old.kind, old.area_m2, old.group,
                            old.episode, old.priority, old.quality,source=old.inspection_source,regions=old.inspection_regions)
                ghost_affordable=(old.group is None or (item is not None and item.travel_s+item.dwell_s<=
                    self.p['joint_ghost_action_budget_seconds']-self.used_ghost_seconds))
                surface_affordable=(old.kind!='surface_inspection' or (item is not None and
                    item.travel_s+item.dwell_s<=surface_remaining))
                if item is not None and ghost_affordable and surface_affordable:
                    item.connectivity_m2=old.connectivity_m2;item.score=self._score(item)
                    candidates.append(item)
        ranked = sorted(groups.items(), key=lambda pair: (not pair[1]['active'],
                        -pair[1]['priority'] / (2.+math.dist(position, np.mean(pair[1]['points'], axis=0)[:2]))))
        occupied = None
        scan_cells = None
        for group, info in ranked[:self.p['joint_ghost_group_limit']]:
            if self.clock() >= deadline:
                break
            episode = (*group, info['episode'], 'active' if info['active'] else 'historical')
            stats = self.stats.get(episode, dict(views=[], cooldown=-float('inf'), measured=0, effective=0))
            if len(stats['views']) >= (max_attempts if info['active'] else 1) or stats['cooldown'] > now:
                reject('attempts_or_cooldown'); continue
            targets = np.asarray(info['points'], dtype=float)
            # Bounded proxy targets, not a claim all voxels have ray support.
            targets = targets[::max(1, int(math.ceil(len(targets)/16.)))]
            center = targets[:, :2].mean(axis=0)
            near_home=(home_xy is not None and
                math.dist(center,home_xy)<=self.p['joint_home_review_radius_m'])
            if return_phase and not near_home:continue
            if (not return_phase and self.p['joint_defer_home_review'] and near_home and
                    (not info['active'] or info['boundary'])):
                # These groups cannot have the connectivity exception. Do not
                # spend the whole ray budget evaluating views we will defer.
                self.report['deferred_home_groups'].append([int(v) for v in group])
                reject('deferred_until_return');continue
            occupied = None  # index just this group's rays, built lazily
            views = []
            for radius in self.p['joint_view_ranges_m']:
                for angle in np.arange(self.p['joint_candidate_azimuth_samples'])*2*math.pi/self.p['joint_candidate_azimuth_samples']:
                    xy = center+radius*np.array([math.cos(angle), math.sin(angle)])
                    cell = grid.cell(xy)
                    if not grid.inside(cell) or not grid.safe[cell] or not np.isfinite(distances[cell]):
                        continue
                    xy = grid.xy(cell)
                    actual_range = math.dist(xy, center)
                    if not range_min <= actual_range <= range_max:
                        continue
                    if math.dist(xy, position) < view_change or any(math.dist(xy, old_xy) < view_change for old_xy in stats['views']):
                        continue
                    item = make(cell, 'verification' if info['active'] else 'history_review',
                                group=group, episode=episode, priority=info['priority'])
                    if item is not None and item.travel_s+item.dwell_s <= self.p['joint_ghost_action_budget_seconds']-self.used_ghost_seconds:
                        views.append(item)
            views.sort(key=lambda v: v.travel_s+.35*v.home_s)
            retained = 0
            used_cells = set()
            for view in views:
                if self.clock() >= deadline:
                    break
                if view.cell in used_cells:
                    continue
                used_cells.add(view.cell)
                if not observed_los(grid, view.xy, center):
                    reject('unknown_or_blocked_los'); continue
                if occupied is None:
                    # Restrict the 3-D index to targets' vicinity. This can only
                    # discard points outside every tested ray's XY bounding box.
                    pts = np.asarray(points)
                    local = pts[np.all(np.abs(pts[:, :2]-center) <= range_max+.3, axis=1)]
                    occupied = set(map(tuple, np.floor(local/voxel_size).astype(np.int32).tolist()))
                    if scan_cells is None:
                        scan_cells = grid.raster(scan_points)
                    if self.clock() >= deadline:
                        break
                view_z=altitude if view_altitude is None else float(view_altitude(view.xy))
                visible = visible_target_count((view.xy[0], view.xy[1], view_z), targets, occupied,
                                                voxel_size, min_elevation, max_elevation)
                if not visible:
                    reject('occluded_or_fov'); continue
                distance = math.dist(view.xy, center)
                range_factor = 1. if distance <= 2.5 else max(.5, 1.-.25*(distance-2.5))
                outcome = max(.25, (stats['effective']+1.)/(stats['measured']+2.))
                view.quality = float(visible)/len(targets)*range_factor*outcome
                if self.p['joint_strategy'] != 'simple' and view.quality < self.p['joint_min_evidence_quality']:
                    reject('low_evidence_quality'); continue
                if info['active'] and not info['boundary'] and self.p['joint_strategy'] != 'simple':
                    view.connectivity_m2 = self._connection(grid, distances, view.cell, targets, scan_cells, now, deadline)
                if (not return_phase and self.p['joint_defer_home_review'] and near_home and
                        view.connectivity_m2<=0.):
                    label=[int(v) for v in group]
                    if label not in self.report['deferred_home_groups']:
                        self.report['deferred_home_groups'].append(label)
                    # Keep every voxel and episode unchanged; only its optional
                    # observation is deferred. Live obstacles are still avoided.
                    reject('deferred_until_return');continue
                # A view that is also a nearby frontier receives its original
                # area reward once. No measured evidence is synthesized.
                for frontier in candidates:
                    if frontier.kind == 'frontier' and math.dist(view.xy, frontier.xy) <= grid.resolution:
                        view.area_m2 = max(view.area_m2, frontier.area_m2)
                if view.area_m2 > 0:
                    view.rewards |= frozenset([('area', self._region(grid, view.cell))])
                view.score = self._score(view)
                candidates.append(view)
                retained += 1
                if retained >= self.p['joint_retained_views_per_group']:
                    break
        unique = {}
        for candidate in candidates:
            key = (candidate.cell, candidate.kind, candidate.episode)
            if key not in unique or candidate.score > unique[key].score:
                unique[key] = candidate
        candidates = [v for v in unique.values() if v.group is None or v.ghost_reward_scale>0.]
        active_available=any(self.route_region(c.xy)==self.active_route_region for c in candidates)
        for candidate in candidates:
            if route_enabled and active_available and self.route_region(candidate.xy)!=self.active_route_region:
                candidate.region_switch_cost_s=self.p['joint_route_region_switch_seconds']
            candidate.score=self._score(candidate)
        # A five-second round-robin generation pass may omit the committed
        # missing-face/floor request while the vehicle is still traveling.
        # It is not evidence the sensing requirement disappeared. Its rebuilt
        # View above still needs current safety, reachability, flight and
        # surface-action budget; arrival / timeout is handled by Explorer.
        committed_missing=next((v for v in candidates if old is not None and
            self.p['joint_unseen_geometry_enabled'] and old.kind=='surface_inspection' and
            old.inspection_source.startswith('missing_') and v.cell==old.cell and
            v.kind==old.kind),None)
        candidates.sort(key=lambda v: (-v.score, v.xy, v.kind))
        corridor_retained=next((v for v in candidates if v.corridor_task),None)
        candidates = candidates[:self.p['joint_total_candidate_limit']]
        if corridor_retained is not None and not any(v.corridor_task for v in candidates):
            candidates=candidates[:-1]+[corridor_retained]
        best = candidates[0] if candidates else None
        sequence_score = best.score if best else 0.
        lookahead_searches=lookahead_pairs=0
        if self.p['joint_lookahead_depth'] == 2:
            beam = self.diverse(candidates,lambda v:self.route_region(v.xy),self.p['joint_lookahead_branching'])
            for first in beam:
                if self.clock() >= deadline:
                    break
                result = native_search(grid.safe, first.cell, grid.resolution)
                lookahead_searches+=1
                if result is None:
                    break
                legs = result[0]
                second_query=RouteQuery(grid,result[1],first.cell,self.route_memory) if route_enabled else None
                for second in beam:
                    if first is second or first.rewards & second.rewards or not np.isfinite(legs[second.cell]):
                        continue
                    outbound = float(distances[first.cell]+legs[second.cell])
                    if not bool(budget.feasible(now, outbound, home_distances[second.cell], first.dwell_s+second.dwell_s,
                                               spend_review=return_phase)):
                        continue
                    if first.group is not None and first.episode == second.episode:
                        continue
                    total = (self.p['joint_score_epsilon_seconds']+float(budget.travel_seconds(outbound))+
                             first.dwell_s+second.dwell_s+.35*second.home_s+first.route_cost_s)
                    if second_query is not None:
                        repeat,_,direction,_=second_query.cost(second.cell)
                        total+=float(budget.travel_seconds(repeat))*self.p['joint_route_repeat_weight']
                        _,_,_,incoming=query.cost(first.cell)
                        total+=reversal_cost(incoming,direction,self.p['joint_route_reverse_seconds'])
                        if self.route_region(first.xy)!=self.route_region(second.xy):
                            total+=self.p['joint_route_region_switch_seconds']
                    lookahead_pairs+=1
                    score = (self._value(first)+self.p['joint_lookahead_discount']*self._value(second))/total
                    if score > sequence_score:
                        sequence_score, best = score, first
        corridor_choice=next((v for v in candidates if v.corridor_task),None)
        if committed_missing is not None:best=committed_missing
        if corridor_choice is not None:best=corridor_choice
        old_valid = next((v for v in candidates if old is not None and v.xy == old.xy and v.kind == old.kind), None)
        if corridor_choice is None and old_valid is not None and best is not None and (
                now-self.last_selection_stamp < self.p['joint_target_min_hold_seconds'] or
                best.score <= old_valid.score*(1.+self.p['joint_switch_improvement_ratio'])):
            best = old_valid
        if best is not None and (old is None or old.xy != best.xy or old.kind != best.kind):
            self.last_selection_stamp = now
        self.selected = best
        if route_enabled and best is not None:self.active_route_region=self.route_region(best.xy)
        self.last_decision = now
        self.report.update(eligible_frontier_count=sum(v.kind=='frontier' for v in candidates),
                           active_ghost_gain_multiplier=self.p['joint_active_ghost_gain_multiplier'],
                           eligible_surface_count=sum(v.kind=='surface_inspection' for v in candidates),
                           region_feedback=self.region_memory.report(),
                           candidates=len(candidates), timed_out=self.clock() >= deadline,
                           compute_ms=(self.clock()-began)*1000.,
                           nominal_budget_ms=self.p['joint_extra_compute_budget_ms'],
                           cooperative_work_budget_ms=1000.*(deadline-began),
                           used_ghost_seconds=self.used_ghost_seconds,
                           used_surface_seconds=self.used_surface_seconds,
                           retained_missing_target=committed_missing is not None,
                           completed_inspection_regions=len(self.inspection_region_visits),
                           selected=best.summary() if best else None,
                           top_candidates=[c.summary() for c in candidates[:6]],
                           route_preferences=dict(enabled=route_enabled,**self.route_memory.report(),
                               active_region=list(self.active_route_region) if self.active_route_region is not None else None,
                               path_cache_limited=bool(query and query.capacity_limited)),
                           lookahead_searches=lookahead_searches,lookahead_pairs=lookahead_pairs,
                           corridor_progress=(self.corridor.report if self.p['joint_corridor_progress_enabled'] and not return_phase
                               else dict(active=False,reason='return_phase' if return_phase else 'disabled')),
                           scope='Heuristic bounded two-view ranking; hypothetical clearance never executed')
        return best
