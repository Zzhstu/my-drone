#!/usr/bin/env python3
"""Decision regressions with measured-map fixtures; no ROS or flight required."""
import json
import sys
import time
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from flight_budget import FlightBudget
from joint_view_policy import JointViewPolicy, View


class JointPolicyTest(unittest.TestCase):
    def test_region_debt_is_applied_before_candidate_quota(self):
        grid=self.grid();d,_=grid.search(grid.cell((2.,2.)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        p=self.policy(joint_surface_inspection_enabled=True,joint_unseen_geometry_enabled=True,
            joint_surface_action_budget_seconds=120.,joint_lookahead_depth=1)
        old=View((0,0),(0,0),'surface_inspection',inspection_source='missing_boundary',
            inspection_regions=(('boundary',1,1),))
        for _ in range(3):p.complete_inspection(old)
        cells=np.asarray([grid.cell((3.+i*.3,2.)) for i in range(10)]+[grid.cell((8.,8.))])
        chosen=p.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
            np.empty((0,3)),np.empty((0,3)),1.2,200.,b,
            inspection_cells=cells,inspection_gains=np.full(len(cells),16.),
            inspection_sources=['missing_boundary']*len(cells),
            inspection_regions=[(('boundary',1,1),)]*10+[(('boundary',4,4),)])
        self.assertEqual(chosen.cell,grid.cell((8.,8.)))
        self.assertIn(list(chosen.xy),[r['xy'] for r in p.report['surface_candidates']])

    def test_different_viewpoints_of_same_patch_do_not_starve_unvisited_region(self):
        grid=self.grid();d,_=grid.search(grid.cell((2.,2.)))
        budget=FlightBudget();budget.observe_arm(False,1.);budget.observe_arm(True,2.)
        policy=self.policy(joint_surface_inspection_enabled=True,joint_unseen_geometry_enabled=True,
            joint_surface_action_budget_seconds=120.,joint_lookahead_depth=1)
        def choose(x):
            return policy.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
                np.empty((0,3)),np.empty((0,3)),1.2,200.,budget,
                inspection_cells=np.asarray([grid.cell((x,2.)),grid.cell((8.,2.))]),
                inspection_gains=np.asarray([16.,16.]),inspection_sources=['missing_boundary']*2,
                inspection_regions=[(('boundary',1,1),),(('boundary',4,1),)])
        first=choose(3.)
        self.assertEqual(first.cell,grid.cell((3.,2.)))
        self.assertEqual(policy.inspection_region_visits,{})
        policy.complete_inspection(first);policy.selected=None
        # The close view moved to a different cell but sees the SAME patch.
        second=choose(4.)
        self.assertEqual(second.cell,grid.cell((8.,2.)))
        self.assertEqual(second.inspection_regions,(('boundary',4,1),))

    def test_coordinate_revision_resets_spatial_debt_but_preserves_action_budget(self):
        policy=self.policy();policy.inspection_region_visits[('boundary',2,3)]=2
        policy.used_surface_seconds=17.
        policy.invalidate_coordinates()
        self.assertEqual(policy.inspection_region_visits,{})
        self.assertEqual(policy.used_surface_seconds,17.)

    def surface_selection(self,policy,now=200.,cells=None):
        grid=self.grid();d,_=grid.search(grid.cell((2.,2.)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        cells=np.asarray([grid.cell((3.,2.))] if cells is None else cells,dtype=int).reshape(-1,2)
        return policy.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
            np.empty((0,3)),np.empty((0,3)),1.2,now,b,
            inspection_cells=cells,inspection_gains=np.full(len(cells),80.))

    def test_surface_inspection_can_run_without_unknown_frontier_or_ghost(self):
        p=self.policy(joint_surface_inspection_enabled=True)
        v=self.surface_selection(p)
        self.assertEqual(v.kind,'surface_inspection')
        self.assertGreater(v.score,0)

    def test_surface_inspection_default_off_preserves_original_policy(self):
        self.assertIsNone(self.surface_selection(self.policy()))

    def test_surface_inspection_is_bounded_by_separate_action_budget(self):
        p=self.policy(joint_surface_inspection_enabled=True)
        p.used_surface_seconds=60.
        self.assertIsNone(self.surface_selection(p))

    def test_surface_inspection_does_not_preempt_initial_exploration(self):
        self.assertIsNone(self.surface_selection(self.policy(joint_surface_inspection_enabled=True),now=100.))

    def test_satisfied_surface_cancels_retained_target(self):
        p=self.policy(joint_surface_inspection_enabled=True)
        self.assertIsNotNone(self.surface_selection(p))
        self.assertIsNone(self.surface_selection(p,now=201.,cells=[]))

    def test_missing_floor_and_back_keep_slots_among_dense_nearby_measured_faces(self):
        grid=self.grid();d,_=grid.search(grid.cell((2.,2.)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        p=self.policy(joint_surface_inspection_enabled=True,joint_unseen_geometry_enabled=True)
        near=[grid.cell((3.+i*.4,2.)) for i in range(10)]
        other=[grid.cell((8.,7.)),grid.cell((8.,8.)),grid.cell((9.,7.)),grid.cell((9.,8.))]
        cells=np.asarray(near+other);gains=np.asarray([1000.]*10+[16.]*4)
        sources=['measured']*10+['missing_floor']*2+['missing_boundary']*2
        p.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
            np.empty((0,3)),np.empty((0,3)),1.2,200.,b,
            inspection_cells=cells,inspection_gains=gains,inspection_sources=sources)
        retained=p.report['surface_candidates']
        self.assertEqual(len(retained),6)
        self.assertEqual(sorted(r['inspection_source'] for r in retained),
            ['measured']*2+['missing_boundary']*2+['missing_floor']*2)
        self.assertTrue(all(grid.safe[grid.cell(r['xy'])] for r in retained))

    def test_missing_target_survives_rotations_during_a_long_travel_leg(self):
        grid=self.grid();d,_=grid.search(grid.cell((2.,2.)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        p=self.policy(joint_surface_inspection_enabled=True,joint_unseen_geometry_enabled=True,
                      joint_surface_action_budget_seconds=120.)
        target=grid.cell((8.,8.));cells=np.asarray([target])
        def choose(now,rotated):
            cc=np.asarray([grid.cell((3.,2.))]) if rotated else cells
            return p.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
                np.empty((0,3)),np.empty((0,3)),1.2,now,b,
                inspection_cells=cc,inspection_gains=np.asarray([16.]),
                inspection_sources=['missing_floor'])
        first=choose(200.,False)
        self.assertGreater(first.travel_s,5.)
        for stamp in (205.,210.,215.):
            chosen=choose(stamp,True)
            self.assertEqual(chosen.cell,target)
            self.assertTrue(p.report['retained_missing_target'])
        # A rotation cannot cancel the leg, but actual budget invalidation can.
        p.used_surface_seconds=120.
        self.assertIsNone(choose(216.,True))

    def test_committed_missing_target_is_cancelled_on_actual_safety_loss(self):
        grid=self.grid();d,_=grid.search(grid.cell((2.,2.)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        p=self.policy(joint_surface_inspection_enabled=True,joint_unseen_geometry_enabled=True,
                      joint_surface_action_budget_seconds=120.)
        target=grid.cell((8.,8.))
        first=p.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
            np.empty((0,3)),np.empty((0,3)),1.2,200.,b,
            inspection_cells=np.asarray([target]),inspection_gains=np.asarray([16.]),
            inspection_sources=['missing_boundary'])
        self.assertEqual(first.cell,target)
        grid.safe[target]=False
        chosen=p.select(grid,(2.,2.),d,d,np.empty((0,2),int),np.empty(0),[],
            np.empty((0,3)),np.empty((0,3)),1.2,205.,b,
            inspection_cells=np.empty((0,2),int),inspection_gains=np.empty(0),inspection_sources=[])
        self.assertIsNone(chosen)

    def grid(self):
        grid=ExplorationGrid([0,0,12,12],.2,.8,.4)
        grid.known[:]=True;grid.update_obstacles([])
        return grid

    def policy(self, **overrides):
        overrides.setdefault('joint_extra_compute_budget_ms',1000.)
        return JointViewPolicy(lambda k,d: overrides.get(k,d))

    def select(self, policy, grid=None, records=None, frontiers=True, now=100., budget=None):
        grid=grid or self.grid();position=(2.,2.)
        distances,_=grid.search(grid.cell(position));home,_=grid.search(grid.cell(position))
        if budget is None:
            budget=FlightBudget();budget.observe_arm(False,1.);budget.observe_arm(True,2.)
        cells=np.asarray([grid.cell((3.,2.))],dtype=int) if frontiers else np.empty((0,2),int)
        gains=np.array([5.]) if frontiers else np.empty(0)
        if records is None:
            records=[dict(position=[6.,6.,1.2],state='suspected',actionable=True,
                          priority=10.,suspected_episodes=1,last_dynamic=1.)]
        points=np.asarray([r['position'] for r in records],dtype=float).reshape((-1,3))
        return policy.select(grid,position,distances,home,cells,gains,records,points,
                             np.empty((0,3)),1.2,now,budget)

    def test_original_measured_grid_is_never_modified(self):
        grid=self.grid();grid.update_obstacles(np.array([[6.,6.,1.2]]))
        before=[getattr(grid,k).copy() for k in ['known','occupied','safe','body_known']]
        result=self.select(self.policy(),grid,frontiers=False)
        self.assertIsNotNone(result)
        self.assertTrue(grid.safe[result.cell])
        for name,old in zip(['known','occupied','safe','body_known'],before):
            np.testing.assert_array_equal(getattr(grid,name),old)

    def test_json_diagnostics_accept_numpy_voxel_group_keys(self):
        policy=self.policy();self.select(policy,frontiers=False)
        json.dumps(policy.report,allow_nan=False)

    def test_expired_budget_rejects_every_candidate(self):
        self.assertIsNone(self.select(self.policy(),now=590.))

    def test_unverified_arm_clock_rejects_every_candidate(self):
        self.assertIsNone(self.select(self.policy(),budget=FlightBudget()))

    def test_stale_or_resolved_history_cancels_existing_ghost_target(self):
        policy=self.policy();self.assertIsNotNone(self.select(policy,frontiers=False))
        self.assertIsNone(self.select(policy,records=[],frontiers=False,now=101.))

    def test_total_ghost_action_budget_cancels_retained_target(self):
        policy=self.policy();self.select(policy,frontiers=False)
        policy.charge(90.)
        self.assertIsNone(self.select(policy,frontiers=False,now=101.))

    def test_missing_scan_evidence_is_not_a_success_and_cools_down(self):
        policy=self.policy();view=self.select(policy,frontiers=False)
        policy.complete_view(view,101.,True,measured=False)
        state=policy.stats[view.episode]
        self.assertEqual(state['effective'],0)
        self.assertEqual(state['measured'],0)
        # Even a caller claiming effective=True cannot certify absent data.
        self.assertIsNone(self.select(policy,frontiers=False,now=102.))

    def test_two_view_attempt_cap_is_not_reset_by_cooldown(self):
        policy=self.policy()
        first=self.select(policy,frontiers=False)
        policy.complete_view(first,101.,False)
        second=self.select(policy,frontiers=False,now=140.)
        self.assertIsNotNone(second)
        self.assertGreaterEqual(np.linalg.norm(np.array(first.xy)-second.xy),.7)
        policy.complete_view(second,141.,False)
        self.assertIsNone(self.select(policy,frontiers=False,now=180.))

    def test_coordinate_invalidation_preserves_budget_and_attempt_memory(self):
        policy=self.policy();view=self.select(policy,frontiers=False)
        policy.complete_view(view,101.,False);policy.charge(7.)
        policy.invalidate_coordinates()
        self.assertEqual(policy.used_ghost_seconds,7.)
        self.assertEqual(len(policy.stats[view.episode]['views']),1)
        self.assertIsNone(policy.selected)

    def test_unknown_los_does_not_generate_ghost_view(self):
        grid=self.grid();grid.known[:]=False
        grid.known[:20,:20]=True;grid.update_obstacles([[6.,6.,1.2]])
        self.assertIsNone(self.select(self.policy(),grid,frontiers=False))

    def test_retained_target_is_immediately_rejected_when_unsafe(self):
        policy=self.policy();grid=self.grid()
        first=self.select(policy,grid,records=[])
        grid.update_obstacles([[*first.xy,1.2]])
        result=self.select(policy,grid,records=[],frontiers=False,now=100.1)
        self.assertIsNone(result)

    def test_connection_value_uses_private_safe_search_and_current_scan_veto(self):
        grid=self.grid();target=np.array([[6.,6.,1.2]])
        wall=np.array([[6.,float(y),1.2] for y in np.arange(0,12,.1) if y<4.5 or y>7.5])
        grid.update_obstacles(np.vstack((wall,target)))
        start=grid.cell((2.,6.));distances,_=grid.search(start)
        self.assertFalse(np.isfinite(distances[grid.cell((10.,6.))]))
        policy=self.policy();old=grid.safe.copy()
        gain=policy._connection(grid,distances,start,target,grid.raster(wall),100.,time.monotonic()+2.)
        self.assertGreater(gain,1.)
        vetoed=policy._connection(grid,distances,start,target,grid.raster(target),100.,time.monotonic()+2.)
        self.assertEqual(vetoed,0.)
        np.testing.assert_array_equal(grid.safe,old)

    def test_capacity_veto_survives_cached_decision_diagnostic_reset(self):
        grid=self.grid();target=np.array([[6.,6.,1.2]])
        wall=np.array([[6.,float(y),1.2] for y in np.arange(0,12,.1) if y<4.5 or y>7.5])
        grid.update_obstacles(np.vstack((wall,target)))
        start=grid.cell((2.,6.));distances,_=grid.search(start)
        policy=self.policy(joint_region_limit=1)
        for stamp in (100.,101.):
            policy.report['region_capacity_limited']=False
            gain=policy._connection(grid,distances,start,target,grid.raster(wall),stamp,time.monotonic()+2.)
            self.assertEqual(gain,0.)
            self.assertTrue(policy.report['region_capacity_limited'])

    def test_connection_proxy_cannot_create_unknown_free_space(self):
        grid=self.grid();grid.known[:,30:]=False
        target=np.array([[6.,6.,1.2]]);grid.update_obstacles(target)
        start=grid.cell((2.,6.));distances,_=grid.search(start)
        policy=self.policy()
        gain=policy._connection(grid,distances,start,target,np.zeros(grid.shape,bool),100.,time.monotonic()+2.)
        self.assertEqual(gain,0.)

    def test_history_review_scale_is_not_saturated_by_point_density(self):
        policy=self.policy()
        active=View((1,1),(1.,1.),'verification',priority=100.,quality=1.)
        history=View((1,1),(1.,1.),'history_review',priority=100.,quality=1.)
        self.assertAlmostEqual(policy._value(history),policy._value(active)*.25)

    def test_invalid_weights_and_disabled_safety_gate_fail_at_start(self):
        with self.assertRaises(ValueError):self.policy(joint_weight_area=.9)
        with self.assertRaises(ValueError):self.policy(joint_replan_on_safety_invalidation=False)
        with self.assertRaises(ValueError):self.policy(joint_view_ranges_m=[float('nan')])


if __name__=='__main__':unittest.main()
