#!/usr/bin/env python3
"""Counterexamples for redundant sweeps, return deferral and landing setup."""
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from viewpoint_sweep import geometry_bound_sweep_gain
from joint_view_policy import JointViewPolicy
from joint_view_policy import View
from flight_budget import FlightBudget
from endurance_landing import EnduranceLanding


class ReturnObservationTest(unittest.TestCase):
    def test_unvisited_wall_space_cannot_replace_floor_back_requests(self):
        mask=np.zeros((8,8),bool);mask[2,2]=mask[3,3]=mask[4,4]=True
        floor=np.zeros_like(mask);floor[2,2]=True
        back=np.zeros_like(mask);back[3,3]=True
        measured=np.zeros_like(mask);measured[4,4]=True
        sweep=np.ones_like(mask);mass=mask.astype(float)
        original=mass.copy();original_mask=mask.copy()
        result,count=geometry_bound_sweep_gain(sweep,np.ones(mask.shape),
            {'missing_floor':floor,'missing_boundary':back,'measured':measured},mask,mass)
        self.assertEqual(count,2);self.assertEqual(result[2,2],17)
        self.assertEqual(result[3,3],17);self.assertEqual(result[4,4],1)
        self.assertEqual(result[7,7],0)
        np.testing.assert_array_equal(mass,original);np.testing.assert_array_equal(mask,original_mask)

    def test_low_geometric_gain_stops_surface_work_without_faking_success(self):
        p=JointViewPolicy(lambda k,d:True if k=='joint_stop_low_gain_surface' else d)
        p.observe_surface_gain(2);self.assertFalse(p.surface_stopped)
        p.observe_surface_gain(8);self.assertEqual(p.surface_low_gain_count,0)
        p.observe_surface_gain(7);p.observe_surface_gain(0)
        self.assertTrue(p.surface_stopped);self.assertEqual(p.inspection_region_visits,{})
        p.invalidate_coordinates();self.assertFalse(p.surface_stopped)

    def test_reserved_review_is_spendable_only_during_return(self):
        b=FlightBudget(review=15.,speed=.5);b.observe_arm(False,1);b.observe_arm(True,2)
        # 67s remain: base return/dwell 45s + 13s travel +4s view fit,
        # while keeping an additional 15s review reserve does not fit.
        self.assertFalse(b.feasible(474.,3.,2.,4.))
        self.assertTrue(b.feasible(474.,3.,2.,4.,spend_review=True))
        self.assertFalse(b.feasible(530.,3.,2.,4.,spend_review=True))
        with self.assertRaises(ValueError):FlightBudget(review=-1.)

    def selection(self,p,records,return_phase=False,now=100.,**kwargs):
        g=ExplorationGrid([0,0,14,14],.2,.8,.4);g.known[:]=True;g.update_obstacles([])
        position=(9.,2.);home=(2.,2.)
        d,_=g.search(g.cell(position));h,_=g.search(g.cell(home))
        b=FlightBudget(review=15.);b.observe_arm(False,1.);b.observe_arm(True,2.)
        before=[g.safe.copy(),g.known.copy(),g.occupied.copy()]
        result=p.select(g,position,d,h,np.empty((0,2),int),np.empty(0),records,
            np.asarray([r['position'] for r in records]).reshape(-1,3),np.empty((0,3)),
            1.2,now,b,home_xy=home,return_phase=return_phase,**kwargs)
        for old,actual in zip(before,[g.safe,g.known,g.occupied]):np.testing.assert_array_equal(old,actual)
        return result

    def policy(self,**overrides):
        cfg=dict(joint_defer_home_review=True,joint_extra_compute_budget_ms=1000.,joint_lookahead_depth=1)
        cfg.update(overrides)
        return JointViewPolicy(lambda k,d:cfg.get(k,d))

    def record(self,xy=(4.,2.)):
        return dict(position=[*xy,1.2],state='suspected',actionable=True,
            priority=10.,suspected_episodes=1,last_dynamic=1.)

    def test_home_ghost_is_deferred_then_really_selected_on_return(self):
        p=self.policy();records=[self.record()];original=copy.deepcopy(records)
        self.assertIsNone(self.selection(p,records))
        self.assertTrue(p.report['deferred_home_groups']);self.assertEqual(p.stats,{})
        v=self.selection(p,records,return_phase=True,now=101.)
        self.assertIsNotNone(v);self.assertEqual(v.kind,'verification')
        self.assertEqual(records,original)
        self.assertEqual(p.stats,{}) # selection/deferral is not measured evidence

    def test_distant_ghost_is_not_deferred_as_a_home_task(self):
        p=self.policy();v=self.selection(p,[self.record((9.,9.))])
        self.assertIsNotNone(v);self.assertFalse(p.report['deferred_home_groups'])

    def test_connectivity_critical_home_ghost_remains_immediate(self):
        p=self.policy();p._connection=Mock(return_value=10.)
        self.assertIsNotNone(self.selection(p,[self.record()]))
        self.assertFalse(p.report['deferred_home_groups'])

    def test_return_cannot_spend_unbounded_detour_or_nominal_reserve(self):
        p=self.policy(joint_home_review_radius_m=12.,joint_return_review_max_detour_m=.01)
        self.assertIsNone(self.selection(p,[self.record((6.,8.))],True))
        self.assertGreater(p.report['rejected'].get('return_review_detour',0),0)
        self.assertIsNone(self.selection(self.policy(),[self.record()],True,now=590.))

    def test_return_does_not_select_a_view_that_exceeds_remaining_action_time(self):
        p=self.policy()
        self.assertIsNone(self.selection(p,[self.record()],True,return_review_remaining_s=2.))
        self.assertGreater(p.report['rejected'].get('return_review_action_time',0),0)
        p=self.policy()
        self.assertIsNotNone(self.selection(p,[self.record()],True,return_review_remaining_s=30.))

    def test_history_height_queries_are_batched_before_ray_budget(self):
        records=[dict(self.record(),position=[4.+.001*i,2.,1.2]) for i in range(800)]
        shapes=[]
        def height(xy):
            q=np.asarray(xy);shapes.append(q.shape)
            return np.full(q.shape[:-1],1.2)
        p=self.policy()
        self.assertIsNotNone(self.selection(p,records,True,view_altitude=height))
        self.assertEqual(shapes[0],(800,2))
        self.assertLess(len(shapes),50)

    def test_productive_frontier_gate_cancels_surface_target_without_cancelling_frontier(self):
        g=ExplorationGrid([0,0,12,12],.2,.8,.4);g.known[:]=True;g.update_obstacles([])
        d,_=g.search(g.cell((2.,2.)));b=FlightBudget();b.observe_arm(False,1);b.observe_arm(True,2)
        p=self.policy(joint_surface_inspection_enabled=True,joint_surface_start_seconds=0.,
            joint_unseen_geometry_enabled=True)
        def choose(allowed,now):
            return p.select(g,(2.,2.),d,d,np.array([g.cell((8.,8.))]),np.array([10.]),[],
                np.empty((0,3)),np.empty((0,3)),1.2,now,b,
                inspection_cells=np.array([g.cell((3.,2.))]),inspection_gains=np.array([80.]),
                inspection_sources=['missing_boundary'],surface_allowed=allowed)
        self.assertEqual(choose(True,100.).kind,'surface_inspection')
        v=choose(False,101.);self.assertEqual(v.kind,'frontier')
        self.assertFalse(p.report['retained_missing_target'])

    def test_return_time_limit_does_not_resolve_pending_voxel(self):
        from test_exploration_mission import MissionTest
        n=MissionTest().ready();n.joint_policy=self.policy();n.return_review_started=80.
        n.ghost_history={'records':[self.record()]};original=copy.deepcopy(n.ghost_history)
        d,parent=n.grid.search(n.grid.cell(n.home_xy))
        self.assertFalse(n.return_review_tick(100.,n.home_xy,n.grid.cell(n.home_xy),d,parent,n.inputs))
        self.assertTrue(n.return_review_done);self.assertEqual(n.ghost_history,original)
        self.assertEqual(n.return_review_report['reason'],'time_limit')

    def test_return_retries_closer_to_home_instead_of_permanently_skipping_review(self):
        from test_exploration_mission import MissionTest
        n=MissionTest().ready();n.joint_policy=self.policy();n.phase='RETURNING'
        n.flight_budget=FlightBudget(review=15.);n.flight_budget.observe_arm(False,1);n.flight_budget.observe_arm(True,2)
        n.ghost_status_fresh=Mock(return_value=True);n.ghost_history_fresh=Mock(return_value=True)
        n.ghost_history={'records':[self.record()]};n.joint_event=Mock()
        n.flight_home_distances,_=n.grid.search(n.grid.cell(n.home_xy))
        v=View(n.grid.cell((1.1,2.1)),(1.1,2.1),'verification',group=(5,2),episode=(5,2,1,'active'),dwell_s=4.)
        n.joint_policy.select=Mock(side_effect=[None,v])
        position=(-1.9,2.1);d,parent=n.grid.search(n.grid.cell(position))
        self.assertFalse(n.return_review_tick(100.,position,n.grid.cell(position),d,parent,n.inputs))
        self.assertFalse(n.return_review_done);self.assertIsNone(n.return_review_started)
        position=(.1,2.1);d,parent=n.grid.search(n.grid.cell(position))
        self.assertTrue(n.return_review_tick(102.,position,n.grid.cell(position),d,parent,n.inputs))
        self.assertEqual(n.return_review_started,102.);self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.publish.call_args[0][0],'RETURN_VERIFYING')

    def test_landing_configuration_requires_disarm_and_exact_readback(self):
        import threading
        n=EnduranceLanding.__new__(EnduranceLanding);n.lock=threading.Lock()
        n.state=SimpleNamespace(armed=False)
        values={'MPC_LAND_SPEED':.7,'MPC_LAND_CRWL':.3,'LNDMC_Z_VEL_MAX':.25}
        def get(param_id):return SimpleNamespace(success=True,value=SimpleNamespace(real=values[param_id]))
        def change(param_id,value):
            values[param_id]=value.real;return SimpleNamespace(success=True)
        with patch('endurance_landing.rospy.wait_for_service'),patch(
                'endurance_landing.rospy.ServiceProxy',side_effect=lambda name,kind:get if name.endswith('/get') else change):
            r=n.configure_slow_descent();self.assertEqual(r['verified'],values)
            self.assertLess(values['LNDMC_Z_VEL_MAX'],values['MPC_LAND_CRWL']/1.2)
            n.state.armed=True
            with self.assertRaises(RuntimeError):n.configure_slow_descent()
        n.state.armed=False
        with patch('endurance_landing.rospy.wait_for_service'),patch(
                'endurance_landing.rospy.ServiceProxy',return_value=Mock(return_value=SimpleNamespace(success=False))):
            with self.assertRaises(RuntimeError):n.configure_slow_descent()


if __name__=='__main__':unittest.main()
