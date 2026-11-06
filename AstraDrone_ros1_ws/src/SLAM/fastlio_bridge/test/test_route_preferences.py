#!/usr/bin/env python3
"""Safety/behavior counterexamples for actual-passage preference scoring."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from exploration_route_memory import RouteMemory, RouteQuery
from joint_view_policy import JointViewPolicy, View
from flight_budget import FlightBudget


class RoutePreferenceTest(unittest.TestCase):
    def grid(self):
        g=ExplorationGrid([0,0,14,14],.2,.8,.4)
        g.known[:]=True;g.update_obstacles([])
        return g

    def policy(self, **values):
        settings=dict(joint_route_memory_enabled=True,joint_lookahead_depth=1,
                      joint_extra_compute_budget_ms=1000.,joint_unknown_gain_multiplier=1.25)
        settings.update(values)
        return JointViewPolicy(lambda k,d:settings.get(k,d))

    def select(self,p,xy=((4.1,6.1),(8.1,6.1)),records=(),return_phase=False):
        g=self.grid();start=g.cell((6.1,6.1));d,parent=g.search(start)
        h,_=g.search(g.cell((2.1,6.1)))
        budget=FlightBudget(review=15.);budget.observe_arm(False,1.);budget.observe_arm(True,2.)
        arrays=[getattr(g,k).copy() for k in ('safe','known','occupied')]
        chosen=p.select(g,(6.1,6.1),d,h,np.asarray([g.cell(v) for v in xy],int).reshape(-1,2),
            np.full(len(xy),12.),list(records),np.empty((0,3)),np.empty((0,3)),1.2,100.,budget,
            home_xy=(2.1,6.1),route_parent=parent,return_phase=return_phase)
        for old,k in zip(arrays,('safe','known','occupied')):np.testing.assert_array_equal(old,getattr(g,k))
        return chosen

    def fly_east(self,m):
        for x in np.arange(2.1,6.11,.25):m.observe((x,6.1))

    def test_planning_and_stationary_noise_do_not_create_visited_corridors(self):
        g=self.grid();m=RouteMemory();start=g.cell((2.1,6.1));_,parent=g.search(start)
        q=RouteQuery(g,parent,start,m)
        self.assertEqual(q.cost(g.cell((8.1,6.1)))[0],0.)
        self.assertEqual(m.visits,{})
        for x in [2.1,2.11,2.09,2.12]*20:m.observe((x,6.1))
        self.assertEqual(m.total_m,0.);self.assertEqual(m.repeated_m,0.)

    def test_actual_return_over_the_same_tiles_is_counted(self):
        m=RouteMemory();self.fly_east(m)
        self.assertEqual(m.repeated_m,0.)
        for x in np.arange(6.1,2.09,-.25):m.observe((x,6.1))
        self.assertGreater(m.repeated_m,3.)
        self.assertLessEqual(m.repeated_m,m.total_m)

    def test_pose_jump_cannot_mark_unflown_middle_tiles(self):
        m=RouteMemory();m.observe((1.,1.));m.observe((10.,1.))
        self.assertNotIn(m.key((5.,1.)),m.visits)
        self.assertEqual(m.jumps,1);self.assertEqual(m.total_m,0.)

    def test_soft_repeat_cost_changes_a_real_backtracking_choice(self):
        baseline=self.policy(joint_route_memory_enabled=False)
        self.assertEqual(self.select(baseline).xy,self.grid().xy(self.grid().cell((4.1,6.1))))
        p=self.policy();self.fly_east(p.route_memory)
        chosen=self.select(p)
        self.assertEqual(chosen.xy,self.grid().xy(self.grid().cell((8.1,6.1))))
        behind=next(v for v in p.report['top_candidates'] if v['xy'][0]<6.)
        self.assertGreater(behind['repeat_cost_s'],0.)
        self.assertGreater(behind['reversal_cost_s'],0.)

    def test_necessary_return_corridor_is_not_made_unreachable(self):
        p=self.policy(joint_route_repeat_weight=100.,joint_route_reverse_seconds=100.)
        self.fly_east(p.route_memory)
        v=self.select(p,xy=((4.1,6.1),))
        self.assertIsNotNone(v);self.assertGreater(v.repeat_cost_s,100.)
        self.assertNotIn('return_budget',p.report['rejected'])

    def test_return_review_ignores_exploration_preferences(self):
        p=self.policy(joint_route_repeat_weight=100.,joint_route_reverse_seconds=100.)
        self.fly_east(p.route_memory)
        record=dict(position=[4.,5.,1.2],state='suspected',actionable=True,
                    priority=10.,suspected_episodes=1,last_dynamic=1.)
        v=self.select(p,xy=(),records=[record],return_phase=True)
        self.assertIsNotNone(v);self.assertEqual(v.route_cost_s,0.)
        self.assertFalse(p.report['route_preferences']['enabled'])

    def test_unknown_boost_does_not_reduce_missing_floor_or_back_utility(self):
        p=self.policy();base=self.policy(joint_unknown_gain_multiplier=1.)
        for source in ('missing_floor','missing_boundary'):
            v=View((1,1),(1.,1.),'surface_inspection',priority=16.,quality=1.,inspection_source=source)
            self.assertEqual(p._value(v),base._value(v))
        v=View((1,1),(1.,1.),'frontier',area_m2=5.)
        self.assertAlmostEqual(p._value(v),1.25*base._value(v))

    def test_region_continuity_is_soft_and_releases_when_region_has_no_candidate(self):
        p=self.policy(joint_route_region_switch_seconds=30.)
        p.active_route_region=p.route_region((4.1,6.1))
        self.select(p)
        other=next(v for v in p.report['top_candidates'] if v['xy'][0]>6.)
        self.assertEqual(other['region_switch_cost_s'],30.)
        p.selected=None;p.active_route_region=(-20,-20)
        self.assertIsNotNone(self.select(p))
        self.assertTrue(all(v['region_switch_cost_s']==0. for v in p.report['top_candidates']))

    def test_grid_growth_preserves_world_passages_but_graph_revision_clears_them(self):
        p=self.policy();self.fly_east(p.route_memory)
        g=self.grid();start=g.cell((6.1,6.1));_,parent=g.search(start)
        before=RouteQuery(g,parent,start,p.route_memory).cost(g.cell((2.1,6.1)))[0]
        g.grow_to_include(np.array([[20.,20.,1.2]]))
        g.known[:]=True;g.update_obstacles([]);_,parent=g.search(g.cell((6.1,6.1)))
        after=RouteQuery(g,parent,g.cell((6.1,6.1)),p.route_memory).cost(g.cell((2.1,6.1)))[0]
        self.assertAlmostEqual(before,after)
        p.used_ghost_seconds=12.;p.invalidate_coordinates()
        self.assertEqual(p.route_memory.visits,{});self.assertEqual(p.used_ghost_seconds,12.)


if __name__=='__main__':unittest.main()
