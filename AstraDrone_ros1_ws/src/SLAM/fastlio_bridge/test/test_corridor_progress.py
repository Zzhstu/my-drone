#!/usr/bin/env python3
"""A corridor task advances only through measured-safe reachable frontiers."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from joint_view_policy import JointViewPolicy
from flight_budget import FlightBudget


class CorridorProgressTest(unittest.TestCase):
    def grid(self, second_wall=True):
        g=ExplorationGrid([0,0,16,16],.2,.8,.4);g.known[:]=True
        points=[[6.,float(y),1.2] for y in np.arange(4.,12.,.1)]
        if second_wall:points += [[8.2,float(y),1.2] for y in np.arange(4.,12.,.1)]
        g.update_obstacles(np.asarray(points));return g

    def policy(self, enabled=True):
        values=dict(joint_corridor_progress_enabled=enabled,joint_route_memory_enabled=True,
                    joint_lookahead_depth=2,joint_extra_compute_budget_ms=1000.)
        p=JointViewPolicy(lambda k,d:values.get(k,d))
        # Arrive from the left; the corridor requires a 90-degree turn.
        for x in np.arange(2.1,7.11,.25):p.observe_position((x,3.1))
        return p

    def select(self,p,g=None,now=100.,position=(7.1,3.1),xy=None,returning=False,focus=None):
        g=self.grid() if g is None else g;start=g.cell(position);d,parent=g.search(start)
        h,_=g.search(g.cell((2.1,2.1)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        targets=xy if xy is not None else [(7.1,6.9),(2.1,2.1)]
        cells=np.asarray([g.cell(v) for v in targets],int).reshape(-1,2)
        gains=np.asarray([1.,35.][:len(cells)])
        before=[getattr(g,k).copy() for k in ('safe','known','occupied')]
        result=p.select(g,position,d,h,cells,gains,[],np.empty((0,3)),np.empty((0,3)),1.2,now,b,
                        route_parent=parent,indoor_weights=focus,return_phase=returning)
        for key,old in zip(('safe','known','occupied'),before):np.testing.assert_array_equal(getattr(g,key),old)
        return result

    def test_small_forward_corridor_beats_distant_large_backtrack(self):
        baseline=self.policy(False);self.assertFalse(self.select(baseline).corridor_task)
        p=self.policy();view=self.select(p)
        self.assertTrue(view.corridor_task)
        self.assertEqual(view.cell,self.grid().cell((7.1,6.9)))
        self.assertEqual(p.report['corridor_progress']['reason'],'advance_observed_corridor')

    def test_committed_leg_does_not_jump_to_a_later_unobserved_endpoint(self):
        p=self.policy();first=self.select(p)
        second=self.select(p,now=101.,xy=[(7.1,9.9),(2.1,2.1)])
        self.assertEqual(second.cell,first.cell)
        self.assertEqual(p.report['corridor_progress']['reason'],'committed_safe_leg')

    def test_unknown_forward_goal_never_becomes_free_and_scan_hold_expires(self):
        p=self.policy();g=self.grid();g.known[30:,:]=False
        occupied=np.argwhere(g.occupied);g.update_obstacles(np.asarray([(*g.xy(tuple(c)),1.2) for c in occupied]))
        first=self.select(p,g)
        self.assertTrue(first is None or first.cell!=g.cell((7.1,6.9)))
        second=self.select(p,g,now=103.)
        self.assertIsNotNone(second);self.assertFalse(second.corridor_task)

    def test_single_wall_does_not_create_an_indoor_corridor(self):
        p=self.policy();view=self.select(p,self.grid(second_wall=False))
        self.assertIsNotNone(view);self.assertFalse(view.corridor_task)

    def test_isolated_short_shelves_cannot_start_a_wall_following_task(self):
        p=self.policy();g=self.grid()
        points=np.asarray([[x,float(y),1.2] for x in (6.,8.2) for y in np.arange(5.1,8.7,.1)])
        g.update_obstacles(points)
        view=self.select(p,g)
        self.assertIsNotNone(view);self.assertFalse(view.corridor_task)
        self.assertGreater(p.report['corridor_progress']['rejected'].get('short_or_nonparallel_structure',0),0)

    def test_collinear_separate_shelves_do_not_form_a_long_wall(self):
        p=self.policy();g=self.grid()
        ys=np.concatenate((np.arange(2.,5.6,.1),np.arange(6.2,9.8,.1),np.arange(10.4,14.,.1)))
        g.update_obstacles(np.asarray([[x,float(y),1.2] for x in (6.,8.2) for y in ys]))
        cue=p.corridor.structure(g,np.array([7.1,7.7]),np.array([0.,1.]))
        self.assertIsNotNone(cue);self.assertFalse(cue['long_parallel_boundary'])

    def test_wall_behind_shelves_cannot_authorize_the_internal_aisle(self):
        p=self.policy();g=self.grid();mask=np.zeros(g.shape,bool)
        for y in np.arange(4.,12.,.1):mask[g.cell((9.,y))]=True
        cue=p.corridor.structure(g,np.array([7.1,7.7]),np.array([0.,1.]),mask)
        self.assertFalse(cue['long_parallel_boundary'])
        for y in np.arange(4.,12.,.1):mask[g.cell((8.2,y))]=True
        cue=p.corridor.structure(g,np.array([7.1,7.7]),np.array([0.,1.]),mask)
        self.assertTrue(cue['long_parallel_boundary'])

    def test_live_safety_loss_preempts_corridor_commitment(self):
        p=self.policy();first=self.select(p);g=self.grid()
        points=np.asarray([(*g.xy(tuple(c)),1.2) for c in np.argwhere(g.occupied)]+[[*first.xy,1.2]])
        g.update_obstacles(points);view=self.select(p,g,now=101.)
        self.assertTrue(view is None or view.cell!=first.cell)
        self.assertEqual(p.report['corridor_progress']['reason'],'leg_invalidated')

    def test_return_and_exhausted_budget_cannot_be_overridden(self):
        p=self.policy();self.assertIsNone(self.select(p,returning=True))
        self.assertEqual(p.report['corridor_progress']['reason'],'return_phase')
        p=self.policy();self.assertIsNone(self.select(p,now=590.))

    def test_task_time_limit_releases_to_normal_frontiers(self):
        p=self.policy();p.corridor.started=1.
        view=self.select(p,now=200.)
        self.assertIsNotNone(view);self.assertFalse(view.corridor_task)
        self.assertEqual(p.report['corridor_progress']['reason'],'task_time_limit')

    def test_corridor_can_interrupt_a_recently_chosen_long_backtrack(self):
        p=self.policy(False);back=self.select(p)
        p.p['joint_corridor_progress_enabled']=True
        for x in np.arange(2.1,7.11,.25):p.corridor.observe((x,3.1))
        back.repeated_path_m=5.;back.xy=(.9,2.1);back.cell=self.grid().cell(back.xy)
        view=self.select(p,now=100.1)
        self.assertTrue(view.corridor_task)

    def test_local_short_step_survives_ordinary_goal_suppression(self):
        p=self.policy();g=self.grid();position=(7.1,3.1);d,_=g.search(g.cell(position))
        mask=np.zeros(g.shape,bool);cell=g.cell((7.1,4.3));mask[cell]=True
        cells=p.corridor.local_frontiers(g,position,d,mask,p.route_memory)
        self.assertIn(cell,list(map(tuple,cells)))

    def test_frame_revision_clears_corridor_intention_not_flight_action_budget(self):
        p=self.policy();self.select(p);p.used_surface_seconds=12.
        p.invalidate_coordinates()
        self.assertIsNone(p.corridor.started);self.assertEqual(len(p.corridor.positions),0)
        self.assertEqual(p.used_surface_seconds,12.)


if __name__=='__main__':unittest.main()
