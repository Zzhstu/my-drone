#!/usr/bin/env python3
"""Corner approach versus aisle axis, with independent safety/false cues."""
import unittest,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from joint_view_policy import JointViewPolicy
from flight_budget import FlightBudget

class CorridorEntryTest(unittest.TestCase):
    def scene(self,enabled=True,wall=True):
        g=ExplorationGrid([0,0,16,16],.2,.8,.4);g.known[:]=True
        pts=np.array([[x,float(y),1.2] for x in (6.,8.2) for y in np.arange(4.,12.,.1)])
        g.update_obstacles(pts);mask=g.raster(pts) if wall else None
        p=JointViewPolicy(lambda k,d:dict(joint_corridor_progress_enabled=True,
            joint_corridor_entry_enabled=enabled,joint_route_memory_enabled=True,
            joint_extra_compute_budget_ms=1000.).get(k,d))
        for x in np.arange(1.1,5.31,.25):p.observe_position((x,3.3))
        return p,g,mask

    def choose(self,p,g,mask,now=100.,position=(5.3,3.3)):
        d,parent=g.search(g.cell(position));h,_=g.search(g.cell((2.1,2.1)))
        b=FlightBudget();b.observe_arm(False,1.);b.observe_arm(True,2.)
        cells=np.array([g.cell(x) for x in [(7.1,5.1),(2.1,2.1)]])
        before=[getattr(g,n).copy() for n in ['known','safe','occupied']]
        view=p.select(g,position,d,h,cells,np.array([1.,35.]),[],np.empty((0,3)),
            np.empty((0,3)),1.2,now,b,route_parent=parent,corridor_wall_mask=mask)
        for n,v in zip(['known','safe','occupied'],before):np.testing.assert_array_equal(getattr(g,n),v)
        return view

    def test_diagonal_approach_can_enter_independently_measured_aisle_axis(self):
        p,g,m=self.scene(False);self.assertFalse(self.choose(p,g,m).corridor_task)
        p,g,m=self.scene();v=self.choose(p,g,m)
        self.assertTrue(v.corridor_task);self.assertEqual(v.cell,g.cell((7.1,5.1)))
        r=p.report['corridor_progress'];self.assertEqual(r['reason'],'enter_observed_turn')
        np.testing.assert_allclose(r['entry_axis'],[0,1],atol=1e-6)
        self.assertGreaterEqual(r['continuous_wall_span_m'],2.39)
        self.assertTrue(any('entry_structure' in row for row in r['candidate_checks']))

    def test_low_shelves_without_measured_high_wall_cannot_bootstrap_entry(self):
        p,g,m=self.scene(wall=False);self.assertFalse(self.choose(p,g,m).corridor_task)

    def test_far_high_wall_hidden_by_low_shelf_cannot_authorize_turn(self):
        p,g,m=self.scene();m[:]=False
        for y in np.arange(4.,12.,.1):m[g.cell((9.1,y))]=True
        self.assertFalse(self.choose(p,g,m).corridor_task)

    def test_only_one_wall_does_not_create_entry(self):
        p,g,m=self.scene();pts=np.array([[8.2,y,1.2] for y in np.arange(4.,12.,.1)])
        g.update_obstacles(pts);self.assertFalse(self.choose(p,g,g.raster(pts)).corridor_task)

    def test_unknown_or_occupied_entry_never_authorized(self):
        for occupied in [False,True]:
            p,g,m=self.scene();cell=g.cell((7.1,5.1))
            if occupied:g.occupied[cell]=True
            else:g.known[cell]=False
            g.safe[cell]=False
            v=self.choose(p,g,m);self.assertTrue(v is None or v.cell!=cell)

    def test_entrance_commit_preserves_axis_until_a_safe_leg_completes(self):
        p,g,m=self.scene();v=self.choose(p,g,m);self.assertTrue(v.corridor_task)
        p.corridor.observe((4.1,3.3));p.corridor.observe((3.1,3.3))
        np.testing.assert_allclose(p.corridor.heading(),[0,1],atol=1e-6)
        v2=self.choose(p,g,m,now=101.);self.assertEqual(v2.cell,v.cell)
        self.assertEqual(p.report['corridor_progress']['reason'],'committed_safe_leg')

    def test_entrance_expiry_and_revision_release_intention(self):
        p,g,m=self.scene();self.choose(p,g,m)
        self.choose(p,g,m,now=131.)
        self.assertEqual(p.report['corridor_progress']['reason'],'entrance_time_limit')
        self.assertIsNone(p.corridor.entry_axis)
        p,g,m=self.scene();self.choose(p,g,m);p.invalidate_coordinates()
        self.assertIsNone(p.corridor.entry_axis)

    def test_safety_loss_and_budget_preempt_entry_commit(self):
        p,g,m=self.scene();v=self.choose(p,g,m);g.safe[v.cell]=False
        self.choose(p,g,m,now=101.)
        self.assertEqual(p.report['corridor_progress']['reason'],'leg_invalidated')
        p,g,m=self.scene();self.assertIsNone(self.choose(p,g,m,now=590.))

if __name__=='__main__':unittest.main()
