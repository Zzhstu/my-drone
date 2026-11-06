#!/usr/bin/env python3
"""Home ghost zero means zero in every phase, never fabricated map evidence."""
import copy
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from joint_view_policy import JointViewPolicy, View, reserved_home_review_seconds
from flight_budget import FlightBudget
import test_return_observation as return_fixture


class HomeGhostZeroTest(unittest.TestCase):
    def fixture(self, **extra):
        helper=return_fixture.ReturnObservationTest()
        p=helper.policy(joint_home_ghost_reward_scale=0.,joint_defer_home_review=False,**extra)
        return helper,p

    def test_active_and_cleared_home_voxels_are_zero_in_both_phases(self):
        h,p=self.fixture()
        for historical in (False,True):
            record=h.record()
            if historical:record.update(state='cleared',occupied_now=False,cleared_episodes=2)
            original=copy.deepcopy(record)
            for returning in (False,True):
                self.assertIsNone(h.selection(p,[record],return_phase=returning))
                hp=p.report['home_ghost_policy']
                self.assertEqual(hp['zeroed_records'],1)
                self.assertEqual(hp['zeroed_samples'][0]['score'],0.)
            self.assertEqual(record,original);self.assertEqual(p.stats,{})

    def test_held_old_target_and_two_step_ranking_cannot_restore_home_reward(self):
        h,p=self.fixture(joint_lookahead_depth=2)
        p.p['joint_home_ghost_reward_scale']=1.
        self.assertIsNotNone(h.selection(p,[h.record()]))
        p.selected.area_m2=100.;p.selected.connectivity_m2=100.
        p.p['joint_home_ghost_reward_scale']=0.
        self.assertIsNone(h.selection(p,[h.record()],now=100.1))
        self.assertIsNone(p.selected)

    def test_incidental_frontier_or_connectivity_gain_does_not_resurrect_ghost_action(self):
        _,p=self.fixture()
        view=View((2,2),(2.,2.),'verification',group=(1,1),area_m2=100.,priority=100.,
                  quality=1.,connectivity_m2=100.,ghost_reward_scale=0.)
        self.assertEqual(p._value(view),0.);self.assertEqual(p._score(view),0.)
        view.group=None;view.kind='frontier'
        self.assertGreater(p._value(view),0.)
        view.kind='surface_inspection';view.inspection_source='missing_boundary'
        self.assertGreater(p._value(view),0.)

    def test_connectivity_exception_cannot_override_explicit_zero(self):
        h,p=self.fixture();p._connection=Mock(return_value=100.)
        self.assertIsNone(h.selection(p,[h.record()]))
        p._connection.assert_not_called()

    def test_ghost_outside_home_radius_retains_its_reward(self):
        h,p=self.fixture();view=h.selection(p,[h.record((9.,9.))])
        self.assertIsNotNone(view);self.assertGreater(view.score,0.)
        self.assertEqual(p.report['home_ghost_policy']['zeroed_records'],0)

    def test_voxels_straddling_radius_are_filtered_before_group_aggregation(self):
        h,p=self.fixture()
        inside=dict(h.record((7.,2.)),priority=1000.)
        outside=dict(h.record((7.01,2.)),priority=1.)
        view=h.selection(p,[inside,outside])
        self.assertIsNotNone(view);self.assertEqual(view.priority,1.)
        self.assertEqual(p.report['home_ghost_policy']['zeroed_records'],1)

    def test_optional_review_reserve_is_removed_without_spending_safety_reserve(self):
        settings=dict(joint_home_ghost_reward_scale=0.,joint_defer_home_review=True,
                      joint_return_review_seconds=15.)
        review=reserved_home_review_seconds(lambda k,d:settings.get(k,d))
        b=FlightBudget(review=review)
        self.assertEqual(b.review,0.);self.assertEqual(b.reserve,60.)
        self.assertEqual(b.total,600.);self.assertEqual(b.fixed_return_seconds,45.)
        settings['joint_home_ghost_reward_scale']=1.
        self.assertEqual(reserved_home_review_seconds(lambda k,d:settings.get(k,d)),15.)

    def test_return_cancels_optional_trip_without_touching_ledger(self):
        from test_exploration_mission import MissionTest
        n=MissionTest().ready();h,p=self.fixture();p.p['joint_defer_home_review']=True
        n.joint_policy=p;n.joint_event=Mock();p.select=Mock()
        n.ghost_history={'records':[h.record()]};before=copy.deepcopy(n.ghost_history)
        n.joint_observation=dict(should_be_cancelled=True)
        d,parent=n.grid.search(n.grid.cell(n.home_xy))
        self.assertFalse(n.return_review_tick(100.,n.home_xy,n.grid.cell(n.home_xy),d,parent,n.inputs))
        self.assertTrue(n.return_review_done);self.assertIsNone(n.joint_observation)
        self.assertEqual(n.return_review_report['reason'],'home_ghost_score_zero')
        self.assertEqual(n.ghost_history,before);p.select.assert_not_called()

    def test_out_of_range_or_nonfinite_scale_is_rejected(self):
        for value in (-.1,1.1,float('nan')):
            with self.assertRaises(ValueError):JointViewPolicy(lambda k,d:value if k=='joint_home_ghost_reward_scale' else d)


if __name__=='__main__':unittest.main()
