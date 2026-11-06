#!/usr/bin/env python3
"""Verify joint observation lifecycle through the real Explorer safety gates."""
import unittest
from unittest.mock import Mock
import numpy as np
import rospy
import test_exploration_mission as legacy_mission
from joint_view_policy import JointViewPolicy, View
from flight_budget import FlightBudget


class JointMissionTest(unittest.TestCase):
    def ready(self):
        helper=legacy_mission.MissionTest();node=helper.ready()
        node.joint_policy=JointViewPolicy()
        node.flight_budget=FlightBudget()
        node.flight_budget.observe_arm(False,1.);node.flight_budget.observe_arm(True,2.)
        node.joint_event=Mock()
        node.ghost_history_fresh=Mock(return_value=True)
        node.ghost_status_fresh=Mock(return_value=True)
        node.ghost_history={'records':[dict(position=[2.1,2.1,1.2],suspected_episodes=1)]}
        return node,helper

    def probe(self,node,arrived=100.,scans=0):
        view=View(node.grid.cell((2.1,2.1)),(2.1,2.1),'verification',
                  group=(2,2),episode=(2,2,1,'active'),quality=.5)
        node.joint_policy.selected=view
        node.joint_observation=dict(view=view,arrived=arrived,last_stamp=-float('inf'),scans=scans,
            before={(1,1,1):(0,0,0,'suspected',0)},selected=99.,quality=.5)
        node.ghost_group_counts=Mock(return_value={(1,1,1):(0,0,0,'suspected',0)})
        return view

    def test_joint_branch_does_not_enter_legacy_endurance_revisit(self):
        n,h=self.ready();n.joint_tick=Mock();n.endurance_revisit_due=Mock(return_value=True)
        h.tick_at_100(n)
        n.joint_tick.assert_called_once();n.endurance_revisit_due.assert_not_called()

    def test_return_budget_preempts_joint_selector(self):
        n,h=self.ready();n.flight_budget=FlightBudget(total=100.,reserve=10.)
        n.flight_budget.observe_arm(False,1.);n.flight_budget.observe_arm(True,2.)
        n.joint_tick=Mock();h.tick_at_100(n)
        self.assertEqual(n.phase,'RETURNING');n.joint_tick.assert_not_called()

    def test_return_phase_cannot_be_reversed_by_joint_policy(self):
        n,h=self.ready();n.phase='RETURNING';n.return_started=90.
        n.return_tick=Mock();n.joint_tick=Mock();h.tick_at_100(n)
        n.return_tick.assert_called_once();n.joint_tick.assert_not_called()
        self.assertEqual(n.phase,'RETURNING')

    def test_arrival_without_eight_new_scans_cannot_claim_evidence(self):
        n,_=self.ready();self.probe(n,94.)
        n.joint_finish_observation(100.,n.inputs,(2.1,2.1))
        self.assertFalse(n.ghost_view_evidence[-1]['effective'])
        self.assertEqual(n.ghost_view_evidence[-1]['status'],'insufficient_fresh_evidence')

    def test_repeated_acquisition_stamp_is_not_multiple_scans(self):
        n,_=self.ready();self.probe(n)
        self.assertTrue(n.joint_finish_observation(100.1,n.inputs,(2.1,2.1)))
        self.assertTrue(n.joint_finish_observation(100.2,n.inputs,(2.1,2.1)))
        self.assertEqual(n.joint_observation['scans'],1)

    def test_fresh_scans_without_map_evidence_do_not_certify_success(self):
        n,_=self.ready();self.probe(n,96.,8)
        n.joint_finish_observation(100.,n.inputs,(2.1,2.1))
        result=n.ghost_view_evidence[-1]
        self.assertEqual(result['status'],'measured');self.assertFalse(result['effective'])

    def test_independent_mapper_free_evidence_can_make_view_effective(self):
        n,_=self.ready();view=self.probe(n,96.,8)
        n.ghost_group_counts.return_value={(1,1,1):(2,0,0,'cleared',0)}
        n.joint_finish_observation(100.,n.inputs,(2.1,2.1))
        self.assertTrue(n.ghost_view_evidence[-1]['effective'])
        self.assertEqual(n.joint_policy.stats[view.episode]['effective'],1)

    def test_new_ghost_episode_cannot_supply_old_probe_evidence(self):
        n,_=self.ready();self.probe(n,96.,8)
        n.ghost_group_counts.return_value={(1,1,1):(2,0,0,'cleared',0)}
        n.ghost_history['records'][0]['suspected_episodes']=2
        n.joint_finish_observation(100.,n.inputs,(2.1,2.1))
        self.assertFalse(n.ghost_view_evidence[-1]['effective'])

    def test_legacy_configuration_does_not_construct_a_joint_policy(self):
        self.assertIsNone(legacy_mission.MissionTest().ready().joint_policy)


if __name__=='__main__':unittest.main()
