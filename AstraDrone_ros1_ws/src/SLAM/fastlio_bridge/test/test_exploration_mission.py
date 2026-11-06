#!/usr/bin/env python3
"""Mission and inspection regressions; no ROS master or simulator required."""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from warehouse_explorer import Explorer
from exploration_grid import ExplorationGrid
from flight_budget import FlightBudget


class MissionTest(unittest.TestCase):
    def test_arm_budget_reserves_revisit_when_legacy_mission_limit_is_disabled(self):
        n=self.ready();n.flight_budget=FlightBudget()
        n.flight_budget.observe_arm(False,1.);n.flight_budget.observe_arm(True,2.)
        original=n.p
        n.p=lambda k,d: {'ghost_endurance_revisit':True,'ghost_revisit_enabled':True,
            'max_mission_seconds':0.,'ghost_endurance_min_exploration_seconds':60.}.get(k,original(k,d))
        n.ghost_status={'enabled':True,'pending_voxels':1};n.ghost_status_fresh=Mock(return_value=True)
        n.ghost_history_fresh=Mock(return_value=True)
        n.ghost_history={'records':[dict(state='suspected',actionable=True,last_dynamic=70.)]}
        self.assertTrue(n.endurance_revisit_due(100.))
        n.ghost_history['records'][0]['last_dynamic']=99.
        self.assertFalse(n.endurance_revisit_due(100.))
        n.ghost_history_fresh.return_value=False
        self.assertFalse(n.endurance_revisit_due(100.))

    def test_endurance_clock_before_arm_origin_holds_without_selecting(self):
        n=self.ready();n.flight_budget=FlightBudget()
        n.flight_budget.observe_arm(False,110.);n.flight_budget.observe_arm(True,111.)
        n.ghost_revisit_tick=Mock();n.return_tick=Mock()
        self.tick_at_100(n)
        self.assertEqual(n.publish.call_args[0][0],'WAIT_FLIGHT_BUDGET_ORIGIN')
        n.ghost_revisit_tick.assert_not_called();n.return_tick.assert_not_called()

    def test_endurance_guard_returns_before_nominal_limit(self):
        n=self.ready();n.flight_budget=FlightBudget(total=100.,reserve=10.)
        n.flight_budget.observe_arm(False,1.);n.flight_budget.observe_arm(True,2.)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.finish_reason,'flight_budget_return_partial')

    def test_endurance_filter_rejects_unaffordable_round_trip(self):
        n=self.ready();n.flight_budget=FlightBudget(total=160.,reserve=10.)
        n.flight_budget.observe_arm(False,1.);n.flight_budget.observe_arm(True,2.)
        mask=np.zeros(n.grid.shape,dtype=bool);mask[n.grid.cell((3.1,2.1))]=True
        n.grid.frontiers=Mock(return_value=(mask,np.ones(n.grid.shape)))
        with patch('warehouse_explorer.visible_unknown_gain',return_value=np.array([2.])):
            self.tick_at_100(n)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.finish_reason,'flight_budget_no_affordable_view_partial')

    def test_multilevel_flight_config_fails_closed_before_publishers(self):
        def param(key, default):
            return True if key == '~ghost_multilevel_flight_enabled' else default
        with patch('warehouse_explorer.rospy.get_param',side_effect=param),\
                patch('warehouse_explorer.rospy.Publisher') as publisher:
            with self.assertRaisesRegex(ValueError,'no certified full 3-D route'):
                Explorer()
        publisher.assert_not_called()

    def test_vertical_view_diagnostic_never_commands_altitude_change(self):
        n=self.ready()
        n.phase='REVISITING';n.ghost_revisit_started=100.
        n.ghost_vertical_diagnostic_enabled=True
        n.ghost_vertical_view_offsets=(-.55,)
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(101)
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[4.,4.,.25]])
                     if name=='suspects' else np.empty((0,3)))
        old=n.p
        n.p=lambda k,d: 1.6 if k=='ghost_view_max_range' else old(k,d)
        start=n.grid.cell((2.1,2.1))
        distances,parent=n.grid.search(start)
        with patch('warehouse_explorer.time.monotonic',return_value=20.),\
                patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(101)):
            n.ghost_revisit_tick(101.,(2.1,2.1),start,distances,parent,n.inputs)
        self.assertEqual(n.publish.call_args[0][0],'GHOST_WAIT_SAFE_VIEW')
        self.assertFalse(n.ghost_vertical_view_diagnostic['flight_authorized'])
        self.assertTrue(n.ghost_vertical_view_diagnostic['views'])
        self.assertEqual(n.altitude,1.2)
        n.goal_pub.publish.assert_not_called()

    def ready(self):
        with patch('warehouse_explorer.rospy.Service'),patch('warehouse_explorer.rospy.Subscriber'),\
                patch('warehouse_explorer.rospy.Publisher'),patch('warehouse_explorer.rospy.get_param',side_effect=lambda k,d:d):
            n=Explorer()
        n.grid=ExplorationGrid([-6,-6,6,6])
        n.grid.known[:]=True;n.grid.update_obstacles([])
        n.inspected=np.zeros(n.grid.shape,bool)
        n.altitude=1.2;n.home_xy=(2.1,2.1)
        odom=Odometry();odom.pose.pose.position.x=odom.pose.pose.position.y=2.1
        odom.pose.pose.position.z=1.2
        n.inputs={k:(PointCloud2(),20.) for k in ('map','free','scan')}
        n.inputs.update(odom=(odom,20.),mavros=(PoseStamped(),20.))
        for msg,_ in n.inputs.values(): msg.header.stamp=rospy.Time(100)
        n.cloud=Mock(return_value=np.empty((0,3)));n.publish=Mock()
        n.p=lambda k,d: False if k=='require_takeoff_confirmation' else 0 if k=='growth_margin' else d
        return n

    def tick_at_100(self,n):
        with patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(100)),\
                patch('warehouse_explorer.time.monotonic',return_value=20.):
            n.tick()

    def test_final_report_uses_mapper_actionable_count_after_revisit(self):
        n=self.ready()
        n.phase='RETURNING'
        n.ghost_status={'enabled':True,'pending_voxels':4,'actionable_voxels':4}
        n.cache['suspects']=(PointCloud2(),np.zeros((7,3)))
        with patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(100)),\
                patch('warehouse_explorer.time.monotonic',return_value=20.):
            Explorer.publish(n,'RETURNING')
        report=n.report
        self.assertEqual(report['ghost_revisit']['actionable_voxels'],4)
        self.assertEqual(report['ghost_revisit']['map_status']['actionable_voxels'],4)

    def test_low_gain_moves_to_inspection_not_fake_full_coverage(self):
        n=self.ready();n.goals_reached=3
        n.progress.update(0,int(n.grid.known.sum()),0,0)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'INSPECTING')
        self.assertFalse(n.exploration_complete)

    def test_three_total_successes_need_not_all_fall_in_last_window(self):
        for goals,expected in ((2,'EXPLORING'),(3,'INSPECTING')):
            n=self.ready();n.goals_reached=goals
            n.progress.update(0,int(n.grid.known.sum()),0,goals)
            self.tick_at_100(n)
            self.assertEqual(n.phase,expected)
            self.assertFalse(n.exploration_complete)

    def test_convergence_completion_requires_surface_quality_and_no_failures(self):
        for failed,qualified,expected in ((False,1.,True),(True,1.,False),(False,.5,False)):
            n=self.ready();n.phase='INSPECTING'
            n.progress.update(0,int(n.grid.known.sum()),0,0)
            n.surface.deficits=Mock(return_value=(np.empty((0,3)),dict(observed_surface_quality_ratio=qualified)))
            if failed: n.failed_visits[(20,20)]=2
            self.tick_at_100(n)
            self.assertEqual(n.exploration_complete,expected)
            if expected: self.assertEqual(n.finish_reason,'observed_reachable_converged')

    def test_wall_budget_returns_partial_not_complete(self):
        n=self.ready();n.active_wall=0.;n.active_sim=0.
        old=n.p;n.p=lambda k,d: 10. if k=='max_mission_wall_seconds' else old(k,d)
        self.tick_at_100(n)
        self.assertEqual(n.finish_reason,'wall_budget_partial')
        self.assertFalse(n.exploration_complete)

    def test_relaxed_acceptance_returns_partial_with_residual_frontiers(self):
        n=self.ready();n.grid.known[:,50:]=False
        n.acceptance=Mock();n.acceptance.update.return_value=True
        n.return_tick=Mock()
        self.tick_at_100(n)
        self.assertEqual(n.finish_reason,'observed_map_accepted_partial')
        self.assertEqual(n.phase,'RETURNING')
        self.assertFalse(n.exploration_complete)
        n.return_tick.assert_called_once()

    def test_accepted_map_revisits_suspect_before_return(self):
        n=self.ready()
        n.grid.known[:,50:]=False
        n.acceptance=Mock();n.acceptance.update.return_value=True
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(100)
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[4.,2.1,1.2]])
                     if name in ('map','suspects') else np.empty((0,3)))
        old=n.p
        n.p=lambda k,d: True if k=='ghost_revisit_enabled' else old(k,d)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'REVISITING')
        self.assertEqual(n.publish.call_args[0][0],'REVISITING')
        self.assertFalse(n.exploration_complete)

    def test_cleared_history_gets_one_optional_view_before_return(self):
        n=self.ready()
        n.acceptance=Mock();n.acceptance.update.return_value=True
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(100)
        n.ghost_status={'enabled':True,'pending_voxels':0}
        n.ghost_status_wall=20.
        n.ghost_history={'version':1,'frame_id':'map','map_stamp':100.,'records':[
            {'position':[4.,2.1,1.2],'state':'cleared',
             'occupied_now':False,'cleared_episodes':1}]}
        n.ghost_history_wall=20.
        old=n.p
        n.p=lambda k,d: True if k=='ghost_revisit_enabled' else old(k,d)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'REVISITING')
        self.assertEqual(n.ghost_revisit_target_kind,'cleared')
        self.assertEqual(n.publish.call_args[0][0],'REVIEWING_CLEARED_HISTORY')
        arrived=n.grid.xy(n.target)
        with patch('warehouse_explorer.time.monotonic',return_value=20.):
            start=n.grid.cell(arrived)
            distances,parent=n.grid.search(start)
            n.ghost_revisit_tick(101.,arrived,start,distances,parent,n.inputs)
            self.assertEqual(len(n.ghost_cleared_view_reached),1)
            self.assertEqual(n.publish.call_args[0][0],'GHOST_CLEARED_REVIEW_OBSERVING')
            n.ghost_history['map_stamp']=102.
            n.inputs['suspects'][0].header.stamp=rospy.Time(102)
            n.ghost_revisit_tick(102.,arrived,start,distances,parent,n.inputs)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.finish_reason,'observed_map_accepted_partial')
        self.assertEqual(len(n.ghost_cleared_visits),1)

    def test_current_suspect_precedes_cleared_history(self):
        n=self.ready()
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(100)
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.ghost_history={'version':1,'frame_id':'map','map_stamp':100.,'records':[
            {'position':[4.,2.1,1.2],'state':'cleared',
             'occupied_now':False,'cleared_episodes':1}]}
        n.ghost_history_wall=20.
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[3.8,4.,1.2]])
                     if name=='suspects' else np.empty((0,3)))
        old=n.p
        n.p=lambda k,d: True if k=='ghost_revisit_enabled' else old(k,d)
        with patch('warehouse_explorer.time.monotonic',return_value=20.),\
                patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(100)):
            self.assertTrue(n.begin_revisit_or_return('observed_map_accepted_partial',100.,n.inputs))
            start=n.grid.cell((2.1,2.1))
            distances,parent=n.grid.search(start)
            n.ghost_revisit_tick(100.,(2.1,2.1),start,distances,parent,n.inputs)
        self.assertEqual(n.ghost_revisit_target_kind,'suspect')
        self.assertEqual(n.publish.call_args[0][0],'REVISITING')

    def test_exhausted_suspect_view_falls_back_without_hiding_unresolved(self):
        n=self.ready()
        n.phase='REVISITING'
        n.ghost_revisit_started=100.
        n.ghost_revisit_reason='observed_map_accepted_partial'
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(101)
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.ghost_history={'version':1,'frame_id':'map','map_stamp':101.,'records':[
            {'position':[4.,2.1,1.2],'state':'cleared',
             'occupied_now':False,'cleared_episodes':1}]}
        n.ghost_history_wall=20.
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[4.,4.,1.2]])
                     if name=='suspects' else np.empty((0,3)))
        n.ghost_revisit_visits[(5,5)]=[(2.,2.),(3.,3.)]
        start=n.grid.cell((2.1,2.1))
        distances,parent=n.grid.search(start)
        with patch('warehouse_explorer.time.monotonic',return_value=20.),\
                patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(122)):
            n.ghost_revisit_tick(101.,(2.1,2.1),start,distances,parent,n.inputs)
            self.assertEqual(n.publish.call_args[0][0],'GHOST_WAIT_SAFE_VIEW')
            self.assertEqual(n.ghost_no_view_since,101.)
            n.inputs['suspects'][0].header.stamp=rospy.Time(122)
            n.ghost_history['map_stamp']=122.
            n.ghost_revisit_tick(122.,(2.1,2.1),start,distances,parent,n.inputs)
            self.assertTrue(n.ghost_cleared_fallback)
            self.assertEqual(n.publish.call_args[0][0],'GHOST_SWITCH_TO_CLEARED_REVIEW')
            n.ghost_revisit_tick(122.,(2.1,2.1),start,distances,parent,n.inputs)
            self.assertEqual(n.ghost_revisit_target_kind,'cleared')
            arrived=n.grid.xy(n.target)
            n.ghost_revisit_tick(123.,arrived,n.grid.cell(arrived),distances,parent,n.inputs)
            n.ghost_history['map_stamp']=124.
            n.inputs['suspects'][0].header.stamp=rospy.Time(124)
            n.ghost_revisit_tick(124.,arrived,n.grid.cell(arrived),distances,parent,n.inputs)
        self.assertEqual(len(n.ghost_cleared_view_reached),1)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.finish_reason,'ghost_unresolved_partial')

    def test_zero_mapper_evidence_allows_bounded_extra_safe_view(self):
        n=self.ready()
        n.phase='REVISITING';n.ghost_revisit_started=100.
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(101)
        n.ghost_status={'enabled':True,'pending_voxels':1,'actionable_voxels':1}
        n.ghost_status_wall=20.
        n.ghost_history={'version':1,'frame_id':'map','map_stamp':101.,
            'voxel_size':.15,'records':[{'key':[1,1,1],
             'position':[4.,4.,1.2],'state':'suspected','actionable':True,
             'priority':1.,'free_evidence_frames':0,
             'post_departure_hit_frames':0}]}
        n.ghost_history_wall=20.
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[4.,4.,1.2]])
                     if name=='suspects' else np.empty((0,3)))
        n.ghost_revisit_visits[(5,5)]=[(2.,2.),(3.,3.)]
        n.ghost_view_evidence=[dict(group='(5, 5)',status='measured',
            free_evidence_delta=0,post_departure_hit_delta=0,
            resolved_voxels=0) for _ in range(2)]
        start=n.grid.cell((2.1,2.1));distances,parent=n.grid.search(start)
        with patch('warehouse_explorer.time.monotonic',return_value=20.),\
                patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(101)):
            n.ghost_revisit_tick(101.,(2.1,2.1),start,distances,parent,n.inputs)
        self.assertIsNotNone(n.target)
        self.assertEqual(n.ghost_revisit_target,(5,5))
        self.assertTrue(n.grid.safe[n.target])

    def test_arrival_is_reported_separately_from_mapper_evidence(self):
        n=self.ready()
        n.ghost_active_view=dict(group=(5,5),view=(2.,2.),selected=100.,
            arrived=101.,before={(1,1,1):(0,0,0,'suspected')})
        n.settle_until=102.
        n.ghost_history={'version':1,'frame_id':'map','map_stamp':102.,
            'records':[{'key':[1,1,1],'position':[4.,4.,1.2],
                        'state':'suspected','free_evidence_frames':2,
                        'protected_endpoint_frames':3,
                        'post_departure_hit_frames':1}]}
        n.ghost_history_wall=20.
        with patch('warehouse_explorer.time.monotonic',return_value=20.):
            n.finish_ghost_view(101.)
            self.assertEqual(n.ghost_view_evidence,[])
            n.finish_ghost_view(102.)
        self.assertEqual(n.ghost_view_evidence[0]['status'],'measured')
        self.assertEqual(n.ghost_view_evidence[0]['free_evidence_delta'],2)
        self.assertEqual(n.ghost_view_evidence[0]['protected_endpoint_delta'],3)
        self.assertEqual(n.ghost_view_evidence[0]['post_departure_hit_delta'],1)

    def test_revisit_budget_returns_partial_with_unresolved_suspect(self):
        n=self.ready()
        n.phase='REVISITING'
        n.ghost_revisit_started=0.
        n.ghost_revisit_reason='observed_map_accepted_partial'
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(100)
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[4.,2.1,1.2]])
                     if name=='suspects' else np.empty((0,3)))
        old=n.p
        n.p=lambda k,d: (True if k=='ghost_revisit_enabled' else
                         10. if k=='ghost_revisit_budget_seconds' else old(k,d))
        self.tick_at_100(n)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.finish_reason,'ghost_revisit_budget_partial')
        self.assertFalse(n.exploration_complete)

    def test_mission_reserves_time_for_known_suspect(self):
        n=self.ready()
        n.mission_started=0.
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(100)
        n.cloud=Mock(side_effect=lambda name,inputs: np.array([[4.,2.1,1.2]])
                     if name in ('map','suspects') else np.empty((0,3)))
        old=n.p
        n.p=lambda k,d: (True if k=='ghost_revisit_enabled' else
                         200. if k=='max_mission_seconds' else old(k,d))
        self.tick_at_100(n)
        self.assertEqual(n.phase,'REVISITING')
        self.assertEqual(n.ghost_revisit_reason,'mission_budget_partial')

    def test_revisit_arrival_is_not_resolution(self):
        n=self.ready()
        n.ghost_status={'enabled':True,'pending_voxels':1}
        n.ghost_status_wall=20.
        n.ghost_history={'version':1,'frame_id':'map','map_stamp':100.,'records':[]}
        n.ghost_history_wall=20.
        n.inputs['suspects']=(PointCloud2(),20.)
        n.inputs['suspects'][0].header.stamp=rospy.Time(100)
        points=np.array([[4.,2.1,1.2]])
        n.cloud=Mock(side_effect=lambda name,inputs: points if name=='suspects'
                     else np.empty((0,3)))
        old=n.p
        n.p=lambda k,d: True if k=='ghost_revisit_enabled' else old(k,d)
        n.grid.update_obstacles(points)
        start=n.grid.cell((2.1,2.1))
        distances,parent=n.grid.search(start)
        with patch('warehouse_explorer.time.monotonic',return_value=20.),\
                patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(100)):
            self.assertTrue(n.begin_revisit_or_return('observed_map_accepted_partial',100.,n.inputs))
            n.ghost_revisit_tick(100.,(2.1,2.1),start,distances,parent,n.inputs)
            self.assertIsNotNone(n.target)
            arrived=n.grid.xy(n.target)
            n.ghost_revisit_tick(101.,arrived,n.grid.cell(arrived),distances,parent,n.inputs)
            self.assertEqual(n.phase,'REVISITING')
            self.assertEqual(n.publish.call_args[0][0],'GHOST_OBSERVING')
            points=np.empty((0,3))
            n.ghost_status={'enabled':True,'pending_voxels':0}
            n.ghost_history['map_stamp']=102.
            n.inputs['suspects'][0].header.stamp=rospy.Time(102)
            distances,parent=n.grid.search(n.grid.cell(arrived))
            n.ghost_revisit_tick(102.,arrived,n.grid.cell(arrived),distances,parent,n.inputs)
            self.assertEqual(n.phase,'RETURNING')
            self.assertEqual(n.finish_reason,'observed_map_accepted_partial')

    def test_stale_inputs_reset_acceptance(self):
        n=self.ready();n.inputs={};n.acceptance=Mock()
        self.tick_at_100(n)
        n.acceptance.reset.assert_called_once()

    def test_inspection_resumes_when_new_frontier_is_visible(self):
        n=self.ready();n.phase='INSPECTING'
        n.grid.known[:,50:]=False
        n.progress.update(0,int(n.grid.known.sum()),0,0)
        n.surface.inspection_candidates=Mock(return_value=(np.zeros(n.grid.shape,bool),np.zeros(n.grid.shape)))
        self.tick_at_100(n)
        self.assertGreater(n.useful_frontier_count,0)
        self.assertEqual(n.phase,'EXPLORING')
        self.assertEqual(n.publish.call_args[0][0],'RESUME_EXPLORATION')
        self.assertFalse(n.progress.samples)
        self.assertFalse(n.exploration_complete)

    def test_unqualified_inspection_view_is_not_permanently_hidden(self):
        n=self.ready();n.phase='INSPECTING';cell=(22,24)
        mask=np.zeros(n.grid.shape,bool);mask[cell]=True
        gain=np.zeros(n.grid.shape);gain[cell]=1.
        n.surface.inspection_candidates=Mock(return_value=(mask,gain))
        n.surface.deficits=Mock(return_value=(np.array([[1.,1.,1.]]),dict(observed_surface_quality_ratio=.5)))
        n.inspection_visited=[(cell,70.)]
        self.tick_at_100(n)
        self.assertEqual(n.inspection_visited,[])
        self.assertEqual(n.target,cell)
        self.assertFalse(n.exploration_complete)

    def test_grid_capacity_returns_on_old_grid_not_unknown(self):
        n=self.ready();n.update_observed_window=Mock(return_value=False)
        with patch('warehouse_explorer.rospy.logerr_throttle'):
            self.tick_at_100(n)
        self.assertTrue(n.grid_capacity_limited)
        self.assertEqual(n.finish_reason,'map_capacity_partial')
        self.assertFalse(n.exploration_complete)

    def test_unsafe_start_without_progress_fails_explicitly(self):
        n=self.ready()
        n.unsafe_since=50.
        n.unsafe_position=(2.1,2.1)
        n.grid.update_obstacles=Mock(side_effect=lambda points: n.grid.safe.fill(False))
        n.grid.reconnect=Mock(return_value=None)
        self.tick_at_100(n)
        self.assertEqual(n.phase,'STALLED_UNSAFE')
        self.assertEqual(n.finish_reason,'unsafe_start_no_progress')
        self.assertEqual(n.publish.call_args[0][0],'STALLED_UNSAFE')
        self.assertFalse(n.exploration_complete)

    def node(self):
        n=Explorer.__new__(Explorer)
        n.p=lambda k,d:d
        n.grid=ExplorationGrid([0,0,12,10],.2,.8,.4)
        n.grid.known[:]=True
        n.grid.update_obstacles([])
        n.home_xy=(2.1,2.1)
        n.home_confirmed=False
        n.inspected=np.zeros(n.grid.shape,bool)
        n.publish=Mock()
        n.save=Mock(return_value=True)
        n.begin_return('test_partial',10.)
        return n

    def test_return_requires_continuous_dwell(self):
        n=self.node()
        start=n.grid.cell(n.home_xy)
        d,p=n.grid.search(start)
        n.return_tick(11,n.home_xy,start,d,p)
        self.assertFalse(n.home_confirmed)
        self.assertEqual(n.publish.call_args[0][0],'HOME_SETTLE')
        n.return_tick(14,(3.,2.1),n.grid.cell((3.,2.1)),d,p)
        self.assertIsNone(n.home_since)
        n.return_tick(15,n.home_xy,start,d,p)
        n.return_tick(19,n.home_xy,start,d,p)
        n.save.assert_not_called()
        n.return_tick(20,n.home_xy,start,d,p)
        self.assertTrue(n.home_confirmed)
        self.assertEqual(n.phase,'COMPLETE')

    def test_failed_save_cannot_complete(self):
        n=self.node()
        n.save.return_value=False
        start=n.grid.cell(n.home_xy)
        d,p=n.grid.search(start)
        n.return_tick(11,n.home_xy,start,d,p)
        n.return_tick(20,n.home_xy,start,d,p)
        self.assertEqual(n.phase,'RETURNING')
        self.assertEqual(n.publish.call_args[0][0],'HOME_SAVE_FAILED')

    def test_return_never_plans_through_wall(self):
        n=self.node()
        n.grid.update_obstacles([(5,y) for y in np.arange(0,10,.1)])
        position=(8.,2.1)
        start=n.grid.cell(position)
        d,p=n.grid.search(start)
        n.return_tick(11,position,start,d,p)
        self.assertEqual(n.publish.call_args[0][0],'RETURN_BLOCKED')
        self.assertFalse(n.home_confirmed)

    def test_return_timeout_is_failure(self):
        n=self.node()
        pos=(8.,2.1)
        start=n.grid.cell(pos)
        d,p=n.grid.search(start)
        n.return_tick(1211,pos,start,d,p)
        self.assertEqual(n.phase,'RETURN_FAILED')
        n.save.assert_not_called()

    def test_return_resets_old_exploration_goal(self):
        n=self.node()
        n.target=(30,30)
        n.settle_until=1000.
        n.begin_return('operator_request',15.)
        self.assertIsNone(n.target)
        self.assertEqual(n.settle_until,0.)
        self.assertEqual(n.finish_reason,'operator_request')

    def test_window_growth_shifts_index_based_mission_memory(self):
        n=self.node()
        old_cell=(20,30)
        n.target=old_cell
        n.visited=[(old_cell,1.)]
        n.blacklist=[(old_cell,2.)]
        n.failed_visits={old_cell:1}
        n.inspected[old_cell]=True
        pads=n.grid.grow_to_include([(-10.,-10.)],margin=2.,chunk=2.)
        self.assertIsNotNone(pads)
        n.shift_grid_indices(pads)
        shifted=(old_cell[0]+pads[0],old_cell[1]+pads[2])
        self.assertEqual(n.target,shifted)
        self.assertEqual(n.visited[0][0],shifted)
        self.assertEqual(n.blacklist[0][0],shifted)
        self.assertEqual(n.failed_visits[shifted],1)
        self.assertTrue(n.inspected[shifted])

    def test_explorer_window_growth_uses_pose_and_fresh_free_points(self):
        n=self.node()
        old_cell=n.grid.cell((2.,5.))
        old_xy=n.grid.xy(old_cell)
        n.target=old_cell
        n.visited=[]
        n.blacklist=[]
        n.failed_visits={}
        observed=np.array([[-1.,5.,1.]])
        self.assertTrue(n.update_observed_window((-1.,5.),observed))
        np.testing.assert_allclose(n.grid.xy(n.target),old_xy,atol=1e-12)
        self.assertGreater(n.target[1],old_cell[1])
        self.assertTrue(n.grid.known[n.grid.cell((-1.,5.))])
        self.assertEqual(n.target,n.grid.cell(old_xy))

    def test_inspection_does_not_cross_shelf(self):
        n=self.node()
        n.grid.update_obstacles([(5,y) for y in np.arange(0,10,.1)])
        n.mark_inspected((4.,5.))
        self.assertTrue(n.inspected[n.grid.cell((3.8,5.))])
        self.assertFalse(n.inspected[n.grid.cell((6.,5.))])

    @patch('warehouse_explorer.rospy.Service')
    @patch('warehouse_explorer.rospy.Subscriber')
    @patch('warehouse_explorer.rospy.Publisher')
    @patch('warehouse_explorer.rospy.get_param',side_effect=lambda k,d:d)
    @patch('warehouse_explorer.time.monotonic',return_value=20.)
    def test_normal_mission_exhaustion_returns_without_goal_limit(self,*mocks):
        n=Explorer()
        n.grid=ExplorationGrid([-6,-6,6,6],.2,.8,.4)
        n.inspected=np.zeros(n.grid.shape,bool)
        n.grid.known[:]=True
        n.home=0.
        n.altitude=1.2
        odom=Odometry()
        odom.pose.pose.position.x=odom.pose.pose.position.y=2.1
        odom.pose.pose.position.z=1.2
        n.inputs={k:(PointCloud2(),20.) for k in ('map','free','scan')}
        n.inputs.update(odom=(odom,20.),mavros=(PoseStamped(),20.))
        n.cloud=Mock(return_value=np.empty((0,3)))
        n.publish=Mock()
        n.save=Mock(return_value=True)
        n.p=lambda k,d: (0. if k=='growth_margin' else
                         1. if k=='no_frontier_seconds' else
                         False if k=='require_takeoff_confirmation' else d)
        for stamp,state in ((10,'WAIT_FRONTIER'),(12,'START_INSPECTION'),
                            (13,'WAIT_FRONTIER'),(15,'RETURNING'),
                            (16,'HOME_SETTLE'),(22,'COMPLETE')):
            for msg,_ in n.inputs.values():
                msg.header.stamp=rospy.Time(stamp)
            with patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(stamp)):
                n.tick()
            self.assertEqual(n.publish.call_args[0][0],state)
        self.assertTrue(n.home_confirmed)
        self.assertEqual(n.finish_reason,'reachable_viewpoints_exhausted')


if __name__=='__main__':
    unittest.main()
