#!/usr/bin/env python3
"""Coordinate/epoch/mission regressions, no ROS master required."""
import copy
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
import numpy as np
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2
from fastlio_bridge.msg import LoopMapSnapshot
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from loop_coordinates import PlanningSnapshot,transform_message,transform_pose
from warehouse_explorer import Explorer
from flight_budget import FlightBudget
from save_navigation_map import MapSaver


def snapshot(revision=0,shift=0.,yaw=0.):
    message=LoopMapSnapshot();message.header=Header(stamp=rospy.Time(100),frame_id='loop_map')
    message.revision=revision;message.graph_revision=revision;message.requested_graph_revision=revision
    message.input_frames=message.processed_frames=10
    for name in ('map','free','suspects'):
        setattr(message,name,point_cloud2.create_cloud_xyz32(message.header,[(1,2,1.2)]))
    message.ghost_status.data=json.dumps(dict(enabled=True,pending_voxels=0))
    message.ghost_history.data=json.dumps(dict(version=1,frame_id='loop_map',map_stamp=100.,records=[]))
    t=message.global_from_local;t.header=message.header;t.child_frame_id='map'
    t.transform.translation.x=shift;t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2)
    message.graph.revision=revision;message.graph.raw.header.frame_id='map'
    message.graph.optimized.header.frame_id='loop_map'
    if revision:
        for stamp in (1,99):
            p=PoseStamped(header=Header(stamp=rospy.Time(stamp),frame_id='map'))
            p.pose.orientation.w=1
            fixed=copy.deepcopy(p);fixed.header.frame_id='loop_map';fixed.pose.position.x=shift
            fixed.pose.orientation=t.transform.rotation
            message.graph.raw.poses.append(p);message.graph.optimized.poses.append(fixed)
    return message


def node():
    with patch('warehouse_explorer.rospy.Service'),patch('warehouse_explorer.rospy.Subscriber'),\
        patch('warehouse_explorer.rospy.Publisher'),patch('warehouse_explorer.rospy.get_param',side_effect=lambda k,d:d):
        n=Explorer()
    n.p=lambda k,d:d
    n.online_loop=True;n.world_frame='loop_map';return n


def inputs():
    odom=Odometry(header=Header(stamp=rospy.Time(100),frame_id='map'))
    odom.pose.pose.orientation.w=1;odom.pose.pose.position.x=5;odom.pose.pose.position.z=1.2
    scan=point_cloud2.create_cloud_xyz32(odom.header,[(6,0,1.2)])
    return dict(odom=(odom,20.),scan=(scan,20.))


class OnlineLoopTest(unittest.TestCase):
    def test_zero_revision_retains_xyz_and_original_messages(self):
        n=node();raw=inputs();out=n.prepare_loop_inputs(raw.copy(),(snapshot(),20.),20.,100.)
        self.assertEqual(out['scan'][0].data,raw['scan'][0].data)
        self.assertEqual(out['odom'][0].pose,raw['odom'][0].pose)
        self.assertEqual(raw['odom'][0].header.frame_id,'map')
        self.assertEqual(out['odom'][0].header.frame_id,'loop_map')

    def test_global_pose_roundtrip_does_not_move_local_control_goal(self):
        n=node();raw=inputs();out=n.prepare_loop_inputs(raw.copy(),(snapshot(1,-.4,.3),20.),20.,100.)
        global_pose=out['odom'][0].pose.pose;n.altitude=global_pose.position.z
        with patch('warehouse_explorer.rospy.Time.now',return_value=rospy.Time(100)):
            goal=n.control_pose((global_pose.position.x,global_pose.position.y))
        self.assertEqual(goal.header.frame_id,'map')
        np.testing.assert_allclose([goal.pose.position.x,goal.pose.position.y,goal.pose.position.z],[5,0,1.2],atol=1e-12)

    def test_graph_switch_clears_indices_and_preserves_return_clock(self):
        n=node();raw=inputs();n.prepare_loop_inputs(raw.copy(),(snapshot(),20.),20.,100.)
        n.phase='RETURNING';n.return_started=80.;n.mission_started=10.;n.return_requested=True
        n.target=(3,4);n.grid.known[:]=True;n.flight_budget=FlightBudget()
        n.flight_budget.observe_arm(False,1.);n.flight_budget.observe_arm(True,2.)
        origin=n.flight_budget.origin
        n.prepare_loop_inputs(raw.copy(),(snapshot(1,-.4),20.),20.,100.)
        self.assertIsNone(n.target);self.assertFalse(n.grid.known.any())
        self.assertEqual(n.phase,'RETURNING');self.assertEqual(n.return_started,80.)
        self.assertEqual(n.mission_started,10.);self.assertTrue(n.return_requested)
        self.assertEqual(n.flight_budget.origin,origin)

    def test_home_uses_historical_correction_instead_of_current_transform(self):
        n=node();raw=inputs();raw['odom'][0].header.stamp=rospy.Time(1)
        n.prepare_loop_inputs(raw.copy(),(snapshot(),20.),20.,100.)
        m=snapshot(1,-.4);m.graph.optimized.poses[0].pose.position.x=0
        raw=inputs();n.prepare_loop_inputs(raw,(m,20.),20.,100.)
        np.testing.assert_allclose(n.home_xy,[5,0],atol=1e-12)

    def test_mixed_ghost_epoch_rejected(self):
        m=snapshot();m.ghost_history.data=m.ghost_history.data.replace('100.0','99.0')
        with self.assertRaisesRegex(ValueError,'ledger'):PlanningSnapshot(m)

    def test_mixed_cloud_stamp_rejected(self):
        m=snapshot();m.free=copy.deepcopy(m.free);m.free.header.stamp=rospy.Time(99)
        with self.assertRaisesRegex(ValueError,'mixed'):PlanningSnapshot(m)

    def test_unsupported_tilt_rejected(self):
        m=snapshot();m.global_from_local.transform.rotation.x=.2
        with self.assertRaisesRegex(ValueError,'gravity'):PlanningSnapshot(m)

    def test_stale_snapshot_holds(self):
        with self.assertRaisesRegex(ValueError,'stale'):
            node().prepare_loop_inputs(inputs(),(snapshot(),20.),25.,100.)

    def test_old_revision_cannot_replace_committed_map(self):
        n=node();n.prepare_loop_inputs(inputs(),(snapshot(1,-.4),20.),20.,100.)
        with self.assertRaisesRegex(ValueError,'backwards'):
            n.prepare_loop_inputs(inputs(),(snapshot(),20.),20.,100.)

    def test_capacity_limit_requests_return_instead_of_fake_completion(self):
        n=node();m=snapshot();m.capacity_limited=True
        n.prepare_loop_inputs(inputs(),(m,20.),20.,100.)
        self.assertEqual(n.phase,'RETURNING');self.assertEqual(n.finish_reason,'loop_map_capacity_partial')
        self.assertTrue(n.grid_capacity_limited)

    def test_input_cache_cannot_mutate_or_reuse_previous_scan(self):
        n=node();m=snapshot(1,-.4);a=inputs()
        first=n.prepare_loop_inputs(a.copy(),(m,20.),20.,100.)['scan'][0]
        b=inputs();b['scan'][0].data=point_cloud2.create_cloud_xyz32(b['scan'][0].header,[(9,0,1.2)]).data
        second=n.prepare_loop_inputs(b,(m,20.),20.,100.)['scan'][0]
        self.assertNotEqual(first.data,second.data)
        self.assertAlmostEqual(list(point_cloud2.read_points(a['scan'][0]))[0][0],6)

    def test_atomic_export_uses_matching_map_and_ledger(self):
        n=MapSaver.__new__(MapSaver);n.lock=threading.Lock();m=snapshot(1,-.4)
        n.receive_loop_snapshot(m)
        self.assertIs(n.cloud,m.map);self.assertEqual(n.ghost_history['frame_id'],'loop_map')
        self.assertEqual(n.loop_mapping['revision'],1)

    def test_final_export_rejects_snapshot_received_before_freeze_acknowledgement(self):
        n=MapSaver.__new__(MapSaver);n.lock=threading.Lock();n.save_lock=threading.Lock()
        n.receive_loop_snapshot(snapshot())
        result=n.save(None,require_frozen=True)
        self.assertFalse(result.success)
        self.assertIn('冻结',result.message)


if __name__=='__main__':unittest.main()
