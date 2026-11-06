#!/usr/bin/env python3
"""Validate real closed/corrupt bags and provenance, without a ROS master."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

import rosbag
import rospy
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from experiment_evidence import finish_manifest, source_fingerprint, sha256_file, run_with_cleanup


class EvidenceTest(unittest.TestCase):
    def test_launcher_exit_before_native_bag_rename_is_not_false_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)/'run'
            final=Path(str(prefix)+'_evidence.bag')
            active=Path(str(final)+'.active')
            with rosbag.Bag(str(active),'w') as bag:
                bag.write('/clock',Clock(clock=rospy.Time(1)),t=rospy.Time(1))
            def finish_native():
                time.sleep(.10)
                active.rename(final)
            writer=threading.Thread(target=finish_native)
            writer.start()
            try:
                result=finish_manifest(prefix,dict(bag_requested=True,bag_error='initial failure'),0)
            finally:writer.join()
            self.assertTrue(result['bag_finalized'])
            self.assertNotIn('bag_error',result)
            self.assertFalse(result['replay_inputs_available'])

    def test_finalized_real_bag_requires_every_replay_input(self):
        from fastlio_bridge.msg import DynamicObjectArray
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)/'run'
            bag_path=Path(str(prefix)+'_evidence.bag')
            with rosbag.Bag(str(bag_path),'w') as bag:
                bag.write('/clock',Clock(clock=rospy.Time(1)),t=rospy.Time(1))
                bag.write('/uav1/fastlio/odom',Odometry(),t=rospy.Time(1))
                bag.write('/uav1/fastlio/registered_scan',PointCloud2(),t=rospy.Time(1))
                bag.write('/uav1/semantic_dynamic_objects',DynamicObjectArray(),t=rospy.Time(1))
            result=finish_manifest(prefix,dict(bag_requested=True),0)
            self.assertTrue(result['replay_inputs_available'])
            self.assertTrue(result['bag_finalized'])
            self.assertEqual(result['bag_sha256'],sha256_file(bag_path))
            self.assertEqual(result['message_count'],4)
            self.assertEqual(json.loads(Path(str(prefix)+'_evidence_manifest.json').read_text()),result)

    def test_only_pose_is_not_replayable(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)/'run'
            with rosbag.Bag(str(prefix)+'_evidence.bag','w') as bag:
                bag.write('/uav1/fastlio/odom',Odometry(),t=rospy.Time(1))
            result=finish_manifest(prefix,dict(bag_requested=True),0)
            self.assertTrue(result['bag_finalized'])
            self.assertFalse(result['replay_inputs_available'])

    def test_compact_queue_overflow_cannot_claim_integrity(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)/'run'
            with rosbag.Bag(str(prefix)+'_evidence.bag','w') as bag:
                bag.write('/clock',Clock(clock=rospy.Time(1)),t=rospy.Time(1))
            stats=dict(finalized=True,queue_overflow={'/clock':1},write_errors=[])
            Path(str(prefix)+'_evidence_recording.json').write_text(json.dumps(stats))
            result=finish_manifest(prefix,dict(bag_requested=True,recording_policy={'profile':'compact'}),0)
            self.assertFalse(result['recording_integrity_valid']);self.assertIn('bag_error',result)

    def test_scan_and_pose_without_objects_cannot_replay_dynamic_mapper(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix=Path(directory)/'run'
            with rosbag.Bag(str(prefix)+'_evidence.bag','w') as bag:
                bag.write('/clock',Clock(clock=rospy.Time(1)),t=rospy.Time(1))
                bag.write('/uav1/fastlio/odom',Odometry(),t=rospy.Time(1))
                bag.write('/uav1/fastlio/registered_scan',PointCloud2(),t=rospy.Time(1))
            result=finish_manifest(prefix,dict(bag_requested=True),0)
            self.assertTrue(result['base_scan_odom_available'])
            self.assertFalse(result['ghost_replay_inputs_available'])
            self.assertFalse(result['replay_inputs_available'])

    def test_metadata_failure_still_cleans_owned_processes(self):
        class Runner:
            stopped=False
            def stop(self):self.stopped=True
        runner=Runner()
        def finalize():raise OSError('metadata disk failure')
        with self.assertRaises(OSError):run_with_cleanup(runner,finalize)
        self.assertTrue(runner.stopped)

    def test_corrupt_or_failed_recorder_never_claims_complete(self):
        for exit_code in (0,1):
            with self.subTest(exit_code=exit_code),tempfile.TemporaryDirectory() as directory:
                prefix=Path(directory)/'run'
                Path(str(prefix)+'_evidence.bag').write_bytes(b'not a rosbag')
                result=finish_manifest(prefix,dict(bag_requested=True),exit_code)
                self.assertIn('bag_error',result)
                self.assertFalse(result.get('bag_finalized',False))
                self.assertFalse(result.get('replay_inputs_available',False))

    def test_fingerprint_changes_with_source_and_ignores_pyc(self):
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory)/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'
            folder.mkdir(parents=True)
            source=folder/'mapper.py';source.write_text('first')
            first=source_fingerprint(directory)
            (folder/'mapper.pyc').write_bytes(b'cache')
            self.assertEqual(first,source_fingerprint(directory))
            source.write_text('second')
            self.assertNotEqual(first['sha256'],source_fingerprint(directory)['sha256'])


if __name__=='__main__': unittest.main()
