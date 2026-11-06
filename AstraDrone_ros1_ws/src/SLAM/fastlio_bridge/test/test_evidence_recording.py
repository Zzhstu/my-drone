import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from evidence_recording import recording_policy, RateGate, write_raw, message_bytes
from experiment_evidence import EVIDENCE_TOPICS, prune_previous_recordings, sha256_file


class RecordingTest(unittest.TestCase):
    def test_compact_preserves_mapper_inputs_and_full_preserves_raw(self):
        p=recording_policy(EVIDENCE_TOPICS)
        for topic in ('/uav1/fastlio/registered_scan','/uav1/fastlio/odom','/uav1/semantic_dynamic_objects'):
            self.assertIn(topic,p['topics']);self.assertNotIn(topic,p['max_hz'])
        self.assertNotIn('/livox/lidar',p['topics'])
        self.assertEqual(p['max_hz']['/gazebo/model_states'],30.)
        self.assertEqual(p['max_hz']['/uav1/loop_slam/snapshot'],.05)
        full=recording_policy(EVIDENCE_TOPICS,'full')
        self.assertEqual(full['topics'],list(EVIDENCE_TOPICS));self.assertEqual(full['max_hz'],{})

    def test_one_khz_truth_becomes_30hz_and_clock_reset_recovers(self):
        g=RateGate({'truth':30.,'map':.05})
        self.assertLessEqual(sum(g.accept('truth',i/1000.) for i in range(10000)),300)
        self.assertTrue(g.accept('truth',.01))
        self.assertTrue(g.accept('map',0.));self.assertFalse(g.accept('map',19.999));self.assertTrue(g.accept('map',20.))
        self.assertTrue(all(g.accept('scan',0.) for _ in range(100)))

    def test_invalid_timing_parameters_rejected(self):
        for kw in ({'truth_hz':0.},{'map_hz':0.},{'map_hz':float('nan')},{'profile':'unknown'}):
            with self.assertRaises(ValueError):recording_policy(EVIDENCE_TOPICS,**kw)

    def test_raw_writer_retains_real_ros_type_bytes_and_header(self):
        import rosbag,rospy
        from std_msgs.msg import String
        msg=String(data='actual payload');buffer=io.BytesIO();msg.serialize(buffer)
        header=dict(type=msg._type,md5sum=msg._md5sum,message_definition=msg._full_text,callerid='/source')
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'sample.bag'
            with rosbag.Bag(str(p),'w') as b:write_raw(b,'/original',buffer.getvalue(),header,rospy.Time(3),{})
            with rosbag.Bag(str(p)) as b:
                topic,value,t=next(b.read_messages());self.assertEqual(topic,'/original');self.assertEqual(value.data,msg.data);self.assertEqual(t,rospy.Time(3))

    def test_typed_clock_from_rospy_shared_subscription_is_preserved(self):
        import rospy
        from rosgraph_msgs.msg import Clock
        msg=Clock(clock=rospy.Time(7,15));payload,header=message_bytes(msg)
        restored=Clock();restored.deserialize(payload)
        self.assertEqual(restored.clock,msg.clock);self.assertEqual(header['type'],'rosgraph_msgs/Clock')

    def make_current(self,folder,name='warehouse_half_joint_channel_20261027_160000'):
        prefix=folder/name;p=Path(str(prefix)+'_evidence.bag');p.write_bytes(b'new verified bag')
        manifest=dict(bag_finalized=True,recording_integrity_valid=True,source_unchanged_during_run=True,bag_sha256=sha256_file(p))
        Path(str(prefix)+'_evidence_manifest.json').write_text(json.dumps(manifest));return prefix

    def test_retention_only_removes_older_bags_preserves_results(self):
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)/'half_warehouse_joint_channel_runs';folder.mkdir();current=self.make_current(folder)
            old=folder/'warehouse_half_joint_simple_20261024_154217_evidence.bag';old.write_bytes(b'old')
            result=folder/'warehouse_half_joint_simple_20261024_154217_result.json';result.write_text('keep')
            unrelated=folder/'unrelated.bag';unrelated.write_bytes(b'keep')
            r=prune_previous_recordings(current)
            self.assertEqual(len(r['deleted']),1);self.assertFalse(old.exists());self.assertTrue(result.exists());self.assertTrue(unrelated.exists())

    def test_failed_or_modified_new_recording_never_prunes_previous(self):
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)/'half_warehouse_joint_channel_runs';folder.mkdir();current=self.make_current(folder)
            old=folder/'warehouse_half_joint_channel_20261026_193012_evidence.bag';old.write_bytes(b'old')
            Path(str(current)+'_evidence.bag').write_bytes(b'changed')
            self.assertEqual(prune_previous_recordings(current)['skipped'],'new_recording_hash_mismatch');self.assertTrue(old.exists())

    def test_retention_uses_timestamp_not_channel_label(self):
        with tempfile.TemporaryDirectory() as td:
            folder=Path(td)/'half_warehouse_joint_channel_runs';folder.mkdir();current=self.make_current(folder)
            older=folder/'warehouse_half_joint_simple_20261024_154217_evidence.bag';older.write_bytes(b'old')
            newer=folder/'warehouse_half_joint_channel_20261027_145711_evidence.bag';newer.write_bytes(b'newer')
            prune_previous_recordings(current,2);self.assertTrue(newer.exists());self.assertFalse(older.exists())


if __name__=='__main__':unittest.main()
