#!/usr/bin/env python3
"""The saved ghost ledger must match its exact XYZ map acquisition."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from save_navigation_map import write_snapshot


class GhostHistoryExportTest(unittest.TestCase):
    def test_sidecar_stamp_and_hash(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'map.pcd'
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=rospy.Time.from_sec(12.),frame_id='map'),
                [(1.,2.,3.)])
            ledger = {'version':1,'frame_id':'map','map_stamp':12.,
                      'records':[{'key':[6,13,20],'state':'cleared',
                                  'cleared_episodes':1}]}
            meta = write_snapshot(cloud,path,'/uav1/fastlio/cloud_map',ledger)
            sidecar = path.with_name('map_ghost_history.json')
            self.assertEqual(json.loads(sidecar.read_text()),ledger)
            self.assertEqual(meta['ghost_history_sha256'],
                             hashlib.sha256(sidecar.read_bytes()).hexdigest())
            original = path.read_bytes()
            with self.assertRaises(ValueError):
                write_snapshot(cloud,path,'/uav1/fastlio/cloud_map',
                               dict(ledger,map_stamp=20.))
            self.assertEqual(path.read_bytes(),original)


if __name__=='__main__':
    unittest.main()
