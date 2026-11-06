"""Actual measured points must retain unresolved labels without adding ghosts."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pending_map_colors import colored_payload,label_points


class PendingColorTest(unittest.TestCase):
    def record(self,position,**kwargs):
        return dict(position=position,key=[1,2,3],state='suspected',**kwargs)

    def history(self,*records):
        return dict(version=1,frame_id='map',map_stamp=12.,records=list(records))

    def test_both_actionable_and_suspended_candidates_are_visible(self):
        p=[[1.,2.,3.],[4.,5.,6.],[7.,8.,9.]]
        ledger=self.history(self.record(p[0],actionable=True,positive_endpoint_frames=1),
            self.record(p[1],actionable=False,static_surface_suppressed=True))
        xyz,rgb,labels,meta=label_points(p,ledger)
        self.assertEqual(list(labels),[1,2,0])
        self.assertEqual(meta['counts']['1'],1)
        np.testing.assert_array_equal(xyz,np.asarray(p,dtype=np.float32))

    def test_no_post_departure_evidence_is_separately_marked(self):
        _,_,labels,_=label_points([[1,2,3]],self.history(self.record([1,2,3],actionable=False)))
        self.assertEqual(list(labels),[3])

    def test_rounded_voxel_boundary_uses_exact_mean_not_recomputed_key(self):
        point=[1.05,.3,1.2]  # float32 flooring can differ from the ledger key
        r=self.record(point,positive_endpoint_frames=1);r['key']=[7,2,8]
        _,_,labels,meta=label_points([point],self.history(r))
        self.assertEqual(list(labels),[1]);self.assertEqual(meta['unmatched_unresolved_records'],[])

    def test_float32_coordinate_collision_preserves_both_points_and_uncertainty(self):
        point=[1.,2.,3.]
        a=self.record(point,static_surface_suppressed=True)
        b=self.record(point,positive_endpoint_frames=1);b['key']=[2,2,3]
        xyz,_,labels,meta=label_points([point,point],self.history(a,b))
        self.assertEqual(len(xyz),2);self.assertEqual(list(labels),[1,1])
        self.assertEqual(len(meta['coordinate_collision_records']),1)

    def test_nearby_unrelated_wall_point_is_not_colored(self):
        r=self.record([1.,2.,3.],positive_endpoint_frames=1)
        _,_,labels,meta=label_points([[1.01,2.,3.]],self.history(r))
        self.assertEqual(list(labels),[0]);self.assertEqual(len(meta['unmatched_unresolved_records']),1)

    def test_cleared_or_evicted_absent_points_are_never_invented(self):
        r=self.record([4,5,6]);r.update(state='cleared',occupied_now=False)
        e=self.record([7,8,9]);e.update(state='evicted_unresolved',occupied_now=False)
        xyz,_,labels,_=label_points([[1,2,3]],self.history(r,e))
        self.assertEqual(len(xyz),1);self.assertEqual(list(labels),[0])

    def test_missing_ledger_is_not_called_clean(self):
        _,_,labels,meta=label_points([[1,2,3]],None)
        self.assertEqual(list(labels),[4]);self.assertIn('not mean certified',meta['scope'])

    def test_binary_roundtrip_preserves_xyz_and_packed_rgb(self):
        p=np.array([[1.,2.,3.],[4.,5.,6.]],dtype='<f4')
        payload,meta=colored_payload(p,self.history(self.record(p[0].tolist(),positive_endpoint_frames=1)))
        header,data=payload.split(b'DATA binary\n',1)
        self.assertIn(b'FIELDS x y z rgb label',header)
        rows=np.frombuffer(data,dtype=[('xyz','<f4',(3,)),('rgb','<u4'),('label','<u4')])
        np.testing.assert_array_equal(rows['xyz'],p)
        self.assertEqual(int(rows['rgb'][0]),(255<<16)|(55<<8)|55)
        self.assertEqual(len(rows),meta['points'])

    def test_saver_automatically_exports_companion_without_changing_xyz(self):
        import rospy
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        from save_navigation_map import write_snapshot
        from warehouse_map_audit import read_xyz_pcd
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'map.pcd'
            cloud=point_cloud2.create_cloud_xyz32(Header(stamp=rospy.Time.from_sec(12.),frame_id='map'),[[1,2,3]])
            meta=write_snapshot(cloud,p,'/map',self.history(self.record([1,2,3],positive_endpoint_frames=1)))
            self.assertTrue(Path(meta['labeled_map']).is_file())
            np.testing.assert_array_equal(read_xyz_pcd(p),np.array([[1,2,3]],dtype=np.float32))
            labelmeta=json.loads(p.with_name('map_labeled.json').read_text())
            self.assertEqual(labelmeta['counts']['1'],1)
            self.assertEqual(labelmeta['frame_id'],'map')
