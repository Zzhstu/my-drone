#!/usr/bin/env python3
"""Pairing boundaries protect against plausible but stale or displaced overlays."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_map_contract import MapState, validate_pair, geometry_disagreement, point_set_sha256


class PairContractTest(unittest.TestCase):
    def setUp(self):
        self.a = MapState(10000000000, 50, 51, 'loop_map')
        self.b = MapState(10000000000, 50, 50, 'map')

    def test_processing_lag_is_allowed_only_at_identical_processed_boundary(self):
        self.assertEqual(validate_pair(self.a, self.b, np.eye(4))['processed_frames'], 50)
        for b in [replace(self.b, processed_frames=49), replace(self.b, stamp_ns=self.b.stamp_ns+1)]:
            with self.assertRaises(ValueError): validate_pair(self.a,b,np.eye(4))

    def test_moved_global_frame_cannot_be_silently_renamed(self):
        transform = np.eye(4); transform[0,3] = .1
        with self.assertRaises(ValueError): validate_pair(self.a,self.b,transform)
        with self.assertRaises(ValueError): validate_pair(replace(self.a,frame_id='map'),self.b,np.eye(4))

    def test_nonzero_or_pending_graph_cannot_pair_with_local_dufo(self):
        for a in [replace(self.a,graph_revision=1),replace(self.a,requested_revision=1),replace(self.a,rebuilding=True)]:
            with self.assertRaises(ValueError): validate_pair(a,self.b,np.eye(4))

    def test_partial_or_unhealthy_maps_are_not_final_results(self):
        for a,b in [(replace(self.a,frozen=True),replace(self.b,frozen=True)),
                    (replace(self.a,capacity_limited=True),self.b),
                    (self.a,replace(self.b,healthy=False)),(self.a,replace(self.b,resolution=.2))]:
            with self.assertRaises(ValueError): validate_pair(a,b,np.eye(4))
        report = validate_pair(replace(self.a,input_frames=50,frozen=True),replace(self.b,frozen=True),np.eye(4))
        self.assertTrue(report['frozen']); self.assertFalse(report['planner_input'])

    def test_sampling_offset_within_voxel_does_not_become_disagreement(self):
        a=np.array([[0,0,0],[1,0,0],[2,0,0]],np.float32)
        b=np.array([[.04,0,0],[1.04,0,0],[3,0,0]],np.float32)
        ca,ba,report=geometry_disagreement(a,b)
        np.testing.assert_array_equal(ca,[[2,0,0]]);np.testing.assert_array_equal(ba,[[3,0,0]])
        self.assertTrue(report['computed'])

    def test_empty_background_and_capacity_are_explicit(self):
        a=np.array([[1,0,0]],np.float32);b=np.empty((0,3),np.float32)
        ca,ba,report=geometry_disagreement(a,b)
        self.assertEqual((len(ca),len(ba)),(1,0))
        _,_,report=geometry_disagreement(a,b,max_points=0)
        self.assertFalse(report['computed'])

    def test_export_hash_ignores_order_but_detects_missing_or_changed_points(self):
        a=np.array([[1,0,0],[2,0,0],[0,1,0]],np.float32)
        self.assertEqual(point_set_sha256(a),point_set_sha256(a[::-1]))
        self.assertNotEqual(point_set_sha256(a),point_set_sha256(a[:2]))
        b=a.copy();b[0,0]+=.00001
        self.assertNotEqual(point_set_sha256(a),point_set_sha256(b))

    def test_spatial_join_matches_brute_force_in_dense_negative_cells(self):
        rng=np.random.RandomState(14)
        a=rng.uniform(-.4,.4,(100,3)).astype(np.float32)
        b=rng.uniform(-.4,.4,(80,3)).astype(np.float32)
        ca,ba,_=geometry_disagreement(a,b,tolerance=.15)
        d=np.sum((a[:,None,:].astype(float)-b[None,:,:].astype(float))**2,axis=2)
        np.testing.assert_array_equal(ca,a[d.min(axis=1)>.15**2])
        np.testing.assert_array_equal(ba,b[d.min(axis=0)>.15**2])


if __name__ == '__main__': unittest.main()
