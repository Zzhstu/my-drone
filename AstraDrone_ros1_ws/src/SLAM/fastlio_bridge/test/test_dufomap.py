import importlib.util
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dufomap_core import sensor_pose, DufoMapEngine


class DufoGeometryTest(unittest.TestCase):
    def test_rotated_sensor_origin_and_quaternion_order(self):
        q = [0., 0., np.sqrt(.5), np.sqrt(.5)]
        pose = sensor_pose([1., 2., 3.], q, [.2, 0., .13])
        np.testing.assert_allclose(pose[:3], [1., 2.2, 3.13], atol=1e-10)
        np.testing.assert_allclose(pose[3:], [q[3], *q[:3]])

    def test_invalid_geometry_rejected(self):
        for q in [[0., 0., 0., 0.], [0., 0., float('nan'), 1.]]:
            with self.assertRaises(ValueError):
                sensor_pose([0., 0., 0.], q, [0., 0., .13])


@unittest.skipUnless(importlib.util.find_spec('dufomap_bind'), 'optional native DUFOMap; run this file with .venvs/dufomap/bin/python')
class DufoNativeTest(unittest.TestCase):
    def test_export_and_publication_share_one_frozen_classification(self):
        import tempfile
        from warehouse_map_audit import read_xyz_pcd
        e=self.engine();self.ingest(e,self.wall(),1)
        native=e.engine
        class CountingNative:
            def __init__(self):self.queries=0
            def oncePropagateCluster(self,*a):return native.oncePropagateCluster(*a)
            def segment(self,*a):self.queries+=1;return native.segment(*a)
            def run(self,*a):return native.run(*a)
        wrapped=CountingNative();e.engine=wrapped
        published=e.snapshot()[0].copy()
        with tempfile.TemporaryDirectory() as temp:
            report=e.save(Path(temp)/'frozen')
            saved=read_xyz_pcd(report['outputs']['static_map']['path'])
            np.testing.assert_array_equal(saved,published)
        np.testing.assert_array_equal(e.snapshot()[0],published)
        self.assertEqual(wrapped.queries,1)
        self.ingest(e,self.wall(),2);e.snapshot()
        self.assertEqual(wrapped.queries,2)

    def engine(self, **kw):
        return DufoMapEngine(lidar_origin=(0., 0., 0.), **kw)

    def wall(self):
        y, z = np.meshgrid(np.arange(-2., 2.01, .04), np.arange(-2., 2.01, .04))
        return np.c_[np.full(y.size, 3.), y.ravel(), z.ravel()].astype(np.float32)

    def ingest(self, engine, points, seq):
        engine.ingest(points, [0., 0., 0.], [0., 0., 0., 1.], seq*100000000, seq)

    def test_dense_void_but_wall_and_unknown_preserved(self):
        e = self.engine(); self.ingest(e, self.wall(), 1)
        e.snapshot()
        q = np.array([[1., 0., 0.], [3., 0., 0.], [10., 10., 10.]], np.float32)
        self.assertEqual(list(e.engine.segment(q, e.last_pose, False)), [True, False, False])

    def test_one_sparse_ray_does_not_establish_void(self):
        e = self.engine(); self.ingest(e, np.array([[3., 0., 0.]], np.float32), 1); e.snapshot()
        self.assertEqual(list(e.engine.segment(np.array([[1., 0., 0.]], np.float32), e.last_pose, False)), [False])

    def test_historical_occluder_removed_after_measured_free_observation(self):
        e = self.engine(); wall = self.wall()
        # Opaque first-frame obstacle: do not provide impossible through-object rays.
        behind = (np.abs(wall[:, 1]/3.) < .21) & (np.abs(wall[:, 2]/3.) < .31)
        y, z = np.meshgrid(np.arange(-.2, .201, .04), np.arange(-.3, .301, .04))
        actor = np.c_[np.ones(y.size), y.ravel(), z.ravel()].astype(np.float32)
        self.ingest(e, np.vstack([wall[~behind], actor]), 1)
        before, removed, _ = e.snapshot()
        self.assertGreater(int((before[:, 0] < 1.1).sum()), 0)
        self.assertEqual(int((removed[:, 0] < 1.1).sum()), 0)
        self.ingest(e, wall, 2)
        after, removed, observed = e.snapshot()
        self.assertEqual(int((after[:, 0] < 1.1).sum()), 0)
        self.assertGreater(len(removed), 0)
        self.assertGreater(int((after[:, 0] > 2.9).sum()), 0)
        raw = {tuple(p) for p in np.vstack([wall, actor])}
        self.assertTrue(all(tuple(p) in raw for p in observed))

    def test_duplicate_gap_capacity_and_snapshot_are_not_silent(self):
        e = self.engine(max_frames=1); self.ingest(e, self.wall(), 1)
        digest = e.stats()['canonical_input_sha256']; first = e.snapshot()
        for seq in [1, 2, 3]:
            with self.assertRaises(ValueError):
                self.ingest(e, self.wall(), seq)
        self.assertEqual(e.frames, 1); self.assertEqual(e.stats()['canonical_input_sha256'], digest)
        for a, b in zip(first, e.snapshot()):
            np.testing.assert_array_equal(a, b)

    def test_output_uses_more_central_measured_return_without_generating_center(self):
        e = self.engine()
        first = np.array([[3., .001, .001]], np.float32)
        central = np.array([[3., .074, .074]], np.float32)
        self.ingest(e, first, 1); self.ingest(e, central, 2)
        np.testing.assert_array_equal(e.snapshot()[2], central)


if __name__ == '__main__':
    unittest.main()
