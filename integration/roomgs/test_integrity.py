"""Critical negative checks: incomplete maps must never enter GS training."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from pipeline import validate_capture
from audit import visible,nearest_distances
from provenance import snapshot_sources,verify_sources


class Integrity(unittest.TestCase):
    def test_one_missing_sample_is_rejected_even_with_true_flag(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'result.json').write_text(json.dumps(dict(success=True,source_unchanged=True)))
            (p/'coverage.json').write_text(json.dumps(dict(passed=True,complete_fraction=1.,
                required_fraction=1.,missing_samples=1,total_samples=1000)))
            with self.assertRaises(RuntimeError):validate_capture(p)

    def test_99_percent_is_not_complete(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'result.json').write_text(json.dumps(dict(success=True,source_unchanged=True)))
            (p/'coverage.json').write_text(json.dumps(dict(passed=True,complete_fraction=.99,
                required_fraction=.99,missing_samples=0,total_samples=1000)))
            with self.assertRaises(RuntimeError):validate_capture(p)

    def test_occluded_surface_does_not_count_as_photo_coverage(self):
        p=np.array([[0.,0.,4.],[1.,0.,4.]])
        n=np.array([[0.,0.,-1.],[0.,0.,-1.]])
        k=np.array([[100,0,160],[0,100,120],[0,0,1.]])
        blocker=dict(minimum=[-.2,-.5,1.],maximum=[.2,.5,2.])
        seen=visible(p,n,np.eye(4),k,320,240,[blocker])
        self.assertFalse(seen[0]);self.assertTrue(seen[1])

    def test_source_snapshot_survives_tool_updates_and_detects_tampering(self):
        import hashlib
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source.py';source.write_text('capture version')
            digest=hashlib.sha256(source.read_bytes()).hexdigest()
            capture=dict(runtime_source_sha256={str(source):digest})
            snapshot_sources(root,capture['runtime_source_sha256'])
            source.write_text('later offline change')
            verify_sources(root,capture)
            (root/'acquisition_sources'/digest).write_text('damaged snapshot')
            with self.assertRaises(RuntimeError):verify_sources(root,capture)

    def test_approximate_miss_is_checked_against_actual_points(self):
        rng=np.random.RandomState(42)
        cloud=rng.uniform(-5,5,(5000,3))
        query=np.array([[2.45,-3.2,1.4],[0,0,0],[50,50,50]])
        cloud=np.concatenate([cloud,query[:1]+.01])
        actual=nearest_distances(cloud,query)
        exact=np.array([np.linalg.norm(cloud-p,axis=1).min() for p in query])
        self.assertTrue(np.array_equal(actual<=.20,exact<=.20))


if __name__=='__main__':unittest.main()
