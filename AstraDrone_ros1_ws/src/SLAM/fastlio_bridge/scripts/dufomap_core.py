#!/usr/bin/env python3
"""Adapter to the author's DUFOMap 1.1.1 native implementation, not a reimplementation.

Reference: Duberg et al., IEEE RA-L 2024, DOI 10.1109/LRA.2024.3387658.
https://github.com/KTH-RPL/dufomap . The native binding is the same one called
by the author's Python wrapper; calling it directly avoids numpy.typing imports
in that wrapper on ROS Noetic's older NumPy. No detector labels or scene truth.
"""
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import struct
import time

import numpy as np


def native_provenance():
    import dufomap_bind
    version = importlib.metadata.version('dufomap')
    if version != '1.1.1':
        raise ValueError('This adapter was validated with dufomap==1.1.1')
    path = Path(dufomap_bind.__file__).resolve()
    return dict(package='dufomap', version=version, native_path=str(path),
                native_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                numpy_version=np.__version__,
                upstream='https://github.com/KTH-RPL/dufomap',
                api='dufomap_bind._dufomap; same native API used by author Python wrapper')


def sensor_pose(position, quaternion_xyzw, origin_in_body):
    p, q, offset = [np.asarray(x, dtype=float) for x in (position, quaternion_xyzw, origin_in_body)]
    if p.shape != (3,) or q.shape != (4,) or offset.shape != (3,):
        raise ValueError('invalid pose/extrinsic shape')
    if not all(np.isfinite(x).all() for x in (p, q, offset)) or np.linalg.norm(q) < 1e-8:
        raise ValueError('nonfinite pose or invalid quaternion')
    q = q / np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    xyz = p + rotation @ offset
    return [float(v) for v in (*xyz, w, x, y, z)]


def atomic_json(path, data):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    os.replace(str(temporary), str(path))


def atomic_pcd(path, points):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    points = np.ascontiguousarray(points, dtype='<f4').reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('nonfinite PCD output')
    header = ('# .PCD v0.7 - DUFOMap measured XYZ, no interpolation\nVERSION 0.7\n'
              'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\n'
              'HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (len(points), len(points))
    payload = header.encode('ascii')+points.tobytes()
    tmp = path.with_name(path.name+'.tmp')
    with tmp.open('wb') as f:
        f.write(payload); f.flush(); os.fsync(f.fileno())
    os.replace(str(tmp), str(path))
    return dict(path=str(path.resolve()), points=len(points), sha256=hashlib.sha256(payload).hexdigest())


class DufoMapEngine:
    def __init__(self, resolution=.15, sensor_error=.20, pose_cells=1, threads=2,
                 max_voxels=800000, max_frames=6500, lidar_origin=(0., 0., .13)):
        if not all(math.isfinite(x) for x in (resolution, sensor_error)) or not .05 <= resolution <= .5 or sensor_error < 0:
            raise ValueError('invalid resolution or sensor error')
        if type(pose_cells) is not int or not 1 <= pose_cells <= 4 or not 1 <= threads <= 8:
            raise ValueError('invalid pose uncertainty or thread count')
        if max_voxels < 1 or max_frames < 1:
            raise ValueError('invalid capacity')
        self.provenance = native_provenance()
        import dufomap_bind
        self.engine = dufomap_bind._dufomap(resolution, sensor_error, pose_cells, threads, True, False)
        self.config = dict(resolution_m=resolution, sensor_error_m=sensor_error,
                           pose_uncertainty_cells=pose_cells, threads=threads,
                           hit_extension=True, ray_passthrough_hits=False,
                           min_range_m=.2, max_voxels=max_voxels, max_frames=max_frames,
                           lidar_origin_in_body=list(lidar_origin))
        self.voxels = {}; self.representative_distance = {}; self.frames = 0; self.last_stamp_ns = 0
        self.total_input_points = 0; self.valid_points = 0; self.timings_ms = []
        self.digest = hashlib.sha256(); self.last_pose = [0., 0., 0., 1., 0., 0., 0.]
        self._snapshot_cache = None

    def ingest(self, xyz, body_position, body_quaternion_xyzw, stamp_ns, seq):
        if seq != self.frames+1 or stamp_ns <= self.last_stamp_ns:
            raise ValueError('input gap, duplicate or nonincreasing canonical scan stamp')
        if self.frames >= self.config['max_frames']:
            raise ValueError('DUFOMap frame capacity reached; no silent truncation')
        raw = np.ascontiguousarray(xyz, dtype='<f4').reshape(-1, 3)
        pose = sensor_pose(body_position, body_quaternion_xyzw, self.config['lidar_origin_in_body'])
        valid = raw[np.isfinite(raw).all(axis=1)]
        valid = valid[np.linalg.norm(valid-np.array(pose[:3]), axis=1) > self.config['min_range_m']]
        # Preserve ONE actual measured return per 15cm output voxel. Never emit
        # voxel centers, synthesize planes, or prefilter using our detector.
        keys = np.floor(valid/self.config['resolution_m']).astype(np.int64)
        distance = np.sum((valid-(keys+.5)*self.config['resolution_m'])**2, axis=1)
        order = np.lexsort((distance, keys[:, 2], keys[:, 1], keys[:, 0]))
        ordered = keys[order]
        indexes = order[np.r_[True, np.any(np.diff(ordered, axis=0), axis=1)]] if len(order) else order
        updates = {tuple(keys[i]): (tuple(float(x) for x in valid[i]), float(distance[i]))
                   for i in indexes if distance[i] < self.representative_distance.get(tuple(keys[i]), float('inf'))}
        new_count = sum(k not in self.voxels for k in updates)
        if len(self.voxels)+new_count > self.config['max_voxels']:
            raise ValueError('DUFOMap output voxel capacity reached; no silent eviction')
        began = time.monotonic()
        self.engine.run(np.ascontiguousarray(valid, dtype=np.float32), pose, False)
        self.timings_ms.append((time.monotonic()-began)*1000.)
        for key, (point, distance) in updates.items():
            self.voxels[key] = point; self.representative_distance[key] = distance
        self.digest.update(struct.pack('<QQ', int(seq), int(stamp_ns)))
        self.digest.update(struct.pack('<Q', len(raw))); self.digest.update(raw.tobytes())
        self.digest.update(np.asarray([*body_position, *body_quaternion_xyzw], dtype='<f8').tobytes())
        self.frames += 1; self.last_stamp_ns = int(stamp_ns); self.last_pose = pose
        self.total_input_points += len(raw); self.valid_points += len(valid)
        self._snapshot_cache = None

    def snapshot(self):
        # Repeated native propagation/query at the SAME input boundary can
        # vary a few boundary labels. Freeze one classification for both ROS
        # publication and file export; a new accepted frame invalidates it.
        if self._snapshot_cache is not None:
            return self._snapshot_cache
        points = np.asarray(list(self.voxels.values()), dtype=np.float32).reshape(-1, 3)
        self.engine.oncePropagateCluster(True, False)
        labels = np.asarray(self.engine.segment(np.ascontiguousarray(points), self.last_pose, False), dtype=bool) if len(points) else np.empty(0, bool)
        if labels.shape != (len(points),):
            raise ValueError('DUFOMap native labels do not match output points')
        product = (points[~labels], points[labels], points)
        for array in product:
            array.flags.writeable = False
        self._snapshot_cache = product
        return product

    def stats(self):
        return dict(config=self.config, implementation=self.provenance, processed_frames=self.frames,
                    last_stamp_ns=self.last_stamp_ns, input_xyz_points=self.total_input_points,
                    valid_range_points=self.valid_points, observed_voxels=len(self.voxels),
                    output_representative='nearest actual measured return to voxel center; no center generation',
                    canonical_input_sha256=self.digest.hexdigest(),
                    native_integration_mean_ms=float(np.mean(self.timings_ms)) if self.timings_ms else None,
                    native_integration_p95_ms=float(np.percentile(self.timings_ms, 95)) if self.timings_ms else None,
                    native_integration_max_ms=max(self.timings_ms) if self.timings_ms else None)

    def save(self, prefix, extra=None):
        static, dynamic, observed = self.snapshot()
        if not self.frames or not len(static):
            raise ValueError('empty/incomplete DUFOMap; refuse successful export')
        outputs = {name: atomic_pcd(str(prefix)+'_dufomap_'+name+'.pcd', points)
                   for name, points in [('static_map', static), ('removed_points', dynamic), ('observed_map', observed)]}
        report = dict(self.stats(), frame_id='map', stamp=self.last_stamp_ns*1e-9,
                      outputs=outputs, **(extra or {}))
        report['scope'] = ('Author DUFOMap void-based filtering of registered scans in continuous local map. '
                           '15cm measured-point representatives; no detector labels, truth, loop correction, '
                           'or post-export cropping. Removed means DUFOMap-classified dynamic, not proven ghost truth.')
        atomic_json(str(prefix)+'_dufomap.json', report)
        return report
