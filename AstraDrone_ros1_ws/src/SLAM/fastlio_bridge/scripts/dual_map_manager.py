#!/usr/bin/env python3
"""Pair actual independently maintained maps at explicit canonical checkpoints.

No planner subscription or occupancy modification. Nonzero graph corrections
are refused until the DUFOMap history-rebuild path is implemented.
"""
import copy
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from fastlio_bridge.msg import LoopMapSnapshot, DufoMapSnapshot, DualMapSnapshot

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dual_map_contract import MapState, validate_pair, geometry_disagreement, point_set_sha256
from dufomap_adapter import scan_xyz, cloud_message
from dufomap_core import atomic_json, atomic_pcd
from loop_coordinates import from_transform
from warehouse_map_audit import read_xyz_pcd


class DualMapManager:
    def __init__(self):
        self.prefix = rospy.get_param('~output')
        self.stride = int(rospy.get_param('~checkpoint_stride_frames', 50))
        if not 1 <= self.stride <= 1000:
            raise ValueError('invalid dual map checkpoint stride')
        self.resolution = float(rospy.get_param('/static_map_builder/voxel_size', .15))
        self.condition = threading.Condition(); self.current = {}; self.background = {}
        self.output_lock = threading.Lock(); self.save_lock = threading.Lock(); self.unsupported_graph_seen = False
        self.pending = None; self.latest = None; self.last_identity = None
        self.pair_revision = 0; self.paired_boundaries = set(); self.reason = 'waiting_for_sources'
        self.last_source_wall = time.monotonic(); self.last_pair_wall = 0.; self.last_status = 0.; self.generation = 0
        self.pair_pub = rospy.Publisher('/dual_map/snapshot', DualMapSnapshot, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/dual_map/status', String, queue_size=1, latch=True)
        self.current_pub = rospy.Publisher('/dual_map/current_map', PointCloud2, queue_size=1, latch=True)
        self.background_pub = rospy.Publisher('/dual_map/background_map', PointCloud2, queue_size=1, latch=True)
        self.current_only_pub = rospy.Publisher('/dual_map/current_only', PointCloud2, queue_size=1, latch=True)
        self.background_only_pub = rospy.Publisher('/dual_map/background_only', PointCloud2, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/dual_map/markers', MarkerArray, queue_size=1, latch=True)
        self.current_sub = rospy.Subscriber('/uav1/loop_slam/snapshot', LoopMapSnapshot, self.receive_current, queue_size=2, buff_size=32000000)
        self.background_sub = rospy.Subscriber('/dufomap/snapshot', DufoMapSnapshot, self.receive_background, queue_size=2, buff_size=16000000)
        self.service = rospy.Service('/demo/dual_map/save_maps', Trigger, self.save)
        self.thread = threading.Thread(target=self.process, daemon=True); self.thread.start()
        self.publish_status()

    def publish_status(self):
        with self.condition:
            pair = self.latest
            report = dict(ready=True, paired=pair is not None and not self.reason,
                          reason=self.reason, paired_checkpoints=len(self.paired_boundaries),
                          cached_current=len(self.current), cached_background=len(self.background),
                          planner_input=False, graph_revision_zero_only=True)
            if pair is not None:
                report.update(pair['metadata'])
        with self.output_lock:
            atomic_json(self.prefix+'_dual_map_status.json', report)
            self.status_pub.publish(String(data=json.dumps(report)))
        marker = Marker(); marker.header.frame_id = 'loop_map'; marker.header.stamp = rospy.Time.now()
        marker.ns = 'dual_map_state'; marker.id = 0; marker.type = Marker.TEXT_VIEW_FACING; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.; marker.pose.position.z = 3.; marker.scale.z = .45
        marker.color.a = 1.; marker.color.g = 1. if report['paired'] else .25; marker.color.r = .2 if report['paired'] else 1.
        marker.text = ('DUAL MAP paired: %d frames\nGeometry differences are NOT ghost labels' % report.get('processed_frames', 0)
                       if report['paired'] else 'DUAL MAP waiting/invalid: '+report['reason'])
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def reject(self, reason):
        with self.condition:
            self.reason = str(reason); self.latest = None; self.pending = None
            self.generation += 1
            self.current.clear(); self.background.clear(); self.condition.notify_all()
        packet = DualMapSnapshot(); packet.header.frame_id = 'loop_map'; packet.header.stamp = rospy.Time.now()
        packet.valid = False; packet.invalid_reason = str(reason)
        self.pair_pub.publish(packet)
        self.publish_status()

    def add(self, source, key, packet):
        with self.condition:
            if self.unsupported_graph_seen:
                return  # A delayed revision-zero callback cannot undo graph invalidation.
            self.last_source_wall = time.monotonic()
            cache = self.current if source == 'current' else self.background
            cache[key] = packet
            for old in sorted(cache)[:-4]:
                del cache[old]
            if key in self.current and key in self.background:
                a, b = self.current[key], self.background[key]
                identity = (key, bool(a.frozen and b.frozen))
                if identity != self.last_identity:
                    self.pending = (a, b, identity, self.generation); self.condition.notify_all()

    def receive_current(self, msg):
        if msg.graph_revision or msg.requested_graph_revision:
            with self.condition:
                self.unsupported_graph_seen = True
        if msg.graph_revision or msg.requested_graph_revision or msg.rebuilding or msg.capacity_limited:
            self.reject('original map rebuilding/capacity-limited or unsupported nonzero graph revision'); return
        if msg.processed_frames == 1 or msg.processed_frames % self.stride == 0 or msg.frozen:
            self.add('current', (msg.graph_revision, msg.processed_frames, msg.header.stamp.to_nsec()), msg)

    def receive_background(self, msg):
        if msg.graph_revision:
            with self.condition:
                self.unsupported_graph_seen = True
        if not msg.healthy or msg.graph_revision:
            self.reject('unhealthy DUFOMap or unsupported graph revision'); return
        self.add('background', (msg.graph_revision, msg.processed_frames, msg.header.stamp.to_nsec()), msg)

    def build_pair(self, a, b):
        if a.map.header.stamp != a.header.stamp or a.map.header.frame_id != a.header.frame_id:
            raise ValueError('original map/header mismatch')
        if b.map.header.stamp != b.header.stamp or b.map.header.frame_id != b.header.frame_id:
            raise ValueError('DUFOMap/header mismatch')
        if (a.global_from_local.header.frame_id != a.header.frame_id or
                a.global_from_local.child_frame_id != b.header.frame_id or
                a.global_from_local.header.stamp != a.header.stamp):
            raise ValueError('source transform/header mismatch')
        ledger = json.loads(a.ghost_history.data)
        if ledger.get('frame_id') != a.header.frame_id or abs(ledger.get('map_stamp', -1)-a.header.stamp.to_sec()) > 1e-6:
            raise ValueError('original ledger/cutoff mismatch')
        bs = json.loads(b.status.data)
        ca = MapState(a.header.stamp.to_nsec(), a.processed_frames, a.input_frames, a.header.frame_id,
                      a.graph_revision, a.requested_graph_revision, self.resolution,
                      rebuilding=a.rebuilding, capacity_limited=a.capacity_limited, frozen=a.frozen)
        cb = MapState(b.header.stamp.to_nsec(), b.processed_frames, b.processed_frames, b.header.frame_id,
                      b.graph_revision, b.graph_revision, b.resolution, healthy=b.healthy, frozen=b.frozen)
        metadata = validate_pair(ca, cb, from_transform(a.global_from_local))
        if bs['processed_frames'] != b.processed_frames or bs['last_stamp_ns'] != b.header.stamp.to_nsec():
            raise ValueError('DUFOMap native metadata/cutoff mismatch')
        current = scan_xyz(a.map); background = scan_xyz(b.map)
        current = current[np.isfinite(current).all(axis=1)]; background = background[np.isfinite(background).all(axis=1)]
        co, bo, geometry = geometry_disagreement(current, background, tolerance=self.resolution)
        metadata.update(current_points=len(current), background_points=len(background),
                        geometry=geometry, canonical_input_sha256=bs['canonical_input_sha256'],
                        current_source_frame=a.header.frame_id, background_source_frame=b.header.frame_id,
                        scope='Two independent maintained states, same observed frame boundary. No union/intersection, occupancy edit, stopped-object label or planning feedback.')
        return dict(current=a, background=b, current_xyz=current, background_xyz=background,
                    current_only=co, background_only=bo, metadata=metadata)

    def publish_pair(self, pair):
        a, b, meta = pair['current'], pair['background'], pair['metadata']
        stamp = a.header.stamp.to_sec()
        co = cloud_message(pair['current_only'], stamp); bo = cloud_message(pair['background_only'], stamp)
        co.header.frame_id = bo.header.frame_id = 'loop_map'
        bg = copy.copy(b.map); bg.header = copy.deepcopy(a.map.header)
        packet = DualMapSnapshot(); packet.header = a.header; packet.valid = True
        packet.pair_revision = self.pair_revision; packet.current = a; packet.background = b
        packet.current_from_background = a.global_from_local
        packet.current_only = co; packet.background_only = bo
        packet.status = String(data=json.dumps(meta))
        self.pair_pub.publish(packet); self.current_pub.publish(a.map); self.background_pub.publish(bg)
        self.current_only_pub.publish(co); self.background_only_pub.publish(bo)

    def process(self):
        while not rospy.is_shutdown():
            with self.condition:
                if self.pending is None:
                    self.condition.wait(.2)
                item, self.pending = self.pending, None
            if item is not None:
                try:
                    a, b, identity, generation = item; pair = self.build_pair(a, b)
                    with self.condition:
                        if generation != self.generation:
                            continue  # A source invalidated this pair while geometry was being compared.
                        self.latest = pair; self.reason = ''; self.last_identity = identity
                        self.last_pair_wall = time.monotonic()
                        self.pair_revision += 1; self.paired_boundaries.add(a.processed_frames)
                        self.condition.notify_all()
                        with Path(self.prefix+'_dual_map_pairs.jsonl').open('a') as f:
                            f.write(json.dumps(pair['metadata'])+'\n')
                        self.publish_pair(pair)
                    self.publish_status()
                except Exception as exc:
                    rospy.logerr('Dual map pair rejected: %s', str(exc)); self.reject(exc)
            if time.monotonic()-self.last_status >= 1.:
                with self.condition:
                    stale = self.latest and not self.latest['metadata']['frozen'] and time.monotonic()-self.last_pair_wall > 15.
                if stale:
                    self.reject('paired_snapshot_stale')
                self.publish_status(); self.last_status = time.monotonic()

    def save(self, _request):
        with self.save_lock:
            return self.save_locked()

    def save_locked(self):
        try:
            until = time.monotonic()+30.
            with self.condition:
                while not (self.latest and not self.reason and self.latest['metadata']['frozen']):
                    remaining = until-time.monotonic()
                    if remaining <= 0:
                        raise ValueError('no complete frozen paired snapshot: '+self.reason)
                    self.condition.wait(min(.2, remaining))
                pair = self.latest
            paths = {'current': Path(self.prefix+'_final_map.pcd'), 'background': Path(self.prefix+'_dufomap_static_map.pcd')}
            current_meta = json.loads(Path(self.prefix+'_final_map.json').read_text())
            background_meta = json.loads(Path(self.prefix+'_dufomap.json').read_text())
            if not background_meta.get('canonical_input_complete'):
                raise ValueError('DUFOMap export is incomplete')
            if background_meta['canonical_input_sha256'] != pair['metadata']['canonical_input_sha256']:
                raise ValueError('DUFOMap export input digest differs from paired snapshot')
            outputs = {}
            for name in ['current', 'background']:
                points = np.asarray(read_xyz_pcd(paths[name]), dtype=np.float32)
                actual = point_set_sha256(points)
                if actual != point_set_sha256(pair[name+'_xyz']):
                    raise ValueError(name+' file differs from frozen paired point set')
                import hashlib
                checksum = hashlib.sha256(paths[name].read_bytes()).hexdigest()
                expected = current_meta['sha256'] if name == 'current' else background_meta['outputs']['static_map']['sha256']
                if checksum != expected:
                    raise ValueError(name+' export checksum mismatch')
                outputs[name] = dict(path=str(paths[name]), points=len(points), sha256=checksum,
                    point_set_sha256=actual, source_frame=pair['metadata'][name+'_source_frame'])
            outputs['current_only'] = atomic_pcd(self.prefix+'_dual_current_only.pcd', pair['current_only'])
            outputs['background_only'] = atomic_pcd(self.prefix+'_dual_background_only.pcd', pair['background_only'])
            report = dict(pair['metadata'], outputs=outputs, paired_checkpoints=len(self.paired_boundaries),
                          mode='read_only_dual_map_maintenance', completed=True,
                          automatic_occupancy_fusion=False, active_view_feedback=False)
            atomic_json(self.prefix+'_dual_map.json', report)
            return TriggerResponse(True, self.prefix+'_dual_map.json')
        except Exception as exc:
            rospy.logerr('Dual map save rejected: %s', str(exc)); return TriggerResponse(False, str(exc))


if __name__ == '__main__':
    rospy.init_node('dual_map_manager')
    manager = DualMapManager()
    rospy.spin()
