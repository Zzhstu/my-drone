#!/usr/bin/env python3
"""Export EXACT maintained navigation topic as finite XYZ binary PCD + provenance.

Does not claim unseen/occluded ghosts are removed and does not crop actors using
Gazebo truth. A currently stationary person is legitimately occupied space.
"""
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import sys

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
# Catkin executable relays are not importable library modules. Resolve this
# script's actual source/install directory before importing the shared helper.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from loop_coordinates import matrix
from pending_map_colors import colored_payload


def write_snapshot(cloud, output, topic, ghost_history=None, loop_mapping=None):
    rows = list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True))
    points = np.asarray(rows, dtype='<f4').reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('地图为空，拒绝覆盖上一次有效地图')
    path = Path(output).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# .PCD v0.7 - navigation map snapshot\nVERSION 0.7\n'
              'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              'WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n') % (len(points), len(points))
    payload = header.encode('ascii') + points.tobytes()
    metadata = dict(topic=topic, frame_id=cloud.header.frame_id,
                    snapshot_schema_version=2,
                    stamp=cloud.header.stamp.to_sec(), points=len(points),
                    sha256=hashlib.sha256(payload).hexdigest(),
                    fields=['x', 'y', 'z'],
                    note='Exact finite XYZ from maintained RViz map; no extra ghost removal at export. '
                         'Unobserved/occluded old occupancy may remain; stationary people are obstacles.')
    history_payload = None
    if loop_mapping is not None:metadata['loop_mapping']=loop_mapping
    history_path = path.with_name(path.stem + '_ghost_history.json')
    if ghost_history is not None:
        if (ghost_history.get('version') != 1 or
                ghost_history.get('frame_id') != cloud.header.frame_id or
                not isinstance(ghost_history.get('records'), list) or
                abs(float(ghost_history.get('map_stamp',-1))-
                    cloud.header.stamp.to_sec()) > 1.5):
            raise ValueError('Ghost history does not match the maintained map snapshot')
        history_payload = json.dumps(ghost_history, ensure_ascii=False,
                                     indent=2, allow_nan=False).encode('utf8')
        metadata['ghost_history'] = str(history_path)
        metadata['ghost_history_sha256'] = hashlib.sha256(history_payload).hexdigest()
        metadata['ghost_history_records']=len(ghost_history['records'])
    # Write in the same filesystem then replace atomically; never truncate a
    # valid map on interruption. JSON checksum detects a split pair after crash.
    outputs = []
    if ghost_history is not None:
        # A colored audit product accompanies the original XYZ map. Preserve
        # suppressed/never-reviewed suspects, not just actionable candidates.
        colored,details=colored_payload(points,ghost_history)
        colored_path=path.with_name(path.stem+'_labeled.pcd')
        details.update(frame_id=cloud.header.frame_id,map_stamp=cloud.header.stamp.to_sec(),
            history_stamp=ghost_history['map_stamp'],source_xyz_sha256=metadata['sha256'])
        metadata['labeled_map']=str(colored_path)
        metadata['labeled_map_sha256']=details['sha256']
        metadata['unresolved_point_labels']=details['counts']
        metadata['unmatched_unresolved_records']=len(details['unmatched_unresolved_records'])
        outputs.extend(((colored_path,colored),(colored_path.with_suffix('.json'),
            json.dumps(details,ensure_ascii=False,indent=2,allow_nan=False).encode('utf8'))))
    if history_payload is not None:
        outputs.append((history_path, history_payload))
    outputs.extend(((path, payload), (path.with_suffix('.json'),
                    json.dumps(metadata, ensure_ascii=False, indent=2).encode('utf8'))))
    for target, data in outputs:
        tmp = target.with_name(target.name + '.tmp')
        with tmp.open('wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(tmp), str(target))
    return metadata


class MapSaver:
    def __init__(self):
        self.lock = threading.Lock()
        self.save_lock = threading.Lock()
        self.cloud = None
        self.ghost_history = None
        self.ghost_history_received = 0.0
        self.loop_mapping=None
        self.map_received = self.scan_received = 0.0
        self.topic = rospy.get_param('~topic', '/uav1/fastlio/cloud_map')
        self.scan_topic = rospy.get_param('~scan_topic','/uav1/fastlio/registered_scan')
        self.output = rospy.get_param('~output')
        self.require_ghost_history = bool(rospy.get_param('~require_ghost_history',False))
        self.ghost_history_topic = rospy.get_param('~ghost_history_topic','/uav1/map/ghost_history')
        snapshot_topic=rospy.get_param('~loop_snapshot_topic','')
        if snapshot_topic:
            from fastlio_bridge.msg import LoopMapSnapshot
            rospy.Subscriber(snapshot_topic,LoopMapSnapshot,self.receive_loop_snapshot,
                queue_size=1,buff_size=32000000)
            self.final_service=rospy.Service('/demo/save_final_loop_map',Trigger,
                lambda req:self.save(req,require_frozen=True))
        else:
            rospy.Subscriber(self.topic, PointCloud2, self.receive, queue_size=1)
            rospy.Subscriber(self.ghost_history_topic, String, self.receive_ghost_history, queue_size=1)
        rospy.Subscriber(self.scan_topic, PointCloud2, self.scan, queue_size=1)
        self.service = rospy.Service('/demo/save_map', Trigger, self.save)

    def receive(self, msg):
        with self.lock:
            self.cloud, self.map_received = msg, time.monotonic()

    def receive_loop_snapshot(self,msg):
        # Map and ledger are captured under one lock from ONE atomic message.
        # Never pair independently latched topics from two graph revisions.
        try:
            history=json.loads(msg.ghost_history.data)
            if (msg.header.frame_id!='loop_map' or msg.map.header.stamp!=msg.header.stamp or
                msg.map.header.frame_id!='loop_map' or history.get('frame_id')!='loop_map' or
                abs(history.get('map_stamp',-1)-msg.header.stamp.to_sec())>1e-6):
                raise ValueError('global map/ledger mismatch')
            mapping=dict(revision=msg.revision,graph_revision=msg.graph_revision,
                    requested_graph_revision=msg.requested_graph_revision,rebuilding=msg.rebuilding,
                    input_frames=msg.input_frames,processed_frames=msg.processed_frames,
                    capacity_limited=msg.capacity_limited,
                    frozen=msg.frozen,
                    scope='online maintained global map; continuous local control unchanged')
            mapping['first_keyframe_fixed']=bool(msg.graph_revision==0 or (
                    msg.graph.raw.poses and msg.graph.optimized.poses and
                    np.allclose(matrix(msg.graph.raw.poses[0].pose),
                        matrix(msg.graph.optimized.poses[0].pose),atol=1e-8,rtol=0)))
            with self.lock:
                self.cloud=msg.map;self.ghost_history=history
                self.map_received=self.ghost_history_received=time.monotonic()
                self.loop_mapping=mapping
        except (ValueError,TypeError,AttributeError) as error:
            rospy.logerr_throttle(3.,'Global map export snapshot rejected: %s',str(error))

    def scan(self, msg):
        self.scan_received = time.monotonic()

    def receive_ghost_history(self, msg):
        try:
            history = json.loads(msg.data)
            if history.get('version') != 1 or not isinstance(history.get('records'),list):
                return
        except (ValueError, TypeError, AttributeError):
            return
        with self.lock:
            self.ghost_history = history
            self.ghost_history_received = time.monotonic()

    def save(self, req, require_frozen=False):
        with self.save_lock:
            with self.lock:
                cloud, received = self.cloud, self.map_received
                history, history_received = self.ghost_history, self.ghost_history_received
                loop_mapping=getattr(self,'loop_mapping',None)
            now = time.monotonic()
            if require_frozen and (not loop_mapping or not loop_mapping.get('frozen')):
                return TriggerResponse(False,'尚未收到冻结后的全局快照，拒绝把截止前旧快照当作最终地图')
            if cloud is None or now-received > 3 or now-self.scan_received > 3:
                return TriggerResponse(False, '地图或注册点云停止更新超过3秒；拒绝把旧数据称为最终地图')
            if self.require_ghost_history and (history is None or now-history_received > 3):
                return TriggerResponse(False, '鬼影历史未更新；拒绝保存缺失审计记录的最终地图')
            if loop_mapping and loop_mapping.get('frozen') and (
                    loop_mapping['rebuilding'] or
                    loop_mapping['graph_revision']!=loop_mapping['requested_graph_revision'] or
                    loop_mapping['input_frames']!=loop_mapping['processed_frames']):
                return TriggerResponse(False,'全局地图正在完成截止前的回环重建，请等待最终一致快照')
            try:
                meta = write_snapshot(cloud, self.output, self.topic,
                    history if history is not None and now-history_received <= 3 else None,loop_mapping)
                return TriggerResponse(True, '%s (%d XYZ points, frame=%s)' % (
                    self.output, meta['points'], meta['frame_id']))
            except Exception as exc:
                rospy.logerr('Map export failed: %s', exc)
                return TriggerResponse(False, str(exc))


if __name__ == '__main__':
    rospy.init_node('navigation_map_saver')
    saver = MapSaver()
    rospy.spin()
