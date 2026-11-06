#!/usr/bin/env python3
"""Independent ROS DUFOMap branch gated by canonical online mapper input ACKs."""
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import message_filters
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, TriggerResponse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dufomap_core import DufoMapEngine, atomic_json


def scan_xyz(message):
    types = {7: 'f4', 8: 'f8'}; fields = {f.name: f for f in message.fields}
    dtype = np.dtype(dict(names=['x', 'y', 'z'],
        formats=[('>' if message.is_bigendian else '<')+types[fields[n].datatype] for n in ('x', 'y', 'z')],
        offsets=[fields[n].offset for n in ('x', 'y', 'z')], itemsize=message.point_step))
    rows = np.ndarray((message.height, message.width), buffer=message.data, dtype=dtype,
                      strides=(message.row_step, message.point_step))
    return np.ascontiguousarray(np.column_stack([rows[n].ravel() for n in ('x', 'y', 'z')]), dtype='<f4')


def body_pose(odom):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    return [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]


def cloud_message(points, stamp):
    points = np.ascontiguousarray(points, dtype='<f4').reshape(-1, 3)
    msg = PointCloud2(header=Header(stamp=rospy.Time.from_sec(stamp), frame_id='map'),
                     height=1, width=len(points), is_bigendian=False, is_dense=True,
                     point_step=12, row_step=12*len(points), data=points.tobytes())
    msg.fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    return msg


class DufoAdapter:
    def __init__(self):
        self.prefix = rospy.get_param('~output')
        self.engine = DufoMapEngine(resolution=rospy.get_param('~resolution', .15),
            sensor_error=rospy.get_param('~sensor_error', .20),
            pose_cells=rospy.get_param('~pose_cells', 1), threads=rospy.get_param('~threads', 2),
            max_frames=rospy.get_param('~max_frames', 6500),
            lidar_origin=rospy.get_param('~lidar_origin_in_body', [0., 0., .13]))
        self.condition = threading.Condition(); self.engine_lock = threading.Lock()
        self.queue = []; self.accepted = 0; self.error = ''; self.stopping = False
        self.publishing_period = 5.; self.last_publish = 0.
        self.checkpoint_stride = int(rospy.get_param('~dual_map_checkpoint_stride_frames', 0))
        if not 0 <= self.checkpoint_stride <= 1000:
            raise ValueError('invalid dual map checkpoint stride')
        self.snapshot_pub = None
        if self.checkpoint_stride:
            from fastlio_bridge.msg import DufoMapSnapshot
            self.snapshot_type = DufoMapSnapshot
            self.snapshot_pub = rospy.Publisher('/dufomap/snapshot', DufoMapSnapshot, queue_size=2, latch=True)
        self.status_pub = rospy.Publisher('/dufomap/status', String, queue_size=1, latch=True)
        self.static_pub = rospy.Publisher('/dufomap/static_map', PointCloud2, queue_size=1, latch=True)
        self.dynamic_pub = rospy.Publisher('/dufomap/removed_points', PointCloud2, queue_size=1, latch=True)
        self.processed_pub = rospy.Publisher('/dufomap/processed_scan', Header, queue_size=120)
        self.scan_sub = message_filters.Subscriber('/uav1/fastlio/registered_scan', PointCloud2, queue_size=120, buff_size=16000000)
        self.odom_sub = message_filters.Subscriber('/uav1/fastlio/odom', Odometry, queue_size=120)
        self.ack_filter = message_filters.SimpleFilter()
        self.sync = message_filters.TimeSynchronizer([self.scan_sub, self.odom_sub, self.ack_filter], 120)
        self.sync.registerCallback(self.accept)
        self.ack_sub = rospy.Subscriber('/uav1/loop_slam/input_ack', Header,
            lambda msg: self.ack_filter.signalMessage(SimpleNamespace(header=msg)), queue_size=120)
        self.save_service = rospy.Service('/demo/dufomap/save_map', Trigger, self.save)
        self.thread = threading.Thread(target=self.process, daemon=True); self.thread.start()
        self.publish_status(ready=True)

    def publish_status(self, ready=True):
        with self.condition:
            state = dict(ready=ready, accepted_frames=self.accepted, queued_frames=len(self.queue),
                         error=self.error, healthy=not self.error, planner_input=False)
        with self.engine_lock:
            state.update(self.engine.stats())
        atomic_json(self.prefix+'_dufomap_status.json', state)
        self.status_pub.publish(String(data=json.dumps(state)))

    def accept(self, scan, odom, ack):
        try:
            if scan.header.frame_id != 'map' or odom.header.frame_id != 'map':
                raise ValueError('DUFOMap requires matching continuous local map scan/pose')
            with self.condition:
                if self.error:
                    return
                if ack.header.seq != self.accepted+1 or len(self.queue) >= 512:
                    raise ValueError('canonical ACK input gap or adapter queue capacity reached')
                self.accepted += 1
                self.queue.append((scan, odom, ack.header.seq)); self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.error = str(exc); self.condition.notify_all()
            rospy.logerr('DUFOMap input rejected: %s', str(exc))
            self.publish_status()

    def publish_clouds(self, frozen=False):
        with self.engine_lock:
            static, dynamic, _ = self.engine.snapshot(); stamp = self.engine.last_stamp_ns*1e-9
            stats = self.engine.stats()
        current = cloud_message(static, stamp); removed = cloud_message(dynamic, stamp)
        exact_stamp = rospy.Time(stats['last_stamp_ns']//1000000000, stats['last_stamp_ns']%1000000000)
        current.header.stamp = removed.header.stamp = exact_stamp
        self.static_pub.publish(current)
        self.dynamic_pub.publish(removed)
        if self.snapshot_pub is not None:
            packet = self.snapshot_type()
            packet.header = current.header; packet.processed_frames = stats['processed_frames']
            packet.graph_revision = 0  # This branch uses continuous local poses.
            packet.resolution = stats['config']['resolution_m']
            packet.healthy = not self.error; packet.frozen = frozen
            packet.map = current; packet.removed = removed
            packet.status = String(data=json.dumps(stats))
            self.snapshot_pub.publish(packet)

    def process(self):
        while not rospy.is_shutdown():
            with self.condition:
                if self.error:
                    self.condition.wait(.2)
                    continue
                if not self.queue:
                    self.condition.wait(.2)
                if not self.queue or self.error:
                    continue
                scan, odom, seq = self.queue.pop(0)
            try:
                p, q = body_pose(odom)
                with self.engine_lock:
                    self.engine.ingest(scan_xyz(scan), p, q, scan.header.stamp.to_nsec(), seq)
                self.processed_pub.publish(Header(seq=seq, stamp=scan.header.stamp, frame_id='map'))
                checkpoint = self.checkpoint_stride and (seq == 1 or seq % self.checkpoint_stride == 0)
                periodic = not self.checkpoint_stride and scan.header.stamp.to_sec()-self.last_publish >= self.publishing_period
                if checkpoint or periodic:
                    self.publish_clouds(); self.last_publish = scan.header.stamp.to_sec()
                    self.publish_status()
            except Exception as exc:
                with self.condition:
                    self.error = str(exc); self.condition.notify_all()
                rospy.logerr('DUFOMap processing failed: %s', str(exc))
                self.publish_status()

    def save(self, _request):
        try:
            reference = json.loads(Path(self.prefix+'_final_map.json').read_text())
            loop = reference.get('loop_mapping') or {}
            if not loop.get('frozen') or loop.get('rebuilding') or loop.get('capacity_limited'):
                raise ValueError('reference mapping must be complete and frozen before DUFOMap export')
            expected = loop['input_frames']
            until = time.monotonic()+90.
            while time.monotonic() < until:
                with self.condition:
                    if self.error:
                        raise ValueError(self.error)
                with self.engine_lock:
                    if self.engine.frames == expected:
                        break
                time.sleep(.1)
            with self.engine_lock:
                if self.engine.frames != expected or self.accepted != expected:
                    raise ValueError('DUFOMap did not receive/process every canonical input frame')
                if abs(self.engine.last_stamp_ns*1e-9-reference['stamp']) > 1e-6:
                    raise ValueError('DUFOMap/reference cutoff stamp mismatch')
                report = self.engine.save(self.prefix, dict(canonical_input_complete=True,
                    reference_map_metadata=self.prefix+'_final_map.json',
                    reference_input_frames=expected, loop_graph_revision=loop['graph_revision'],
                    loop_corrections_applied=False, planner_input=False))
            self.publish_clouds(frozen=True); self.publish_status()
            return TriggerResponse(success=True, message=report['outputs']['static_map']['path'])
        except Exception as exc:
            rospy.logerr('DUFOMap save failed: %s', str(exc))
            return TriggerResponse(success=False, message=str(exc))


if __name__ == '__main__':
    rospy.init_node('dufomap_adapter')
    adapter = DufoAdapter()
    rospy.spin()
