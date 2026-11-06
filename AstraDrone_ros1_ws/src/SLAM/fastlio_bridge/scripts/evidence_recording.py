#!/usr/bin/env python3
"""Record-only sampling. Never changes live sensors, Gazebo or controllers."""
import math
import io


COMPACT_OMIT = ('/livox/lidar', '/livox/imu', '/uav1/loop_closure/registered_scan')
FINAL_TOPICS = ('/uav1/loop_slam/snapshot', '/uav1/loop_closure/map', '/uav1/map/ghost_history')


def recording_policy(topics, profile='compact', truth_hz=30., map_hz=.05, light=False):
    if profile not in ('compact', 'full'):
        raise ValueError('Unknown evidence profile')
    if not math.isfinite(truth_hz) or not 1 <= truth_hz <= 1000:
        raise ValueError('Evidence truth frequency must be in [1,1000] Hz')
    if not math.isfinite(map_hz) or not .01 <= map_hz <= 2:
        raise ValueError('Evidence snapshot frequency must be in [.01,2] Hz')
    rates = {} if profile == 'full' else {
        '/clock': 50., '/gazebo/model_states': float(truth_hz),
        '/uav1/loop_slam/snapshot': float(map_hz),
        '/uav1/loop_closure/map': float(map_hz),
        '/uav1/stable_static_map': .1, '/uav1/local_free_space': .2,
        '/uav1/foreground_points': .2, '/uav1/map/ghost_history': .2,
        '/uav1/map/ghost_suspects': 1., '/uav1/exploration/status': 1.,
        '/uav1/loop_closure/path': 1., '/uav1/loop_closure/raw_path': 1.,
        '/uav1/loop_closure/graph': 1.}
    omitted = list(COMPACT_OMIT) if profile == 'compact' else []
    if light and profile == 'compact':
        rates.update({'/clock': 10., '/uav1/local_free_space': .1,
            '/uav1/stable_static_map': .05, '/uav1/map/ghost_history': .05,
            '/uav1/map/ghost_suspects': .5, '/uav1/exploration/status': .2,
            '/uav1/loop_closure/path': .2, '/uav1/loop_closure/raw_path': .2,
            '/uav1/loop_closure/graph': .2, '/dual_map/status': 1.})
    return dict(profile=profile, topics=[t for t in topics if t not in omitted],
                max_hz=rates, omitted_topics=omitted,
                final_sample_topics=list(FINAL_TOPICS) if profile=='compact' else [],
                pack_registered_scan_xyz=bool(light and profile=='compact'),
                scope='Only disk recording is sampled. Live published topics/control unchanged. '
                      'Compact keeps original-rate bridge scan/odom/objects, but is not complete raw-LIO '
                      'or callback-timing evidence. Low-rate final samples are also kept on close. '
                      'Light mode stores registered scans as bit-identical XYZ float32 fields only; '
                      'other fields are omitted from disk, not from live messages.')


def packed_xyz(message):
    """Lossless XYZ byte packing for mapping replay; never spatially subsample."""
    import numpy as np
    from sensor_msgs.msg import PointCloud2, PointField
    fields={f.name:f for f in message.fields}
    for name in ('x','y','z'):
        f=fields.get(name)
        if f is None or f.datatype!=PointField.FLOAT32 or f.count!=1:
            raise ValueError('XYZ packing requires scalar float32 coordinates')
    view=np.ndarray((message.height,message.width,message.point_step),dtype=np.uint8,
        buffer=message.data,strides=(message.row_step,message.point_step,1))
    packed=np.empty((message.height,message.width,12),dtype=np.uint8)
    for i,name in enumerate(('x','y','z')):
        offset=fields[name].offset
        packed[:,:,4*i:4*i+4]=view[:,:,offset:offset+4]
    return PointCloud2(header=message.header,height=message.height,width=message.width,
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(('x','y','z'))],
        is_bigendian=message.is_bigendian,point_step=12,row_step=message.width*12,
        data=packed.tobytes(),is_dense=message.is_dense)


class RateGate:
    def __init__(self, rates):
        self.rates = rates
        self.last = {}

    def accept(self, topic, stamp):
        if not math.isfinite(stamp) or stamp < 0:
            return False
        hz = self.rates.get(topic)
        if hz is None:
            return True
        last = self.last.get(topic)
        if last is None or stamp < last-.001 or stamp-last >= 1./hz-1e-9:
            self.last[topic] = stamp
            return True
        return False


def write_raw(bag, topic, payload, header, stamp, cache):
    """Preserve source topic/type/definition and bytes, without decoding clouds."""
    key = (header['type'], header['md5sum'])
    if key not in cache:
        cache[key] = type('RecordedMessage', (), dict(
            _type=key[0], _md5sum=key[1], _full_text=header['message_definition']))
    bag.write(topic, (key[0], payload, key[1], cache[key]), t=stamp,
              raw=True, connection_header=header)


def message_bytes(message):
    # rospy shares a transport between subscribers to the same topic. Its own
    # /clock subscription can therefore deliver a typed Clock to an AnyMsg
    # callback. Preserve that payload too, rather than silently losing clocks.
    payload=getattr(message,'_buff',None)
    if payload is None:
        stream=io.BytesIO();message.serialize(stream);payload=stream.getvalue()
    header=getattr(message,'_connection_header',None)
    if not header:
        header=dict(type=message._type,md5sum=message._md5sum,message_definition=message._full_text)
    return payload,dict(header)


def main():
    import argparse
    from collections import Counter
    import json
    import os
    from pathlib import Path
    import queue
    import signal
    import threading
    import rosbag
    import rospy
    from experiment_evidence import EVIDENCE_TOPICS

    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--truth-hz',type=float,default=30.)
    p.add_argument('--map-hz',type=float,default=.05)
    p.add_argument('--light',action='store_true')
    a=p.parse_args()
    policy=recording_policy(EVIDENCE_TOPICS,'compact',a.truth_hz,a.map_hz,a.light)
    active=Path(str(a.output)+'.active')
    stats_path=a.output.with_name(a.output.stem+'_recording.json')
    if a.output.exists() or active.exists():
        raise FileExistsError('Refusing to overwrite evidence bag')
    gate=RateGate(policy['max_hz']); stop=threading.Event(); lock=threading.Lock()
    pending=queue.Queue(maxsize=1024); queued_bytes=0; closed=False
    received=Counter();throttled=Counter();overflow=Counter();written=Counter()
    latest={};last_written={};cache={};errors=[]
    max_queued_bytes=0

    def save_stats(finalized):
        stats=dict(profile='compact',policy=policy,ready=True,finalized=finalized,
                   received=dict(received),throttled=dict(throttled),
                   queue_overflow=dict(overflow),written=dict(written),write_errors=errors,
                   max_queued_bytes=max_queued_bytes,queue_limit_bytes=64*1024**2)
        tmp=stats_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(stats,indent=2,allow_nan=False)+'\n');tmp.replace(stats_path)

    def callback(message, topic):
        nonlocal queued_bytes,max_queued_bytes
        stamp=rospy.Time.now().to_sec()
        with lock:
            if closed or stop.is_set():return
            received[topic]+=1
            try:
                if policy['pack_registered_scan_xyz'] and topic=='/uav1/fastlio/registered_scan':
                    from sensor_msgs.msg import PointCloud2
                    payload,header=message_bytes(message)
                    cloud=PointCloud2();cloud.deserialize(payload)
                    payload,_=message_bytes(packed_xyz(cloud))
                else:payload,header=message_bytes(message)
            except Exception as error:
                errors.append('callback '+topic+': '+str(error));stop.set();return
            entry=(topic,payload,header,stamp)
            if topic in FINAL_TOPICS:latest[topic]=entry
            if not gate.accept(topic,stamp):
                throttled[topic]+=1;return
            if queued_bytes+len(payload)>64*1024**2 or pending.full():
                overflow[topic]+=1;return
            queued_bytes+=len(payload);max_queued_bytes=max(max_queued_bytes,queued_bytes)
            pending.put_nowait(entry)

    def write_entry(bag,entry):
        topic,payload,header,stamp=entry
        write_raw(bag,topic,payload,header,rospy.Time.from_sec(stamp),cache)
        written[topic]+=1;last_written[topic]=stamp

    def request_stop(*unused):stop.set()
    signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
    rospy.init_node('compact_evidence_recorder',anonymous=True,disable_signals=True)
    rospy.on_shutdown(request_stop)
    subscribers=[]
    try:
        with rosbag.Bag(str(active),'w',compression=rosbag.Compression.LZ4,chunk_threshold=4*1024**2) as bag:
            for topic in policy['topics']:
                subscribers.append(rospy.Subscriber(topic,rospy.AnyMsg,callback,callback_args=topic,
                    queue_size=8 if topic not in policy['max_hz'] else 1,buff_size=16*1024**2))
            save_stats(False)
            while not stop.is_set():
                try:entry=pending.get(timeout=.1)
                except queue.Empty:continue
                with lock:queued_bytes-=len(entry[1])
                write_entry(bag,entry)
            for subscriber in subscribers:subscriber.unregister()
            with lock:closed=True
            while not pending.empty():
                entry=pending.get_nowait()
                with lock:queued_bytes-=len(entry[1])
                write_entry(bag,entry)
            for topic,entry in latest.items():
                if entry[3]>last_written.get(topic,-float('inf')):
                    write_entry(bag,entry)
        # Publish a final bag only after rosbag closes its index successfully.
        os.link(active,a.output);active.unlink()
        save_stats(True)
    except Exception as error:
        errors.append(str(error));stop.set()
        for subscriber in subscribers:subscriber.unregister()
        with lock:closed=True
        save_stats(False)
        raise
    finally:
        rospy.signal_shutdown('evidence recorder closed')
    if overflow or errors:
        raise SystemExit('Evidence callback/write/queue error; recording is not complete')


if __name__=='__main__':main()
