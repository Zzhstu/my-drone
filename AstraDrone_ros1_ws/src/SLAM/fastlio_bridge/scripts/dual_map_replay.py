#!/usr/bin/env python3
"""Run BOTH actual mapping workers on canonical registered inputs of a closed flight.

Uses an owned private ROS master; does not fly or recompute FAST-LIO poses.
No new bag is recorded. Nonzero graph corrections currently fail closed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dufomap_core import atomic_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path, help='source flight prefix, without suffix')
    parser.add_argument('--output-prefix', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=2.)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--keep-open', action='store_true', help='keep final RViz and private master until Ctrl-C')
    parser.add_argument('--recover-partial', action='store_true',
                        help='rebuild both maps through the saved unfrozen map cutoff from a verified partial flight')
    parser.add_argument('--max-wall-seconds', type=float, default=600.)
    args = parser.parse_args()
    if not .1 <= args.speed <= 10 or not 60 <= args.max_wall_seconds <= 3600:
        parser.error('speed must be 0.1..10 and wall limit 60..3600')
    root = Path(__file__).resolve().parents[5]
    scripts = Path(__file__).resolve().parent
    source = str(args.prefix.expanduser().absolute()); output = str(args.output_prefix.expanduser().absolute())
    if source == output or list(Path(output).parent.glob(Path(output).name+'_*')):
        raise ValueError('choose an unused output prefix; existing results will not be overwritten')
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    reference = json.loads(Path(source+'_final_map.json').read_text())
    manifest = json.loads(Path(source+'_evidence_manifest.json').read_text())
    source_result = json.loads(Path(source+'_result.json').read_text())
    mapping = reference['loop_mapping']; expected = mapping['input_frames']
    complete_input = (manifest['recording_integrity_valid'] and manifest['bag_finalized'] and
                      mapping['processed_frames'] == expected and not mapping['rebuilding'] and
                      not mapping['capacity_limited'] and expected > 0)
    if args.recover_partial:
        if not complete_input or mapping['frozen']:
            raise ValueError('partial recovery requires verified recording and an unfrozen complete source snapshot')
        native_reference = None
    else:
        native_reference = json.loads(Path(source+'_dufomap.json').read_text())
        if not complete_input or not mapping['frozen']:
            raise ValueError('source flight must have verified recording and complete frozen boundary')
    if mapping['graph_revision'] or mapping['requested_graph_revision']:
        raise ValueError('nonzero loop graph replay is not supported by this dual-map prototype')
    bag_path = Path(source+'_evidence.bag')
    digest = hashlib.sha256()
    with bag_path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(chunk)
    if digest.hexdigest() != manifest['bag_sha256']:
        raise ValueError('source bag checksum mismatch')
    import rosbag
    ack, odometry, objects = {}, {}, {}
    takeoff = None
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, receipt in bag.read_messages(topics=['/uav1/loop_slam/input_ack', '/uav1/fastlio/odom',
                '/uav1/semantic_dynamic_objects', '/mavros_avoidance_controller/takeoff_complete', '/uav1/loop_closure/graph']):
            if topic.endswith('input_ack'):
                ns = msg.stamp.to_nsec()
                if ns in ack:
                    raise ValueError('duplicate canonical ACK')
                ack[ns] = msg.seq
            elif topic.endswith('/odom'):
                odometry[msg.header.stamp.to_nsec()] = msg
            elif topic.endswith('semantic_dynamic_objects'):
                objects[msg.header.stamp.to_nsec()] = msg
            elif topic.endswith('takeoff_complete') and msg.data and takeoff is None:
                takeoff = receipt.to_nsec()
            elif topic.endswith('/graph') and msg.revision:
                raise ValueError('source recording includes unsupported nonzero graph revision')
    if args.recover_partial:
        ack = {ns:seq for ns,seq in ack.items() if seq <= expected}
    if len(ack) != expected or sorted(ack.values()) != list(range(1, expected+1)):
        raise ValueError('canonical ACK sequence incomplete')
    if args.recover_partial and abs(max(ack)*1e-9-reference['stamp']) > 1e-6:
        raise ValueError('saved partial map stamp differs from the last canonical ACK')
    if not all(ns in odometry and ns in objects for ns in ack):
        raise ValueError('matching scan/pose/object input missing')
    children, logs = [], []
    def start(name, command):
        stream = open(output+'_'+name+'.log', 'w'); logs.append(stream)
        child = subprocess.Popen(command, stdout=stream, stderr=stream, start_new_session=True)
        children.append(child); return child
    began = time.monotonic()
    def check():
        if any(child.poll() is not None for child in children):
            raise RuntimeError('owned ROS process exited; inspect '+output+'_*log')
        if time.monotonic()-began > args.max_wall_seconds:
            raise RuntimeError('dual-map replay wall-time limit exceeded')
    def wait_for(predicate, seconds=30.):
        until = time.monotonic()+seconds
        while not predicate():
            check()
            if time.monotonic()>until:
                raise RuntimeError('timeout waiting for replay synchronization')
            time.sleep(.02)
    def interrupted(_signal, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupted)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    try:
        start('core', ['roscore', '-p', str(port)])
        import rosgraph
        wait_for(rosgraph.is_master_online, 20.)
        import rospy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import PointCloud2
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Bool, Header, String
        from std_srvs.srv import Trigger, TriggerResponse
        from fastlio_bridge.msg import DynamicObjectArray, LoopMapSnapshot
        from save_navigation_map import write_snapshot
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('dual_map_recorded_input_replay', disable_signals=True)
        params = json.loads(Path(source+'_runtime_parameters.json').read_text())
        rospy.set_param('/static_map_builder', params['/static_map_builder'])
        original = {k:v for k,v in params.get('/online_loop_mapper', {}).items() if k != 'engine'}
        original['dual_map_checkpoint_stride_frames'] = 50
        rospy.set_param('/online_loop_mapper', original)
        backend = rospy.Service('/loop_closure_backend/freeze_keyframes', Trigger,
            lambda _req: TriggerResponse(True, 'Recorded fixed trajectory, no active backend in this replay'))
        state = {'current':0, 'background':0, 'snapshot':None, 'paired':None}
        def progress(key, msg): state[key] = msg.seq
        def snapshot(msg): state['snapshot'] = msg
        def paired(msg): state['paired'] = json.loads(msg.data)
        subscriptions = [rospy.Subscriber('/uav1/loop_slam/processed_scan', Header, lambda m:progress('current',m), queue_size=120),
            rospy.Subscriber('/dufomap/processed_scan', Header, lambda m:progress('background',m), queue_size=120),
            rospy.Subscriber('/uav1/loop_slam/snapshot', LoopMapSnapshot, snapshot, queue_size=2, buff_size=32000000),
            rospy.Subscriber('/dual_map/status', String, paired, queue_size=4)]
        start('current', [str(root/'AstraDrone_ros1_ws/devel/lib/fastlio_bridge/online_loop_mapper_node')])
        start('manager', [sys.executable, str(scripts/'dual_map_manager.py'), '_output:='+output])
        calibration = json.loads(Path(source+'_sensor_geometry.json').read_text())
        rospy.set_param('/dufomap_adapter/lidar_origin_in_body', calibration['laser_origin_in_body'])
        start('background', [str(root/'.venvs/dufomap/bin/python'), str(scripts/'dufomap_adapter.py'),
            '_output:='+output, '_dual_map_checkpoint_stride_frames:=50'])
        rospy.wait_for_service('/uav1/loop_slam/freeze_mapping', timeout=30.)
        rospy.wait_for_service('/demo/dufomap/save_map', timeout=30.)
        wait_for(lambda: state['paired'] is not None)
        if not args.headless:
            start('rviz', ['rviz', '-d', str(scripts.parent/'rviz/half_warehouse_dual_map.rviz')])
        pubs = dict(scan=rospy.Publisher('/uav1/fastlio/registered_scan', PointCloud2, queue_size=30),
            odom=rospy.Publisher('/uav1/fastlio/odom', Odometry, queue_size=30),
            objects=rospy.Publisher('/uav1/semantic_dynamic_objects', DynamicObjectArray, queue_size=30),
            clock=rospy.Publisher('/clock', Clock, queue_size=10, latch=True),
            takeoff=rospy.Publisher('/mavros_avoidance_controller/takeoff_complete', Bool, queue_size=2, latch=True))
        wait_for(lambda: pubs['scan'].get_num_connections()>=2 and pubs['odom'].get_num_connections()>=2
                 and pubs['objects'].get_num_connections()>=1 and pubs['takeoff'].get_num_connections()>=1)
        time.sleep(.3)
        previous_ns = None; sent = 0; takeoff_sent = False
        with rosbag.Bag(str(bag_path)) as bag:
            for _, scan, _ in bag.read_messages(topics=['/uav1/fastlio/registered_scan']):
                ns = scan.header.stamp.to_nsec()
                if ns not in ack:
                    continue
                if ack[ns] != sent+1 or scan.header.frame_id != 'map':
                    raise ValueError('canonical scan ordering/frame invalid')
                check()
                wait_for(lambda: min(state['current'],state['background']) >= max(0,sent-8))
                if previous_ns is not None:
                    time.sleep(min(1., (ns-previous_ns)*1e-9/args.speed))
                pubs['clock'].publish(Clock(clock=scan.header.stamp))
                if not takeoff_sent and takeoff is not None and ns>=takeoff:
                    pubs['takeoff'].publish(Bool(data=True)); takeoff_sent=True
                    time.sleep(.02)
                pubs['scan'].publish(scan); pubs['odom'].publish(odometry[ns]); pubs['objects'].publish(objects[ns])
                previous_ns = ns; sent += 1
                if sent % 500 == 0:
                    print('Dual map replay %d/%d: original=%d DUFO=%d paired=%s' % (sent,expected,
                        state['current'],state['background'],(state['paired'] or {}).get('processed_frames')), flush=True)
        wait_for(lambda: state['current']==expected and state['background']==expected, 90.)
        response = rospy.ServiceProxy('/uav1/loop_slam/freeze_mapping', Trigger)()
        if not response.success:
            raise ValueError(response.message)
        def final_ready():
            msg = state['snapshot']
            return msg and msg.frozen and msg.input_frames==msg.processed_frames==expected and not msg.rebuilding and not msg.capacity_limited
        wait_for(final_ready, 30.)
        final = state['snapshot']
        if final.header.stamp.to_nsec() != previous_ns or final.graph_revision or final.requested_graph_revision:
            raise ValueError('final original snapshot cutoff/graph mismatch')
        loop = {k:getattr(final,k) for k in ('revision','graph_revision','requested_graph_revision','rebuilding',
            'input_frames','processed_frames','capacity_limited','frozen')}
        loop.update(first_keyframe_fixed=True, scope='Recorded registered scan/pose replay; no new frontend or flight')
        write_snapshot(final.map, output+'_final_map.pcd', '/uav1/loop_slam/cloud_map', json.loads(final.ghost_history.data), loop)
        for service in ['/demo/dufomap/save_map','/demo/dual_map/save_maps']:
            response = rospy.ServiceProxy(service, Trigger)()
            if not response.success:
                raise ValueError(service+': '+response.message)
        native = json.loads(Path(output+'_dufomap.json').read_text())
        dual = json.loads(Path(output+'_dual_map.json').read_text())
        same_input = (native['canonical_input_sha256'] == native_reference['canonical_input_sha256']
                      if native_reference is not None else None)
        same_native = (native['outputs']['static_map']['sha256'] == native_reference['outputs']['static_map']['sha256']
                       if native_reference is not None else None)
        completed = bool(dual['completed'] and native['canonical_input_complete'] and
                         (args.recover_partial or (same_input and same_native)))
        result = dict(completed=completed,
            mode='partial_recorded_input_recovery' if args.recover_partial else 'recorded_input_replay',
            flight_performed=False, raw_frontend_recomputed=False, source_prefix=source, output_prefix=output,
            source_bag_sha256=digest.hexdigest(), source_graph_revision=0, processed_frames=expected,
            cutoff_stamp_ns=previous_ns, current_points=dual['current_points'], background_points=dual['background_points'],
            paired_checkpoints=dual['paired_checkpoints'], same_native_input_digest=same_input,
            native_static_pcd_byte_identical=same_native, wall_seconds=time.monotonic()-began,
            source_snapshot_frozen=bool(mapping['frozen']),
            source_flight_success=bool(source_result.get('task_success', False)),
            source_termination=source_result.get('termination'),
            limitations=['No new flight or FAST-LIO frontend replay',
                'Recorded takeoff Bool receipt approximates original callback time',
                'Partial recovery uses the saved original map cutoff and is not a successful flight' if args.recover_partial else
                'Compared with the original completed DUFOMap export',
                'Nonzero loop graph is explicitly refused', 'Geometric differences are not ghost classifications'])
        atomic_json(output+'_dual_replay_result.json', result)
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        if not result['completed']:
            raise ValueError('replay pairing/export validation failed')
        if args.keep_open:
            print('Final paired maps retained on private ROS master; Ctrl-C to close.', flush=True)
            while True: time.sleep(.5)
    except BaseException as exc:
        if not isinstance(exc, KeyboardInterrupt):
            atomic_json(output+'_replay_failure.json', dict(error=str(exc), completed=False, flight_performed=False))
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try: child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL); child.wait()
        for stream in logs: stream.close()


if __name__ == '__main__':
    main()
