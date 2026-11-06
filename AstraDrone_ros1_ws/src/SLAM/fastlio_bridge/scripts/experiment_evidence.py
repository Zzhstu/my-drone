#!/usr/bin/env python3
"""Versioned evidence manifests for replayable experiments (no flight control)."""
import hashlib
import json
import re
import time
from pathlib import Path


EVIDENCE_TOPICS = (
    '/clock', '/livox/lidar', '/livox/imu', '/Odometry',
    '/uav1/fastlio/odom', '/uav1/fastlio/registered_scan',
    '/uav1/local_free_space', '/uav1/map/ghost_suspects',
    '/uav1/map/ghost_history', '/uav1/map/ghost_revisit_status',
    '/uav1/dynamic_objects', '/uav1/semantic_dynamic_objects',
    # Preserve actual callback inputs for timing-sensitive structural veto
    # diagnosis; scan-only direct replays do not reproduce ROS queue order.
    '/uav1/foreground_points', '/uav1/stable_static_map',
    '/uav1/collision_risk_level', '/uav1/static_collision_risk_level',
    '/uav1/fused_collision_risk_level', '/uav1/fused_avoidance_velocity',
    '/uav1/fused_avoidance_source', '/uav1/exploration/status',
    '/mavros/state', '/mavros/extended_state', '/mavros/battery',
    '/mavros/local_position/pose', '/mavros_avoidance_controller/command_preview',
    '/mavros/setpoint_velocity/cmd_vel', '/small_indoor/map_velocity',
    '/mavros_avoidance_controller/takeoff_complete',
    '/gazebo/model_states', '/tf', '/tf_static',
    '/uav1/loop_closure/odom', '/uav1/loop_closure/path',
    '/uav1/loop_closure/raw_path', '/uav1/loop_closure/map',
    '/uav1/loop_closure/registered_scan', '/uav1/loop_closure/map_to_raw',
    '/uav1/map/processed_scan',
    '/uav1/loop_closure/constraints', '/uav1/loop_closure/diagnostics',
    '/loop_closure_backend/seed_evidence',
    '/uav1/loop_closure/graph','/uav1/loop_slam/snapshot','/uav1/loop_slam/status',
    '/uav1/loop_slam/input_ack','/uav1/loop_slam/processed_scan',
    '/demo/pedestrian/1/patrol_status','/demo/pedestrian/1/blocked','/dual_map/status')
REQUIRED_REPLAY_TOPICS = ('/clock', '/uav1/fastlio/odom',
                          '/uav1/fastlio/registered_scan')


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def source_fingerprint(root):
    root = Path(root)
    package = root/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge'
    files = []
    for directory in ('src', 'scripts', 'config', 'launch','include','msg'):
        files += [p for p in (package/directory).rglob('*')
                  if p.is_file() and p.suffix in ('.cpp', '.hpp', '.h', '.py', '.yaml', '.launch','.msg')]
    files += [package/name for name in ('CMakeLists.txt', 'package.xml')]
    lio=root/'AstraDrone_ros1_ws/src/SLAM/FAST_LIO'
    files += [lio/name for name in ('launch/mapping_mid360.launch',
                                   'config/mid360.yaml','config/mid360_sim.yaml')]
    for directory in ('src','include'):
        files += [p for p in (lio/directory).rglob('*')
                  if p.is_file() and p.suffix in ('.cpp','.h','.hpp')]
    files += [lio/'CMakeLists.txt',lio/'package.xml']
    hashes = {str(p.relative_to(root)): sha256_file(p) for p in sorted(set(files)) if p.is_file()}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    return dict(sha256=digest, files=hashes)


def start_manifest(root, prefix, options, world):
    root = Path(root)
    prefix = Path(prefix)
    inputs = {}
    for p in (Path(world), Path(str(prefix)+'_scene.json')):
        if p.is_file():
            inputs[str(p)] = sha256_file(p)
    from evidence_recording import recording_policy
    policy=recording_policy(EVIDENCE_TOPICS,options.get('evidence_profile','full'),
        options.get('evidence_truth_hz',30.),options.get('evidence_map_hz',.05),
        options.get('efficient_demo',False))
    manifest = dict(version=1, scope='SITL evidence; ground truth is evaluator-only',
                    source=source_fingerprint(root), input_sha256=inputs,
                    run_options=options, requested_topics=policy['topics'],recording_policy=policy,
                    bag_requested=bool(options.get('record_evidence')),
                    bag_finalized=False, replay_inputs_available=False,
                    limitations=['A bag enables same-input mapping replay; closed-loop planner runs '
                                 'still have different sensor viewpoints.',
                                 'No real-aircraft safety or complete dynamic/static point labels implied.'])
    binaries = [root/'AstraDrone_ros1_ws/devel/lib/fastlio_bridge'/name for name in (
        'static_map_builder_node','mavros_avoidance_controller_node','loop_closure_backend_node',
        'online_loop_mapper_node')]
    binaries += [root/'AstraDrone_ros1_ws/devel/lib/fast_lio/fastlio_mapping']
    manifest['binary_sha256'] = {str(p): sha256_file(p) for p in binaries if p.is_file()}
    write_manifest(prefix, manifest)
    return manifest


def write_manifest(prefix, manifest):
    path = Path(str(prefix)+'_evidence_manifest.json')
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def run_with_cleanup(runner, operation, *args):
    """Also clean up when saving metadata or finalizing a recorder raises."""
    try:
        return operation(*args)
    finally:
        runner.stop()


def snapshot_parameters(prefix, getter, manifest):
    namespaces = ('/static_map_builder','/mapping','/common','/preprocess',
                  '/filter_size_surf','/filter_size_map','/point_filter_num','/max_iteration',
                  '/feature_extract_enable','/cube_side_length','/dynamic_filter',
                  '/dynamic_detector','/dynamic_cluster','/semantic_fusion',
                  '/warehouse_explorer','/mavros_avoidance_controller',
                  '/loop_closure_backend','/static_collision_risk','/collision_risk',
                  '/fastlio_bridge','/fastlio_cloud_bridge','/online_loop_mapper','/dufomap_adapter','/dual_map_manager')
    values = {name: getter(name,{}) for name in namespaces}
    path = Path(str(prefix)+'_runtime_parameters.json')
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(values,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)
    manifest['runtime_parameters'] = dict(path=str(path.resolve()),sha256=sha256_file(path))
    write_manifest(prefix,manifest)
    return values


def inspect_bag(path):
    """Open a closed bag and record actual counts, rather than process exit alone."""
    import rosbag
    with rosbag.Bag(str(path), 'r') as bag:
        counts = {name: int(info.message_count)
                  for name, info in bag.get_type_and_topic_info().topics.items()}
        types = {name: info.msg_type for name, info in bag.get_type_and_topic_info().topics.items()}
        return dict(topic_counts=counts, topic_types=types,start_time=bag.get_start_time(),
                    end_time=bag.get_end_time(), message_count=bag.get_message_count())


def wait_closed_bag(path, timeout=5.):
    """rosbag's Python launcher can exit before its native child renames .active."""
    deadline=time.monotonic()+timeout
    while not path.is_file() or Path(str(path)+'.active').exists():
        if time.monotonic()>=deadline:return False
        time.sleep(.05)
    return True


def finish_manifest(prefix, manifest, recorder_exit=None, configuration=None):
    manifest['configuration'] = configuration or {}
    if manifest['bag_requested']:
        path = Path(str(prefix)+'_evidence.bag')
        manifest.update(bag_path=str(path), recorder_exit=recorder_exit)
        try:
            if recorder_exit != 0 or not wait_closed_bag(path):
                raise ValueError('recorder failed or bag did not finalize')
            manifest.pop('bag_error',None)
            contents = inspect_bag(path)
            manifest.update(contents, bag_finalized=True, bag_sha256=sha256_file(path),
                            bag_bytes=path.stat().st_size)
            compact=manifest.get('recording_policy',{}).get('profile')=='compact'
            manifest['recording_integrity_valid']=True
            if compact:
                stats_path=Path(str(prefix)+'_evidence_recording.json')
                stats=json.loads(stats_path.read_text())
                manifest['recorder_statistics']=dict(path=str(stats_path),sha256=sha256_file(stats_path),
                    queue_overflow=stats.get('queue_overflow'),write_errors=stats.get('write_errors'))
                if not stats.get('finalized') or stats.get('queue_overflow') or stats.get('write_errors'):
                    raise ValueError('Compact recorder did not finalize cleanly or dropped queued data')
            manifest['raw_lio_replay_inputs_available']=all(contents['topic_counts'].get(t,0)>0
                for t in ('/livox/lidar','/livox/imu'))
            manifest['full_callback_timing_evidence']=not compact
            base_available = all(
                contents['topic_counts'].get(name, 0) > 0 for name in REQUIRED_REPLAY_TOPICS)
            params = {}
            snapshot = manifest.get('runtime_parameters')
            if snapshot:
                if sha256_file(snapshot['path']) != snapshot['sha256']:
                    raise ValueError('runtime parameter snapshot changed')
                params = json.loads(Path(snapshot['path']).read_text()).get('/static_map_builder',{})
            objects_topic = params.get('objects_topic','/uav1/semantic_dynamic_objects')
            objects_available = (contents['topic_counts'].get(objects_topic,0)>0 and
                contents['topic_types'].get(objects_topic)=='fastlio_bridge/DynamicObjectArray')
            expected_types = dict(zip(REQUIRED_REPLAY_TOPICS,
                ('rosgraph_msgs/Clock','nav_msgs/Odometry','sensor_msgs/PointCloud2')))
            base_available &= all(contents['topic_types'].get(k)==v for k,v in expected_types.items())
            manifest.update(base_scan_odom_available=base_available,
                mapper_objects_topic=objects_topic,
                ghost_replay_inputs_available=bool(base_available and objects_available),
                replay_inputs_available=bool(base_available and (objects_available or
                    params.get('enable_dynamic_map_clearing') is False)),
                replay_timestamp_validation='requires replay preflight')
            if manifest.get('run_options',{}).get('loop_closure_audit'):
                manifest['loop_audit_valid'] = all(contents['topic_counts'].get(t,0)>0 for t in (
                    '/uav1/loop_closure/odom','/uav1/loop_closure/path',
                    '/uav1/loop_closure/raw_path','/uav1/loop_closure/map',
                    '/uav1/loop_closure/diagnostics'))
            if manifest.get('run_options',{}).get('online_loop'):
                manifest['online_loop_valid']=all(contents['topic_counts'].get(t,0)>0 for t in (
                    '/uav1/loop_closure/graph','/uav1/loop_slam/snapshot','/uav1/loop_slam/status',
                    '/uav1/loop_slam/input_ack','/uav1/loop_slam/processed_scan'))
        except Exception as exc:
            manifest['bag_error'] = str(exc)
            manifest['recording_integrity_valid']=False
    write_manifest(prefix, manifest)
    return manifest


def prune_previous_recordings(prefix, keep=1):
    """After a verified new recording, apply the user's bounded retention rule.

    Maps, results and historical manifests are never removed. Failed/incomplete
    new recordings cannot trigger deletion of the previous usable recording.
    """
    prefix=Path(prefix)
    if keep<0:raise ValueError('Recording retention must be nonnegative')
    if keep==0:return dict(enabled=False,deleted=[])
    if prefix.parent.name not in ('half_warehouse_online_loop_ten_minute_runs','half_warehouse_joint_channel_runs','half_warehouse_dufomap_runs','half_warehouse_dual_map_runs'):
        return dict(enabled=False,reason='outside_two_demo_directories',deleted=[])
    manifest=json.loads(Path(str(prefix)+'_evidence_manifest.json').read_text())
    current=Path(str(prefix)+'_evidence.bag')
    if not (current.is_file() and manifest.get('bag_finalized') and
            manifest.get('recording_integrity_valid',True) and not manifest.get('bag_error') and
            manifest.get('source_unchanged_during_run')):
        return dict(enabled=True,skipped='new_recording_not_verified',deleted=[])
    if sha256_file(current)!=manifest['bag_sha256']:
        return dict(enabled=True,skipped='new_recording_hash_mismatch',deleted=[])
    pattern=re.compile(r'^warehouse_half_(?:dual_map|dufomap|ten_minute|joint_(?:channel|simple))_(\d{8}_\d{6})_evidence\.bag$')
    files=[]
    for p in prefix.parent.glob('*_evidence.bag'):
        match=pattern.match(p.name)
        if match and p!=current and p.is_file() and not p.is_symlink() and not Path(str(p)+'.active').exists():
            files.append((match.group(1),p))
    files.sort(reverse=True)
    removed=[]
    for _,p in files[max(0,keep-1):]:
        size=p.stat().st_size
        p.unlink();removed.append(dict(path=str(p),bytes=size))
    result=dict(enabled=True,keep=keep,current=str(current),deleted=removed,
                note='Only previous raw recordings removed; maps/results/manifests preserved.')
    target=Path(str(prefix)+'_evidence_retention.json')
    temporary=target.with_suffix('.tmp');temporary.write_text(json.dumps(result,indent=2)+'\n');temporary.replace(target)
    return result
