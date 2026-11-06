#!/usr/bin/env python3
"""Same-input DUFOMap replay of ONE closed flight, no ROS master or new flight."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dufomap_core import DufoMapEngine


def replay(prefix, output, resolution=.15):
    import rosbag
    from dufomap_adapter import scan_xyz, body_pose
    s = str(prefix)
    reference = json.loads(Path(s+'_final_map.json').read_text())
    manifest = json.loads(Path(s+'_evidence_manifest.json').read_text())
    if not manifest.get('recording_integrity_valid') or not manifest.get('bag_finalized'):
        raise ValueError('a verified closed recording is required')
    mapping = reference['loop_mapping']; expected = mapping['input_frames']
    if not mapping['frozen'] or mapping['rebuilding'] or mapping['capacity_limited']:
        raise ValueError('reference map must have a complete frozen input boundary')
    calibration = json.loads(Path(s+'_sensor_geometry.json').read_text())
    engine = DufoMapEngine(resolution=resolution, lidar_origin=calibration['laser_origin_in_body'])
    ack, odometry = {}, {}
    began = time.monotonic()
    with rosbag.Bag(s+'_evidence.bag') as bag:
        for topic, msg, _ in bag.read_messages(topics=['/uav1/loop_slam/input_ack', '/uav1/fastlio/odom']):
            stamp = msg.stamp.to_nsec() if topic.endswith('input_ack') else msg.header.stamp.to_nsec()
            if topic.endswith('input_ack'):
                if stamp in ack:
                    raise ValueError('duplicate input ACK')
                ack[stamp] = msg.seq
            else:
                if stamp in odometry:
                    raise ValueError('duplicate odometry stamp')
                odometry[stamp] = body_pose(msg)
    if len(ack) != expected or sorted(ack.values()) != list(range(1, expected+1)):
        raise ValueError('canonical input ACKs are incomplete')
    with rosbag.Bag(s+'_evidence.bag') as bag:
        for _, scan, _ in bag.read_messages(topics=['/uav1/fastlio/registered_scan']):
            stamp = scan.header.stamp.to_nsec()
            if stamp not in ack:
                continue
            if scan.header.frame_id != 'map' or stamp not in odometry:
                raise ValueError('missing matching continuous-local odometry')
            p, q = odometry[stamp]
            engine.ingest(scan_xyz(scan), p, q, stamp, ack[stamp])
            if engine.frames % 500 == 0:
                print('DUFOMap replay %d/%d frames' % (engine.frames, expected), flush=True)
    if engine.frames != expected or abs(engine.last_stamp_ns*1e-9-reference['stamp']) > 1e-6:
        raise ValueError('canonical replay input/cutoff mismatch')
    report = engine.save(output, dict(mode='closed_bag_replay', source_flight_prefix=s,
        source_bag_sha256=manifest['bag_sha256'], canonical_input_complete=True,
        reference_input_frames=expected, loop_graph_revision=mapping['graph_revision'],
        loop_corrections_applied=False, planner_input=False,
        replay_wall_seconds=time.monotonic()-began))
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('prefix', type=Path); p.add_argument('--output-prefix', type=Path, required=True)
    p.add_argument('--resolution', type=float, default=.15)
    args = p.parse_args()
    replay(args.prefix, args.output_prefix, args.resolution)
