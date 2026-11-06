#!/usr/bin/env python3
"""Isolated SITL intervention after a measured pedestrian ghost remains.

Gazebo truth is used only by this experiment trigger and evaluator; it is never
published to the mapper or explorer. Both arms run the same ghost-aware mapper.
The baseline hovers at the decision pose; the revisit arm uses the existing
explorer. Both receive up to 120 sim seconds before the return deadline;
the explorer may return sooner when no safe view remains.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import SetBool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix')
    parser.add_argument('--arm', choices=('baseline', 'revisit'), required=True)
    args = parser.parse_args()
    prefix = Path(args.prefix)
    events = Path(str(prefix) + '_challenge.jsonl')
    outcome = Path(str(prefix) + '_challenge_result.json')
    state = {}
    rospy.init_node('ghost_revisit_challenge_' + args.arm)

    def receive(message, name):
        state[name] = message

    for name, topic, kind in (
        ('map', '/uav1/static_map', PointCloud2),
        ('suspects', '/uav1/map/ghost_suspects', PointCloud2),
        ('ghost_status', '/uav1/map/ghost_revisit_status', String),
        ('mission', '/uav1/exploration/status', String),
        ('gazebo', '/gazebo/model_states', ModelStates)):
        rospy.Subscriber(topic, kind, receive, name, queue_size=1,
                         buff_size=16000000 if kind is PointCloud2 else 65536)

    def record(kind, data):
        entry = dict(event=kind, wall_time=time.time(), sim_time=rospy.Time.now().to_sec(), **data)
        with events.open('a', encoding='utf-8') as output:
            output.write(json.dumps(entry, ensure_ascii=False) + '\n')
        return entry

    def snapshot(rotation, translation):
        if not all(key in state for key in ('map', 'suspects', 'ghost_status', 'mission', 'gazebo')):
            return None
        now = rospy.Time.now().to_sec()
        if now <= 0 or any(now-state[k].header.stamp.to_sec() > 2.5 for k in ('map', 'suspects')):
            return None
        try:
            mission = json.loads(state['mission'].data)
            ghosts = json.loads(state['ghost_status'].data)
            person_pose = state['gazebo'].pose[state['gazebo'].name.index('0')]
        except (ValueError, KeyError):
            return None

        def roi_count(message):
            points = np.asarray(list(point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=float).reshape((-1, 3))
            if not len(points):
                return 0
            world = (points-translation) @ rotation
            near = (np.linalg.norm(world[:, :2]-[-8., -4.], axis=1) < .4) & \
                   (world[:, 2] > .3) & (world[:, 2] < 1.9)
            return int(near.sum())

        person = person_pose.position
        return dict(person_world_xyz=[person.x, person.y, person.z],
                    person_departure_xy_m=math.hypot(person.x+8., person.y+4.),
                    map_roi_points=roi_count(state['map']),
                    suspect_roi_points=roi_count(state['suspects']),
                    ghost_status=ghosts,
                    mission_phase=mission.get('mission_phase'),
                    mission_sim_seconds=mission.get('mission_sim_seconds'),
                    travel_xy_m=mission.get('travel_xy_m'),
                    finish_reason=mission.get('finish_reason'))

    start_wall = time.monotonic()
    last_sample_sim = -math.inf
    decision = None
    rotation = translation = None
    baseline_disabled = False
    while not rospy.is_shutdown():
        if args.arm == 'baseline' and rospy.has_param('/warehouse_explorer/ghost_revisit_enabled') \
                and not baseline_disabled:
            rospy.set_param('/warehouse_explorer/ghost_revisit_enabled', False)
            baseline_disabled = True
        if rotation is None:
            path = Path(str(prefix) + '_alignment.json')
            if path.is_file():
                alignment = json.loads(path.read_text(encoding='utf-8'))
                rotation = np.asarray(alignment['map_from_world_rotation'], dtype=float)
                translation = np.asarray(alignment['map_from_world_translation'], dtype=float)
        row = snapshot(rotation, translation) if rotation is not None else None
        if row and rospy.Time.now().to_sec()-last_sample_sim >= 5.:
            record('sample', row)
            last_sample_sim = rospy.Time.now().to_sec()
        if row and decision is None and row['mission_phase'] == 'EXPLORING' and \
                row['mission_sim_seconds'] is not None and row['mission_sim_seconds'] >= 40. and \
                row['person_departure_xy_m'] >= 2.5 and row['map_roi_points'] >= 5 and \
                row['suspect_roi_points'] >= 1 and row['ghost_status'].get('pending_voxels', 0) >= 1:
            # These are experiment-only runtime parameters. The normal demo is
            # unaffected, and both arms retain the same controller/safety map.
            limit = float(row['mission_sim_seconds']) + 120.
            if args.arm == 'baseline':
                rospy.set_param('/warehouse_explorer/ghost_revisit_enabled', False)
                rospy.wait_for_service('/uav1/exploration/set_enabled', timeout=5.)
                answer = rospy.ServiceProxy('/uav1/exploration/set_enabled', SetBool)(False)
                if not answer.success:
                    raise RuntimeError('Baseline hover request was rejected')
            rospy.set_param('/warehouse_explorer/max_mission_seconds', limit)
            decision = record('decision', dict(**row, arm=args.arm,
                                                mission_limit_sim_seconds=limit))
            outcome.write_text(json.dumps(dict(arm=args.arm, trigger='matched_residual_roi',
                                               decision=decision), indent=2) + '\n', encoding='utf-8')
        if decision and row and args.arm == 'baseline' and \
                row['mission_sim_seconds'] is not None and \
                row['mission_sim_seconds'] >= decision['mission_sim_seconds'] + 115.:
            # Give the existing return planner a few seconds before deadline.
            answer = rospy.ServiceProxy('/uav1/exploration/set_enabled', SetBool)(True)
            if not answer.success:
                raise RuntimeError('Baseline resume request was rejected')
            record('baseline_resumed', row)
            decision = dict(decision, mission_sim_seconds=math.inf)
        if row and decision is None and (row['mission_sim_seconds'] or 0) >= 240.:
            record('no_decision', row)
            outcome.write_text(json.dumps(dict(arm=args.arm, trigger='no_decision', last=row),
                                          indent=2) + '\n', encoding='utf-8')
            raise RuntimeError('No measured residual person ROI and suspect before 240 sim seconds')
        if decision is None and time.monotonic()-start_wall > 600.:
            record('no_decision_wall_timeout', row or {})
            outcome.write_text(json.dumps(dict(arm=args.arm, trigger='no_decision_wall_timeout',
                                               last=row), indent=2) + '\n', encoding='utf-8')
            raise RuntimeError('Challenge decision wall timeout')
        time.sleep(.5)


if __name__ == '__main__':
    main()
