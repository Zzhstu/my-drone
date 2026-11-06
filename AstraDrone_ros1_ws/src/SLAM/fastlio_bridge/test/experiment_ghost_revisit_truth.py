#!/usr/bin/env python3
"""Controlled, labelled ROS replay of ghost revisit versus no revisit.

Both arms run the same C++ mapper and receive identical scans through the
decision point. After that, only the revisit arm receives a scan through the
vacated person location. These are synthetic measurements, not Gazebo LiDAR.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from exploration_grid import ExplorationGrid
from ghost_revisit import select_revisit_view, suspect_groups


def contains(points, target, tolerance=.04):
    return any(math.dist(point, target) < tolerance for point in points)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--port', type=int, default=11349)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % args.port
    import rosgraph
    import rospy
    from fastlio_bridge.msg import DynamicObject, DynamicObjectArray
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, String
    from std_srvs.srv import Empty, Trigger

    if rosgraph.is_master_online():
        raise RuntimeError('Port already has a ROS master; refuse to use it')
    processes, handles = [], []
    def start(name, command):
        handle = (output / (name + '.log')).open('w')
        handles.append(handle)
        processes.append(subprocess.Popen(command, stdout=handle,
                                          stderr=subprocess.STDOUT,
                                          start_new_session=True))
    try:
        start('core', ['roscore', '-p', str(args.port)])
        deadline = time.monotonic() + 15.
        while not rosgraph.is_master_online():
            if time.monotonic() > deadline:
                raise RuntimeError('ROS master start timed out')
            time.sleep(.1)
        rospy.init_node('ghost_revisit_truth_experiment', disable_signals=True)
        names = ('baseline', 'revisit')
        for name in names:
            start(name, ['rosrun', 'fastlio_bridge', 'static_map_builder_node',
                '__name:=' + name, '_cloud_topic:=/' + name + '/scan',
                '_odom_topic:=/' + name + '/odom',
                '_objects_topic:=/' + name + '/objects',
                '_output_topic:=/' + name + '/map',
                '_legacy_output_topic:=/' + name + '/old',
                '_stable_output_topic:=/' + name + '/stable',
                '_local_output_topic:=/' + name + '/local',
                '_free_space_output_topic:=/' + name + '/free',
                '_suspect_output_topic:=/' + name + '/suspects',
                '_suspect_status_topic:=/' + name + '/status',
                '_ghost_history_topic:=/' + name + '/history',
                '_minimum_static_hits:=1', '_stable_minimum_static_hits:=3',
                '_stable_free_miss_threshold:=3',
                '_free_space_miss_threshold:=3',
                '_ghost_positive_endpoint_confirmation:='+('true' if name=='revisit' else 'false'),
                '_publish_rate:=20', '_ghost_departure_seconds:=0.5',
                '_evidence_only_clearing:=true', '_enable_ghost_revisit:=true'])
            rospy.wait_for_service('/' + name + '/clear', 10.)
        saved_map = output/'revisit_map.pcd'
        start('saver',['rosrun','fastlio_bridge','save_navigation_map.py',
              '__name:=ghost_replay_saver',
              '_topic:=/revisit/map',
              '_scan_topic:=/revisit/scan',
              '_ghost_history_topic:=/revisit/history',
              '_require_ghost_history:=true',
              '_output:='+str(saved_map)])
        rospy.wait_for_service('/demo/save_map',10.)
        pubs = {name: {
            'scan': rospy.Publisher('/' + name + '/scan', PointCloud2, queue_size=1),
            'odom': rospy.Publisher('/' + name + '/odom', Odometry, queue_size=1),
            'objects': rospy.Publisher('/' + name + '/objects', DynamicObjectArray,
                                       queue_size=1)} for name in names}
        maps, suspects, statuses, histories = {}, {}, {}, {}
        def cloud_callback(message, target_and_name):
            target, name = target_and_name
            target[name] = list(point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True))
        for name in names:
            rospy.Subscriber('/' + name + '/map', PointCloud2, cloud_callback,
                             (maps, name), queue_size=1)
            rospy.Subscriber('/' + name + '/suspects', PointCloud2, cloud_callback,
                             (suspects, name), queue_size=1)
            rospy.Subscriber('/' + name + '/status', String,
                             lambda message, n: statuses.__setitem__(n, json.loads(message.data)),
                             name, queue_size=1)
            rospy.Subscriber('/' + name + '/history', String,
                             lambda message, n: histories.__setitem__(n, json.loads(message.data)),
                             name, queue_size=1)
        time.sleep(.5)

        def send(name, points, repeat=1, tracked=None, origin=(0., 0.)):
            for _ in range(repeat):
                stamp = rospy.Time.now()
                header = Header(stamp=stamp, frame_id='map')
                pose = Odometry(header=header)
                pose.pose.pose.position.x, pose.pose.pose.position.y = origin
                pose.pose.pose.position.z = 1.02
                pose.pose.pose.orientation.w = 1.
                pubs[name]['odom'].publish(pose)
                time.sleep(.015)
                objects = DynamicObjectArray(header=header)
                if tracked is not None:
                    item = DynamicObject(id=1)
                    item.pose.position.x, item.pose.position.y, item.pose.position.z = tracked
                    item.twist.linear.x = .3
                    objects.objects.append(item)
                pubs[name]['objects'].publish(objects)
                pubs[name]['scan'].publish(point_cloud2.create_cloud_xyz32(header, points))
                time.sleep(.12)

        def pair(points, repeat=1, tracked=None):
            for _ in range(repeat):
                for name in names:
                    send(name, points, tracked=tracked)

        def snapshot():
            time.sleep(.15)
            # PointCloud2 XYZ is float32 before the C++ voxel floor; 1.05m is
            # just below an exact cell boundary after that conversion.
            person_key = tuple(math.floor(struct.unpack('<f',struct.pack('<f',value))[0]/.15)
                               for value in person)
            def person_record(name):
                return next((record for record in histories.get(name,{}).get('records',[])
                             if tuple(record['key']) == person_key),None)
            return {name: {
                'map_points': len(maps.get(name, [])),
                'person_ghost_present': contains(maps.get(name, []), person),
                'shelf_present': contains(maps.get(name, []), shelf),
                'person_suspect_present': contains(suspects.get(name, []), person),
                'person_history': person_record(name),
                'status': dict(statuses.get(name, {}))} for name in names}

        def reset():
            for name in names:
                rospy.ServiceProxy('/' + name + '/clear', Empty)()
            time.sleep(.2)

        person = (1.05, .015, 1.02)
        shelf = (2.1, 1.5, 1.02)
        ground_truth = {'person_initial': list(person),
                        'person_after_departure': 'absent',
                        'shelf': list(shelf),
                        'scene': 'labelled synthetic returns; room free cells assumed observed'}
        result = {'protocol': 'same C++ mapper parameters and pre-decision scans; '
                    'baseline stays and scans away from vacated point; revisit moves to selected view',
                  'ground_truth': ground_truth}

        # Old occupied return. The detector then confirms a moving person.
        pair([person, shelf], repeat=3)
        pair([person, shelf], repeat=3, tracked=person)
        pair([(0., 2., 1.02)], repeat=5)
        before = snapshot()
        if not all(before[n]['person_ghost_present'] and
                   before[n]['person_suspect_present'] for n in names):
            raise AssertionError('Matched pre-decision ghost/suspect was not created')
        grid = ExplorationGrid([-3., -3., 5., 4.], .2, .8, .4)
        grid.known[:] = True  # explicit synthetic free-space assumption
        grid.update_obstacles(maps.get('revisit', []))
        distances, _ = grid.search(grid.cell((0., 0.)))
        groups = suspect_groups(suspects.get('revisit', []), 1.02)
        view, group = select_revisit_view(grid, distances, groups, (0., 0.), {})
        if view is None:
            raise AssertionError('No reachable safe revisit view in synthetic known room')
        xy = grid.xy(view)
        length = math.dist(person[:2], xy)
        behind = (person[0] + 1.2 * (person[0]-xy[0])/length,
                  person[1] + 1.2 * (person[1]-xy[1])/length, person[2])
        start = time.monotonic()
        for _ in range(22):
            send('baseline', [(0., 2., 1.02)])
            send('revisit', [behind], origin=xy)
        replay_wall = time.monotonic() - start
        after = snapshot()
        save_reply = rospy.ServiceProxy('/demo/save_map',Trigger)()
        history_file = saved_map.with_name(saved_map.stem+'_ghost_history.json')
        saved_meta = json.loads(saved_map.with_suffix('.json').read_text()) if save_reply.success else {}
        saved_history = json.loads(history_file.read_text()) if save_reply.success else {}
        saved_key = tuple(math.floor(struct.unpack('<f',struct.pack('<f',value))[0]/.15)
                          for value in person)
        saved_person = next((record for record in saved_history.get('records',[])
                             if tuple(record['key'])==saved_key),None)
        result['saved_map_history'] = {
            'service_success':save_reply.success,
            'person_state':saved_person['state'] if saved_person else None,
            'history_sha_matches':bool(save_reply.success and
                saved_meta.get('ghost_history_sha256')==
                hashlib.sha256(history_file.read_bytes()).hexdigest())}
        result['vacated_person'] = {'before': before, 'after': after,
            'selected_view_xy': list(xy), 'selected_view_safe': bool(grid.safe[view]),
            'selected_view_reachable': bool(math.isfinite(float(distances[view]))),
            'estimated_one_way_grid_path_m': float(distances[view]),
            'replay_pair_wall_seconds': replay_wall,
            'extra_scans_from_view': 22}

        # The same voxel can be occupied again. Its previous clearing remains
        # attached to the new suspicion and raises the revisit evidence weight.
        send('revisit',[person,shelf],repeat=3,tracked=person)
        send('revisit',[(0.,2.,1.02)],repeat=5)
        repeated = snapshot()['revisit']['person_history']
        result['repeated_episode'] = repeated
        send('revisit',[behind],repeat=22,origin=xy)
        repeated_cleared = snapshot()['revisit']['person_history']
        result['repeated_episode_cleared'] = repeated_cleared

        # No new ray crosses the old voxel: keeping an unresolved label is correct.
        reset()
        pair([person, shelf], repeat=3)
        pair([person, shelf], repeat=3, tracked=person)
        pair([(0., 2., 1.02)], repeat=10)
        result['unobserved_after_departure'] = snapshot()

        # A well-supported shelf near a track must not be erased by the track.
        reset()
        pair([shelf], repeat=10)
        pair([shelf], repeat=4, tracked=shelf)
        result['false_track_at_shelf'] = snapshot()

        # The old lifetime hits<=8 gate missed a long-observed pedestrian.
        # Its track may now create a low-priority hypothesis, but only new
        # free-ray evidence may remove the measured occupancy.
        reset()
        pair([person,shelf], repeat=12)
        pair([person,shelf], repeat=3, tracked=person)
        pair([(0.,2.,1.02)], repeat=5)
        result['high_hit_person'] = snapshot()

        # The same static surface can straddle two 15cm keys. A real return
        # 4cm away must be positive evidence even though endpoint margin means
        # no free ray ever traverses the old point behind it. Nothing is erased.
        reset()
        pair([person,shelf],repeat=3,tracked=person)
        pair([(0.,2.,1.02)],repeat=5)
        for n in names:
            send(n,[(person[0]+.04,person[1],person[2]),shelf],repeat=4,origin=(0.,.0))
            send(n,[(person[0]+.04,person[1],person[2]),shelf],repeat=4,origin=(0.,.7))
        boundary=snapshot();result['near_point_positive_multiview']=boundary

        # A 12cm-neighbor is outside the 6cm point tolerance and must not
        # certify the original hypothesis, even with a view baseline.
        reset()
        pair([person,shelf],repeat=3,tracked=person)
        pair([(0.,2.,1.02)],repeat=5)
        for n in names:
            send(n,[(person[0]+.12,person[1],person[2]),shelf],repeat=4,origin=(0.,0.))
            send(n,[(person[0]+.12,person[1],person[2]),shelf],repeat=4,origin=(0.,.7))
        distant=snapshot();result['distant_endpoint_not_confirmation']=distant

        vacated = result['vacated_person']
        unobserved = result['unobserved_after_departure']
        rack = result['false_track_at_shelf']
        result['checks'] = {
            'matched_before_revisit': before['baseline']['status'] == before['revisit']['status'],
            'baseline_residual_ghost': after['baseline']['person_ghost_present'],
            'revisit_removed_ghost': not after['revisit']['person_ghost_present'],
            'revisit_cleared_by_measured_free_rays':
                after['revisit']['status'].get('cleared_by_free_evidence', 0) >
                before['revisit']['status'].get('cleared_by_free_evidence', 0),
            'cleared_voxel_history_retained':
                after['revisit']['person_history'] is not None and
                after['revisit']['person_history']['state']=='cleared' and
                after['revisit']['person_history']['cleared_episodes']>=1,
            'final_map_history_saved_and_linked':
                result['saved_map_history']['service_success'] and
                result['saved_map_history']['person_state']=='cleared' and
                result['saved_map_history']['history_sha_matches'],
            'previous_clear_raises_new_candidate_weight':
                repeated is not None and repeated['state']=='suspected' and
                repeated['cleared_episodes']>=1 and
                repeated['suspected_episodes']>=2 and
                repeated['priority']>1.,
            'second_clear_keeps_both_episodes':
                repeated_cleared is not None and
                repeated_cleared['state']=='cleared' and
                repeated_cleared['cleared_episodes']>=2,
            'shelf_preserved_in_both': all(after[n]['shelf_present'] for n in names),
            'unobserved_ghost_reported_unresolved': all(
                unobserved[n]['person_ghost_present'] and
                unobserved[n]['person_suspect_present'] for n in names),
            'false_track_shelf_preserved': all(rack[n]['shelf_present'] for n in names),
            'high_hit_person_remains_candidate': all(
                result['high_hit_person'][n]['person_suspect_present'] and
                result['high_hit_person'][n]['person_history'] is not None and
                result['high_hit_person'][n]['person_history']['state']=='suspected'
                for n in names),
            'near_point_positive_requires_multiview_and_keeps_occupancy':
                boundary['revisit']['person_history']['state']=='reobserved_occupied' and
                boundary['revisit']['person_history']['resolution_reason']=='measured_near_point_multiview' and
                boundary['revisit']['person_ghost_present'] and boundary['revisit']['shelf_present'] and
                boundary['baseline']['person_history']['state']=='suspected',
            'distant_endpoint_cannot_confirm':all(distant[n]['person_history']['state']=='suspected' for n in names)}
        result['all_checks_passed'] = all(result['checks'].values())
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False,
                                                       indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        return 0 if result['all_checks_passed'] else 1
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    raise SystemExit(main())
