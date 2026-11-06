#!/usr/bin/env python3
"""Labelled same-input C++ integration test; synthetic returns, not a flight.

Each arm gets identical scans/odometry/tracks. Only the candidate guard differs.
The production mapper must keep occupancy bit-for-bit identical and preserve
actionable human ghosts, including a paused person 8 cm in front of a shelf.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=11359)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % args.port
    import rosgraph
    import rospy
    from fastlio_bridge.msg import DynamicObject, DynamicObjectArray
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, String
    from std_srvs.srv import Empty

    if rosgraph.is_master_online():
        raise RuntimeError('Refuse to reuse an existing master')
    processes, handles = [], []
    checks, snapshots = {}, {}
    names = ('unguarded', 'guarded')
    def start(name, command):
        handle = (args.output/(name+'.log')).open('w')
        handles.append(handle)
        processes.append(subprocess.Popen(command, stdout=handle,
            stderr=subprocess.STDOUT, start_new_session=True))
    try:
        start('core', ['roscore', '-p', str(args.port)])
        deadline = time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:
                raise RuntimeError('master timed out')
            time.sleep(.1)
        rospy.init_node('surface_guard_labelled_test', disable_signals=True)
        maps, histories, statuses = {}, {}, {}
        pubs = {}
        for name in names:
            prefix = '/'+name
            command = ['rosrun', 'fastlio_bridge', 'static_map_builder_node', '__name:='+name]
            for param, suffix in dict(cloud_topic='scan', odom_topic='odom',
                objects_topic='objects', output_topic='map', legacy_output_topic='old',
                stable_output_topic='stable', local_output_topic='local',
                free_space_output_topic='free', suspect_output_topic='suspects',
                suspect_status_topic='status', ghost_history_topic='history').items():
                command.append('_%s:=%s/%s' % (param, prefix, suffix))
            command += ['_evidence_only_clearing:=true', '_enable_ghost_revisit:=true',
                '_ghost_static_surface_guard:='+('true' if name=='guarded' else 'false'),
                '_publish_rate:=20', '_ghost_departure_seconds:=0.5']
            start(name, command)
            rospy.wait_for_service(prefix+'/clear', 15.)
            rospy.Subscriber(prefix+'/map', PointCloud2,
                lambda m,n: maps.__setitem__(n, (m.header.stamp.to_nsec(),
                    sorted(tuple(p) for p in point_cloud2.read_points(m,
                        field_names=('x','y','z'), skip_nans=True)))), name)
            rospy.Subscriber(prefix+'/history', String,
                lambda m,n: histories.__setitem__(n,json.loads(m.data)), name)
            rospy.Subscriber(prefix+'/status', String,
                lambda m,n: statuses.__setitem__(n,json.loads(m.data)), name)
            pubs[name] = {key: rospy.Publisher(prefix+'/'+key, typ, queue_size=1)
                for key,typ in [('scan',PointCloud2),('odom',Odometry),
                                ('objects',DynamicObjectArray)]}
        time.sleep(.6)
        stamp_ns = 0
        def pair(points, count=1, tracked=None):
            nonlocal stamp_ns
            for _ in range(count):
                header = Header(stamp=rospy.Time.now(), frame_id='map')
                stamp_ns = header.stamp.to_nsec()
                odom = Odometry(header=header)
                odom.pose.pose.position.z = 1.025
                odom.pose.pose.orientation.w = 1.
                objects = DynamicObjectArray(header=header)
                if tracked is not None:
                    item = DynamicObject(id=1)
                    item.pose.position.x,item.pose.position.y,item.pose.position.z=tracked
                    item.twist.linear.x=.3
                    objects.objects.append(item)
                for n in names:
                    pubs[n]['odom'].publish(odom)
                    pubs[n]['objects'].publish(objects)
                time.sleep(.025)
                scan = point_cloud2.create_cloud_xyz32(header,points)
                for n in names:
                    pubs[n]['scan'].publish(scan)
                deadline = time.monotonic()+3.
                while any(maps.get(n,(0,))[0]!=stamp_ns for n in names):
                    if time.monotonic()>deadline:
                        raise AssertionError('map receipt timed out')
                    time.sleep(.01)
                time.sleep(.07)

        def record(name, point):
            key=[math.floor(struct.unpack('<f',struct.pack('<f',v))[0]/.15) for v in point]
            return next((r for r in histories[name]['records']
                if r['key']==key),None)

        def snapshot(label, point):
            deadline = time.monotonic()+3.
            while any(histories.get(n,{}).get('map_stamp')!=stamp_ns/1e9 for n in names):
                if time.monotonic()>deadline:
                    raise AssertionError('history receipt timed out')
                time.sleep(.01)
            snapshots[label]={n:dict(record=record(n,point),status=statuses[n]) for n in names}
            checks[label+'_identical_occupied_map']=maps[names[0]][1]==maps[names[1]][1]
            print(label, flush=True)
            return snapshots[label]['guarded']['record']

        def reset():
            for n in names:
                rospy.ServiceProxy('/'+n+'/clear',Empty)()
            time.sleep(.15)

        shelf=(2.11,.075,1.025)
        patch=[(shelf[0],shelf[1]+i*.15,shelf[2]+j*.15)
               for i in range(-6,7) for j in range(-5,6)]
        away=[(0.,-3.,1.025)]
        pair(patch,18)
        pair(patch,3,tracked=shelf)
        pair(patch,18)
        r=snapshot('shelf_face',shelf)
        checks['shelf_excluded_from_revisit']=r['static_surface_suppressed'] and not r['actionable']
        checks['shelf_not_falsely_cleared']=r['state']=='suspected' and r['occupied_now']
        checks['unguarded_shelf_remains_candidate']=snapshots['shelf_face']['unguarded']['record']['actionable']
        pair(patch,tracked=shelf)
        r=snapshot('new_encounter',shelf)
        checks['new_track_revokes_certificate']=not r['static_surface_suppressed'] and r['surface_positive_frames']==0

        reset()
        pair(patch,18)
        pair(patch,3,tracked=shelf)
        pair(away,8)
        pair(patch)
        time.sleep(.6)  # allow the bounded asynchronous geometry timer once
        r=snapshot('one_positive_scan',shelf)
        checks['one_scan_insufficient']=r['actionable'] and not r['static_surface_suppressed']
        evidence_frames=r['surface_consistent_frames']
        time.sleep(1.5)
        r=snapshot('cached_scan_only',shelf)
        checks['timer_cannot_invent_evidence']=(r['surface_consistent_frames']==evidence_frames
            and r['actionable'] and not r['static_surface_suppressed'])

        for offset in (.08,.31):
            reset()
            person=(shelf[0]-offset,shelf[1],shelf[2])
            scene=patch+[person]
            pair(scene,18)
            pair(scene,3,tracked=person)
            pair(scene,18)
            label='paused_person_%.2fm'%offset
            r=snapshot(label,person)
            checks[label+'_retained_candidate']=r['actionable'] and not r['static_surface_suppressed']

        reset()
        pair([shelf],18)
        pair([shelf],3,tracked=shelf)
        pair(patch,6)
        r=snapshot('newly_seen_patch',shelf)
        checks['immature_patch_cannot_certify']=r['actionable'] and not r['static_surface_suppressed']
        pair(patch,24)
        r=snapshot('new_patch_repeatedly_confirmed',shelf)
        checks['fresh_mature_patch_can_certify']=r['static_surface_suppressed'] and not r['actionable']

        reset()
        pair(patch,18)
        pair(patch,3,tracked=shelf)
        pair(patch,18)
        # Current rays pass through the queried point. Stop after the first
        # negative observation, before the unchanged clearing threshold fires.
        through=[(3.11,shelf[1]*3.11/shelf[0],shelf[2])]
        for _ in range(15):
            pair(through)
            r=snapshot('free_contradiction',shelf)
            if r and r['free_evidence_frames']:
                break
        checks['free_ray_revokes_certificate']=bool(r and r['free_evidence_frames'] and
            not r['static_surface_suppressed'] and r['actionable'])
        pair(through,10)
        r=snapshot('ghost_cleared',shelf)
        checks['normal_negative_evidence_still_clears']=r['state']=='cleared' and not r['occupied_now']
        report=dict(scope=__doc__,checks=checks,snapshots=snapshots,
                    passed=all(checks.values()),checks_count=len(checks))
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(passed=report['passed'],checks=checks),indent=2))
        if not report['passed']:
            raise AssertionError('controlled surface guard regression failed')
    except Exception as error:
        (args.output/'failure.json').write_text(json.dumps(dict(error=str(error),
            checks=checks,snapshots=snapshots),indent=2)+'\n')
        raise
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
        for p in reversed(processes):
            try:p.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
        for h in handles:h.close()


if __name__=='__main__':
    main()
