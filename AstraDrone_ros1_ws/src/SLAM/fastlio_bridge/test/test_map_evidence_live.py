#!/usr/bin/env python3
"""Real C++ node regression on an isolated ROS master, no Gazebo/truth injection.

Runs legacy and evidence-only nodes on identical synthetic measured returns.
Logs and quantitative outcomes remain in the requested output directory.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from exploration_grid import ExplorationGrid
from ghost_revisit import select_revisit_view, suspect_groups


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--port',type=int,default=11339)
    args=parser.parse_args()
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.port
    import rosgraph
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    from std_msgs.msg import String
    from std_srvs.srv import Empty
    from fastlio_bridge.msg import DynamicObjectArray,DynamicObject
    if rosgraph.is_master_online():
        raise RuntimeError('Isolated master port already in use; refusing to touch it')
    processes=[]
    handles=[]
    result={}
    def start(name,cmd):
        f=(output/(name+'.log')).open('w')
        handles.append(f)
        processes.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
    try:
        start('core',['roscore','-p',str(args.port)])
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline: raise RuntimeError('master timeout')
            time.sleep(.1)
        rospy.init_node('map_evidence_test',disable_signals=True)
        for name in ('legacy','evidence'):
            start(name,['rosrun','fastlio_bridge','static_map_builder_node','__name:='+name,
                '_cloud_topic:=/fixture/scan','_odom_topic:=/fixture/odom','_objects_topic:=/fixture/objects',
                '_output_topic:=/'+name+'/map','_legacy_output_topic:=/'+name+'/old',
                '_stable_output_topic:=/'+name+'/stable','_local_output_topic:=/'+name+'/local',
                '_free_space_output_topic:=/'+name+'/free','_minimum_static_hits:=1',
                '_suspect_output_topic:=/'+name+'/ghost_suspects',
                '_suspect_status_topic:=/'+name+'/ghost_status',
                '_ghost_history_topic:=/'+name+'/ghost_history',
                '_enable_ghost_revisit:='+('true' if name=='evidence' else 'false'),
                '_ghost_departure_seconds:=0.5',
                '_stable_minimum_static_hits:=3','_stable_free_miss_threshold:=3',
                '_free_space_miss_threshold:=3','_publish_rate:=20',
                '_evidence_only_clearing:='+('true' if name=='evidence' else 'false')])
            rospy.wait_for_service('/'+name+'/clear',10)
        scans=rospy.Publisher('/fixture/scan',PointCloud2,queue_size=1)
        odom=rospy.Publisher('/fixture/odom',Odometry,queue_size=1)
        objects=rospy.Publisher('/fixture/objects',DynamicObjectArray,queue_size=1)
        maps={}
        suspects={}
        statuses={}
        histories={}
        def receive(msg,name):
            maps[name]=list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True))
        for name in ('legacy','evidence'):
            rospy.Subscriber('/'+name+'/map',PointCloud2,receive,name,queue_size=1)
            rospy.Subscriber('/'+name+'/ghost_suspects',PointCloud2,
                lambda msg,n: suspects.__setitem__(n,list(point_cloud2.read_points(
                    msg,field_names=('x','y','z'),skip_nans=True))),name,queue_size=1)
            rospy.Subscriber('/'+name+'/ghost_status',String,
                lambda msg,n: statuses.__setitem__(n,json.loads(msg.data)),name,queue_size=1)
            rospy.Subscriber('/'+name+'/ghost_history',String,
                lambda msg,n: histories.__setitem__(n,json.loads(msg.data)),name,queue_size=1)
        time.sleep(.5)
        def send(points,repeat=1,dynamic=False,stale=False,dynamic_center=(2.1,.03,1.02),
                 origin_x=0.,origin_y=0.):
            for _ in range(repeat):
                stamp=rospy.Time.now()
                o=Odometry(header=Header(stamp=stamp-rospy.Duration(5 if stale else 0),frame_id='map'))
                o.pose.pose.position.x=origin_x
                o.pose.pose.position.y=origin_y
                o.pose.pose.position.z=1.02
                o.pose.pose.orientation.w=1.
                odom.publish(o)
                time.sleep(.015)
                m=DynamicObjectArray(header=Header(stamp=stamp,frame_id='map'))
                if dynamic:
                    obj=DynamicObject(id=1)
                    obj.pose.position.x,obj.pose.position.y,obj.pose.position.z=dynamic_center
                    obj.twist.linear.x=.3
                    m.objects.append(obj)
                objects.publish(m)
                scans.publish(point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id='map'),points))
                time.sleep(.12)
        def present(name,p):
            return any(sum((a-b)**2 for a,b in zip(q,p))<.001 for q in maps.get(name,[]))
        def reset():
            for name in ('legacy','evidence'):
                rospy.ServiceProxy('/'+name+'/clear',Empty)()
            time.sleep(.15)
        rack=(2.1,.03,1.02)
        send([rack],10)
        send([rack],8,dynamic=True)
        result['false_track_static_rack_preserved']=present('evidence',rack)
        result['legacy_false_track_erased_rack']=not present('legacy',rack)
        reset()
        person=(1.05,.015,1.02)
        send([person,rack],10)
        result['stationary_person_initially_in_map']=present('evidence',person)
        send([rack],22)
        result['vacated_person_cleared']=not present('evidence',person)
        result['background_rack_preserved']=present('evidence',rack)
        reset()
        # Same voxel traversal but 8cm away from the stored thin surface.
        edge=(1.06,.13,1.02)
        send([edge],10)
        send([(2.1,.1,1.02)],22)
        result['off_ray_thin_structure_preserved']=present('evidence',edge)
        reset()
        send([person,rack],10)
        send([rack],22,stale=True)
        result['stale_pose_does_not_clear']=present('evidence',person)
        reset()
        send([person,rack],10)
        send([(0.,2.,1.02)],6)  # wait for minimum observation age
        for _ in range(4):
            send([rack])
            send([(0.,2.,1.02)],9)  # no evidence here for >1 second
        result['separated_misses_do_not_accumulate']=present('evidence',person)
        send([rack],22)
        result['fresh_reobservation_still_clears']=not present('evidence',person)
        reset()
        send([person,rack],3)
        send([person,rack],3,dynamic=True,dynamic_center=person)
        # No ray through the vacated location: it must remain a hypothesis,
        # not be silently erased merely because the tracker stopped.
        send([(0.,2.,1.02)],10)
        result['unobserved_ghost_remains_suspect']=present('evidence',person) and any(
            sum((a-b)**2 for a,b in zip(q,person))<.001
            for q in suspects.get('evidence',[]))
        result['unobserved_ghost_has_no_free_ray_evidence']=any(
            r.get('state')=='suspected' and r.get('free_evidence_frames')==0
            for r in histories.get('evidence',{}).get('records',[]))
        # Close the mapping-planning loop on a synthetic fully observed room:
        # use the mapper's actual suspect cloud, then scan from the safe view
        # selected by the same planner helper used in the flight demo.
        grid=ExplorationGrid([-3.,-3.,5.,3.],.2,.8,.4)
        grid.known[:]=True
        grid.update_obstacles(maps.get('evidence',[]))
        distances,_=grid.search(grid.cell((0.,0.)))
        groups=suspect_groups(suspects.get('evidence',[]),1.02)
        view,_=select_revisit_view(grid,distances,groups,(0.,0.),{})
        result['safe_view_selected_for_ghost']=view is not None and bool(grid.safe[view])
        if view is not None:
            xy=grid.xy(view)
            length=sum((person[i]-xy[i])**2 for i in range(2))**.5
            behind=(person[0]+1.2*(person[0]-xy[0])/length,
                    person[1]+1.2*(person[1]-xy[1])/length,person[2])
            send([behind],22,origin_x=xy[0],origin_y=xy[1])
        result['revisited_ghost_cleared_by_rays']=(not present('evidence',person) and
            not suspects.get('evidence') and
            statuses.get('evidence',{}).get('cleared_by_free_evidence',0)>0)
        result['cleared_ghost_keeps_measured_free_ray_history']=any(
            r.get('state')=='cleared' and r.get('free_evidence_frames',0)>=3
            for r in histories.get('evidence',{}).get('records',[]))
        reset()
        send([person],3)
        send([person],3,dynamic=True,dynamic_center=person)
        send([person],6)
        send([person],3,origin_x=.8)
        result['reobserved_occupied_is_preserved']=(present('evidence',person) and
            not suspects.get('evidence') and
            statuses.get('evidence',{}).get('reobserved_occupied',0)>0)
        result['retained_ghost_keeps_post_departure_hit_history']=any(
            r.get('state')=='reobserved_occupied' and
            r.get('post_departure_hit_frames',0)>=3
            for r in histories.get('evidence',{}).get('records',[]))
        reset()
        # A far ray crosses the old point, while another current endpoint is
        # in its neighboring 15 cm voxel.  The neighbor must win even if the
        # far ray was processed first within this scan.
        boundary=(1.04,.015,1.02)
        neighbor=(1.09,.015,1.02)
        far=(2.1,.015,1.02)
        send([boundary],3)
        send([boundary],3,dynamic=True,dynamic_center=boundary)
        send([neighbor,far],12)
        result['neighbor_endpoint_preserves_old_surface']=present('evidence',boundary)
        result['neighbor_protection_is_audited']=any(
            r.get('state')=='suspected' and
            r.get('protected_endpoint_frames',0)>0
            for r in histories.get('evidence',{}).get('records',[]))
        boundary_records=[r for r in histories.get('evidence',{}).get('records',[])
                          if r.get('state')=='suspected' and
                          len(r.get('position',[]))==3 and
                          sum((a-b)**2 for a,b in zip(r.get('position',[]),boundary))<.001]
        result['measured_ray_crossing_audited']=any(
            r.get('sampled_ray_crossing_frames',0)>0 and
            r.get('close_endpoint_frames',0)>0 and
            not r.get('static_boundary_candidate',False)
            for r in boundary_records)
        # Close neighbor support from a second measured sensor origin can
        # lower revisit priority, but must not silently certify or delete it.
        send([neighbor,far],7,origin_x=.8)
        boundary_after=[r for r in histories.get('evidence',{}).get('records',[])
                        if r.get('state')=='suspected' and
                        len(r.get('position',[]))==3 and
                        sum((a-b)**2 for a,b in zip(r.get('position',[]),boundary))<.001]
        result['cross_view_boundary_support_is_tentative']=(
            bool(boundary_records) and present('evidence',boundary) and any(
                r.get('static_boundary_candidate') is True and
                r.get('close_endpoint_baseline',0)>.5 and
                r.get('closest_endpoint_distance',1)<.09 and
                r.get('priority',1)<boundary_records[0].get('priority',0)
                for r in boundary_after) and
            statuses.get('evidence',{}).get('pending_voxels',0)>0 and
            statuses.get('evidence',{}).get('static_boundary_candidates',0)>0)
        send([boundary,neighbor,far],dynamic=True,dynamic_center=boundary)
        result['new_track_invalidates_old_boundary_hint']=any(
            r.get('state')=='suspected' and
            len(r.get('position',[]))==3 and
            sum((a-b)**2 for a,b in zip(r['position'],boundary))<.001 and
            r.get('static_boundary_candidate') is False and
            r.get('close_endpoint_frames')==0
            for r in histories.get('evidence',{}).get('records',[]))
        reset()
        # A measured ray may cross the suspect voxel without passing close
        # enough to the stored point to count as free evidence.
        off_point=(1.06,.13,1.02)
        send([off_point],3)
        send([off_point],3,dynamic=True,dynamic_center=off_point)
        send([(2.1,.10,1.02)],12)
        result['crossed_but_off_point_does_not_clear']=(
            present('evidence',off_point) and any(
                r.get('state')=='suspected' and
                r.get('sampled_ray_crossing_frames',0)>0 and
                r.get('off_point_ray_frames',0)>0 and
                r.get('free_evidence_frames',0)==0
                for r in histories.get('evidence',{}).get('records',[])))
        result['all_passed']=all(result.values())
        (output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2))
        return 0 if result['all_passed'] else 1
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try: p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid,signal.SIGKILL)
                    p.wait()
        for f in handles: f.close()


if __name__=='__main__':
    raise SystemExit(main())
