#!/usr/bin/env python3
"""One PX4 OFFBOARD publisher for the fixed simulation inspection mission."""
import json
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,ExtendedState
from mavros_msgs.srv import CommandBool,SetMode
from std_msgs.msg import String,Bool
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry import matrix_from_pose
from sensor_guard import count_in_tube
from tracking import relative_setpoint,NativeIntegrator
from mission import waypoints


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi


def main():
    rospy.init_node('roomgs_inspection_flight')
    warehouse=rospy.get_param('~scene','room')=='warehouse20'
    if warehouse:
        from warehouse20_mission import waypoints as inspection_waypoints
    else:inspection_waypoints=waypoints
    body_from_lidar=np.array(rospy.get_param('~body_from_lidar',np.eye(4).tolist()))
    latest={};cloud=[None]
    requested_abort=[None]
    rospy.Subscriber('/roomgs/abort',String,lambda m:requested_abort.__setitem__(0,m.data),queue_size=1)
    for topic,kind,key in [('/mavros/local_position/pose',PoseStamped,'pose'),('/mavros/state',State,'state'),
                            ('/mavros/extended_state',ExtendedState,'landed')]:
        rospy.Subscriber(topic,kind,lambda m,k=key:latest.update({k:(m,time.monotonic())}),queue_size=1)
    if warehouse:
        rospy.Subscriber(rospy.get_param('~native_odom_topic','/r3live/body_odom'),Odometry,lambda m:latest.update({'native':(m.pose.pose,time.monotonic())}),queue_size=1)
    def scan(m):
        fs={f.name:f.offset for f in m.fields}
        dtype=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,
                            offsets=[fs[k] for k in ['x','y','z']],itemsize=m.point_step))
        p=np.frombuffer(m.data,dtype=dtype,count=m.width*m.height)[::4]
        xyz=np.column_stack([p[k] for k in ['x','y','z']])
        if warehouse:
            valid=np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)>=.5)&(np.linalg.norm(xyz,axis=1)<=25.)
            xyz=xyz[valid]@body_from_lidar[:3,:3].T+body_from_lidar[:3,3]
        cloud[0]=(xyz,time.monotonic())
    rospy.Subscriber('/livox/lidar' if warehouse else '/cloud_registered',PointCloud2,scan,queue_size=1,buff_size=8000000)
    pub=rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    status=rospy.Publisher('/roomgs/mission',String,queue_size=1,latch=True)
    done=rospy.Publisher('/roomgs/finished',Bool,queue_size=1,latch=True)
    arm=rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    mode=rospy.ServiceProxy('/mavros/set_mode',SetMode)
    rate=rospy.Rate(20)
    required=('pose','state','landed','native') if warehouse else ('pose','state','landed')
    while not rospy.is_shutdown() and not all(k in latest for k in required):rate.sleep()
    current=latest['pose'][0].pose
    home=np.array([current.position.x,current.position.y,current.position.z])
    home_yaw=yaw(current.orientation)
    if warehouse:
        native_home=matrix_from_pose(latest['native'][0])
        rospy.set_param('~measured_native_home',native_home.tolist())
    if warehouse and not rospy.has_param('~route'):raise RuntimeError('Warehouse needs validated offline inspection plan')
    sweep_stations=set(rospy.get_param('~sweep_stations',[])) if warehouse else set()
    position_tolerance=float(rospy.get_param('~position_tolerance',.10 if warehouse else .18))
    yaw_tolerance=float(rospy.get_param('~yaw_tolerance',.06 if warehouse else .14))
    dwell_normal=float(rospy.get_param('~dwell_seconds',.8))
    dwell_sweep=float(rospy.get_param('~sweep_dwell_seconds',1.5))
    integral_enabled=warehouse and rospy.get_param('~integral_feedback',False)
    integrator=NativeIntegrator()
    route=np.array(rospy.get_param('~route') if warehouse else inspection_waypoints())
    if warehouse:
        route[:,:3]=route[:,:3]@native_home[:3,:3].T+native_home[:3,3]
        home=native_home[:3,3].copy();home_yaw=yaw(latest['native'][0].orientation)
    else:route[:,:3]+=home
    route[:,3]+=home_yaw
    target=home.copy();angle=home_yaw;phase='PRESTREAM';idx=0;phase_start=rospy.get_time()
    start=time.monotonic();last=rospy.get_time();last_request=-100.;reached=None;goal_start=rospy.get_time()
    last_status=-100.;failure=None
    while not rospy.is_shutdown():
        now=rospy.get_time();dt=max(0.,min(.15,now-last));last=now
        p=latest['pose'][0].pose
        position=np.array([p.position.x,p.position.y,p.position.z])
        tracking=latest['native'][0] if warehouse else p
        tracking_position=np.array([tracking.position.x,tracking.position.y,tracking.position.z])
        state=latest['state'][0]
        if requested_abort[0]:failure='Resource/mission abort: '+requested_abort[0]
        if time.monotonic()-latest['pose'][1]>2.5:failure='FCU pose stale'
        if warehouse and time.monotonic()-latest['native'][1]>1.:failure='Native R3LIVE body pose stale'
        if time.monotonic()-start>rospy.get_param('~max_wall_seconds',2400):failure='Mission wall timeout'
        if failure and phase!='LANDING':
            rospy.logerr(failure);phase='LANDING';phase_start=now
        if phase=='PRESTREAM' and now-phase_start>=4:
            phase='ARMING';phase_start=now
        elif phase=='ARMING':
            if state.armed and state.mode=='OFFBOARD':phase='INSPECT';goal_start=now
            elif now-last_request>1:
                if state.mode!='OFFBOARD':mode(custom_mode='OFFBOARD')
                else:arm(value=True)
                last_request=now
            if now-phase_start>25:failure='PX4 refused normal OFFBOARD arming'
        elif phase=='INSPECT':
            goal=route[idx]
            delta=goal[:3]-target;distance=np.linalg.norm(delta)
            candidate=target+delta*min(1.,(.35 if warehouse else .6)*dt/max(distance,1e-6))
            if integral_enabled:integrator.update(candidate-tracking_position,dt,np.linalg.norm(candidate-goal[:3])<.005)
            # Conservative measured obstacle tube. No scene geometry is used.
            if cloud[0] is not None and time.monotonic()-cloud[0][1]<2.5:
                xyz=cloud[0][0];segment=candidate-tracking_position+integrator.correction
                if warehouse:
                    # Instantaneous raw scan and a relative command share body
                    # coordinates. No asynchronously registered map/pose mix.
                    segment=matrix_from_pose(tracking)[:3,:3].T@segment
                    count=count_in_tube(xyz,segment)
                else:count=count_in_tube(xyz-position,segment)
                if count>5:failure='Measured obstacle within flight tube'
            target=candidate
            angular=.5 if warehouse else .30
            angle+=np.clip(wrap(goal[3]-angle),-angular*dt,angular*dt)
            settled=np.linalg.norm(goal[:3]-tracking_position)<position_tolerance and abs(wrap(goal[3]-yaw(tracking.orientation)))<yaw_tolerance
            reached=(now if reached is None else reached) if settled else None
            dwell=(dwell_sweep if idx in sweep_stations else dwell_normal) if warehouse else .35
            if reached is not None and now-reached>dwell:
                rospy.loginfo('Inspection waypoint %d/%d reached',idx+1,len(route))
                idx+=1;reached=None;goal_start=now
                if idx==len(route):phase='LANDING';phase_start=now
            if now-goal_start>55:failure='Waypoint did not settle within 55 simulation seconds'
        elif phase=='LANDING':
            if state.mode!='AUTO.LAND' and now-last_request>1:
                mode(custom_mode='AUTO.LAND');last_request=now
            ext=latest['landed'][0]
            if not state.armed and ext.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
                result=dict(success=failure is None,phase='DONE',waypoints_completed=idx,
                            total_waypoints=len(route),error=failure)
                status.publish(String(data=json.dumps(result)))
                if failure is None:done.publish(Bool(data=True))
                rospy.loginfo('Inspection finished: %s',json.dumps(result))
                rospy.sleep(1);return
            if now-phase_start>55:raise RuntimeError('Landing timed out: '+str(failure))
        out=PoseStamped();out.header.stamp=rospy.Time.now();out.header.frame_id='map'
        if warehouse:command,command_yaw=relative_setpoint(matrix_from_pose(p),matrix_from_pose(tracking),target+integrator.correction,angle)
        else:command,command_yaw=target,angle
        out.pose.position.x,out.pose.position.y,out.pose.position.z=command
        out.pose.orientation.z=math.sin(command_yaw/2);out.pose.orientation.w=math.cos(command_yaw/2)
        pub.publish(out)
        if now-last_status>.5:
            status.publish(String(data=json.dumps(dict(phase=phase,waypoint=idx,total=len(route),error=failure,
                tracking_target=target.tolist(),tracking_position=tracking_position.tolist(),
                tracking_error_m=float(np.linalg.norm(target-tracking_position)),integral_correction_m=integrator.correction.tolist()))))
            last_status=now
        rate.sleep()


if __name__=='__main__':main()
