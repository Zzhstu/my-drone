#!/usr/bin/env python3
"""Actor-only patrol driven by simulation time; never commands the UAV."""
import math


class Patrol:
    def __init__(self,waypoints,duration=180.,speed=.45,yield_distance=0.):
        self.waypoints=[tuple(map(float,p)) for p in waypoints]
        if (len(self.waypoints)<2 or any(len(p)!=2 or not all(map(math.isfinite,p)) for p in self.waypoints)
                or not math.isfinite(duration) or duration<=0 or not 0<speed<=1.
                or not math.isfinite(yield_distance) or yield_distance<0):
            raise ValueError('Invalid pedestrian patrol')
        self.duration=float(duration);self.speed=float(speed)
        self.yield_distance=float(yield_distance);self.yielding=False
        self.started=None;self.finished=None;self.index=0

    def velocity(self,now,xy,airborne,uav_xy=None):
        self.yielding=False
        if not math.isfinite(now) or now<=0 or not all(map(math.isfinite,xy)):return 0.,0.
        if self.started is None:
            if not airborne:return 0.,0.
            self.started=now
        if self.finished is not None:return 0.,0.
        if now-self.started>=self.duration:
            self.finished=now;return 0.,0.
        if (self.yield_distance and uav_xy is not None and all(map(math.isfinite,uav_xy))
                and math.dist(xy,uav_xy)<self.yield_distance):
            # Simulator actor yields along its aisle when the UAV approaches.
            # This uses Gazebo truth only in the actor script, never the UAV planner.
            self.yielding=True
            if all(abs(p[1]-self.waypoints[0][1])<.01 for p in self.waypoints):
                direction=-1. if uav_xy[0]>xy[0] else 1.
                lo=min(p[0] for p in self.waypoints);hi=max(p[0] for p in self.waypoints)
                if lo<=xy[0]+direction*.03<=hi:
                    return direction*self.speed,0.
            return 0.,0.
        x,y=self.waypoints[self.index];dx,dy=x-xy[0],y-xy[1];d=math.hypot(dx,dy)
        if d<.07:
            self.index=(self.index+1)%len(self.waypoints)
            x,y=self.waypoints[self.index];dx,dy=x-xy[0],y-xy[1];d=math.hypot(dx,dy)
        if d<1e-9:return 0.,0.
        return self.speed*dx/d,self.speed*dy/d


def main():
    import json,time
    from pathlib import Path
    import rospy
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rospy.init_node('right_pedestrian_patrol')
    scene=json.loads(Path(rospy.get_param('~scene_file')).read_text());spec=scene['extra_pedestrian']
    output=Path(rospy.get_param('~output'));model=spec['model_name'];state={};history=[]
    patrol=Patrol(spec['waypoints_world_xy'],spec['duration_sim_seconds'],spec['speed_mps'],
                  rospy.get_param('~uav_yield_distance',0.))
    def receive(msg):
        state.update({n:msg.pose[i].position for i,n in enumerate(msg.name) if n in (model,'iris_mid360')})
    rospy.Subscriber('/gazebo/model_states',ModelStates,receive,queue_size=1)
    pub=rospy.Publisher('/demo/pedestrian/'+model+'/cmd_vel',Twist,queue_size=1)
    status=rospy.Publisher('/demo/pedestrian/'+model+'/patrol_status',String,queue_size=1,latch=True)
    last_saved=-float('inf');last_xy=None;travel=0.;observed_finish=None
    def save(now):
        data=dict(spec=spec,uav_yield_distance_m=patrol.yield_distance,
            yield_scope='simulator actor uses Gazebo truth; not a UAV avoidance input',
            start_sim=patrol.started,finish_sim=patrol.finished,
            completed=patrol.finished is not None,duration_sim_seconds=(patrol.finished-patrol.started if patrol.finished is not None else None),
            last_sim=now,measured_xy_travel_m=travel,history=history,
            scope='Actor controller status; motion/trail/encounter independently checked from recorded Gazebo states. Ground truth not an input to UAV.')
        tmp=output.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2)+'\n');tmp.replace(output)
    rate=rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            now=rospy.Time.now().to_sec();actor=state.get(model);uav=state.get('iris_mid360')
            if actor is None:rate.sleep();continue
            xy=(actor.x,actor.y)
            vx,vy=patrol.velocity(now,xy,uav is not None and uav.z>.9,
                                  (uav.x,uav.y) if uav is not None else None)
            if patrol.started is not None and patrol.finished is None:
                if last_xy is not None:travel+=math.dist(last_xy,xy)
                last_xy=xy
            msg=Twist();msg.linear.x,msg.linear.y=vx,vy;pub.publish(msg)
            phase=('finished' if patrol.finished is not None else 'yielding' if patrol.yielding else
                   'moving' if patrol.started is not None else 'waiting_for_takeoff')
            if now-last_saved>=1. or (patrol.finished is not None and observed_finish is None):
                history.append(dict(sim=now,world_xy=list(xy),phase=phase));save(now);last_saved=now
                status.publish(json.dumps(dict(phase=phase,start_sim=patrol.started,finish_sim=patrol.finished)))
            if patrol.finished is not None:
                observed_finish=now
                rospy.loginfo('Actor %s completed %.3fs simulation patrol; stopped in place.',model,patrol.finished-patrol.started)
                rospy.spin();break
            rate.sleep()
    finally:
        pub.publish(Twist());save(rospy.Time.now().to_sec())


if __name__=='__main__':main()
