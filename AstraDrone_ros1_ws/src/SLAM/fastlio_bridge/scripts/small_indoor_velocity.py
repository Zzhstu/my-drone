#!/usr/bin/env python3
"""Office-only map velocity -> MAVROS local ENU using paired body attitudes.

Both frames are gravity aligned. The same body orientation in the two frames
determines their yaw difference. Translation is irrelevant for a velocity
vector. No Gazebo truth, scene map or change to safety distances is used.
"""
import math


def yaw(q):
    values=(q.x,q.y,q.z,q.w)
    norm=math.sqrt(sum(v*v for v in values))
    if not all(math.isfinite(v) for v in values) or norm<.5:
        raise ValueError('Invalid attitude quaternion')
    x,y,z,w=(v/norm for v in values)
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def rotate_xy(vx,vy,map_yaw,local_yaw):
    if not all(math.isfinite(v) for v in (vx,vy,map_yaw,local_yaw)):
        raise ValueError('Nonfinite velocity/heading')
    angle=local_yaw-map_yaw
    c,s=math.cos(angle),math.sin(angle)
    return c*vx-s*vy,s*vx+c*vy


def fresh(now,items,wall_now,max_age=.35):
    return all(item is not None and 0<=now-item[0].header.stamp.to_sec()<=max_age and
               wall_now-item[1]<=1. for item in items)


def main():
    import threading
    import time
    import rospy
    from geometry_msgs.msg import PoseStamped,TwistStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float64
    rospy.init_node('small_indoor_velocity')
    lock=threading.Lock();latest={}
    def receive(key,msg):
        with lock:
            latest[key]=(msg,time.monotonic())
    rospy.Subscriber('/small_indoor/map_velocity',TwistStamped,lambda m:receive('command',m),queue_size=1)
    rospy.Subscriber('/uav1/fastlio/odom',Odometry,lambda m:receive('map',m),queue_size=1)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,lambda m:receive('local',m),queue_size=1)
    publisher=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=1)
    offset_pub=rospy.Publisher('/small_indoor/map_to_local_yaw_deg',Float64,queue_size=1)
    def tick(event):
        now=rospy.Time.now();out=TwistStamped();out.header.stamp=now;out.header.frame_id='local_enu'
        with lock:
            command,estimate,local=[latest.get(k) for k in ('command','map','local')]
        try:
            valid=fresh(now.to_sec(),[command,estimate,local],time.monotonic())
            valid=valid and command[0].header.frame_id in ('map','/map')
            valid=valid and abs((estimate[0].header.stamp-local[0].header.stamp).to_sec())<=.10
            if valid:
                map_yaw=yaw(estimate[0].pose.pose.orientation);local_yaw=yaw(local[0].pose.orientation)
                v=command[0].twist.linear
                if not math.isfinite(v.z):
                    raise ValueError('Nonfinite vertical command')
                out.twist.linear.x,out.twist.linear.y=rotate_xy(v.x,v.y,map_yaw,local_yaw)
                out.twist.linear.z=v.z
                # Existing controller sends zero yaw-rate; no heading target is added.
                offset=math.degrees(math.atan2(math.sin(local_yaw-map_yaw),math.cos(local_yaw-map_yaw)))
                offset_pub.publish(Float64(offset))
                rospy.loginfo_throttle(2.,'Office map/local yaw offset %.1f deg; map->ENU velocity conversion active',offset)
            else:
                rospy.logwarn_throttle(2.,'Office velocity adapter: stale/unpaired input, zero velocity')
        except (ValueError,TypeError,AttributeError) as exc:
            # Construct anew: never leave a partially assigned nonzero command.
            out=TwistStamped();out.header.stamp=now;out.header.frame_id='local_enu'
            rospy.logerr_throttle(2.,'Office velocity adapter rejected input: %s',str(exc))
        publisher.publish(out)
    rospy.Timer(rospy.Duration(1./30),tick)
    rospy.spin()


if __name__=='__main__':
    main()
