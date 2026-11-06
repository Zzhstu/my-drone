#!/usr/bin/env python3
"""Real compiled bridge integration; requires an isolated ROS master."""
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


def main():
    rospy.init_node('sensor_origin_test',anonymous=True,disable_signals=True)
    received={};checks={};processes=[]
    rospy.Subscriber('/origin_test/body',Odometry,lambda m:received.update(body=m))
    rospy.Subscriber('/origin_test/scan',PointCloud2,lambda m:received.update(scan=m))
    pub=rospy.Publisher('/origin_test/imu',Odometry,queue_size=3)
    px4=rospy.Publisher('/origin_test/px4',PoseStamped,queue_size=3)
    cloud=rospy.Publisher('/origin_test/raw_scan',PointCloud2,queue_size=3)
    for name in ('origin_pose','origin_cloud'):
        rospy.set_param('/'+name+'/imu_origin_in_body',[0.,0.,.12412])
    common=['_use_px4_initial_alignment:=true',
            '/Odometry:=/origin_test/imu','/uav1/mavros/local_position/pose:=/origin_test/px4']
    try:
        processes.append(subprocess.Popen(['rosrun','fastlio_bridge','fastlio_bridge_node',
            '__name:=origin_pose','_publish_mavros_vision_pose:=false',
            '/uav1/fastlio/odom:=/origin_test/body']+common,stdout=subprocess.DEVNULL))
        processes.append(subprocess.Popen(['rosrun','fastlio_bridge','cloud_bridge_node',
            '__name:=origin_cloud','_output_topic:=/origin_test/scan',
            '/cloud_registered:=/origin_test/raw_scan']+common,stdout=subprocess.DEVNULL))
        # First align with body at [10,20,2], IMU at [0,0,.12412].
        # Subsequent IMU tilts by 30 degrees about Y: body must subtract
        # the ROTATED lever arm, not just a constant world-Z shift.
        for tilted in (False,True):
            received.clear();deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                stamp=rospy.Time.now();o=Odometry();o.header.stamp=stamp
                angle=math.pi/6 if tilted else 0.
                o.pose.pose.orientation.y=math.sin(angle/2);o.pose.pose.orientation.w=math.cos(angle/2)
                o.pose.pose.position.z=.12412
                p=PoseStamped();p.header.stamp=stamp;p.pose.orientation.w=1.
                p.pose.position.x=10;p.pose.position.y=20;p.pose.position.z=2
                pub.publish(o);px4.publish(p)
                fields=[PointField(name,4*i,PointField.FLOAT32,1) for i,name in enumerate(('x','y','z','intensity'))]
                cloud.publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='camera_init'),fields,[[0,0,0,1]]))
                time.sleep(.05)
                if 'body' in received and 'scan' in received:
                    body=received['body'].pose.pose.position
                    expected=np.array([10-.12412*math.sin(angle),20,2+.12412*(1-math.cos(angle))])
                    value=np.array([body.x,body.y,body.z])
                    scan=np.array(list(point_cloud2.read_points(received['scan'],field_names=('x','y','z'))))[0]
                    if np.allclose(value,expected,atol=1e-5) and np.allclose(scan,[10,20,2],atol=1e-5):
                        checks['tilted_lever_arm' if tilted else 'initial_body_origin']=True;break
            else:raise RuntimeError('Compiled bridge pose/cloud inconsistency: '+str(received))
        # Verify actual mapper rays begin at the laser, not the new body pose.
        rospy.set_param('/origin_mapper',dict(cloud_topic='/origin_test/ray_scan',
            odom_topic='/origin_test/ray_body',free_space_output_topic='/origin_test/free',
            evidence_only_clearing=True,voxel_size=.05,free_space_ray_step=.05,
            lidar_origin_in_odom=[0.,0.,.13]))
        rospy.Subscriber('/origin_test/free',PointCloud2,lambda m:received.update(free=m))
        ray_scan=rospy.Publisher('/origin_test/ray_scan',PointCloud2,queue_size=3)
        ray_body=rospy.Publisher('/origin_test/ray_body',Odometry,queue_size=3)
        processes.append(subprocess.Popen(['rosrun','fastlio_bridge','static_map_builder_node',
            '__name:=origin_mapper'],stdout=subprocess.DEVNULL))
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            stamp=rospy.Time.now();o=Odometry();o.header=Header(stamp=stamp,frame_id='map')
            o.pose.pose.orientation.w=1.;o.pose.pose.position.z=1.
            ray_body.publish(o)
            ray_scan.publish(point_cloud2.create_cloud(Header(stamp=stamp,frame_id='map'),fields,[[2,0,1.13,1]]))
            time.sleep(.05)
            if 'free' in received:
                pts=np.asarray(list(point_cloud2.read_points(received['free'],field_names=('x','y','z'))))
                if len(pts) and np.allclose(pts[:,2],1.125,atol=1e-5):
                    checks['mapper_laser_origin']=True;break
        else:raise RuntimeError('Free rays did not use calibrated laser origin')
        print(json.dumps(checks,indent=2))
    finally:
        for p in processes:p.send_signal(signal.SIGINT)
        for p in processes:p.wait(timeout=8)


if __name__=='__main__':main()
