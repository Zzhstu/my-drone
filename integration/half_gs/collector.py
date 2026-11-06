#!/usr/bin/env python3
"""Export measured static cloud and RGB with model-calibrated LIO camera poses."""
import hashlib
import json
from pathlib import Path
import sys
import threading
from collections import OrderedDict
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage,PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'roomgs'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'lio_dataset'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'))
import message_filters
from dufomap_core import DufoMapEngine
from geometry import matrix_from_pose
from poses import PoseBuffer,VoxelMap,xyz_from_cloud,quaternion


def dump(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def main():
    rospy.init_node('lio_dataset_collector');output=Path(rospy.get_param('~output'))
    images=output/'dataset/images';images.mkdir(parents=True)
    g=json.loads((output/'sensor_geometry.json').read_text())
    extrinsic=np.linalg.inv(np.array(g['body_from_imu']))@np.array(g['body_from_camera'])
    poses=PoseBuffer(extrinsic);engine=DufoMapEngine(resolution=.10,sensor_error=.12,pose_cells=1,threads=2,max_frames=40000,max_voxels=900000,lidar_origin=g['lidar_in_imu_translation']);pending=OrderedDict();frames=[];lock=threading.RLock()
    frozen=[False];last=[None];last_stamp=[-10**18]
    stats=dict(received_images=0,received_odometry=0,received_clouds=0,exported=0,
        expired_images=0,rejected_gap_images=0,errors=0,map_points=0)
    journal=(output/'dataset/frames.jsonl').open('w',buffering=1)
    trajectory=(output/'trajectory_imu.jsonl').open('w',buffering=1)
    pub=rospy.Publisher('/lio_dataset/optical_pose',PoseStamped,queue_size=10)
    status=rospy.Publisher('/lio_dataset/status',String,queue_size=1,latch=True)
    def drain():
        for stamp,msg in list(pending.items()):
            if not poses.stamps or stamp>poses.stamps[-1]:continue
            pending.pop(stamp)
            solved=poses.camera(stamp)
            if solved is None:stats['rejected_gap_images']+=1;continue
            c2w,association=solved
            if last[0] is not None:
                distance=np.linalg.norm(c2w[:3,3]-last[0][:3,3])
                angle=np.arccos(np.clip((np.trace(last[0][:3,:3].T@c2w[:3,:3])-1)/2,-1,1))
                if stamp-last_stamp[0]<190000000:continue
                if distance<.04 and angle<.08 and stamp-last_stamp[0]<500000000:continue
            name='%019d.jpg'%stamp;path=images/name;path.write_bytes(bytes(msg.data))
            row=dict(file_path='images/'+name,stamp_ns=stamp,c2w=c2w.tolist(),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),pose_association=association,
                source='FAST-LIO LiDAR+MID360 IMU state and SDF optical extrinsic; RGB unused by estimator')
            frames.append(row);journal.write(json.dumps(row)+'\n');last[0]=c2w;last_stamp[0]=stamp
            out=PoseStamped();out.header=msg.header;out.header.frame_id='camera_init'
            out.pose.position.x,out.pose.position.y,out.pose.position.z=c2w[:3,3]
            q=quaternion(c2w[:3,:3])
            out.pose.orientation.x,out.pose.orientation.y,out.pose.orientation.z,out.pose.orientation.w=q
            pub.publish(out);stats['exported']=len(frames)
    def odom(msg):
        with lock:
            if frozen[0]:return
            try:
                if msg.header.frame_id!='camera_init':raise ValueError('Unexpected LIO map frame')
                stamp=msg.header.stamp.to_nsec();matrix=matrix_from_pose(msg.pose.pose)
                poses.add(stamp,matrix);stats['received_odometry']+=1
                trajectory.write(json.dumps(dict(stamp_ns=stamp,map_from_imu=matrix.tolist()))+'\n');drain()
            except Exception as e:stats['errors']+=1;rospy.logerr(str(e))
    def image(msg):
        with lock:
            if frozen[0]:return
            stats['received_images']+=1;stamp=msg.header.stamp.to_nsec();pending[stamp]=msg;drain()
            while len(pending)>16:pending.popitem(last=False);stats['expired_images']+=1
    def cloud(msg,odom_msg):
        with lock:
            if frozen[0]:return
            try:
                if msg.header.frame_id!='camera_init' or msg.header.stamp!=odom_msg.header.stamp:raise ValueError('Unpaired registered scan/IMU state')
                matrix=matrix_from_pose(odom_msg.pose.pose)
                engine.ingest(xyz_from_cloud(msg),matrix[:3,3],quaternion(matrix[:3,:3]),msg.header.stamp.to_nsec(),engine.frames+1)
                stats['received_clouds']+=1;stats['map_points']=len(engine.voxels)
            except Exception as e:stats['errors']+=1;rospy.logerr(str(e))
    def save(_):
        with lock:
            drain();frozen[0]=True;journal.flush();trajectory.flush()
            stats['pending_at_end']=len(pending);xyz,removed,observed=engine.snapshot();engine.save(output/'dufo');stats['dufomap']=engine.stats()
            np.savez_compressed(output/'removed_map.npz',xyz=removed);np.savez_compressed(output/'observed_map.npz',xyz=observed)
            np.savez_compressed(output/'static_map.npz',xyz=xyz)
            for ext in ['ply','pcd']:
                with (output/('static_map.'+ext)).open('wb') as f:
                    if ext=='ply':header='ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n'%len(xyz)
                    else:header='# Measured FAST-LIO world points, voxel 0.05 m\nVERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n'%(len(xyz),len(xyz))
                    f.write(header.encode());f.write(xyz.astype('<f4').tobytes())
            dump(output/'dataset/frames.json',frames);dump(output/'dataset/export_status.json',stats)
            return TriggerResponse(success=len(frames)>20 and len(xyz)>100 and stats['errors']==0,message=json.dumps(stats))
    rospy.Subscriber('/Odometry',Odometry,odom,queue_size=20)
    rospy.Subscriber('/lio_dataset/rgb/compressed',CompressedImage,image,queue_size=2,buff_size=2000000)
    scan_sub=message_filters.Subscriber('/cloud_registered',PointCloud2,queue_size=10,buff_size=4000000)
    pose_sub=message_filters.Subscriber('/Odometry',Odometry,queue_size=20)
    synchronizer=message_filters.TimeSynchronizer([scan_sub,pose_sub],40);synchronizer.registerCallback(cloud)
    rospy.Timer(rospy.Duration(1),lambda _:status.publish(String(data=json.dumps(stats))))
    service=rospy.Service('/lio_dataset/finalize',Trigger,save)
    rospy.spin()


if __name__=='__main__':main()
