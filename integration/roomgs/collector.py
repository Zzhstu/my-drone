#!/usr/bin/env python3
"""Exact measurement-time RGB/c2w export. Never reads simulator truth."""
from collections import OrderedDict
from pathlib import Path
import json
import threading
import hashlib
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image,CompressedImage
from std_srvs.srv import Trigger,TriggerResponse
from geometry import matrix_from_pose


def main():
    rospy.init_node('roomgs_dataset_collector')
    output=Path(rospy.get_param('~output'));images=output/'dataset/images';images.mkdir(parents=True)
    minimum_interval=rospy.get_param('~minimum_interval',.5)
    minimum_translation=rospy.get_param('~minimum_translation',.12)
    maximum_idle_interval=rospy.get_param('~maximum_idle_interval',2.)
    compressed=rospy.get_param('~compressed_input',False)
    cache_limit=int(rospy.get_param('~cache_frames',16))
    finalized=[False]
    frames=[];pending={};cache=OrderedDict();lock=threading.RLock();bridge=CvBridge()
    counters=dict(images=0,valid_visual_poses=0,exact_pairs=0,exported=0,unmatched_expired=0)
    last=[None];last_t=[-100.]
    journal=(output/'dataset/frames.jsonl').open('w',buffering=1)
    def pair(stamp):
        if stamp not in cache or stamp not in pending:return
        msg=cache[stamp];pose=pending.pop(stamp);counters['exact_pairs']+=1
        c2w=matrix_from_pose(pose.pose)
        if last[0] is not None:
            distance=np.linalg.norm(c2w[:3,3]-last[0][:3,3])
            rotation=np.arccos(np.clip((np.trace(c2w[:3,:3]@last[0][:3,:3].T)-1)/2,-1,1))
            elapsed=stamp*1e-9-last_t[0]
            if elapsed<minimum_interval or (distance<minimum_translation and rotation<.10 and elapsed<maximum_idle_interval):return
        name=('%019d.jpg' if compressed else '%019d.png')%stamp;path=images/name
        if compressed:
            if msg.format!='jpeg':raise RuntimeError('Expected acquisition JPEG')
            path.write_bytes(bytes(msg.data))
        else:
            array=bridge.imgmsg_to_cv2(msg,'bgr8')
            if not cv2.imwrite(str(path),array):raise RuntimeError('PNG save failed')
        row=dict(file_path='images/'+name,stamp_ns=stamp,c2w=c2w.tolist(),
                 sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                 source='actual R3LIVE successfully fused optical pose and exact acquisition image stamp',encoding='acquisition_jpeg' if compressed else 'png')
        frames.append(row);journal.write(json.dumps(row)+'\n')
        counters['exported']=len(frames);last[0]=c2w;last_t[0]=stamp*1e-9
    def image(m):
        with lock:
            if finalized[0]:return
            stamp=m.header.stamp.to_nsec();cache[stamp]=m;counters['images']+=1
            pair(stamp)
            while len(cache)>cache_limit:cache.popitem(last=False)
    def pose(m):
        with lock:
            if finalized[0]:return
            stamp=m.header.stamp.to_nsec();pending[stamp]=m;counters['valid_visual_poses']+=1
            pair(stamp)
            if cache:
                oldest=next(iter(cache))
                for key in list(pending):
                    if key<oldest:pending.pop(key);counters['unmatched_expired']+=1
    def save(_):
        with lock:
            finalized[0]=True
            journal.flush()
            with (output/'dataset/frames.json').open('w') as stream:json.dump(frames,stream,indent=2)
            (output/'dataset/export_status.json').write_text(json.dumps(counters,indent=2)+'\n')
        return TriggerResponse(success=len(frames)>20,message=json.dumps(counters))
    rospy.Subscriber('/r3live/input/image/compressed' if compressed else '/r3live/input/image',CompressedImage if compressed else Image,image,queue_size=2,buff_size=2000000)
    rospy.Subscriber('/roomgs/optical_pose',PoseStamped,pose,queue_size=100)
    service=rospy.Service('/roomgs/finalize_dataset',Trigger,save)
    rospy.on_shutdown(lambda:journal.close())
    rospy.spin()


if __name__=='__main__':main()
