#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, csv, time
from datetime import datetime
import rospy
from spirecv_msgs.msg import TargetsInFrame

class Logger:
    def __init__(self):
        self.uav_id=rospy.get_param('~uav_id',1); self.model=rospy.get_param('~model_name','unknown')
        self.topic=rospy.get_param('~target_topic',f'/uav{self.uav_id}/spirecv/target')
        self.outdir=os.path.expanduser(rospy.get_param('~output_dir','~/p450_yolo_logs')); os.makedirs(self.outdir,exist_ok=True)
        fn=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+self.model.replace('/','_')+'.csv'; self.path=os.path.join(self.outdir,fn)
        self.f=open(self.path,'w',newline=''); self.w=csv.writer(self.f)
        self.w.writerow(['wall_time','ros_time','model','frame_id','fps','width','height','num_targets','no_track','locked','category','score','cx','cy','w','h','area','err_x','err_y','los_ax','los_ay','px','py','pz'])
        rospy.Subscriber(self.topic,TargetsInFrame,self.cb,queue_size=100)
        rospy.loginfo('benchmark csv: %s', self.path)
    def cb(self,msg):
        t=None; locked=False
        for x in msg.targets:
            if x.mode: t=x; locked=True; break
        if t is None and msg.targets: t=max(msg.targets,key=lambda x:x.score)
        if t is None: tail=[False,'',0,0,0,0,0,0,0,0,0,0,0,0,0]
        else: tail=[locked,t.category,t.score,t.cx,t.cy,t.w,t.h,t.w*t.h,t.cx-0.5,t.cy-0.5,t.los_ax,t.los_ay,t.px,t.py,t.pz]
        self.w.writerow([time.time(),msg.header.stamp.to_sec(),self.model,msg.frame_id,msg.fps,msg.width,msg.height,len(msg.targets),msg.no_track_frame_count]+tail); self.f.flush()
    def close(self): self.f.close()

if __name__=='__main__':
    rospy.init_node('benchmark_logger'); node=Logger(); rospy.on_shutdown(node.close); rospy.spin()
