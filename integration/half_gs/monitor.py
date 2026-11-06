"""Read-only progress and independent actor audit; never publishes commands."""
import argparse
import json
import os
from pathlib import Path
import time
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates


def main(args):
    os.environ['ROS_MASTER_URI']=args.master_uri;root=Path(args.dataset)
    rospy.init_node('half_gs_readonly_monitor',anonymous=True,disable_signals=True)
    history=(root/'mission_history.jsonl').open('a',buffering=1);actors=(root/'actor_audit.jsonl').open('a',buffering=1);last=[-1.]
    def atomic(name,row):
        p=root/(name+'.json');temporary=p.with_suffix('.tmp');temporary.write_text(json.dumps(row,indent=2)+'\n');temporary.replace(p)
    def mission(msg):
        data=json.loads(msg.data);data['sim_stamp']=rospy.get_time();data['wall_unix']=time.time()
        history.write(json.dumps(data)+'\n');atomic('mission_live',data)
    def status(msg):atomic('collector_live',json.loads(msg.data))
    def models(msg):
        now=rospy.get_time()
        if now-last[0]<.5:return
        last[0]=now;result=dict(sim_receipt_stamp=now,scope='Headerless simulator state, offline actor audit only; not estimator/mask/control input')
        for name in ['0','1','iris_mid360']:
            if name in msg.name:
                p=msg.pose[msg.name.index(name)].position;result[name]=[p.x,p.y,p.z]
        actors.write(json.dumps(result)+'\n')
    rospy.Subscriber('/roomgs/mission',String,mission,queue_size=1)
    rospy.Subscriber('/lio_dataset/status',String,status,queue_size=1)
    rospy.Subscriber('/gazebo/model_states',ModelStates,models,queue_size=1)
    while not rospy.is_shutdown() and not (root/'result.json').exists():time.sleep(.5)
    history.close();actors.close()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('dataset');p.add_argument('--master-uri',required=True);main(p.parse_args())
