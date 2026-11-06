#!/usr/bin/env python3
"""Run the deterministic production-source replay on an isolated ROS master.

Baseline is the unchanged numerical detector/cluster behavior with the two new
flags disabled, not a claim that the instrumented test is the old executable.
All three arms use only recorded scan/odometry; no model/actor truth is online.
"""
import argparse
from pathlib import Path
import os
import signal
import subprocess
import time
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--output-prefix',type=Path,required=True)
    parser.add_argument('--port',type=int,default=11365)
    parser.add_argument('--guarded-only',action='store_true')
    args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    root=package.parents[4]
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:%d'%args.port
    import rosgraph
    import rospy
    if rosgraph.is_master_online():raise RuntimeError('refuse to reuse an existing master')
    log=open(str(args.output_prefix)+'_roscore.log','w')
    core=subprocess.Popen(['roscore','-p',str(args.port)],stdout=log,stderr=subprocess.STDOUT,
                          start_new_session=True)
    try:
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('isolated master timed out')
            time.sleep(.1)
        rospy.init_node('static_motion_replay_setup',disable_signals=True)
        profile=yaml.safe_load((package.parent/'FAST_LIO/config/mid360_sim.yaml').read_text())
        rospy.set_param('/static_map_builder/lidar_origin_in_odom',
                        profile['static_map_builder']['lidar_origin_in_odom'])
        for name,guard in [('baseline',False),('guarded',True)]:
            detector=yaml.safe_load((package/'config/dynamic_detector.yaml').read_text())
            detector.update(publish_observed_free_motion=guard,preserve_measured_centroids=guard)
            cluster=yaml.safe_load((package/'config/dynamic_cluster.yaml').read_text())
            cluster['require_observed_free_motion']=guard
            cluster['static_plane_motion_veto']=guard
            mapper=yaml.safe_load((package/'config/static_map.yaml').read_text())
            mapper.update(evidence_only_clearing=True,enable_ghost_revisit=True,
                          lidar_origin_in_odom=profile['static_map_builder']['lidar_origin_in_odom'])
            for kind,value in [('detector',detector),('cluster',cluster),('map',mapper)]:
                rospy.set_param('/'+name+'_'+kind,value)
        with open(str(args.output_prefix)+'_replay.log','w') as output:
            command=[str(args.binary.resolve()),str(args.bag.resolve()),str(args.output_prefix.resolve())]
            if args.guarded_only:command.append('guarded')
            subprocess.run(command,stdout=output,
                           stderr=subprocess.STDOUT,check=True)
    finally:
        os.killpg(core.pid,signal.SIGTERM)
        try:core.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(core.pid,signal.SIGKILL)
        log.close()


if __name__=='__main__':main()
