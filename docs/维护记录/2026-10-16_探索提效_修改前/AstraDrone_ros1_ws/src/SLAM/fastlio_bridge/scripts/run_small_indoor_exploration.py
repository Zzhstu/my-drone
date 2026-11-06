#!/usr/bin/env python3
"""Run AstraDrone exploration in the small office or half-area warehouse."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rosgraph
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Float64, Bool
from std_srvs.srv import Trigger

from analyze_dynamic_demo_bag import rotation
from run_dynamic_comparison import CaseRunner, cleanup_interrupt, process_running, wait_topic
from run_warehouse_exploration import mission_deadline
from small_indoor_scene import prepare, interior_wall_samples
from small_indoor_startup import wait_heading_ready
from startup_checks import wait_ground_pose
from warehouse_map_audit import read_xyz_pcd, supported, write_xyz_pcd


def evaluate(root, prefix, alignment, scene='office'):
    """Offline evaluation only, after flight processes stop."""
    pcd = Path(str(prefix)+'_final_map.pcd')
    points = read_xyz_pcd(pcd)
    metadata = json.loads(Path(str(prefix)+'_final_map.json').read_text())
    if hashlib.sha256(pcd.read_bytes()).hexdigest() != metadata['sha256']:
        raise ValueError('Saved PCD does not match saver metadata')
    if alignment is None:
        raise ValueError('No evaluator world/map alignment; map remains saved')
    r = np.asarray(alignment['map_from_world_rotation'])
    t = np.asarray(alignment['map_from_world_translation'])
    world = (points-t)@r
    if scene == 'cangku-half':
        from warehouse_map_audit import audit
        metadata = json.loads(Path(str(prefix)+'_scene.json').read_text())
        report, missing = audit(metadata, points, alignment)
        Path(str(prefix)+'_shelf_audit.json').write_text(json.dumps(report,indent=2)+'\n')
        write_xyz_pcd(str(prefix)+'_shelf_missing_reference.pcd',missing)
    else:
        source = root/'simulation/astra_gazebo_worlds/small_office/office_small.original.world'
        reference = interior_wall_samples(source)
        match = supported(reference,world,.25)
        report = dict(scope='Inward-facing original room wall-box surfaces only, world z=.25..2.20m; '
                        'excludes furniture, outward faces, floor, ceiling and window edges. NOT full 3D completeness.',
                  reference_samples=len(reference),supported_samples=int(match.sum()),
                  inward_wall_support_ratio=float(match.mean()),sampling_step_m=.20,
                  euclidean_tolerance_m=.25,map_points=len(points),full_3d_map_complete=False,
                  alignment=alignment)
        Path(str(prefix)+'_wall_audit.json').write_text(json.dumps(report,indent=2)+'\n')
        write_xyz_pcd(str(prefix)+'_wall_missing_reference.pcd',reference[~match]@r.T+t)
    # A standalone result plot, never a privileged input to the explorer.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    samples = json.loads(Path(str(prefix)+'_flight_samples.json').read_text())
    fig,ax = plt.subplots(figsize=(7,7))
    band=world[(world[:,2]>.25)&(world[:,2]<2.20)]
    ax.scatter(band[::3,0],band[::3,1],s=.3,c='#267c9a',label='Saved maintained point cloud')
    if samples:
        track=(np.asarray([s['map_xyz'] for s in samples])-t)@r
        ax.plot(track[:,0],track[:,1],c='#d45c27',lw=1.3,label='Estimated flown trajectory')
        ax.scatter(track[0,0],track[0,1],marker='*',s=100,c='#188448',label='Start')
    bounds = metadata['bounds_world'] if scene == 'cangku-half' else [-5,-5,5,5]
    ax.set(xlim=(bounds[0]-1,bounds[2]+1),ylim=(bounds[1]-1,bounds[3]+1),
           xlabel='World X (m), initial rigid alignment',ylabel='World Y (m)',
           title=('Half warehouse' if scene == 'cangku-half' else 'Small office')+
                 ': actual maintained map and flown trajectory')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(str(prefix)+'_map_and_trajectory.png',dpi=180);plt.close(fig)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',choices=('office','cangku-half'),default='office')
    parser.add_argument('--headless',action='store_true',help='run Gazebo server without either GUI')
    parser.add_argument('--no-gazebo-gui',action='store_true',help='keep RViz only')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--keep-open',action='store_true',help='keep simulation after successful return/save')
    parser.add_argument('--manual-pedestrian',action='store_true',
                        help='half warehouse only: skip the automatic walk so keyboard control can start immediately')
    parser.add_argument('--return-after-goals',type=int,default=0,help='explicit partial-demo return after N reached viewpoints')
    parser.add_argument('--test-seconds',type=float,default=0.,help='TEST ONLY: force cutoff, never task success')
    parser.add_argument('--max-wall-seconds',type=float,default=None,help='hard wall guard: office 1200s, half warehouse 2400s')
    args=parser.parse_args()
    if args.manual_pedestrian and args.scene != 'cangku-half':
        parser.error('--manual-pedestrian is currently supported only by --scene cangku-half')
    if args.max_wall_seconds is None:
        args.max_wall_seconds = 2400. if args.scene == 'cangku-half' else 1200.
    try:
        mission_deadline(0.,args.test_seconds,args.max_wall_seconds)
        if args.return_after_goals<0:
            raise ValueError('return-after-goals must be nonnegative')
    except ValueError as exc:
        parser.error(str(exc))
    if rosgraph.is_master_online() or any(process_running(n) for n in ('gzserver','px4','fastlio_mapping')):
        parser.error('请先退出已有ROS/PX4/Gazebo演示；不会关闭用户已有进程。')
    if not args.headless and not (args.no_rviz and args.no_gazebo_gui) and not os.environ.get('DISPLAY'):
        parser.error('没有DISPLAY，请使用 --headless')
    root=Path(os.environ['ASTRA_ROOT']);px4=Path(os.environ['PX4_DIR'])
    scripts=Path(__file__).resolve().parent
    half = args.scene == 'cangku-half'
    folder=root/('half_warehouse_demo_runs' if half else 'small_indoor_demo_runs');folder.mkdir(exist_ok=True)
    prefix=folder/(('warehouse_half_' if half else 'office_')+datetime.now().strftime('%Y%m%d_%H%M%S'))
    run=CaseRunner(root,prefix,os.environ.copy())
    lock=threading.Lock();mission={};history=[];samples=[];alignment=None;truth_snapshot=None;yaw_history=[]
    saver=False;saved=False;failed=False;termination='interrupted';subs=[]
    start_wall=time.monotonic()
    try:
        if half:
            from half_warehouse_scene import prepare as prepare_half
            world=prepare_half(root,prefix)
        else:
            world=prepare(root,prefix)
        scene_metadata=json.loads(Path(str(prefix)+'_scene.json').read_text())
        spawn=scene_metadata['spawn_world']
        core=run.start('core',['roscore'])
        until=time.monotonic()+30
        while not rosgraph.is_master_online():
            if core.poll() is not None or time.monotonic()>until:
                raise RuntimeError('ROS master startup failed')
            time.sleep(.2)
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('small_indoor_runner',anonymous=True,disable_signals=True)
        def status(msg):
            nonlocal mission
            try:
                d=json.loads(msg.data)
            except (ValueError,TypeError):
                return
            with lock:
                mission=d;history.append(dict(wall_elapsed=time.monotonic()-start_wall,**d))
        def truth(msg):
            nonlocal truth_snapshot
            truth_snapshot=(msg,time.monotonic())
        def odom(msg):
            nonlocal alignment
            with lock:
                if not samples or msg.header.stamp.to_sec()-samples[-1]['stamp']>=.20:
                    p=msg.pose.pose.position
                    samples.append(dict(stamp=msg.header.stamp.to_sec(),map_xyz=[p.x,p.y,p.z]))
            snapshot=truth_snapshot
            if alignment is not None or snapshot is None or time.monotonic()-snapshot[1]>.05:
                return
            try:
                body=snapshot[0].pose[snapshot[0].name.index('iris_mid360')]
            except ValueError:
                return
            estimate=msg.pose.pose
            r=rotation(estimate.orientation)@rotation(body.orientation).T
            t=np.asarray([estimate.position.x,estimate.position.y,estimate.position.z])-r@np.asarray([body.position.x,body.position.y,body.position.z])
            alignment=dict(map_from_world_rotation=r.tolist(),map_from_world_translation=t.tolist(),
                           odom_stamp=msg.header.stamp.to_sec(),truth_receipt_age_wall=time.monotonic()-snapshot[1],
                           note='Evaluator-only initial alignment; ModelStates has no timestamp. No long-term drift correction.')
            Path(str(prefix)+'_alignment.json').write_text(json.dumps(alignment,indent=2)+'\n')
            truth_sub.unregister()
        subs.append(rospy.Subscriber('/uav1/exploration/status',String,status,queue_size=1))
        def yaw_offset(msg):
            stamp=rospy.Time.now().to_sec()
            with lock:
                if not yaw_history or stamp-yaw_history[-1]['stamp']>=1. or abs(msg.data-yaw_history[-1]['degrees'])>.5:
                    yaw_history.append(dict(stamp=stamp,degrees=msg.data))
        subs.append(rospy.Subscriber('/small_indoor/map_to_local_yaw_deg',Float64,yaw_offset,queue_size=1))
        truth_sub=rospy.Subscriber('/gazebo/model_states',ModelStates,truth,queue_size=1)
        subs.extend([truth_sub,rospy.Subscriber('/uav1/fastlio/odom',Odometry,odom,queue_size=1)])
        setup=px4/'Tools/simulation/gazebo-classic/setup_gazebo.bash'
        launch=root/'simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch'
        cmd='source {} {} {}; exec roslaunch {} world:={} gui:={} interactive:=false x:={} y:={}'.format(
            *[shlex.quote(str(p)) for p in (setup,px4,px4/'build/px4_sitl_default',launch,world)],
            'false' if args.headless or args.no_gazebo_gui else 'true',spawn[0],spawn[1])
        gazebo=run.start('gazebo',['bash','-lc',cmd])
        wait_topic('/livox/lidar',PointCloud2,120,gazebo)
        stable=wait_ground_pose(gazebo)
        wait_heading_ready(gazebo)
        print('PX4未解锁且航向连续稳定；速度坐标转换器另行处理解锁时的航向对齐变化。',flush=True)
        stable=wait_ground_pose(gazebo)
        print('地面基准稳定，Z={:.3f}m；启动FAST-LIO与自主探索。'.format(stable.pose.position.z),flush=True)
        lio=run.start('fastlio',['roslaunch','fast_lio','mapping_mid360.launch','rviz:=false','sim_lidar:=true','dynamic_filter:=false'])
        wait_topic('/Odometry',Odometry,90,lio)
        if half:
            # One pose owner moves the included model and guards it against
            # the half-scene collision boxes; truth is not sent to Explorer.
            human=run.start('pedestrian',[sys.executable,str(scripts/'warehouse_pedestrian.py'),
                                          '_scene_file:='+str(prefix)+'_scene.json'])
            wait_topic('/demo/pedestrian/ready',Bool,20,human)
        run.start('exploration',['roslaunch','fastlio_bridge',
                  'half_warehouse_exploration.launch' if half else 'small_indoor_exploration.launch',
                  'rviz:='+('false' if args.headless or args.no_rviz else 'true'),
                  'output:='+str(prefix),'return_after_goals:='+str(args.return_after_goals)])
        rospy.wait_for_service('/demo/save_map',timeout=40);saver=True
        if half and not args.manual_pedestrian:
            run.start('walk_once',[sys.executable,str(scripts/'exploration_pedestrian_walk.py')])
        print(('半面积仓库' if half else '小办公室')+'自主探索已启动：未知几何选点 → 补扫 → 返航驻留 → 保存。',flush=True)
        if half:
            mode='行人可立即键盘控制' if args.manual_pedestrian else '单名行人自动行走一轮后可键盘接管'
            print('半面积仓库：'+mode+'；导航速度上限0.40m/s，安全净距保持0.80m。',flush=True)
            print('另开终端运行 bash docs/演示启动命令/行人键盘控制.sh（WASD为世界XY，空格停止，Q退出）。',flush=True)
        else:
            print('本轮无行人/语义网络/回环；输出前缀：'+str(prefix),flush=True)
        deadline,timeout_reason=mission_deadline(time.monotonic(),args.test_seconds,args.max_wall_seconds)
        while time.monotonic()<deadline or (args.keep_open and mission.get('state')=='COMPLETE' and mission.get('home_confirmed')):
            for name,process in run.processes:
                if process.poll() is not None:
                    raise RuntimeError(name+' 提前退出，请检查对应日志')
            state=mission.get('state')
            if state in ('STALLED_UNSAFE','RETURN_FAILED'):
                termination=state.lower();failed=True
                break
            if state=='COMPLETE' and mission.get('home_confirmed'):
                termination='returned_home_and_saved'
                if not args.keep_open:
                    break
            time.sleep(.3)
        else:
            termination=timeout_reason;failed=timeout_reason=='wall_guard_timeout_partial'
    except KeyboardInterrupt:
        print('收到Ctrl-C，保存当前地图并清理本轮进程。',flush=True)
    except Exception as exc:
        failed=True
        if termination=='interrupted':
            termination='runtime_error'
        print('小室内演示失败：'+str(exc),file=sys.stderr,flush=True)
    finally:
        signal.signal(signal.SIGINT,cleanup_interrupt)
        if saver and rosgraph.is_master_online():
            try:
                response=rospy.ServiceProxy('/demo/save_map',Trigger)()
                saved=bool(response.success);failed |= not saved
                print('最终点云保存：{} {}'.format(saved,response.message),flush=True)
            except Exception as exc:
                failed=True;print('保存失败：'+str(exc),flush=True)
        for sub in subs:
            sub.unregister()
        with lock:
            Path(str(prefix)+'_status_history.json').write_text(json.dumps(history,indent=2)+'\n')
            Path(str(prefix)+'_flight_samples.json').write_text(json.dumps(samples,indent=2)+'\n')
            Path(str(prefix)+'_frame_offsets.json').write_text(json.dumps(yaw_history,indent=2)+'\n')
        task_success=bool(termination=='returned_home_and_saved' and mission.get('home_confirmed') and saved and not failed)
        summary=dict(termination=termination,failed=failed,task_success=task_success,map_saved=saved,
                     wall_elapsed_seconds=time.monotonic()-start_wall,mission=mission,
                     note='SITL only; successful return/save is separate from exploration/3D completeness. '
                          'No collision-contact or physical-aircraft landing claim.')
        Path(str(prefix)+'_result.json').write_text(json.dumps(summary,indent=2)+'\n')
        run.stop()
        if saved:
            try:
                report=evaluate(root,prefix,alignment,args.scene)
                key='shelf_side_coverage' if half else 'inward_wall_support_ratio'
                print('离线{}支持率：{:.2%}（不是全三维完整率）'.format(
                    '货架侧面' if half else '内侧墙面',report[key]),flush=True)
            except Exception as exc:
                Path(str(prefix)+'_audit_error.txt').write_text(str(exc)+'\n')
                print('离线审计未完成；原地图保留：'+str(exc),flush=True)
        print('结果：'+str(prefix)+'_result.json',flush=True)
    return 1 if failed else 0


if __name__=='__main__':
    sys.exit(main())
