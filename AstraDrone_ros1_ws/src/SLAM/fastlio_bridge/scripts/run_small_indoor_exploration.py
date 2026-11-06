#!/usr/bin/env python3
"""Run AstraDrone exploration in the small office or half-area warehouse."""
import argparse
from datetime import datetime
import hashlib
import json
import math
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
from std_msgs.msg import String, Float64, Bool, Empty
from std_srvs.srv import Trigger

from analyze_dynamic_demo_bag import rotation
from run_dynamic_comparison import CaseRunner, cleanup_interrupt, process_running, wait_topic
from run_warehouse_exploration import mission_deadline
from small_indoor_scene import prepare, interior_wall_samples
from small_indoor_startup import wait_heading_ready
from startup_checks import wait_ground_pose
from warehouse_map_audit import read_xyz_pcd, supported, write_xyz_pcd
from experiment_evidence import (EVIDENCE_TOPICS, start_manifest, finish_manifest,
                                 source_fingerprint, snapshot_parameters, run_with_cleanup,
                                 prune_previous_recordings)
from evidence_recording import recording_policy


def suspend_aware_clock():
    """Elapsed real time on Linux, including any time spent suspended."""
    return time.clock_gettime(time.CLOCK_BOOTTIME)


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
    if scene == 'office-50':
        from office50_scene import evaluate as evaluate_office50
        report,missing=evaluate_office50(prefix,points,alignment)
        write_xyz_pcd(str(prefix)+'_wall_missing_reference.pcd',missing)
        metadata=json.loads(Path(str(prefix)+'_scene.json').read_text())
    elif scene == 'cangku-half':
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
    bounds = metadata['bounds_world'] if scene in ('cangku-half','office-50') else [-5,-5,5,5]
    ax.set(xlim=(bounds[0]-1,bounds[2]+1),ylim=(bounds[1]-1,bounds[3]+1),
           xlabel='World X (m), initial rigid alignment',ylabel='World Y (m)',
           title=('Half warehouse' if scene == 'cangku-half' else '50.4m2 furnished office' if scene=='office-50' else 'Small office')+
                 ': actual maintained map and flown trajectory')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(str(prefix)+'_map_and_trajectory.png',dpi=180);plt.close(fig)
    return report


def evaluate_freedom(prefix, alignment):
    """Audit FreeDOM's independent camera_init map against the same shelves."""
    from warehouse_map_audit import audit
    pcd = Path(str(prefix)+'_freedom_static_map_point.pcd')
    points = read_xyz_pcd(pcd)
    if alignment is None:
        raise ValueError('No evaluator world/camera_init alignment; FreeDOM map remains saved')
    scene = json.loads(Path(str(prefix)+'_scene.json').read_text())
    report, missing = audit(scene, points, alignment)
    Path(str(prefix)+'_freedom_shelf_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    write_xyz_pcd(str(prefix)+'_freedom_shelf_missing_reference.pcd',missing)
    r = np.asarray(alignment['map_from_world_rotation'])
    t = np.asarray(alignment['map_from_world_translation'])
    world = (points-t)@r
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax = plt.subplots(figsize=(7,7))
    band = world[(world[:,2]>.25)&(world[:,2]<2.20)]
    ax.scatter(band[::3,0],band[::3,1],s=.3,c='#6956a7',label='FreeDOM static map')
    bounds = scene['bounds_world']
    ax.set(xlim=(bounds[0]-1,bounds[2]+1),ylim=(bounds[1]-1,bounds[3]+1),
           xlabel='World X (m), initial rigid alignment',ylabel='World Y (m)',
           title='Half warehouse: FreeDOM static point cloud')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(str(prefix)+'_freedom_map.png',dpi=180);plt.close(fig)
    return report, len(points)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',choices=('office','cangku-half','office-50'),default='office')
    parser.add_argument('--planner',choices=('baseline','frontier-15cm','coverage-guided'),default='baseline',
                        help='half warehouse: original frontier, matched 15 cm frontier, or FALCON-inspired guidance')
    parser.add_argument('--headless',action='store_true',help='run Gazebo server without either GUI')
    parser.add_argument('--no-gazebo-gui',action='store_true',help='keep RViz only')
    parser.add_argument('--no-rviz',action='store_true')
    parser.add_argument('--keep-open',action='store_true',help='keep simulation after successful return/save')
    parser.add_argument('--output-dir',help='separate directory for diagnostic runs; normal demo default unchanged')
    parser.add_argument('--manual-pedestrian',action='store_true',
                        help='half warehouse only: skip the automatic walk so keyboard control can start immediately')
    parser.add_argument('--right-pedestrian',action='store_true',
                        help='add actor 1 in right-middle aisle; patrol after takeoff')
    parser.add_argument('--right-pedestrian-duration',type=float,default=180.,
                        help='actor 1 movement duration in simulation seconds; legacy default 180')
    parser.add_argument('--ghost-score-scale',type=float,default=1.,
                        help='ten-minute joint policy only: active ghost verification utility multiplier')
    parser.add_argument('--freedom-baseline',action='store_true',
                        help='half warehouse only: parallel FreeDOM static map, never a planner input')
    parser.add_argument('--dufomap-baseline',action='store_true',
                        help='half warehouse online loop only: independent author DUFOMap branch, never planner input')
    parser.add_argument('--dufomap-python',default=None,
                        help='isolated Python with dufomap==1.1.1 native binding')
    parser.add_argument('--dual-map',action='store_true',
                        help='paired read-only original/DUFOMap maintenance; enables DUFOMap, no planning fusion')
    parser.add_argument('--ghost-revisit',action='store_true',
                        help='half warehouse baseline only: investigate dynamic-map ghost hypotheses before return')
    parser.add_argument('--ten-minute',action='store_true',
                        help='independent half-warehouse 600s flight budget, dynamic return reserve and AUTO.LAND')
    parser.add_argument('--lio-voxel-size',type=float,default=None,
                        help='FAST-LIO surface/map voxel in metres; ten-minute default .20, other demos .50')
    parser.add_argument('--mission-policy',choices=('legacy','joint-simple','joint-channel'),default='legacy',
                        help='ten-minute only: independent joint observation policy; legacy remains default')
    parser.add_argument('--quality-target',type=float,default=None,
                        help='ten-minute observed-surface exit target; legacy .85, joint .70; never safety evidence')
    parser.add_argument('--surface-inspection',action='store_true',
                        help='experimental measured-surface/floor inspection; requires ten-minute joint policy')
    recording=parser.add_mutually_exclusive_group()
    recording.add_argument('--record-evidence',dest='record_evidence',action='store_true',
                           help='record scans/poses/truth and provenance for mapping replay; default on for ghost revisit')
    recording.add_argument('--no-record-evidence',dest='record_evidence',action='store_false')
    parser.set_defaults(record_evidence=None)
    parser.add_argument('--evidence-profile',choices=('compact','full'),default=None,
                        help='two recent ten-minute demos default compact; full retains original raw diagnostics')
    parser.add_argument('--evidence-truth-hz',type=float,default=30.,help='compact recorded Gazebo states / Hz; live topics unchanged')
    parser.add_argument('--evidence-map-hz',type=float,default=.05,help='compact recorded full snapshots / Hz')
    parser.add_argument('--keep-evidence-bags',type=int,default=None,
                        help='two recent demos default 1 verified raw bag; 0 preserves all')
    parser.add_argument('--loop-closure-audit',action='store_true',
                        help='half baseline only: run the separate loop backend and record it; no control-frame feedback')
    parser.add_argument('--online-loop',action='store_true',
                        help='half baseline: versioned online global mapping/planning, continuous local flight control')
    parser.add_argument('--vertical-view-audit',action='store_true',
                        help='ghost revisit only: diagnose alternative heights and 3D free-space checks; no altitude commands')
    parser.add_argument('--ghost-challenge-arm',choices=('baseline','revisit'),default=None,
                        help='isolated half-warehouse diagnostic: wait at a residual ghost or revisit it')
    parser.add_argument('--ghost-challenge-max-hits',type=int,default=100,
                        help='challenge-only high-support priority cutoff; normal demo uses 8')
    parser.add_argument('--return-after-goals',type=int,default=0,help='explicit partial-demo return after N reached viewpoints')
    parser.add_argument('--test-seconds',type=float,default=0.,help='TEST ONLY: force cutoff, never task success')
    parser.add_argument('--max-wall-seconds',type=float,default=None,help='hard wall guard: office 1200s, half warehouse 2400s')
    parser.add_argument('--no-wall-time-limit',action='store_true',
                        help='dual-map ten-minute demo: disable runner and explorer wall-clock mission cutoffs')
    parser.add_argument('--no-time-limit',action='store_true',
                        help='dual-map ten-minute demo: no wall-clock or simulated flight duration cap')
    parser.add_argument('--efficient-demo',action='store_true',
                        help='dual-map only: private unused-camera removal, compact XYZ recording and slower diagnostics')
    args=parser.parse_args()
    if not np.isfinite(args.right_pedestrian_duration) or args.right_pedestrian_duration<=0:
        parser.error('--right-pedestrian-duration must be finite and positive')
    args.no_wall_time_limit |= args.no_time_limit
    if args.dual_map:args.dufomap_baseline=True
    if args.efficient_demo and not args.dual_map:
        parser.error('--efficient-demo requires --dual-map')
    if args.lio_voxel_size is None:args.lio_voxel_size=.20 if args.ten_minute else .50
    if not np.isfinite(args.lio_voxel_size) or not .05<=args.lio_voxel_size<=1.:
        parser.error('--lio-voxel-size must be finite and between .05 and 1 metre')
    if (args.mission_policy!='legacy' or args.quality_target is not None) and not args.ten_minute:
        parser.error('mission-policy and quality-target overrides require --ten-minute')
    if args.surface_inspection and (not args.ten_minute or args.mission_policy=='legacy'):
        parser.error('--surface-inspection requires --ten-minute and a joint mission policy')
    if args.quality_target is None:
        args.quality_target=.70 if args.mission_policy!='legacy' else .85
    if not np.isfinite(args.quality_target) or not .5<=args.quality_target<=1.:
        parser.error('--quality-target must be finite in [.5,1]')
    if args.online_loop:
        args.loop_closure_audit=True;args.ghost_revisit=True
    if args.ten_minute and (args.scene not in ('cangku-half','office-50') or args.planner!='baseline' or args.ghost_challenge_arm):
        parser.error('--ten-minute requires a supported baseline indoor scene')
    if args.ten_minute:args.ghost_revisit=True
    recent_demo=args.ten_minute and (args.online_loop or args.mission_policy!='legacy')
    if args.evidence_profile is None:args.evidence_profile='compact' if recent_demo else 'full'
    if args.keep_evidence_bags is None:args.keep_evidence_bags=1 if recent_demo else 0
    try:
        recording_policy(EVIDENCE_TOPICS,args.evidence_profile,args.evidence_truth_hz,args.evidence_map_hz)
        if args.keep_evidence_bags<0:raise ValueError('keep-evidence-bags must be nonnegative')
    except ValueError as error:parser.error(str(error))
    if args.manual_pedestrian and args.scene != 'cangku-half':
        parser.error('--manual-pedestrian is currently supported only by --scene cangku-half')
    if args.right_pedestrian and args.scene!='cangku-half':
        parser.error('--right-pedestrian requires --scene cangku-half')
    if not math.isfinite(args.ghost_score_scale) or not 1.<=args.ghost_score_scale<=2.:
        parser.error('--ghost-score-scale must be finite in [1,2]')
    if args.ghost_score_scale!=1. and not (args.ten_minute and args.mission_policy!='legacy'):
        parser.error('--ghost-score-scale requires ten-minute joint policy')
    if args.freedom_baseline and args.scene != 'cangku-half':
        parser.error('--freedom-baseline is currently supported only by --scene cangku-half')
    if args.dufomap_baseline and not (args.scene in ('cangku-half','office-50') and args.online_loop):
        parser.error('--dufomap-baseline requires a supported indoor scene and --online-loop')
    if args.planner != 'baseline' and args.scene != 'cangku-half':
        parser.error('--planner coverage-guided requires --scene cangku-half')
    if args.ghost_revisit and (args.scene not in ('cangku-half','office-50') or args.planner != 'baseline'):
        parser.error('--ghost-revisit requires a supported baseline indoor scene')
    if args.loop_closure_audit and (args.scene not in ('cangku-half','office-50') or args.planner != 'baseline'):
        parser.error('--loop-closure-audit requires a supported baseline indoor scene')
    if args.scene=='office-50' and not (args.dual_map and args.ten_minute and args.online_loop and args.surface_inspection):
        parser.error('--scene office-50 uses the existing dual-map ten-minute surface demo; use office50_dual_map_demo.sh')
    if args.record_evidence is None:
        args.record_evidence = args.ghost_revisit or args.loop_closure_audit
    if args.loop_closure_audit and not args.record_evidence:
        parser.error('--loop-closure-audit requires evidence recording')
    if args.vertical_view_audit and not args.ghost_revisit:
        parser.error('--vertical-view-audit requires --ghost-revisit')
    if args.ghost_challenge_arm and not args.ghost_revisit:
        parser.error('--ghost-challenge-arm requires --ghost-revisit so both arms track the same hypotheses')
    if not 1 <= args.ghost_challenge_max_hits <= 1000:
        parser.error('--ghost-challenge-max-hits must be in [1,1000]')
    if args.max_wall_seconds is None:
        args.max_wall_seconds = 1500. if args.ten_minute else 2400. if args.scene == 'cangku-half' else 1200.
    try:
        if args.no_wall_time_limit and not (args.dual_map and args.ten_minute):
            raise ValueError('unlimited duration requires a dual-map ten-minute demo')
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
    if args.dufomap_baseline:
        # Keep the venv executable path: resolving its symlink selects system Python.
        args.dufomap_python=str(Path(args.dufomap_python or root/'.venvs/dufomap/bin/python').expanduser().absolute())
        try:
            subprocess.check_output([args.dufomap_python,'-c',
                'import sys,json;sys.path.insert(0,sys.argv[1]);from dufomap_core import native_provenance;print(json.dumps(native_provenance()))',
                str(scripts)],stderr=subprocess.STDOUT,timeout=20)
        except (OSError,subprocess.SubprocessError) as error:
            parser.error('DUFOMap preflight failed; run scripts/run_sh/install_dufomap_demo.sh: '+str(error))
    half = args.scene in ('cangku-half','office-50')
    guided = args.planner == 'coverage-guided'
    matched_frontier = args.planner == 'frontier-15cm'
    folder=root/('office50_dual_map_runs' if args.scene=='office-50' else
                 'half_warehouse_dual_map_runs' if args.dual_map else
                 'half_warehouse_dufomap_runs' if args.dufomap_baseline else
                 'half_warehouse_joint_channel_runs' if args.mission_policy!='legacy' else
                 'half_warehouse_online_loop_ten_minute_runs' if args.online_loop and args.ten_minute else
                 'half_warehouse_online_loop_runs' if args.online_loop else
                 'half_warehouse_ten_minute_runs' if args.ten_minute else
                 'ghost_revisit_challenge_runs' if args.ghost_challenge_arm else
                 'half_warehouse_ghost_revisit_runs' if args.ghost_revisit else
                 'half_warehouse_falcon_guided_runs' if guided else
                 'half_warehouse_frontier_15cm_runs' if matched_frontier else
                 'half_warehouse_demo_runs' if half else 'small_indoor_demo_runs')
    if args.output_dir is not None:folder=Path(args.output_dir).expanduser().resolve()
    folder.mkdir(parents=True,exist_ok=True)
    prefix=folder/(('office50_dual_map_' if args.scene=='office-50' else
                    'warehouse_half_dual_map_' if args.dual_map else
                    'warehouse_half_dufomap_' if args.dufomap_baseline else
                    'warehouse_half_'+args.mission_policy.replace('-','_')+'_' if args.mission_policy!='legacy' else
                    'warehouse_half_ten_minute_' if args.ten_minute else
                    'warehouse_half_challenge_'+args.ghost_challenge_arm+'_' if args.ghost_challenge_arm else
                    'warehouse_half_ghost_revisit_' if args.ghost_revisit else
                    'warehouse_half_falcon_guided_' if guided else
                    'warehouse_half_frontier_15cm_' if matched_frontier else
                    'warehouse_half_' if half else 'office_')+datetime.now().strftime('%Y%m%d_%H%M%S'))
    run=CaseRunner(root,prefix,os.environ.copy())
    return run_with_cleanup(run,_run_case,args,root,px4,prefix,run)


def _run_case(args,root,px4,prefix,run):
    scripts=Path(__file__).resolve().parent
    half=args.scene in ('cangku-half','office-50')
    office50=args.scene=='office-50'
    guided=args.planner=='coverage-guided'
    matched_frontier=args.planner=='frontier-15cm'
    lock=threading.Lock();mission={};history=[];samples=[];alignment=None;freedom_alignment=None;truth_snapshot=None;yaw_history=[]
    pedestrian_truth=[];extra_pedestrian_truth=[]
    evidence_manifest=None; recorder=None
    endurance=None;landing_started=False;landing_report=None
    saver=False;saved=False;freedom_started=False;freedom_saved=False;failed=False;termination='interrupted';subs=[]
    dufomap_started=False;dufomap_saved=False;dual_map_started=False;dual_map_saved=False
    start_wall=time.monotonic()
    start_boottime=suspend_aware_clock()
    def save_after_global_rebuild(timeout=360.):
        # Nonzero loops can require a full history replay. The endurance path
        # lands BEFORE this bounded wait; never spend battery hovering for it.
        next_notice=time.monotonic()
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            response=rospy.ServiceProxy('/demo/save_final_loop_map',Trigger)()
            if response.success:
                print('全局回环最终地图保存：'+response.message,flush=True)
                return True
            if time.monotonic()>=next_notice:
                print('等待冻结后的全局地图一致重建：'+response.message,flush=True)
                next_notice=time.monotonic()+15.
            time.sleep(.3)
        print('全局地图未能在落地后等待窗口内完成一致重建。',flush=True)
        return False
    try:
        from sim_sensor_geometry import load_sim_geometry
        sensor_calibration=load_sim_geometry(root,px4)
        Path(str(prefix)+'_sensor_geometry.json').write_text(json.dumps(sensor_calibration,indent=2)+'\n')
        if office50:
            from office50_scene import prepare as prepare_office50
            world=prepare_office50(root,prefix)
        elif half:
            from half_warehouse_scene import prepare as prepare_half
            world=prepare_half(root,prefix,extra_pedestrian=args.right_pedestrian,
                               extra_pedestrian_duration=args.right_pedestrian_duration)
        else:
            world=prepare(root,prefix)
        scene_metadata=json.loads(Path(str(prefix)+'_scene.json').read_text())
        evidence_manifest=start_manifest(root,prefix,vars(args),world)
        evidence_manifest['sensor_geometry']=sensor_calibration
        evidence_manifest['input_sha256'].update(sensor_calibration['source_sha256'])
        efficiency=None
        if args.efficient_demo:
            from demo_efficiency import prepare_camera_free_vehicle
            efficiency=prepare_camera_free_vehicle(root,px4,prefix)
            evidence_manifest['input_sha256'].update(efficiency['input_sha256'])
        spawn=scene_metadata['spawn_world']
        core=run.start('core',['roscore'])
        until=time.monotonic()+30
        while not rosgraph.is_master_online():
            if core.poll() is not None or time.monotonic()>until:
                raise RuntimeError('ROS master startup failed')
            time.sleep(.2)
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('small_indoor_runner',anonymous=True,disable_signals=True)
        if args.ten_minute:
            rospy.set_param('/warehouse_explorer/joint_active_ghost_gain_multiplier',args.ghost_score_scale)
        if args.dual_map:
            rospy.set_param('/online_loop_mapper/dual_map_checkpoint_stride_frames',50)
        if args.dual_map and args.no_time_limit:
            # Keep the paired branches aligned. Stop exploration with ample
            # mapping capacity left for a planned return and frozen export.
            rospy.set_param('/online_loop_mapper/max_history_frames',12000)
            rospy.set_param('/online_loop_mapper/max_history_megabytes',3072)
            rospy.set_param('/warehouse_explorer/loop_map_return_at_frames',10500)
        for node in ('/fastlio_bridge','/fastlio_cloud_bridge'):
            rospy.set_param(node+'/imu_origin_in_body',sensor_calibration['imu_origin_in_body'])
        rospy.set_param('/static_map_builder/lidar_origin_in_odom',sensor_calibration['laser_origin_in_body'])
        rospy.set_param('/warehouse_explorer/lidar_origin_in_body',sensor_calibration['laser_origin_in_body'])
        if args.ten_minute:
            from endurance_landing import EnduranceLanding
            endurance=EnduranceLanding()
            print(('本轮不设任务墙钟和仿真飞行时长上限；仍核查返航与AUTO.LAND落地。' if args.no_time_limit else
                   '续航上限：解锁至上锁600秒仿真时间，保留60秒储备；电脑墙钟运行时间随仿真实时率变化。'),flush=True)
        if args.record_evidence:
            if args.evidence_profile=='compact':
                recorder=run.start('evidence',[sys.executable,str(scripts/'evidence_recording.py'),
                    '--output',str(prefix)+'_evidence.bag','--truth-hz',str(args.evidence_truth_hz),
                    '--map-hz',str(args.evidence_map_hz)]+(['--light'] if args.efficient_demo else []))
                ready=Path(str(prefix)+'_evidence_recording.json');until=time.monotonic()+8.
                while not ready.is_file():
                    if recorder.poll() is not None or time.monotonic()>until:
                        raise RuntimeError('Compact recorder did not become ready')
                    time.sleep(.05)
                print('精简录包：真值%.1fHz、地图快照%.3fHz，建图扫描/里程计保持原频率；只影响记录。'%(
                    args.evidence_truth_hz,args.evidence_map_hz),flush=True)
            else:
                recorder=run.start('evidence',['rosbag','record','--lz4','--buffsize=256',
                    '-O',str(prefix)+'_evidence.bag']+list(EVIDENCE_TOPICS))
        def status(msg):
            nonlocal mission
            if landing_started:return
            try:
                d=json.loads(msg.data)
            except (ValueError,TypeError):
                return
            with lock:
                mission=d
                if (not args.efficient_demo or not history or
                    d.get('stamp',0)-history[-1].get('stamp',0)>=2. or
                    d.get('mission_phase')!=history[-1].get('mission_phase')):
                    history.append(dict(wall_elapsed=time.monotonic()-start_wall,**d))
        def truth(msg):
            nonlocal truth_snapshot
            truth_snapshot=(msg,time.monotonic())
            if half:
                stamp=rospy.Time.now().to_sec()
                if not office50 and stamp>0 and (not pedestrian_truth or stamp-pedestrian_truth[-1]['sim_stamp']>=.5):
                    try:
                        pedestrian_pose=msg.pose[msg.name.index('0')]
                    except ValueError:
                        return
                    p=pedestrian_pose.position
                    pedestrian_truth.append(dict(sim_stamp=stamp,world_xyz=[p.x,p.y,p.z]))
                if (args.right_pedestrian or office50) and stamp>0 and (not extra_pedestrian_truth or stamp-extra_pedestrian_truth[-1]['sim_stamp']>=.5):
                    if '1' in msg.name:
                        p=msg.pose[msg.name.index('1')].position
                        extra_pedestrian_truth.append(dict(sim_stamp=stamp,world_xyz=[p.x,p.y,p.z]))
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
            if not half and (not args.freedom_baseline or freedom_alignment is not None):
                truth_sub.unregister()
        def freedom_odom(msg):
            nonlocal freedom_alignment
            snapshot=truth_snapshot
            if freedom_alignment is not None or snapshot is None or time.monotonic()-snapshot[1]>.05:
                return
            try:
                body=snapshot[0].pose[snapshot[0].name.index('iris_mid360')]
            except ValueError:
                return
            estimate=msg.pose.pose
            r=rotation(estimate.orientation)@rotation(body.orientation).T
            t=np.asarray([estimate.position.x,estimate.position.y,estimate.position.z])-r@np.asarray(
                [body.position.x,body.position.y,body.position.z])
            freedom_alignment=dict(map_from_world_rotation=r.tolist(),map_from_world_translation=t.tolist(),
                odom_stamp=msg.header.stamp.to_sec(),truth_receipt_age_wall=time.monotonic()-snapshot[1],
                note='Evaluator-only world to camera_init initial alignment; no drift correction or planner input')
            Path(str(prefix)+'_freedom_alignment.json').write_text(json.dumps(freedom_alignment,indent=2)+'\n')
            if alignment is not None and not half:
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
        if args.freedom_baseline:
            subs.append(rospy.Subscriber('/Odometry',Odometry,freedom_odom,queue_size=1))
        setup=px4/'Tools/simulation/gazebo-classic/setup_gazebo.bash'
        launch=root/'simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch'
        cmd='source {} {} {}; exec roslaunch {} world:={} gui:={} interactive:=false x:={} y:={}'.format(
            *[shlex.quote(str(p)) for p in (setup,px4,px4/'build/px4_sitl_default',launch,world)],
            'false' if args.headless or args.no_gazebo_gui else 'true',spawn[0],spawn[1])
        if efficiency is not None:
            cmd+=' sdf:='+shlex.quote(efficiency['vehicle_sdf'])
        gazebo=run.start('gazebo',['bash','-lc',cmd])
        wait_topic('/livox/lidar',PointCloud2,120,gazebo)
        stable=wait_ground_pose(gazebo)
        wait_heading_ready(gazebo)
        if args.surface_inspection and endurance is not None:
            descent=endurance.configure_slow_descent()
            Path(str(prefix)+'_landing_configuration.json').write_text(json.dumps(descent,indent=2)+'\n')
            print('慢降参数已读回：AUTO.LAND 0.20m/s，近地爬行0.12m/s，落地竖向阈值0.08m/s。',flush=True)
        print('PX4未解锁且航向连续稳定；速度坐标转换器另行处理解锁时的航向对齐变化。',flush=True)
        stable=wait_ground_pose(gazebo)
        print('地面基准稳定，Z={:.3f}m；启动FAST-LIO与自主探索。'.format(stable.pose.position.z),flush=True)
        lio=run.start('fastlio',['roslaunch','fast_lio','mapping_mid360.launch','rviz:=false','sim_lidar:=true','dynamic_filter:=false',
                               'surface_voxel_size:='+str(args.lio_voxel_size),
                               'map_voxel_size:='+str(args.lio_voxel_size)])
        wait_topic('/Odometry',Odometry,90,lio)
        if args.freedom_baseline:
            run.start('freedom',['roslaunch','freedom','astra_mid360_baseline.launch','output:='+str(prefix)])
            freedom_started=True
        if office50:
            extra=run.start('pedestrian_office',[sys.executable,str(scripts/'warehouse_pedestrian.py'),
                '_scene_file:='+str(prefix)+'_scene.json','_model_name:=1','__name:=office50_pedestrian'])
            wait_topic('/demo/pedestrian/1/ready',Bool,20,extra)
        elif half:
            # One pose owner moves the included model and guards it against
            # the half-scene collision boxes; truth is not sent to Explorer.
            human=run.start('pedestrian',[sys.executable,str(scripts/'warehouse_pedestrian.py'),
                                          '_scene_file:='+str(prefix)+'_scene.json'])
            wait_topic('/demo/pedestrian/ready',Bool,20,human)
            if args.right_pedestrian:
                extra=run.start('pedestrian_right',[sys.executable,str(scripts/'warehouse_pedestrian.py'),
                    '_scene_file:='+str(prefix)+'_scene.json','_model_name:=1','__name:=warehouse_pedestrian_right'])
                wait_topic('/demo/pedestrian/1/ready',Bool,20,extra)
        if args.ghost_challenge_arm:
            # Challenge arms may share a diagnostic high-support priority
            # cutoff. It no longer blocks candidate creation or authorizes
            # deletion; measured free rays remain mandatory.
            rospy.set_param('/static_map_builder/ghost_suspect_max_hits',
                            args.ghost_challenge_max_hits)
        if args.dufomap_baseline:
            if args.dual_map:
                manager=run.start('dual_map',[sys.executable,str(scripts/'dual_map_manager.py'),
                    '_output:='+str(prefix),'_checkpoint_stride_frames:=50'])
                wait_topic('/dual_map/status',String,30,manager);dual_map_started=True
            dufo=run.start('dufomap',[args.dufomap_python,str(scripts/'dufomap_adapter.py'),
                '_output:='+str(prefix),'_resolution:=0.15','_sensor_error:=0.20',
                '_pose_cells:=1','_threads:=2',
                '_dual_map_checkpoint_stride_frames:='+('50' if args.dual_map else '0'),
                '_max_frames:='+('12000' if args.dual_map and args.no_time_limit else '6500'),
                '_lidar_origin_in_body:='+json.dumps(sensor_calibration['laser_origin_in_body'])])
            wait_topic('/dufomap/status',String,30,dufo)
            dufomap_started=True
            if not args.headless and not args.no_rviz:
                config='half_warehouse_dual_map.rviz' if args.dual_map else 'half_warehouse_dufomap.rviz'
                run.start('dufomap_rviz',['rviz','-d',str(scripts.parent/'rviz'/config)])
        run.start('exploration',['roslaunch','fastlio_bridge',
                  'half_warehouse_ten_minute.launch' if args.ten_minute else
                  'half_warehouse_falcon_guided.launch' if guided else
                  'half_warehouse_frontier_15cm.launch' if matched_frontier else
                  'half_warehouse_exploration.launch' if half else 'small_indoor_exploration.launch',
                  'rviz:='+('false' if args.headless or args.no_rviz or args.dufomap_baseline else 'true'),
                  'output:='+str(prefix),'return_after_goals:='+str(args.return_after_goals)]+
                  (['enable_ghost_revisit:=true'] if args.ghost_revisit else [])+
                  (['enable_loop_closure:=true'] if args.loop_closure_audit else [])+
                  (['online_loop:=true'] if args.online_loop else [])+
                  (['vertical_view_audit:=true'] if args.vertical_view_audit else [])+
                  (['mission_policy:='+args.mission_policy,'quality_target:='+str(args.quality_target)]
                    if args.ten_minute else [])+
                  (['surface_inspection:=true'] if args.surface_inspection else [])+
                  (['max_mission_wall_seconds:=0'] if args.no_wall_time_limit else [])+
                  (['unlimited_flight:=true'] if args.no_time_limit else []))
        rospy.wait_for_service('/demo/save_map',timeout=40);saver=True
        snapshot_parameters(prefix,rospy.get_param,evidence_manifest)
        if half and not office50 and not args.manual_pedestrian:
            run.start('walk_once',[sys.executable,str(scripts/'exploration_pedestrian_walk.py')])
        if args.right_pedestrian or office50:
            run.start('right_patrol',[sys.executable,str(scripts/'pedestrian_patrol.py'),
                '_scene_file:='+str(prefix)+'_scene.json','_output:='+str(prefix)+'_pedestrian_1_patrol.json']+
                (['_uav_yield_distance:=2.5'] if args.dual_map and args.no_time_limit and args.right_pedestrian else []))
            print('50.4平方米办公室：12件静态家具，单名行人起飞后运动60秒再停止。' if office50 else
                  '新增右侧中部行人1：world (3,-12)，在x=1..5,y=-12往返；起飞后移动{:g}秒仿真时间再停止。'.format(
                      args.right_pedestrian_duration),flush=True)
        if args.ghost_challenge_arm:
            run.start('ghost_challenge',[sys.executable,str(scripts/'ghost_revisit_challenge.py'),
                                         str(prefix),'--arm',args.ghost_challenge_arm])
        print(('50.4平方米办公室双图探索' if office50 else '半面积仓库动态鬼影主动复扫' if args.ghost_revisit else
               '半面积仓库15cm覆盖分区引导' if guided else
               '半面积仓库15cm原前沿规划' if matched_frontier else
               '半面积仓库' if half else '小办公室')+
              '自主探索已启动：未知几何选点 → 补扫 → 返航驻留 → 保存。',flush=True)
        if half and not office50:
            mode='行人可立即键盘控制' if args.manual_pedestrian else '单名行人自动行走一轮后可键盘接管'
            cruise=rospy.get_param('/mavros_avoidance_controller/max_navigation_speed',.3)
            print('半面积仓库：'+mode+'；导航速度上限{:.2f}m/s，探索规划机心安全距离0.80m（另计网格量化保护）。'.format(cruise),flush=True)
            print('另开终端运行 bash docs/演示启动命令/行人键盘控制.sh（WASD为世界XY，空格停止，Q退出）。',flush=True)
            if args.freedom_baseline:
                print('FreeDOM旁路建图中：同航次扫描/位姿，独立camera_init静态图，不接管选点或避障。',flush=True)
            if args.loop_closure_audit:
                print(('在线回环全局地图与探索规划已启用；控制航点转换回连续局部坐标。' if args.online_loop else
                       '回环旁路审计已启用；优化轨迹独立录包，探索与控制继续使用连续里程计。'),flush=True)
        elif not office50:
            print('本轮无行人/语义网络/回环；输出前缀：'+str(prefix),flush=True)
        # CLOCK_MONOTONIC stops in system suspend. The hard wall guard must
        # count overnight sleep, then save a clearly partial run on resume.
        if args.no_wall_time_limit:
            deadline=(suspend_aware_clock()+args.test_seconds if args.test_seconds else float('inf'))
            timeout_reason='explicit_test_timeout_partial' if args.test_seconds else 'no_wall_time_limit'
        else:
            deadline,timeout_reason=mission_deadline(suspend_aware_clock(),args.test_seconds,args.max_wall_seconds)
            if start_boottime+args.max_wall_seconds < deadline:
                deadline=start_boottime+args.max_wall_seconds
                timeout_reason='wall_guard_timeout_partial'
        while suspend_aware_clock()<deadline or (args.keep_open and mission.get('state')=='COMPLETE' and mission.get('home_confirmed')):
            for name,process in run.processes:
                if process.poll() is not None:
                    raise RuntimeError(name+' 提前退出，请检查对应日志')
            state=mission.get('state')
            if endurance is not None and not args.no_time_limit and endurance.forced_landing_due():
                # Stale planning must never bypass the arm-clock budget. This
                # is a PARTIAL failure, never a successful return.
                landing_started=True;failed=True;termination='flight_budget_failsafe_landing_partial'
                if args.online_loop:
                    try:rospy.ServiceProxy('/uav1/loop_slam/freeze_mapping',Trigger)()
                    except Exception:pass
                if evidence_manifest is not None:
                    evidence_manifest['mapping_cutoff_sim_stamp']=rospy.Time.now().to_sec()
                landing_report=endurance.land(total=600.,reserve=60.,unlimited=args.no_time_limit)
                if args.online_loop:
                    saved=save_after_global_rebuild()
                else:
                    response=rospy.ServiceProxy('/demo/save_map',Trigger)()
                    saved=bool(response.success)
                break
            if state in ('STALLED_UNSAFE','RETURN_FAILED'):
                termination=state.lower();failed=True
                break
            if state=='COMPLETE' and mission.get('home_confirmed'):
                termination='returned_home_and_saved'
                if endurance is not None:
                    # Freeze the mapping snapshot before descent. Landing
                    # scans remain in the evidence bag, not this mission map.
                    landing_started=True
                    if args.online_loop:
                        try:
                            cutoff=rospy.ServiceProxy('/uav1/loop_slam/freeze_mapping',Trigger)()
                            if not cutoff.success:
                                failed=True;print(cutoff.message,file=sys.stderr,flush=True)
                        except Exception as error:
                            failed=True;print('全局建图截止失败，仍执行预算内降落：'+str(error),file=sys.stderr,flush=True)
                    else:
                        response=rospy.ServiceProxy('/demo/save_map',Trigger)()
                        saved=bool(response.success)
                    if evidence_manifest is not None:
                        evidence_manifest['mapping_cutoff_sim_stamp']=mission.get('stamp')
                    landing_report=endurance.land(total=600.,reserve=60.,unlimited=args.no_time_limit)
                    if args.online_loop:saved=save_after_global_rebuild()
                    failed |= not (saved and landing_report['passed'])
                    termination=('returned_home_landed_and_saved' if not failed else
                                 'endurance_landing_or_budget_failed')
                    break
                if not args.keep_open:
                    if args.online_loop:
                        cutoff=rospy.ServiceProxy('/uav1/loop_slam/freeze_mapping',Trigger)()
                        if not cutoff.success:raise RuntimeError(cutoff.message)
                        saved=save_after_global_rebuild()
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
        # A stalled planner, wall guard or Ctrl-C must not skip the ten-minute
        # landing phase and immediately destroy an airborne SITL process.
        if endurance is not None and not landing_started and endurance.is_armed() and rosgraph.is_master_online():
            landing_started=True;failed=True
            print('任务提前结束，先执行AUTO.LAND再保存部分地图。',flush=True)
            if args.online_loop:
                try:
                    cutoff=rospy.ServiceProxy('/uav1/loop_slam/freeze_mapping',Trigger)()
                    if not cutoff.success:
                        failed=True;print('提前结束后的全局建图截止失败：'+cutoff.message,file=sys.stderr,flush=True)
                except Exception as exc:
                    failed=True;print('提前结束后的全局建图截止失败：'+str(exc),file=sys.stderr,flush=True)
            try:
                landing_report=endurance.land(total=600.,reserve=60.,unlimited=args.no_time_limit)
            except Exception as exc:
                print('提前结束后的降落未确认：'+str(exc),file=sys.stderr,flush=True)
        if saver and rosgraph.is_master_online() and not (args.online_loop and saved) and not (args.ten_minute and landing_started and saved):
            try:
                if args.online_loop:
                    cutoff=rospy.ServiceProxy('/uav1/loop_slam/freeze_mapping',Trigger)()
                    if not cutoff.success:raise RuntimeError(cutoff.message)
                    saved=save_after_global_rebuild()
                    failed |= not saved
                else:
                    response=rospy.ServiceProxy('/demo/save_map',Trigger)()
                    saved=bool(response.success);failed |= not saved
                    print('最终点云保存：{} {}'.format(saved,response.message),flush=True)
            except Exception as exc:
                failed=True;print('冻结后全局地图保存失败：'+str(exc) if args.online_loop else '保存失败：'+str(exc),file=sys.stderr,flush=True)
            if args.online_loop and not saved:
                # Preserve an explicitly partial map even when the frozen
                # consistency gate fails; DUFOMap still validates it separately.
                try:
                    response=rospy.ServiceProxy('/demo/save_map',Trigger)()
                    saved=bool(response.success)
                    print('部分点云保存：{} {}'.format(saved,response.message),flush=True)
                except Exception as exc:
                    print('部分点云保存失败：'+str(exc),file=sys.stderr,flush=True)
        flight_failed=failed
        if dufomap_started and saved and rosgraph.is_master_online():
            try:
                response=rospy.ServiceProxy('/demo/dufomap/save_map',Trigger)()
                dufomap_saved=bool(response.success)
                if not dufomap_saved:raise RuntimeError(response.message)
                print('DUFOMap独立清理地图保存：'+response.message,flush=True)
            except Exception as error:
                print('DUFOMap地图未通过完整输入/保存校验：'+str(error),file=sys.stderr,flush=True)
        if args.dufomap_baseline:failed |= not dufomap_saved
        if dual_map_started and dufomap_saved and rosgraph.is_master_online():
            try:
                response=rospy.ServiceProxy('/demo/dual_map/save_maps',Trigger)()
                dual_map_saved=bool(response.success)
                if not dual_map_saved:raise RuntimeError(response.message)
                print('双图共同截止快照保存：'+response.message,flush=True)
            except Exception as error:
                print('双图配对未通过，原图/DUFO独立结果保留：'+str(error),file=sys.stderr,flush=True)
        if args.dual_map:failed |= not dual_map_saved
        freedom_map=Path(str(prefix)+'_freedom_static_map_point.pcd')
        if freedom_started and rosgraph.is_master_online():
            pub=None
            try:
                pub=rospy.Publisher('/freedom/save_map',Empty,queue_size=1)
                until=time.monotonic()+10
                while pub.get_num_connections()==0 and time.monotonic()<until:
                    time.sleep(.1)
                if pub.get_num_connections()==0:
                    raise RuntimeError('FreeDOM save subscriber unavailable')
                pub.publish(Empty())
                done=rospy.wait_for_message('/freedom/save_map_done',Bool,timeout=30)
                freedom_saved=bool(done.data and freedom_map.is_file() and freedom_map.stat().st_size>0)
                if not freedom_saved:
                    raise RuntimeError('FreeDOM save incomplete')
                print('FreeDOM独立静态地图保存：'+str(freedom_map),flush=True)
            except Exception as exc:
                failed=True;print('FreeDOM地图保存失败：'+str(exc),file=sys.stderr,flush=True)
            finally:
                if pub is not None:
                    pub.unregister()
        for sub in subs:
            sub.unregister()
        if endurance is not None:endurance.close()
        with lock:
            if args.efficient_demo and mission and (not history or history[-1].get('stamp')!=mission.get('stamp')):
                history.append(dict(wall_elapsed=time.monotonic()-start_wall,**mission))
            Path(str(prefix)+'_status_history.json').write_text(json.dumps(history,indent=2)+'\n')
            Path(str(prefix)+'_flight_samples.json').write_text(json.dumps(samples,indent=2)+'\n')
            Path(str(prefix)+'_frame_offsets.json').write_text(json.dumps(yaw_history,indent=2)+'\n')
            if half and not office50:
                Path(str(prefix)+'_pedestrian_truth.json').write_text(json.dumps({
                    'source':'/gazebo/model_states model 0, evaluator only; receipt time from /clock',
                    'sampling_interval_sim_seconds':.5,
                    'samples':pedestrian_truth},indent=2)+'\n')
            if args.right_pedestrian or office50:
                Path(str(prefix)+'_pedestrian_1_truth.json').write_text(json.dumps(dict(
                    source='/gazebo/model_states model 1, evaluator only; receipt time from /clock',
                    sampling_interval_sim_seconds=.5,samples=extra_pedestrian_truth),indent=2)+'\n')
        ghost_history_path=None
        ghost_history_validated=False
        if saved and args.ghost_revisit:
            try:
                map_metadata=json.loads(Path(str(prefix)+'_final_map.json').read_text())
                ghost_history_path=map_metadata.get('ghost_history')
                if ghost_history_path:
                    history_bytes=Path(ghost_history_path).read_bytes()
                    ghost_history_validated=(hashlib.sha256(history_bytes).hexdigest()==
                        map_metadata.get('ghost_history_sha256'))
            except (OSError,ValueError,TypeError):
                ghost_history_validated=False
        task_success=bool(termination in ('returned_home_and_saved','returned_home_landed_and_saved') and mission.get('home_confirmed')
                          and saved and not flight_failed and
                          (not args.ghost_revisit or ghost_history_validated))
        first_status=history[0]['wall_elapsed'] if history else None
        return_status=next((r for r in history if r.get('mission_phase')=='RETURNING'),None)
        last_status=history[-1]['wall_elapsed'] if history else None
        timing=dict(startup_wall_seconds=first_status,
                    before_return_wall_seconds=return_status['wall_elapsed']-first_status
                        if return_status is not None and first_status is not None else None,
                    return_and_dwell_wall_seconds=last_status-return_status['wall_elapsed']
                        if return_status is not None else None,
                    sim_seconds_at_return=return_status.get('mission_sim_seconds')
                        if return_status is not None else None)
        configuration={k:rospy.get_param(k,None) for k in (
            '/mavros_avoidance_controller/max_navigation_speed','/mavros_avoidance_controller/max_total_speed',
            '/warehouse_explorer/planner_mode','/warehouse_explorer/resolution',
            '/warehouse_explorer/coverage_region_size',
            '/warehouse_explorer/lookahead','/warehouse_explorer/planning_hz','/warehouse_explorer/settle_seconds',
            '/warehouse_explorer/clearance','/warehouse_explorer/observed_map_acceptance',
            '/warehouse_explorer/surface_quality_target','/warehouse_explorer/max_mission_seconds',
            '/warehouse_explorer/ghost_revisit_enabled','/static_map_builder/enable_ghost_revisit',
            '/static_map_builder/ghost_suspect_max_hits',
            '/warehouse_explorer/ghost_height_below','/warehouse_explorer/ghost_height_above',
            '/warehouse_explorer/ghost_expected_speed_mps',
            '/warehouse_explorer/ghost_revisit_budget_seconds',
            '/warehouse_explorer/ghost_cleared_max_groups')
            } if rosgraph.is_master_online() else {}
        final_surface=mission.get('surface_quality',{})
        final_quality=final_surface.get('observed_surface_quality_ratio')
        target=configuration.get('/warehouse_explorer/surface_quality_target')
        map_quality_accepted=bool(target is not None and final_quality is not None and final_quality>=target
            and final_surface.get('observed_surface_voxels',0)>=1000
            and not final_surface.get('surface_evidence_capacity_limited',False)
            and not mission.get('grid_capacity_limited',False))
        if recorder is not None and recorder.poll() is None:
            try:
                os.killpg(recorder.pid,signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                recorder.wait(timeout=30)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(recorder.pid,signal.SIGTERM)
                except ProcessLookupError:
                    pass
                recorder.wait(timeout=5)
        if evidence_manifest is not None:
            evidence_manifest['source_unchanged_during_run']=(
                source_fingerprint(root)['sha256']==evidence_manifest['source']['sha256'])
            evidence_manifest=finish_manifest(prefix,evidence_manifest,
                recorder.poll() if recorder is not None else None,configuration)
        evidence_ok=bool(not args.record_evidence or (evidence_manifest and
                         evidence_manifest.get('replay_inputs_available') and
                         evidence_manifest.get('recording_integrity_valid',True) and
                         evidence_manifest.get('source_unchanged_during_run') and
                         (not args.loop_closure_audit or evidence_manifest.get('loop_audit_valid')) and
                         (not args.online_loop or evidence_manifest.get('online_loop_valid'))))
        failed |= not evidence_ok
        task_success &= evidence_ok
        active_wall_elapsed=time.monotonic()-start_wall
        elapsed_including_suspend=suspend_aware_clock()-start_boottime
        saved_map_snapshot=None;saved_metadata_sha256=None
        if saved:
            saved_path=Path(str(prefix)+'_final_map.json')
            saved_bytes=saved_path.read_bytes();saved_map_snapshot=json.loads(saved_bytes)
            saved_metadata_sha256=hashlib.sha256(saved_bytes).hexdigest()
        summary=dict(termination=termination,failed=failed,task_success=task_success,map_saved=saved,
                     saved_map_snapshot=saved_map_snapshot,saved_map_metadata_sha256=saved_metadata_sha256,
                     endurance_landing=landing_report,
                     ghost_history_path=ghost_history_path,
                     ghost_history_validated=ghost_history_validated if args.ghost_revisit else None,
                     evidence_manifest=str(prefix)+'_evidence_manifest.json' if evidence_manifest else None,
                     evidence_recording_valid=evidence_ok if args.record_evidence else None,
                     freedom_baseline=args.freedom_baseline,freedom_map_saved=freedom_saved,
                     freedom_demo_success=bool(task_success and freedom_saved) if args.freedom_baseline else None,
                     freedom_point_map=str(freedom_map) if args.freedom_baseline else None,
                     dufomap_baseline=args.dufomap_baseline,dufomap_map_saved=dufomap_saved,
                     dufomap_demo_success=bool(task_success and dufomap_saved) if args.dufomap_baseline else None,
                     dufomap_point_map=str(prefix)+'_dufomap_static_map.pcd' if args.dufomap_baseline else None,
                     dual_map_enabled=args.dual_map,dual_map_saved=dual_map_saved,
                     dual_map_demo_success=bool(task_success and dual_map_saved) if args.dual_map else None,
                     map_quality_accepted=map_quality_accepted,
                     wall_elapsed_seconds=active_wall_elapsed,
                     elapsed_including_suspend_seconds=elapsed_including_suspend,
                     suspended_seconds_estimate=max(0.,elapsed_including_suspend-active_wall_elapsed),
                     mission=mission,
                     run_options=vars(args),timing=timing,configuration=configuration,
                     note='SITL only; successful return/save is separate from exploration/3D completeness. '
                          'No collision-contact or physical-aircraft landing claim.')
        Path(str(prefix)+'_result.json').write_text(json.dumps(summary,indent=2)+'\n')
        run.stop()
        if saved:
            try:
                report=evaluate(root,prefix,alignment,args.scene)
                key='shelf_side_coverage' if half and not office50 else 'inward_wall_support_ratio'
                print('离线{}支持率：{:.2%}（不是全三维完整率）'.format(
                    '货架侧面' if half and not office50 else '内侧墙面',report[key]),flush=True)
            except Exception as exc:
                Path(str(prefix)+'_audit_error.txt').write_text(str(exc)+'\n')
                print('离线审计未完成；原地图保留：'+str(exc),flush=True)
        if freedom_saved:
            try:
                freedom_report,freedom_count=evaluate_freedom(prefix,freedom_alignment)
                print('FreeDOM离线货架侧面支持率：{:.2%}，{}点（不是全三维完整率）'.format(
                    freedom_report['shelf_side_coverage'],freedom_count),flush=True)
            except Exception as exc:
                Path(str(prefix)+'_freedom_audit_error.txt').write_text(str(exc)+'\n')
                print('FreeDOM离线审计未完成；地图保留：'+str(exc),flush=True)
        print('结果：'+str(prefix)+'_result.json',flush=True)
        if evidence_ok and args.record_evidence:
            try:
                retention=prune_previous_recordings(prefix,args.keep_evidence_bags)
                print('录包保留策略：'+json.dumps(retention,ensure_ascii=False),flush=True)
            except Exception as error:
                print('录包清理未完成，已有文件保留：'+str(error),flush=True)
    return 1 if failed else 0


if __name__=='__main__':
    sys.exit(main())
