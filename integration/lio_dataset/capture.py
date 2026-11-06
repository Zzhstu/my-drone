#!/usr/bin/env python3
"""Static 20 sqm warehouse: LiDAR+internal IMU localization, monocular RGB export."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
import rospy
import rosgraph
import yaml
from sensor_msgs.msg import CameraInfo,PointCloud2,CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import State,ExtendedState
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'integration/roomgs'))
from integration.r3live.calibration import read_geometry
from integration.roomgs.lowflow_camera import make_rgb_camera
from integration.roomgs.geometry import matrix_from_pose
from integration.roomgs.resources import ResourceMeter
from integration.roomgs.provenance import snapshot_sources
from integration.roomgs.capture import offline_tool
from run_dynamic_comparison import CaseRunner,process_running,wait_topic,finish_recording,cleanup_interrupt
from startup_checks import wait_ground_pose
from small_indoor_startup import wait_heading_ready


def dump(path,value):path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(args):
    for name in ['rosmaster','gzserver','px4','r3live_mapping','fastlio_mapping']:
        if process_running(name):raise RuntimeError('Existing simulation is running: '+name)
    output=Path(args.output_dir).resolve() if args.output_dir else ROOT/'lio_dataset_runs'/datetime.now().strftime('warehouse20_%Y%m%d_%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    if shutil.disk_usage(output).free<28*2**30:raise RuntimeError('Need 28 GiB free disk reserve')
    px4=Path(os.environ.get('PX4_DIR','/home/a/PX4-Autopilot'))
    original=px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/iris_mid360.sdf'
    lidar=ROOT/'simulation/astra_gazebo_models/mid360/mid360.sdf'
    original_camera=ROOT/'simulation/astra_gazebo_models/D435i/model.sdf'
    tree=ET.parse(original);model=tree.getroot().find('model')
    for e in list(model):
        if (e.tag=='include' and e.findtext('uri')=='model://fpv_cam') or (e.tag=='joint' and e.get('name')=='fpv_cam_joint'):model.remove(e)
    vehicle=output/'iris_mid360.sdf';tree.write(str(vehicle),encoding='utf-8',xml_declaration=True)
    camera=make_rgb_camera(original_camera,output/'sensor_models')
    geometry=read_geometry(vehicle,lidar,camera);dump(output/'sensor_geometry.json',geometry)
    world=output/'room.world';shutil.copyfile(ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.world',world)
    world_root=ET.parse(world).getroot().find('world')
    if world_root.findall('actor') or world_root.findall('plugin'):raise RuntimeError('Dataset scene must have no moving actors/plugins')
    reference=ROOT/'integration/lio_dataset/reference'
    manifest=json.loads((reference/'target_manifest.json').read_text())
    scene=json.loads((ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.json').read_text())
    mesh=ROOT/scene['rack']['original_mesh']
    if sha(mesh)!=manifest['mesh_sha256']:raise RuntimeError('Runtime rack mesh differs from frozen targets')
    if sha(world)!=manifest['world_sha256'] or sha(reference/'mesh_reference.npz')!=manifest['reference_sha256']:
        raise RuntimeError('Fixed audit scene/reference mismatch')
    for name in ['mesh_reference.npz','target_manifest.json']:shutil.copyfile(reference/name,output/name)
    plan_path=Path(args.plan).resolve() if args.plan else ROOT/'integration/roomgs/config/warehouse20_inspection.json'
    plan=json.loads(plan_path.read_text())
    for key in ['world_sha256','mesh_sha256','reference_sha256']:
        if plan[key]!=manifest[key]:raise RuntimeError('Plan does not match frozen scene')
    dump(output/'inspection_plan.json',plan)
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_LOG_DIR=str(output/'ros_logs'),GAZEBO_MASTER_URI='http://127.0.0.1:%d'%(port+1))
    env=os.environ.copy();env['GAZEBO_MODEL_PATH']=str(output/'sensor_models')+':'+env.get('GAZEBO_MODEL_PATH','')
    runner=CaseRunner(ROOT,output/'demo',env);meter=ResourceMeter(output)
    sources=list((ROOT/'integration/lio_dataset').glob('*.py'))+[ROOT/'integration/roomgs/flight.py',ROOT/'integration/roomgs/tracking.py',ROOT/'integration/r3live/calibration.py',ROOT/'integration/r3live/input_bridge.py',ROOT/'integration/roomgs/lowflow_camera.py',ROOT/'integration/roomgs/camera_profiles.py',ROOT/'integration/roomgs/warehouse20_audit.py',original,lidar,original_camera,world,vehicle,camera,plan_path,ROOT/'AstraDrone_ros1_ws/devel/lib/fast_lio/fastlio_mapping']
    sources += [ROOT/'integration/roomgs'/name for name in ['capture.py','resources.py','geometry.py','provenance.py','sensor_guard.py','mission.py','warehouse20_mission.py']]
    sources += [ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'/name for name in ['run_dynamic_comparison.py','startup_checks.py','small_indoor_startup.py']]
    sources += [reference/'mesh_reference.npz',reference/'target_manifest.json',mesh,ROOT/'scripts/run_sh/warehouse20_lio_dataset.sh']
    sources += [ROOT/name for name in scene['rack']['textures']]
    fingerprints={str(p):sha(p) for p in sources}
    dump(output/'source_manifest.json',fingerprints)
    snapshot_sources(output,fingerprints)
    report=dict(success=False,scope='SITL static warehouse, FAST-LIO-only RGB pose dataset; no 3DGS training',
        output=str(output),world_sha256=sha(world),pedestrians=False,pose_source='FAST-LIO LiDAR + MID360 internal IMU; no RGB updates',
        pose_time_policy='Image acquisition nanoseconds; bracketed LIO linear translation and SO3 SLERP; no extrapolation',
        flow=dict(rgb_hz=5,width=640,height=480,jpeg_quality=92,lidar_stride=6,map_voxel_m=.05,pose_max_gap_ms=150),
        source_unchanged=False,runtime_source_sha256=fingerprints)
    latest_truth=[None];aligned=[False];terminal=[None];recorder=None
    def truth(msg):
        if not aligned[0] and 'iris_mid360' in msg.name:latest_truth[0]=matrix_from_pose(msg.pose[msg.name.index('iris_mid360')])
    def odom(msg):
        if not aligned[0] and latest_truth[0] is not None:
            alignment=latest_truth[0]@np.array(geometry['body_from_imu'])@np.linalg.inv(matrix_from_pose(msg.pose.pose))
            dump(output/'audit_alignment.json',dict(world_from_estimator=alignment.tolist(),stamp_ns=msg.header.stamp.to_nsec(),policy='One initial frozen alignment; OFFLINE coverage audit only, never published to estimator/control/exported poses'))
            aligned[0]=True
    def mission(msg):
        value=json.loads(msg.data)
        if value.get('phase')=='DONE':terminal[0]=value
    start=time.monotonic()
    try:
        core=runner.start('core',['roscore','-p',str(port)]);deadline=time.monotonic()+20
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline or core.poll() is not None:raise RuntimeError('Private ROS master failed')
            time.sleep(.1)
        rospy.set_param('/use_sim_time',True);rospy.init_node('lio_dataset_run',anonymous=True,disable_signals=True)
        launch_tree=ET.parse(ROOT/'simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch')
        ET.SubElement(launch_tree.getroot(),'param',name='/mavros/vision_pose/tf/listen',value='false',type='bool')
        launch=output/'sitl.launch';launch_tree.write(str(launch),encoding='utf-8',xml_declaration=True)
        setup=px4/'Tools/simulation/gazebo-classic/setup_gazebo.bash'
        command='source {} {} {}; exec roslaunch {} world:={} sdf:={} gui:={} interactive:=false x:=-1.3 y:=-1.2 z:=0.06'.format(*[shlex.quote(str(p)) for p in [setup,px4,px4/'build/px4_sitl_default',launch,world,vehicle]],'false' if args.headless else 'true')
        gazebo=runner.start('gazebo',['bash','-lc',command]);wait_topic('/livox/lidar',PointCloud2,100,gazebo)
        info=wait_topic('/camera/color/camera_info',CameraInfo,30,gazebo)
        if not info.K[0] or not info.K[4]:raise RuntimeError('Invalid camera intrinsics')
        dump(output/'camera_input.json',dict(K=list(info.K),D=list(info.D),width=info.width,height=info.height,
            image_topic='/camera/color/image_raw',timestamp='Gazebo RGB acquisition header, shared /clock with MID360 IMU and LiDAR',optical_axes='x right, y down, z forward',camera_model='PINHOLE' if not any(info.D) else 'OPENCV'))
        config=yaml.safe_load((ROOT/'AstraDrone_ros1_ws/src/SLAM/FAST_LIO/config/mid360.yaml').read_text())
        config['common']['lid_topic']='/lio_dataset/cloud';config['preprocess']['lidar_type']=4
        config['mapping']['extrinsic_T']=geometry['lidar_in_imu_translation'];config['mapping']['extrinsic_R']=[1,0,0,0,1,0,0,0,1]
        config['mapping']['extrinsic_est_en']=False;config['mapping']['det_range']=25.
        config['pcd_save']['pcd_save_en']=False
        config.update(feature_extract_enable=False,point_filter_num=1,max_iteration=3,filter_size_surf=.10,filter_size_map=.10,cube_side_length=100.,runtime_pos_log_enable=False,dynamic_filter=dict(enable=False))
        for key,value in config.items():rospy.set_param('/'+key,value)
        (output/'fastlio.yaml').write_text(yaml.safe_dump(config))
        subscriptions=[rospy.Subscriber('/gazebo/model_states',ModelStates,truth,queue_size=1),rospy.Subscriber('/Odometry',Odometry,odom,queue_size=10),rospy.Subscriber('/roomgs/mission',String,mission,queue_size=10)]
        runner.start('collector',['/usr/bin/python3',str(ROOT/'integration/lio_dataset/collector.py'),'_output:='+str(output)])
        mapper=runner.start('fastlio',['rosrun','fast_lio','fastlio_mapping'])
        runner.start('sensors',['/usr/bin/python3',str(ROOT/'integration/lio_dataset/sensors.py'),'_image_topic:=/camera/color/image_raw'])
        wait_topic('/Odometry',Odometry,45,mapper);wait_topic('/lio_dataset/optical_pose',PoseStamped,30,mapper)
        rospy.set_param('/lio_dataset_pose_bridge/imu_origin_in_body',np.array(geometry['body_from_imu'])[:3,3].tolist())
        bridge=runner.start('pose_bridge',['rosrun','fastlio_bridge','fastlio_bridge_node','__name:=lio_dataset_pose_bridge',
            '_use_px4_initial_alignment:=false','_publish_mavros_vision_pose:=true',
            '_path_topic:=/lio_dataset/body_path','/uav1/fastlio/odom:=/lio_dataset/body_odom','/uav1/mavros/vision_pose/pose:=/mavros/vision_pose/pose'])
        wait_topic('/lio_dataset/body_odom',Odometry,15,bridge)
        topics=['/clock','/lio_dataset/cloud','/livox/imu','/lio_dataset/rgb/compressed','/camera/color/camera_info','/Odometry','/lio_dataset/optical_pose','/lio_dataset/body_odom','/roomgs/mission','/mavros/state','/mavros/extended_state']
        dump(output/'recorded_topics.json',dict(topics=topics,policy='Only used adapted LiDAR, main IMU, one JPEG RGB stream, calibration and estimated states; no truth or depth'))
        recorder=runner.start('recorder',['rosbag','record','--lz4','--buffsize=32','-O',str(output/'sensors.bag')]+topics)
        if not args.no_flight:
            wait_ground_pose(gazebo);wait_heading_ready(gazebo)
            settings=dict(scene='warehouse20',route=plan['waypoints'],native_odom_topic='/lio_dataset/body_odom',max_wall_seconds=args.max_wall_seconds,
                sweep_stations=plan.get('sweep_station_indices',[]),position_tolerance=.10,yaw_tolerance=.06,dwell_seconds=.8,sweep_dwell_seconds=2.,integral_feedback=True)
            imu_from_lidar=np.eye(4);imu_from_lidar[:3,3]=geometry['lidar_in_imu_translation']
            settings['body_from_lidar']=(np.array(geometry['body_from_imu'])@imu_from_lidar).tolist()
            for key,value in settings.items():rospy.set_param('/roomgs_inspection_flight/'+key,value)
            runner.start('flight',['/usr/bin/python3',str(ROOT/'integration/roomgs/flight.py')])
        active=rospy.get_time();abort=rospy.Publisher('/roomgs/abort',String,queue_size=1,latch=True)
        last_resource=0.;abort_time=None
        with (output/'resources.jsonl').open('w',buffering=1) as metrics:
            while not rospy.is_shutdown():
                if mapper.poll() is not None or gazebo.poll() is not None:raise RuntimeError('Critical simulation/estimator exited')
                if time.monotonic()-last_resource>5:
                    sample=meter.sample();metrics.write(json.dumps(sample)+'\n');dump(output/'resource_latest.json',sample);last_resource=time.monotonic()
                    reason=sample['reason'] or ('Wall time budget exceeded' if time.monotonic()-start>args.max_wall_seconds else None)
                    if reason:
                        report['resource_abort']=reason
                        if args.no_flight:raise RuntimeError(reason)
                        abort_time=abort_time or time.monotonic();abort.publish(String(data=reason))
                if abort_time and time.monotonic()-abort_time>90:raise RuntimeError('Resource abort landing timeout')
                if args.no_flight and rospy.get_time()-active>=args.stationary_seconds:break
                if terminal[0] is not None:break
                time.sleep(.2)
        if not args.no_flight:
            state=wait_topic('/mavros/state',State,5,gazebo);landed=wait_topic('/mavros/extended_state',ExtendedState,5,gazebo)
            if state.armed or landed.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:raise RuntimeError('Not confirmed landed/disarmed')
            report.update(landed=True,disarmed=True,flight_result=terminal[0])
        exported=rospy.ServiceProxy('/lio_dataset/finalize',Trigger)()
        report['export_status']=json.loads(exported.message)
        if not exported.success:raise RuntimeError('Dataset export failed: '+exported.message)
        finish_recording(recorder,output/'sensors.bag');recorder=None
        from quality import filter_dataset
        report['quality_filter']=filter_dataset(output)
        report['source_unchanged']=all(sha(p)==value for p,value in fingerprints.items())
        report['resource_usage']=meter.sample();report['wall_seconds']=time.monotonic()-start
        dump(output/'result.json',report)
        import subprocess
        subprocess.run(['/usr/bin/python3',str(ROOT/'integration/lio_dataset/export.py'),str(output)],check=True,env=env)
        if not args.no_flight:
            code='import sys,json;sys.path.insert(0,"'+str(ROOT/'integration/roomgs')+'");from warehouse20_audit import audit;print(json.dumps(audit(sys.argv[1],"static_map.npz","FAST-LIO")))'
            audit_env=env.copy();audit_env.pop('PYTHONPATH',None)
            subprocess.run([str(ROOT/'.envs/roomgs/bin/python'),'-c',code,str(output)],check=True,env=audit_env)
            report['coverage']=json.loads((output/'coverage.json').read_text())
        validation=json.loads((output/'dataset_validation.json').read_text())
        report['dataset_validated']=validation['passed']
        report['final_dataset_frames']=validation['images']
        report['stationary_preflight_passed']=bool(args.no_flight and report['dataset_validated'] and report['source_unchanged'])
        report['success']=bool(not args.no_flight and terminal[0]['success'] and report['dataset_validated'] and report['source_unchanged'] and not report.get('resource_abort'))
        report['complete_map']=bool(report.get('coverage',{}).get('passed'))
        report['geometry_complete']=report.get('coverage',{}).get('geometry_fraction')==1.
        report['photo_complete']=report.get('coverage',{}).get('multiview_fraction')==1.
        dump(output/'result.json',report)
        if not (report['stationary_preflight_passed'] if args.no_flight else report['success']):raise RuntimeError('Capture did not pass: see result.json')
        print('DATASET CAPTURE FINISHED '+str(output),flush=True)
    except BaseException as error:
        report.update(error=str(error),wall_seconds=time.monotonic()-start)
        dump(output/'result.json',report);raise
    finally:
        signal.signal(signal.SIGINT,cleanup_interrupt);runner.stop()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir');parser.add_argument('--headless',action='store_true')
    parser.add_argument('--no-flight',action='store_true');parser.add_argument('--stationary-seconds',type=float,default=20.)
    parser.add_argument('--max-wall-seconds',type=float,default=5400);parser.add_argument('--plan')
    run(parser.parse_args())
