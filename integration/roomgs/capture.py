#!/usr/bin/env python3
"""Isolated real R3LIVE simulation: sensor measurements -> LIO/VIO -> RGB PCD."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shlex
import signal
import shutil
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
import yaml
import rospy
import rosgraph
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from integration.r3live.calibration import read_geometry
from scene import make_world
from geometry import matrix_from_pose
from resources import ResourceMeter

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts'))
from run_dynamic_comparison import CaseRunner, process_running, wait_topic, cleanup_interrupt, finish_recording
from startup_checks import wait_ground_pose
from small_indoor_startup import wait_heading_ready


def dump(path, value): path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def offline_tool(name,arguments):
    env=os.environ.copy();env.pop('PYTHONPATH',None)
    env['OPENBLAS_NUM_THREADS']='1'
    prefix=ROOT/'.envs/roomgs'
    env['LD_LIBRARY_PATH']=str(prefix/'lib')+':'+env.get('LD_LIBRARY_PATH','')
    subprocess.run([str(prefix/'bin/python'),str(ROOT/'integration/roomgs'/name),*arguments],env=env,check=True)


def run(args):
    for process in ('rosmaster', 'gzserver', 'px4', 'r3live_mapping', 'fastlio_mapping'):
        if process_running(process): raise RuntimeError('请先退出已有演示；不终止用户进程：'+process)
    binary = ROOT/'r3live_roomgs_ws/devel/lib/r3live/r3live_mapping'
    if not binary.is_file(): raise RuntimeError('先运行 bash scripts/run_sh/build_roomgs_demo.sh')
    output = Path(args.output_dir) if args.output_dir else ROOT/'roomgs_runs'/datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    meter=ResourceMeter(output)
    if shutil.disk_usage(output).free<28*2**30:raise RuntimeError('Need 8 GiB run budget plus 20 GiB disk reserve')
    dump(output/'resource_profile.json',dict(name='low_flow',rgb_hz=5.,rgb_encoding='acquisition JPEG',jpeg_quality=92,
        lidar_stride=6,imu_rate='unchanged native sensor rate',collector_cache_frames=16,
        training_images='lazy single-frame decode',training_depth_cache_frames=32,
        limits=meter.limits,final_total_budget_gib=8.,training_storage_reserve_gib=1.,geometry_and_photo_acceptance='unchanged frozen 4407 targets for warehouse20'))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0)); port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port, ROS_IP='127.0.0.1',
                      ROS_LOG_DIR=str(output/'ros_logs'), GAZEBO_MASTER_URI='http://127.0.0.1:%d'%(port+1))
    env = os.environ.copy(); px4 = Path(env.get('PX4_DIR', '/home/a/PX4-Autopilot'))
    runner = CaseRunner(ROOT, output/'demo', env)
    world = output/'room.world'
    if args.scene=='warehouse20':
        shutil.copyfile(ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.world',world)
        shutil.copyfile(ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.json',output/'audit_scene.json')
        offline_tool('warehouse20_audit.py',['--prepare',str(output)])
        plan_path=Path(args.inspection_plan).resolve() if args.inspection_plan else ROOT/'integration/roomgs/config/warehouse20_inspection.json'
        plan=json.loads(plan_path.read_text())
        definition=json.loads((output/'target_manifest.json').read_text())
        for key in ('world_sha256','mesh_sha256','reference_sha256'):
            if plan[key]!=definition[key]:raise RuntimeError('Inspection plan/reference mismatch: '+key)
        shutil.copyfile(plan_path,output/'inspection_plan.json')
        spawn=[-1.3,-1.2,.06]
    else:
        make_world(world);spawn=[-3.,-2.,.06]
    original = px4/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_mid360/iris_mid360.sdf'
    tree = ET.parse(original); model = tree.getroot().find('model')
    for element in list(model):
        if element.tag == 'include' and element.findtext('uri') == 'model://fpv_cam': model.remove(element)
        elif element.tag == 'joint' and element.get('name') == 'fpv_cam_joint': model.remove(element)
    vehicle = output/'iris_mid360.sdf'; tree.write(str(vehicle), encoding='utf-8', xml_declaration=True)
    lidar = ROOT/'simulation/astra_gazebo_models/mid360/mid360.sdf'
    camera_original = ROOT/'simulation/astra_gazebo_models/D435i/model.sdf'
    from lowflow_camera import make_rgb_camera
    camera=make_rgb_camera(camera_original,output/'sensor_models')
    env['GAZEBO_MODEL_PATH']=str(output/'sensor_models')+':'+env.get('GAZEBO_MODEL_PATH','')
    geometry = read_geometry(vehicle, lidar, camera)
    dump(output/'sensor_geometry.json', geometry)
    report = dict(success=False, output=str(output), mode='SITL_room_inspection',scene=args.scene,
                  source='hku-mars/r3live', revision='6143a38537f28cb36eb24e9bbe2e39c8f7967157',
                  scope='Actual R3LIVE room inspection; strict finite-resolution completeness gate before offline gsplat. '
                        'Fixed SITL flight uses PX4 local pose fused with R3LIVE external vision.',
                  input_source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (original,lidar,camera)})
    source_files=[p for p in (ROOT/'integration/roomgs').glob('*.py') if p.name not in ('train.py','viewer.py')]
    source_files += [original,lidar,camera_original,camera,camera.with_name('model.config'),camera.with_name('policy.json')]
    source_files += [ROOT/'integration/roomgs/r3live.patch',ROOT/'integration/r3live/calibration.py',ROOT/'integration/r3live/input_bridge.py',ROOT/'integration/r3live/scene.py']
    source_files += [ROOT/'scripts/run_sh/roomgs_demo.sh', ROOT/'scripts/run_sh/build_roomgs_demo.sh', binary,
                     ROOT/'AstraDrone_ros1_ws/devel/lib/fastlio_bridge/fastlio_bridge_node',
                     ROOT/'AstraDrone_ros1_ws/devel/lib/offboard/loop_closure_demo_flight']
    source_files += [p for p in (ROOT/'third_party/r3live_roomgs/r3live').rglob('*') if p.is_file() and p.suffix in ('.hpp','.h','.cpp','.txt','.xml')]
    if args.scene=='warehouse20':
        source_files += [ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.world',ROOT/'simulation/astra_gazebo_worlds/cangku_20sqm.json']
        source_files += [plan_path]
        source_files += list((ROOT/'simulation/astra_gazebo_models/Shelf_with_ARUco_boxes/meshes').glob('*'))
    report['runtime_source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_files}
    from provenance import snapshot_sources
    snapshot_sources(output,report['runtime_source_sha256'])
    odoms=[]; camera_odoms=[]; statuses=[]; vio_statuses=[]; finished=[False];terminal=[None]
    subscriptions=[]; rgb_parts={}; rgb_subs={}
    start=time.monotonic(); recorder=None
    truth_latest=[None]; alignment=[None]
    def truth(msg):
        if alignment[0] is not None:return
        if 'iris_mid360' in msg.name:
            truth_latest[0]=matrix_from_pose(msg.pose[msg.name.index('iris_mid360')])
    def odom(msg):
        odoms.append([msg.header.stamp.to_sec(),msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
        if alignment[0] is None and truth_latest[0] is not None:
            b_i=np.array(geometry['body_from_imu'])
            alignment[0]=truth_latest[0]@b_i@np.linalg.inv(matrix_from_pose(msg.pose.pose))
            dump(output/'audit_alignment.json',dict(world_from_estimator=alignment[0].tolist(),
                 policy='single initial rigid alignment frozen before flight; OFFLINE audit only',stamp=msg.header.stamp.to_sec()))
    def camera_odom(msg): camera_odoms.append(msg.header.stamp.to_sec())
    def status(msg): statuses.append(json.loads(msg.data))
    def vio_status(msg): vio_statuses.append(json.loads(msg.data))
    def done(msg): finished[0]=bool(msg.data)
    def mission(msg):
        value=json.loads(msg.data)
        if value.get('phase')=='DONE':terminal[0]=value
    try:
        core=runner.start('core',['roscore','-p',str(port)])
        deadline=time.monotonic()+20
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline or core.poll() is not None: raise RuntimeError('Private ROS master failed')
            time.sleep(.1)
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('astra_r3live_demo',anonymous=True,disable_signals=True)
        setup=px4/'Tools/simulation/gazebo-classic/setup_gazebo.bash'
        launch=ROOT/'simulation/px4_sim_files/px4_launch/astra_launch/astra_example.launch'
        # Local MAVROS installation listens to camera_init->body TF by default.
        # Override only this private launch to accept our measured pose topic.
        launch_tree=ET.parse(launch)
        ET.SubElement(launch_tree.getroot(),'param',name='/mavros/vision_pose/tf/listen',value='false',type='bool')
        launch=output/'sitl.launch'
        launch_tree.write(str(launch),encoding='utf-8',xml_declaration=True)
        command='source {} {} {}; exec roslaunch {} world:={} sdf:={} gui:={} interactive:=false x:={} y:={} z:={}'.format(
            *[shlex.quote(str(p)) for p in (setup,px4,px4/'build/px4_sitl_default',launch,world,vehicle)],
            'false' if args.headless else 'true',*spawn)
        gazebo=runner.start('gazebo',['bash','-lc',command])
        wait_topic('/livox/lidar',PointCloud2,100,gazebo)
        deadline=time.monotonic()+40; topic=None
        while not topic and time.monotonic()<deadline:
            for name,kind in rospy.get_published_topics():
                if kind=='sensor_msgs/Image' and name.endswith('/color/image_raw'): topic=name; break
            if not topic: time.sleep(.2)
        if not topic: raise RuntimeError('No actual color image publisher; inspect Gazebo camera plugin log')
        info_topic=topic.rsplit('/',1)[0]+'/camera_info'
        info=wait_topic(info_topic,CameraInfo,15,gazebo)
        image=wait_topic(topic,Image,15,gazebo)
        if list(info.K)==[0.]*9 or (image.width,image.height)!=(info.width,info.height): raise RuntimeError('Invalid camera calibration')
        config=dict(r3live_common=dict(map_output_dir=str(output),enable_gui=not args.headless,
            record_offline_map=0,pub_pt_minimum_views=1,minimum_pts_size=.06,image_downsample_ratio=1.,
            estimate_i2c_extrinsic=0,estimate_intrinsic=0,maximum_vio_tracked_pts=300,maximum_image_buffer=4,
            tracker_minimum_depth=.5,tracker_maximum_depth=25.,track_windows_size=20,append_global_map_point_step=1),
            r3live_lio=dict(lidar_in_imu_translation=geometry['lidar_in_imu_translation'],max_iteration=3,
                fov_degree=360.,voxel_downsample_size_surf=.2,voxel_downsample_size_axis_z=.2,filter_size_map=.2,
                lio_update_point_step=2,cube_side_length=100.,lidar_time_delay=0.),
            r3live_vio=dict(image_width=info.width,image_height=info.height,camera_intrinsic=list(info.K),
                camera_dist_coeffs=(list(info.D)+[0.]*5)[:5],camera_ext_R=geometry['camera_in_imu_rotation'],
                camera_ext_t=geometry['camera_in_imu_translation']))
        for key,value in config.items(): rospy.set_param('/'+key,value)
        rospy.set_param('/LiDAR_pointcloud_topic','/r3live/input/cloud')
        rospy.set_param('/IMU_topic','/livox/imu'); rospy.set_param('/Image_topic','/r3live/input/image')
        (output/'r3live.yaml').write_text(yaml.safe_dump(config))
        dump(output/'camera_input.json',dict(image_topic=topic,info_topic=info_topic,K=list(info.K),D=list(info.D),
            width=info.width,height=info.height,encoding='acquisition JPEG92 decoded BGR8 for R3LIVE',source_encoding=image.encoding))
        subscriptions += [rospy.Subscriber('/aft_mapped_to_init',Odometry,odom,queue_size=100),
                          rospy.Subscriber('/camera_odom',Odometry,camera_odom,queue_size=100),
                          rospy.Subscriber('/r3live/input/status',String,status),
                          rospy.Subscriber('/r3live/vio_status',String,vio_status),
                          rospy.Subscriber('/roomgs/finished',Bool,done),
                          rospy.Subscriber('/roomgs/mission',String,mission),
                          rospy.Subscriber('/gazebo/model_states',ModelStates,truth,queue_size=1)]
        # Only measured RGB/IMU/LiDAR and estimator outputs. No Gazebo pose enters R3LIVE.
        record_topics=['/clock','/livox/lidar','/livox/imu',topic,info_topic,'/aft_mapped_to_init','/camera_odom',
                       '/track_img','/r3live/input/status','/r3live/vio_status','/roomgs/finished','/roomgs/mission','/roomgs/optical_pose','/mavros/state','/mavros/extended_state',
                       '/mavros/vision_pose/pose','/mavros/local_position/pose','/r3live/body_odom','/r3live/input/image']
        if args.scene=='warehouse20':
            # Preserve every RGB image actually delivered to R3LIVE, plus all
            # raw LiDAR/IMU. Avoid duplicate unused 30Hz RGB and track overlays.
            record_topics=[name for name in record_topics if name not in (topic,'/track_img','/livox/lidar','/r3live/input/image')]
            record_topics+=['/r3live/input/cloud','/r3live/input/image/compressed']
        dump(output/'recorded_topics.json',dict(topics=record_topics,
            policy='Estimator acquisition JPEG and adapted LiDAR, raw main IMU, calibration and estimated outputs; source dense LiDAR/unused camera frames/tracking overlays omitted in warehouse mode'))
        recorder=runner.start('recorder',['rosbag','record','--lz4','--buffsize=32','-O',str(output/'sensors.bag')]+record_topics)
        if args.scene=='warehouse20':
            rospy.set_param('/roomgs_dataset_collector/minimum_interval',.18)
            rospy.set_param('/roomgs_dataset_collector/minimum_translation',.04)
            rospy.set_param('/roomgs_dataset_collector/maximum_idle_interval',.5)
        rospy.set_param('/roomgs_dataset_collector/compressed_input',True)
        rospy.set_param('/roomgs_dataset_collector/cache_frames',16)
        if args.auxiliary_capture:
            rospy.set_param('/roomgs_dataset_collector/minimum_translation',0.)
            rospy.set_param('/roomgs_dataset_collector/maximum_idle_interval',.2)
        runner.start('collector',['/usr/bin/python3',str(ROOT/'integration/roomgs/collector.py'),'_output:='+str(output)])
        mapper=runner.start('r3live',['rosrun','r3live','r3live_mapping'])
        adapter=runner.start('input',['/usr/bin/python3',str(ROOT/'integration/r3live/input_bridge.py'),
                                     '_image_topic:='+topic,'_profile:=sim','_image_rate:=5','_jpeg_quality:=92','_point_step:=6'])
        wait_topic('/aft_mapped_to_init',Odometry,45,mapper)
        wait_topic('/camera_odom',Odometry,45,mapper)
        wait_topic('/roomgs/optical_pose',PoseStamped,45,mapper)
        if not args.no_flight:
            # This MID360 airframe has GPS/magnetometer disabled and requires EV.
            # Reuse the project's frame/lever-arm adapter, without PX4 alignment
            # or another estimator. Sensor-derived R3LIVE pose is the only EV.
            rospy.set_param('/r3live_pose_bridge/imu_origin_in_body',np.array(geometry['body_from_imu'])[:3,3].tolist())
            bridge=runner.start('pose_bridge',['rosrun','fastlio_bridge','fastlio_bridge_node',
                '__name:=r3live_pose_bridge','_use_px4_initial_alignment:=false','_publish_mavros_vision_pose:=true',
                '_path_topic:=/r3live/body_path','/Odometry:=/aft_mapped_to_init',
                '/uav1/fastlio/odom:=/r3live/body_odom',
                '/uav1/mavros/vision_pose/pose:=/mavros/vision_pose/pose'])
            wait_topic('/r3live/body_odom',Odometry,10,bridge)
            wait_ground_pose(gazebo)
            wait_heading_ready(gazebo)
        if not args.headless:
            runner.start('rviz',['rviz','-d',str(ROOT/'integration/r3live/demo.rviz')])
        if not args.no_flight:
            rospy.set_param('/roomgs_inspection_flight/scene',args.scene)
            if args.scene=='warehouse20':
                rospy.set_param('/roomgs_inspection_flight/route',plan['waypoints'])
                rospy.set_param('/roomgs_inspection_flight/max_wall_seconds',5400)
                rospy.set_param('/roomgs_inspection_flight/sweep_stations',plan.get('sweep_station_indices',[]))
                for key,default in [('position_tolerance',.10),('yaw_tolerance',.06),('dwell_seconds',.8),('sweep_dwell_seconds',1.5),('integral_feedback',False)]:
                    rospy.set_param('/roomgs_inspection_flight/'+key,plan.get(key,default))
                imu_from_lidar=np.eye(4);imu_from_lidar[:3,3]=geometry['lidar_in_imu_translation']
                body_from_lidar=np.array(geometry['body_from_imu'])@imu_from_lidar
                rospy.set_param('/roomgs_inspection_flight/body_from_lidar',body_from_lidar.tolist())
            runner.start('flight',['/usr/bin/python3',str(ROOT/'integration/roomgs/flight.py')])
        active=rospy.Time.now().to_sec();last_resource=-100.;resource_abort=None;resource_abort_time=None
        abort=rospy.Publisher('/roomgs/abort',String,queue_size=1,latch=True)
        metrics=(output/'resources.jsonl').open('w',buffering=1)
        while not rospy.is_shutdown():
            if mapper.poll() is not None or gazebo.poll() is not None: raise RuntimeError('Critical node exited')
            if time.monotonic()-last_resource>5.:
                measured=meter.sample();metrics.write(json.dumps(measured)+'\n');dump(output/'resource_latest.json',measured)
                if measured['reason'] and resource_abort is None:resource_abort=measured['reason']
                last_resource=time.monotonic()
            if time.monotonic()-start>args.max_wall_seconds:resource_abort=resource_abort or 'Demo wall-clock timeout'
            if resource_abort:
                if args.no_flight:raise RuntimeError(resource_abort)
                if resource_abort_time is None:resource_abort_time=time.monotonic()
                abort.publish(String(data=resource_abort))
                if time.monotonic()-resource_abort_time>90:raise RuntimeError('Landing after resource abort timed out: '+resource_abort)
            if args.no_flight and rospy.Time.now().to_sec()-active>=args.stationary_seconds: break
            if not args.no_flight and (finished[0] or terminal[0] is not None): break
            time.sleep(.2)
        metrics.close()
        report['resource_usage']=meter.sample()
        report['resource_abort']=resource_abort
        if not args.no_flight:
            state=wait_topic('/mavros/state',State,5,gazebo)
            landed=wait_topic('/mavros/extended_state',ExtendedState,5,gazebo)
            if state.armed or landed.landed_state!=ExtendedState.LANDED_STATE_ON_GROUND:
                raise RuntimeError('Flight finished notification without verified landed and disarmed state')
            report.update(landed=True,disarmed=True)
        rospy.wait_for_service('/r3live/save_map',timeout=5)
        response=rospy.ServiceProxy('/r3live/save_map',Trigger)()
        if not response.success: raise RuntimeError(response.message)
        until=time.monotonic()+35
        while not (output/'save_complete.txt').is_file():
            if (output/'save_error.txt').is_file(): raise RuntimeError((output/'save_error.txt').read_text())
            if mapper.poll() is not None or time.monotonic()>until: raise RuntimeError('R3LIVE native map save timeout')
            time.sleep(.2)
        rospy.wait_for_service('/roomgs/finalize_dataset',timeout=5)
        exported=rospy.ServiceProxy('/roomgs/finalize_dataset',Trigger)()
        if not exported.success:raise RuntimeError('Dataset export failed: '+exported.message)
        report['export_status']=json.loads(exported.message)
        finish_recording(recorder,output/'sensors.bag'); recorder=None
        report.update(lio_messages=len(odoms),vio_messages=len(camera_odoms),input_status=statuses[-1] if statuses else {},
                      visual_updates=sum(s['esikf'] and s['photometric'] for s in vio_statuses),
                      max_visual_tracks=max((s['tracked'] for s in vio_statuses),default=0),
                      flight_finished=finished[0],wall_elapsed_seconds=time.monotonic()-start)
        if terminal[0] is not None:report['flight_result']=terminal[0]
        dump(output/'odometry.json',odoms)
        from integration.r3live.evaluate import evaluate
        report.update(evaluate(output))
        if args.scene=='warehouse20':
            offline_tool('warehouse20_audit.py',['--capture',str(output)])
            report['coverage']=json.loads((output/'coverage.json').read_text())
        else:
            from audit import audit
            report['coverage']=audit(output)
        report['source_unchanged']=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha
                                       for p,sha in report['runtime_source_sha256'].items())
        if args.resource_preflight:
            report['resource_preflight_passed']=bool(args.no_flight and report['colored_points']>100
                and report['visual_updates']>10 and report['max_visual_tracks']>50
                and report['input_status'].get('errors',1)==0 and report['source_unchanged']
                and report['export_status']['exported']>20 and not report['resource_usage']['reason'])
            dump(output/'result.json',report)
            if not report['resource_preflight_passed']:raise RuntimeError('Low-flow resource preflight failed')
            print('低数据流静态预检通过；不是完整地图或飞行验收：'+str(output),flush=True)
            return
        if args.auxiliary_capture:
            report['auxiliary_capture_passed']=bool(report.get('landed') and report.get('disarmed') and finished[0]
                and report['colored_points']>100 and report['visual_updates']>10 and report['source_unchanged']
                and report['input_status'].get('errors',1)==0 and not report.get('resource_abort'))
            dump(output/'result.json',report)
            if not report['auxiliary_capture_passed']:raise RuntimeError('Targeted auxiliary acquisition did not finish')
            print('定向补采完成；不是全地图验收，需要合并真实观测后再审计：'+str(output),flush=True)
            return
        report['success']=bool(report['colored_points']>100 and len(camera_odoms)>10 and len(odoms)>10
                               and (args.no_flight or finished[0]) and report['input_status'].get('errors',1)==0
                               and report['visual_updates']>10 and report['unique_colors']>10 and report['source_unchanged']
                               and report['coverage']['passed'])
        if not report['success']: raise RuntimeError('RGB/VIO/flight acceptance failed; inspect result.json')
        dump(output/'result.json',report)
        print('完整地图门槛通过：'+str(output),flush=True)
        print('保存地图 rgb_pt.pcd；报告 result.json；预览 colored_map.png',flush=True)
        if args.keep_open:
            print('保持本次仿真/RViz，Ctrl-C退出。',flush=True)
            while not rospy.is_shutdown(): time.sleep(.5)
    except (Exception, KeyboardInterrupt) as error:
        if isinstance(error,KeyboardInterrupt) and report['success']:
            return
        report.update(error=str(error) or 'Interrupted by user',wall_elapsed_seconds=time.monotonic()-start)
        dump(output/'result.json',report)
        raise
    finally:
        signal.signal(signal.SIGINT,cleanup_interrupt)
        runner.stop()
        if report['success']: dump(output/'result.json',report)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--headless',action='store_true')
    p.add_argument('--keep-open',action='store_true')
    p.add_argument('--no-flight',action='store_true',help='Stationary sensor/estimator check only')
    p.add_argument('--resource-preflight',action='store_true')
    p.add_argument('--auxiliary-capture',action='store_true')
    p.add_argument('--inspection-plan')
    p.add_argument('--stationary-seconds',type=float,default=12)
    p.add_argument('--side',type=float,default=2.)
    p.add_argument('--max-wall-seconds',type=float,default=6000)
    p.add_argument('--output-dir')
    p.add_argument('--scene',choices=['warehouse20','room'],default='warehouse20')
    a=p.parse_args()
    if not 0<a.side<=2.5: p.error('Room flight side must be in (0,2.5] metres')
    run(a)
