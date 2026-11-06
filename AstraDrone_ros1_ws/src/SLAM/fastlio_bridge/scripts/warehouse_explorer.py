#!/usr/bin/env python3
"""Autonomous frontier exploration on observed space; fixed-altitude simulator.

Never subscribes to Gazebo truth. The maintained map clears historical people;
CURRENT registered returns remain safety obstacles even when dynamic-masked
out of the persistent map. No semantic claim is made by this planner.
"""
import json
import math
import os
from pathlib import Path
import threading
import time
import sys
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import State as FlightState
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
# catkin devel relays execute into a separate context and cannot be imported as
# library modules. Prefer this actual source/install directory over relay dir.
sys.path.insert(0,str(Path(__file__).resolve().parent))
from exploration_grid import ExplorationGrid, expand
from exploration_native import SEARCH
from exploration_coverage import (SurfaceEvidence, ProgressWindow, ObservedMapAcceptance, suppress_disk,
                                  representative_cells, visible_unknown_gain)
from coverage_path_guidance import CoveragePathGuide
from flight_budget import FlightBudget
from ground_relative_altitude import GroundRelativeAltitude
from viewpoint_sweep import sweep_candidates, geometry_bound_sweep_gain, ScanDwell
from joint_view_policy import JointViewPolicy, reserved_home_review_seconds
from loop_coordinates import PlanningSnapshot, transform_message, transform_pose
from ghost_revisit import (suspect_groups, history_group_priorities,
                          cleared_history_groups, select_revisit_view,
                          occupied_voxel_keys, diagnose_alternative_height_views,
                          validate_height_offsets, pending_diagnostics)


def xyz(msg):
    """Zero-copy structured PointCloud2 view, including organized row padding."""
    fields = {f.name: f for f in msg.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in ('x','y','z')):
        raise ValueError('Expected FLOAT32 XYZ cloud')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                          offsets=[fields[k].offset for k in ('x','y','z')], itemsize=msg.point_step))
    view = np.ndarray((msg.height,msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step,msg.point_step))
    points = np.stack([view[k].ravel() for k in ('x','y','z')], axis=1)
    return points[np.isfinite(points).all(axis=1)]


class Explorer:
    def __init__(self):
        self.p = lambda k, default: rospy.get_param('~'+k, default)
        self.online_loop=bool(self.p('online_loop',False))
        self.world_frame='loop_map' if self.online_loop else 'map'
        self.loop_snapshot=None
        self.loop_active=None
        self.loop_applied_revision=None
        self.loop_home_anchor=None
        self.loop_local_altitude=None
        self.loop_transformed_cache={}
        self.loop_seed_revision=None
        budget_seconds=float(self.p('flight_budget_seconds',0.))
        unlimited_flight=bool(self.p('flight_budget_unlimited',False))
        if not math.isfinite(budget_seconds) or budget_seconds<0:
            raise ValueError('flight_budget_seconds must be finite and nonnegative')
        if unlimited_flight and budget_seconds!=0:
            raise ValueError('unlimited flight requires flight_budget_seconds=0')
        self.flight_budget=(FlightBudget(budget_seconds,0. if unlimited_flight else self.p('flight_reserve_seconds',60.),
            self.p('flight_return_speed_mps',.35),self.p('flight_return_time_factor',1.3),
            self.p('flight_landing_seconds',25.),self.p('home_settle_seconds',5.),
            self.p('flight_return_overhead_seconds',15.),
            reserved_home_review_seconds(self.p)) if budget_seconds or unlimited_flight else None)
        self.flight_budget_report=dict(enabled=bool(self.flight_budget))
        self.flight_home_distances=None
        self.joint_policy = JointViewPolicy(self.p) if self.p('joint_selection_enabled',False) else None
        if self.joint_policy is not None and (self.flight_budget is None or not self.p('ghost_revisit_enabled',False)):
            raise ValueError('Joint selection requires a flight budget and ghost evidence')
        self.joint_last_tick = None
        self.joint_observation = None
        self.joint_surface_observation = None
        self.joint_no_candidates_since = None
        self.joint_low_gain_since = None
        self.joint_surface_plan = None
        self.joint_surface_last_plan = -float('inf')
        self.joint_surface_grid = None
        self.joint_surface_baseline=None
        self.return_review_started=None
        self.return_review_last_decision=-float('inf')
        self.return_review_done=False
        self.return_review_report=dict(started=False,reason='not_started')
        # Fail explicitly on edits which would fake evidence / divide by zero.
        # Do not silently accept e.g. quality_target=0 or min_views=0.
        for key,default,lower,upper in (
                ('planning_hz',2.,.1,30.),('visualization_hz',.5,.1,30.),
                ('surface_voxel',.4,.15,1.),('surface_min_hits',3,2,100),
                ('surface_min_views',2,1,8),('surface_view_baseline',.75,.3,10.),
                ('surface_quality_target',.95,.5,1.),('convergence_seconds',90.,1.,3600.),
                ('convergence_free_area',1.,.01,1000.),('convergence_new_surfaces',25,1,100000),
                ('frontier_min_gain_area',.5,.01,100.),('inspection_budget_seconds',600.,1.,3600.),
                ('clearance_cost_weight',1.5,0,100),('recovery_command_horizon',.2,.03,.25),
                ('ghost_expected_speed_mps',.45,.1,2.),
                ('ghost_cleared_max_groups',12,1,64),
                ('ghost_no_evidence_extra_attempts',2,0,6),
                ('ghost_sensor_min_elevation_deg',-22.2,-90.,0.),
                ('ghost_sensor_max_elevation_deg',37.2,0.,90.),
                ('ghost_view_time_reserve_seconds',5.,0.,60.),
                ('ghost_endurance_min_exploration_seconds',240.,0.,600.),
                ('ghost_pending_min_age_seconds',20.,.5,120.),
                ('ghost_observation_seconds',3.,.5,20.),
                ('ghost_vertical_body_radius_m',.45,.45,1.2),
                ('ghost_vertical_body_half_height_m',.55,.55,1.2)):
            value=float(self.p(key,default))
            if not math.isfinite(value) or not lower<=value<=upper:
                raise ValueError('%s must be finite in [%s,%s]' % (key,lower,upper))
        if self.p('ghost_multilevel_flight_enabled',False):
            raise ValueError('Multilevel flight has no certified full 3-D route or controller integration; vertical waypoints are disabled')
        self.ghost_vertical_view_offsets = validate_height_offsets(
            self.p('ghost_vertical_view_offsets_m',[-.4,.4]))
        self.ghost_vertical_diagnostic_enabled = (
            self.p('ghost_vertical_view_diagnostic_enabled',False) is True)
        radius = float(self.p('initial_window_radius',12.0))
        if not math.isfinite(radius) or radius < 4.0:
            raise ValueError('initial_window_radius must be finite and >= 4 m')
        self.grid = ExplorationGrid([-radius,-radius,radius,radius],
                                    self.p('resolution',.2),self.p('clearance',.8),
                                    self.p('unknown_margin',.4),self.p('max_grid_cells',200000))
        self.planner_mode = self.p('planner_mode','frontier')
        if self.planner_mode not in ('frontier','coverage_guided'):
            raise ValueError('Unknown planner_mode: %s' % self.planner_mode)
        self.coverage_guide = (CoveragePathGuide(
            self.p('coverage_region_size',2.5),self.p('coverage_max_regions',8),
            self.p('coverage_lookahead_weight',.6),self.p('coverage_revisit_weight',.4))
            if self.planner_mode == 'coverage_guided' else None)
        self.guidance_report = {}
        self.lock = threading.Lock()
        self.inputs = {}
        self.takeoff_state = None
        self.paused = False
        self.target = None
        self.blacklist = []
        self.visited = []
        self.inspection_visited = []
        self.home = self.altitude = None
        self.ground_navigation = GroundRelativeAltitude()
        self.home_xy = None
        self.phase = 'EXPLORING'
        self.finish_reason = ''
        self.mission_started = self.return_started = self.home_since = None
        self.home_confirmed = False
        self.return_requested = False
        self.inspected = np.zeros(self.grid.shape,dtype=bool)
        self.failed_visits = {}
        self.target_kind = 'frontier'
        self.ready_since = self.no_frontier_since = None
        self.settle_until = 0.
        self.last_progress = 0.
        self.progress_position = None
        self.unsafe_since = None
        self.unsafe_position = None
        self.last_save = time.monotonic()
        self.last_free = None
        self.cache = {}
        self.goals_selected = self.goals_reached = self.failures = 0
        self.start_wall = time.monotonic()
        self.travel = 0.
        self.previous = None
        self.surface = SurfaceEvidence(self.p('surface_voxel',.4),
            self.p('surface_min_hits',3),self.p('surface_min_views',2),
            self.p('surface_view_baseline',.75),self.p('surface_max_voxels',150000))
        self.progress = ProgressWindow(self.p('convergence_seconds',90.),
            self.p('convergence_free_area',1.),self.p('convergence_new_surfaces',25))
        self.acceptance = (ObservedMapAcceptance(self.p('surface_quality_target',.95),
                           self.p('acceptance_window_seconds',60.),self.p('acceptance_hold_seconds',10.),
                           self.p('acceptance_min_mission_seconds',180.))
                           if self.p('observed_map_acceptance',False) else None)
        self.surface_report = {}
        self.progress_report = {}
        self.exploration_complete = False
        self.grid_capacity_limited = False
        self.raw_frontier_count = self.useful_frontier_count = 0
        self.remaining_inspection_count = 0
        self.last_visualization = -float('inf')
        self.last_written_state = None
        self.tick_times = []
        self.last_compute_ms = 0.
        self.active_wall = self.active_sim = None
        self.inspection_started = None
        self.inspection_last_plan = -float('inf')
        self.inspection_plan = None
        self.ghost_status = {}
        self.ghost_status_wall = None
        self.ghost_history = {}
        self.ghost_history_wall = None
        self.ghost_revisit_started = None
        self.ghost_revisit_reason = ''
        self.ghost_revisit_visits = {}
        self.ghost_cleared_visits = {}
        self.ghost_cleared_view_reached = set()
        self.ghost_cleared_candidate_groups = 0
        self.ghost_revisit_target = None
        self.ghost_revisit_target_kind = None
        self.ghost_no_view_since = None
        self.ghost_cleared_fallback = False
        self.ghost_active_view = None
        self.ghost_view_evidence = []
        self.ghost_view_selection = {}
        self.ghost_budget_trigger = {}
        self.ghost_vertical_view_diagnostic = dict(
            enabled=self.ghost_vertical_diagnostic_enabled,
            flight_authorized=False,scope='read_only_voxel_proxy',views=[])
        self.ghost_vertical_diagnostic_stamp = -float('inf')
        self.report_file = Path(self.p('report','/tmp/warehouse_exploration.json'))
        if self.flight_budget:
            rospy.Subscriber('/mavros/state',FlightState,self.receive_flight_state,queue_size=5)
        for name, topic, cls in (
            ('odom','/uav1/fastlio/odom',Odometry),
            ('mavros','/mavros/local_position/pose',PoseStamped),
            ('map','/uav1/fastlio/cloud_map',PointCloud2),
            ('scan','/uav1/fastlio/registered_scan',PointCloud2),
            ('free','/uav1/local_free_space',PointCloud2)):
            if self.online_loop and name=='map':continue
            rospy.Subscriber(topic,cls,self.receive,name,queue_size=1,buff_size=16000000)
        if self.online_loop:
            from fastlio_bridge.msg import LoopMapSnapshot
            rospy.Subscriber('/uav1/loop_slam/snapshot',LoopMapSnapshot,
                self.receive_loop_snapshot,queue_size=1,buff_size=32000000)
        else:
            rospy.Subscriber('/uav1/map/ghost_suspects',PointCloud2,
                             self.receive,'suspects',queue_size=1,buff_size=16000000)
            rospy.Subscriber('/uav1/map/ghost_revisit_status',String,
                             self.receive_ghost_status,queue_size=1)
            rospy.Subscriber('/uav1/map/ghost_history',String,
                             self.receive_ghost_history,queue_size=1)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
        rospy.Subscriber('/mavros_avoidance_controller/takeoff_complete',Bool,
                         self.receive_takeoff,queue_size=1)
        self.waypoint_pub = rospy.Publisher('/uav1/planner/waypoint',PoseStamped,queue_size=1)
        # Dedicated, non-latched authorization. The flight controller accepts
        # level-1/2 motion only while this short escape goal is fresh and also
        # matches the ordinary waypoint; a stale escape cannot move the UAV.
        self.recovery_pub = rospy.Publisher('/uav1/exploration/recovery_waypoint',PoseStamped,queue_size=1)
        self.path_pub = rospy.Publisher('/uav1/planner/local_path',RosPath,queue_size=1,latch=True)
        self.grid_pub = rospy.Publisher('/uav1/exploration/grid',OccupancyGrid,queue_size=1,latch=True)
        self.status_pub = rospy.Publisher('/uav1/exploration/status',String,queue_size=1,latch=True)
        self.markers_pub = rospy.Publisher('/uav1/exploration/markers',MarkerArray,queue_size=1,latch=True)
        rospy.Service('/uav1/exploration/set_enabled',SetBool,self.enable)
        rospy.Service('/uav1/exploration/return_home',Trigger,self.request_return)

    def request_return(self, req):
        # Service callbacks only request a transition; tick owns mission state.
        self.return_requested = True
        return TriggerResponse(True,'Return requested; observed-space planning and safety guards remain active')

    def receive_loop_snapshot(self,message):
        with self.lock:self.loop_snapshot=(message,time.monotonic())

    def reset_loop_planning(self):
        """A nonrigid graph update invalidates voxel indices and old viewpoints.

        Rebuild observed free space from the SAME snapshot's historical ray
        evidence. Preserve mission/flight clocks, return request, home anchor
        and the current phase; never restart the battery budget.
        """
        radius=float(self.p('initial_window_radius',12.))
        self.grid=ExplorationGrid([-radius,-radius,radius,radius],self.p('resolution',.2),
            self.p('clearance',.8),self.p('unknown_margin',.4),self.p('max_grid_cells',200000))
        self.region_observation=None;self.region_frontier_pending=None
        self.joint_indoor_grid=None;self.joint_ceiling_plane=None
        self.target=None;self.blacklist=[];self.visited=[];self.inspection_visited=[]
        if getattr(self,'ground_navigation',None) is not None:
            self.ground_navigation.reset_coordinates()
        self.inspected=np.zeros(self.grid.shape,dtype=bool);self.failed_visits={}
        self.inspection_plan=None;self.inspection_last_plan=-float('inf')
        self.last_free=None;self.cache={};self.previous=None;self.flight_home_distances=None
        self.progress_position=None;self.unsafe_since=self.unsafe_position=None
        self.home_since=None;self.settle_until=0.;self.no_frontier_since=None
        self.ghost_revisit_target=None;self.ghost_revisit_target_kind=None
        self.ghost_revisit_visits={};self.ghost_cleared_visits={};self.ghost_cleared_view_reached=set()
        self.ghost_view_selection={};self.ghost_budget_trigger={}
        self.ghost_active_view=None;self.ghost_no_view_since=None
        if getattr(self,'joint_policy',None) is not None:
            self.joint_policy.invalidate_coordinates()
            self.joint_observation=None
            self.joint_surface_observation=None
            self.joint_surface_baseline=None
            self.joint_no_candidates_since=None
            self.joint_low_gain_since=None
        self.surface=SurfaceEvidence(self.p('surface_voxel',.4),self.p('surface_min_hits',3),
            self.p('surface_min_views',2),self.p('surface_view_baseline',.75),self.p('surface_max_voxels',150000))
        self.surface_report={};self.exploration_complete=False
        if self.coverage_guide is not None:
            self.coverage_guide=CoveragePathGuide(self.p('coverage_region_size',2.5),self.p('coverage_max_regions',8),
                self.p('coverage_lookahead_weight',.6),self.p('coverage_revisit_weight',.4))
        self.reset_progress()

    def prepare_loop_inputs(self,inputs,pair,wall,now):
        if pair is None or wall-pair[1]>self.p('input_timeout_wall',3.):
            raise ValueError('global snapshot absent/stale')
        message,received=pair
        if not -.1<=now-message.header.stamp.to_sec()<=self.p('input_timeout_sim',1.5):
            raise ValueError('global map acquisition is stale')
        if 'odom' not in inputs or 'scan' not in inputs:raise ValueError('missing local scan/odometry')
        if self.loop_active is None or self.loop_active.message is not message:
            parsed=PlanningSnapshot(message)
            if self.loop_active is not None and (
                    message.revision<self.loop_active.message.revision or
                    message.processed_frames<self.loop_active.message.processed_frames):
                raise ValueError('global map revision/frame counter moved backwards')
            if self.loop_applied_revision is not None and message.revision!=self.loop_applied_revision:
                self.reset_loop_planning()
                self.loop_seed_revision=message.revision
            self.loop_applied_revision=message.revision;self.loop_active=parsed
        parsed=self.loop_active
        if self.loop_home_anchor is None:
            p=inputs['odom'][0].pose.pose.position
            self.loop_home_anchor=(np.array([p.x,p.y,p.z]),inputs['odom'][0].header.stamp.to_sec())
        anchor,stamp=self.loop_home_anchor;c=parsed.at(stamp)
        home=c[:3,:3]@anchor+c[:3,3];self.home_xy=tuple(map(float,home[:2]))
        for name in ('odom','scan','free'):
            if name not in inputs:continue
            source,receipt=inputs[name]
            cached=self.loop_transformed_cache.get(name)
            if cached is None or cached[0] is not source or cached[1]!=message.revision:
                cached=(source,message.revision,transform_message(source,parsed.transform,
                    'scan' if name=='free' else name))
                self.loop_transformed_cache[name]=cached
            inputs[name]=(cached[2],receipt)
        inputs['map']=(message.map,received)
        # Keep the old current-ray feed/rate during ordinary exploration.
        # On graph changes only, seed the reset grid from rebuilt historical
        # rays, so old coordinates are never reused as if globally corrected.
        if self.loop_seed_revision is not None or 'free' not in inputs:inputs['free']=(message.free,received)
        inputs['suspects']=(message.suspects,received)
        self.ghost_status=parsed.status;self.ghost_status_wall=received
        self.ghost_history=parsed.history;self.ghost_history_wall=received
        if self.loop_local_altitude is not None:
            self.altitude=float((parsed.transform@np.array([0.,0.,self.loop_local_altitude,1.]))[2])
        if message.capacity_limited and self.phase not in ('RETURNING','COMPLETE','RETURN_FAILED'):
            self.grid_capacity_limited=True
            self.begin_return('loop_map_capacity_partial',now)
        return_at=int(self.p('loop_map_return_at_frames',0))
        if (return_at>0 and message.input_frames>=return_at and
                self.phase not in ('RETURNING','COMPLETE','RETURN_FAILED')):
            self.grid_capacity_limited=True
            self.begin_return('loop_map_capacity_approaching_partial',now)
        return inputs

    def receive_flight_state(self,msg):
        stamp=msg.header.stamp.to_sec()
        now=rospy.Time.now().to_sec()
        if -0.1<=now-stamp<=2.5:
            with self.lock:self.flight_budget.observe_arm(bool(msg.armed),stamp)

    def reset_progress(self):
        self.progress.reset()
        if getattr(self,'acceptance',None) is not None:
            self.acceptance.reset()

    def begin_return(self, reason, now):
        if self.region_feedback_enabled():
            try:
                path=self.report_file.with_suffix('.regions.json')
                active=getattr(self,'region_observation',None)
                data=self.joint_policy.region_memory.export(now,active['view'].summary() if active else None)
                path.write_text(json.dumps(data,allow_nan=False,indent=2)+'\n',encoding='utf8')
                self.joint_policy.report['region_state_file']=str(path)
            except (OSError,ValueError,TypeError,AttributeError) as error:
                self.joint_policy.report['region_export_error']=str(error)
        self.region_observation=None;self.region_frontier_pending=None
        self.phase = 'RETURNING'
        self.finish_reason = reason
        self.return_started = now
        self.home_since = None
        self.target = None
        self.settle_until = 0.
        if getattr(self,'joint_policy',None) is not None:
            self.joint_policy.selected=None
            self.joint_policy.corridor.reset()
            self.joint_policy.report['corridor_progress']=dict(active=False,reason='return_phase')
            self.joint_observation=None;self.joint_surface_observation=None

    def receive_ghost_status(self, msg):
        try:
            report = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.ghost_status = report
            self.ghost_status_wall = time.monotonic()

    def ghost_status_fresh(self):
        return (self.ghost_status_wall is not None and
                time.monotonic()-self.ghost_status_wall <= self.p('input_timeout_wall',3.))

    def endurance_revisit_due(self,now):
        """Reserve a real revisit phase with the arm clock, even if limit=0."""
        if (self.flight_budget is None or not self.p('ghost_endurance_revisit',False) or
                not self.p('ghost_revisit_enabled',False) or self.phase=='REVISITING' or
                not self.ghost_status_fresh() or not self.ghost_history_fresh(now)):
            return False
        elapsed=self.flight_budget.elapsed(now)
        minimum=self.p('ghost_endurance_min_exploration_seconds',240.)
        age=self.p('ghost_pending_min_age_seconds',20.)
        persistent=sum(r.get('state')=='suspected' and r.get('actionable') is True and
            now-float(r.get('last_dynamic',now))>=age
            for r in self.ghost_history.get('records',[]))
        self.ghost_budget_trigger=dict(persistent_pending=persistent,
            minimum_exploration_seconds=minimum,elapsed_seconds=elapsed,
            reason='persistent_suspects_reserved_phase' if persistent else 'no_aged_suspects')
        return bool(elapsed is not None and elapsed>=minimum and persistent)

    def receive_ghost_history(self, msg):
        try:
            history = json.loads(msg.data)
            if (history.get('version') != 1 or history.get('frame_id') != getattr(self,'world_frame','map') or
                    not isinstance(history.get('records'), list)):
                return
        except (ValueError, TypeError, AttributeError):
            return
        with self.lock:
            self.ghost_history = history
            self.ghost_history_wall = time.monotonic()

    def ghost_history_fresh(self, now):
        history = getattr(self,'ghost_history',{})
        received = getattr(self,'ghost_history_wall',None)
        stamp = history.get('map_stamp')
        return (received is not None and
                time.monotonic()-received <= self.p('input_timeout_wall',3.) and
                isinstance(stamp,(int,float)) and math.isfinite(stamp) and
                -0.1 <= now-stamp <= self.p('input_timeout_sim',1.5))

    def begin_revisit_or_return(self, reason, now, inputs):
        if not self.p('ghost_revisit_enabled',False):
            self.begin_return(reason,now)
            return False
        if (not self.ghost_status_fresh() or not self.ghost_status.get('enabled',False) or
                'suspects' not in inputs or
                now-inputs['suspects'][0].header.stamp.to_sec()>self.p('input_timeout_sim',1.5)):
            self.exploration_complete = False
            self.begin_return('ghost_status_unavailable_partial',now)
            return False
        suspects = self.cloud('suspects',inputs)
        if not len(suspects) and not self.ghost_status.get('pending_voxels',0):
            if self.ghost_status.get('history_unresolved',0):
                self.exploration_complete = False
                self.begin_return('ghost_unresolved_partial',now)
                return False
            if not self.ghost_history_fresh(now):
                self.exploration_complete = False
                self.begin_return('ghost_history_unavailable_partial',now)
                return False
            cleared, _ = cleared_history_groups(
                self.ghost_history,self.altitude,
                self.p('ghost_height_below',1.25),self.p('ghost_height_above',.85),
                limit=self.p('ghost_cleared_max_groups',12))
            self.ghost_cleared_candidate_groups = len(cleared)
            if not cleared:
                self.begin_return(reason,now)
                return False
        self.phase = 'REVISITING'
        self.exploration_complete = False
        self.ghost_revisit_started = now
        self.ghost_revisit_reason = reason
        self.ghost_no_view_since = None
        self.ghost_cleared_fallback = False
        self.ghost_active_view = None
        self.ghost_view_evidence = []
        self.ghost_vertical_view_diagnostic = dict(
            enabled=self.ghost_vertical_diagnostic_enabled,
            flight_authorized=False,scope='read_only_voxel_proxy',views=[])
        self.ghost_vertical_diagnostic_stamp = -float('inf')
        self.target = None
        self.ghost_revisit_target_kind = None
        self.settle_until = 0.
        return True

    def ghost_group_counts(self, key, active_only=False):
        """Episode-local evidence for actionable voxels in one 0.8 m group."""
        counts = {}
        for record in self.ghost_history.get('records',[]):
            point = record.get('position',())
            voxel = record.get('key',())
            if len(point) != 3 or len(voxel) != 3 or not all(
                    math.isfinite(float(v)) for v in point):
                continue
            if tuple(np.floor(np.asarray(point[:2])/.8).astype(int)) != key:
                continue
            if active_only and (record.get('state') != 'suspected' or
                                not record.get('actionable')):
                continue
            if ('free_evidence_frames' not in record or
                    'post_departure_hit_frames' not in record):
                continue
            counts[tuple(voxel)] = (int(record['free_evidence_frames']),
                                    int(record.get('protected_endpoint_frames',0)),
                                    int(record['post_departure_hit_frames']),
                                    record.get('state'),int(record.get('positive_endpoint_frames',0)))
        return counts

    def finish_ghost_view(self, now):
        """Judge a flown view using actual mapper evidence, never arrival alone."""
        probe = self.ghost_active_view
        if probe is None or 'arrived' not in probe or now < self.settle_until:
            return
        result = dict(group=str(probe['group']),view_xy=list(probe['view']),
                      selected_stamp=probe['selected'],arrived_stamp=probe['arrived'],
                      checked_stamp=now,status='history_unavailable',
                      free_evidence_delta=0,protected_endpoint_delta=0,
                          post_departure_hit_delta=0,
                          positive_endpoint_delta=0,
                      resolved_voxels=0)
        if probe['before'] and self.ghost_history_fresh(now):
            after = self.ghost_group_counts(probe['group'])
            free = protected = hits = resolved = positive = 0
            for voxel, before_counts in probe['before'].items():
                before_free,before_protected,before_hits,_=before_counts[:4]
                current = after.get(voxel)
                if current is None:
                    continue
                free += max(0,current[0]-before_free)
                protected += max(0,current[1]-before_protected)
                hits += max(0,current[2]-before_hits)
                positive += max(0,(current[4] if len(current)>4 else 0)-
                    (before_counts[4] if len(before_counts)>4 else 0))
                resolved += current[3] != 'suspected'
            result.update(status='measured',free_evidence_delta=free,
                          protected_endpoint_delta=protected,
                          post_departure_hit_delta=hits,resolved_voxels=resolved)
            result['positive_endpoint_delta']=positive
        self.ghost_view_evidence.append(result)
        self.ghost_active_view = None

    def ghost_revisit_tick(self, now, position, start, distances, parent, inputs):
        if (not self.ghost_status_fresh() or not self.ghost_status.get('enabled',False) or
                'suspects' not in inputs or
                now-inputs['suspects'][0].header.stamp.to_sec()>self.p('input_timeout_sim',1.5)):
            self.begin_return('ghost_status_unavailable_partial',now)
            self.return_tick(now,position,start,distances,parent)
            return
        self.finish_ghost_view(now)
        if now-self.ghost_revisit_started >= self.p('ghost_revisit_budget_seconds',120.):
            cleared_only = (self.ghost_cleared_candidate_groups > 0 and
                            not self.ghost_status.get('pending_voxels',0))
            self.begin_return('ghost_cleared_review_budget_partial' if cleared_only else
                              'ghost_revisit_budget_partial',now)
            self.return_tick(now,position,start,distances,parent)
            return
        priorities = (history_group_priorities(self.ghost_history,self.altitude,
                      self.p('ghost_height_below',1.25),self.p('ghost_height_above',.85))
                      if self.ghost_history_fresh(now) else None)
        groups = ([] if self.ghost_cleared_fallback else
                  suspect_groups(self.cloud('suspects',inputs),self.altitude,
                    self.p('ghost_height_below',1.25),self.p('ghost_height_above',.85),
                    limit=self.p('ghost_max_groups',12),priorities=priorities))
        kind = 'suspect'
        visits = self.ghost_revisit_visits
        base_attempts = int(self.p('ghost_max_attempts',2))
        extra_attempts = int(self.p('ghost_no_evidence_extra_attempts',2))
        attempts = {}
        for key,_,_ in groups:
            measured = [item for item in self.ghost_view_evidence
                        if item['group']==str(key) and item['status']=='measured']
            # Only a verified sequence of uninformative views earns extra
            # attempts. Missing mapper diagnostics retain the old bound.
            no_evidence = (len(measured) >= base_attempts and
                all(item['free_evidence_delta']==0 and
                    item['post_departure_hit_delta']==0 and
                    item.get('positive_endpoint_delta',0)==0 and
                    item['resolved_voxels']==0
                    for item in measured[:base_attempts]))
            attempts[key] = base_attempts + (extra_attempts if no_evidence else 0)
        if not groups:
            pending = (self.ghost_status.get('pending_voxels',0) or
                       self.ghost_status.get('history_unresolved',0))
            if pending and not self.ghost_cleared_fallback:
                self.ghost_no_view_since = self.ghost_no_view_since or now
                if now-self.ghost_no_view_since < self.p('ghost_no_view_seconds',20.):
                    self.publish('GHOST_WAIT_OBSERVABLE',position)
                    return
                # Current suspects have no reachable candidate. Spend only
                # remaining budget on low-priority historical locations;
                # unresolved suspects remain explicit in the final result.
                self.ghost_cleared_fallback = True
                self.ghost_no_view_since = None
            if not self.ghost_history_fresh(now):
                self.begin_return('ghost_unresolved_partial' if pending else
                                  'ghost_history_unavailable_partial',now)
                self.return_tick(now,position,start,distances,parent)
                return
            groups, priorities = cleared_history_groups(
                self.ghost_history,self.altitude,
                self.p('ghost_height_below',1.25),self.p('ghost_height_above',.85),
                limit=self.p('ghost_cleared_max_groups',12))
            self.ghost_cleared_candidate_groups = len(groups)
            kind = 'cleared'
            visits = self.ghost_cleared_visits
            attempts = 1
            if not groups:
                unresolved = bool(pending)
                self.exploration_complete = (not unresolved and
                    self.ghost_revisit_reason=='observed_reachable_converged')
                self.begin_return('ghost_unresolved_partial' if unresolved else
                                  self.ghost_revisit_reason,now)
                self.return_tick(now,position,start,distances,parent)
                return
        active_keys = {key for key,_,_ in groups}
        if self.target is not None and (self.ghost_revisit_target not in active_keys or
                                        self.ghost_revisit_target_kind != kind):
            self.target = self.ghost_revisit_target = self.ghost_revisit_target_kind = None
            self.ghost_active_view = None
        if self.target is not None:
            if math.dist(position,self.grid.xy(self.target)) <= self.p('arrival_tolerance',.3):
                self.goals_reached += 1
                visits.setdefault(self.ghost_revisit_target,[]).append(self.grid.xy(self.target))
                if kind == 'suspect' and self.ghost_active_view is not None:
                    self.ghost_active_view['arrived'] = now
                if kind == 'cleared':
                    self.ghost_cleared_view_reached.add(self.ghost_revisit_target)
                self.target = self.ghost_revisit_target = self.ghost_revisit_target_kind = None
                self.settle_until = now+max(self.p('settle_seconds',3.),
                    self.p('ghost_observation_seconds',self.p('settle_seconds',3.)) if kind=='suspect' else 0.)
                self.publish('GHOST_CLEARED_REVIEW_OBSERVING' if kind == 'cleared' else
                             'GHOST_OBSERVING',position)
                return
            if math.dist(position,self.progress_position) >= .25:
                self.progress_position,self.last_progress = position,now
            if not np.isfinite(distances[self.target]) or now-self.last_progress > self.p('progress_timeout',25.):
                visits.setdefault(self.ghost_revisit_target,[]).append(self.grid.xy(self.target))
                self.target = self.ghost_revisit_target = self.ghost_revisit_target_kind = None
                self.ghost_active_view = None
                self.failures += 1
                self.publish('GHOST_REPLAN_BLOCKED',position)
                return
        if self.target is None:
            home_distances = None
            if self.home_xy is not None and self.grid.inside(self.grid.cell(self.home_xy)):
                home_distances,_ = self.grid.search(self.grid.cell(self.home_xy),
                    clearance_weight=self.p('clearance_cost_weight',1.5))
            target_points = occupied_voxels = None
            sensor_altitude = None
            voxel_size = .15
            if kind == 'suspect':
                suspect_points = self.cloud('suspects',inputs)
                target_points = {}
                for point in suspect_points:
                    key = tuple(np.floor(point[:2]/.8).astype(int))
                    target_points.setdefault(key,[]).append(point)
                voxel_size = float(self.ghost_history.get('voxel_size',.15))
                occupied_voxels = occupied_voxel_keys(self.cloud('map',inputs),voxel_size)
                # Evaluate the commanded fixed flight layer, not transient
                # odometry Z at the selection instant.
                sensor_altitude = self.altitude+float(self.p('lidar_origin_in_body',[0.,0.,0.])[2])
            self.target,self.ghost_revisit_target = select_revisit_view(
                self.grid,distances,groups,position,visits,
                self.p('ghost_view_min_range',1.2),self.p('ghost_view_max_range',3.),
                self.p('ghost_view_change',.7),attempts,
                priorities=priorities,
                speed_mps=min(self.p('ghost_expected_speed_mps',.45),
                    self.flight_budget.speed/self.flight_budget.factor) if self.flight_budget is not None
                    else self.p('ghost_expected_speed_mps',.45),
                dwell_seconds=max(self.p('settle_seconds',3.),
                    self.p('ghost_observation_seconds',self.p('settle_seconds',3.)) if kind=='suspect' else 0.),
                phase_remaining_seconds=self.p('ghost_revisit_budget_seconds',120.)-
                    (now-self.ghost_revisit_started),
                remaining_seconds=(self.flight_budget.total-self.flight_budget.reserve-self.flight_budget.elapsed(now)-
                    self.flight_budget.fixed_return_seconds if self.flight_budget is not None and
                    not self.flight_budget.unlimited else None),
                return_speed_mps=self.flight_budget.speed/self.flight_budget.factor
                    if self.flight_budget is not None else self.p('ghost_expected_speed_mps',.45),
                return_distances=home_distances,
                reserve_seconds=self.p('ghost_view_time_reserve_seconds',5.),
                target_points=target_points,occupied_voxels=occupied_voxels,
                sensor_altitude=sensor_altitude,voxel_size=voxel_size,
                min_elevation_deg=self.p('ghost_sensor_min_elevation_deg',-22.2),
                max_elevation_deg=self.p('ghost_sensor_max_elevation_deg',37.2),
                diagnostics=self.ghost_view_selection)
            if self.target is None:
                if (kind == 'suspect' and self.ghost_vertical_diagnostic_enabled and
                        now-self.ghost_vertical_diagnostic_stamp >= 5.):
                    # This is a read-only counterfactual. The 2-D grid only
                    # establishes an XY route at the captured flight height.
                    # Even a clear vertical voxel sweep cannot authorize a
                    # multilevel trajectory or command a different Z.
                    free_keys = occupied_voxel_keys(self.cloud('free',inputs),voxel_size)
                    occupied_keys = occupied_voxel_keys(
                        np.concatenate((self.cloud('map',inputs),
                                        self.cloud('scan',inputs))),voxel_size)
                    views = diagnose_alternative_height_views(
                        self.grid,distances,groups[:3],position,target_points,
                        occupied_keys,free_keys,self.altitude,
                        self.ghost_vertical_view_offsets,
                        inputs['free'][0].header.stamp.to_sec(),
                        min(inputs['map'][0].header.stamp.to_sec(),
                            inputs['scan'][0].header.stamp.to_sec()),
                        now,self.p('input_timeout_sim',1.5),voxel_size,
                        self.p('ghost_vertical_body_radius_m',.45),
                        self.p('ghost_vertical_body_half_height_m',.55),
                        self.p('ghost_view_min_range',1.2),
                        self.p('ghost_view_max_range',3.),
                        self.p('ghost_sensor_min_elevation_deg',-22.2),
                        self.p('ghost_sensor_max_elevation_deg',37.2),priorities)
                    self.ghost_vertical_view_diagnostic = dict(
                        enabled=True,flight_authorized=False,
                        scope='read_only_voxel_proxy',checked_stamp=now,
                        considered_groups=min(3,len(groups)),
                        total_groups=len(groups),views=views[:6])
                    self.ghost_vertical_diagnostic_stamp = now
                if kind == 'cleared':
                    # Optional historical review has no safe fresh view (or
                    # all selected groups were visited). Never call it a map
                    # failure or claim that a viewpoint certified free space.
                    unresolved = bool(self.ghost_status.get('pending_voxels',0) or
                                      self.ghost_status.get('history_unresolved',0))
                    self.exploration_complete = (not unresolved and
                        self.ghost_revisit_reason=='observed_reachable_converged')
                    self.begin_return('ghost_unresolved_partial' if unresolved else
                                      self.ghost_revisit_reason,now)
                    self.return_tick(now,position,start,distances,parent)
                    return
                self.ghost_no_view_since = self.ghost_no_view_since or now
                if now-self.ghost_no_view_since >= self.p('ghost_no_view_seconds',20.):
                    self.ghost_cleared_fallback = True
                    self.ghost_no_view_since = None
                    self.publish('GHOST_SWITCH_TO_CLEARED_REVIEW',position)
                else:
                    self.publish('GHOST_WAIT_SAFE_VIEW',position)
                return
            self.ghost_no_view_since = None
            self.ghost_revisit_target_kind = kind
            if kind == 'suspect':
                self.ghost_active_view = dict(group=self.ghost_revisit_target,
                    view=self.grid.xy(self.target),selected=now,
                    before=self.ghost_group_counts(self.ghost_revisit_target,
                        active_only=True) if self.ghost_history_fresh(now) else {})
            self.goals_selected += 1
            self.last_progress,self.progress_position = now,position
            self.goal_pub.publish(self.pose(self.grid.xy(self.target)))
        self.publish('REVIEWING_CLEARED_HISTORY' if kind == 'cleared' else
                     'REVISITING',position,self.grid.path(parent,start,self.target))

    def mark_inspected(self, position):
        # Close-range *viewpoint* coverage, not a claim of 3D surface coverage.
        # Limit marking to line-of-sight in the occupied grid: walls must not
        # make the opposite aisle appear inspected simply by Euclidean distance.
        start = self.grid.cell(position)
        r = int(math.ceil(self.p('inspection_radius',2.0)/self.grid.resolution))
        for y in range(max(0,start[0]-r),min(self.grid.shape[0],start[0]+r+1)):
            for x in range(max(0,start[1]-r),min(self.grid.shape[1],start[1]+r+1)):
                if (y-start[0])**2+(x-start[1])**2<=r*r and self.grid.safe[y,x] and not self.inspected[y,x]:
                    if self.grid.segment_safe(position,self.grid.xy((y,x))):
                        self.inspected[y,x]=True

    def return_tick(self, now, position, start, distances, parent, inputs=None):
        home = self.grid.cell(self.home_xy)
        if inputs is not None and self.return_review_tick(now,position,start,distances,parent,inputs):return
        error = math.dist(position,self.home_xy)
        if error <= self.p('home_tolerance',.30):
            self.home_since = now if self.home_since is None else self.home_since
            if now-self.home_since >= self.p('home_settle_seconds',5.):
                self.home_confirmed = True
                # Save confirmation is part of completion. Failed save is
                # retried while holding at home, never reported as success.
                if self.save():
                    self.phase = 'COMPLETE'
                    self.publish('COMPLETE',position)
                else:
                    self.publish('HOME_SAVE_FAILED',position)
            else:
                self.publish('HOME_SETTLE',position)
            return
        self.home_since = None
        if now-self.return_started > self.p('return_timeout',1200.):
            self.phase = 'RETURN_FAILED'
            self.publish('RETURN_FAILED',position)
            return
        if not self.grid.inside(home) or not np.isfinite(distances[home]):
            self.publish('RETURN_BLOCKED',position)
            return
        self.target = home
        path = self.grid.path(parent,start,home)
        self.publish('RETURNING',position,path)

    def return_review_tick(self, now, position, start, distances, parent, inputs):
        """Bounded on-route checks before landing, using unchanged real safety."""
        policy=getattr(self,'joint_policy',None)
        if policy is None or self.return_review_done:return False
        if policy.p['joint_home_ghost_reward_scale']==0.:
            self.return_review_done=True;policy.selected=None;self.joint_observation=None
            self.return_review_report.update(started=False,reason='home_ghost_score_zero',end_sim=now)
            self.joint_event('return_review_disabled',now,reason='home_ghost_score_zero')
            return False
        if not policy.p['joint_defer_home_review']:return False
        home=self.grid.cell(self.home_xy)
        if not self.grid.inside(home) or not np.isfinite(distances[home]):return False
        # Do not leave the safe homeward path to service a distant review.
        if distances[home]>policy.p['joint_home_review_radius_m']:return False
        used=0. if self.return_review_started is None else now-self.return_review_started
        exhausted=used>=policy.p['joint_return_review_seconds']
        fresh=self.ghost_status_fresh() and self.ghost_history_fresh(now)
        if exhausted or not fresh:
            self.return_review_done=True;policy.selected=None;self.joint_observation=None
            self.return_review_report.update(reason='time_limit' if exhausted else 'stale_evidence',end_sim=now)
            return False
        selected=policy.selected
        if selected is not None:
            cell=self.grid.cell(selected.xy)
            valid=(self.grid.inside(cell) and self.grid.safe[cell] and np.isfinite(distances[cell]) and
                bool(self.flight_budget.feasible(now,distances[cell],self.flight_home_distances[cell],
                    selected.dwell_s,spend_review=True)))
            if not valid:
                policy.selected=None;self.joint_observation=None
        if self.joint_finish_observation(now,inputs,position):return True
        selected=policy.selected
        if selected is None:
            if now-self.return_review_last_decision<1.:return False
            self.return_review_last_decision=now
            selected=policy.select(self.grid,position,distances,self.flight_home_distances,
                np.empty((0,2),int),np.empty(0),self.ghost_history.get('records',[]),
                np.concatenate((self.cloud('map',inputs),self.cloud('scan',inputs))),
                self.slice(self.cloud('scan',inputs)),self.altitude,now,self.flight_budget,
                pending_age=self.p('ghost_pending_min_age_seconds',20.),
                max_attempts=self.p('ghost_max_attempts',2),dwell=self.p('ghost_observation_seconds',4.),
                view_altitude=self.navigation_z,home_xy=self.home_xy,return_phase=True,
                return_review_remaining_s=max(0.,policy.p['joint_return_review_seconds']-used))
            self.joint_event('return_review_decision',now,**policy.report)
            if selected is None:
                # No fabricated free evidence or resolution on skip/timeout.
                at_home=math.dist(position,self.home_xy)<=self.p('home_tolerance',.30)
                self.return_review_done=at_home
                self.return_review_report.update(reason=('no_safe_affordable_review' if at_home
                    else 'retry_while_approaching_home'),last_attempt_sim=now)
                return False
            if self.return_review_started is None:
                self.return_review_started=now
                self.return_review_report.update(started=True,start_sim=now,reason='reviewing_on_return')
            self.goals_selected+=1
            self.return_review_report.update(selected=selected.summary())
        if math.dist(position,selected.xy)<=self.p('arrival_tolerance',.3):
            self.joint_observation=dict(view=selected,arrived=now,last_stamp=-float('inf'),scans=0,
                before=self.ghost_group_counts(selected.group),selected=policy.last_selection_stamp,quality=selected.quality)
            self.joint_event('return_observation_started',now,selected=selected.summary())
            self.publish('RETURN_VERIFY_OBSERVING',position);return True
        self.target=self.grid.cell(selected.xy)
        self.publish('RETURN_VERIFYING',position,self.grid.path(parent,start,self.target))
        return True

    def receive(self, msg, name):
        if name != 'mavros' and msg.header.frame_id.lstrip('/') != 'map':
            rospy.logerr_throttle(5,'Exploration rejects non-map frame on %s',name)
            return
        with self.lock:
            self.inputs[name] = (msg,time.monotonic())

    def receive_takeoff(self, msg):
        with self.lock:
            self.takeoff_state = (bool(msg.data),time.monotonic())

    def enable(self, req):
        self.paused = not req.data
        return SetBoolResponse(True,'exploration enabled' if req.data else 'hover requested')

    def pose(self, xy):
        msg = PoseStamped()
        msg.header.frame_id, msg.header.stamp = getattr(self,'world_frame','map'),rospy.Time.now()
        msg.pose.position.x,msg.pose.position.y = xy
        msg.pose.position.z = float(self.navigation_z(xy))
        msg.pose.orientation.w = 1.
        return msg

    def control_pose(self,xy):
        message=self.pose(xy)
        if getattr(self,'online_loop',False):
            if self.loop_active is None:raise ValueError('no paired loop transform for control goal')
            message.pose=transform_pose(message.pose,self.loop_active.inverse)
            message.header.frame_id='map'
        return message

    def navigation_z(self, xy):
        nav=getattr(self,'ground_navigation',None)
        if nav is not None and self.p('ground_relative_navigation',False):
            return nav.z(xy,self.altitude or 0.)
        return self.altitude or 0.

    def cloud(self, name, inputs):
        msg = inputs[name][0]
        if name not in self.cache or self.cache[name][0] is not msg:
            self.cache[name] = (msg,xyz(msg))
        return self.cache[name][1]

    def slice(self, points, below=None, above=None):
        lower = self.p('height_below',.45) if below is None else below
        upper = self.p('height_above',.55) if above is None else above
        height=self.navigation_z(points[:,:2])
        return points[(points[:,2] >= height-lower) & (points[:,2] <= height+upper)]

    def shift_grid_indices(self, pads):
        low, high, left, right = pads
        if not any(pads):
            return
        if self.target is not None:
            self.target = (self.target[0]+low, self.target[1]+left)
        self.visited = [((c[0]+low,c[1]+left),t) for c,t in self.visited]
        self.inspection_visited = [((c[0]+low,c[1]+left),t) for c,t in getattr(self,'inspection_visited',[])]
        self.blacklist = [((c[0]+low,c[1]+left),t) for c,t in self.blacklist]
        self.failed_visits = {(c[0]+low,c[1]+left):n for c,n in self.failed_visits.items()}
        self.inspected = np.pad(self.inspected, ((low,high),(left,right)), constant_values=False)
        self.inspection_plan = None

    def update_observed_window(self, position, free_points):
        """Grow the local planning window only from pose and fresh lidar freespace."""
        growth_points = np.vstack((np.asarray(position,dtype=float).reshape(1,2),
                                  free_points[:,:2]))
        pads = self.grid.grow_to_include(growth_points,
                    self.p('growth_margin',4.0),self.p('growth_chunk',8.0))
        if pads is None:
            return False
        self.shift_grid_indices(pads)
        if len(free_points):
            self.grid.observe_free(free_points)
        return True

    def publish(self, state, position=None, path=None, candidates=None, recovery=None):
        now = rospy.Time.now()
        route = RosPath()
        route.header.frame_id,route.header.stamp = getattr(self,'world_frame','map'),now
        if position is not None:
            waypoint = self.grid.waypoint(path,position,self.p('lookahead',.6)) if path else position
            if recovery is not None:
                waypoint = recovery
                self.recovery_pub.publish(self.control_pose(recovery))
            self.waypoint_pub.publish(self.control_pose(waypoint))
            route.poses = [self.pose(self.grid.xy(c)) for c in path] if path else [self.pose(position)]
            if recovery is not None:
                route.poses.append(self.pose(recovery))
        self.path_pub.publish(route)
        visualize = (time.monotonic()-self.last_visualization >= 1./self.p('visualization_hz',.5) or
                     (state in ('COMPLETE','RETURN_FAILED') and state!=self.last_written_state))
        if visualize:
            cells = np.full(self.grid.shape,-1,dtype=np.int8)
            cells[self.grid.known] = 0
            cells[self.grid.occupied] = 100
            grid_msg = OccupancyGrid()
            grid_msg.header = route.header
            grid_msg.info.resolution = self.grid.resolution
            grid_msg.info.height,grid_msg.info.width = self.grid.shape
            grid_msg.info.origin.position.x,grid_msg.info.origin.position.y = self.grid.x0,self.grid.y0
            grid_msg.info.origin.orientation.w = 1.
            grid_msg.data = cells.ravel().tolist()
            self.grid_pub.publish(grid_msg)
        self.report = dict(state=state,stamp=now.to_sec(),wall_seconds=time.monotonic()-self.start_wall,
                           known_free_area_m2=float((self.grid.known & ~self.grid.occupied).sum()*self.grid.resolution**2),
                           known_cells=int(self.grid.known.sum()),map_points=len(self.cache.get('map',(None,[]))[1]),
                           goals_selected=self.goals_selected,goals_reached=self.goals_reached,
                           stalled_goals=self.failures,travel_xy_m=self.travel,
                           mission_phase=self.phase,finish_reason=self.finish_reason,
                           home_map_xy=self.home_xy,home_confirmed=self.home_confirmed,
                           home_error_xy_m=math.dist(position,self.home_xy) if position is not None and self.home_xy is not None else None,
                           inspection_cells=int(self.inspected.sum()),
                           coverage_complete=False,
                           flight_budget=self.flight_budget_report,
                           exploration_complete=self.exploration_complete,
                           grid_capacity_limited=self.grid_capacity_limited,
                           completion_scope='observed reachable fixed-altitude space; NOT all 3D surfaces',
                           raw_frontier_candidates=self.raw_frontier_count,
                           useful_frontier_candidates=self.useful_frontier_count,
                           remaining_inspection_candidates=self.remaining_inspection_count,
                           surface_quality=self.surface_report,progress=self.progress_report,
                           observed_map_acceptance=self.acceptance.report if self.acceptance else {},
                           ghost_revisit=dict(enabled=self.p('ghost_revisit_enabled',False),
                               map_status=self.ghost_status,
                               actionable_voxels=int(self.ghost_status.get('actionable_voxels',0)),
                               history_fresh=self.ghost_history_fresh(now.to_sec()),
                               history_records=int(self.ghost_status.get('history_records',0)),
                               known_candidates_resolved=bool(self.ghost_status_fresh() and
                                   self.ghost_history_fresh(now.to_sec()) and
                                   self.ghost_status.get('enabled',False) and
                                   self.ghost_status.get('pending_voxels',0)==0 and
                                   self.ghost_status.get('history_unresolved',0)==0 and
                                   self.ghost_status.get('history_evicted',0)==0 and
                                   self.ghost_status.get('history_overflow',0)==0),
                               attempted_groups={str(k):len(v) for k,v in self.ghost_revisit_visits.items()},
                               view_evidence=self.ghost_view_evidence[-48:],
                               view_selection=self.ghost_view_selection,
                               endurance_trigger=self.ghost_budget_trigger,
                               pending_diagnostics=pending_diagnostics(self.ghost_history,
                                   self.phase,self.ghost_view_selection),
                               vertical_view_diagnostic=self.ghost_vertical_view_diagnostic,
                               cleared_review_candidate_groups=self.ghost_cleared_candidate_groups,
                               cleared_review_reached_groups=len(self.ghost_cleared_view_reached),
                               cleared_review_attempted_groups={str(k):len(v) for k,v in self.ghost_cleared_visits.items()},
                               cleared_review_fallback=self.ghost_cleared_fallback,
                               budget_seconds=self.p('ghost_revisit_budget_seconds',120.)),
                           compute_ms=self.last_compute_ms,
                           compute_p95_ms=float(np.percentile(self.tick_times,95)) if self.tick_times else 0.,
                           search_backend='native' if SEARCH is not None else 'python',
                           planner_mode=self.planner_mode,
                           return_review=getattr(self,'return_review_report',{}),
                           joint_planning=(self.joint_policy.report if getattr(self,'joint_policy',None)
                                           is not None else dict(enabled=False)),
                           coverage_guidance=self.guidance_report,
                           planning_resolution_m=self.grid.resolution,
                           ground_relative_navigation=(self.ground_navigation.summary()
                               if getattr(self,'ground_navigation',None) is not None and
                               self.p('ground_relative_navigation',False) else dict(enabled=False)),
                           planning_frame=getattr(self,'world_frame','map'),
                           online_loop=(dict(enabled=True,revision=self.loop_active.message.revision,
                               graph_revision=self.loop_active.message.graph_revision,
                               requested_graph_revision=self.loop_active.message.requested_graph_revision,
                               rebuilding=self.loop_active.message.rebuilding,
                               input_frames=self.loop_active.message.input_frames,
                               processed_frames=self.loop_active.message.processed_frames)
                               if getattr(self,'loop_active',None) is not None else dict(enabled=False)),
                           mission_sim_seconds=now.to_sec()-self.mission_started if self.mission_started is not None else 0.,
                           estimated_rtf=(now.to_sec()-self.active_sim)/(time.monotonic()-self.active_wall)
                               if self.active_wall is not None and time.monotonic()>self.active_wall else None,
                           target_map_xy=self.grid.xy(self.target) if self.target else None,
                           frontier_candidates=int(candidates.sum()) if candidates is not None else 0,
                           note='Observed 2D area, not 3D completeness. Ghost removal needs reobservation.')
        self.status_pub.publish(json.dumps(self.report))
        if not visualize:
            return  # waypoint/status stay full rate, only heavy visual layers slow
        self.last_visualization = time.monotonic()
        markers = MarkerArray()
        text = Marker(header=route.header,ns='exploration',id=0,type=Marker.TEXT_VIEW_FACING,action=Marker.ADD)
        text.pose.orientation.w = 1.
        if position is not None:
            text.pose.position.x,text.pose.position.y = position
        text.pose.position.z = (self.altitude or 1.2)+1.5
        text.scale.z = .4
        text.color.r,text.color.g,text.color.a = 1.,1.,1.
        text.text = '%s | %.1f m2 | goals %d | useful %d | surface %s' % (
            state,self.report['known_free_area_m2'],self.goals_reached,self.useful_frontier_count,
            ('%.0f%% observed' % (100*self.surface_report['observed_surface_quality_ratio']))
            if self.surface_report.get('observed_surface_quality_ratio') is not None else 'N/A')
        markers.markers.append(text)
        frontier = Marker(header=route.header,ns='exploration',id=1,type=Marker.POINTS,action=Marker.ADD)
        frontier.pose.orientation.w = 1.
        frontier.scale.x = frontier.scale.y = .15
        frontier.color.b = frontier.color.g = frontier.color.a = 1.
        if candidates is not None:
            for c in np.argwhere(candidates)[::3]:
                x,y = self.grid.xy(c)
                frontier.points.append(Point(x=x,y=y,z=self.altitude))
        markers.markers.append(frontier)
        self.markers_pub.publish(markers)
        self.report_file.parent.mkdir(parents=True,exist_ok=True)
        tmp = self.report_file.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.report,indent=2),encoding='utf8')
        os.replace(str(tmp),str(self.report_file))
        self.last_written_state = state
        rospy.loginfo_throttle(10,'Exploration %s area=%.1f m2 reached=%d target=%s',state,
                              self.report['known_free_area_m2'],self.goals_reached,self.report['target_map_xy'])

    def tick(self):
        wall, now = time.monotonic(),rospy.Time.now().to_sec()
        if getattr(self,'joint_policy',None) is not None:
            previous=getattr(self,'joint_last_tick',None)
            selected=self.joint_policy.selected
            if (previous is not None and selected is not None and selected.group is not None
                    and self.phase not in ('COMPLETE','RETURN_FAILED')):
                self.joint_policy.charge(max(0.,now-previous))
            if (previous is not None and selected is not None and selected.kind=='surface_inspection'
                    and self.phase not in ('RETURNING','COMPLETE','RETURN_FAILED')):
                self.joint_policy.used_surface_seconds+=max(0.,now-previous)
            self.joint_last_tick=now
        with self.lock:
            inputs = self.inputs.copy()
            takeoff_state = getattr(self,'takeoff_state',None)
            loop_snapshot=getattr(self,'loop_snapshot',None)
        if getattr(self,'online_loop',False):
            try:inputs=self.prepare_loop_inputs(inputs,loop_snapshot,wall,now)
            except (ValueError,TypeError,AttributeError,KeyError) as error:
                self.home_since=None;self.reset_progress();self.publish('WAIT_LOOP_SNAPSHOT')
                rospy.logerr_throttle(3.,'Online loop planner holds: %s',str(error));return
        if any(name not in inputs or wall-inputs[name][1] > self.p('input_timeout_wall',3.)
               for name in ('odom','map','scan','free','mavros')):
            # Stop publishing waypoints: existing controller watchdog holds.
            self.home_since = None
            self.reset_progress()
            self.publish('WAIT_FRESH_DATA')
            return
        # A publisher's timer can keep receiving fresh while its source scan
        # has stopped. Check acquisition stamps, not receipt time alone.
        if any(now-inputs[name][0].header.stamp.to_sec()>self.p('input_timeout_sim',1.5)
               or inputs[name][0].header.stamp.to_sec()>now+.1
               for name in ('odom','map','scan','free')):
            self.home_since = None
            self.reset_progress()
            self.publish('WAIT_FRESH_DATA')
            return
        odom = inputs['odom'][0]
        p = odom.pose.pose.position
        position = (p.x,p.y)
        if self.home_xy is None:
            self.home_xy = position
        # Default is fail-closed. Only an explicit synthetic fixture may turn
        # this off; a first MAVROS sample is not a valid ground reference.
        if self.p('require_takeoff_confirmation',True) and not (
                takeoff_state and takeoff_state[0] and wall-takeoff_state[1]<=3.):
            self.home_since = self.ready_since = None
            self.reset_progress()
            self.publish('WAIT_TAKEOFF_CONFIRMATION')
            return
        if self.altitude is None:
            # The controller already checks armed/OFFBOARD, airborne and
            # continuous altitude dwell. Use the CURRENT map altitude.
            self.altitude = p.z
            if getattr(self,'online_loop',False):
                self.loop_local_altitude=float((self.loop_active.inverse@np.array([p.x,p.y,p.z,1.]))[2])
            self.settle_until = now+4.
            self.publish('OBSERVING',position)
            return
        if self.p('ground_relative_navigation',False) and self.ground_navigation.plane is None:
            plane=SurfaceEvidence.fit_ground_plane(self.cloud('map',inputs),p.z)
            if not self.ground_navigation.initialize(plane,(p.x,p.y,p.z)):
                self.publish('WAIT_MEASURED_GROUND',position)
                return
        if self.previous is not None:
            self.travel += math.dist(position,self.previous)
        self.previous = position
        if getattr(self,'joint_policy',None) is not None:
            self.joint_policy.observe_position(position)
        free_msg = inputs['free'][0]
        free_points = np.empty((0,3),dtype=float)
        if free_msg is not self.last_free:
            # Only near-flight-height rays establish navigable space, not
            # floor/roof rays which could pass under/over shelving.
            free_points = self.slice(self.cloud('free',inputs),.2,.2)
            self.last_free = free_msg
        # The bounded grid is only a memory/computation window, not a known
        # warehouse outline. Expand it using this vehicle pose and fresh lidar
        # free-space evidence only; never use Gazebo collision boxes or map
        # extrema as a prior. Expansion leaves new cells unknown.
        if not self.update_observed_window(position,free_points):
            self.home_since = None
            self.grid_capacity_limited = True
            if self.phase not in ('RETURNING','COMPLETE','RETURN_FAILED'):
                self.begin_return('map_capacity_partial',now)
            # Retain old grid atomically and use only in-window fresh evidence.
            # Attempt a SAFE return; outside unknown cells are still forbidden.
            self.grid.observe_free(free_points)
            rospy.logerr_throttle(5,'Exploration grid capacity exhausted; attempting safe return, PARTIAL')
        if getattr(self,'online_loop',False):self.loop_seed_revision=None
        points = np.concatenate([self.slice(self.cloud('map',inputs)),self.slice(self.cloud('scan',inputs))])
        self.grid.update_obstacles(points)
        if self.active_wall is None:
            self.active_wall,self.active_sim = wall,now
        def surface_slice(cloud):
            height=self.navigation_z(cloud[:,:2])
            return cloud[(cloud[:,2]>=height-self.p('surface_height_below',.95)) &
                         (cloud[:,2]<=height+self.p('surface_height_above',2.))]
        self.surface.observe(surface_slice(self.cloud('scan',inputs)),(p.x,p.y,p.z),
            inputs['scan'][0].header.stamp.to_sec(),self.p('surface_observation_range',6.))
        deficits,self.surface_report = self.surface.deficits(surface_slice(self.cloud('map',inputs)))
        self.joint_surface_deficits=deficits
        start = self.grid.cell(position)
        if self.mission_started is None:
            self.mission_started = now
        if self.phase in ('COMPLETE','RETURN_FAILED','STALLED_UNSAFE'):
            self.publish(self.phase,position)
            return
        if self.phase != 'RETURNING':
            limit = self.p('max_mission_seconds',3600.)
            goal_limit = self.p('return_after_goals',0)
            if self.return_requested:
                self.begin_return('operator_request',now)
            elif goal_limit>0 and self.goals_reached>=goal_limit:
                self.begin_return('explicit_goal_limit_partial',now)
            elif limit>0 and now-self.mission_started>=limit:
                self.begin_return('mission_budget_partial',now)
            elif self.p('max_mission_wall_seconds',3600.)>0 and wall-self.active_wall>=self.p('max_mission_wall_seconds',3600.):
                self.begin_return('wall_budget_partial',now)
            elif (self.phase!='REVISITING' and self.p('ghost_revisit_enabled',False) and
                  limit>0 and self.ghost_status.get('pending_voxels',0)>0 and
                  now-self.mission_started>=max(0.,limit-self.p('ghost_revisit_budget_seconds',120.))):
                # Reserve the tail of a finite mission for suspected ghosts.
                # The same hard mission deadline still applies during revisit.
                self.begin_revisit_or_return('mission_budget_partial',now,inputs)
        elif now-self.return_started>self.p('return_timeout',1200.):
            self.phase='RETURN_FAILED'
            self.publish('RETURN_FAILED',position)
            return
        if self.paused or now < self.settle_until:
            if self.paused:
                self.reset_progress()  # paused time cannot certify convergence
            self.home_since = None
            self.last_progress,self.progress_position = now,position
            self.publish('PAUSED' if self.paused else 'OBSERVING',position)
            return
        if not self.grid.inside(start) or not self.grid.safe[start]:
            self.reset_progress()
            self.home_since = None
            if self.unsafe_since is None or (self.unsafe_position is not None and
                    math.dist(position,self.unsafe_position) >= .20):
                self.unsafe_since,self.unsafe_position = now,position
            # A permanently unsafe start is an explicit partial-map failure,
            # not an endless hover disguised as ongoing exploration.
            if now-self.unsafe_since > self.p('unsafe_stall_seconds',30.):
                self.phase='STALLED_UNSAFE'
                self.finish_reason='unsafe_start_no_progress'
                self.publish('STALLED_UNSAFE',position)
                return
            recovery = self.grid.reconnect(position,points,self.p('recovery_minimum_clearance',.45),
                                           command_horizon=self.p('recovery_command_horizon',.20))
            if recovery is not None:
                self.last_progress,self.progress_position = now,position
            self.publish('RECOVER_CLEARANCE' if recovery is not None else 'HOLD_START_NOT_SAFE',
                         position,recovery=recovery)
            return
        self.unsafe_since = self.unsafe_position = None
        distances,parent = self.grid.search(start,clearance_weight=self.p('clearance_cost_weight',1.5))
        if self.flight_budget is not None:
            home=self.grid.cell(self.home_xy)
            home_distance=float(distances[home]) if self.grid.inside(home) else float('inf')
            self.flight_budget_report=self.flight_budget.report(now,home_distance)
            if not self.flight_budget.origin_verified or self.flight_budget_report['elapsed_seconds'] is None:
                self.publish('WAIT_FLIGHT_BUDGET_ORIGIN',position)
                return
            if self.phase!='RETURNING' and self.flight_budget_report.get('hold_for_home_path'):
                self.publish('HOLD_REPLAN_HOME_PATH',position)
                return
            if self.phase!='RETURNING' and self.flight_budget_report['return_now']:
                self.begin_return('flight_budget_return_partial',now)
            if self.grid.inside(home) and self.grid.safe[home]:
                self.flight_home_distances,_=self.grid.search(home,
                    clearance_weight=self.p('clearance_cost_weight',1.5))
        if self.phase == 'RETURNING':
            self.return_tick(now,position,start,distances,parent,inputs)
            return
        if getattr(self,'joint_policy',None) is not None:
            self.joint_tick(now,position,start,distances,parent,inputs)
            return
        if self.endurance_revisit_due(now):
            self.begin_revisit_or_return('endurance_reserved_ghost_revisit_partial',now,inputs)
        if self.phase == 'REVISITING':
            self.ghost_revisit_tick(now,position,start,distances,parent,inputs)
            return
        self.progress.update(now,int(self.grid.known.sum()),len(self.surface.ever),self.goals_reached)
        self.progress_report = self.progress.summary(self.grid.resolution)
        if self.acceptance is not None and self.acceptance.update(
                now,inputs['scan'][0].header.stamp.to_sec(),int(self.grid.known.sum()),
                self.grid.resolution,self.goals_reached,now-self.mission_started,
                self.surface_report,self.grid_capacity_limited):
            # Residual frontiers may remain: this relaxed demo exit is PARTIAL.
            if self.begin_revisit_or_return('observed_map_accepted_partial',now,inputs):
                self.ghost_revisit_tick(now,position,start,distances,parent,inputs)
            else:
                self.return_tick(now,position,start,distances,parent)
            return
        candidates,gain = self.grid.frontiers(distances)
        self.raw_frontier_count = int(candidates.sum())
        self.blacklist = [(c,t) for c,t in self.blacklist if t > now]
        for c,t in self.blacklist:
            suppress_disk(candidates,c,1.5/self.grid.resolution)
        # Mission-long memory prevents the old 240-second revisit loop. The
        # separate close-range inspection pass covers already-known aisles.
        for c,t in self.visited:
            suppress_disk(candidates,c,1.8/self.grid.resolution)
        for c,count in self.failed_visits.items():
            if count >= self.p('max_target_attempts',2):
                suppress_disk(candidates,c,1.5/self.grid.resolution)
        cells = representative_cells(candidates,gain,distances,self.grid.resolution)
        visible_gain = visible_unknown_gain(self.grid,cells,self.p('frontier_sensor_range',6.))
        candidates[:] = False
        gain = np.zeros(self.grid.shape,dtype=float)
        useful = cells[visible_gain>=self.p('frontier_min_gain_area',.5)]
        if len(useful):
            candidates[tuple(useful.T)] = True
            gain[tuple(useful.T)] = visible_gain[visible_gain>=self.p('frontier_min_gain_area',.5)]
        self.useful_frontier_count = int(candidates.sum())
        # Low gain does not equal full coverage. It moves to surface inspection
        # and retains the residual frontier count in the report.
        if (self.phase=='EXPLORING' and self.progress_report['low_gain'] and
                self.goals_reached>=self.p('convergence_min_goals',3)):
            self.phase='INSPECTING'
            self.inspection_started=now
            self.target=None
            self.no_frontier_since=None
            self.inspection_plan=None
        self.remaining_inspection_count = 0
        if self.phase == 'INSPECTING':
            frontier_candidates,frontier_gain = candidates,gain
            # Plans are coarse observed-surface views; safety/reachability is
            # always rechecked at full tick rate, including new moving people.
            if self.inspection_plan is None or now-self.inspection_last_plan>=self.p('inspection_plan_seconds',3.):
                self.inspection_plan = self.surface.inspection_candidates(self.grid,distances,deficits,
                    limit=self.p('inspection_max_surface_targets',60),
                    range_m=self.p('inspection_surface_range',3.),
                    arrival_tolerance=self.p('arrival_tolerance',.3))
                self.inspection_last_plan = now
            candidates,gain = (v.copy() for v in self.inspection_plan)
            candidates &= self.grid.safe & np.isfinite(distances)
            for c,count in self.failed_visits.items():
                if count >= self.p('max_target_attempts',2):
                    suppress_disk(candidates,c,1.5/self.grid.resolution)
            for c,t in self.blacklist:
                suppress_disk(candidates,c,1.5/self.grid.resolution)
            # An arrival is NOT proof that the surface gained a second view.
            # Revisit a still-deficient safe view after a cooldown instead of
            # permanently hiding holes as the frontier visit memory does.
            self.inspection_visited=[(c,t) for c,t in self.inspection_visited
                                     if now-t<self.p('inspection_revisit_seconds',20.)]
            for c,t in self.inspection_visited:
                suppress_disk(candidates,c,self.p('inspection_visit_spacing',.8)/self.grid.resolution)
            self.remaining_inspection_count=int(candidates.sum())
            if not self.remaining_inspection_count and self.useful_frontier_count:
                # Inspection can reveal a new opening (e.g. a person leaves).
                # Do not abandon that frontier just because we entered the
                # inspection phase earlier. Start a NEW convergence window;
                # old low-gain evidence must not immediately flip us back.
                self.phase='EXPLORING'
                self.target=None
                self.no_frontier_since=None
                self.reset_progress()
                candidates,gain=frontier_candidates,frontier_gain
                self.publish('RESUME_EXPLORATION',position,candidates=candidates)
                return
            quality=self.surface_report.get('observed_surface_quality_ratio')
            qualified=(quality is not None and quality>=self.p('surface_quality_target',.95) and
                       not self.surface.capacity_limited)
            if (self.progress_report['low_gain'] and qualified and not self.useful_frontier_count and
                    not self.remaining_inspection_count and not self.failed_visits and
                    not self.grid_capacity_limited):
                self.exploration_complete=True
                if self.begin_revisit_or_return('observed_reachable_converged',now,inputs):
                    self.ghost_revisit_tick(now,position,start,distances,parent,inputs)
                else:
                    self.publish('RETURNING',position)
                return
            if self.inspection_started is None:
                self.inspection_started=now
            if now-self.inspection_started>=self.p('inspection_budget_seconds',600.):
                if self.begin_revisit_or_return('inspection_budget_partial',now,inputs):
                    self.ghost_revisit_tick(now,position,start,distances,parent,inputs)
                else:
                    self.publish('RETURNING',position)
                return
        if self.flight_budget is not None and self.flight_home_distances is not None:
            had_candidates=bool(candidates.any())
            affordable=self.flight_budget.feasible(now,distances,self.flight_home_distances,
                self.p('settle_seconds',1.)+self.p('ghost_view_time_reserve_seconds',5.))
            candidates &= affordable
            if self.target is not None and not affordable[self.target]:
                self.target=None
            if had_candidates and not candidates.any():
                if (self.p('ghost_endurance_revisit',False) and
                        self.ghost_status.get('pending_voxels',0)>0 and
                        self.begin_revisit_or_return('flight_budget_no_affordable_view_partial',now,inputs)):
                    self.ghost_revisit_tick(now,position,start,distances,parent,inputs)
                else:
                    self.begin_return('flight_budget_no_affordable_view_partial',now)
                    self.return_tick(now,position,start,distances,parent)
                return
        if self.target is not None:
            if math.dist(position,self.grid.xy(self.target)) <= self.p('arrival_tolerance',.3):
                self.goals_reached += 1
                self.visited.append((self.target,now))
                if self.phase=='INSPECTING':
                    self.inspection_visited.append((self.target,now))
                self.target = None
                self.settle_until = now+self.p('settle_seconds',3.)
                self.publish('OBSERVING',position,candidates=candidates)
                return
            if math.dist(position,self.progress_position) >= .25:
                self.progress_position,self.last_progress = position,now
            if not np.isfinite(distances[self.target]) or now-self.last_progress > self.p('progress_timeout',25.):
                self.failed_visits[self.target] = self.failed_visits.get(self.target,0)+1
                self.blacklist.append((self.target,now+self.p('blacklist_seconds',60.)))
                self.target = None
                self.failures += 1
                # Recompute suppression next tick before selecting another goal.
                self.publish('REPLAN_BLOCKED',position,candidates=candidates)
                return
        if self.target is None:
            options = np.argwhere(candidates)
            if len(options):
                if self.phase == 'EXPLORING' and self.coverage_guide is not None:
                    self.target,self.guidance_report = self.coverage_guide.select(
                        self.grid,options,gain,distances,self.visited,
                        self.p('clearance_cost_weight',1.5))
                else:
                    # Baseline and inspection both retain the proven scorer.
                    if self.flight_budget is not None and self.flight_home_distances is not None:
                        scores=[gain[tuple(c)]/(2.+self.p('settle_seconds',1.)+
                            float(self.flight_budget.travel_seconds(distances[tuple(c)]))+
                            .35*float(self.flight_budget.travel_seconds(self.flight_home_distances[tuple(c)])))
                            for c in options]
                    else:
                        scores = [gain[tuple(c)]/(2.+distances[tuple(c)]) for c in options]
                    self.target = tuple(options[int(np.argmax(scores))])
                    self.guidance_report = {}
                if self.target is None:
                    self.publish('WAIT_FRONTIER',position,candidates=candidates)
                    return
                self.goals_selected += 1
                self.last_progress,self.progress_position = now,position
                self.goal_pub.publish(self.pose(self.grid.xy(self.target)))
                self.no_frontier_since = None
            else:
                self.no_frontier_since = self.no_frontier_since or now
                state = 'WAIT_FRONTIER'
                if now-self.no_frontier_since > self.p('no_frontier_seconds',30.):
                    if self.phase == 'EXPLORING':
                        self.phase='INSPECTING'
                        self.inspection_started=now
                        self.inspection_plan=None
                        self.no_frontier_since=None
                        state='START_INSPECTION'
                    else:
                        if (qualified and not self.useful_frontier_count and
                                not self.failed_visits and not self.grid_capacity_limited and
                                not self.progress_report['low_gain']):
                            # Wait for the full evidence window instead of
                            # declaring completion after only a short pause.
                            self.publish('VERIFY_CONVERGENCE',position,candidates=candidates)
                            return
                        if self.begin_revisit_or_return('reachable_viewpoints_exhausted',now,inputs):
                            self.ghost_revisit_tick(now,position,start,distances,parent,inputs)
                            return
                        state='RETURNING'
                self.publish(state,position,candidates=candidates)
                return
        path = self.grid.path(parent,start,self.target)
        self.publish(self.phase,position,path,candidates)

    def joint_event(self, event, now, **values):
        """Append bounded decision evidence separately from timer diagnostics."""
        record=dict(event=event,stamp=float(now),**values)
        path=self.report_file.with_suffix('.joint_events.jsonl')
        path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('a',encoding='utf8') as stream:
            stream.write(json.dumps(record,allow_nan=False)+'\n')

    def joint_finish_observation(self, now, inputs, position):
        probe=self.joint_observation
        if probe is None:return False
        view=probe['view'];policy=self.joint_policy
        close=math.dist(position,view.xy)<=self.p('arrival_tolerance',.3)
        stamp=inputs['scan'][0].header.stamp.to_sec()
        if close and stamp>=probe['arrived'] and stamp>probe['last_stamp']:
            probe['scans']+=1;probe['last_stamp']=stamp
        elapsed=now-probe['arrived']
        minimum=policy.p['joint_min_fresh_view_scans']
        if elapsed<self.p('ghost_observation_seconds',4.) or (
                probe['scans']<minimum and elapsed<policy.p['joint_max_view_seconds']):
            self.publish('JOINT_OBSERVING',position)
            return True
        measured=bool(close and probe['scans']>=minimum and self.ghost_history_fresh(now))
        episodes=[int(r.get('suspected_episodes',1)) for r in self.ghost_history.get('records',[])
                  if len(r.get('position',[]))==3 and
                  tuple(np.floor(np.asarray(r['position'][:2])/.8).astype(int))==view.group]
        measured=bool(measured and episodes and max(episodes)==view.episode[2])
        after=self.ghost_group_counts(view.group) if measured else {}
        free=positive=hits=resolved=0
        for key,before in probe['before'].items():
            current=after.get(key)
            if current is None:continue
            free+=max(0,current[0]-before[0])
            hits+=max(0,current[2]-before[2])
            positive+=max(0,current[4]-before[4])
            resolved+=int(before[3]=='suspected' and current[3]!='suspected')
        effective=bool(measured and (free or hits or positive or resolved))
        policy.complete_view(view,now,effective,measured)
        result=dict(group=str(view.group),view_xy=list(view.xy),
            selected_stamp=probe['selected'],arrived_stamp=probe['arrived'],checked_stamp=now,
            status='measured' if measured else 'insufficient_fresh_evidence',
            fresh_scans=probe['scans'],free_evidence_delta=free,post_departure_hit_delta=hits,
            positive_endpoint_delta=positive,resolved_voxels=resolved,
            effective=effective,predicted_quality=probe['quality'])
        self.ghost_view_evidence.append(result)
        self.joint_event('observation_result',now,**result)
        self.goals_reached+=1
        if view.kind=='history_review':self.ghost_cleared_view_reached.add(view.group)
        self.joint_observation=None;self.target=None
        self.no_frontier_since=None
        return False

    def region_feedback_enabled(self):
        policy=getattr(self,'joint_policy',None)
        return bool(policy is not None and getattr(policy,'p',{}).get('joint_region_feedback_enabled',False))

    def region_feedback_begin(self,view,now):
        if not self.region_feedback_enabled() or view.group is not None:return
        identity=(tuple(round(v,5) for v in view.xy),view.kind)
        active=getattr(self,'region_observation',None)
        if active is not None and active['identity']==identity:return
        if active is not None:self.joint_event('region_observation_interrupted',now,view=active['view'].summary())
        self.region_observation=dict(identity=identity,view=view,started=now,
            scans=self.surface.scan_count,arrived=None,arrival_snapshot=None,
            before=self.joint_policy.region_memory.capture(view.benefit_claims,self.grid,self.surface))

    def region_feedback_arrive(self,view,now):
        if not self.region_feedback_enabled():return
        active=getattr(self,'region_observation',None)
        if active is None:return
        if active['arrived'] is None:
            active['arrived']=now
            active['arrival_snapshot']=self.joint_policy.region_memory.capture(view.benefit_claims,self.grid,self.surface)

    def region_feedback_finish(self,view,now,valid=True):
        if not self.region_feedback_enabled():return
        active=getattr(self,'region_observation',None)
        if active is None:return
        self.region_observation=None
        memory=self.joint_policy.region_memory
        after=memory.capture(active['view'].benefit_claims,self.grid,self.surface)
        fresh=self.surface.scan_count-active['scans']
        valid=bool(valid and active['arrived'] is not None and fresh>=8)
        details=memory.finish(active['view'].benefit_claims,active['before'],after,now,active['view'].xy,valid)
        arrival=active['arrival_snapshot'] or active['before']
        def phase(a,b):
            return [dict(key=list(c.key),**memory.delta(a.get(c.key,{}),b.get(c.key,{}))) for c in active['view'].benefit_claims]
        self.joint_event('region_observation_result',now,selected=active['view'].summary(),valid=valid,
            elapsed_seconds=now-active['started'],fresh_scans=fresh,details=details,
            travel_evidence=phase(active['before'],arrival),dwell_evidence=phase(arrival,after),
            memory=memory.report())
        return dict(valid=valid,useful=any(r['useful'] for r in details),details=details)

    def joint_finish_surface_observation(self, now, inputs, position):
        probe=getattr(self,'joint_surface_observation',None)
        if probe is None:return False
        view=probe['view'];cell=self.grid.cell(view.xy);policy=self.joint_policy
        complete,expired=probe['dwell'].update(now,inputs['scan'][0].header.stamp.to_sec(),
            math.dist(position,view.xy)<=self.p('arrival_tolerance',.3))
        if not complete and not expired:
            self.publish('JOINT_SURFACE_OBSERVING',position)
            return True
        self.joint_event('surface_observation_result',now,selected=view.summary(),
            complete=complete,fresh_scan_samples=probe['dwell'].scans,
            new_surface_voxels=max(0,len(self.surface.ever)-self.joint_surface_baseline[1])
                if getattr(self,'joint_surface_baseline',None) is not None else None,
            elapsed_seconds=now-probe['dwell'].started)
        if complete:
            self.goals_reached+=1;self.visited.append((cell,now))
            self.inspection_visited.append((cell,now));policy.complete_inspection(view)
            if not self.region_feedback_enabled() and getattr(self,'joint_surface_baseline',None) is not None:
                policy.observe_surface_gain(max(0,len(self.surface.ever)-self.joint_surface_baseline[1]))
        else:
            self.failed_visits[cell]=self.failed_visits.get(cell,0)+1
            self.blacklist.append((cell,now+self.p('blacklist_seconds',60.)))
        policy.selected=None;policy.last_decision=-float('inf');self.target=None
        self.joint_surface_observation=None
        self.joint_surface_baseline=None
        feedback=self.region_feedback_finish(view,now,complete)
        if complete and feedback is not None and feedback['valid']:
            policy.observe_surface_gain(policy.p['joint_surface_min_new_voxels'] if feedback['useful'] else 0)
        return False

    def joint_tick(self, now, position, start, distances, parent, inputs):
        """Independent task policy, after ALL original freshness/safety gates."""
        policy=self.joint_policy
        if self.region_feedback_enabled() and self.ghost_history_fresh(now):
            zero=policy.p['joint_home_ghost_reward_scale']==0.
            policy.region_memory.notify_scene(self.ghost_history.get('records',[]),now,
                self.home_xy if zero else None,policy.p['joint_home_review_radius_m'] if zero else 0.)
        pending=getattr(self,'region_frontier_pending',None)
        if pending is not None:
            self.region_feedback_finish(pending,now,True);self.region_frontier_pending=None
        if policy.p['joint_viewpoint_sweep_enabled']:
            self.mark_inspected(position)
        self.phase='EXPLORING'
        self.progress.update(now,int(self.grid.known.sum()),len(self.surface.ever),self.goals_reached)
        self.progress_report=self.progress.summary(self.grid.resolution)
        accepted=(self.acceptance.update(now,inputs['scan'][0].header.stamp.to_sec(),
            int(self.grid.known.sum()),self.grid.resolution,self.goals_reached,
            now-self.mission_started,self.surface_report,self.grid_capacity_limited)
            if self.acceptance is not None else False)
        selected=policy.selected
        if selected is not None:
            cell=self.grid.cell(selected.xy)
            if not self.grid.inside(cell) or not self.grid.safe[cell] or not np.isfinite(distances[cell]):
                self.joint_event('target_invalidated',now,selected=selected.summary())
                policy.selected=None;policy.last_decision=-float('inf');self.target=None
                self.joint_observation=None
                self.joint_surface_observation=None
                self.region_observation=None
        if self.joint_finish_surface_observation(now,inputs,position):return
        if self.joint_finish_observation(now,inputs,position):return
        selected=policy.selected
        if selected is not None:
            cell=self.grid.cell(selected.xy)
            if math.dist(position,selected.xy)<=self.p('arrival_tolerance',.3):
                self.region_feedback_arrive(selected,now)
                if selected.group is not None:
                    self.joint_observation=dict(view=selected,arrived=now,last_stamp=-float('inf'),
                        scans=0,before=self.ghost_group_counts(selected.group),
                        selected=policy.last_selection_stamp,quality=selected.quality)
                    self.joint_event('observation_started',now,selected=selected.summary())
                    self.publish('JOINT_OBSERVING',position)
                    return
                if selected.kind=='surface_inspection':
                    self.joint_surface_observation=dict(view=selected,
                        dwell=ScanDwell(now,selected.dwell_s,
                            min(8,max(2,int(math.ceil(2*selected.dwell_s))))))
                    self.publish('JOINT_SURFACE_OBSERVING',position)
                    return
                self.goals_reached+=1;self.visited.append((cell,now))
                if self.region_feedback_enabled():self.region_frontier_pending=selected
                policy.selected=None;policy.last_decision=-float('inf');self.target=None
                self.settle_until=now+self.p('settle_seconds',1.)
                self.publish('OBSERVING',position)
                return
            elif math.dist(position,self.progress_position)>=.25:
                self.last_progress,self.progress_position=now,position
            elif now-self.last_progress>self.p('progress_timeout',25.):
                self.failed_visits[cell]=self.failed_visits.get(cell,0)+1
                self.blacklist.append((cell,now+self.p('blacklist_seconds',60.)))
                if selected.group is not None:policy.complete_view(selected,now,False,False)
                else:policy.selected=None
                policy.last_decision=-float('inf');self.target=None;self.failures+=1
                self.region_observation=None
        if now-policy.last_decision>=1./policy.p['joint_decision_hz'] or policy.selected is None:
            point_map=np.concatenate((self.cloud('map',inputs),self.cloud('scan',inputs)))
            indoor_weights=None
            if policy.p['joint_indoor_structure_focus']:
                focus_signature=(self.grid.shape,self.grid.x0,self.grid.y0)
                if (getattr(self,'joint_indoor_grid',None)!=focus_signature or
                        now-getattr(self,'joint_indoor_stamp',-float('inf'))>=5.):
                    ceiling,on=self.surface.measured_ceiling(point_map,self.altitude)
                    self.joint_indoor_weights=None;self.joint_ceiling_plane=ceiling
                    if ceiling is not None:
                        focus=expand(self.grid.raster(point_map[on]),.8/self.grid.resolution)
                        self.joint_indoor_weights=np.where(focus,1.,policy.p['joint_outside_ceiling_gain_scale'])
                    self.joint_indoor_grid=focus_signature;self.joint_indoor_stamp=now
                indoor_weights=self.joint_indoor_weights
            corridor_wall_mask=None
            ceiling=getattr(self,'joint_ceiling_plane',None)
            if policy.p['joint_corridor_progress_enabled'] and ceiling is not None:
                gap=point_map[:,:2]@ceiling[:2]+ceiling[2]-point_map[:,2]
                near_top=point_map[(gap>=.15)&(gap<=.55)]
                corridor_wall_mask=expand(self.grid.raster(near_top),1.01)
            candidates,gain=self.grid.frontiers(distances)
            self.raw_frontier_count=int(candidates.sum())
            self.blacklist=[(c,t) for c,t in self.blacklist if t>now]
            for cell,_ in self.blacklist:suppress_disk(candidates,cell,1.5/self.grid.resolution)
            for cell,count in self.failed_visits.items():
                if count>=self.p('max_target_attempts',2):suppress_disk(candidates,cell,1.5/self.grid.resolution)
            frontier_claim_cache={}
            corridor_cells=np.empty((0,2),dtype=int);corridor_gains=np.empty(0)
            if policy.p['joint_corridor_progress_enabled']:
                corridor_cells=policy.corridor.local_frontiers(self.grid,position,distances,candidates,
                    policy.route_memory)
                if self.region_feedback_enabled():
                    corridor_gains,claims=visible_unknown_gain(self.grid,corridor_cells,self.p('frontier_sensor_range',6.),return_claims=True)
                    frontier_claim_cache.update({tuple(c):(np.array([g]),[cl]) for c,g,cl in zip(corridor_cells,corridor_gains,claims)})
                else:corridor_gains=visible_unknown_gain(self.grid,corridor_cells,self.p('frontier_sensor_range',6.))
                if indoor_weights is not None:
                    corridor_gains*=indoor_weights[tuple(corridor_cells.T)]
            for cell,_ in self.visited:suppress_disk(candidates,cell,1.8/self.grid.resolution)
            cells=representative_cells(candidates,gain,distances,self.grid.resolution)
            # Preserve the original ray-based unknown-gain estimator.
            if self.region_feedback_enabled():
                gains,claims=visible_unknown_gain(self.grid,cells,self.p('frontier_sensor_range',6.),return_claims=True)
                frontier_claim_cache.update({tuple(c):(np.array([g]),[cl]) for c,g,cl in zip(cells,gains,claims)})
            else:gains=visible_unknown_gain(self.grid,cells,self.p('frontier_sensor_range',6.))
            raw_cells,raw_gains=cells,gains.copy()
            if indoor_weights is not None:
                # Prefer observed covered viewpoints, without assuming that
                # unknown space beyond the measured roof is outdoors.
                gains=gains*indoor_weights[tuple(cells.T)]
            valid=gains>=self.p('frontier_min_gain_area',.5)
            cells,gains=cells[valid],gains[valid]
            self.useful_frontier_count=len(cells)
            gate=policy.p['joint_surface_frontier_gate_m2']
            surface_allowed=not (gate>0 and len(cells) and np.any((gains>=gate) &
                self.flight_budget.feasible(now,distances[tuple(cells.T)],
                    self.flight_home_distances[tuple(cells.T)],1.)))
            policy.report['surface_waiting_for_productive_frontier']=not surface_allowed
            before=policy.selected
            fresh=self.ghost_status_fresh() and self.ghost_history_fresh(now)
            inspection_cells=np.empty((0,2),dtype=int);inspection_gains=np.empty(0);inspection_sources=[];inspection_regions=[];inspection_claims=[]
            elapsed=self.flight_budget.elapsed(now)
            if (surface_allowed and (not policy.surface_stopped or self.region_feedback_enabled()) and policy.p['joint_surface_inspection_enabled'] and elapsed is not None and
                    elapsed>=policy.p['joint_surface_start_seconds'] and
                    policy.used_surface_seconds<policy.p['joint_surface_action_budget_seconds']):
                signature=(self.grid.shape,self.grid.x0,self.grid.y0)
                if (getattr(self,'joint_surface_plan',None) is None or
                        getattr(self,'joint_surface_grid',None)!=signature or
                        now-getattr(self,'joint_surface_last_plan',-float('inf'))>=5.):
                    self.joint_surface_plan=self.surface.inspection_candidates(
                        self.grid,distances,getattr(self,'joint_surface_deficits',np.empty((0,3))),
                        limit=12,range_m=5.0,altitude=self.altitude,robust_quota=True,
                        view_altitude=self.navigation_z,
                        arrival_tolerance=self.p('arrival_tolerance',.3),
                        min_elevation=self.p('ghost_sensor_min_elevation_deg',-22.2),
                        max_elevation=self.p('ghost_sensor_max_elevation_deg',37.2))
                    if policy.p['joint_unseen_geometry_enabled']:
                        extra,extra_gain=self.surface.missing_geometry_candidates(
                            self.grid,distances,point_map,self.altitude,limit=32,range_m=6.,
                            arrival_tolerance=self.p('arrival_tolerance',.3),
                            min_elevation=self.p('ghost_sensor_min_elevation_deg',-22.2),
                            max_elevation=self.p('ghost_sensor_max_elevation_deg',37.2),
                            lidar_height=float(self.p('lidar_origin_in_body',[0.,0.,.13])[2]),
                            view_altitude=self.navigation_z,task_claims_enabled=self.region_feedback_enabled())
                        old_mask,old_gain=self.joint_surface_plan
                        self.joint_surface_plan=(old_mask|extra,old_gain+extra_gain)
                    self.joint_surface_grid=signature;self.joint_surface_last_plan=now
                mask,mass=(v.copy() for v in self.joint_surface_plan)
                mask &= self.grid.safe & np.isfinite(distances)
                self.inspection_visited=[(c,t) for c,t in self.inspection_visited if now-t<45.]
                for c,_ in self.inspection_visited:suppress_disk(mask,c,.8/self.grid.resolution)
                for c,_ in self.blacklist:suppress_disk(mask,c,1.5/self.grid.resolution)
                for c,count in self.failed_visits.items():
                    if count>=self.p('max_target_attempts',2):suppress_disk(mask,c,1.5/self.grid.resolution)
                channels={}
                if policy.p['joint_unseen_geometry_enabled']:
                    channels={('missing_'+name):(layer & mask) for name,layer in
                        getattr(self.surface,'last_missing_geometry_masks',{}).items()}
                channels['measured']=mask & ~np.logical_or.reduce(list(channels.values())) if channels else mask
                if policy.p['joint_viewpoint_sweep_enabled']:
                    sweep,sweep_gain,sweep_report=sweep_candidates(self.grid,distances,self.inspected,
                        focus=None if indoor_weights is None else indoor_weights>.5,
                        radius=self.p('inspection_radius',2.))
                    for c,_ in self.blacklist:suppress_disk(sweep,c,1.5/self.grid.resolution)
                    policy.report['viewpoint_sweep']=sweep_report
                    if policy.p['joint_bind_sweep_to_geometry']:
                        # A nearby-body-visit proxy must never replace the
                        # real floor/back requests or invent a wall-sweep task.
                        mass,count=geometry_bound_sweep_gain(sweep,sweep_gain,channels,mask,mass,
                            policy.p['joint_missing_geometry_gain_scale'])
                        policy.report['geometry_bound_sweep_cells']=count
                    elif sweep.any():
                        mask=sweep;mass=sweep_gain*16.;channels={'missing_sweep':sweep}
                parts=[];values=[]
                for source,layer in channels.items():
                    reps=representative_cells(layer,mass,distances,self.grid.resolution)
                    if len(reps):
                        parts.append(reps);values.append(mass[tuple(reps.T)])
                        if indoor_weights is not None:
                            values[-1]=values[-1]*indoor_weights[tuple(reps.T)]
                        inspection_sources.extend([source]*len(reps))
                        region_lookup=getattr(self.surface,'last_missing_geometry_regions',{}).get(source.replace('missing_',''),{})
                        claim_lookup=(getattr(self.surface,'last_measured_inspection_claims',{}) if source=='measured' else
                        getattr(self.surface,'last_missing_geometry_claims',{}).get(source.replace('missing_',''),{}))
                    inspection_claims.extend([tuple(claim_lookup.get(tuple(c),())) for c in reps])
                    inspection_regions.extend([(('sweep',*map(int,np.floor(
                            np.asarray(self.grid.xy(tuple(c)))/2.4))),) if source=='missing_sweep'
                            else tuple(sorted(region_lookup.get(tuple(c),()))) for c in reps])
                inspection_cells=np.concatenate(parts) if parts else np.empty((0,2),dtype=int)
                inspection_gains=np.concatenate(values) if values else np.empty(0)
                self.remaining_inspection_count=int(mask.sum())
            def choose(fc,fg):
                return policy.select(self.grid,position,distances,self.flight_home_distances,fc,fg,
                    self.ghost_history.get('records',[]) if fresh else [],point_map,
                    self.slice(self.cloud('scan',inputs)),self.altitude,now,self.flight_budget,
                    ghost_fresh=fresh,pending_age=self.p('ghost_pending_min_age_seconds',20.),
                    max_attempts=self.p('ghost_max_attempts',2),view_change=self.p('ghost_view_change',.7),
                    dwell=self.p('ghost_observation_seconds',4.),
                    range_min=self.p('ghost_view_min_range',1.2),range_max=self.p('ghost_view_max_range',4.5),
                    min_elevation=self.p('ghost_sensor_min_elevation_deg',-22.2),
                    max_elevation=self.p('ghost_sensor_max_elevation_deg',37.2),
                    cleared_group_limit=self.p('ghost_cleared_max_groups',2),
                    inspection_cells=inspection_cells,inspection_gains=inspection_gains,
                    inspection_sources=inspection_sources,inspection_regions=inspection_regions,
                    view_altitude=self.navigation_z,home_xy=self.home_xy,surface_allowed=surface_allowed,
                    route_parent=parent,indoor_weights=indoor_weights,
                    corridor_cells=corridor_cells,corridor_gains=corridor_gains,
                    corridor_wall_mask=corridor_wall_mask,inspection_claims=inspection_claims,
                    frontier_claim_cache=frontier_claim_cache)
            selected=choose(cells,gains)
            focus_fallback=False
            if selected is None and indoor_weights is not None:
                valid_raw=raw_gains>=self.p('frontier_min_gain_area',.5)
                if valid_raw.any():
                    selected=choose(raw_cells[valid_raw],raw_gains[valid_raw]);focus_fallback=True
            policy.report['surface_generation']=getattr(self.surface,'last_inspection_report',{})
            policy.report['measured_ceiling_focus']=dict(enabled=policy.p['joint_indoor_structure_focus'],
                inferred=indoor_weights is not None,unweighted_frontier_fallback=focus_fallback,
                scope='Soft viewpoint reward only; navigation/free evidence unchanged; unknown rooms may still be explored')
            policy.report['missing_geometry_generation']=getattr(self.surface,'last_missing_geometry_report',{})
            self.joint_event('decision',now,**policy.report)
            if selected is not None:self.region_feedback_begin(selected,now)
            if selected is not None and (before is None or selected.xy!=before.xy or selected.kind!=before.kind):
                self.joint_surface_baseline=((selected.xy,len(self.surface.ever))
                    if selected.kind=='surface_inspection' else None)
                self.goals_selected+=1;self.last_progress,self.progress_position=now,position
                self.goal_pub.publish(self.pose(selected.xy))
        selected=policy.selected
        low=self.progress_report.get('window_ready') and self.progress_report.get('low_gain')
        if low:self.joint_low_gain_since=now if self.joint_low_gain_since is None else self.joint_low_gain_since
        else:self.joint_low_gain_since=None
        # ProgressWindow already spans 90s by default. Require the additional
        # 45s windows only when more than two windows were explicitly asked.
        low_duration=max(0,policy.p['joint_low_gain_windows']-2)*self.p('acceptance_window_seconds',45.)
        surface_pending=(not policy.surface_stopped and policy.p['joint_surface_inspection_enabled'] and
            self.remaining_inspection_count>0 and
            policy.used_surface_seconds<policy.p['joint_surface_action_budget_seconds'])
        if (accepted and low and not surface_pending and self.joint_low_gain_since is not None and
                now-self.joint_low_gain_since>=low_duration):
            self.begin_return('joint_observed_map_accepted_partial',now)
            self.return_tick(now,position,start,distances,parent);return
        if selected is None:
            self.target=None
            if policy.report.get('corridor_progress',{}).get('hold'):
                self.joint_no_candidates_since=None
                self.publish('CORRIDOR_SCAN_HOLD',position);return
            if self.region_feedback_enabled() and policy.report.get('tail_mode') and not policy.report.get('eligible_surface_count',0) and not policy.report.get('eligible_frontier_count',0):
                self.begin_return('low_expected_incremental_gain_partial',now)
                self.return_tick(now,position,start,distances,parent,inputs);return
            if policy.surface_stopped and self.useful_frontier_count==0:
                self.begin_return('joint_low_marginal_surface_gain_partial',now)
                self.return_tick(now,position,start,distances,parent,inputs)
                return
            self.joint_no_candidates_since=now if self.joint_no_candidates_since is None else self.joint_no_candidates_since
            wait=(2. if policy.report.get('rejected',{}).get('return_budget',0)
                  else self.p('no_frontier_seconds',15.))
            if now-self.joint_no_candidates_since>=wait:
                self.begin_return('joint_no_safe_affordable_view_partial',now)
                self.return_tick(now,position,start,distances,parent)
            else:self.publish('JOINT_WAIT_SAFE_VIEW',position)
            return
        self.joint_no_candidates_since=None
        self.target=self.grid.cell(selected.xy)
        path=self.grid.path(parent,start,self.target)
        self.publish('JOINT_VERIFYING' if selected.group is not None else 'JOINT_EXPLORING',position,path)

    def save(self):
        try:
            rospy.wait_for_service('/demo/save_map',timeout=1.)
            result = rospy.ServiceProxy('/demo/save_map',Trigger)()
            if not result.success:
                rospy.logwarn('Exploration map save: %s',result.message)
            return result.success
        except (rospy.ROSException,rospy.ServiceException) as exc:
            rospy.logwarn('Exploration map save unavailable: %s',exc)
            return False

    def run(self):
        while not rospy.is_shutdown():
            began = time.monotonic()
            self.tick()
            self.last_compute_ms = (time.monotonic()-began)*1000.
            self.tick_times.append(self.last_compute_ms)
            self.tick_times = self.tick_times[-120:]
            if self.phase!='COMPLETE' and began-self.last_save >= self.p('save_every_wall_seconds',60.):
                self.save()
                self.last_save = began
            # Wall time keeps diagnostics live when /clock pauses.
            time.sleep(max(.01,1./self.p('planning_hz',2.)-(time.monotonic()-began)))


if __name__ == '__main__':
    rospy.init_node('warehouse_explorer')
    Explorer().run()
