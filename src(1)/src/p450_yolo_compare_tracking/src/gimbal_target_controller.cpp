#include <cmath>
#include <algorithm>
#include <ros/ros.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVControlState.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/GimbalState.h>

static prometheus_msgs::UAVState uav_state;
static prometheus_msgs::UAVControlState control_state;
static spirecv_msgs::GimbalState gimbal_state;
static spirecv_msgs::Target target;
static bool has_uav=false, has_control=false, has_gimbal=false, has_target=false;
static int no_track_count=999;
static uint32_t cmd_id=0;

static double clamp_abs(double v, double m){m=std::fabs(m); return std::min(std::max(v,-m),m);} 
static double deg2rad(double d){return d*M_PI/180.0;}

static void uavCb(const prometheus_msgs::UAVState::ConstPtr& msg){uav_state=*msg; has_uav=true;}
static void ctrlCb(const prometheus_msgs::UAVControlState::ConstPtr& msg){control_state=*msg; has_control=true;}
static void gimbalCb(const spirecv_msgs::GimbalState::ConstPtr& msg){gimbal_state=*msg; has_gimbal=true;}
static void targetCb(const spirecv_msgs::TargetsInFrame::ConstPtr& msg){
  no_track_count=msg->no_track_frame_count; has_target=false;
  for(const auto& t: msg->targets){ if(t.mode){ target=t; has_target=true; return; } }
  if(!msg->targets.empty()) target=msg->targets[0];
}
static double heightNow(){
  if(!has_uav) return 0.0;
  if(uav_state.location_source==prometheus_msgs::UAVState::RTK || uav_state.location_source==prometheus_msgs::UAVState::GPS) return uav_state.rel_alt;
  return uav_state.position[2];
}

int main(int argc,char** argv){
  ros::init(argc,argv,"gimbal_target_controller");
  ros::NodeHandle nh, pnh("~");
  int uav_id=1, lost_threshold=20;
  double kp_x=0.25,kp_z=0.1,kp_gimbal=1.0,max_velocity=1.0,max_yaw_rate=30.0,tracking_distance=10.0;
  double distance_deadband=0.5, near_distance_band=2.0, yaw_deadband=10.0, min_pitch=1.0, yaw_bias=-3.0, gimbal_height_offset=0.13, expected_height=-1.0;
  bool enable_height_hold=false, require_command_control=true, require_gimbal_tracking=true, dry_run=false;
  pnh.param("uav_id",uav_id,uav_id); pnh.param("kp_x",kp_x,kp_x); pnh.param("kp_z",kp_z,kp_z); pnh.param("kp_gimbal",kp_gimbal,kp_gimbal);
  pnh.param("max_velocity",max_velocity,max_velocity); pnh.param("max_yaw_rate",max_yaw_rate,max_yaw_rate); pnh.param("tracking_distance",tracking_distance,tracking_distance);
  pnh.param("distance_deadband",distance_deadband,distance_deadband); pnh.param("near_distance_band",near_distance_band,near_distance_band); pnh.param("yaw_deadband",yaw_deadband,yaw_deadband);
  pnh.param("min_pitch",min_pitch,min_pitch); pnh.param("gimbal_yaw_bias",yaw_bias,yaw_bias); pnh.param("gimbal_height_offset",gimbal_height_offset,gimbal_height_offset);
  pnh.param("expected_height",expected_height,expected_height); pnh.param("enable_height_hold",enable_height_hold,enable_height_hold);
  pnh.param("require_command_control",require_command_control,require_command_control); pnh.param("require_gimbal_tracking",require_gimbal_tracking,require_gimbal_tracking);
  pnh.param("dry_run",dry_run,dry_run); pnh.param("lost_threshold",lost_threshold,lost_threshold);
  std::string base="/uav"+std::to_string(uav_id);
  std::string target_topic=base+"/spirecv/target", state_topic=base+"/prometheus/state", ctrl_topic=base+"/prometheus/control_state", gimbal_topic=base+"/gimbal/state", cmd_topic=base+"/prometheus/command";
  pnh.param("target_topic",target_topic,target_topic); pnh.param("state_topic",state_topic,state_topic); pnh.param("control_state_topic",ctrl_topic,ctrl_topic); pnh.param("gimbal_state_topic",gimbal_topic,gimbal_topic); pnh.param("command_topic",cmd_topic,cmd_topic);
  ros::Subscriber s1=nh.subscribe(state_topic,10,uavCb), s2=nh.subscribe(ctrl_topic,10,ctrlCb), s3=nh.subscribe(gimbal_topic,10,gimbalCb), s4=nh.subscribe(target_topic,10,targetCb);
  ros::Publisher pub=nh.advertise<prometheus_msgs::UAVCommand>(cmd_topic,10);
  ROS_INFO("gimbal_target_controller target=%s command=%s dry_run=%d", target_topic.c_str(), cmd_topic.c_str(), dry_run);
  ROS_WARN("Real flight safety: keep RC takeover ready; first run dry_run:=true, then low max_velocity.");
  ros::Rate rate(50);
  while(ros::ok()){
    ros::spinOnce();
    prometheus_msgs::UAVCommand cmd; cmd.header.stamp=ros::Time::now(); cmd.Command_ID=++cmd_id; cmd.Agent_CMD=prometheus_msgs::UAVCommand::Current_Pos_Hover;
    bool safe=true;
    if(require_command_control) safe = safe && has_control && control_state.control_state==prometheus_msgs::UAVControlState::COMMAND_CONTROL;
    safe = safe && has_uav && has_gimbal && has_target && target.mode && no_track_count<=lost_threshold;
    if(require_gimbal_tracking) safe = safe && gimbal_state.moveMode==3;
    double h=heightNow(), gh=h+gimbal_height_offset, pitch=gimbal_state.angleRT[1], yaw=gimbal_state.angleRT[2]+yaw_bias;
    if(safe && pitch>min_pitch && gh>0.0){
      double dist=gh*std::tan(deg2rad(90.0-pitch)); if(!std::isfinite(dist)||dist<0) dist=0;
      double err=dist-tracking_distance, vx=0, vz=0, yr=0; bool yr_mode=false;
      if(std::fabs(err)>distance_deadband && (std::fabs(err)>near_distance_band || std::fabs(yaw)<yaw_deadband)) vx=kp_x*err;
      if(enable_height_hold){ if(expected_height<0) expected_height=h; vz=kp_z*(expected_height-h); }
      if(std::fabs(yaw)>yaw_deadband){ yr_mode=true; yr=(gimbal_state.type==3?kp_gimbal*yaw:-kp_gimbal*yaw); }
      cmd.Agent_CMD=prometheus_msgs::UAVCommand::Move; cmd.Move_mode=prometheus_msgs::UAVCommand::XYZ_VEL_BODY; cmd.Yaw_Rate_Mode=yr_mode;
      cmd.velocity_ref[0]=clamp_abs(vx,max_velocity); cmd.velocity_ref[1]=0; cmd.velocity_ref[2]=clamp_abs(vz,max_velocity); cmd.yaw_rate_ref=clamp_abs(yr,max_yaw_rate)*M_PI/180.0;
      ROS_INFO_THROTTLE(0.5,"Track dist=%.2f err=%.2f yaw=%.2f pitch=%.2f vx=%.2f yr=%.2f",dist,err,yaw,pitch,cmd.velocity_ref[0],yr);
    }else{
      ROS_INFO_THROTTLE(1.0,"Hover safe=%d has_target=%d mode=%d no_track=%d pitch=%.2f h=%.2f",(int)safe,(int)has_target,(int)target.mode,no_track_count,pitch,gh);
    }
    if(cmd_id<10) cmd.Agent_CMD=prometheus_msgs::UAVCommand::Init_Pos_Hover;
    if(!dry_run) pub.publish(cmd);
    rate.sleep();
  }
  return 0;
}
