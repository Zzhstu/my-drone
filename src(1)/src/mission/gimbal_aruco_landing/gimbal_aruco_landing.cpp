#include <ros/ros.h>
#include <ros/time.h>
#include <sstream>
#include <Eigen/Eigen>
#include <iostream>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVControlState.h>
#include <prometheus_msgs/TextInfo.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <spirecv_msgs/GimbalState.h>
#include <spirecv_msgs/Control.h>
#include <mission_utils.h>
#include "printf_utils.h"
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <std_srvs/SetBool.h>
#include <queue>
#include <algorithm>

using namespace std;
using namespace Eigen;
using namespace cv;
#define NODE_NAME "gimbal_aruco_landing"
prometheus_msgs::UAVState g_UAVState;
Eigen::Vector3f g_drone_pos;
float height;
prometheus_msgs::UAVControlState g_uavcontrol_state;
spirecv_msgs::Target g_Detection_raw;      //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
spirecv_msgs::TargetsInFrame det;
prometheus_msgs::UAVCommand g_command_now; //发送给控制模块的命令
int g_uav_id;

float kpx_track, kpy_track, kpz_track; //控制参数 - 比例参数
bool is_detected = false;              // 是否检测到目标标志
float gimbal_roll, gimbal_pitch, gimbal_yaw; //吊舱RYP参数
spirecv_msgs::GimbalState g_GimbalState;

int tracked_id = -1;
float lost_time = 0; // 秒
// 记录时间戳
ros::Time last_time;

std::queue<float> last_x_vel; // 存储在跟踪状态下无人机的移动数据
std::queue<float> last_y_vel;
std::queue<float> last_z_vel;
std::queue<float> last_yaw_rate; // 存储在跟踪状态下无人机的偏航数据

bool  land_Pxyz_flag              = false ;
float land_coordinate_origin_x    = 0.0;
float land_coordinate_origin_y    = 0.0;
float land_coordinate_origin_z    = 0.0;
float land_rel_alt_origin_z       = 0.0;
float land_rel_alt_det_z          = 0.0;
bool  land_coordinate_origin_init = false;
float yaw_tracking_enu            = 0;   //rad
bool  Cx_Cy_init                  = false;
bool  agent                       = true;


void droneStateCb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    g_UAVState = *msg;
    g_drone_pos[0] = g_UAVState.position[0];
    g_drone_pos[1] = g_UAVState.position[1];
    g_drone_pos[2] = g_UAVState.position[2];
    if(g_UAVState.location_source == prometheus_msgs::UAVState::RTK || g_UAVState.location_source == prometheus_msgs::UAVState::GPS)
    	height = g_UAVState.rel_alt;
    else
    	height = g_UAVState.position[2];
}

void VisionCb(const spirecv_msgs::TargetsInFrame::ConstPtr &msg)
{
    det = *msg;
    // car_id = det.frame_id;
    g_Detection_raw.mode = false;
    bool is_update = false;
    for(auto &tar : msg->targets)
    {
        if(!tar.mode && !(tracked_id != -1 && tracked_id == tar.tracked_id)){
                continue;
        }
        g_Detection_raw = tar;
        tracked_id = g_Detection_raw.tracked_id;
        last_time = ros::Time::now();
        is_update = true;
        lost_time = 0.0;
        break;
    }
    if(!is_update && tracked_id != -1)
    {
    	ros::Time now_time = ros::Time::now();
        lost_time = (now_time - last_time).toSec();
    }
}

void GimbalStateCb(const spirecv_msgs::GimbalState::ConstPtr &msg)
{
    g_GimbalState = *msg;
    gimbal_roll = g_GimbalState.angleRT[0];
    gimbal_pitch = g_GimbalState.angleRT[1];
    gimbal_yaw = g_GimbalState.angleRT[2];
}

void droneControlStateCb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    g_uavcontrol_state = *msg;
}

inline float clamp(float value, float max)
{
    max = std::abs(max);
    return std::fmin(std::fmax(value,-max),max);
}

void pop_back(std::queue<float>& last_queue)
{
    int size = last_queue.size();
    std::queue<float> new_queue;
    if(size < 1) return;
    for(int i = 0; i < size - 1; i++)
    {
        float num = last_queue.front();
        new_queue.push(num);
        last_queue.pop();
    }
    std::swap(last_queue,new_queue);
}

void clear(std::queue<float>& last_queue)
{
    std::queue<float> emptyQueue;
    std::swap(last_queue,emptyQueue);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,NODE_NAME);
    ros::NodeHandle nh;
    
    float kp_x, kp_z, kp_gimbal, max_velocity, max_yaw_rate, expect_height, ignore_error_pitch, land_height,
        max_lost_time, min_lost_time , static_vel_thresh , gimbal_offset, ignore_error_yaw, threshold_distance;
    nh.param<float>(ros::this_node::getName() + "/kp_x", kp_x, 100);
    nh.param<float>(ros::this_node::getName() + "/kp_z", kp_z, 0.005);
    nh.param<float>(ros::this_node::getName() + "/kp_gimbal", kp_gimbal, 0.01);
    nh.param<float>(ros::this_node::getName() + "/max_velocity", max_velocity, 1);
    nh.param<float>(ros::this_node::getName() + "/max_yaw_rate", max_yaw_rate, 10);
    nh.param<float>(ros::this_node::getName() + "/ignore_error_pitch", ignore_error_pitch, 2);
    nh.param<float>(ros::this_node::getName() + "/ignore_error_yaw", ignore_error_yaw, 30);
    nh.param<float>(ros::this_node::getName() + "/land_height", land_height, 0.35);
    nh.param<float>(ros::this_node::getName() + "/max_lost_time", max_lost_time, 30.0);
    nh.param<float>(ros::this_node::getName() + "/min_lost_time", min_lost_time, 1.0);
    nh.param<float>(ros::this_node::getName() + "/static_vel_thresh", static_vel_thresh, 0.05);
    nh.param<float>(ros::this_node::getName() + "/gimbal_offset", gimbal_offset, 0.1);
    nh.param<float>(ros::this_node::getName() + "/threshold_distance", threshold_distance, 0.35);
    nh.param<int>(ros::this_node::getName() + "/uav_id", g_uav_id, 1);
    //获取无人机ENU下位置
    ros::Subscriber curr_pos_sub = nh.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(g_uav_id) + "/prometheus/state", 10, droneStateCb);
    //获取遥控器控制状态
    ros::Subscriber uav_control_state_sub = nh.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(g_uav_id) + "/prometheus/control_state", 10, droneControlStateCb);
    //获取吊舱状态
    ros::Subscriber gimbal_state_sub = nh.subscribe<spirecv_msgs::GimbalState>("/uav" + std::to_string(g_uav_id) + "/gimbal/state", 10, GimbalStateCb);
    //获取视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(g_uav_id) + "/spirecv/target", 10, VisionCb);
    //发布控制指令到uav_controller
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(g_uav_id) + "/prometheus/command", 10);
    //发布服务让吊舱回中
    ros::ServiceClient gimbal_home_client_ = nh.serviceClient<std_srvs::SetBool>("/uav" + std::to_string(g_uav_id) + "/gimbal_server");
    //发布画面点击
    ros::Publisher target_pub = nh.advertise<spirecv_msgs::Control>("/uav" + std::to_string(g_uav_id) + "/spirecv/control", 1);
    //锁定吊舱角度
    ros::ServiceClient gimbal_downward_lock_client_ = nh.serviceClient<std_srvs::SetBool>("/uav" + std::to_string(g_uav_id) + "/gimbal_downward_lock");
    //反馈给地面站消息
    ros::Publisher text_info_pub = nh.advertise<prometheus_msgs::TextInfo>("/uav" + std::to_string(g_uav_id) + "/prometheus/text_info", 1);

    //1.kalman filter setup
    const int stateNum=6;                                      //状态值6×1向量(x,y,z,△x,△y,△z)
    const int measureNum=3;                                    //测量值3×1向量(x,y,z)	
    KalmanFilter KF(stateNum, measureNum, 0);	
    KF.transitionMatrix = (cv::Mat_<float>(6, 6) <<  
        1, 0, 0, 1, 0, 0,  
        0, 1, 0, 0, 1, 0,  
        0, 0, 1, 0, 0, 1,  
        0, 0, 0, 1, 0, 0,  
        0, 0, 0, 0, 1, 0,  
        0, 0, 0, 0, 0, 1); // 转移矩阵  
    setIdentity(KF.measurementMatrix);                                             //测量矩阵H
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));                            //系统噪声方差矩阵Q
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));                        //测量噪声方差矩阵R
    setIdentity(KF.errorCovPost, Scalar::all(1));                                  //后验错误估计协方差矩阵P
    //rng.fill(KF.statePost,RNG::UNIFORM,0,winHeight>winWidth?winWidth:winHeight);   //初始状态值x(0)
    Mat statenum = Mat::zeros(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);                           //初始测量值x'(0)，因为后面要更新这个值，所以必须先定义   

    double distance = 0;// 估计距离
    float gimbal_height = 0.1; //吊舱高度
    //height = 1;
    
    std::string last_message = "";
    std::string message = "Gimbal Aruco Landing Start!!!";
    uint8_t message_type = prometheus_msgs::TextInfo::INFO;

    // 程序执行状态
    enum State{
        INIT = 0,
        TRACKING = 1,
        LANDING = 2
    };

    // 丢失状态
    enum LostState{
        NOT_LOST = 0,//未丢失
        LOST_FIND = 1,//丢失查找
        COMPLETELY_LOST = 2//完全丢失
    };

    State current_state = State::INIT;
    LostState lost_state = LostState::NOT_LOST;

    float tracking_height = 0.0;

    int hz = 20;
    ros::Rate rate(hz);
    while(ros::ok())
    {
        ros::spinOnce();

        if(g_uavcontrol_state.control_state != prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
            PCOUT(-1, WHITE, "Waiting for enter COMMAND_CONTROL state");
            expect_height = height;
        }else
        {
            float x_vel = 0.0;
            float y_vel = 0.0;
            float z_vel = 0.0;
            float yaw_rate = 0.0;
            if (land_coordinate_origin_init)
            {
                g_command_now.header.frame_id = "ENU";
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                // g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL;
                g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                g_command_now.yaw_ref = g_UAVState.attitude[2];
                g_command_now.Yaw_Rate_Mode = false;
            }else
            {
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                g_command_now.Move_mode = prometheus_msgs::UAVCommand::XYZ_VEL_BODY;
                g_command_now.Yaw_Rate_Mode = true;
            }
            
            


            if(current_state == State::INIT)
            {
                // 判断是否已经点击目标
                if(tracked_id != -1 && gimbal_pitch > 0.0 && g_GimbalState.moveMode == 3)
                {
                    current_state = State::TRACKING;
                }else
                {
                    PCOUT(-1, GREEN, "Waiting for click target && gimbal track");
                }
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                g_command_now.Yaw_Rate_Mode = false;
            }else if(current_state == State::TRACKING)
            {
                // 处理跟踪过程中的处理逻辑
                
                //2.kalman prediction
                Mat prediction = KF.predict();
                Point3f predict_pt = Point3f(prediction.at<float>(0),prediction.at<float>(1),prediction.at<float>(2) );   //预测值(x',y',z')
                //3.update Position
                printf("pitch=%f \n",gimbal_pitch);
                printf("yaw=%f \n",gimbal_yaw);

                // 吊舱具体高度做纠偏
                gimbal_height = height + gimbal_offset;
                
                float measurementX = gimbal_height * tan((90-gimbal_pitch)*M_PI / 180.0) * sin(gimbal_yaw*M_PI / 180.0);
                float measurementY = gimbal_height;
                float measurementZ = gimbal_height * tan((90-gimbal_pitch)*M_PI / 180.0) * cos(gimbal_yaw*M_PI / 180.0);
                measurement.at<float>(0) = measurementX;
                measurement.at<float>(1) = measurementY;
                measurement.at<float>(2) = measurementZ;
                //4.update
                KF.correct(measurement);
                double last_distance = distance;
                distance = sqrt(KF.statePost.at<float>(0)*KF.statePost.at<float>(0)+KF.statePost.at<float>(1)*KF.statePost.at<float>(1)+KF.statePost.at<float>(2)*KF.statePost.at<float>(2));
                
                // 如果前后两帧差距过大咋认为其还在数据收敛阶段
                if(std::abs(distance - last_distance) > 5)
                {
                    rate.sleep();
                    continue;
                }

                printf("kalman Position = (x, y, z) = (%.3f, %.3f, %.3f)\n",KF.statePost.at<float>(0),KF.statePost.at<float>(1),KF.statePost.at<float>(2));   //修正后值(x'',y'',z'')

                // 吊舱的yaw控制SDK左右相反
                if(g_GimbalState.type == 0)
                {
                    // G1 yaw control
                    yaw_rate = -kp_gimbal * gimbal_yaw;
                }
                else if(g_GimbalState.type == 3)
                {
                    // GX40 yaw control
                    yaw_rate = kp_gimbal * gimbal_yaw;
                }else
                {
                    PCOUT(-1, GREEN, "gimbal type error, cann't control gimbal yaw!");
                }

		        static bool is_spirecv_cmd = false;

                // 判断是否丢失目标
                if(lost_time < min_lost_time)
                {
                    // 如果上一帧处于丢失目标的状态，则将队列清空
                    if(lost_state != LostState::NOT_LOST)
                    {
                        clear(last_x_vel);
                        clear(last_y_vel);
                        clear(last_yaw_rate);
                    }

                    float threshold = (0.8 / height) < threshold_distance ? threshold_distance : (0.8 / height);
                    if(std::abs(KF.statePost.at<float>(2)) < threshold && (std::abs(gimbal_yaw) < ignore_error_yaw || g_GimbalState.type == 0))
                    {
                        g_command_now.Yaw_Rate_Mode = false;
                        yaw_rate = 0.0;
                        y_vel = kp_x * -KF.statePost.at<float>(0);
                    }

                    // 控制无人机前进
                    float ref_x_vel = kp_x * KF.statePost.at<float>(2) * ((90.0 - gimbal_pitch)/90.0);
                    x_vel = ref_x_vel < 0.1?0.1:ref_x_vel;
                    
                    // 只需要存储当前循环的频率乘以最少的丢失时间
                    if(last_x_vel.size() == min_lost_time * hz)
                    {
                        last_x_vel.pop();
                        last_y_vel.pop();
                        last_yaw_rate.pop();
                    }
                    // 存储
                    last_x_vel.push(x_vel);
                    last_y_vel.push(y_vel);
                    last_yaw_rate.push(yaw_rate);
                    
                    // 改变丢失状态
                    lost_state == LostState::NOT_LOST;

                    // 判断是否能进入降落状态，  判断条件  吊舱的pitch角 或者 估计的平面距离(因为yaw角可能对pitch角存在误差)
                    if(std::abs(90 - gimbal_pitch) < ignore_error_pitch * (height > 1.0?height:1.0) || sqrt(KF.statePost.at<float>(0)*KF.statePost.at<float>(0) + KF.statePost.at<float>(2)*KF.statePost.at<float>(2)) < 0.2)
                    {
                        // 如果可以进入则先保持悬停
                        g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                        g_command_now.Yaw_Rate_Mode = false;
                        
                        // 取消跟踪进入锁头模式
                        if(g_GimbalState.moveMode == 3)
                        {
                            spirecv_msgs::Control cmd;
                            cmd.mouse = spirecv_msgs::Control::MOUSE_RIGHT;
                            cmd.x = -1;
                            cmd.y = -1;
                            target_pub.publish(cmd);
                            PCOUT(0, GREEN, "cancel tracking!!!");
                            message = "cancel tracking!!!";
                            message_type = prometheus_msgs::TextInfo::INFO;
                            is_spirecv_cmd = true;
                        }else{ //进入锁头模式后 进入降落状态
                            if(g_GimbalState.type == 0)// G1吊舱可以进入俯拍模式
                            {
                                // 判断角度是否进入俯拍，没有进入则进入
                                if(std::abs(gimbal_yaw) > 1.0 || std::abs(90 - gimbal_pitch) > 1.0)
                                {
                                    std_srvs::SetBool lock;
                                    lock.request.data = true;
                                    gimbal_downward_lock_client_.call(lock);

                                    //gimbal_downward_lock
                                    PCOUT(0, GREEN, "g1 gimbal downward lock!!!");
                                    message = "g1 gimbal downward lock!!!";
                                    message_type = prometheus_msgs::TextInfo::INFO;
                                }else
                                {
                                    // 进入后 进入降落状态
                                    current_state = State::LANDING;
                                    is_spirecv_cmd = false;
                                    // 清空队列
                                    clear(last_x_vel);
                                    clear(last_y_vel);
                                    clear(last_yaw_rate);

                                    PCOUT(0, GREEN, "g1 gimbal lock head!!!");
                                    message = "g1 gimbal lock head!!!";
                                    message_type = prometheus_msgs::TextInfo::INFO;
                                }
                            }else// gx40直接进入降落状态
                            {
                                
                                // 进入降落状态
                                current_state = State::LANDING;
                                is_spirecv_cmd = false;
                                // 清空队列
                                clear(last_x_vel);
                                clear(last_y_vel);
                                clear(last_yaw_rate);
                            }

                            tracking_height = height;
                        }
                    }
                }else if(lost_time < 2 * min_lost_time)
                {
                    // 丢失后，根据存储的数据反向控制
                    if(!last_x_vel.empty())
                    {
                        x_vel = -last_x_vel.back();
                        y_vel = -last_y_vel.back();
                        yaw_rate = -last_yaw_rate.back();
                    
                        // 抛出最后一个数据
                        pop_back(last_x_vel);
                        pop_back(last_y_vel);
                        pop_back(last_yaw_rate);
                    }
                    
                    // 改变丢失状态
                    lost_state == LostState::LOST_FIND;
                }else
                {
                    // 如果丢失时间大于最大丢失时间，触发降落 或者 进行其他操作
                    if(lost_time > max_lost_time)
                    {
                        g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                        PCOUT(0, RED, "No target found, execute landing!!!");
                        message = "No target found, execute landing!!!";
                        message_type = prometheus_msgs::TextInfo::WARN;
                    }else
                    {
                        // 保持悬停
                        g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                        g_command_now.Yaw_Rate_Mode = false;
                        PCOUT(0, YELLOW, "Drone raises to search for targets!!!");
                        message = "Drone raises to search for targets!!!";
                        message_type = prometheus_msgs::TextInfo::WARN;
                    }
                    // 完全丢失
                    lost_state == LostState::COMPLETELY_LOST;
                }

                // 如果处于跟踪状态 但是吊舱不在跟踪状态 说明 用户取消跟踪
                if(current_state == State::TRACKING && g_GimbalState.moveMode != 3 && !is_spirecv_cmd)
                {
                    // 初始化
                    current_state = State::INIT;
                    lost_state = LostState::NOT_LOST;
                    tracked_id = -1;
                    lost_time = 0.0;
                    // 清空队列
                    clear(last_x_vel);
                    clear(last_yaw_rate);
                    // 悬停
                    g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                    g_command_now.Yaw_Rate_Mode = false;
                }
                
            }else if(current_state == State::LANDING)
            {
                // 根据目标ID，查到对应检测的目标的 cx 和 cy 控制无人机 细微调整位置
                // 计算吊舱的弧度
                float gimbal_yaw_radian = M_PI / 180.0 * gimbal_yaw;
                // 转换画面坐标原点 画面中心为原点的坐标系
                float new_cx = g_Detection_raw.cx - 0.5;
                float new_cy = g_Detection_raw.cy - 0.5;
                // 计算将吊舱转回0度时的cx，cy位置
                float new_cx_rotated = new_cx * std::cos(-gimbal_yaw_radian) - new_cy * std::sin(-gimbal_yaw_radian);
                float new_cy_rotated = new_cx * std::sin(-gimbal_yaw_radian) + new_cy * std::cos(-gimbal_yaw_radian);
                // 转换回原始坐标系
                float cx = new_cx_rotated + 0.5;
                float cy = new_cy_rotated + 0.5;
                // 判断旋转后的数据是否越界
                if(cx < 0 || cy < 0 || cx > 1 || cy > 1)
                {
                    PCOUT(0, RED, "Coordinate Transform Error!!!");
                    message = "Coordinate Transform Error!!!";
                    message_type = prometheus_msgs::TextInfo::ERROR;
                    break;  
                }
                // 计算速度，使目标在画面中间  根据高度一定程度加快速度
                x_vel = kp_x * (0.5 - cy)  * (3 - (tracking_height - height)/(tracking_height - land_height));
                y_vel = kp_x * (0.5 - cx)  * (3 - (tracking_height - height)/(tracking_height - land_height));

                // 不同高度不同的静止速度，防止降落过程中出现丢失目标的情况
                if(std::abs(x_vel) + std::abs(y_vel) < static_vel_thresh * (3 - (tracking_height - height)/(tracking_height - land_height)))
                {
                    Cx_Cy_init = true;
                }
                if(Cx_Cy_init)
                {
                    //在水平对齐后悬停，记录目标二维码位置建立降落模型
                    // 控制降落
                    if(!land_Pxyz_flag)
                    {
                        // 控制Z轴速度下降
                        // z_vel = kp_z * (land_height - height);
                        // 悬停 
                        // 循环判断
                        int count_number_time = hz;// 1 s
                        while((count_number_time--) > 0)
                        {
                            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                            g_command_now.header.stamp = ros::Time::now();
                            g_command_now.Command_ID = g_command_now.Command_ID + 1;
                            command_pub.publish(g_command_now);
                            ros::spinOnce();
                            rate.sleep();
                        }
                        land_Pxyz_flag = true ;
                        land_rel_alt_origin_z = g_Detection_raw.pz ;
                        land_rel_alt_det_z = g_UAVState.position[2] - land_rel_alt_origin_z ;
                    }
                    if(g_command_now.Agent_CMD == prometheus_msgs::UAVCommand::Current_Pos_Hover && !land_coordinate_origin_init)
                    {
                        
                        yaw_tracking_enu = g_UAVState.attitude[2];//rad
                        Eigen::Vector3d v_body(-g_Detection_raw.py + 0.15, -g_Detection_raw.px,-g_Detection_raw.pz );//因为机体系下吊舱向下安装
                        // 计算旋转矩阵
                        double cr = cos(g_UAVState.attitude[0]);
                        double sr = sin(g_UAVState.attitude[0]);
                        double cp = cos(g_UAVState.attitude[1]);
                        double sp = sin(g_UAVState.attitude[1]);
                        double cy = cos(g_UAVState.attitude[2]);
                        double sy = sin(g_UAVState.attitude[2]);

                        Eigen::Matrix3d R;
                        R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
                            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
                            -sp, cp * sr, cp * cr;

                        // 转换到ENU坐标系
                        Eigen::Vector3d v_enu = R * v_body;
                        land_coordinate_origin_x = g_UAVState.position[0] + v_enu.x();
                        land_coordinate_origin_y = g_UAVState.position[1] + v_enu.y();
                        land_coordinate_origin_z = g_UAVState.position[2] + v_enu.z();
                        land_coordinate_origin_init = true ;

                    }
                    if (land_coordinate_origin_init)
                    {
                        g_command_now.Agent_CMD == prometheus_msgs::UAVCommand::Move ;
                        g_command_now.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS;
                        g_command_now.position_ref[0] = land_coordinate_origin_x;
                        g_command_now.position_ref[1] = land_coordinate_origin_y;
                        g_command_now.position_ref[2] = land_rel_alt_det_z - 0.2;
                    }
                }   
            }

            if(g_command_now.Agent_CMD == prometheus_msgs::UAVCommand::Move && g_command_now.Move_mode == prometheus_msgs::UAVCommand::XYZ_VEL_BODY)
            {
                g_command_now.velocity_ref[0] = clamp(x_vel, max_velocity);
                g_command_now.velocity_ref[1] = clamp(y_vel, max_velocity);
                g_command_now.velocity_ref[2] = clamp(z_vel, max_velocity);
                g_command_now.yaw_rate_ref = clamp(yaw_rate, max_yaw_rate) * M_PI / 180;
                printf("x_vel_ref = %f [m/s] \n",g_command_now.velocity_ref[0]);
                printf("y_vel_ref = %f [m/s] \n",g_command_now.velocity_ref[1]);
                printf("z_vel_ref = %f [m/s] \n",g_command_now.velocity_ref[2]);
                printf("yaw_rate_ref = %f [ang/s] \n",g_command_now.yaw_rate_ref);
            }
            if (g_command_now.Agent_CMD == prometheus_msgs::UAVCommand::Move && g_command_now.Move_mode == prometheus_msgs::UAVCommand::XYZ_POS)
            {
                g_command_now.position_ref[0] = land_coordinate_origin_x;
                g_command_now.position_ref[1] = land_coordinate_origin_y;
                g_command_now.position_ref[2] = land_rel_alt_det_z - 0.2;
                g_command_now.yaw_ref = yaw_tracking_enu;
                g_command_now.Yaw_Rate_Mode = false;
                // printf("ENU land_x_vel = %f [m/s] \n", g_command_now.velocity_ref[0]);
                // printf("ENU land_y_vel = %f [m/s] \n", g_command_now.velocity_ref[1]);
                // printf("ENU land_z_vel = %f [m/s] \n", g_command_now.velocity_ref[2]);
                printf("ENU land_XYZ = %f %f %f  \n", g_command_now.position_ref[0], g_command_now.position_ref[1], g_command_now.position_ref[2]);
            }
            else
            {
                printf("ENU XYZ_VEL Failed \n");
            }
            if (abs(g_UAVState.position[2] - land_rel_alt_det_z) < 0.22)
            { // 控制无人机降落到0.22时直接触发降落
                g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Land;
                PCOUT(-1, GREEN, "Find the target and successfully land!!!");
                message = "Find the target and start land!!!";
                message_type = prometheus_msgs::TextInfo::INFO;
            }else if(abs(g_UAVState.position[2] - land_rel_alt_det_z)< 1 && agent)
	    {
	        Cx_Cy_init = false;
	        land_Pxyz_flag = false;
	        agent = false;
	        land_coordinate_origin_init = false;
	        printf("agent-------------------------------- \n");
	    }

        }

        // Publish
        g_command_now.header.stamp = ros::Time::now();
        g_command_now.Command_ID = g_command_now.Command_ID + 1;
        if (g_command_now.Command_ID < 10)
            g_command_now.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
        command_pub.publish(g_command_now);
        
        // 发布消息反馈
        if(last_message != message && message != "")
        {
            prometheus_msgs::TextInfo text_info;
            text_info.header.stamp = ros::Time::now();
            text_info.Message = message;
            text_info.MessageType = message_type;
            text_info_pub.publish(text_info);
            last_message = message;
        }

        // 判断是否触发了降落
        bool is_land = g_command_now.Agent_CMD == prometheus_msgs::UAVCommand::Land || g_uavcontrol_state.control_state == prometheus_msgs::UAVControlState::LAND_CONTROL;
        // 控制降落 
        while (is_land)
        {
            // 如果无人机上锁则退出该循环
            if(!g_UAVState.armed)
            {
                // 吊舱是否需要归中
                std_srvs::SetBool set_home;
                set_home.request.data       = true;
                gimbal_home_client_.call(set_home);
                // 初始化
                tracked_id                  = -1;
                current_state               = State::INIT;
                lost_state                  = LostState::NOT_LOST;
                lost_time                   = 0.0;
                land_Pxyz_flag              = false ;
                land_coordinate_origin_x    = 0.0;
                land_coordinate_origin_y    = 0.0;
                land_coordinate_origin_z    = 0.0;
                land_rel_alt_origin_z       = 0.0;
                land_rel_alt_det_z          = 0.0;
                land_coordinate_origin_init = false;
                yaw_tracking_enu            = 0;   //rad
                Cx_Cy_init                  = false;
                agent                       = true;

                // 清空队列
                clear(last_x_vel);
                clear(last_y_vel);
                clear(last_z_vel); 
                clear(last_yaw_rate); 

                g_command_now.Agent_CMD     = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                g_command_now.Yaw_Rate_Mode = false;
                // 退出降落循环
                break;
            }
            //command_pub.publish(g_command_now);
            rate.sleep();
            ros::spinOnce();
        }

        rate.sleep();
    }
    return 0;
}

