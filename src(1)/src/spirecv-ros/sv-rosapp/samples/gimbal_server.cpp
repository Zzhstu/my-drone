/********************
* 吊舱控制服务端
* 订阅：1、uav1/gimbal/control
*       2、/uav1/spirecv/detection_with_tracking
* 发布：1、uav1/gimbal/state
*       2、uav1/gimbal/image_raw
*       3、uav1/gimbal/camera_info
*/
// ros
#include <ros/ros.h>
// sv core
#include <sv_world.h>
// spirecv_msgs
#include <spirecv_msgs/GimbalControl.h>
#include <spirecv_msgs/GimbalState.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/AttitudeCorrection.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

// spirecv_services
#include <spirecv_srvs/CameraService.h>
#include <spirecv_srvs/GimbalSetHome.h>
#include <std_srvs/SetBool.h>


#include <iostream>
#include <map>
#include <iterator>

void downward_locked_angle();
double gimbalEulerAngle[6];
double gimbalFov[2];

sv::Gimbal *gimbal;
spirecv_msgs::GimbalState gimbalState;
int car_id;
spirecv_msgs::TargetsInFrame det;
spirecv_msgs::Target g_Detection_raw;      //目标位置[机体系下：前方x为正，右方y为正，下方z为正]
cv::Mat img;
bool downward_locked_angle_mark = false;
std::string camera_type = "SU17";
spirecv_msgs::AttitudeCorrection attitude_correction_message;



// gimbal state callback
void updateState(double &frameAngleRoll, double &frameAnglePitch, double &frameAngleYaw,
                 double &imuAngleRoll, double &imuAnglePitch, double &imuAngleYaw,
                 double &fovX, double &fovY)
{
    gimbalState.angleRT[0] = imuAngleRoll;
    gimbalState.angleRT[1] = imuAnglePitch;
    gimbalState.angleRT[2] = frameAngleYaw;
    gimbalState.fov[0] = fovX;
    gimbalState.fov[1] = fovY;
    //std::cout << "imu yaw:" << imuAngleYaw << std::endl;
}

void gimableCallbackA(double &frame_ang_r, double &frame_ang_p, double &frame_ang_y,
                     double &imu_ang_r, double &imu_ang_p, double &imu_ang_y,
                     double &fov_x, double &fov_y);
void attitudeCorrectionCallback(const spirecv_msgs::AttitudeCorrection::ConstPtr &msg);


// gimbal track callback
void gimbalTrack(sv::Gimbal *gimbal, double xOffset, double speedx, double yOffset, double speedy)
{
    gimbal->setAngleRateEuler(0, yOffset * speedy, xOffset * speedx);
}

// gimbal no track callback
void gimbalNoTrack(sv::Gimbal *gimbal)
{
    gimbal->setAngleRateEuler(0, 0, 0);
}

typedef struct
{
    sv::GimbalType type;
    sv::GimbalLink link;
    sv::CameraType camera;
} SV_GIMBAL_TYPE_T;

typedef std::map<std::string, SV_GIMBAL_TYPE_T> typeMap;
typeMap svGimbalTypeList =
    {
        {"G1", {sv::GimbalType::G1, sv::GimbalLink::SERIAL, sv::CameraType::G1}},
        {"Q10f", {sv::GimbalType::Q10f, sv::GimbalLink::SERIAL, sv::CameraType::WEBCAM}},
        {"GX40", {sv::GimbalType::GX40, sv::GimbalLink::SERIAL, sv::CameraType::GX40}},
        {"SU17", {sv::GimbalType::SU17, sv::GimbalLink::ETHERNET_UDP2, sv::CameraType::SU17}},
};

SV_GIMBAL_TYPE_T findSVGimbal(const std::string &type)
{
    typeMap::iterator temp = svGimbalTypeList.find(type);

    if (temp != svGimbalTypeList.end())
    {
        return (temp->second);
    }
    std::cout << type << " is Unsupported device type!" << std::endl;
    exit(-1);
}

// spirecv detection with tracking callback
void VisionCb(const spirecv_msgs::TargetsInFrame::ConstPtr &msg)
{
    det = *msg;
    car_id = det.frame_id;
    g_Detection_raw.mode = false;
    for(auto &tar : msg->targets)
    {
        g_Detection_raw = tar;
    }
}

// gimbal control callback
void gimbalControlCallback(const spirecv_msgs::GimbalControl::ConstPtr &msg)
{
    if (gimbalState.moveMode == 3)
    {
        return;
    }

    if (msg->mode == 1)
    {
        gimbal->setAngleRateEuler(msg->speed[0], msg->speed[1], msg->speed[2]);
    }
    else if (msg->mode == 2)
    {
        gimbal->setAngleEuler(msg->angle[0], msg->angle[1], msg->angle[2],
                              00, 00, 00);
        std::cout << "r:" << gimbalState.angleRT[0] << "p:" << gimbalState.angleRT[1] << "y:" << gimbalState.angleRT[2] << std::endl;
    }
    else if (msg->mode == 3)
    {
        gimbal->setAngleEuler(msg->angle[0], msg->angle[1], msg->angle[2],
                              msg->speed[0], msg->speed[1], msg->speed[2]);
    }
}

// gimbal camera server callback
bool cameraServerCallback(spirecv_srvs::CameraService::Request &req, spirecv_srvs::CameraService::Response &res)
{
    res.stateResponse = false;

    switch (req.action)
    {
    case 1:
        res.stateResponse = gimbal->takePhoto();
        break;
    case 2:
        break;
    case 3:
        res.stateResponse = gimbal->takeVideo(1);
        if (gimbal->getVideoState() == 1)
        {
            gimbalState.isRec |= 0X01;
        }
        break;
    case 4:
        break;
    case 5:
        res.stateResponse = gimbal->takeVideo(0);
        if (gimbal->getVideoState() == 1)
        {
            gimbalState.isRec &= (~0X01);
        }
        break;
    case 6:
        break;

    default:
        break;
    }
    return res.stateResponse;
}

bool gimbalTakePhotoServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = false;
    res.success = gimbal->takePhoto();
    std::cout << "TakePhoto: " << (res.success?"success":"faild") << std::endl; 
    return res.success;
}

bool localTakePhotoServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = false;
    return res.success;
}

bool gimbaRecordServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = false;
    if(req.data)
    {
        // start record
        res.success = gimbal->takeVideo(1);
        if (gimbal->getVideoState() == 1)
        {
            gimbalState.isRec |= 0X01;
        }
        std::cout << "Start Record: " << (res.success?"success":"faild") << std::endl; 
    }else
    {   
        // stop record
        res.success = gimbal->takeVideo(0);
        if (gimbal->getVideoState() == 1)
        {
            gimbalState.isRec &= (~0X01);
        }
        std::cout << "Stop Record: " << (res.success?"success":"faild") << std::endl; 
    }
    return res.success;
}

bool localRecordServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = false;
    return res.success;
}

// gimbal track server callback
// bool gimbalServerCallback(spirecv_srvs::GimbalSetHome::Request &req, spirecv_srvs::GimbalSetHome::Response &res)
bool gimbalServerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = false;
    if (gimbalState.moveMode == 3)
    {
        return res.success;
    }

    if (req.data == true)
    {
        if(findSVGimbal(camera_type).camera == sv::CameraType::GX40 || findSVGimbal(camera_type).camera == sv::CameraType::G1)
        {
            res.success = gimbal->setHome();
        }
        else if(findSVGimbal(camera_type).camera == sv::CameraType::SU17)
        {
            gimbal->setAngleEuler(0,0,0);
            res.success = true;
        }
    }
    return res.success;
}

bool gimbalDownwardLockCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    res.success = false;
    if (gimbalState.moveMode == 3)
    {
        return res.success;
    }

    if (req.data == true)
    {
    	downward_locked_angle();
        res.success = true;
    }
    return res.success;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gimbal_server");
    ros::NodeHandle nh("~");

    int uav_id = 1;
    //云台状态
    std::string gimbalStateTopic = "/uav" + std::to_string(uav_id) + "/gimbal/state";
    //云台控制
    std::string gimbalControlTopic = "/uav" + std::to_string(uav_id) + "/gimbal/control";
    //目标识别结果
    std::string gimbalTrackTopic = "/uav" + std::to_string(uav_id) + "/spirecv/target";
    //相机原始图像camera
    std::string imageOriginal = "/uav" + std::to_string(uav_id) + "/gimbal/image_original";
    //相机原始图像camera_info
    std::string camInfo = "/uav" + std::to_string(uav_id) + "/gimbal/camera_info";
    //相机服务
    std::string cameraServerName = "/uav" + std::to_string(uav_id) + "/camera_server";
    //云台SD卡拍照存储服务
    std::string gimbalTakePhotoServerName = "/uav" + std::to_string(uav_id) + "/gimbal_take_server";
    //本地拍照存储服务
    std::string localTakePhotoServerName = "/uav" + std::to_string(uav_id) + "/local_take_server";
    //云台SD卡录制存储服务
    std::string gimbaRecordServerName = "/uav" + std::to_string(uav_id) + "/gimbal_record_server";
    //本地录制存储服务
    std::string localRecordServerName = "/uav" + std::to_string(uav_id) + "/local_record_server";
    //云台服务
    std::string gimbalServerName = "/uav" + std::to_string(uav_id) + "/gimbal_server";
    //俯拍模式
    std::string gimbalDownwardLockName = "/uav" + std::to_string(uav_id) + "/gimbal_downward_lock";

    std::string serialPotr = "/dev/ttyTHS1";
    std::string gimbalType = "SU17";

    std::string cameraName = "192.168.1.99";
    int cameraImageFps = 30;
    int cameraHeight = 720;
    int cameraWidth = 1280;

    nh.getParam("uav_id", uav_id);
    nh.getParam("gimbalStateTopic", gimbalStateTopic);
    nh.getParam("gimbalControlTopic", gimbalControlTopic);
    nh.getParam("gimbakTrackTopic", gimbalTrackTopic);
    nh.getParam("imageOriginal", imageOriginal);
    nh.getParam("camInfo", camInfo);
    nh.getParam("cameraServerName", cameraServerName);
    nh.getParam("gimbalServerName", gimbalServerName);
    nh.getParam("Port", serialPotr);
    nh.getParam("Type", gimbalType);
    nh.getParam("camera", cameraName);
    nh.getParam("FPS", cameraImageFps);
    nh.getParam("H", cameraHeight);
    nh.getParam("W", cameraWidth);

    camera_type = gimbalType;
 
    // out params
    std::cout << "uav_id:" << uav_id << std::endl;
    std::cout << "pub topic:" << std::endl;
    std::cout << gimbalStateTopic << std::endl;
    std::cout << imageOriginal << std::endl;
    std::cout << camInfo << std::endl;
    std::cout << "sub topic:" << std::endl;
    std::cout << gimbalControlTopic << std::endl;
    std::cout << gimbalTrackTopic << std::endl;// todo
    std::cout << "seriver list:" << std::endl;
    std::cout << cameraServerName << std::endl;
    std::cout << gimbalServerName << std::endl;
    std::cout << "gimbal config:" << std::endl;
    std::cout << gimbalType << ";used:" << serialPotr << ";" << std::endl;
    std::cout << cameraName << ";FPS:" << cameraImageFps << ";size:" << cameraWidth << "x" << cameraHeight << std::endl;
    gimbal = new sv::Gimbal(findSVGimbal(gimbalType).type, findSVGimbal(gimbalType).link);
    if (findSVGimbal(gimbalType).link == sv::GimbalLink::SERIAL || findSVGimbal(gimbalType).link == sv::GimbalLink::ETHERNET_UDP2)
    {
        gimbal->setSerialPort(serialPotr);
    }
    else
    {
        std::cout << "didn't open gimbal control port" << std::endl;
        exit(-1);
    }
    gimbal->open(updateState);
    gimbalState.moveMode = 1;
    gimbalState.type = (uint8_t)findSVGimbal(gimbalType).type;
    gimbalState.imageMode = 0;

    sv::Camera cap;
    cap.setFps(cameraImageFps);
    cap.setWH(cameraWidth, cameraHeight);
    
    if (findSVGimbal(gimbalType).camera == sv::CameraType::GX40 || findSVGimbal(gimbalType).camera == sv::CameraType::G1 || findSVGimbal(gimbalType).camera == sv::CameraType::SU17)
    {
        cap.setIp(cameraName);
        cap.open(findSVGimbal(gimbalType).camera);
    }
    else
    {
        cap.open(findSVGimbal(gimbalType).camera, std::stoi(cameraName));
    }

    
    int frame_id = 0;
    sensor_msgs::ImagePtr imageOri;
    sensor_msgs::CameraInfo camera_info;

    image_transport::Publisher imageOriPub;
    // image_transport::Publisher camera_info_pub;
    image_transport::ImageTransport Ori(nh);
    imageOriPub = Ori.advertise(imageOriginal, 100);
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(camInfo,100);

    ros::Publisher gimbal_state_pub =
        nh.advertise<spirecv_msgs::GimbalState>(gimbalStateTopic, 100);

    ros::Subscriber sub =
        nh.subscribe<spirecv_msgs::GimbalControl>(gimbalControlTopic, 100, gimbalControlCallback);
    
    // 获取视觉反馈
    ros::Subscriber vision_sub = nh.subscribe<spirecv_msgs::TargetsInFrame>(gimbalTrackTopic, 100, VisionCb);
    ros::Subscriber attitude_correction_sub = nh.subscribe<spirecv_msgs::AttitudeCorrection>("/uav" + std::to_string(uav_id) + "/spirecv/attitude_correction", 30, attitudeCorrectionCallback);


    ros::ServiceServer camera_server = nh.advertiseService(cameraServerName, cameraServerCallback);
    ros::ServiceServer gimbal_take_server = nh.advertiseService(gimbalTakePhotoServerName, gimbalTakePhotoServerCallback);
    ros::ServiceServer local_take_server  = nh.advertiseService(localTakePhotoServerName, localTakePhotoServerCallback);
    ros::ServiceServer gimbal_record_server = nh.advertiseService(gimbaRecordServerName, gimbaRecordServerCallback);
    ros::ServiceServer local_record_server = nh.advertiseService(localRecordServerName, localRecordServerCallback);
    ros::ServiceServer gimbal_server = nh.advertiseService(gimbalServerName, gimbalServerCallback);
    ros::ServiceServer gimbal_downward_lock = nh.advertiseService(gimbalDownwardLockName, gimbalDownwardLockCallback);


    ros::Rate loop_rate(60);

    while (ros::ok())
    {
        ros::spinOnce();

        try
        {
            /* code */
            cap.read(img);
            imageOri = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
            imageOriPub.publish(imageOri);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            if (findSVGimbal(gimbalType).camera == sv::CameraType::GX40 || findSVGimbal(gimbalType).camera == sv::CameraType::G1 || findSVGimbal(gimbalType).camera == sv::CameraType::SU17)
            {
                cap.setIp(cameraName);
                cap.open(findSVGimbal(gimbalType).camera);
            }
            else
            {
                cap.open(findSVGimbal(gimbalType).camera, std::stoi(cameraName));
            }
        }
	
	if (gimbalState.type == 3)
	{
		attitude_correction_message.uav_attitude = {-100.0, 0.0, 0.0};  // yaw, roll, pitch
		attitude_correction_message.uav_attitude_rate = {0.0, 0.0, 0.0};  // x, y, z 速度
		attitude_correction_message.uav_attitude_acceleration = {0.0, 0.0, 0.0};  // x, y, z 加速度
		attitude_correction_message.latitude =  30;
		attitude_correction_message.longitude =  50;
		attitude_correction_message.altitude =  70;
		attitude_correction_message.rel_alt = 0;
		attitude_correction_message.satellite_number = 20;
		//attitude_correction_pub.publish(attitude_correction_message);
		std::cout<<"send attitude_correction message"<<std::endl;
	}
        


        camera_info.header = std_msgs::Header();
        camera_info.height = cameraHeight;
        camera_info.width = cameraWidth;
        camera_info.distortion_model = "plumb_bob";
        camera_info.D = {0,0,0,0,0};
        camera_info.K = {
            910.75830078125,0.,661.21228027343750,
            0.0,910.4266967773438,359.0803222656250,
            0.,0,1.};
        camera_info.R = {1,0,0,0,1,0,0,0,1};
        camera_info.P = {709.60461425781250000,0,618.37634277343750000,0,0, 709.84454345703125000,364.7024125490000,0,0,0,1,0};
        camera_info_pub.publish(camera_info);

        sv::TargetsInFrame tgts(frame_id++);

        if (!g_Detection_raw.mode)
        {
            if((det.no_track_frame_count > 6) && (det.no_track_frame_count < 50)) {
                //std::cout<<"g_Detection_raw.mode: "<<g_Detection_raw.mode<<std::endl;
                gimbalNoTrack(gimbal);
                //std::cout << "no track!!!:"<< std::endl;
                gimbalState.moveMode = 1;
            }
            
        }
        else{
            gimbalState.moveMode = 3;
            if (gimbalState.type == 3)
            {
                gimbalTrack(gimbal, -(g_Detection_raw.cx - 0.5f), 120, g_Detection_raw.cy - 0.5f, 75);
                //std::cout << "track!!!:"<< std::endl;
            }
            else
            {
                gimbalTrack(gimbal,g_Detection_raw.cx - 0.5f,120,g_Detection_raw.cy - 0.5f,75);
            }
        }
        gimbal_state_pub.publish(gimbalState);
        loop_rate.sleep();
    }                     
}






void downward_locked_angle(){
    if (camera_type == "GX40"){
        gimbal->setGimbalDownwardHome();
    }
    else if (camera_type == "G1" || camera_type == "SU17")
    {                
        gimbal->setAngleEuler(0,90,0);
    }
}





void attitudeCorrectionCallback(const spirecv_msgs::AttitudeCorrection::ConstPtr &msg)
{
    std::cout<<"receive attirude correctrion message"<<std::endl;
    sv::GimbalPosT pos = {msg->uav_attitude[0], msg->uav_attitude[1], msg->uav_attitude[2]};
    sv::GimbalVelocityT speed = {msg->uav_attitude_rate[0], msg->uav_attitude_rate[1], msg->uav_attitude_rate[2]};
    sv::GimbalVelocityT acc = {msg->uav_attitude_acceleration[0], msg->uav_attitude_acceleration[1], msg->uav_attitude_acceleration[2]};
    float latitude = msg->latitude;
    float longitude = msg->longitude;
    float altitude = msg->altitude;
    float rel_alt = msg->rel_alt;
    uint32_t nState = msg->satellite_number;
    gimbal->attitudeCorrection(pos,speed,acc,longitude,latitude,altitude,nState,rel_alt,nullptr);
}



void gimableCallbackA(double &frame_ang_r, double &frame_ang_p, double &frame_ang_y,
                     double &imu_ang_r, double &imu_ang_p, double &imu_ang_y,
                     double &fov_x, double &fov_y)
{
  gimbalEulerAngle[0] = frame_ang_r;
  gimbalEulerAngle[1] = frame_ang_p; 
  gimbalEulerAngle[2] = frame_ang_y;
  gimbalEulerAngle[3] = imu_ang_r;
  gimbalEulerAngle[4] = imu_ang_p;
  gimbalEulerAngle[5] = imu_ang_y;
  gimbalFov[0] = fov_x;
  gimbalFov[1] = fov_y;
  
  gimbalState.angleRT[0] = imu_ang_r;
  gimbalState.angleRT[1] = imu_ang_p;
  gimbalState.angleRT[2] = frame_ang_y;
  gimbalState.fov[0] = fov_x;
  gimbalState.fov[1] = fov_y;

  static int print_count = 0;

  if (print_count == 300 )
  {

  std::cout<<"frame_ang_r : "<<frame_ang_r<<" ; frame_ang_p: "<<frame_ang_p<<" ; frame_ang_y: "<<frame_ang_y<<std::endl;
  std::cout<<"imu_ang_r : "<<imu_ang_r<<" ; imu_ang_p: "<<imu_ang_p<<" ; imu_ang_y: "<<imu_ang_y<<std::endl;
  std::cout<<"fov_x : "<<fov_x<<" ; fov_y: "<<fov_y<<std::endl;
  print_count = 0;

  }
  print_count++;

}

    

