#include <iostream>
#include <string>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;

// 定义窗口名称
// static const std::string RGB_WINDOW = "Image window";

cv::Mat ros_img;
bool get_ros_img = false;
bool TakePhotos = false;


std::string getTimestamp() {
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm *tm_now = std::localtime(&now);

    // 格式化时间为字符串
    std::ostringstream oss;
    oss << std::put_time(tm_now, "%Y%m%d_%H%M%S"); // 格式为 YYYYMMDD_HHMMSS
    return oss.str();
}

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cam_image;
  try
  {
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cam_image->image.copyTo(ros_img);

    get_ros_img = true;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}

void trigger_cb(const std_msgs::Int32::ConstPtr &msg)
{
    if(msg->data == 0)
    {
    	//拍照
    	TakePhotos = true;
    }else if(msg->data == 1)
    {
    	// gimbal->setAngleEuler(0, -90, 0);
    }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "landing_marker_detection");
  ros::NodeHandle nh("~");
  // 更新频率为60HZ
  ros::Rate loop_rate(20);
  int uav_id = 1;
  std::string camera_params_yaml = sv::get_home() + "/SpireCV/calib_webcam_640x480.yaml";
  std::string algorithm_params_yaml = sv::get_home() + "/SpireCV/confs/sv_algorithm_params.json";
  nh.getParam("uav_id", uav_id);
  nh.getParam("camera_parameters", camera_params_yaml);
  nh.getParam("algorithm_parameters", algorithm_params_yaml);
  printf("UAV_ID: %d\n", uav_id);
  printf("CAMERA_PARAMS: %s\n", camera_params_yaml.c_str());
  printf("ALGORITHM_PARAMS: %s\n", algorithm_params_yaml.c_str());

  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/Target", 1);
  //【订阅】无人机触发动作
  ros::Subscriber trigger_sub = nh.subscribe<std_msgs::Int32>("/uav" + std::to_string(uav_id) + "/prometheus/trigger", 10, trigger_cb);

  // 实例化 圆形降落标志 检测器类
  sv::LandingMarkerDetector lmd;
  // 手动导入相机参数，如果使用Amov的G1等吊舱或相机，则可以忽略该步骤，将自动下载相机参数文件
  lmd.loadCameraParams(camera_params_yaml);
  lmd.loadAlgorithmParams(algorithm_params_yaml);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_subscriber;
  image_subscriber = it.subscribe("/uav1/gimbal/image_original", 10, cameraCallback);

  cv::Mat img;
  int frame_id = 0;

  while (ros::ok())
  {
    if (get_ros_img == true)
    {
      // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
      sv::TargetsInFrame tgts(frame_id++);

      get_ros_img = false;
      img = ros_img.clone();
      cv::resize(img, img, cv::Size(lmd.image_width, lmd.image_height));
      lmd.detect(img, tgts);

      if (TakePhotos) {
            // 保存图像
            std::string filename = sv::get_home() + "/captured_photo_" + getTimestamp() + ".jpg";
            cv::imwrite(filename, img);
            std::cout << "Photo captured and saved as " << filename << std::endl;
            TakePhotos = false;
      } 
      // 可视化检测结果，叠加到img上
      sv::drawTargetsInFrame(img, tgts);

      spirecv_msgs::TargetsInFrame ros_tgts;
      ros_tgts.header.frame_id = "frame";
      ros_tgts.header.stamp = ros::Time::now();
      ros_tgts.header.seq = 1;
      ros_tgts.frame_id = tgts.frame_id;
      ros_tgts.height = tgts.height;
      ros_tgts.width = tgts.width;
      ros_tgts.fps = tgts.fps;
      ros_tgts.fov_x = tgts.fov_x;
      ros_tgts.fov_y = tgts.fov_y;

      // 控制台打印 降落标志 检测结果
      printf("Frame-[%d]\n", frame_id);
      // 打印当前检测的FPS
      printf("  FPS = %.2f\n", tgts.fps);
      // 打印当前相机的视场角（degree）
      printf("  FOV (fx, fy) = (%.2f, %.2f)\n", tgts.fov_x, tgts.fov_y);

      for (int i=0; i<tgts.targets.size(); i++)
      {
        // 仅追踪 X 类型的标靶
        // if (tgts.targets[i].category_id == landing_marker_id)
        // {
        //   gimbalTrack(tgts.targets[0].cx - 0.5f, tgts.targets[0].cy - 0.5f);
        // }   
        spirecv_msgs::Target ros_target;
        ros_target.cx = tgts.targets[i].cx;
        ros_target.cy = tgts.targets[i].cy;
        ros_target.w = tgts.targets[i].w;
        ros_target.h = tgts.targets[i].h;

        ros_target.score = tgts.targets[i].score;
        ros_target.category = tgts.targets[i].category;
        ros_target.category_id = tgts.targets[i].category_id;

        ros_target.los_ax = tgts.targets[i].los_ax;
        ros_target.los_ay = tgts.targets[i].los_ay;

        ros_target.px = tgts.targets[i].px;
        ros_target.py = tgts.targets[i].py;
        ros_target.pz = tgts.targets[i].pz;
        ros_tgts.targets.push_back(ros_target);

        printf("Frame-[%d], Marker-[%d]\n", frame_id, i);
        // 打印每个 降落标志 的中心位置，cx，cy的值域为[0, 1] 
        printf("  Marker Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[i].cx, tgts.targets[i].cy);
        // 打印每个 降落标志 的外接矩形框的宽度、高度，w，h的值域为(0, 1] 
        printf("  Marker Size (w, h) = (%.3f, %.3f)\n", tgts.targets[i].w, tgts.targets[i].h);
        // 打印每个 降落标志 的置信度
        printf("  Marker Score = %.3f\n", tgts.targets[i].score);
        // 打印每个 降落标志 的类别，字符串类型，"h"、"x"、"1"、"2"、"3"...
        printf("  Marker Category = %s, Category ID = %d\n", tgts.targets[i].category.c_str(), tgts.targets[i].category_id);
        // 打印每个 降落标志 的视线角，跟相机视场相关
        printf("  Marker Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[i].los_ax, tgts.targets[i].los_ay);
        // 打印每个 降落标志 的3D位置（在相机坐标系下），跟降落标志实际半径、相机参数相关
        printf("  Marker Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[i].px, tgts.targets[i].py, tgts.targets[i].pz);
      }
    
      spirecv_msg_pub.publish(ros_tgts);

      // 显示检测结果img
      // cv::imshow(RGB_WINDOW, img);
      cv::waitKey(1);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
