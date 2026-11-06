#include <iostream>
#include <string>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>


using namespace std;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "aruco_detection");
  ros::NodeHandle nh("~");
  // 更新频率为60HZ
  ros::Rate loop_rate(60);
  int uav_id = 1;
  std::string camera_params_yaml = sv::get_home() + "/SpireCV/calib_webcam_640x480.yaml";
  std::string local_saving_path = "";

  nh.getParam("uav_id", uav_id);
  nh.getParam("camera_parameters", camera_params_yaml);
  nh.getParam("local_saving_path", local_saving_path);
  printf("UAV_ID: %d\n", uav_id);
  printf("CAMERA_PARAMS: %s\n", camera_params_yaml.c_str());
  printf("LOCAL_SAVING_PATH: %s\n", local_saving_path.c_str());

  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/aruco_detection", 1);

  // 实例化Aruco检测器类
  sv::ArucoDetector ad;
  // 手动导入相机参数，如果使用Amov的G1等吊舱或相机，则可以忽略该步骤，将自动下载相机参数文件
  ad.loadCameraParams(camera_params_yaml);
  
  // 打开摄像头
  sv::Camera cap;
  cap.setWH(ad.image_width, ad.image_height);
  cap.setFps(30);
  cap.open(sv::CameraType::MIPI, 0);
  
  // 实例化OpenCV的Mat类，用于内存单帧图像
  cv::Mat img;
  int frame_id = 0;

  // 实例化视频保存类
  sv::VideoWriter vw;
  if (local_saving_path.size() > 0)
  {
    cap.read(img);
    // 设置保存路径"/home/amov/Videos"，保存图像尺寸（640，480），帧频25Hz，同步保存检测结果（.svj）
    vw.setup(local_saving_path, cv::Size(img.cols, img.rows), 25, true);
  }

  while (ros::ok())
  {
    // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
    sv::TargetsInFrame tgts(frame_id++);
    // 读取一帧图像到img
    cap.read(img);
    cv::resize(img, img, cv::Size(ad.image_width, ad.image_height));

    // 执行Aruco二维码检测
    ad.detect(img, tgts);
    
    // 同步保存视频流 和 检测结果信息
    if (local_saving_path.size() > 0)
      vw.write(img, tgts);

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

    // 控制台打印Aruco检测结果
    printf("Frame-[%d]\n", frame_id);
    // 打印当前检测的FPS
    printf("  FPS = %.2f\n", tgts.fps);
    // 打印当前相机的视场角（degree）
    printf("  FOV (fx, fy) = (%.2f, %.2f)\n", tgts.fov_x, tgts.fov_y);
    for (int i=0; i<tgts.targets.size(); i++)
    {
      spirecv_msgs::Target ros_target;
      ros_target.cx = tgts.targets[i].cx;
      ros_target.cy = tgts.targets[i].cy;
      ros_target.w = tgts.targets[i].w;
      ros_target.h = tgts.targets[i].h;

      ros_target.score = 1.0f;
      ros_target.category = tgts.targets[i].category;
      ros_target.category_id = tgts.targets[i].category_id;

      ros_target.yaw_a = tgts.targets[i].yaw_a;
      ros_target.tracked_id = tgts.targets[i].tracked_id;

      ros_target.los_ax = tgts.targets[i].los_ax;
      ros_target.los_ay = tgts.targets[i].los_ay;

      ros_target.px = tgts.targets[i].px;
      ros_target.py = tgts.targets[i].py;
      ros_target.pz = tgts.targets[i].pz;
      ros_tgts.targets.push_back(ros_target);

      printf("Frame-[%d], Aruco-[%d]\n", frame_id, i);
      // 打印每个二维码的中心位置，cx，cy的值域为[0, 1] 
      printf("  Aruco Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[i].cx, tgts.targets[i].cy);
      // 打印每个二维码的外接矩形框的宽度、高度，w，h的值域为(0, 1] 
      printf("  Aruco Size (w, h) = (%.3f, %.3f)\n", tgts.targets[i].w, tgts.targets[i].h);
      // 打印每个二维码的方位角，值域为[-180, 180]
      printf("  Aruco Yaw-angle = %.2f\n", tgts.targets[i].yaw_a);
      // 打印每个二维码的类别，字符串类型，"aruco-?"
      printf("  Aruco Category = %s\n", tgts.targets[i].category.c_str());
      // 打印每个二维码的ID号
      printf("  Aruco Tracked-ID = %d\n", tgts.targets[i].tracked_id);
      // 打印每个二维码的视线角，跟相机视场相关
      printf("  Aruco Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[i].los_ax, tgts.targets[i].los_ay);
      // 打印每个二维码的3D位置（在相机坐标系下），跟二维码实际边长、相机参数相关
      printf("  Aruco Position = (x, y, z) = (%.3f, %.3f, %.3f)\n", tgts.targets[i].px, tgts.targets[i].py, tgts.targets[i].pz);
    }

    spirecv_msg_pub.publish(ros_tgts);
    ros::spinOnce();
    loop_rate.sleep();
    
    // 显示检测结果img
    cv::imshow("img", img);
    cv::waitKey(10);
  }

  return 0;
}
