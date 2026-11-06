#include <iostream>
#include <string>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>


using namespace std;
using namespace cv;
using namespace sv;
// 定义窗口名称
static const std::string RGB_WINDOW = "Image window";
// 框选到的矩形
cv::Rect rect_sel;
// 框选起始点
cv::Point pt_origin;
// 是否按下左键
bool b_clicked = false;
// 是否得到一个新的框选区域
bool b_renew_ROI = false;
// 是否开始跟踪
bool b_begin_TRACK = false;
// 实现框选逻辑的回调函数
void onMouse(int event, int x, int y, int, void*);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "single_object_tracking");
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

  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/single_object_tracking", 1);

  // 定义一个新的窗口，可在上面进行框选操作
  cv::namedWindow(RGB_WINDOW);
  // 设置窗口操作回调函数，该函数实现整个框选逻辑
  cv::setMouseCallback(RGB_WINDOW, onMouse, 0);
  // 实例化 框选目标跟踪类
  sv::SingleObjectTracker sot;
  // 手动导入相机参数，如果使用Amov的G1等吊舱或相机，则可以忽略该步骤，将自动下载相机参数文件
  sot.loadCameraParams(camera_params_yaml);
  
  // 打开摄像头
  sv::Camera cap;
  cap.setWH(sot.image_width, sot.image_height);
  cap.setFps(30);
  cap.open(sv::CameraType::MIPI, 0);
  // 实例化OpenCV的Mat类，用于内存单帧图像
  cv::Mat img;
  int frame_id = 0;

  // 实例化视频保存类
  //sv::VideoWriter vw;
  //if (local_saving_path.size() > 0)
  //{
    //cap.read(img);
    // 设置保存路径"/home/amov/Videos"，保存图像尺寸（640，480），帧频25Hz，同步保存检测结果（.svj）
    //vw.setup(local_saving_path, cv::Size(img.cols, img.rows), 25, true);
  //}

  while (ros::ok())
  {
    // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
    sv::TargetsInFrame tgts(frame_id++);
    // 读取一帧图像到img
    cap.read(img);

    // 开始 单目标跟踪 逻辑
    // 是否有新的目标被手动框选
    if (b_renew_ROI)
    {
      // 拿新的框选区域 来 初始化跟踪器
      sot.init(img, rect_sel);
      // 重置框选标志
      b_renew_ROI = false;
      // 开始跟踪
      b_begin_TRACK = true;
    }
    else if (b_begin_TRACK)
    {
      // 以前一帧的结果继续跟踪
      sot.track(img, tgts);

      // 同步保存视频流 和 检测结果信息
      // if (local_saving_path.size() > 0)
      //   vw.write(img, tgts);

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

      // 控制台打印 单目标跟踪 结果
      printf("Frame-[%d]\n", frame_id);
      // 打印当前检测的FPS
      printf("  FPS = %.2f\n", tgts.fps);
      // 打印当前相机的视场角（degree）
      printf("  FOV (fx, fy) = (%.2f, %.2f)\n", tgts.fov_x, tgts.fov_y);
      if (tgts.targets.size() > 0)
      {
        spirecv_msgs::Target ros_target;
        ros_target.cx = tgts.targets[0].cx;
        ros_target.cy = tgts.targets[0].cy;
        ros_target.w = tgts.targets[0].w;
        ros_target.h = tgts.targets[0].h;

        ros_target.los_ax = tgts.targets[0].los_ax;
        ros_target.los_ay = tgts.targets[0].los_ay;

        ros_tgts.targets.push_back(ros_target);

        printf("Frame-[%d]\n", frame_id);
        // 打印 跟踪目标 的中心位置，cx，cy的值域为[0, 1] 
        printf("  Tracking Center (cx, cy) = (%.3f, %.3f)\n", tgts.targets[0].cx, tgts.targets[0].cy);
        // 打印 跟踪目标 的外接矩形框的宽度、高度，w，h的值域为(0, 1] 
        printf("  Tracking Size (w, h) = (%.3f, %.3f)\n", tgts.targets[0].w, tgts.targets[0].h);
        // 打印 跟踪目标 的视线角，跟相机视场相关
        printf("  Tracking Line-of-sight (ax, ay) = (%.3f, %.3f)\n", tgts.targets[0].los_ax, tgts.targets[0].los_ay);
      }
      spirecv_msg_pub.publish(ros_tgts);

    }

    ros::spinOnce();
    loop_rate.sleep();
    
    // 显示检测结果img
    cv::imshow(RGB_WINDOW, img);
    cv::waitKey(10);
  }

  return 0;
}

void onMouse(int event, int x, int y, int, void*)
{
  if (b_clicked)
  {
    // 更新框选区域坐标
    rect_sel.x = MIN(pt_origin.x, x);        
    rect_sel.y = MIN(pt_origin.y, y);
    rect_sel.width = abs(x - pt_origin.x);   
    rect_sel.height = abs(y - pt_origin.y);
  }
  // 左键按下
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    b_begin_TRACK = false;  
    b_clicked = true; 
    pt_origin = cv::Point(x, y);       
    rect_sel = cv::Rect(x, y, 0, 0);  
  }
  // 左键松开
  else if (event == cv::EVENT_LBUTTONUP)
  {
    // 框选区域需要大于8x8像素
    if (rect_sel.width * rect_sel.height < 64)
    {
      ;
    }
    else
    {
      b_clicked = false;
      b_renew_ROI = true;    
    } 
  }
}
