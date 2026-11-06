#include <iostream>
#include <string>
// 包含SpireCV SDK头文件
#include <sv_world.h>
#include <ros/ros.h>
#include <spirecv_msgs/TargetsInFrame.h>
#include <spirecv_msgs/Target.h>
#include <spirecv_msgs/ROI.h>
#include <spirecv_msgs/Control.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <spirecv_msgs/GimbalState.h>

using namespace std;
using namespace sv;


std::vector<int> aruco_id_vector;
int clicked_id = -1;


// 定义窗口名称
static const std::string RGB_WINDOW = "Image window";
// 鼠标点击事件的回调函数
void onMouse(int event, int x, int y, int, void *);
void remoteMouse(spirecv_msgs::Control ctl_msg);

struct Point {
    int x, y; // 点的坐标
    Point(int initX, int initY) : x(initX), y(initY) {}
} ;

struct Rect {
    int x1, y1; // 左上角坐标
    int x2, y2; // 右下角坐标
};

// 判断点P是否在矩形A内部
bool isPointInside(const Point& P, const Rect& A) {
    return P.x >= A.x1 && P.x <= A.x2 &&
           P.y >= A.y1 && P.y <= A.y2;
}


Point clicked_point(-1,-1);

// find clicked id


void drawSpecificTargetsInFrame(
  cv::Mat& img_,
  const TargetsInFrame& tgts_,
  bool with_all=true,
  bool with_category=false,
  bool with_tid=false,
  bool with_seg=false,
  bool with_box=false,
  bool with_ell=false,
  bool with_aruco=false,
  bool with_yaw=false
);



cv::Mat ros_img;
bool get_ros_img = false;
int img_width, img_height;
image_transport::Publisher aruco_pub;
ros::Publisher spirecv_msg_pub;
spirecv_msgs::GimbalState gimbalState;


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

struct node
{
  double x, y;
};
node p1, p2, p3, p4;
node p;
double getCross(node p1, node p2, node p)
{
  return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}




int main(int argc, char *argv[])
{
  ros::init(argc, argv, "gx40_aruco_clicked_tracking");
  ros::NodeHandle nh("~");
  // 更新频率为60HZ
  ros::Rate loop_rate(60);
  int uav_id = 1;
  std::string camera_params_yaml = "";
  std::string algorithm_params_yaml = sv::get_home() + "/SpireCV/confs/sv_algorithm_params.json";
  std::string gimbalStateTopic = "/spirecv/gimbal_state";
  std::string local_saving_path = "";
  std::string input_image_topic = "";
  std::string output_image_topic = "";


  nh.getParam("uav_id", uav_id);
  nh.getParam("camera_parameters", camera_params_yaml);
  nh.getParam("algorithm_parameters", algorithm_params_yaml);
  nh.getParam("local_saving_path", local_saving_path);
  nh.getParam("input_image_topic", input_image_topic);
  nh.getParam("output_image_topic", output_image_topic);
  nh.getParam("gimbalStateTopic",gimbalStateTopic);

  printf("UAV_ID: %d\n", uav_id);
  printf("CAMERA_PARAMS: %s\n", camera_params_yaml.c_str());
  printf("ALGORITHM_PARAMS: %s\n", algorithm_params_yaml.c_str());
  printf("LOCAL_SAVING_PATH: %s\n", local_saving_path.c_str());
  printf("INPUT_IMAGE_TOPIC: %s\n", input_image_topic.c_str());
  printf("OUTPUT_IMAGE_TOPIC: %s\n", output_image_topic.c_str());

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_subscriber;
  image_subscriber = it.subscribe(input_image_topic, 10, cameraCallback);
  ros::Publisher spirecv_msg_pub = nh.advertise<spirecv_msgs::TargetsInFrame>("/uav" + std::to_string(uav_id) + "/spirecv/target", 1);
  ros::Publisher gimbal_state_pub = nh.advertise<spirecv_msgs::GimbalState>(gimbalStateTopic,1000);

  ros::Subscriber spirecv_ctl_sub = nh.subscribe("/uav" + std::to_string(uav_id) + "/spirecv/control", 10, remoteMouse);

  aruco_pub = it.advertise(output_image_topic.c_str(), 1);

  // 定义一个新的窗口，可在上面进行框选操作
  cv::namedWindow(RGB_WINDOW);
  // 设置窗口操作回调函数，该函数实现整个框选逻辑
  cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

  sv::ArucoDetector ad;
  // 手动导入相机参数，如果使用Amov的G1等吊舱或相机，则可以忽略该步骤，将自动下载相机参数文件
  ad.loadCameraParams(camera_params_yaml);


  cv::Mat img;
  int frame_id = 0;

  img_height = ad.image_height;
  img_width = ad.image_width;

  // sv::VideoStreamer streamer;
  // streamer.setup(cv::Size(img_width, img_height), 8554, 1);

  int no_track_frame_count = 0;
  printf("startr loop");
  while (ros::ok())
  {
    if (get_ros_img == true)
    {
      get_ros_img = false;
      img = ros_img.clone();


      // 实例化SpireCV的 单帧检测结果 接口类 TargetsInFrame
      sv::TargetsInFrame tgts(frame_id++);

      cv::resize(img, img, cv::Size(ad.image_width, ad.image_height));
      
      ad.detect(img, tgts);
      drawSpecificTargetsInFrame(img, tgts);

      spirecv_msgs::TargetsInFrame ros_tgts;
      ros_tgts.header.frame_id = "frame";
      ros_tgts.header.stamp = ros::Time::now();
      ros_tgts.header.seq = 1;
      ros_tgts.frame_id = tgts.frame_id;
      ros_tgts.height = tgts.height;
      ros_tgts.fps = tgts.fps;
      ros_tgts.fov_x = tgts.fov_x;
      ros_tgts.fov_y = tgts.fov_y;



      for (int i=0; i<tgts.targets.size(); i++)
      {

        if (clicked_id == -1)
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

        }

        aruco_id_vector.push_back(tgts.targets[i].category_id);
        if (clicked_id == -1 && clicked_point.x > 0  && clicked_point.y > 0)
        {
          clicked_id = findClosestTargetID(tgts,clicked_point.x,clicked_point.y);
        }

        if (tgts.targets[i].category_id == clicked_id)
        {
            //gimbalTrack(-(tgts.targets[i].cx - 0.5f), tgts.targets[i].cy - 0.5f);
            no_track_frame_count = 0;
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
            ros_target.mode = true;
            ros_tgts.targets.push_back(ros_target);
            std::cout<<"px:  "<<tgts.targets[i].px<<" , py:  "<<tgts.targets[i].py<<" , pz:  "<<tgts.targets[i].pz<<std::endl;


            gimbalState.moveMode = 1;
        }
      }
      no_track_frame_count++;
      if (no_track_frame_count >= 10000) no_track_frame_count = 51;
      ros_tgts.no_track_frame_count = no_track_frame_count;



      spirecv_msg_pub.publish(ros_tgts);
      gimbal_state_pub.publish(gimbalState);




      // 将img推流到 地址：rtsp://ip:8554/live
      // streamer.stream(img);

      // 显示检测结果img
      cv::imshow(RGB_WINDOW, img);
      cv::waitKey(10);

      // 显示检测结果img
      sensor_msgs::ImagePtr det_output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      aruco_pub.publish(det_output_msg);
    }

    else{
      //printf("no frame");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}





void onMouse(int event, int x, int y, int, void *)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    // 保存点击的点
    clicked_point = Point(x, y);

    // 打印并显示点击的坐标 
    cout << "Clicked at: " << x << ", " << y << endl;
    // gimbal->setAngleRateEuler(0, 0.02 * (y - 720 / 2), 0.02 * (x - 1280 / 2));
  }
  if (event == cv::EVENT_RBUTTONDOWN)
  {
    clicked_id = -1;
    clicked_point = Point(-1, -1);
    //gimbalNoTrack();
    //gimbal->setHome();                                              
  }
}



int findClosestTargetID(sv::TargetsInFrame& lastTgts, int clickX, int clickY) {
    std::set<std::pair<int, int>> distancesAndIDs; // 存储距离平方和对应的tracked_id
    for (int i = 0; i < lastTgts.targets.size(); i++) {
        int halfWidth = static_cast<int>((lastTgts.targets[i].w * lastTgts.width) / 2);
        int halfHeight = static_cast<int>((lastTgts.targets[i].h * lastTgts.height) / 2);
        int x = static_cast<int>(lastTgts.targets[i].cx * lastTgts.width);
        int y = static_cast<int>(lastTgts.targets[i].cy * lastTgts.height);

        int diffX = x - clickX;
        int diffY = y - clickY;

        if ((abs(diffX) < halfWidth) && (abs(diffY) < halfHeight)) {
            int distanceSquared = diffX * diffX + diffY * diffY;
            distancesAndIDs.insert({distanceSquared, lastTgts.targets[i].tracked_id});
        }
    }

    if (!distancesAndIDs.empty()) {
        // 返回距离最近的目标检测框的ID
        return distancesAndIDs.begin()->second;
    } else {
        // 如果没有找到任何框，则返回-1
        return -1;
    }
}




void drawSpecificTargetsInFrame(
  cv::Mat& img_,
  const TargetsInFrame& tgts_,
  bool with_all,
  bool with_category,
  bool with_tid,
  bool with_seg,
  bool with_box,
  bool with_ell,
  bool with_aruco,
  bool with_yaw
)
{


  //std::cout<<"go in void function"<<std::endl;
  if (tgts_.rois.size() > 0)
  {
    cv::Mat image_ret;
    cv::addWeighted(img_, 0.5, cv::Mat::zeros(cv::Size(img_.cols, img_.rows), CV_8UC3), 0, 0, image_ret);
    cv::Rect roi = cv::Rect(tgts_.rois[0].x1, tgts_.rois[0].y1, tgts_.rois[0].x2 - tgts_.rois[0].x1, tgts_.rois[0].y2 - tgts_.rois[0].y1);
    img_(roi).copyTo(image_ret(roi));
    image_ret.copyTo(img_);
  }
  std::vector<std::vector<cv::Point2f> > aruco_corners;
  std::vector<int> aruco_ids;

  //int i = 0;
  //for (Target tgt : tgts_.targets)
  for (int i=0; i<tgts_.targets.size(); i++)
  {
    //i++;
    if (clicked_id != -1 && tgts_.targets[i].category_id != clicked_id) continue;

    cv::circle(img_, cv::Point(int(tgts_.targets[i].cx * tgts_.width), int(tgts_.targets[i].cy * tgts_.height)), 4, cv::Scalar(0,255,0), 2);
    if ((with_all || with_aruco) && tgts_.targets[i].has_aruco)
    {
      std::vector<cv::Point2f> a_corners;
      int a_id;
      if (((Target)tgts_.targets[i]).getAruco(a_id, a_corners))  { aruco_ids.push_back(a_id); aruco_corners.push_back(a_corners); }
    }
    if ((with_all || with_box) && tgts_.targets[i].has_box)
    {
      Box b;
      ((Target)tgts_.targets[i]).getBox(b);
      cv::rectangle(img_, cv::Rect(b.x1, b.y1, b.x2-b.x1+1, b.y2-b.y1+1), cv::Scalar(0,0,255), 1, 1, 0);
      if ((with_all || with_category) && tgts_.targets[i].has_category)
      {
        cv::putText(img_, tgts_.targets[i].category, cv::Point(b.x1, b.y1-4), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(255,0,0));
      }
      if ((with_all || with_tid) && tgts_.targets[i].has_tid)
      {
        char tmp[32];
        sprintf(tmp, "TID: %d", tgts_.targets[i].tracked_id);
        cv::putText(img_, tmp, cv::Point(b.x1, b.y1-14), cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,0,255));
      }
    }
    if ((with_all || with_ell) && tgts_.targets[i].has_ell)
    {
      double xc, yc, a, b, rad;
      
    }
    if ((with_all || with_seg) && tgts_.targets[i].has_seg)
    {

      std::cout<<"here is mask "<<std::endl;
      cv::Mat mask = ((Target)tgts_.targets[i]).getMask() * 255;
      cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);
      mask.convertTo(mask, CV_8UC1);

      cv::resize(mask, mask, cv::Size(1280,720));
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      cv::Mat mask_disp = img_.clone();
      cv::fillPoly(mask_disp, contours, cv::Scalar(255,255,255), cv::LINE_AA);
      cv::polylines(img_, contours, true, cv::Scalar(255,255,255), 2, cv::LINE_AA);

      double alpha = 0.6;
      cv::addWeighted(img_, alpha, mask_disp, 1.0-alpha, 0, img_);
    }
    
    if ((with_all || with_aruco) && aruco_ids.size() > 0)
    {
    cv::aruco::drawDetectedMarkers(img_, aruco_corners, aruco_ids);
    }
  }
}

void remoteMouse(spirecv_msgs::Control ctl_msg)
{
  // 左键按下
  if (ctl_msg.mouse == spirecv_msgs::Control::MOUSE_LEFT)
  {
    // 保存点击的点
    clicked_point = Point(ctl_msg.x * img_width, ctl_msg.y * img_height);
    // 打印并显示点击的坐标 
    cout << "Clicked at: " << ctl_msg.x << ", " << ctl_msg.y << endl;
  }

  else if (ctl_msg.mouse == spirecv_msgs::Control::MOUSE_RIGHT)
  {
    clicked_id = -1;
    clicked_point = Point(-1, -1);
  }
}

