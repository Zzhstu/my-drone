/********************
* 默认原始推流
* GX40有多路流，无需推送原始流，可根据GX40推流地址进行合理配置选择
* G1只有一路视频流，需要gimbal_server启动之后获取到原始流之后，并重新再推流出来
* SU17有多路流，可多路同时拉流解码，无需推送原始流
* 仿真中插件生成原始图像，需要重新再推流出来
*/
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <prometheus_msgs/ParamSettings.h>
#include <sv_world.h>

// static const std::string RGB_WINDOW = "Image window11";

class VideoStreaming {
public:
    VideoStreaming(ros::NodeHandle& nh) 
        : it(nh), input_image(0), get_ros_img(false) 
    {
        // 获取参数
        nh.getParam("input_original_image_topic", input_original_image_topic);
        nh.getParam("input_process_image_topic", input_process_image_topic);
        nh.getParam("mode", current_mode);
        nh.getParam("video_streamer_width", width);
        nh.getParam("video_streamer_height", height);
        
        // 订阅初始图像话题
        //image_subscriber = it.subscribe(input_original_image_topic, 10, &VideoStreaming::cameraCallback, this);
        switchImage(current_mode);
        
        printf("INPUT_ORIGINAL_IMAGE_TOPIC: %s\n", input_original_image_topic.c_str());
        printf("INPUT_PROCESS_IMAGE_TOPIC: %s\n", input_process_image_topic.c_str());
        
        // 订阅切换图像话题
        switch_image_sub = nh.subscribe<prometheus_msgs::ParamSettings>(
            "/uav" + std::to_string(uav_id) + "/prometheus/param_settings", 
            10, 
            &VideoStreaming::switchImageCb, 
            this
        );

        // 设置视频流
        streamer.setup(cv::Size(width, height), 8554, 1);
    }

    void spin() {
        ros::Rate loop_rate(30);
        while (ros::ok()) {
            if (get_ros_img) {
                get_ros_img = false;  // 重置标志
                cv::Mat img = ros_img.clone();  // 克隆图像
                
                streamer.stream(img);  // 进行视频流
                
                // cv::imshow(RGB_WINDOW, img);  // 可选择显示图像
                cv::waitKey(1);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cam_image;
        try {
            cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cam_image->image.copyTo(ros_img);
            get_ros_img = true;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void switchImageCb(const prometheus_msgs::ParamSettings::ConstPtr& msg) {
        int size = msg->param_name.size();
    	for(int i = 0; i < size; i++)
    	{
    	   if(msg->param_name[i] == "/video_streaming/mode")
    	   {
    	       try
               {
                current_mode = std::stoi(msg->param_value[i]);
                switchImage(current_mode);
               }
               catch(const std::exception& e)
               {
                std::cerr << e.what() << '\n';
               }
    	   }    
        }
    }
    
    void switchImage(int mode)
    {
       if(image_subscriber)
           image_subscriber.shutdown();
       if (mode == Video_Streaming_Mode::ORIGINAL_IMAGE) {
            image_subscriber = it.subscribe(input_original_image_topic, 10, &VideoStreaming::cameraCallback, this);
            printf("INPUT_ORIGINAL_IMAGE_TOPIC: %s\n", input_original_image_topic.c_str());
       } else if(mode == Video_Streaming_Mode::PROCESS_IMAGE){
            image_subscriber = it.subscribe(input_process_image_topic, 10, &VideoStreaming::cameraCallback, this);
            printf("INPUT_PROCESS_IMAGE_TOPIC: %s\n", input_process_image_topic.c_str());
       }
    }

    image_transport::ImageTransport it;
    image_transport::Subscriber image_subscriber;
    ros::Subscriber switch_image_sub;

    int input_image;
    std::string input_original_image_topic;
    std::string input_process_image_topic;

    cv::Mat ros_img;
    bool get_ros_img;

    sv::VideoStreamer streamer;
    int width = 640;
    int height = 480;
    int uav_id = 1;

    enum Video_Streaming_Mode
    {
      ORIGINAL_IMAGE = 0,
      PROCESS_IMAGE = 1
    };
    
    int current_mode = Video_Streaming_Mode::ORIGINAL_IMAGE;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "video_streaming");
    ros::NodeHandle nh("~");
    
    VideoStreaming video_streamer(nh);  // 创建 VideoStreaming 实例
    video_streamer.spin();  // 进入循环

    return 0;
}
