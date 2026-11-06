#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/conversions.h> 
#include <pcl_ros/point_cloud.h>
#include <prometheus_msgs/ParamSettings.h>

class RadiusFilter {
public:
    RadiusFilter() {
        // ROS节点初始化
        ros::NodeHandle nh("~");

        // 获取参数
        nh.param("outrem_radius", radius_, 0.1);
        nh.param("outrem_neighbors", min_neighbors_, 5);
        nh.param("uav_id", uav_id_, 1);
        
        std::cout << "outrem_radius: " << radius_ << std::endl;
        std::cout << "outrem_neighbors: " << min_neighbors_ << std::endl;
        
        // 订阅和发布
        sub_ = nh.subscribe("/cloud_in", 1, &RadiusFilter::cloudCallback, this);
        paramsettings_sub_ = nh.subscribe("/uav" + std::to_string(uav_id_) + "/prometheus/param_settings", 1, &RadiusFilter::paramSettingsCallback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filter_cloud_out", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(pcl_pc2, cloud);

        // 应用半径滤波器
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
        radius_filter.setInputCloud(cloud.makeShared());
        radius_filter.setRadiusSearch(radius_);
        radius_filter.setMinNeighborsInRadius(min_neighbors_);

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        radius_filter.filter(filtered_cloud);

        // 发布过滤后的点云
        sensor_msgs::PointCloud2 output;
        pcl::toPCLPointCloud2(filtered_cloud, pcl_pc2);
        pcl_conversions::fromPCL(pcl_pc2, output);
        pub_.publish(output);
    }
    
    void paramSettingsCallback(const prometheus_msgs::ParamSettings::ConstPtr &msg) {
    	int size = msg->param_name.size();
    	for(int i = 0; i < size; i++)
    	{
    	   if(msg->param_name[i] == "/radius_filter_node/outrem_radius")
    	   {
    	       try
               {
                radius_ = std::stod(msg->param_value[i]);
                std::cout << "outrem_radius: " << radius_ << std::endl;
               }
               catch(const std::exception& e)
               {
                std::cerr << e.what() << '\n';
               }
    	   }
           if (msg->param_name[i] == "/radius_filter_node/outrem_neighbors")
           {
               try
               {
                min_neighbors_ = std::stoi(msg->param_value[i]);
                std::cout << "outrem_neighbors: " << min_neighbors_ << std::endl;
               }
               catch(const std::exception& e)
               {
                std::cerr << e.what() << '\n';
               }
           }
           
    	}
    }

private:
    ros::Subscriber sub_;
    ros::Subscriber paramsettings_sub_;
    ros::Publisher pub_;
    double radius_;
    int min_neighbors_;
    int uav_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "radius_filter_node");
    RadiusFilter rf;
    ros::spin();
    return 0;
}


