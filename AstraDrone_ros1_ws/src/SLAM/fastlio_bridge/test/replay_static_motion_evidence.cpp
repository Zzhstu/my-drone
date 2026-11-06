// Deterministic same-input replay of production detector, cluster and mapper.
// Compile with the package node flags/link libraries; run with an isolated ROS
// master and six loaded config namespaces (see run_static_motion_replay.py).
#define FASTLIO_DYNAMIC_REPLAY
#define FASTLIO_MAP_BUILDER_LIBRARY
#include "../src/dynamic_detector.cpp"
#include "../src/dynamic_cluster.cpp"
#include "../src/static_map_builder.cpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <map>
#include <memory>
#include <chrono>

struct Arm {
    std::string name;
    DynamicDetector detector;
    DynamicCluster cluster;
    StaticMapBuilder mapper;
    sensor_msgs::PointCloud2::ConstPtr stable;
    StaticMapBuilder::Snapshot snapshot;
    Arm(const std::string& n):name(n),detector("/"+n+"_detector"),
        cluster("/"+n+"_cluster"),mapper(true,"/"+n+"_map") {}
};

int main(int argc,char** argv) {
    ros::init(argc,argv,"static_motion_replay",ros::init_options::NoSigintHandler);
    if (argc<3) throw std::invalid_argument("bag_path output_prefix");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Error);
    ros::console::notifyLoggerLevelsChanged();
    const std::string prefix=argv[2];
    rosbag::Bag bag(argv[1],rosbag::bagmode::Read);
    std::map<std::uint64_t,nav_msgs::Odometry::ConstPtr> odoms;
    for (const auto& item:rosbag::View(bag,rosbag::TopicQuery(std::string("/uav1/fastlio/odom")))) {
        auto odom=item.instantiate<nav_msgs::Odometry>();
        odoms.emplace(odom->header.stamp.toNSec(),odom);
    }
    Arm old("baseline"), fixed("guarded");
    const bool guarded_only=argc>3 && std::string(argv[3])=="guarded";
    const std::vector<Arm*> arms=guarded_only ? std::vector<Arm*>{&fixed} : std::vector<Arm*>{&old,&fixed};
    std::ofstream objects(prefix+"_objects.csv"),frames(prefix+"_frames.csv");
    std::ofstream debug(prefix+"_planes.csv");
    debug<<"stamp,fg_points,removed,stable_points,stable_stamp,cluster_points,cx,cy,cz,supported,inliers,distance,rms,span,nx,ny,nz\n";
    objects<<"stamp,arm,id,predicted,x,y,z,vx,vy,vz\n";
    frames<<"stamp,arm,foreground,prior_free,objects,pending\n";
    std::uint64_t count=0;const auto begin=std::chrono::steady_clock::now();
    for (const auto& item:rosbag::View(bag,rosbag::TopicQuery(std::string("/uav1/fastlio/registered_scan")))) {
        const auto scan=item.instantiate<sensor_msgs::PointCloud2>();
        auto it=odoms.lower_bound(scan->header.stamp.toNSec());
        if (it==odoms.end()) --it;
        if (it!=odoms.begin()) {
            auto previous=std::prev(it);
            if (std::abs((previous->second->header.stamp-scan->header.stamp).toSec())<
                std::abs((it->second->header.stamp-scan->header.stamp).toSec())) it=previous;
        }
        if (std::abs((it->second->header.stamp-scan->header.stamp).toSec())>.08)
            throw std::runtime_error("unmatched odometry");
        const auto odom=it->second;
        for (Arm* arm:arms) {
            const auto foreground=boost::make_shared<sensor_msgs::PointCloud2>(arm->detector.replay(scan,odom));
            const auto output=boost::make_shared<fastlio_bridge::DynamicObjectArray>(
                arm->cluster.replay(foreground,odom,arm->stable));
            if (arm==&fixed && scan->header.stamp.toSec()>220. && scan->header.stamp.toSec()<232.) {
                pcl::PointCloud<pcl::PointXYZ> cloud;pcl::fromROSMsg(*foreground,cloud);
                for (const auto& c:arm->cluster.replay_input_clusters_) {
                    Eigen::Vector3d center=Eigen::Vector3d::Zero();
                    for (int i:c.indices) center+=Eigen::Vector3d(cloud[i].x,cloud[i].y,cloud[i].z);
                    center/=double(c.indices.size());
                    const auto p=arm->cluster.replayPlane(*foreground,c);
                    debug<<std::setprecision(12)<<scan->header.stamp.toSec()<<','<<cloud.size()<<','
                        <<arm->cluster.replay_structure_removed_<<','<<(arm->stable?arm->stable->width:0)<<','
                        <<(arm->stable?arm->stable->header.stamp.toSec():0.)<<','<<c.indices.size()<<','
                        <<center.x()<<','<<center.y()<<','<<center.z()<<','<<p.supported<<','<<p.inliers<<','
                        <<p.distance<<','<<p.rms<<','<<p.minor_span<<','<<p.normal.x()<<','<<p.normal.y()<<','<<p.normal.z()<<'\n';
                }
                if (count%5==0 && arm->stable) {
                    pcl::PointCloud<pcl::PointXYZ> stable;pcl::fromROSMsg(*arm->stable,stable);
                    pcl::io::savePCDFileBinary(prefix+"_stable_"+std::to_string(count)+".pcd",stable);
                    pcl::io::savePCDFileBinary(prefix+"_fg_"+std::to_string(count)+".pcd",cloud);
                }
            }
            arm->mapper.ingest(scan,odom,output,false);
            if (count%5==0) {
                arm->snapshot=arm->mapper.snapshot(1.2,false);
                arm->stable=boost::make_shared<sensor_msgs::PointCloud2>(arm->snapshot.stable);
            }
            unsigned evidence_count=0;
            if (arm==&fixed) {
                pcl::PointCloud<pcl::PointXYZI> cloud;pcl::fromROSMsg(*foreground,cloud);
                for (const auto& p:cloud.points) if (p.intensity>.5f) ++evidence_count;
            }
            frames<<std::setprecision(12)<<scan->header.stamp.toSec()<<','<<arm->name<<','
                <<foreground->width*foreground->height<<','<<evidence_count<<','
                <<output->objects.size()<<','<<arm->snapshot.suspects.width<<'\n';
            for (const auto& object:output->objects)
                objects<<std::setprecision(12)<<scan->header.stamp.toSec()<<','<<arm->name<<','
                    <<object.id<<','<<int(object.predicted)<<','<<object.pose.position.x<<','
                    <<object.pose.position.y<<','<<object.pose.position.z<<','
                    <<object.twist.linear.x<<','<<object.twist.linear.y<<','<<object.twist.linear.z<<'\n';
        }
        ++count;
        if (count%250==0) std::cout<<"processed "<<count<<" stamp "<<scan->header.stamp.toSec()<<std::endl;
    }
    for (Arm* arm:arms) {
        arm->snapshot=arm->mapper.snapshot(1.2,false);
        std::ofstream(prefix+"_"+arm->name+"_history.json")<<arm->snapshot.history.data<<'\n';
        std::ofstream(prefix+"_"+arm->name+"_status.json")<<arm->snapshot.status.data<<'\n';
        pcl::PointCloud<pcl::PointXYZ> cloud;pcl::fromROSMsg(arm->snapshot.map,cloud);
        pcl::io::savePCDFileBinary(prefix+"_"+arm->name+"_map.pcd",cloud);
    }
    const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
    std::ofstream(prefix+"_completion.json")<<"{\"processed_scans_per_arm\":"<<count
        <<",\"matched_odometry_per_arm\":"<<count<<",\"guarded_only\":"
        <<(guarded_only ? "true" : "false")<<",\"wall_seconds\":"<<seconds<<"}\n";
}
