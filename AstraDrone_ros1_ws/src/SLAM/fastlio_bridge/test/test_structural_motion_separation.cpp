#define FASTLIO_DYNAMIC_REPLAY
#include "../src/dynamic_cluster.cpp"
#include <cassert>
#include <iostream>

using RichCloud=pcl::PointCloud<pcl::PointXYZI>;

sensor_msgs::PointCloud2::Ptr message(const RichCloud& cloud,double stamp,const std::string& frame="map") {
    auto msg=boost::make_shared<sensor_msgs::PointCloud2>();pcl::toROSMsg(cloud,*msg);
    msg->header.stamp.fromSec(stamp);msg->header.frame_id=frame;return msg;
}

int main(int argc,char** argv) {
    ros::init(argc,argv,"structural_separation_fixture",ros::init_options::NoSigintHandler);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Error);
    ros::console::notifyLoggerLevelsChanged();
    ros::NodeHandle nh;
    nh.setParam("/structural_fixture/static_plane_motion_veto",true);
    nh.setParam("/structural_fixture/require_observed_free_motion",true);
    nh.setParam("/structural_fixture/cluster_tolerance",.30);
    nh.setParam("/structural_fixture/static_confirm_frames",3);
    RichCloud wall,person;
    auto add=[](RichCloud& cloud,double x,double y,double z,float evidence) {
        pcl::PointXYZI point;point.x=x;point.y=y;point.z=z;point.intensity=evidence;cloud.push_back(point);
    };
    for (int y=-6;y<=6;++y) for (int z=-5;z<=5;++z) add(wall,2.11,y*.15,1.025+z*.15,0);
    for (int y=-2;y<=2;++y) for (int z=-3;z<=3;++z) add(person,2.03,y*.08,1.025+z*.15,1);
    RichCloud mixed=wall;mixed+=person;
    auto stable=message(wall,1.0),scan=message(mixed,1.1);
    DynamicCluster separator("/structural_fixture");
    auto retained=separator.replaySeparation(scan,stable);
    assert(retained.size()==person.size());
    for (int point_id:retained) assert(std::size_t(point_id)>=wall.size());
    assert(separator.replaySeparation(message(mixed,3.0),stable).size()==mixed.size());
    assert(separator.replaySeparation(message(mixed,1.1,"other_map"),stable).size()==mixed.size());
    std::cout<<"mixed 80% wall + 20% person at 8 cm: wall removed, every person point retained; stale/frame rejected\n";

    auto odom=boost::make_shared<nav_msgs::Odometry>();odom->pose.pose.orientation.w=1.;
    unsigned real_detections=0;
    for (int frame=0;frame<50;++frame) {
        const double stamp=2.+frame*.1;RichCloud scene=wall;
        for (auto point:person.points) {point.y+=frame*.05;scene.push_back(point);}
        scan=message(scene,stamp);odom->header=scan->header;
        const auto output=separator.replay(scan,odom,message(wall,stamp-.1));
        if (frame>=20) assert(!output.objects.empty());
        for (const auto& cluster:separator.replay_cluster_indices_)
            for (int point_id:cluster.indices) assert(std::size_t(point_id)>=wall.size());
        pcl::PointCloud<pcl::PointXYZ> dynamic_points;
        pcl::fromROSMsg(separator.last_replay_dynamic_points_,dynamic_points);
        assert(dynamic_points.size()<=person.size());
        for (const auto& point:dynamic_points) assert(point.x<2.07);
        for (const auto& object:output.objects) {
            ++real_detections;assert(object.pose.position.x<2.07);
        }
    }
    assert(real_detections>0);
    std::cout<<"moving mixed-cluster person remains dynamically detected (given prior real free-ray evidence), detections="<<real_detections<<"\n";

    DynamicCluster fixed("/structural_fixture");
    for (int frame=0;frame<50;++frame) {
        const double stamp=10.+frame*.1;RichCloud scene=wall;
        // A fixed protruding rail sampled through a sliding visibility window:
        // its centroid moves, but no part entered previously measured free.
        for (int y=frame;y<frame+8;++y) for (int z=-3;z<=3;++z)
            add(scene,2.03,(y-25)*.04,1.025+z*.15,0);
        scan=message(scene,stamp);odom->header=scan->header;
        assert(fixed.replay(scan,odom,message(wall,stamp-.1)).objects.empty());
    }
    std::cout<<"fixed 8 cm protrusion with moving scan crop never reported dynamic\n";
    DynamicCluster paused("/structural_fixture");
    for (int frame=0;frame<50;++frame) {
        const double stamp=20.+frame*.1;RichCloud scene=wall;
        for (std::size_t i=0;i<person.size();++i) if ((i+frame)%4) {
            auto point=person.points[i];point.intensity=0;scene.push_back(point);
        }
        scan=message(scene,stamp);odom->header=scan->header;
        assert(paused.replay(scan,odom,message(wall,stamp-.1)).objects.empty());
    }
    std::cout<<"stationary person with changing sample visibility never reported dynamic\n";

    RichCloud two_faces=wall;
    for (int y=-14;y<=14;++y) for (int z=-10;z<=10;++z)
        add(two_faces,1.51,y*.08,1.025+z*.075,0);
    DynamicCluster opposite("/structural_fixture");
    retained=opposite.replaySeparation(message(mixed,30.1),message(two_faces,30.));
    assert(retained.size()==person.size());
    for (int i:retained) assert(std::size_t(i)>=wall.size());
    std::cout<<"denser opposite face cannot replace support for actual foreground plane\n";

    RichCloud minority=wall;
    for (int y=-30;y<=30;++y) for (int z=-20;z<=20;++z)
        add(minority,1.51,y*.035,1.025+z*.04,0);
    DynamicCluster sparse_back("/structural_fixture");
    retained=sparse_back.replaySeparation(message(mixed,31.1),message(minority,31.));
    assert(retained.size()==person.size());
    for (int i:retained) assert(std::size_t(i)>=wall.size());
    std::cout<<"minority back face below 15% global prior still separated; nearby person retained\n";

    DynamicCluster unseen("/structural_fixture");
    unsigned separated=0;
    for (int frame=0;frame<25;++frame) {
        const double stamp=40.+frame*.1;RichCloud scene;
        // Deliberately provide apparent prior-free evidence, reproducing
        // the old wall's boundary/free-voxel aliasing. The mature map is empty.
        for (int y=frame;y<frame+12;++y) for (int z=-5;z<=5;++z)
            add(scene,2.11,(y-15)*.10,1.025+z*.15,1);
        scan=message(scene,stamp);odom->header=scan->header;
        assert(unseen.replay(scan,odom,message(RichCloud(),stamp-.1)).objects.empty());
        if (frame>=8) {assert(unseen.replay_structure_removed_>=scene.size()-2);++separated;}
    }
    assert(separated==17);
    std::cout<<"new static face with sliding sampling filtered before per-voxel map maturity\n";

    DynamicCluster moving_board("/structural_fixture");
    unsigned board_detections=0;
    for (int frame=0;frame<40;++frame) {
        const double stamp=50.+frame*.1;RichCloud scene=wall;
        for (auto& point:scene) {point.x+=frame*.05;point.intensity=1;}
        scan=message(scene,stamp);odom->header=scan->header;
        const auto output=moving_board.replay(scan,odom,message(RichCloud(),stamp-.1));
        assert(moving_board.replay_structure_removed_==0);
        board_detections+=output.objects.size();
    }
    assert(board_detections>0);
    std::cout<<"moving broad plane is not self-certified static by the temporal cache\n";
}
