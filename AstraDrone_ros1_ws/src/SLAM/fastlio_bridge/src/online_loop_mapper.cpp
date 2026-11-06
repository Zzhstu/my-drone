// Online, versioned global mapping. Local FAST-LIO/avoidance inputs are never
// changed. A separate worker rebuilds the SAME evidence engine on graph changes.
#define FASTLIO_MAP_BUILDER_LIBRARY
#include "static_map_builder.cpp"
#include <fastlio_bridge/loop_geometry.hpp>
#include <fastlio_bridge/LoopMapSnapshot.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

class OnlineLoopMapper
{
    struct Frame
    {
        sensor_msgs::PointCloud2::ConstPtr scan;
        nav_msgs::Odometry::ConstPtr odom;
        fastlio_bridge::DynamicObjectArray::ConstPtr objects;
    };
    struct Pending
    {
        std::unique_ptr<StaticMapBuilder> engine;
        std::shared_ptr<const online_loop::Graph> graph;
        std::size_t frames{0};
        double seconds{0};
    };
public:
    OnlineLoopMapper() : pnh_("~"),scan_sub_(nh_,"/uav1/fastlio/registered_scan",30),
        odom_sub_(nh_,"/uav1/fastlio/odom",30),
        objects_sub_(nh_,"/uav1/semantic_dynamic_objects",30),
        synchronizer_(scan_sub_,odom_sub_,objects_sub_,60)
    {
        pnh_.param("max_history_frames",max_frames_,6500);
        pnh_.param("full_height_free_space",full_height_free_space_,false);
        pnh_.param("dual_map_checkpoint_stride_frames",dual_checkpoint_stride_,0);
        if (dual_checkpoint_stride_<0 || dual_checkpoint_stride_>1000)
            throw std::invalid_argument("invalid dual map checkpoint stride");
        int megabytes=1536;pnh_.param("max_history_megabytes",megabytes,1536);
        max_bytes_=std::size_t(std::max(32,megabytes))*1024*1024;
        if (max_frames_<1) throw std::invalid_argument("max_history_frames must be positive");
        // Copy full live mapper parameters, including calibration and ghost
        // evidence policy. Only the output coordinate frame is changed.
        XmlRpc::XmlRpcValue configuration;
        if (!nh_.getParam("/static_map_builder",configuration))
            throw std::invalid_argument("missing full local mapper configuration");
        configuration["world_frame"]=std::string("loop_map");
        pnh_.setParam("engine",configuration);
        engine_namespace_=pnh_.getNamespace()+"/engine";
        active_.reset(new StaticMapBuilder(true,engine_namespace_));
        snapshot_pub_=nh_.advertise<fastlio_bridge::LoopMapSnapshot>("/uav1/loop_slam/snapshot",1,true);
        map_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/uav1/loop_slam/cloud_map",1,true);
        history_pub_=nh_.advertise<std_msgs::String>("/uav1/loop_slam/ghost_history",1,true);
        status_pub_=nh_.advertise<std_msgs::String>("/uav1/loop_slam/status",1,true);
        input_ack_pub_=nh_.advertise<std_msgs::Header>("/uav1/loop_slam/input_ack",20);
        processed_pub_=nh_.advertise<std_msgs::Header>("/uav1/loop_slam/processed_scan",20);
        graph_sub_=nh_.subscribe("/uav1/loop_closure/graph",2,&OnlineLoopMapper::graphCallback,this);
        takeoff_sub_=nh_.subscribe("/mavros_avoidance_controller/takeoff_complete",2,
            &OnlineLoopMapper::takeoffCallback,this);
        freeze_service_=nh_.advertiseService("/uav1/loop_slam/freeze_mapping",
            &OnlineLoopMapper::freeze,this);
        tf_timer_=nh_.createTimer(ros::Duration(1./30),&OnlineLoopMapper::publishTransform,this);
        synchronizer_.registerCallback(boost::bind(&OnlineLoopMapper::frameCallback,this,_1,_2,_3));
        processing_thread_=std::thread(&OnlineLoopMapper::process,this);
        rebuild_thread_=std::thread(&OnlineLoopMapper::rebuild,this);
    }

    ~OnlineLoopMapper()
    {
        stopping_=true;condition_.notify_all();
        if (processing_thread_.joinable()) processing_thread_.join();
        if (rebuild_thread_.joinable()) rebuild_thread_.join();
    }

private:
    static sensor_msgs::PointCloud2::ConstPtr compact(
        const sensor_msgs::PointCloud2::ConstPtr& source)
    {
        if (source->header.frame_id!="map") throw std::invalid_argument("nonlocal input scan");
        // The evidence engine consumes XYZ only. Preserve EVERY XYZ value and
        // point order; strip unused intensity/alignment padding to bound RAM.
        pcl::PointCloud<pcl::PointXYZ> points;pcl::fromROSMsg(*source,points);
        sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2);
        output->header=source->header;output->height=1;output->width=points.size();
        output->point_step=12;output->row_step=12*points.size();output->is_dense=source->is_dense;
        for (int i=0;i<3;++i)
        {
            sensor_msgs::PointField field;field.name=i==0?"x":i==1?"y":"z";
            field.offset=i*4;field.datatype=sensor_msgs::PointField::FLOAT32;field.count=1;
            output->fields.push_back(field);
        }
        output->data.resize(output->row_step);
        for (std::size_t i=0;i<points.size();++i)
        {
            std::memcpy(&output->data[i*12],&points[i].x,4);
            std::memcpy(&output->data[i*12+4],&points[i].y,4);
            std::memcpy(&output->data[i*12+8],&points[i].z,4);
        }
        return output;
    }

    void frameCallback(const sensor_msgs::PointCloud2::ConstPtr& scan,
        const nav_msgs::Odometry::ConstPtr& odom,
        const fastlio_bridge::DynamicObjectArray::ConstPtr& objects)
    {
        try
        {
            Frame frame{compact(scan),odom,objects};
            std_msgs::Header ack=scan->header;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (frozen_) return;
                if (!frames_.empty() && scan->header.stamp<=frames_.back().scan->header.stamp)
                    throw std::invalid_argument("nonincreasing synchronized scan stamps");
                if (frames_.size()>=std::size_t(max_frames_) || bytes_+frame.scan->data.size()>max_bytes_)
                {
                    capacity_limited_=true;
                    error_="history capacity exhausted; no silent eviction";
                    return;
                }
                bytes_+=frame.scan->data.size();frames_.push_back(std::move(frame));
                last_input_wall_=ros::WallTime::now();
                ack.seq=frames_.size();
            }
            input_ack_pub_.publish(ack);condition_.notify_all();
        }
        catch(const std::exception& error){fail(error.what());}
    }

    void graphCallback(const fastlio_bridge::LoopGraph::ConstPtr& message)
    {
        try
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (frozen_ && !message->raw.poses.empty() &&
                    message->raw.poses.back().header.stamp>freeze_stamp_) return;
            }
            std::shared_ptr<const online_loop::Graph> graph(new online_loop::Graph(*message));
            std::lock_guard<std::mutex> lock(mutex_);
            if (requested_ && (message->revision<requested_->message.revision ||
                (message->revision==requested_->message.revision &&
                 message->raw.poses.size()<requested_->message.raw.poses.size())))
                return; // Concurrent backend callbacks may deliver an older publication last.
            requested_=graph;condition_.notify_all();
        }
        catch(const std::exception& error){fail(error.what());}
    }

    bool freeze(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frozen_=true;
            if (!frames_.empty()) freeze_stamp_=frames_.back().scan->header.stamp;
        }
        // Stop adding descent scans to the loop graph as well. A match already
        // in progress may finish and is accepted if all keyframes predate cutoff.
        std_srvs::Trigger request;
        auto client=nh_.serviceClient<std_srvs::Trigger>("/loop_closure_backend/freeze_keyframes");
        const bool stopped=client.call(request) && request.response.success;
        response.success=stopped;
        response.message=stopped ? "Map input cutoff frozen; background rebuild may finish during landing" :
            "Map frozen but loop graph freeze service failed";
        return true;
    }

    void takeoffCallback(const std_msgs::Bool::ConstPtr& message)
    {
        if (!message->data) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (takeoff_stamp_.isZero()) takeoff_stamp_=ros::Time::now();
    }

    void fail(const std::string& reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);error_=reason;capacity_limited_=true;
        ROS_ERROR("[OnlineLoopMapper] %s",reason.c_str());
    }

    void publishTransform(const ros::TimerEvent&)
    {
        geometry_msgs::TransformStamped transform;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (display_transform_.header.frame_id.empty()) return;
            transform=display_transform_;
        }
        transform.header.stamp=ros::Time::now();tf_broadcaster_.sendTransform(transform);
    }

    Frame frameAt(std::size_t index)
    {
        std::lock_guard<std::mutex> lock(mutex_);return frames_.at(index);
    }

    void ingest(StaticMapBuilder& engine,const Frame& frame,
                const std::shared_ptr<const online_loop::Graph>& graph)
    {
        const auto transform=graph ? graph->at(frame.scan->header.stamp.toSec()) : Eigen::Isometry3d::Identity();
        ros::Time takeoff;
        { std::lock_guard<std::mutex> lock(mutex_);takeoff=takeoff_stamp_; }
        engine.ingest(online_loop::transformCloud(frame.scan,transform),
            online_loop::transformOdom(frame.odom,transform),
            online_loop::transformObjects(frame.objects,transform),
            !takeoff.isZero() && frame.scan->header.stamp>=takeoff);
    }

    void process()
    {
        std::size_t next=0;std::uint64_t version=0;
        std::shared_ptr<const online_loop::Graph> graph;
        ros::WallTime published;double processing_ms=0,rebuild_seconds=0;
        ros::Time published_stamp;std::uint64_t published_version=std::numeric_limits<std::uint64_t>::max();
        try
        {
            while (!stopping_ && ros::ok())
            {
                std::unique_ptr<Pending> pending;
                std::size_t available;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait_for(lock,std::chrono::milliseconds(20),[&]{
                        return stopping_ || pending_ || next<frames_.size();});
                    pending=std::move(pending_);available=frames_.size();
                    if (requested_ && requested_->message.revision==active_revision_) graph=requested_;
                }
                if (pending)
                {
                    // Catch the tiny tail between worker completion and commit.
                    // The old map remains active until this succeeds completely.
                    while (pending->frames<next)
                    {
                        ingest(*pending->engine,frameAt(pending->frames),pending->graph);
                        ++pending->frames;
                    }
                    next=pending->frames;active_=std::move(pending->engine);graph=pending->graph;
                    rebuild_seconds=pending->seconds;++version;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        active_revision_=graph->message.revision;rebuilding_=false;
                    }
                    condition_.notify_all();
                    if (next)
                    {
                        auto ack=frameAt(next-1).scan->header;ack.seq=next;processed_pub_.publish(ack);
                    }
                    ROS_INFO("[OnlineLoopMapper] atomic commit version=%lu graph=%lu frames=%zu",
                        version,graph->message.revision,next);
                }
                if (next<available)
                {
                    const auto frame=frameAt(next);const auto start=ros::WallTime::now();
                    ingest(*active_,frame,graph);++next;
                    processing_ms=(ros::WallTime::now()-start).toSec()*1000.;
                    std_msgs::Header ack=frame.scan->header;ack.seq=next;processed_pub_.publish(ack);
                }
                bool frozen,idle;
                { std::lock_guard<std::mutex> lock(mutex_);frozen=frozen_;
                  idle=(ros::WallTime::now()-last_input_wall_).toSec()>.5; }
                const ros::Time acquired=next?frameAt(next-1).scan->header.stamp:ros::Time();
                const bool paired_checkpoint=dual_checkpoint_stride_>0 && next &&
                    next%std::size_t(dual_checkpoint_stride_)==0 && acquired>published_stamp;
                if (next && ((acquired-published_stamp).toSec()>=.5 || version!=published_version ||
                    paired_checkpoint ||
                    (frozen && (ros::WallTime::now()-published).toSec()>=.5) ||
                    (idle && acquired>published_stamp)))
                {
                    publish(next,version,graph,processing_ms,rebuild_seconds);
                    published=ros::WallTime::now();
                    published_stamp=acquired;published_version=version;
                }
            }
        }
        catch(const std::exception& error){fail(error.what());}
    }

    void rebuild()
    {
        try
        {
            while (!stopping_ && ros::ok())
            {
                std::shared_ptr<const online_loop::Graph> graph;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    condition_.wait_for(lock,std::chrono::milliseconds(100),[&]{
                        return stopping_ || (requested_ && !rebuilding_ &&
                            requested_->message.revision>active_revision_);});
                    if (stopping_) return;
                    if (!requested_ || rebuilding_ || requested_->message.revision<=active_revision_) continue;
                    graph=requested_;rebuilding_=true;
                }
                const auto start=ros::WallTime::now();
                std::unique_ptr<Pending> ready(new Pending);
                ready->engine.reset(new StaticMapBuilder(true,engine_namespace_));ready->graph=graph;
                while (!stopping_ && ros::ok())
                {
                    Frame frame;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if (ready->frames==frames_.size())
                        {
                            ready->seconds=(ros::WallTime::now()-start).toSec();
                            pending_=std::move(ready);condition_.notify_all();break;
                        }
                        frame=frames_[ready->frames];
                    }
                    ingest(*ready->engine,frame,graph);++ready->frames;
                }
            }
        }
        catch(const std::exception& error){fail(error.what());}
    }

    void publish(std::size_t count,std::uint64_t version,
        const std::shared_ptr<const online_loop::Graph>& graph,double processing_ms,double rebuild_seconds)
    {
        const auto latest=frameAt(count-1);
        const auto transform=graph ? graph->at(latest.scan->header.stamp.toSec()) : Eigen::Isometry3d::Identity();
        const auto pose=transform*online_loop::poseMatrix(latest.odom->pose.pose);
        // Ground-relative planning may have different map Z at each XY.
        // Preserve observed 3-D ray evidence for graph-revision reseeding;
        // the consumer applies its own measured height-field slice.
        auto maps=active_->snapshot(pose.translation().z(),true,full_height_free_space_);
        fastlio_bridge::LoopMapSnapshot output;
        output.header=maps.map.header;output.header.seq=version;output.revision=version;
        output.graph_revision=graph?graph->message.revision:0;
        output.processed_frames=count;output.processing_ms=processing_ms;output.rebuild_wall_seconds=rebuild_seconds;
        std::string error;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            output.requested_graph_revision=requested_?requested_->message.revision:0;
            output.input_frames=frames_.size();output.input_bytes=bytes_;output.rebuilding=rebuilding_;
            output.capacity_limited=capacity_limited_ || active_->freeCapacityLimited();error=error_;
            output.frozen=frozen_;
        }
        output.map=std::move(maps.map);output.free=std::move(maps.free);output.suspects=std::move(maps.suspects);
        output.ghost_status=std::move(maps.status);output.ghost_history=std::move(maps.history);
        if (graph) output.graph=graph->message;
        output.global_from_local.header=output.header;output.global_from_local.child_frame_id="map";
        const auto converted=online_loop::poseMessage(transform);
        output.global_from_local.transform.translation.x=converted.position.x;
        output.global_from_local.transform.translation.y=converted.position.y;
        output.global_from_local.transform.translation.z=converted.position.z;
        output.global_from_local.transform.rotation=converted.orientation;
        { std::lock_guard<std::mutex> lock(mutex_);display_transform_=output.global_from_local; }
        snapshot_pub_.publish(output);
        // Display/export aliases only. Planner consumes the atomic snapshot.
        map_pub_.publish(output.map);history_pub_.publish(output.ghost_history);
        std_msgs::String status;std::ostringstream s;
        s<<"{\"version\":"<<version<<",\"graph_revision\":"<<output.graph_revision
         <<",\"requested_graph_revision\":"<<output.requested_graph_revision
         <<",\"input_frames\":"<<output.input_frames<<",\"processed_frames\":"<<count
         <<",\"input_bytes\":"<<output.input_bytes<<",\"rebuilding\":"<<(output.rebuilding?"true":"false")
         <<",\"capacity_limited\":"<<(output.capacity_limited?"true":"false")
         <<",\"processing_ms\":"<<processing_ms<<",\"rebuild_wall_seconds\":"<<rebuild_seconds
         <<",\"error_present\":"<<(!error.empty()?"true":"false")<<"}";
        status.data=s.str();status_pub_.publish(status);
    }

    ros::NodeHandle nh_,pnh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<fastlio_bridge::DynamicObjectArray> objects_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,nav_msgs::Odometry,
        fastlio_bridge::DynamicObjectArray> synchronizer_;
    ros::Subscriber graph_sub_,takeoff_sub_;
    ros::ServiceServer freeze_service_;
    ros::Timer tf_timer_;tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped display_transform_;
    ros::Publisher snapshot_pub_,map_pub_,history_pub_,status_pub_,input_ack_pub_,processed_pub_;
    std::mutex mutex_;std::condition_variable condition_;
    std::vector<Frame> frames_;std::size_t bytes_{0},max_bytes_{0};int max_frames_{6500};
    bool capacity_limited_{false},rebuilding_{false};std::string error_,engine_namespace_;
    ros::Time takeoff_stamp_;
    ros::WallTime last_input_wall_;
    bool frozen_{false};ros::Time freeze_stamp_;
    bool full_height_free_space_{false};
    int dual_checkpoint_stride_{0};
    std::shared_ptr<const online_loop::Graph> requested_;
    std::uint64_t active_revision_{0};
    std::unique_ptr<StaticMapBuilder> active_;
    std::unique_ptr<Pending> pending_;
    std::thread processing_thread_,rebuild_thread_;
    std::atomic<bool> stopping_{false};
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"online_loop_mapper");
    try
    {
        OnlineLoopMapper node;
        ros::AsyncSpinner spinner(2);spinner.start();ros::waitForShutdown();
    }
    catch(const std::exception& error){ROS_FATAL("Online mapper startup: %s",error.what());return 1;}
    return 0;
}
