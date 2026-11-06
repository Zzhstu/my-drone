#include <ros/ros.h>

#include <fastlio_bridge/DynamicObjectArray.h>
#include <fastlio_bridge/static_surface_guard.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class StaticMapBuilder
{
public:
    using PointT = pcl::PointXYZ;
    using CloudT = pcl::PointCloud<PointT>;

    struct Snapshot
    {
        sensor_msgs::PointCloud2 map, free, suspects, stable;
        std_msgs::String status, history;
    };

    struct Key
    {
        int x{0};
        int y{0};
        int z{0};
        bool operator==(const Key& rhs) const
        {
            return x == rhs.x && y == rhs.y && z == rhs.z;
        }
    };

    struct KeyHash
    {
        std::size_t operator()(const Key& key) const
        {
            // Coordinates are correlated; x^(y<<1)^(z<<2) collapsed large
            // planar grids into very few buckets and made ray clearing costly.
            std::size_t seed = 0;
            for (int value : {key.x,key.y,key.z})
                seed ^= std::hash<int>()(value) + 0x9e3779b9U + (seed<<6U) + (seed>>2U);
            return seed;
        }
    };

    struct Cell
    {
        Eigen::Vector3d mean{Eigen::Vector3d::Zero()};
        std::uint32_t hits{0};
        std::uint64_t last_frame{0};
        // Consecutive current scans whose rays traversed this voxel without a
        // return. This is the negative evidence needed to remove old people.
        std::uint16_t free_misses{0};
        ros::Time last_miss;
        ros::Time last_seen;
        ros::Time stable_since;
        // A confirmed moving track only creates a revisit hypothesis. It
        // never authorizes deletion of a measured surface.
        bool suspect{false};
        ros::Time last_dynamic_seen;
        std::uint16_t post_dynamic_hits{0};
        Eigen::Vector3d first_post_origin{Eigen::Vector3d::Zero()};
        // Occupancy and ghost history are separate: a cleared voxel may later
        // receive a new return without forgetting the previous episode.
        bool ever_ghost_suspect{false};
        std::uint32_t previous_ghost_clears{0};
    };

    struct GhostHistory
    {
        // 0: suspected, 1: cleared by free rays, 2: reobserved occupied,
        // 3: unresolved after occupancy-memory eviction.
        int state{0};
        std::uint32_t suspected_episodes{0};
        std::uint32_t cleared_episodes{0};
        std::uint32_t retained_episodes{0};
        // Episode-local scan counts.  A reached viewpoint is not evidence:
        // only a matched free ray or a post-departure return changes these.
        std::uint32_t free_evidence_frames{0};
        // Actual selected LiDAR rays, not visibility predicted by the view
        // planner. A traversed voxel can still miss the stored point.
        std::uint32_t sampled_ray_crossing_frames{0};
        std::uint32_t off_point_ray_frames{0};
        // Ray passes near the old point, but a return in a neighboring voxel
        // protects the surface.  This explains many boundary ambiguities.
        std::uint32_t protected_endpoint_frames{0};
        // A close neighboring return observed from separated sensor origins
        // suggests a voxel-boundary split of one static surface. It only
        // reduces revisit priority; it cannot clear or certify the voxel.
        std::uint32_t close_endpoint_frames{0};
        double closest_endpoint_distance{std::numeric_limits<double>::infinity()};
        double close_endpoint_baseline{0.0};
        Eigen::Vector3d first_close_endpoint_origin{Eigen::Vector3d::Zero()};
        std::uint32_t post_departure_hit_frames{0};
        std::uint32_t positive_endpoint_frames{0};
        double positive_endpoint_baseline{0.0};
        double positive_endpoint_distance{std::numeric_limits<double>::infinity()};
        Eigen::Vector3d first_positive_origin{Eigen::Vector3d::Zero()};
        std::string resolution_reason{"suspected"};
        // A surface certificate only pauses active revisits; it does not
        // resolve this history entry or change measured occupied/free cells.
        ros::Time encounter_started, last_positive_endpoint, surface_checked;
        std::uint32_t surface_positive_frames{0};
        astra::SurfaceEvidence surface;
        bool surface_suppressed{false};
        std::uint32_t surface_consistent_frames{0}, surface_support_points{0};
        std::uint64_t surface_evidence_frame{0};
        ros::Time surface_last_support;
        ros::Time last_sampled_ray_crossing;
        ros::Time last_off_point_ray;
        ros::Time last_free_evidence;
        ros::Time last_protected_endpoint;
        ros::Time last_close_endpoint;
        ros::Time last_post_departure_hit;
        ros::Time first_suspected;
        ros::Time last_dynamic;
        ros::Time last_event;
    };

    struct TrailSample
    {
        Eigen::Vector3d center{Eigen::Vector3d::Zero()};
        ros::Time stamp;
    };

    struct FrameCell
    {
        Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
        std::uint32_t count{0};
    };

    explicit StaticMapBuilder(bool headless=false, const std::string& config_namespace="~")
        : nh_(), pnh_(config_namespace), headless_(headless)
    {
        pnh_.param<std::string>("cloud_topic", cloud_topic_,
                                "/uav1/fastlio/registered_scan");
        pnh_.param<std::string>("objects_topic", objects_topic_,
                                "/uav1/dynamic_objects");
        pnh_.param<std::string>("output_topic", output_topic_,
                                "/uav1/fastlio/cloud_map");
        pnh_.param<std::string>("legacy_output_topic", legacy_output_topic_,
                                "/uav1/static_map");
        pnh_.param<std::string>("stable_output_topic", stable_output_topic_,
                                "/uav1/stable_static_map");
        pnh_.param<std::string>("local_output_topic", local_output_topic_,
                                "/uav1/local_static_map");
        pnh_.param<std::string>("free_space_output_topic",
                                free_space_output_topic_,
                                "/uav1/local_free_space");
        pnh_.param<std::string>("suspect_output_topic", suspect_output_topic_,
                                "/uav1/map/ghost_suspects");
        pnh_.param<std::string>("suspect_status_topic", suspect_status_topic_,
                                "/uav1/map/ghost_revisit_status");
        pnh_.param<std::string>("ghost_history_topic", ghost_history_topic_,
                                "/uav1/map/ghost_history");
        pnh_.param<std::string>("odom_topic", odom_topic_,
                                "/uav1/fastlio/odom");
        pnh_.param<std::string>("world_frame", world_frame_, "map");
        pnh_.param("voxel_size", voxel_size_, 0.15);
        pnh_.param("dynamic_mask_radius", dynamic_mask_radius_, 0.65);
        pnh_.param("dynamic_mask_half_height", dynamic_mask_half_height_, 1.0);
        pnh_.param("backward_trail_seconds", backward_trail_seconds_, 1.5);
        pnh_.param("trail_step", trail_step_, 0.15);
        pnh_.param("dynamic_history_seconds", dynamic_history_seconds_, 20.0);
        // Ablation switch: keep detection and the safety controller identical
        // while removing only the navigation map's dynamic-object cleanup.
        pnh_.param("enable_dynamic_map_clearing",
                   enable_dynamic_map_clearing_, true);
        pnh_.param("minimum_static_hits", minimum_static_hits_, 2);
        pnh_.param("stable_minimum_static_hits",
                   stable_minimum_static_hits_, 8);
        pnh_.param("local_minimum_hits", local_minimum_hits_, 1);
        pnh_.param("local_max_age_frames", local_max_age_frames_, 15);
        pnh_.param("local_radius", local_radius_, 6.0);
        pnh_.param("enable_free_space_clearing",
                   enable_free_space_clearing_, true);
        pnh_.param("free_space_miss_threshold",
                   free_space_miss_threshold_, 3);
        pnh_.param("free_space_endpoint_margin",
                   free_space_endpoint_margin_, 0.30);
        pnh_.param("free_space_max_range", free_space_max_range_, 6.0);
        pnh_.param("free_space_ray_step", free_space_ray_step_, 0.15);
        pnh_.param("sensor_origin_z_offset", sensor_origin_z_offset_, 0.0);
        pnh_.param("max_free_rays", max_free_rays_, 8000);
        pnh_.param("max_free_voxels", max_free_voxels_, 200000);
        pnh_.param("object_timeout", object_timeout_, 0.40);
        pnh_.param("publish_rate", publish_rate_, 2.0);
        pnh_.param("max_voxels", max_voxels_, 500000);
        // Opt-in for the exploration demo: a track is a hypothesis, NOT
        // permission to erase a cylinder through a permanently observed rack.
        pnh_.param("evidence_only_clearing", evidence_only_clearing_, false);
        pnh_.param("stable_free_miss_threshold", stable_free_misses_, 6);
        pnh_.param("free_evidence_max_gap", evidence_max_gap_, 1.0);
        pnh_.param("free_evidence_min_age", evidence_min_age_, 0.5);
        pnh_.param("free_ray_point_tolerance", ray_point_tolerance_, 0.06);
        pnh_.param("clearing_odom_max_dt", clearing_odom_max_dt_, 0.08);
        pnh_.param("enable_ghost_revisit", enable_ghost_revisit_, false);
        pnh_.param("ghost_suspect_max_hits", ghost_suspect_max_hits_, 8);
        pnh_.param("ghost_confirm_hits", ghost_confirm_hits_, 3);
        pnh_.param("ghost_confirm_baseline", ghost_confirm_baseline_, 0.5);
        pnh_.param("ghost_departure_seconds", ghost_departure_seconds_, 2.0);
        pnh_.param("ghost_positive_endpoint_confirmation",ghost_positive_endpoint_confirmation_,true);
        pnh_.param("ghost_static_surface_guard",ghost_static_surface_guard_,false);
        pnh_.param("ghost_surface_distance",ghost_surface_distance_,0.04);
        pnh_.param("ghost_surface_radius",ghost_surface_radius_,0.9);
        pnh_.param("ghost_surface_plane_tolerance",ghost_surface_plane_tolerance_,0.025);
        pnh_.param("ghost_surface_min_span",ghost_surface_min_span_,0.7);
        pnh_.param("ghost_surface_stable_age",ghost_surface_stable_age_,0.5);
        pnh_.param("ghost_surface_min_points",ghost_surface_min_points_,18);
        if (!std::isfinite(ghost_surface_distance_) || ghost_surface_distance_<=0. ||
            ghost_surface_distance_>0.06 || !std::isfinite(ghost_surface_radius_) ||
            ghost_surface_radius_<0.5 || ghost_surface_radius_>1.2 ||
            !std::isfinite(ghost_surface_plane_tolerance_) || ghost_surface_plane_tolerance_<=0. ||
            ghost_surface_plane_tolerance_>0.04 || !std::isfinite(ghost_surface_min_span_) ||
            ghost_surface_min_span_<0.6 || ghost_surface_min_span_>1.5 ||
            !std::isfinite(ghost_surface_stable_age_) || ghost_surface_stable_age_<0. ||
            ghost_surface_min_points_<12 || ghost_surface_min_points_>128)
            throw std::invalid_argument("invalid conservative ghost surface guard parameters");
        pnh_.param("max_ghost_suspects", max_ghost_suspects_, 10000);
        pnh_.param("max_ghost_history", max_ghost_history_, 10000);
        // FAST-LIO registered returns and odometry share the end-of-scan IMU
        // frame. Rotate the SAME configured LiDAR-to-IMU translation into map.
        std::vector<double> extrinsic;
        if (nh_.getParam("/mapping/extrinsic_T", extrinsic) && extrinsic.size() == 3)
            lidar_offset_ = Eigen::Vector3d(extrinsic[0], extrinsic[1], extrinsic[2]);
        // A bridge may publish vehicle-body odometry instead of raw IMU
        // odometry. Its ray origin must then use laser-in-BODY calibration.
        if (pnh_.hasParam("lidar_origin_in_odom"))
        {
            if (!pnh_.getParam("lidar_origin_in_odom", extrinsic) || extrinsic.size()!=3 ||
                !std::isfinite(extrinsic[0]) || !std::isfinite(extrinsic[1]) || !std::isfinite(extrinsic[2]))
                throw std::invalid_argument("lidar_origin_in_odom must be three finite coordinates");
            lidar_offset_ = Eigen::Vector3d(extrinsic[0],extrinsic[1],extrinsic[2]);
            explicit_lidar_origin_ = true;
        }
        stable_free_misses_ = std::max(1, std::min(65535, stable_free_misses_));
        evidence_max_gap_ = std::max(0.05, evidence_max_gap_);
        evidence_min_age_ = std::max(0.0, evidence_min_age_);
        ray_point_tolerance_ = std::max(0.01, std::min(0.1, ray_point_tolerance_));
        clearing_odom_max_dt_ = std::max(0.001, clearing_odom_max_dt_);
        ghost_suspect_max_hits_ = std::max(1, ghost_suspect_max_hits_);
        ghost_confirm_hits_ = std::max(2, ghost_confirm_hits_);
        ghost_confirm_baseline_ = std::max(0.1, ghost_confirm_baseline_);
        ghost_departure_seconds_ = std::max(0.5, ghost_departure_seconds_);
        max_ghost_suspects_ = std::max(1, max_ghost_suspects_);
        max_ghost_history_ = std::max(1, max_ghost_history_);
        if (enable_ghost_revisit_ &&
            (!evidence_only_clearing_ || !enable_dynamic_map_clearing_))
        {
            ROS_ERROR("[StaticMapBuilder] ghost revisit requires evidence-only clearing and dynamic tracks; disabling revisit status");
            enable_ghost_revisit_ = false;
        }

        voxel_size_ = std::max(0.03, voxel_size_);
        dynamic_mask_radius_ = std::max(voxel_size_, dynamic_mask_radius_);
        trail_step_ = std::max(voxel_size_, trail_step_);
        minimum_static_hits_ = std::max(1, minimum_static_hits_);
        stable_minimum_static_hits_ = std::max(
            minimum_static_hits_, stable_minimum_static_hits_);
        local_minimum_hits_ = std::max(1, local_minimum_hits_);
        local_max_age_frames_ = std::max(1, local_max_age_frames_);
        local_radius_ = std::max(1.0, local_radius_);
        free_space_miss_threshold_ = std::max(1, free_space_miss_threshold_);
        free_space_endpoint_margin_ = std::max(
            voxel_size_, free_space_endpoint_margin_);
        free_space_max_range_ = std::max(1.0, free_space_max_range_);
        free_space_ray_step_ = std::max(voxel_size_, free_space_ray_step_);
        max_free_rays_ = std::max(100, max_free_rays_);
        max_free_voxels_ = std::max(1000, max_free_voxels_);
        dynamic_history_seconds_ = std::max(1.0, dynamic_history_seconds_);
        publish_rate_ = std::max(0.2, publish_rate_);

        // The online global coordinator uses independent instances of this
        // SAME evidence engine. They have no ROS subscriptions/publishers and
        // consume event time, so background historical rebuilds cannot be
        // corrupted by the current wall/simulator clock.
        if (headless_) return;

        cloud_sub_ = nh_.subscribe(cloud_topic_, 2,
            &StaticMapBuilder::cloudCallback, this);
        objects_sub_ = nh_.subscribe(objects_topic_, 10,
            &StaticMapBuilder::objectsCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 20,
            &StaticMapBuilder::odomCallback, this);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1, true);
        if (!legacy_output_topic_.empty() &&
            legacy_output_topic_ != output_topic_)
        {
            legacy_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                legacy_output_topic_, 1, true);
            publish_legacy_map_ = true;
        }
        stable_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            stable_output_topic_, 1, true);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            local_output_topic_, 1, true);
        free_space_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            free_space_output_topic_, 1, true);
        suspect_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            suspect_output_topic_, 1, true);
        suspect_status_pub_ = nh_.advertise<std_msgs::String>(
            suspect_status_topic_, 1, true);
        ghost_history_pub_ = nh_.advertise<std_msgs::String>(
            ghost_history_topic_, 1, true);
        processed_scan_pub_ = nh_.advertise<std_msgs::Header>(
            "/uav1/map/processed_scan", 10);
        processed_odom_pub_ = nh_.advertise<std_msgs::Header>(
            "/uav1/map/processed_odom", 10);
        processed_objects_pub_ = nh_.advertise<std_msgs::Header>(
            "/uav1/map/processed_objects", 10);
        clear_service_ = pnh_.advertiseService("clear",
            &StaticMapBuilder::clearCallback, this);
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
            &StaticMapBuilder::timerCallback, this);

        ROS_INFO("[StaticMapBuilder] scan=%s objects=%s global=%s stable=%s "
                 "local=%s free=%s voxel=%.2f mask=%.2f history=%.1fs",
                 cloud_topic_.c_str(), objects_topic_.c_str(),
                 output_topic_.c_str(), stable_output_topic_.c_str(),
                 local_output_topic_.c_str(), free_space_output_topic_.c_str(),
                 voxel_size_,
                 dynamic_mask_radius_, dynamic_history_seconds_);
    }

    void ingest(const sensor_msgs::PointCloud2::ConstPtr& scan,
                const nav_msgs::Odometry::ConstPtr& odom,
                const fastlio_bridge::DynamicObjectArray::ConstPtr& objects,
                bool accumulate_free)
    {
        if (!headless_) throw std::logic_error("ingest is for isolated evidence engines");
        processing_stamp_ = scan->header.stamp;
        odomCallback(odom);
        objectsCallback(objects);
        cloudCallback(scan);
        if (accumulate_free)
            for (const auto& key : latest_free_cells_)
            {
                if (observed_free_cells_.size()>=1000000U && !observed_free_cells_.count(key))
                    { observed_free_capacity_limited_=true; continue; }
                observed_free_cells_.insert(key);
            }
    }

    Snapshot snapshot(double planning_altitude, bool cumulative_free=true,
                      bool full_height_free=false)
    {
        if (!headless_) throw std::logic_error("snapshot is for isolated evidence engines");
        timerCallback(ros::TimerEvent());
        if (cumulative_free)
        {
            CloudT free;
            for (const auto& key : observed_free_cells_)
            {
                const auto point=centerFor(key);
                if (!full_height_free && std::abs(point.z()-planning_altitude)>.22) continue;
                free.push_back(PointT(float(point.x()),float(point.y()),float(point.z())));
            }
            pcl::toROSMsg(free,last_snapshot_.free);
            last_snapshot_.free.header=last_snapshot_.map.header;
        }
        return last_snapshot_;
    }

    bool freeCapacityLimited() const { return observed_free_capacity_limited_; }

    std::uint64_t processedFrames() const { return frame_index_; }

private:
    static std::string normalizeFrame(std::string frame)
    {
        while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
        return frame;
    }

    Key keyFor(const Eigen::Vector3d& p) const
    {
        return {static_cast<int>(std::floor(p.x() / voxel_size_)),
                static_cast<int>(std::floor(p.y() / voxel_size_)),
                static_cast<int>(std::floor(p.z() / voxel_size_))};
    }

    Eigen::Vector3d centerFor(const Key& key) const
    {
        return Eigen::Vector3d((static_cast<double>(key.x) + 0.5) * voxel_size_,
                               (static_cast<double>(key.y) + 0.5) * voxel_size_,
                               (static_cast<double>(key.z) + 0.5) * voxel_size_);
    }

    bool nearConfirmedObject(const Eigen::Vector3d& point,
                             const ros::Time& now) const
    {
        if (!enable_ghost_revisit_ || !have_objects_ ||
            (now - objects_receive_time_).toSec() > object_timeout_)
            return false;
        for (const auto& object : latest_objects_.objects)
        {
            if (object.predicted) continue;
            const Eigen::Vector3d center(object.pose.position.x,
                object.pose.position.y, object.pose.position.z);
            if (std::abs(point.z()-center.z()) <= dynamic_mask_half_height_ &&
                (point.head<2>()-center.head<2>()).norm() <= dynamic_mask_radius_)
                return true;
        }
        return false;
    }

    GhostHistory* historyFor(const Key& key, const ros::Time& now)
    {
        auto found = ghost_history_.find(key);
        if (found != ghost_history_.end()) return &found->second;
        if (ghost_history_.size() >= static_cast<std::size_t>(max_ghost_history_))
        {
            auto oldest = ghost_history_.end();
            for (auto it = ghost_history_.begin(); it != ghost_history_.end(); ++it)
                if (it->second.state != 0 &&
                    (oldest == ghost_history_.end() ||
                     it->second.last_event < oldest->second.last_event))
                    oldest = it;
            if (oldest == ghost_history_.end())
            {
                ++ghost_history_overflow_;
                return nullptr;
            }
            ghost_history_.erase(oldest);
            ++ghost_history_evicted_;
        }
        auto result = ghost_history_.emplace(key, GhostHistory{});
        result.first->second.first_suspected = now;
        return &result.first->second;
    }

    void markGhostSuspect(const Key& key, Cell& cell, const ros::Time& now)
    {
        if (!cell.suspect)
        {
            // A lifetime hits cap missed long-observed people. High-support
            // surfaces remain candidates but get lower revisit priority; the
            // track never deletes their measured occupancy.
            if (suspect_count_ >= static_cast<std::size_t>(max_ghost_suspects_))
                return;
            cell.suspect = true;
            cell.ever_ghost_suspect = true;
            ++suspect_count_;
            if (GhostHistory* history = historyFor(key, now))
            {
                ++history->suspected_episodes;
                history->free_evidence_frames = 0;
                history->sampled_ray_crossing_frames = 0;
                history->off_point_ray_frames = 0;
                history->protected_endpoint_frames = 0;
                history->close_endpoint_frames = 0;
                history->closest_endpoint_distance = std::numeric_limits<double>::infinity();
                history->close_endpoint_baseline = 0.0;
                history->first_close_endpoint_origin = Eigen::Vector3d::Zero();
                history->post_departure_hit_frames = 0;
                history->last_sampled_ray_crossing = ros::Time();
                history->last_off_point_ray = ros::Time();
                history->last_free_evidence = ros::Time();
                history->last_protected_endpoint = ros::Time();
                history->last_close_endpoint = ros::Time();
                history->last_post_departure_hit = ros::Time();
                cell.previous_ghost_clears = history->cleared_episodes;
                history->state = 0;
                history->last_event = now;
                history->encounter_started = now;
                history->surface_positive_frames = 0;
                history->surface_suppressed = false;
                history->surface_checked = ros::Time();
            }
        }
        else if ((now-cell.last_dynamic_seen).toSec() >= ghost_departure_seconds_)
        {
            // A new track encounter invalidates the old post-departure
            // boundary-support window. Do not discount a fresh possible
            // moving return using static evidence from before this encounter.
            auto found = ghost_history_.find(key);
            if (found != ghost_history_.end())
            {
                GhostHistory& history = found->second;
                history.close_endpoint_frames = 0;
                history.closest_endpoint_distance = std::numeric_limits<double>::infinity();
                history.close_endpoint_baseline = 0.0;
                history.first_close_endpoint_origin = Eigen::Vector3d::Zero();
                history.last_close_endpoint = ros::Time();
                history.encounter_started = now;
            }
        }
        cell.last_dynamic_seen = now;
        cell.post_dynamic_hits = 0;
        if (auto* h=historyFor(key,now))
        {
            h->positive_endpoint_frames=0;h->positive_endpoint_baseline=0.;
            h->positive_endpoint_distance=std::numeric_limits<double>::infinity();
            h->first_positive_origin=Eigen::Vector3d::Zero();h->resolution_reason="suspected";
            h->surface_positive_frames=0;h->surface_suppressed=false;
            h->surface_checked=ros::Time();h->surface=astra::SurfaceEvidence();
            h->surface_consistent_frames=0;h->surface_evidence_frame=0;
            h->surface_last_support=ros::Time();
        }
        auto found = ghost_history_.find(key);
        if (found != ghost_history_.end())
            found->second.last_dynamic = now;
    }

    void retireGhostSuspect(const Key& key, Cell& cell, bool cleared,
                            const ros::Time& now)
    {
        if (!cell.suspect) return;
        cell.suspect = false;
        --suspect_count_;
        if (cleared) ++suspects_cleared_;
        else ++suspects_reobserved_;
        auto found = ghost_history_.find(key);
        if (found != ghost_history_.end())
        {
            GhostHistory& history = found->second;
            history.state = cleared ? 1 : 2;
            history.surface_suppressed = false;
            history.resolution_reason=cleared ? "point_aligned_free_rays" : "same_voxel_multiview";
            history.last_event = now;
            if (cleared)
            {
                ++history.cleared_episodes;
                cell.previous_ghost_clears = history.cleared_episodes;
            }
            else ++history.retained_episodes;
        }
    }

    void updateFreeSpace(
        const std::unordered_map<Key, FrameCell, KeyHash>& frame_cells)
    {
        latest_free_cells_.clear();
        if (!enable_free_space_clearing_ || !have_odom_) return;

        Eigen::Vector3d sensor_origin = uav_position_;
        if (explicit_lidar_origin_)
        {
            if (odom_history_.empty()) return;
            const auto& o=odom_history_.back()->pose.pose.orientation;
            Eigen::Quaterniond q(o.w,o.x,o.y,o.z);
            if (!q.coeffs().allFinite() || q.norm()<1e-6) return;
            sensor_origin += q.normalized()*lidar_offset_;
        }
        sensor_origin.z() += sensor_origin_z_offset_;
        latest_free_cells_.reserve(std::min<std::size_t>(
            frame_cells.size() * 8U,
            static_cast<std::size_t>(max_free_voxels_)));

        const std::size_t ray_stride = std::max<std::size_t>(
            1U, (frame_cells.size() + static_cast<std::size_t>(max_free_rays_) - 1U) /
                    static_cast<std::size_t>(max_free_rays_));
        std::size_t ray_index = 0;
        for (const auto& item : frame_cells)
        {
            if ((ray_index++ % ray_stride) != 0U) continue;
            const Eigen::Vector3d endpoint =
                item.second.sum / static_cast<double>(item.second.count);
            const Eigen::Vector3d ray = endpoint - sensor_origin;
            const double distance = ray.norm();
            const double trace_length = std::min(
                free_space_max_range_, distance - free_space_endpoint_margin_);
            if (distance < 1e-6 || trace_length <= free_space_ray_step_) continue;
            const Eigen::Vector3d direction = ray / distance;
            const int samples = static_cast<int>(std::floor(
                trace_length / free_space_ray_step_));
            for (int i = 1; i <= samples; ++i)
            {
                latest_free_cells_.insert(keyFor(
                    sensor_origin + direction * (i * free_space_ray_step_)));
                if (latest_free_cells_.size() >=
                    static_cast<std::size_t>(max_free_voxels_))
                    break;
            }
            if (latest_free_cells_.size() >=
                static_cast<std::size_t>(max_free_voxels_))
                break;
        }

        // A measured endpoint always wins over a traversed ray in the same
        // frame. This protects thin real structures from accidental clearing.
        for (const auto& item : frame_cells)
            latest_free_cells_.erase(item.first);

        for (const auto& key : latest_free_cells_)
        {
            auto cell = cells_.find(key);
            if (cell == cells_.end()) continue;
            cell->second.free_misses = static_cast<std::uint16_t>(std::min<int>(
                free_space_miss_threshold_, cell->second.free_misses + 1));
            if (cell->second.free_misses >= free_space_miss_threshold_)
                cells_.erase(cell);
        }
    }

    bool maskedByDynamicObject(const Eigen::Vector3d& point,
                               const ros::Time& now) const
    {
        const double radius_sq = dynamic_mask_radius_ * dynamic_mask_radius_;
        // Clear the complete recently observed trajectory.  Looking only behind
        // the latest velocity estimate leaves gaps whenever a track is briefly
        // lost or changes direction, which is perceived by A* as a false wall.
        for (const auto& item : dynamic_trails_)
        {
            for (const auto& sample : item.second)
            {
                if ((now - sample.stamp).toSec() > dynamic_history_seconds_)
                    continue;
                if (std::abs(point.z() - sample.center.z()) >
                    dynamic_mask_half_height_)
                    continue;
                if ((point.head<2>() - sample.center.head<2>()).squaredNorm() <=
                    radius_sq)
                    return true;
            }
        }

        if (!have_objects_ ||
            (now - objects_receive_time_).toSec() > object_timeout_)
            return false;

        for (const auto& object : latest_objects_.objects)
        {
            const Eigen::Vector3d center(object.pose.position.x,
                                         object.pose.position.y,
                                         object.pose.position.z);
            const Eigen::Vector3d velocity(object.twist.linear.x,
                                           object.twist.linear.y,
                                           object.twist.linear.z);
            const double speed = velocity.head<2>().norm();
            const int samples = speed > 0.05
                ? std::max(1, static_cast<int>(std::ceil(
                      speed * backward_trail_seconds_ / trail_step_)))
                : 1;

            for (int i = 0; i <= samples; ++i)
            {
                const double t = samples > 0
                    ? backward_trail_seconds_ * static_cast<double>(i) /
                          static_cast<double>(samples)
                    : 0.0;
                const Eigen::Vector3d trail_center = center - velocity * t;
                if (std::abs(point.z() - trail_center.z()) >
                    dynamic_mask_half_height_)
                    continue;
                if ((point.head<2>() - trail_center.head<2>()).squaredNorm() <=
                    radius_sq)
                    return true;
            }
        }
        return false;
    }

    void clearDynamicVoxels(const ros::Time& now)
    {
        if (evidence_only_clearing_) return;
        const auto erase_cylinder = [&](const Eigen::Vector3d& center)
        {
            const Key key = keyFor(center);
            const int radius_cells = static_cast<int>(std::ceil(
                dynamic_mask_radius_ / voxel_size_)) + 1;
            const int height_cells = static_cast<int>(std::ceil(
                dynamic_mask_half_height_ / voxel_size_)) + 1;
            const double conservative_radius = dynamic_mask_radius_ + voxel_size_;
            for (int dz = -height_cells; dz <= height_cells; ++dz)
                for (int dy = -radius_cells; dy <= radius_cells; ++dy)
                    for (int dx = -radius_cells; dx <= radius_cells; ++dx)
                    {
                        if (std::hypot(dx * voxel_size_, dy * voxel_size_) >
                            conservative_radius)
                            continue;
                        cells_.erase({key.x + dx, key.y + dy, key.z + dz});
                    }
        };

        for (const auto& item : dynamic_trails_)
            for (const auto& sample : item.second)
                if ((now - sample.stamp).toSec() <= dynamic_history_seconds_)
                    erase_cylinder(sample.center);

        if (!have_objects_ ||
            (now - objects_receive_time_).toSec() > object_timeout_)
            return;
        for (const auto& object : latest_objects_.objects)
        {
            const Eigen::Vector3d center(object.pose.position.x,
                                         object.pose.position.y,
                                         object.pose.position.z);
            const Eigen::Vector3d velocity(object.twist.linear.x,
                                           object.twist.linear.y,
                                           object.twist.linear.z);
            const double speed = velocity.head<2>().norm();
            const int samples = speed > 0.05
                ? std::max(1, static_cast<int>(std::ceil(
                      speed * backward_trail_seconds_ / trail_step_)))
                : 1;
            for (int i = 0; i <= samples; ++i)
            {
                const double t = backward_trail_seconds_ *
                    static_cast<double>(i) / static_cast<double>(samples);
                erase_cylinder(center - velocity * t);
            }
        }
    }

    void objectsCallback(const fastlio_bridge::DynamicObjectArray::ConstPtr& msg)
    {
        if (!enable_dynamic_map_clearing_) {
            publishInputReceipt(processed_objects_pub_,msg->header,++objects_input_count_);
            return;
        }
        if (enable_ghost_revisit_ &&
            normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_))
        {
            ROS_ERROR_THROTTLE(5.0,"[StaticMapBuilder] reject ghost track outside map frame");
            return;
        }
        latest_objects_ = *msg;
        objects_receive_time_ = headless_ ? processing_stamp_ : ros::Time::now();
        have_objects_ = true;
        pruneDynamicTrails(objects_receive_time_);
        for (const auto& object : msg->objects)
        {
            // Predictions share the same track id but are future positions.  Do
            // not record them as already traversed space.
            if (object.predicted) continue;
            const Eigen::Vector3d center(object.pose.position.x,
                                         object.pose.position.y,
                                         object.pose.position.z);
            if (!center.allFinite()) continue;
            if (enable_ghost_revisit_)
            {
                const Key key = keyFor(center);
                const int radius = static_cast<int>(std::ceil(dynamic_mask_radius_/voxel_size_))+1;
                const int height = static_cast<int>(std::ceil(dynamic_mask_half_height_/voxel_size_))+1;
                for (int dz=-height; dz<=height; ++dz)
                    for (int dy=-radius; dy<=radius; ++dy)
                        for (int dx=-radius; dx<=radius; ++dx)
                        {
                            auto it = cells_.find({key.x+dx,key.y+dy,key.z+dz});
                            if (it == cells_.end() ||
                                std::abs((objects_receive_time_-it->second.last_seen).toSec()) > 1.0 ||
                                !nearConfirmedObject(it->second.mean, objects_receive_time_))
                                continue;
                            markGhostSuspect(it->first, it->second, objects_receive_time_);
                        }
            }
            auto& trail = dynamic_trails_[object.id];
            if (trail.empty() ||
                (trail.back().center.head<2>() - center.head<2>()).norm() >=
                    trail_step_)
                trail.push_back({center, objects_receive_time_});
            else
            {
                trail.back().center = center;
                trail.back().stamp = objects_receive_time_;
            }
        }
        clearDynamicVoxels(objects_receive_time_);
        publishInputReceipt(processed_objects_pub_,msg->header,++objects_input_count_);
    }

    static void publishInputReceipt(ros::Publisher& publisher,
                                   const std_msgs::Header& header,std::uint32_t sequence)
    {
        std_msgs::Header receipt = header;
        receipt.seq = sequence;
        if (publisher) publisher.publish(receipt);
    }

    void pruneDynamicTrails(const ros::Time& now)
    {
        for (auto it = dynamic_trails_.begin(); it != dynamic_trails_.end();)
        {
            auto& trail = it->second;
            while (!trail.empty() &&
                   (now - trail.front().stamp).toSec() > dynamic_history_seconds_)
                trail.pop_front();
            if (trail.empty())
                it = dynamic_trails_.erase(it);
            else
                ++it;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_)) return;
        odom_history_.push_back(msg);
        while (odom_history_.size() > 100) odom_history_.pop_front();
        uav_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z);
        have_odom_ = true;
        if (pending_cloud_ && std::abs((pending_cloud_->header.stamp-msg->header.stamp).toSec()) <= clearing_odom_max_dt_)
        {
            const auto pending = pending_cloud_;
            pending_cloud_.reset();
            cloudCallback(pending);
        }
        publishInputReceipt(processed_odom_pub_,msg->header,++odom_input_count_);
    }

    void updateEvidenceFreeSpace(const CloudT& cloud,
        const std::unordered_map<Key, FrameCell, KeyHash>& endpoints,
        const ros::Time& stamp)
    {
        latest_free_cells_.clear();
        if (!enable_free_space_clearing_ || stamp.isZero()) return;
        nav_msgs::Odometry::ConstPtr nearest;
        double best_dt = std::numeric_limits<double>::infinity();
        for (const auto& odom : odom_history_)
        {
            const double dt = std::abs((odom->header.stamp - stamp).toSec());
            if (dt < best_dt) { best_dt = dt; nearest = odom; }
        }
        if (!nearest || best_dt > clearing_odom_max_dt_)
        {
            ROS_WARN_THROTTLE(5.0, "[StaticMapBuilder] skip clearing: no time-matched odom (dt=%.3f)", best_dt);
            return; // Never manufacture free evidence from stale/mismatched pose.
        }
        const auto& pose = nearest->pose.pose;
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        if (!q.coeffs().allFinite() || q.norm() < 1e-6) return;
        const Eigen::Vector3d origin = Eigen::Vector3d(pose.position.x, pose.position.y,
            pose.position.z) + q.normalized() * lidar_offset_;
        if (!origin.allFinite()) return;

        // Voxel-neighbour protection handles surfaces straddling voxel borders.
        // All real returns count, including ones inside a dynamic track mask.
        std::unordered_set<Key, KeyHash> protected_cells, checked_cells;
        std::unordered_set<Key, KeyHash> sampled_crossings, point_aligned_crossings;
        std::unordered_map<Key, double, KeyHash> close_endpoint_distances;
        std::unordered_set<Key, KeyHash> evidence;
        const std::size_t stride = std::max<std::size_t>(1,
            (cloud.size()+max_free_rays_-1)/max_free_rays_);
        const double step = voxel_size_ * 0.5;
        for (std::size_t i=0; i<cloud.size(); i+=stride)
        {
            const auto& p = cloud[i];
            const Eigen::Vector3d ray = Eigen::Vector3d(p.x,p.y,p.z)-origin;
            const double range = ray.norm();
            if (!ray.allFinite() || range < 1e-6) continue;
            const Eigen::Vector3d direction = ray/range;
            const double length = std::min(free_space_max_range_,range-free_space_endpoint_margin_);
            for (double d=step; d<=length; d+=step)
            {
                const Key key = keyFor(origin+direction*d);
                if (endpoints.count(key)) continue;
                latest_free_cells_.insert(key);
                auto it = cells_.find(key);
                if (it != cells_.end() &&
                    (it->second.suspect ||
                     (!evidence.count(key) && !protected_cells.count(key))))
                {
                    const Eigen::Vector3d relative = it->second.mean-origin;
                    const double along = relative.dot(direction);
                    // A ray passing elsewhere in the same voxel is NOT a
                    // contradiction of this point (thin shelves, leaves, edges).
                    const bool point_aligned = along>0 && along<length &&
                        (relative-direction*along).norm() <= ray_point_tolerance_;
                    if (it->second.suspect &&
                        (stamp-it->second.last_dynamic_seen).toSec() >= ghost_departure_seconds_)
                    {
                        sampled_crossings.insert(key);
                        if (point_aligned) point_aligned_crossings.insert(key);
                    }
                    if (point_aligned && !evidence.count(key) && !protected_cells.count(key))
                    {
                        if (checked_cells.insert(key).second)
                            for (int dz=-1; dz<=1; ++dz)
                                for (int dy=-1; dy<=1; ++dy)
                                    for (int dx=-1; dx<=1; ++dx)
                                    {
                                        const auto endpoint = endpoints.find(
                                            {key.x+dx,key.y+dy,key.z+dz});
                                        if (endpoint != endpoints.end())
                                        {
                                            protected_cells.insert(key);
                                            if (it->second.suspect)
                                            {
                                                const Eigen::Vector3d measured =
                                                    endpoint->second.sum /
                                                    static_cast<double>(endpoint->second.count);
                                                const double separation =
                                                    (measured-it->second.mean).norm();
                                                if (separation <= voxel_size_*0.6)
                                                {
                                                    auto old = close_endpoint_distances.find(key);
                                                    if (old == close_endpoint_distances.end() ||
                                                        separation < old->second)
                                                        close_endpoint_distances[key] = separation;
                                                }
                                            }
                                        }
                                    }
                        if (!protected_cells.count(key)) evidence.insert(key);
                    }
                }
                if (latest_free_cells_.size() >= static_cast<std::size_t>(max_free_voxels_)) break;
            }
            if (latest_free_cells_.size() >= static_cast<std::size_t>(max_free_voxels_)) break;
        }
        std::size_t erased=0;
        for (const auto& key : sampled_crossings)
        {
            auto history = ghost_history_.find(key);
            if (history == ghost_history_.end()) continue;
            ++history->second.sampled_ray_crossing_frames;
            history->second.last_sampled_ray_crossing = stamp;
            if (!point_aligned_crossings.count(key))
            {
                ++history->second.off_point_ray_frames;
                history->second.last_off_point_ray = stamp;
            }
        }
        for (const auto& key : protected_cells)
        {
            auto cell = cells_.find(key);
            auto history = ghost_history_.find(key);
            if (cell != cells_.end() && cell->second.suspect &&
                history != ghost_history_.end())
            {
                ++history->second.protected_endpoint_frames;
                history->second.last_protected_endpoint = stamp;
                const auto close = close_endpoint_distances.find(key);
                if (close != close_endpoint_distances.end() &&
                    (stamp-cell->second.last_dynamic_seen).toSec() >= ghost_departure_seconds_)
                {
                    GhostHistory& record = history->second;
                    if (record.close_endpoint_frames == 0)
                        record.first_close_endpoint_origin = origin;
                    ++record.close_endpoint_frames;
                    record.closest_endpoint_distance = std::min(
                        record.closest_endpoint_distance, close->second);
                    record.close_endpoint_baseline = std::max(
                        record.close_endpoint_baseline,
                        (origin-record.first_close_endpoint_origin).norm());
                    record.last_close_endpoint = stamp;
                }
            }
        }
        for (const auto& key : evidence)
        {
            // A different ray may discover the neighboring real endpoint
            // after this voxel was first inserted in evidence.  Protection
            // must win regardless of the point order within the scan.
            if (protected_cells.count(key)) continue;
            auto it = cells_.find(key);
            if (it == cells_.end()) continue;
            Cell& cell = it->second;
            if ((stamp-cell.last_seen).toSec() < evidence_min_age_) continue;
            if (cell.suspect)
            {
                auto history = ghost_history_.find(key);
                if (history != ghost_history_.end())
                {
                    ++history->second.free_evidence_frames;
                    history->second.last_free_evidence = stamp;
                    history->second.surface_positive_frames = 0;
                    history->second.surface_suppressed = false;
                    history->second.surface_checked = ros::Time();
                    history->second.surface_consistent_frames = 0;
                    history->second.surface_evidence_frame = 0;
                    history->second.surface_last_support = ros::Time();
                }
            }
            const double gap = (stamp-cell.last_miss).toSec();
            if (gap<=0 || gap>evidence_max_gap_) cell.free_misses=0;
            cell.last_miss=stamp;
            const int threshold = cell.hits>=static_cast<unsigned>(stable_minimum_static_hits_)
                ? stable_free_misses_ : free_space_miss_threshold_;
            cell.free_misses=static_cast<std::uint16_t>(std::min(threshold,int(cell.free_misses)+1));
            if (cell.free_misses>=threshold)
            {
                retireGhostSuspect(key, cell, true, stamp);
                cells_.erase(it);
                ++erased;
            }
        }
        if (!headless_) ROS_INFO_THROTTLE(2.0,"[StaticMapBuilder] evidence-only erased=%zu ray_candidates=%zu odom_dt=%.4f (no track-cylinder deletion)",
                          erased,evidence.size(),best_dt);
    }

    void confirmPositiveEndpoints(const std::unordered_map<Key,FrameCell,KeyHash>& endpoints,
                                  const ros::Time& stamp)
    {
        if (!enable_ghost_revisit_ || !ghost_positive_endpoint_confirmation_) return;
        for (auto& entry:cells_)
        {
            Cell& cell=entry.second;
            if (!cell.suspect || (stamp-cell.last_dynamic_seen).toSec()<ghost_departure_seconds_) continue;
            auto found=ghost_history_.find(entry.first);
            if (found==ghost_history_.end()) continue;
            double nearest=std::numeric_limits<double>::infinity();
            for (int dz=-1;dz<=1;++dz) for (int dy=-1;dy<=1;++dy) for (int dx=-1;dx<=1;++dx)
            {
                auto endpoint=endpoints.find({entry.first.x+dx,entry.first.y+dy,entry.first.z+dz});
                if (endpoint==endpoints.end()) continue;
                const Eigen::Vector3d point=endpoint->second.sum/static_cast<double>(endpoint->second.count);
                if (nearConfirmedObject(point,stamp)) continue;
                nearest=std::min(nearest,(point-cell.mean).norm());
            }
            // Positive returns must not depend on a free ray going BEHIND an
            // opaque shelf. Keep the same 6cm point tolerance, frame count and
            // view baseline as existing evidence; never erase occupancy here.
            if (nearest>ray_point_tolerance_) continue;
            GhostHistory& history=found->second;
            if (!history.positive_endpoint_frames) history.first_positive_origin=uav_position_;
            ++history.positive_endpoint_frames;
            ++history.surface_positive_frames;
            history.last_positive_endpoint=stamp;
            history.positive_endpoint_distance=std::min(history.positive_endpoint_distance,nearest);
            history.positive_endpoint_baseline=std::max(history.positive_endpoint_baseline,
                (uav_position_-history.first_positive_origin).norm());
            if (history.positive_endpoint_frames>=static_cast<unsigned>(ghost_confirm_hits_) &&
                history.positive_endpoint_baseline>=ghost_confirm_baseline_)
            {
                retireGhostSuspect(entry.first,cell,false,stamp);
                history.resolution_reason="measured_near_point_multiview";
            }
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        if (normalizeFrame(msg->header.frame_id) != normalizeFrame(world_frame_))
        {
            ROS_ERROR_THROTTLE(1.0,
                "[StaticMapBuilder] cloud frame '%s' != '%s'; no relabeling is performed.",
                msg->header.frame_id.c_str(), world_frame_.c_str());
            return;
        }

        if (evidence_only_clearing_)
        {
            bool matched = false;
            for (const auto& odom : odom_history_)
                if (std::abs((odom->header.stamp-msg->header.stamp).toSec()) <= clearing_odom_max_dt_)
                    { matched=true; break; }
            if (!matched)
            {
                // Cloud and odom travel through different ROS connections.
                // A bounded latest-only pending scan tolerates callback order
                // without using the previous scan's pose or an unbounded queue.
                pending_cloud_=msg;
                ROS_WARN_THROTTLE(5.0,"[StaticMapBuilder] waiting for scan-matched odometry");
                return;
            }
            pending_cloud_.reset();
        }

        CloudT cloud;
        pcl::fromROSMsg(*msg, cloud);
        const ros::Time now = headless_ ? processing_stamp_ : ros::Time::now();
        ++frame_index_;
        if (enable_dynamic_map_clearing_)
        {
            pruneDynamicTrails(now);
            clearDynamicVoxels(now);
        }

        std::size_t accepted = 0;
        std::unordered_map<Key, FrameCell, KeyHash> frame_cells;
        frame_cells.reserve(cloud.size());
        for (const auto& p : cloud.points)
        {
            const Eigen::Vector3d point(p.x, p.y, p.z);
            if (!point.allFinite() ||
                (!evidence_only_clearing_ && enable_dynamic_map_clearing_ && maskedByDynamicObject(point, now)))
                continue;
            FrameCell& frame_cell = frame_cells[keyFor(point)];
            frame_cell.sum += point;
            ++frame_cell.count;
            ++accepted;
        }

        // hits counts independent LiDAR frames, not raw points.  Otherwise a
        // dense pedestrian return can become "stable" in a single scan.
        for (const auto& item : frame_cells)
        {
            const Eigen::Vector3d observation =
                item.second.sum / static_cast<double>(item.second.count);
            Cell& cell = cells_[item.first];
            const bool near_dynamic = nearConfirmedObject(observation, now);
            if (near_dynamic)
                markGhostSuspect(item.first, cell, now);
            else if (cell.suspect &&
                     (now-cell.last_dynamic_seen).toSec() >= ghost_departure_seconds_)
            {
                auto history = ghost_history_.find(item.first);
                if (history != ghost_history_.end())
                {
                    ++history->second.post_departure_hit_frames;
                    history->second.last_post_departure_hit = msg->header.stamp;
                }
                if (cell.post_dynamic_hits == 0)
                    cell.first_post_origin = uav_position_;
                cell.post_dynamic_hits = static_cast<std::uint16_t>(std::min<int>(
                    ghost_confirm_hits_,cell.post_dynamic_hits+1));
                if (cell.post_dynamic_hits >= ghost_confirm_hits_ &&
                    (uav_position_-cell.first_post_origin).norm() >= ghost_confirm_baseline_)
                    retireGhostSuspect(item.first, cell, false, now);
            }
            ++cell.hits;
            if (cell.stable_since.isZero() &&
                cell.hits>=static_cast<unsigned>(stable_minimum_static_hits_))
                cell.stable_since=msg->header.stamp;
            cell.last_frame = frame_index_;
            cell.last_seen = msg->header.stamp;
            cell.free_misses = 0;
            const double alpha = 1.0 / static_cast<double>(
                std::min<std::uint32_t>(cell.hits, 20));
            cell.mean = (1.0 - alpha) * cell.mean + alpha * observation;
        }

        confirmPositiveEndpoints(frame_cells,msg->header.stamp);
        // Use current local rays as negative occupancy observations. A stale
        // body/vehicle point is removed after several scans see through it.
        if (evidence_only_clearing_)
            updateEvidenceFreeSpace(cloud, frame_cells, msg->header.stamp);
        else
            updateFreeSpace(frame_cells);

        enforceMemoryLimit(msg->header.stamp);
        last_header_ = msg->header;
        // Offline replay waits for this receipt before sending the next scan.
        // Publish only after all occupancy/free-space updates have completed.
        std_msgs::Header processed = last_header_;
        processed.seq = static_cast<std::uint32_t>(frame_index_);
        if (processed_scan_pub_) processed_scan_pub_.publish(processed);
        if (!headless_) ROS_INFO_THROTTLE(1.0,
            "[StaticMapBuilder] input=%zu accepted=%zu voxels=%zu",
            cloud.size(), accepted, cells_.size());
    }

    void enforceMemoryLimit(const ros::Time& stamp)
    {
        if (max_voxels_ <= 0 || cells_.size() <= static_cast<std::size_t>(max_voxels_))
            return;
        std::vector<std::pair<Key, std::uint64_t>> ages;
        ages.reserve(cells_.size());
        for (const auto& item : cells_) ages.emplace_back(item.first, item.second.last_frame);
        const std::size_t remove_count = cells_.size() - static_cast<std::size_t>(max_voxels_);
        std::nth_element(ages.begin(), ages.begin() + remove_count, ages.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        for (std::size_t i = 0; i < remove_count; ++i)
        {
            auto it = cells_.find(ages[i].first);
            if (it == cells_.end()) continue;
            if (it->second.suspect)
            {
                --suspect_count_;
                ++suspects_evicted_;
                auto history = ghost_history_.find(ages[i].first);
                if (history != ghost_history_.end())
                {
                    history->second.state = 3;
                    history->second.last_event = stamp;
                }
            }
            cells_.erase(it);
        }
    }

    bool surfaceSuppressed(const Key& key) const
    {
        auto history=ghost_history_.find(key);
        return ghost_static_surface_guard_ && history!=ghost_history_.end() &&
            history->second.surface_suppressed;
    }

    void updateSurfaceGuards(const ros::Time& now)
    {
        if (!ghost_static_surface_guard_ || !enable_ghost_revisit_) return;
        // Work is bounded and sorted, so hash ordering cannot starve candidates.
        // An expired/unevaluated certificate always falls back to actionable.
        std::vector<Key> pending;
        for (auto& item:ghost_history_) {
            GhostHistory& h=item.second;
            // A certificate persists until a NEW track/free-ray contradiction.
            // These events revoke it synchronously in their own callbacks.
            if (h.surface_suppressed) continue;
            if (h.surface_evidence_frame==frame_index_ ||
                (!h.surface_checked.isZero() && (now-h.surface_checked).toSec()<.45)) continue;
            auto cell=cells_.find(item.first);
            if (cell==cells_.end() || !cell->second.suspect ||
                h.positive_endpoint_frames==0 ||
                (now-cell->second.last_dynamic_seen).toSec()<ghost_departure_seconds_ ||
                cell->second.free_misses!=0 || h.encounter_started.isZero()) continue;
            pending.push_back(item.first);
        }
        std::sort(pending.begin(),pending.end(),[&](const Key& a,const Key& b) {
            const ros::Time x=ghost_history_.at(a).surface_checked, y=ghost_history_.at(b).surface_checked;
            if (x!=y) return x<y;
            if (a.x!=b.x) return a.x<b.x;
            if (a.y!=b.y) return a.y<b.y;
            return a.z<b.z;
        });
        const int radius=static_cast<int>(std::ceil(ghost_surface_radius_/voxel_size_));
        if (radius>12) return; // unsupported resolution falls back conservatively
        for (std::size_t i=0; i<std::min<std::size_t>(16,pending.size()); ++i) {
            const Key key=pending[i]; GhostHistory& h=ghost_history_.at(key);
            const Eigen::Vector3d query=cells_.at(key).mean;
            std::vector<Eigen::Vector3d> support;
            for (int dz=-radius;dz<=radius;++dz)
                for (int dy=-radius;dy<=radius;++dy)
                    for (int dx=-radius;dx<=radius;++dx) {
                        const auto found=cells_.find({key.x+dx,key.y+dy,key.z+dz});
                        if (found==cells_.end()) continue;
                        const Cell& cell=found->second;
                        // Every supporting point was measured in THIS scan.
                        // Repeated timer publication cannot create evidence.
                        // A newly seen face must first mature in the voxel map;
                        // paused narrow people still fail the 2-D extent test.
                        if (cell.stable_since.isZero() || cell.free_misses ||
                            cell.last_frame!=frame_index_ ||
                            (now-cell.stable_since).toSec()<ghost_surface_stable_age_)
                            continue;
                        const double distance=(cell.mean-query).norm();
                        // Do not let the questioned voxel certify itself.
                        if (distance>=.25 && distance<=ghost_surface_radius_)
                            support.push_back(cell.mean);
                    }
            if (support.size()>512) {
                std::vector<Eigen::Vector3d> bounded; bounded.reserve(512);
                for (std::size_t j=0;j<512;++j) bounded.push_back(support[j*support.size()/512]);
                support=std::move(bounded);
            }
            h.surface=astra::staticSurfaceEvidence(query,support,ghost_surface_distance_,
                ghost_surface_plane_tolerance_,ghost_surface_min_points_,ghost_surface_min_span_);
            h.surface_support_points=support.size();
            h.surface_checked=now;
            h.surface_evidence_frame=frame_index_;
            if (!h.surface.supported || (!h.surface_last_support.isZero() &&
                (now-h.surface_last_support).toSec()>2.)) h.surface_consistent_frames=0;
            if (h.surface.supported) {
                ++h.surface_consistent_frames;
                h.surface_last_support=now;
            }
            h.surface_suppressed=h.surface_consistent_frames>=static_cast<unsigned>(ghost_confirm_hits_);
        }
    }

    void publishGhostHistory(const ros::Time& publish_now)
    {
        if (!enable_ghost_revisit_) return;
        std_msgs::String message;
        std::ostringstream out;
        out << std::setprecision(17)
            << "{\"version\":1,\"frame_id\":\"" << world_frame_
            << "\",\"map_stamp\":" << last_header_.stamp.toSec()
            << ",\"voxel_size\":" << voxel_size_
            << ",\"history_evicted\":" << ghost_history_evicted_
            << ",\"history_overflow\":" << ghost_history_overflow_
            << ",\"records\":[";
        bool first = true;
        for (const auto& item : ghost_history_)
        {
            const Key& key = item.first;
            const GhostHistory& history = item.second;
            auto occupied = cells_.find(key);
            const bool active = occupied != cells_.end() && occupied->second.suspect;
            const Cell* cell = active ? &occupied->second : nullptr;
            const bool actionable = cell &&
                cell->hits >= static_cast<std::uint32_t>(minimum_static_hits_) &&
                (publish_now-cell->last_dynamic_seen).toSec() >= ghost_departure_seconds_ &&
                !surfaceSuppressed(key);
            const bool static_boundary_candidate = active &&
                history.close_endpoint_frames >= static_cast<std::uint32_t>(ghost_confirm_hits_) &&
                history.close_endpoint_baseline >= ghost_confirm_baseline_ &&
                history.free_evidence_frames == 0;
            // Historical clearance is a revisit hint, not permission to call
            // the cell free; a new return always remains an obstacle.
            double priority = 0.0;
            if (actionable)
            {
                const double support = cell->hits > static_cast<std::uint32_t>(ghost_suspect_max_hits_)
                    ? 0.4 : 1.0;
                const double repeat = 1.0 + 0.25*std::min<std::uint32_t>(
                    history.cleared_episodes, 3U);
                const double free_hint = 1.0 + 0.2*std::min<int>(cell->free_misses, 3);
                priority = support*repeat*free_hint*
                    (static_boundary_candidate ? 0.25 : 1.0);
            }
            const Eigen::Vector3d position = occupied != cells_.end()
                ? occupied->second.mean : centerFor(key);
            if (!first) out << ',';
            first = false;
            out << "{\"key\":[" << key.x << ',' << key.y << ',' << key.z
                << "],\"position\":[" << position.x() << ',' << position.y() << ',' << position.z()
                << "],\"state\":\"" << (active ? "suspected" :
                    history.state == 1 ? "cleared" :
                    history.state == 2 ? "reobserved_occupied" : "evicted_unresolved")
                << "\",\"raw_cell_present\":" << (occupied != cells_.end() ? "true" : "false")
                << ",\"occupied_now\":" << (occupied != cells_.end() &&
                    occupied->second.hits >= static_cast<std::uint32_t>(minimum_static_hits_)
                    ? "true" : "false")
                << ",\"actionable\":" << (actionable ? "true" : "false")
                << ",\"static_surface_suppressed\":" << (surfaceSuppressed(key) ? "true" : "false")
                << ",\"candidate_filter_reason\":\"" <<
                    (surfaceSuppressed(key) ? "persistent_measured_vertical_surface" : "none") << "\""
                << ",\"surface_consistent_frames\":" << history.surface_consistent_frames
                << ",\"surface_support_points\":" << history.surface_support_points
                << ",\"surface_positive_frames\":" << history.surface_positive_frames
                << ",\"surface_inliers\":" << history.surface.inliers
                << ",\"surface_distance\":" << history.surface.distance
                << ",\"surface_rms\":" << history.surface.rms
                << ",\"surface_minor_span\":" << history.surface.minor_span
                << ",\"surface_checked\":" << history.surface_checked.toSec()
                << ",\"encounter_started\":" << history.encounter_started.toSec()
                << ",\"last_positive_endpoint\":" << history.last_positive_endpoint.toSec()
                << ",\"static_boundary_candidate\":" <<
                    (static_boundary_candidate ? "true" : "false")
                << ",\"priority\":" << priority
                << ",\"hits\":" << (cell ? cell->hits : 0)
                << ",\"free_misses\":" << (cell ? cell->free_misses : 0)
                << ",\"suspected_episodes\":" << history.suspected_episodes
                << ",\"cleared_episodes\":" << history.cleared_episodes
                << ",\"retained_episodes\":" << history.retained_episodes
                << ",\"free_evidence_frames\":" << history.free_evidence_frames
                << ",\"sampled_ray_crossing_frames\":" << history.sampled_ray_crossing_frames
                << ",\"off_point_ray_frames\":" << history.off_point_ray_frames
                << ",\"protected_endpoint_frames\":" << history.protected_endpoint_frames
                << ",\"close_endpoint_frames\":" << history.close_endpoint_frames
                << ",\"closest_endpoint_distance\":" <<
                    (std::isfinite(history.closest_endpoint_distance) ?
                     history.closest_endpoint_distance : -1.0)
                << ",\"close_endpoint_baseline\":" << history.close_endpoint_baseline
                << ",\"post_departure_hit_frames\":" << history.post_departure_hit_frames
                << ",\"positive_endpoint_frames\":" << history.positive_endpoint_frames
                << ",\"positive_endpoint_baseline\":" << history.positive_endpoint_baseline
                << ",\"positive_endpoint_distance\":" <<
                    (std::isfinite(history.positive_endpoint_distance) ? history.positive_endpoint_distance : -1.)
                << ",\"resolution_reason\":\"" << history.resolution_reason << "\""
                << ",\"last_sampled_ray_crossing\":" << history.last_sampled_ray_crossing.toSec()
                << ",\"last_off_point_ray\":" << history.last_off_point_ray.toSec()
                << ",\"last_free_evidence\":" << history.last_free_evidence.toSec()
                << ",\"last_protected_endpoint\":" << history.last_protected_endpoint.toSec()
                << ",\"last_close_endpoint\":" << history.last_close_endpoint.toSec()
                << ",\"last_post_departure_hit\":" << history.last_post_departure_hit.toSec()
                << ",\"first_suspected\":" << history.first_suspected.toSec()
                << ",\"last_dynamic\":" << history.last_dynamic.toSec()
                << ",\"last_event\":" << history.last_event.toSec() << '}';
        }
        out << "]}";
        message.data = out.str();
        if (headless_) last_snapshot_.history=message;
        else ghost_history_pub_.publish(message);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        CloudT map;
        CloudT stable_map;
        CloudT local_map;
        CloudT free_space;
        CloudT suspects;
        map.reserve(cells_.size());
        stable_map.reserve(cells_.size());
        local_map.reserve(cells_.size());
        free_space.reserve(latest_free_cells_.size());
        suspects.reserve(suspect_count_);
        const std::uint64_t oldest_local_frame =
            frame_index_ > static_cast<std::uint64_t>(local_max_age_frames_)
                ? frame_index_ - static_cast<std::uint64_t>(local_max_age_frames_)
                : 0;
        const double local_radius_sq = local_radius_ * local_radius_;
        const ros::Time publish_now = headless_ ? processing_stamp_ : ros::Time::now();
        updateSurfaceGuards(publish_now);
        for (const auto& item : cells_)
        {
            PointT p;
            p.x = static_cast<float>(item.second.mean.x());
            p.y = static_cast<float>(item.second.mean.y());
            p.z = static_cast<float>(item.second.mean.z());
            if (item.second.suspect &&
                item.second.hits >= static_cast<std::uint32_t>(minimum_static_hits_) &&
                (publish_now-item.second.last_dynamic_seen).toSec() >= ghost_departure_seconds_ &&
                !surfaceSuppressed(item.first))
                suspects.push_back(p);
            if (have_odom_ && item.second.free_misses == 0 &&
                item.second.last_frame >= oldest_local_frame &&
                item.second.hits >= static_cast<std::uint32_t>(local_minimum_hits_) &&
                (item.second.mean.head<2>() - uav_position_.head<2>()).squaredNorm() <=
                    local_radius_sq)
                local_map.push_back(p);

            if (item.second.hits < static_cast<std::uint32_t>(minimum_static_hits_))
                continue;
            map.push_back(p);
            // A second, conservative map is used for dynamic-object veto and
            // global planning.  Requiring many observations prevents a moving
            // pedestrian trail from being mistaken for permanent structure.
            if (item.second.hits >=
                static_cast<std::uint32_t>(stable_minimum_static_hits_))
                stable_map.push_back(p);
        }
        for (const auto& key : latest_free_cells_)
        {
            const Eigen::Vector3d center = centerFor(key);
            PointT p;
            p.x = static_cast<float>(center.x());
            p.y = static_cast<float>(center.y());
            p.z = static_cast<float>(center.z());
            free_space.push_back(p);
        }
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(map, output);
        output.header = last_header_;
        // A timer republishing an old map must not pretend a new scan was
        // integrated. The explorer checks this acquisition time as well.
        output.header.stamp = evidence_only_clearing_ ? last_header_.stamp : ros::Time::now();
        output.header.frame_id = world_frame_;
        if (headless_) last_snapshot_.map=output;
        else map_pub_.publish(output);
        if (publish_legacy_map_) legacy_map_pub_.publish(output);

        sensor_msgs::PointCloud2 stable_output;
        pcl::toROSMsg(stable_map, stable_output);
        stable_output.header = output.header;
        if (headless_) last_snapshot_.stable=stable_output;
        if (!headless_) stable_map_pub_.publish(stable_output);

        sensor_msgs::PointCloud2 local_output;
        pcl::toROSMsg(local_map, local_output);
        local_output.header = output.header;
        if (!headless_) local_map_pub_.publish(local_output);

        sensor_msgs::PointCloud2 free_output;
        pcl::toROSMsg(free_space, free_output);
        free_output.header = output.header;
        if (headless_) last_snapshot_.free=free_output;
        else free_space_pub_.publish(free_output);
        sensor_msgs::PointCloud2 suspect_output;
        pcl::toROSMsg(suspects, suspect_output);
        suspect_output.header = output.header;
        if (headless_) last_snapshot_.suspects=suspect_output;
        else suspect_pub_.publish(suspect_output);
        std_msgs::String suspect_status;
        std::ostringstream status;
        std::size_t unresolved_history = 0;
        std::size_t pending_map_voxels = 0;
        std::size_t pending_unmapped_voxels = 0;
        std::size_t static_boundary_candidates = 0;
        std::size_t static_surface_suppressed = 0;
        for (const auto& item : ghost_history_)
            if (item.second.state == 3) ++unresolved_history;
        for (const auto& item : cells_)
            if (item.second.suspect)
            {
                if (surfaceSuppressed(item.first)) ++static_surface_suppressed;
                auto history = ghost_history_.find(item.first);
                if (history != ghost_history_.end() &&
                    history->second.close_endpoint_frames >=
                        static_cast<std::uint32_t>(ghost_confirm_hits_) &&
                    history->second.close_endpoint_baseline >= ghost_confirm_baseline_ &&
                    history->second.free_evidence_frames == 0)
                    ++static_boundary_candidates;
                if (item.second.hits >= static_cast<std::uint32_t>(minimum_static_hits_))
                    ++pending_map_voxels;
                else
                    ++pending_unmapped_voxels;
            }
        status << "{\"enabled\":" << (enable_ghost_revisit_ ? "true" : "false")
               << ",\"pending_voxels\":" << suspect_count_
               << ",\"pending_map_voxels\":" << pending_map_voxels
               << ",\"pending_unmapped_voxels\":" << pending_unmapped_voxels
               << ",\"static_boundary_candidates\":" << static_boundary_candidates
               << ",\"static_surface_guard_enabled\":" << (ghost_static_surface_guard_ ? "true" : "false")
               << ",\"static_surface_suppressed\":" << static_surface_suppressed
               << ",\"actionable_voxels\":" << suspects.size()
               << ",\"cleared_by_free_evidence\":" << suspects_cleared_
               << ",\"reobserved_occupied\":" << suspects_reobserved_
               << ",\"evicted_unresolved\":" << suspects_evicted_
               << ",\"history_records\":" << ghost_history_.size()
               << ",\"history_unresolved\":" << unresolved_history
               << ",\"history_evicted\":" << ghost_history_evicted_
               << ",\"history_overflow\":" << ghost_history_overflow_ << "}";
        suspect_status.data = status.str();
        if (headless_) last_snapshot_.status=suspect_status;
        else suspect_status_pub_.publish(suspect_status);
        if (headless_ || (++ghost_history_publish_count_ % 2U) == 1U)
            publishGhostHistory(publish_now);
    }

    bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
    {
        cells_.clear();
        ghost_history_.clear();
        suspect_count_ = suspects_cleared_ = suspects_reobserved_ = suspects_evicted_ = 0;
        ghost_history_evicted_ = ghost_history_overflow_ = ghost_history_publish_count_ = 0;
        dynamic_trails_.clear();
        latest_free_cells_.clear();
        pending_cloud_.reset();
        frame_index_ = 0;
        ROS_WARN("[StaticMapBuilder] map cleared by service request.");
        return true;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber objects_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher map_pub_, legacy_map_pub_, stable_map_pub_, local_map_pub_;
    ros::Publisher free_space_pub_, suspect_pub_, suspect_status_pub_, ghost_history_pub_;
    ros::Publisher processed_scan_pub_, processed_odom_pub_, processed_objects_pub_;
    std::uint32_t odom_input_count_{0}, objects_input_count_{0};
    ros::ServiceServer clear_service_;
    ros::Timer timer_;
    std::string cloud_topic_, objects_topic_, odom_topic_, output_topic_;
    std::string legacy_output_topic_;
    std::string stable_output_topic_, local_output_topic_, world_frame_;
    std::string free_space_output_topic_;
    std::string suspect_output_topic_, suspect_status_topic_, ghost_history_topic_;
    double voxel_size_{0.15}, dynamic_mask_radius_{0.65};
    double dynamic_mask_half_height_{1.0}, backward_trail_seconds_{1.5};
    double trail_step_{0.15}, dynamic_history_seconds_{20.0};
    double object_timeout_{0.40}, publish_rate_{2.0}, local_radius_{6.0};
    double free_space_endpoint_margin_{0.30}, free_space_max_range_{6.0};
    double free_space_ray_step_{0.15}, sensor_origin_z_offset_{0.0};
    int minimum_static_hits_{2}, stable_minimum_static_hits_{8};
    int local_minimum_hits_{1}, local_max_age_frames_{15};
    int free_space_miss_threshold_{3}, max_free_rays_{8000};
    int max_free_voxels_{200000};
    int max_voxels_{500000};
    std::uint64_t frame_index_{0};
    std_msgs::Header last_header_;
    std::unordered_map<Key, Cell, KeyHash> cells_;
    std::unordered_set<Key, KeyHash> latest_free_cells_;
    std::unordered_map<int, std::deque<TrailSample>> dynamic_trails_;
    fastlio_bridge::DynamicObjectArray latest_objects_;
    ros::Time objects_receive_time_;
    bool have_objects_{false};
    Eigen::Vector3d uav_position_{Eigen::Vector3d::Zero()};
    bool have_odom_{false};
    bool publish_legacy_map_{false};
    bool enable_free_space_clearing_{true};
    bool enable_dynamic_map_clearing_{true};
    bool evidence_only_clearing_{false};
    int stable_free_misses_{6};
    double evidence_max_gap_{1.0}, evidence_min_age_{0.5};
    double ray_point_tolerance_{0.06}, clearing_odom_max_dt_{0.08};
    bool enable_ghost_revisit_{false};
    bool ghost_positive_endpoint_confirmation_{true};
    bool ghost_static_surface_guard_{false};
    double ghost_surface_distance_{0.04}, ghost_surface_radius_{0.9};
    double ghost_surface_plane_tolerance_{0.025}, ghost_surface_min_span_{0.7};
    double ghost_surface_stable_age_{0.5};
    int ghost_surface_min_points_{18};
    int ghost_suspect_max_hits_{8}, ghost_confirm_hits_{3};
    int max_ghost_suspects_{10000}, max_ghost_history_{10000};
    double ghost_confirm_baseline_{0.5}, ghost_departure_seconds_{2.0};
    std::size_t suspect_count_{0}, suspects_cleared_{0};
    std::size_t suspects_reobserved_{0}, suspects_evicted_{0};
    std::size_t ghost_history_evicted_{0}, ghost_history_overflow_{0};
    std::size_t ghost_history_publish_count_{0};
    std::unordered_map<Key, GhostHistory, KeyHash> ghost_history_;
    Eigen::Vector3d lidar_offset_{Eigen::Vector3d::Zero()};
    bool explicit_lidar_origin_{false};
    std::deque<nav_msgs::Odometry::ConstPtr> odom_history_;
    sensor_msgs::PointCloud2::ConstPtr pending_cloud_;
    bool headless_{false};
    ros::Time processing_stamp_;
    Snapshot last_snapshot_;
    std::unordered_set<Key,KeyHash> observed_free_cells_;
    bool observed_free_capacity_limited_{false};
};

#ifndef FASTLIO_MAP_BUILDER_LIBRARY
int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_map_builder");
    StaticMapBuilder node;
    ros::spin();
    return 0;
}
#endif
