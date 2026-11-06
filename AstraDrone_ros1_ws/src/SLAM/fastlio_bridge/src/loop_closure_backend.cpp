#include <ros/ros.h>
#include <fastlio_bridge/LoopGraph.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
constexpr double kPi = 3.14159265358979323846;

double clampValue(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

Eigen::Matrix3d skew(const Eigen::Vector3d& vector)
{
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vector.z(), vector.y(),
              vector.z(), 0.0, -vector.x(),
              -vector.y(), vector.x(), 0.0;
    return matrix;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega)
{
    const double theta = omega.norm();
    if (theta < 1e-10)
        return Eigen::Matrix3d::Identity() + skew(omega);
    return Eigen::AngleAxisd(theta, omega / theta).toRotationMatrix();
}

Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation)
{
    const Eigen::AngleAxisd angle_axis(rotation);
    double angle = angle_axis.angle();
    if (!std::isfinite(angle) || angle < 1e-10)
        return Eigen::Vector3d::Zero();
    if (angle > kPi) angle -= 2.0 * kPi;
    return angle_axis.axis() * angle;
}

Eigen::Isometry3d perturbation(const Vector6d& delta)
{
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = delta.head<3>();
    transform.linear() = expSO3(delta.tail<3>());
    return transform;
}

Vector6d poseResidual(const Eigen::Isometry3d& from,
                      const Eigen::Isometry3d& to,
                      const Eigen::Isometry3d& measurement)
{
    const Eigen::Isometry3d error = measurement.inverse() * from.inverse() * to;
    Vector6d residual;
    residual.head<3>() = error.translation();
    residual.tail<3>() = logSO3(error.rotation());
    return residual;
}

double rotationDistance(const Eigen::Matrix3d& first,
                        const Eigen::Matrix3d& second)
{
    return logSO3(first.transpose() * second).norm();
}

Eigen::Isometry3d odometryPose(const nav_msgs::Odometry& message)
{
    Eigen::Quaterniond quaternion(message.pose.pose.orientation.w,
                                  message.pose.pose.orientation.x,
                                  message.pose.pose.orientation.y,
                                  message.pose.pose.orientation.z);
    if (quaternion.norm() < 1e-8 || !std::isfinite(quaternion.norm()))
        quaternion = Eigen::Quaterniond::Identity();
    else
        quaternion.normalize();
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = quaternion.toRotationMatrix();
    pose.translation() = Eigen::Vector3d(message.pose.pose.position.x,
                                         message.pose.pose.position.y,
                                         message.pose.pose.position.z);
    return pose;
}

geometry_msgs::Pose eigenPose(const Eigen::Isometry3d& transform)
{
    geometry_msgs::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    Eigen::Quaterniond quaternion(transform.rotation());
    quaternion.normalize();
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
}

std::string normalizedFrame(std::string frame)
{
    while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
    return frame;
}
}  // namespace

// Compact, dependency-free implementation of the Scan Context descriptor.
// It follows the published polar maximum-height descriptor, not source copied
// from either GPLv2 reference repository.
class ScanContextDescriptor
{
public:
    ScanContextDescriptor(int rings, int sectors, double max_radius,
                          double lidar_height, bool normalize_height,
                          double ground_percentile, double max_height)
        : rings_(std::max(5, rings)),
          sectors_(std::max(12, sectors)),
          max_radius_(std::max(2.0, max_radius)),
          lidar_height_(lidar_height),
          normalize_height_(normalize_height),
          ground_percentile_(clampValue(ground_percentile, 0.01, 0.40)),
          max_height_(std::max(1.0, max_height))
    {
    }

    Eigen::MatrixXf make(const CloudT& cloud) const
    {
        // A fixed sensor-height offset is suitable for a ground vehicle but
        // not for a UAV whose altitude changes.  Estimate a robust local
        // ground/reference height so revisits at slightly different heights
        // produce comparable positive descriptors.
        double height_offset = lidar_height_;
        if (normalize_height_)
        {
            std::vector<float> heights;
            heights.reserve(cloud.size());
            for (const PointT& point : cloud.points)
            {
                if (!pcl::isFinite(point)) continue;
                const double radius = std::hypot(point.x, point.y);
                if (radius > 0.5 && radius <= max_radius_)
                    heights.push_back(point.z);
            }
            if (heights.size() >= 50U)
            {
                const std::size_t percentile_index = std::min(
                    heights.size() - 1U,
                    static_cast<std::size_t>(ground_percentile_ *
                                             heights.size()));
                std::nth_element(heights.begin(),
                                 heights.begin() + percentile_index,
                                 heights.end());
                height_offset = -static_cast<double>(heights[percentile_index]) +
                                0.05;
            }
        }
        Eigen::MatrixXf descriptor = Eigen::MatrixXf::Constant(
            rings_, sectors_, -std::numeric_limits<float>::infinity());
        for (const PointT& point : cloud.points)
        {
            if (!pcl::isFinite(point)) continue;
            const double radius = std::hypot(point.x, point.y);
            if (radius < 1e-3 || radius > max_radius_) continue;
            double angle = std::atan2(point.y, point.x);
            if (angle < 0.0) angle += 2.0 * kPi;
            const int ring = std::min(
                rings_ - 1, static_cast<int>(radius / max_radius_ * rings_));
            const int sector = std::min(
                sectors_ - 1,
                static_cast<int>(angle / (2.0 * kPi) * sectors_));
            descriptor(ring, sector) = std::max(
                descriptor(ring, sector),
                static_cast<float>(clampValue(point.z + height_offset,
                                              0.0, max_height_)));
        }
        for (int row = 0; row < descriptor.rows(); ++row)
            for (int column = 0; column < descriptor.cols(); ++column)
                if (!std::isfinite(descriptor(row, column)))
                    descriptor(row, column) = 0.0F;
        return descriptor;
    }

    Eigen::VectorXf ringKey(const Eigen::MatrixXf& descriptor) const
    {
        return descriptor.rowwise().mean();
    }

    // Returns (cosine distance, circular sector shift). Empty sectors are
    // ignored to avoid giving sparse MID-360 scans an artificially good score.
    std::pair<double, int> distance(const Eigen::MatrixXf& query,
                                    const Eigen::MatrixXf& candidate) const
    {
        double best_distance = std::numeric_limits<double>::infinity();
        int best_shift = 0;
        for (int shift = 0; shift < sectors_; ++shift)
        {
            double similarity_sum = 0.0;
            int valid_columns = 0;
            for (int query_column = 0; query_column < sectors_; ++query_column)
            {
                const int candidate_column = (query_column + shift) % sectors_;
                const Eigen::VectorXf first = query.col(query_column);
                const Eigen::VectorXf second = candidate.col(candidate_column);
                const double first_norm = first.norm();
                const double second_norm = second.norm();
                if (first_norm < 1e-6 || second_norm < 1e-6) continue;
                similarity_sum += first.dot(second) / (first_norm * second_norm);
                ++valid_columns;
            }
            if (valid_columns < std::max(3, sectors_ / 12)) continue;
            const double current = 1.0 - similarity_sum / valid_columns;
            if (current < best_distance)
            {
                best_distance = current;
                best_shift = shift;
            }
        }
        return {best_distance, best_shift};
    }

    double sectorAngle() const
    {
        return 2.0 * kPi / static_cast<double>(sectors_);
    }

private:
    int rings_;
    int sectors_;
    double max_radius_;
    double lidar_height_;
    bool normalize_height_;
    double ground_percentile_;
    double max_height_;
};

class LoopClosureBackend
{
public:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry, sensor_msgs::PointCloud2>;

    struct Keyframe
    {
        ros::Time stamp;
        Eigen::Isometry3d raw_pose{Eigen::Isometry3d::Identity()};
        Eigen::Isometry3d optimized_pose{Eigen::Isometry3d::Identity()};
        CloudT::Ptr local_cloud{new CloudT};
        Eigen::MatrixXf descriptor;
        Eigen::VectorXf ring_key;
    };

    struct Edge
    {
        int from{0};
        int to{0};
        Eigen::Isometry3d measurement{Eigen::Isometry3d::Identity()};
        double translation_weight{1.0};
        double rotation_weight{1.0};
        bool loop{false};
        double fitness{0.0};
    };

    struct LoopCandidate
    {
        int index{-1};
        double spatial_distance{std::numeric_limits<double>::infinity()};
        double descriptor_score{std::numeric_limits<double>::infinity()};
        int sector_shift{0};
        bool strict_descriptor_match{false};
    };

    struct CandidateDiagnostics
    {
        int current{-1};
        int candidate{-1};
        std::size_t spatial_count{0};
        double spatial_distance{std::numeric_limits<double>::infinity()};
        double descriptor_score{std::numeric_limits<double>::infinity()};
        double sector_shift_deg{0.0};
        double fitness{std::numeric_limits<double>::infinity()};
        double overlap{0.0};
        double processing_ms{0.0};
        std::string stage{"waiting"};
    };

    LoopClosureBackend()
        : nh_(),
          pnh_("~"),
          odom_filter_sub_(nh_, "/uav1/fastlio/odom", 30),
          cloud_filter_sub_(nh_, "/uav1/fastlio/registered_scan", 10),
          synchronizer_(SyncPolicy(30), odom_filter_sub_, cloud_filter_sub_),
          scan_context_(readInt("scan_context_rings", 20),
                        readInt("scan_context_sectors", 60),
                        readDouble("scan_context_max_radius", 30.0),
                        readDouble("scan_context_lidar_height", 2.0),
                        readBool("scan_context_normalize_height", true),
                        readDouble("scan_context_ground_percentile", 0.10),
                        readDouble("scan_context_max_height", 15.0))
    {
        pnh_.param<std::string>("odom_topic", odom_topic_,
                                "/uav1/fastlio/odom");
        pnh_.param<std::string>("cloud_topic", cloud_topic_,
                                "/uav1/fastlio/registered_scan");
        pnh_.param<std::string>("world_frame", world_frame_, "loop_map");
        pnh_.param<std::string>("optimized_odom_topic", optimized_odom_topic_,
                                "/uav1/loop_closure/odom");
        pnh_.param<std::string>("corrected_scan_topic", corrected_scan_topic_,
                                "/uav1/loop_closure/registered_scan");
        pnh_.param<std::string>("optimized_map_topic", optimized_map_topic_,
                                "/uav1/loop_closure/map");
        pnh_.param<std::string>("optimized_path_topic", optimized_path_topic_,
                                "/uav1/loop_closure/path");
        pnh_.param<std::string>("raw_keyframe_path_topic", raw_path_topic_,
                                "/uav1/loop_closure/raw_path");
        pnh_.param("keyframe_distance", keyframe_distance_, 1.0);
        pnh_.param("keyframe_angle_deg", keyframe_angle_deg_, 10.0);
        pnh_.param("keyframe_voxel_size", keyframe_voxel_size_, 0.25);
        pnh_.param("max_keyframe_points", max_keyframe_points_, 18000);
        pnh_.param("min_keyframe_points", min_keyframe_points_, 250);
        pnh_.param("max_keyframes", max_keyframes_, 3000);
        pnh_.param("loop_detection_rate", loop_detection_rate_, 0.5);
        pnh_.param("min_loop_time", min_loop_time_, 20.0);
        pnh_.param("min_loop_keyframe_gap", min_loop_keyframe_gap_, 25);
        pnh_.param("loop_cooldown_keyframes", loop_cooldown_keyframes_, 10);
        pnh_.param("loop_search_radius", loop_search_radius_, 2.0);
        // UAV revisits must be close in height as well as XY. Without this
        // gate, a scan collected on the ground before takeoff can look like
        // the same place at cruise altitude in a mostly planar scene.
        pnh_.param("max_loop_vertical_separation",
                    max_loop_vertical_separation_, 1.0);
        pnh_.param("max_spatial_candidates", max_spatial_candidates_, 15);
        pnh_.param("scan_context_threshold", scan_context_threshold_, 0.22);
        pnh_.param("scan_context_geometric_threshold",
                    scan_context_geometric_threshold_, 0.35);
        pnh_.param("geometric_fallback_radius",
                    geometric_fallback_radius_, 2.5);
        pnh_.param("max_icp_candidates", max_icp_candidates_, 5);
        pnh_.param("descriptor_submap_keyframes",
                    descriptor_submap_keyframes_, 4);
        pnh_.param("descriptor_voxel_size", descriptor_voxel_size_, 0.35);
        pnh_.param("descriptor_max_points", descriptor_max_points_, 40000);
        pnh_.param("use_scan_context_yaw_seeds",
                    use_scan_context_yaw_seeds_, true);
        pnh_.param("submap_neighbors", submap_neighbors_, 5);
        pnh_.param("icp_voxel_size", icp_voxel_size_, 0.30);
        pnh_.param("icp_max_correspondence_distance",
                    icp_max_correspondence_distance_, 1.2);
        pnh_.param("icp_max_iterations", icp_max_iterations_, 60);
        pnh_.param("icp_fitness_threshold", icp_fitness_threshold_, 0.30);
        pnh_.param("icp_min_overlap", icp_min_overlap_, 0.35);
        pnh_.param("max_loop_translation_correction",
                    max_loop_translation_correction_, 4.0);
        pnh_.param("max_loop_rotation_correction_deg",
                    max_loop_rotation_correction_deg_, 35.0);
        pnh_.param("optimizer_iterations", optimizer_iterations_, 8);
        pnh_.param("optimizer_huber_delta", optimizer_huber_delta_, 2.5);
        pnh_.param("odom_translation_weight", odom_translation_weight_, 30.0);
        pnh_.param("odom_rotation_weight", odom_rotation_weight_, 40.0);
        pnh_.param("loop_translation_weight", loop_translation_weight_, 12.0);
        pnh_.param("loop_rotation_weight", loop_rotation_weight_, 18.0);
        pnh_.param("map_publish_rate", map_publish_rate_, 0.2);
        pnh_.param("map_voxel_size", map_voxel_size_, 0.20);
        pnh_.param("publish_corrected_scan", publish_corrected_scan_, true);
        pnh_.param("deterministic_replay", deterministic_replay_, false);
        pnh_.param("max_sync_time_difference", max_sync_time_difference_, 0.02);
        pnh_.param("verification_distance", verification_distance_, 0.25);
        pnh_.param("verification_min_overlap", verification_min_overlap_, 0.60);
        pnh_.param("verification_max_rmse", verification_max_rmse_, 0.14);
        pnh_.param("verification_min_gain", verification_min_gain_, 0.02);
        pnh_.param("verification_min_eigen_ratio", verification_min_eigen_ratio_, 0.0001);
        pnh_.param("fine_icp_voxel_size", fine_icp_voxel_size_, 0.12);
        pnh_.param("fine_icp_correspondence_distance", fine_icp_correspondence_distance_, 0.40);
        pnh_.param("min_loop_translation_correction", min_loop_translation_correction_, 0.04);
        pnh_.param("min_loop_rotation_correction_deg", min_loop_rotation_correction_deg_, 0.30);
        pnh_.param("graph_max_loop_residual_translation", graph_max_loop_residual_translation_, 0.25);
        pnh_.param("graph_max_loop_residual_rotation_deg", graph_max_loop_residual_rotation_deg_, 3.0);
        pnh_.param("loop_confirmation_keyframes", loop_confirmation_keyframes_, 2);
        pnh_.param("loop_confirmation_translation", loop_confirmation_translation_, 0.10);
        pnh_.param("loop_confirmation_rotation_deg", loop_confirmation_rotation_deg_, 1.0);
        pnh_.param("gravity_aligned_graph", gravity_aligned_graph_, true);
        pnh_.param("correction_noise_multiplier", correction_noise_multiplier_, 2.0);
        pnh_.param("allow_verified_small_loops", allow_verified_small_loops_, false);
        pnh_.param("small_loop_min_holdout_gain", small_loop_min_holdout_gain_, 0.005);
        pnh_.param("small_loop_weight_scale", small_loop_weight_scale_, 0.5);
        pnh_.param("small_loop_confirmation_keyframes", small_loop_confirmation_keyframes_, 3);
        pnh_.param("small_loop_confirmation_translation", small_loop_confirmation_translation_, 0.03);
        pnh_.param("small_loop_confirmation_rotation_deg", small_loop_confirmation_rotation_deg_, 0.3);
        pnh_.param("validate_graph_geometry", validate_graph_geometry_, true);
        pnh_.param("graph_geometry_max_regression", graph_geometry_max_regression_, 0.01);
        if (!(small_loop_min_holdout_gain_ >= 0.0 && small_loop_min_holdout_gain_ < 1.0 &&
              small_loop_weight_scale_ > 0.0 && small_loop_weight_scale_ <= 1.0 &&
              small_loop_confirmation_keyframes_ >= 2 && small_loop_confirmation_translation_ > 0.0 &&
              small_loop_confirmation_rotation_deg_ > 0.0 && graph_geometry_max_regression_ >= 0.0))
            throw std::invalid_argument("invalid small loop evidence policy");
        seed_evidence_pub_ = pnh_.advertise<std_msgs::String>("seed_evidence", 100);

        keyframe_distance_ = std::max(0.2, keyframe_distance_);
        keyframe_angle_deg_ = std::max(1.0, keyframe_angle_deg_);
        keyframe_voxel_size_ = std::max(0.05, keyframe_voxel_size_);
        max_keyframe_points_ = std::max(500, max_keyframe_points_);
        min_keyframe_points_ = std::max(50, min_keyframe_points_);
        max_keyframes_ = std::max(50, max_keyframes_);
        loop_detection_rate_ = clampValue(loop_detection_rate_, 0.05, 5.0);
        min_loop_time_ = std::max(5.0, min_loop_time_);
        min_loop_keyframe_gap_ = std::max(5, min_loop_keyframe_gap_);
        loop_cooldown_keyframes_ = std::max(1, loop_cooldown_keyframes_);
        loop_search_radius_ = std::max(1.0, loop_search_radius_);
        max_loop_vertical_separation_ = clampValue(
            max_loop_vertical_separation_, 0.2, loop_search_radius_);
        max_spatial_candidates_ = std::max(1, max_spatial_candidates_);
        scan_context_threshold_ = clampValue(scan_context_threshold_, 0.02, 0.8);
        scan_context_geometric_threshold_ = clampValue(
            scan_context_geometric_threshold_, scan_context_threshold_, 0.9);
        geometric_fallback_radius_ = clampValue(
            geometric_fallback_radius_, 0.5, loop_search_radius_);
        max_icp_candidates_ = std::max(1, max_icp_candidates_);
        descriptor_submap_keyframes_ = std::max(1, descriptor_submap_keyframes_);
        descriptor_voxel_size_ = std::max(0.05, descriptor_voxel_size_);
        descriptor_max_points_ = std::max(1000, descriptor_max_points_);
        submap_neighbors_ = std::max(0, submap_neighbors_);
        icp_voxel_size_ = std::max(0.05, icp_voxel_size_);
        icp_max_correspondence_distance_ = std::max(
            icp_voxel_size_ * 2.0, icp_max_correspondence_distance_);
        icp_max_iterations_ = std::max(10, icp_max_iterations_);
        icp_fitness_threshold_ = std::max(1e-4, icp_fitness_threshold_);
        icp_min_overlap_ = clampValue(icp_min_overlap_, 0.05, 0.95);
        optimizer_iterations_ = std::max(1, optimizer_iterations_);
        optimizer_huber_delta_ = std::max(0.1, optimizer_huber_delta_);
        map_publish_rate_ = clampValue(map_publish_rate_, 0.02, 2.0);
        map_voxel_size_ = std::max(0.05, map_voxel_size_);
        if (!(max_sync_time_difference_ > 0.0 && verification_distance_ > 0.0 &&
              verification_min_overlap_ > 0.0 && verification_min_overlap_ <= 1.0 &&
              verification_max_rmse_ > 0.0 && verification_min_gain_ >= 0.0 &&
              verification_min_gain_ < 1.0 && verification_min_eigen_ratio_ > 0.0 &&
              fine_icp_voxel_size_ > 0.0 && fine_icp_correspondence_distance_ > 0.0))
            throw std::invalid_argument("invalid loop verification parameters");

        odom_filter_sub_.unsubscribe();
        cloud_filter_sub_.unsubscribe();
        odom_filter_sub_.subscribe(nh_, odom_topic_, 30);
        cloud_filter_sub_.subscribe(nh_, cloud_topic_, 10);
        synchronizer_.registerCallback(
            boost::bind(&LoopClosureBackend::keyframeCallback, this, _1, _2));
        odom_sub_ = nh_.subscribe(odom_topic_, 50,
                                 &LoopClosureBackend::odomCallback, this);
        cloud_sub_ = nh_.subscribe(cloud_topic_, 5,
                                   &LoopClosureBackend::cloudCallback, this);

        optimized_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(
            optimized_odom_topic_, 20);
        corrected_scan_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            corrected_scan_topic_, 2);
        optimized_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
            optimized_map_topic_, 1, true);
        optimized_path_pub_ = nh_.advertise<nav_msgs::Path>(
            optimized_path_topic_, 1, true);
        raw_path_pub_ = nh_.advertise<nav_msgs::Path>(raw_path_topic_, 1, true);
        graph_pub_ = nh_.advertise<fastlio_bridge::LoopGraph>(
            "/uav1/loop_closure/graph",1,true);
        loop_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/uav1/loop_closure/constraints", 1, true);
        correction_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
            "/uav1/loop_closure/map_to_raw", 5, true);
        diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
            "/uav1/loop_closure/diagnostics", 5, true);
        input_ack_pub_ = pnh_.advertise<std_msgs::Header>("input_ack", 5);
        freeze_service_=pnh_.advertiseService("freeze_keyframes",
            &LoopClosureBackend::freezeKeyframes,this);
        process_service_ = pnh_.advertiseService("process_pending",
            &LoopClosureBackend::processPending, this);
        if (!deterministic_replay_) {
        loop_timer_ = nh_.createTimer(ros::Duration(1.0 / loop_detection_rate_),
                                      &LoopClosureBackend::loopTimer, this);
        map_timer_ = nh_.createTimer(ros::Duration(1.0 / map_publish_rate_),
                                     &LoopClosureBackend::mapTimer, this);
        }

        ROS_INFO("[LoopClosure] input odom=%s cloud=%s output map=%s frame=%s",
                 odom_topic_.c_str(), cloud_topic_.c_str(),
                 optimized_map_topic_.c_str(), world_frame_.c_str());
        ROS_WARN("[LoopClosure] optimized outputs are separate from flight "
                 "control odometry; validate loops before routing to control.");
    }

private:
    int readInt(const std::string& name, int fallback)
    {
        int value = fallback;
        pnh_.param(name, value, fallback);
        return value;
    }

    double readDouble(const std::string& name, double fallback)
    {
        double value = fallback;
        pnh_.param(name, value, fallback);
        return value;
    }

    bool readBool(const std::string& name, bool fallback)
    {
        bool value = fallback;
        pnh_.param(name, value, fallback);
        return value;
    }

    CloudT::Ptr downsample(const CloudT::ConstPtr& input, double leaf) const
    {
        CloudT::Ptr output(new CloudT);
        pcl::VoxelGrid<PointT> filter;
        filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                           static_cast<float>(leaf));
        filter.setInputCloud(input);
        filter.filter(*output);
        return output;
    }

    void limitCloud(CloudT& cloud, int maximum_points) const
    {
        if (cloud.size() <= static_cast<std::size_t>(maximum_points)) return;
        CloudT reduced;
        reduced.reserve(maximum_points);
        const double stride = static_cast<double>(cloud.size()) /
                              static_cast<double>(maximum_points);
        for (int index = 0; index < maximum_points; ++index)
            reduced.push_back(cloud.points[static_cast<std::size_t>(index * stride)]);
        reduced.width = static_cast<std::uint32_t>(reduced.size());
        reduced.height = 1;
        reduced.is_dense = false;
        cloud.swap(reduced);
    }

    CloudT::Ptr buildDescriptorSubmap(int center) const
    {
        CloudT::Ptr submap(new CloudT);
        if (center < 0 || center >= static_cast<int>(keyframes_.size()))
            return submap;
        const int first = std::max(0, center - descriptor_submap_keyframes_ + 1);
        const Eigen::Isometry3d center_inverse =
            keyframes_[center].raw_pose.inverse();
        for (int index = first; index <= center; ++index)
        {
            CloudT transformed;
            const Eigen::Isometry3d relative =
                center_inverse * keyframes_[index].raw_pose;
            pcl::transformPointCloud(*keyframes_[index].local_cloud, transformed,
                                     relative.matrix().cast<float>());
            *submap += transformed;
        }
        submap = downsample(submap, descriptor_voxel_size_);
        limitCloud(*submap, descriptor_max_points_);
        return submap;
    }

    bool shouldAddKeyframe(const Eigen::Isometry3d& pose) const
    {
        if (keyframes_.empty()) return true;
        const Keyframe& previous = keyframes_.back();
        const double translation =
            (pose.translation() - previous.raw_pose.translation()).norm();
        const double rotation = rotationDistance(previous.raw_pose.rotation(),
                                                 pose.rotation());
        return translation >= keyframe_distance_ ||
               rotation >= keyframe_angle_deg_ * kPi / 180.0;
    }

    void keyframeCallback(const nav_msgs::Odometry::ConstPtr& odometry,
                          const sensor_msgs::PointCloud2::ConstPtr& cloud_message)
    {
        if (frozen_keyframes_) return;
        addKeyframe(odometry, cloud_message);
        std_msgs::Header ack = cloud_message->header;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ack.seq = keyframes_.size();
        }
        input_ack_pub_.publish(ack);
    }

    bool freezeKeyframes(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response)
    {
        frozen_keyframes_=true;
        { std::lock_guard<std::mutex> lock(mutex_); }
        publishPaths();
        response.success=true;
        response.message="Keyframe acquisition frozen before descent";return true;
    }

    bool processPending(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
    {
        if (!deterministic_replay_) {
            response.success = false;
            response.message = "requires deterministic_replay=true";
            return true;
        }
        loopTimer(ros::TimerEvent());
        publishPaths();
        publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK, "replay step complete");
        response.success = true;
        return true;
    }

    void addKeyframe(const nav_msgs::Odometry::ConstPtr& odometry,
                     const sensor_msgs::PointCloud2::ConstPtr& cloud_message)
    {
        if (std::abs((odometry->header.stamp-cloud_message->header.stamp).toSec()) >
            max_sync_time_difference_) {
            ++rejected_sync_count_;
            return;
        }
        if (normalizedFrame(odometry->header.frame_id) !=
            normalizedFrame(cloud_message->header.frame_id))
        {
            ROS_ERROR_THROTTLE(2.0,
                "[LoopClosure] rejected data: odom frame '%s' != cloud '%s'.",
                odometry->header.frame_id.c_str(),
                cloud_message->header.frame_id.c_str());
            return;
        }
        const Eigen::Isometry3d raw_pose = odometryPose(*odometry);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!shouldAddKeyframe(raw_pose) ||
                keyframes_.size() >= static_cast<std::size_t>(max_keyframes_))
                return;
        }

        CloudT::Ptr world_cloud(new CloudT);
        pcl::fromROSMsg(*cloud_message, *world_cloud);
        if (world_cloud->empty()) return;
        CloudT::Ptr local_cloud(new CloudT);
        pcl::transformPointCloud(*world_cloud, *local_cloud,
                                 raw_pose.inverse().matrix().cast<float>());
        local_cloud = downsample(local_cloud, keyframe_voxel_size_);
        limitCloud(*local_cloud, max_keyframe_points_);
        if (local_cloud->size() < static_cast<std::size_t>(min_keyframe_points_))
        {
            ROS_WARN_THROTTLE(2.0,
                "[LoopClosure] keyframe rejected: %zu filtered points.",
                local_cloud->size());
            return;
        }

        Keyframe keyframe;
        keyframe.stamp = cloud_message->header.stamp;
        keyframe.raw_pose = raw_pose;
        keyframe.local_cloud = local_cloud;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!shouldAddKeyframe(raw_pose) ||
                keyframes_.size() >= static_cast<std::size_t>(max_keyframes_))
                return;
            keyframe.optimized_pose = map_to_raw_ * raw_pose;
            const int new_index = static_cast<int>(keyframes_.size());
            if (new_index > 0)
            {
                Edge edge;
                edge.from = new_index - 1;
                edge.to = new_index;
                edge.measurement =
                    keyframes_.back().raw_pose.inverse() * raw_pose;
                edge.translation_weight = odom_translation_weight_;
                edge.rotation_weight = odom_rotation_weight_;
                edges_.push_back(edge);
            }
            keyframes_.push_back(std::move(keyframe));
            raw_frame_ = normalizedFrame(odometry->header.frame_id);
            // Aggregate several consecutive MID-360 sweeps in the newest
            // keyframe coordinates.  A solid-state non-repetitive scanner is
            // much less repeatable when Scan Context is built from one frame.
            CloudT::Ptr descriptor_cloud = buildDescriptorSubmap(new_index);
            keyframes_.back().descriptor = scan_context_.make(*descriptor_cloud);
            keyframes_.back().ring_key =
                scan_context_.ringKey(keyframes_.back().descriptor);
            map_dirty_ = true;
        }
        publishPaths();
        publishLoopMarkers();
        publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK,
                           "keyframe added");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& message)
    {
        Eigen::Isometry3d correction;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            correction = map_to_raw_;
        }
        nav_msgs::Odometry output = *message;
        output.header.frame_id = world_frame_;
        output.pose.pose = eigenPose(correction * odometryPose(*message));
        optimized_odom_pub_.publish(output);

        geometry_msgs::TransformStamped transform;
        transform.header.stamp = message->header.stamp;
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = normalizedFrame(message->header.frame_id);
        transform.transform.translation.x = correction.translation().x();
        transform.transform.translation.y = correction.translation().y();
        transform.transform.translation.z = correction.translation().z();
        Eigen::Quaterniond quaternion(correction.rotation());
        quaternion.normalize();
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        correction_pub_.publish(transform);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
    {
        if (!publish_corrected_scan_ || corrected_scan_pub_.getNumSubscribers() == 0)
            return;
        Eigen::Isometry3d correction;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            correction = map_to_raw_;
        }
        CloudT input;
        CloudT output;
        pcl::fromROSMsg(*message, input);
        pcl::transformPointCloud(input, output, correction.matrix().cast<float>());
        sensor_msgs::PointCloud2 result;
        pcl::toROSMsg(output, result);
        result.header.stamp = message->header.stamp;
        result.header.frame_id = world_frame_;
        corrected_scan_pub_.publish(result);
    }

    std::vector<int> spatialCandidates(int current) const
    {
        std::vector<std::pair<double, int>> candidates;
        const Keyframe& latest = keyframes_[current];
        const int newest_allowed = current - min_loop_keyframe_gap_;
        for (int index = 0; index <= newest_allowed; ++index)
        {
            if ((latest.stamp - keyframes_[index].stamp).toSec() < min_loop_time_)
                continue;
            const double vertical_separation = std::abs(
                latest.raw_pose.translation().z() -
                keyframes_[index].raw_pose.translation().z());
            if (vertical_separation > max_loop_vertical_separation_)
                continue;
            const double distance = (latest.raw_pose.translation() -
                                     keyframes_[index].raw_pose.translation()).norm();
            if (distance <= loop_search_radius_)
                candidates.push_back({distance, index});
        }
        std::sort(candidates.begin(), candidates.end());
        if (candidates.size() > static_cast<std::size_t>(max_spatial_candidates_))
            candidates.resize(max_spatial_candidates_);
        std::vector<int> indices;
        indices.reserve(candidates.size());
        for (const auto& candidate : candidates) indices.push_back(candidate.second);
        return indices;
    }

    std::vector<LoopCandidate> descriptorCandidates(
        int current, const std::vector<int>& spatial) const
    {
        std::vector<LoopCandidate> candidates;
        candidates.reserve(spatial.size());
        // The spatial list is deliberately bounded (15 by default), so full
        // comparison is cheap and avoids discarding the true revisit merely
        // because its sparse MID-360 ring key was not in the old top five.
        for (const int index : spatial)
        {
            const auto result = scan_context_.distance(
                keyframes_[current].descriptor, keyframes_[index].descriptor);
            LoopCandidate candidate;
            candidate.index = index;
            candidate.spatial_distance =
                (keyframes_[current].raw_pose.translation() -
                 keyframes_[index].raw_pose.translation()).norm();
            candidate.descriptor_score = result.first;
            candidate.sector_shift = result.second;
            candidate.strict_descriptor_match =
                result.first <= scan_context_threshold_;
            candidates.push_back(candidate);
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const LoopCandidate& first, const LoopCandidate& second)
                  {
                      if (first.strict_descriptor_match !=
                          second.strict_descriptor_match)
                          return first.strict_descriptor_match;
                      const double first_score = first.descriptor_score +
                          0.05 * first.spatial_distance;
                      const double second_score = second.descriptor_score +
                          0.05 * second.spatial_distance;
                      return first_score < second_score;
                  });
        return candidates;
    }

    CloudT::Ptr buildCandidateSubmap(int candidate, int current) const
    {
        CloudT::Ptr submap(new CloudT);
        const int first = std::max(0, candidate - submap_neighbors_);
        const int last = std::min(static_cast<int>(keyframes_.size()) - 1,
                                  candidate + submap_neighbors_);
        const Eigen::Isometry3d candidate_inverse =
            keyframes_[candidate].raw_pose.inverse();
        for (int index = first; index <= last; ++index)
        {
            // Neither the fitting scan nor the independent preceding scan can
            // leak into the historical reference, even with custom small gaps.
            if (index >= current-1 ||
                (keyframes_[current].stamp-keyframes_[index].stamp).toSec() < min_loop_time_)
                continue;
            CloudT transformed;
            const Eigen::Isometry3d relative =
                candidate_inverse * keyframes_[index].raw_pose;
            pcl::transformPointCloud(*keyframes_[index].local_cloud, transformed,
                                     relative.matrix().cast<float>());
            *submap += transformed;
        }
        return downsample(submap, fine_icp_voxel_size_);
    }

    struct Verification {
        double overlap{0.0};
        double rmse{std::numeric_limits<double>::infinity()};
        double clipped_mse{0.0};
        double eigen_ratio{0.0};
        double lever_arm_radius{1.0};
    };

    Verification verifyGeometry(const CloudT::ConstPtr& source,
                                const CloudT::ConstPtr& target,
                                const Eigen::Matrix4f& transform,
                                bool observability) const
    {
        Verification result;
        if (source->empty() || target->size() < 12) return result;
        CloudT aligned;
        pcl::transformPointCloud(*source, aligned, transform);
        pcl::KdTreeFLANN<PointT> tree;
        tree.setInputCloud(target);
        const double cap = verification_distance_ * verification_distance_;
        std::size_t matched = 0, planes = 0;
        double sum = 0.0;
        Matrix6d information = Matrix6d::Zero();
        // Normalize rotation lever arms to make the spectrum independent of
        // coordinate origin and approximately independent of room scale.
        double radius_squared = 0.0;
        for (const PointT& p : source->points)
            radius_squared += p.getVector3fMap().squaredNorm();
        const double radius = std::max(1.0, std::sqrt(radius_squared/source->size()));
        result.lever_arm_radius = radius;
        for (const PointT& point : aligned.points) {
            std::vector<int> neighbors(observability ? 12 : 1);
            std::vector<float> distances(neighbors.size());
            const int found = tree.nearestKSearch(point, neighbors.size(), neighbors, distances);
            if (found < 1) continue;
            result.clipped_mse += std::min(cap, static_cast<double>(distances[0]));
            if (distances[0] > cap) continue;
            ++matched;
            sum += distances[0];
            if (!observability || found < 12 || distances[11] > 1.0F) continue;
            Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
            for (int index : neighbors) centroid += target->points[index].getVector3fMap().cast<double>();
            centroid /= neighbors.size();
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for (int index : neighbors) {
                const Eigen::Vector3d d = target->points[index].getVector3fMap().cast<double>()-centroid;
                covariance += d*d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> plane(covariance);
            if (plane.info() != Eigen::Success || plane.eigenvalues()(1) < 1e-6 ||
                plane.eigenvalues()(0) > 0.10*plane.eigenvalues()(1)) continue;
            const Eigen::Vector3d normal = plane.eigenvectors().col(0);
            const Eigen::Vector3d lever = point.getVector3fMap().cast<double>()-
                                          transform.block<3,1>(0,3).cast<double>();
            Vector6d row;
            row.head<3>() = normal;
            row.tail<3>() = lever.cross(normal)/radius;
            information += row*row.transpose();
            ++planes;
        }
        result.overlap = static_cast<double>(matched)/source->size();
        result.clipped_mse /= source->size();
        if (matched) result.rmse = std::sqrt(sum/matched);
        if (planes >= 50) {
            Eigen::SelfAdjointEigenSolver<Matrix6d> spectrum(information);
            if (spectrum.info() == Eigen::Success && spectrum.eigenvalues()(5) > 1e-9)
                result.eigen_ratio = std::max(0.0,spectrum.eigenvalues()(0))/spectrum.eigenvalues()(5);
        }
        return result;
    }

    bool sufficientGeometry(const Verification& quality, bool observability) const
    {
        return quality.overlap >= verification_min_overlap_ &&
               quality.rmse <= verification_max_rmse_ &&
               (!observability || quality.eigen_ratio >= verification_min_eigen_ratio_);
    }

    bool refinePointToPlane(const CloudT::ConstPtr& source,
                            const CloudT::ConstPtr& target,
                            Eigen::Matrix4f* estimate, bool translation_only=false) const
    {
        pcl::KdTreeFLANN<PointT> tree;
        tree.setInputCloud(target);
        std::vector<Eigen::Vector3d> normals(target->size(),Eigen::Vector3d::Zero());
        for (std::size_t j=0;j<target->size();++j) {
            std::vector<int> ids(12); std::vector<float> distances(12);
            if (tree.nearestKSearch(target->points[j],12,ids,distances)<12 || distances[11]>1.0F)
                continue;
            Eigen::Vector3d mean=Eigen::Vector3d::Zero();
            for (int i:ids) mean+=target->points[i].getVector3fMap().cast<double>();
            mean/=12.0;
            Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
            for (int i:ids) {
                const Eigen::Vector3d d=target->points[i].getVector3fMap().cast<double>()-mean;
                cov+=d*d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
            if (solver.info()==Eigen::Success && solver.eigenvalues()(1)>1e-6 &&
                solver.eigenvalues()(0)<0.10*solver.eigenvalues()(1))
                normals[j]=solver.eigenvectors().col(0);
        }
        Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
        pose.matrix()=estimate->cast<double>();
        for (int iteration=0;iteration<20;++iteration) {
            Matrix6d h=Matrix6d::Zero(); Vector6d b=Vector6d::Zero();
            int count=0;
            for (const PointT& point:source->points) {
                const Eigen::Vector3d local=point.getVector3fMap().cast<double>();
                const Eigen::Vector3d p=pose*local;
                PointT query;query.x=p.x();query.y=p.y();query.z=p.z();
                std::vector<int> ids(1);std::vector<float> distances(1);
                if (tree.nearestKSearch(query,1,ids,distances)<1 ||
                    distances[0]>fine_icp_correspondence_distance_*fine_icp_correspondence_distance_)
                    continue;
                const Eigen::Vector3d& n=normals[ids[0]];
                if (n.squaredNorm()<0.5) continue;
                const Eigen::Vector3d q=target->points[ids[0]].getVector3fMap().cast<double>();
                const double residual=n.dot(p-q);
                const double weight=std::min(1.0,0.05/std::max(1e-9,std::abs(residual)));
                Vector6d row;
                row.head<3>()=pose.rotation().transpose()*n;
                row.tail<3>()=translation_only ? Eigen::Vector3d::Zero().eval() : local.cross(row.head<3>()).eval();
                h+=weight*row*row.transpose();b+=weight*row*residual;++count;
            }
            if (count<50) return false;
            Vector6d delta=Vector6d::Zero();
            if (translation_only) {
                const Eigen::Matrix3d ht=h.topLeftCorner<3,3>();
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> spectrum(ht);
                if (spectrum.info()!=Eigen::Success || spectrum.eigenvalues()(0)<1e-6) return false;
                delta.head<3>()=-ht.ldlt().solve(b.head<3>());
            } else {
                Eigen::SelfAdjointEigenSolver<Matrix6d> spectrum(h);
                if (spectrum.info()!=Eigen::Success || spectrum.eigenvalues()(0)<1e-6) return false;
                delta=-h.ldlt().solve(b);
            }
            if (!delta.allFinite() || delta.head<3>().norm()>0.5 || delta.tail<3>().norm()>0.15)
                return false;
            pose=pose*perturbation(delta);
            if (delta.norm()<1e-6) break;
        }
        *estimate=pose.matrix().cast<float>();
        return true;
    }

    void publishSeedEvidence(int current, int candidate, int seed, const std::string& decision,
                             const Verification& before, const Verification& checked,
                             const Verification& heldout_before, const Verification& heldout,
                             double translation, double angle, double translation_limit,
                             double angle_limit)
    {
        auto number=[](double x) { std::ostringstream o; o<<std::setprecision(12);
            if (std::isfinite(x)) o<<x; else o<<"null"; return o.str(); };
        std::ostringstream o;
        o<<"{\"current\":"<<current<<",\"candidate\":"<<candidate
         <<",\"seed\":"<<seed<<",\"decision\":\""<<decision<<"\""
         <<",\"stamp\":"<<number(keyframes_[current].stamp.toSec())
         <<",\"before_mse\":"<<number(before.clipped_mse)
         <<",\"after_mse\":"<<number(checked.clipped_mse)
         <<",\"gain\":"<<number(1.0-checked.clipped_mse/std::max(1e-12,before.clipped_mse))
         <<",\"holdout_before_mse\":"<<number(heldout_before.clipped_mse)
         <<",\"holdout_after_mse\":"<<number(heldout.clipped_mse)
         <<",\"holdout_gain\":"<<number(1.0-heldout.clipped_mse/std::max(1e-12,heldout_before.clipped_mse))
         <<",\"rmse_m\":"<<number(checked.rmse)<<",\"overlap\":"<<number(checked.overlap)
         <<",\"eigen_ratio\":"<<number(checked.eigen_ratio)
         <<",\"translation_m\":"<<number(translation)<<",\"rotation_rad\":"<<number(angle)
         <<",\"translation_deadband_m\":"<<number(translation_limit)
         <<",\"rotation_deadband_rad\":"<<number(angle_limit)<<"}";
        std_msgs::String msg; msg.data=o.str(); seed_evidence_pub_.publish(msg);
    }

    bool registerLoop(int current, int candidate, int sector_shift,
                      Eigen::Isometry3d* measurement, double* fitness,
                      double* overlap)
    {
        CloudT::Ptr source = downsample(keyframes_[current].local_cloud,
                                        fine_icp_voxel_size_);
        CloudT::Ptr target = buildCandidateSubmap(candidate, current);
        if (source->size() < static_cast<std::size_t>(min_keyframe_points_) ||
            target->size() < static_cast<std::size_t>(min_keyframe_points_))
            return false;
        const Eigen::Isometry3d initial =
            keyframes_[candidate].raw_pose.inverse() * keyframes_[current].raw_pose;
        const Verification before = verifyGeometry(source,target,initial.matrix().cast<float>(),false);
        const Eigen::Isometry3d holdout_relative = keyframes_[current].raw_pose.inverse() *
                                                 keyframes_[current-1].raw_pose;
        CloudT::Ptr holdout(new CloudT);
        pcl::transformPointCloud(*keyframes_[current-1].local_cloud, *holdout,
                                 holdout_relative.matrix().cast<float>());
        const Verification holdout_before = verifyGeometry(holdout,target,initial.matrix().cast<float>(),false);
        std::vector<Eigen::Isometry3d> guesses{initial};
        if (use_scan_context_yaw_seeds_ && sector_shift != 0)
        {
            double shift_angle = sector_shift * scan_context_.sectorAngle();
            if (shift_angle > kPi) shift_angle -= 2.0 * kPi;
            // The sign depends on the query/candidate convention.  Try both;
            // hard geometric gates still decide whether either is valid.
            for (const double sign : {-1.0, 1.0})
            {
                Eigen::Isometry3d seed = initial;
                seed.linear() = Eigen::AngleAxisd(
                    sign * shift_angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                    initial.rotation();
                guesses.push_back(seed);
            }
        }

        bool accepted = false;
        double best_quality = std::numeric_limits<double>::infinity();
        *fitness = std::numeric_limits<double>::infinity();
        *overlap = 0.0;
        int seed_index=-1;
        for (const Eigen::Isometry3d& guess : guesses)
        {
            ++seed_index;
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setInputSource(downsample(source,icp_voxel_size_));
            icp.setInputTarget(downsample(target,icp_voxel_size_));
            icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
            icp.setMaximumIterations(icp_max_iterations_);
            icp.setTransformationEpsilon(1e-7);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACOutlierRejectionThreshold(
                icp_max_correspondence_distance_ * 0.75);
            CloudT aligned;
            icp.align(aligned, guess.matrix().cast<float>());
            if (!icp.hasConverged()) continue;

            pcl::IterativeClosestPoint<PointT, PointT> fine;
            fine.setInputSource(source);
            fine.setInputTarget(target);
            fine.setMaxCorrespondenceDistance(fine_icp_correspondence_distance_);
            fine.setMaximumIterations(icp_max_iterations_);
            fine.setTransformationEpsilon(1e-8);
            fine.setEuclideanFitnessEpsilon(1e-8);
            fine.align(aligned,icp.getFinalTransformation());
            if (!fine.hasConverged()) continue;
            Eigen::Matrix4f final_matrix = fine.getFinalTransformation();
            if (!refinePointToPlane(source,target,&final_matrix)) {
                ++rejected_degenerate_count_;
                continue;
            }
            // A residual-scale yaw fit must not move distant points by an
            // apparent translation. Small-loop refinement keeps LIO attitude.
            Eigen::Isometry3d trial=Eigen::Isometry3d::Identity();trial.matrix()=final_matrix.cast<double>();
            const auto initial_delta=initial.inverse()*trial;
            const auto preliminary=verifyGeometry(source,target,final_matrix,false);
            const bool translation_only=allow_verified_small_loops_ &&
                initial_delta.translation().norm()<std::max(min_loop_translation_correction_,correction_noise_multiplier_*preliminary.rmse) &&
                logSO3(initial_delta.rotation()).norm()<std::max(min_loop_rotation_correction_deg_*kPi/180.,correction_noise_multiplier_*preliminary.rmse/preliminary.lever_arm_radius);
            if (translation_only) {
                trial.linear()=initial.rotation();final_matrix=trial.matrix().cast<float>();
                if (!refinePointToPlane(source,target,&final_matrix,true)) { ++rejected_degenerate_count_;continue; }
            }
            const Verification checked = verifyGeometry(source,target,final_matrix,true);
            const double candidate_fitness = checked.rmse*checked.rmse;
            const double candidate_overlap = checked.overlap;
            if (candidate_fitness < *fitness)
            {
                *fitness = candidate_fitness;
                *overlap = candidate_overlap;
            }
            const Verification heldout = verifyGeometry(holdout,target,final_matrix,false);
            Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
            final_pose.matrix() = final_matrix.cast<double>();
            Eigen::Quaterniond normalized(final_pose.rotation());
            normalized.normalize(); final_pose.linear() = normalized.toRotationMatrix();
            const Eigen::Isometry3d correction = initial.inverse()*final_pose;
            const double translation=correction.translation().norm();
            const double angle=logSO3(correction.rotation()).norm();
            const double translation_deadband=std::max(min_loop_translation_correction_,
                                                       correction_noise_multiplier_*checked.rmse);
            const double rotation_deadband=std::max(min_loop_rotation_correction_deg_*kPi/180.0,
                correction_noise_multiplier_*checked.rmse/checked.lever_arm_radius);
            const bool small=translation<translation_deadband && angle<rotation_deadband;
            bool independent_targets=true;
            if (small && allow_verified_small_loops_) {
                // Independent sweeps also have to improve; fitting the same
                // blurred historical submap twice is insufficient evidence.
                for (int index : {candidate, candidate>0 ? candidate-1 : candidate+1}) {
                    CloudT::Ptr individual(new CloudT);
                    const auto relative=keyframes_[candidate].raw_pose.inverse()*keyframes_[index].raw_pose;
                    pcl::transformPointCloud(*keyframes_[index].local_cloud,*individual,relative.matrix().cast<float>());
                    const auto original=verifyGeometry(source,individual,initial.matrix().cast<float>(),false);
                    const auto proposed=verifyGeometry(source,individual,final_matrix,false);
                    Eigen::Matrix4f separately_fitted=initial.matrix().cast<float>();
                    const bool fit_ok=refinePointToPlane(source,individual,&separately_fitted,true);
                    const double disagreement=(separately_fitted.block<3,1>(0,3)-final_matrix.block<3,1>(0,3)).norm();
                    if (!sufficientGeometry(proposed,false) || proposed.clipped_mse>original.clipped_mse*0.995 ||
                        !fit_ok || disagreement>0.01)
                        independent_targets=false;
                }
            }
            std::string decision;
            if (checked.eigen_ratio < verification_min_eigen_ratio_)
                { ++rejected_degenerate_count_; decision="degenerate"; }
            else if (!sufficientGeometry(checked,true) || !std::isfinite(candidate_fitness) ||
                     candidate_fitness>icp_fitness_threshold_ || candidate_overlap<icp_min_overlap_)
                { ++rejected_geometry_count_; decision="geometry"; }
            else if (!sufficientGeometry(heldout,false) ||
                     heldout.clipped_mse>holdout_before.clipped_mse*1.01)
                { ++rejected_holdout_count_; decision="holdout"; }
            else if (checked.clipped_mse>(1.0-verification_min_gain_)*before.clipped_mse)
                { ++rejected_gain_count_; decision="gain"; }
            else if (translation>max_loop_translation_correction_ ||
                     angle>max_loop_rotation_correction_deg_*kPi/180.0)
                { ++rejected_large_correction_count_; decision="large"; }
            else if (small && !allow_verified_small_loops_)
                { ++rejected_tiny_correction_count_; decision="tiny"; }
            else if (small && heldout.clipped_mse>
                     (1.0-small_loop_min_holdout_gain_)*holdout_before.clipped_mse)
                { ++rejected_small_evidence_count_; decision="small_without_holdout_gain"; }
            else if (small && !independent_targets)
                { ++rejected_small_evidence_count_; decision="small_without_independent_targets"; }
            else decision=small ? "verified_small_proposal" : "verified_proposal";
            publishSeedEvidence(current,candidate,seed_index,decision,before,checked,
                holdout_before,heldout,translation,angle,translation_deadband,rotation_deadband);
            if (decision!="verified_small_proposal" && decision!="verified_proposal") continue;
            last_registration_small_=small;
            const double quality = candidate_fitness /
                std::max(0.05, candidate_overlap);
            if (quality < best_quality)
            {
                best_quality = quality;
                *fitness = candidate_fitness;
                *overlap = candidate_overlap;
                *measurement = final_pose;
                accepted = true;
                last_verification_ = checked;
                last_holdout_ = heldout;
            }
            // Odometry is the preferred seed. Only try descriptor yaw seeds
            // when that seed fails independent verification; this also avoids
            // choosing a repeated-shelf rotation just for a marginally lower MSE.
            if (accepted) break;
        }
        return accepted;
    }

    Eigen::Isometry3d graphPerturbation(const Eigen::Isometry3d& pose,
                                      const Vector6d& delta, bool freeze_yaw=false) const
    {
        if (freeze_yaw) {
            Eigen::Isometry3d output=pose; output.translation()+=delta.head<3>();return output;
        }
        if (!gravity_aligned_graph_) return pose*perturbation(delta);
        // LIO supplies gravity. An odometry-only graph has no IMU factors to
        // justify changing roll/pitch independently at each node. Optimize
        // global XYZ and yaw while preserving each measured gravity direction.
        Eigen::Isometry3d output=pose;
        output.translation()+=delta.head<3>();
        output.linear()=Eigen::AngleAxisd(freeze_yaw ? 0. : delta(5),Eigen::Vector3d::UnitZ()).toRotationMatrix()*pose.rotation();
        return output;
    }

    Matrix6d numericalJacobian(const Edge& edge,
                               const Eigen::Isometry3d& from,
                               const Eigen::Isometry3d& to,
                               bool perturb_from, bool freeze_yaw=false) const
    {
        Matrix6d jacobian;
        const double epsilon = 1e-5;
        for (int column = 0; column < 6; ++column)
        {
            Vector6d plus = Vector6d::Zero();
            Vector6d minus = Vector6d::Zero();
            plus(column) = epsilon;
            minus(column) = -epsilon;
            Vector6d residual_plus;
            Vector6d residual_minus;
            if (perturb_from)
            {
                residual_plus = poseResidual(graphPerturbation(from,plus,freeze_yaw), to,
                                             edge.measurement);
                residual_minus = poseResidual(graphPerturbation(from,minus,freeze_yaw), to,
                                              edge.measurement);
            }
            else
            {
                residual_plus = poseResidual(from, graphPerturbation(to,plus,freeze_yaw),
                                             edge.measurement);
                residual_minus = poseResidual(from, graphPerturbation(to,minus,freeze_yaw),
                                              edge.measurement);
            }
            jacobian.col(column) =
                (residual_plus - residual_minus) / (2.0 * epsilon);
        }
        return jacobian;
    }

    bool optimizePoseGraph(bool freeze_yaw=false)
    {
        const int pose_count = static_cast<int>(keyframes_.size());
        if (pose_count < 2) return false;
        std::vector<Eigen::Isometry3d> poses;
        poses.reserve(keyframes_.size());
        for (const Keyframe& keyframe : keyframes_)
            poses.push_back(keyframe.optimized_pose);
        const int dimensions = 6 * (pose_count - 1);
        auto cost = [this](const std::vector<Eigen::Isometry3d>& estimates) {
            double total = 0.0;
            for (const Edge& edge : edges_) {
                Vector6d r = poseResidual(estimates[edge.from], estimates[edge.to], edge.measurement);
                r.head<3>() *= edge.translation_weight;
                r.tail<3>() *= edge.rotation_weight;
                const double norm = r.norm();
                total += norm <= optimizer_huber_delta_ ? 0.5*norm*norm :
                    optimizer_huber_delta_*(norm-0.5*optimizer_huber_delta_);
            }
            return total;
        };
        const double initial_cost = cost(poses);

        for (int iteration = 0; iteration < optimizer_iterations_; ++iteration)
        {
            std::vector<Eigen::Triplet<double>> triplets;
            Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimensions);
            triplets.reserve(edges_.size() * 144U +
                             static_cast<std::size_t>(dimensions));
            for (const Edge& edge : edges_)
            {
                Vector6d residual = poseResidual(
                    poses[edge.from], poses[edge.to], edge.measurement);
                Matrix6d scaling = Matrix6d::Zero();
                scaling.diagonal().head<3>().setConstant(edge.translation_weight);
                scaling.diagonal().tail<3>().setConstant(edge.rotation_weight);
                residual = scaling * residual;
                Matrix6d from_jacobian = scaling * numericalJacobian(
                    edge, poses[edge.from], poses[edge.to], true,freeze_yaw);
                Matrix6d to_jacobian = scaling * numericalJacobian(
                    edge, poses[edge.from], poses[edge.to], false,freeze_yaw);
                const double norm = residual.norm();
                const double robust_scale = norm <= optimizer_huber_delta_
                    ? 1.0 : std::sqrt(optimizer_huber_delta_ / norm);
                residual *= robust_scale;
                from_jacobian *= robust_scale;
                to_jacobian *= robust_scale;

                auto addBlock = [&triplets](int row, int column,
                                             const Matrix6d& block)
                {
                    for (int block_row = 0; block_row < 6; ++block_row)
                        for (int block_column = 0; block_column < 6; ++block_column)
                            triplets.emplace_back(row + block_row,
                                                  column + block_column,
                                                  block(block_row, block_column));
                };
                const int from_offset = 6 * (edge.from - 1);
                const int to_offset = 6 * (edge.to - 1);
                if (edge.from > 0)
                {
                    addBlock(from_offset, from_offset,
                             from_jacobian.transpose() * from_jacobian);
                    gradient.segment<6>(from_offset) +=
                        from_jacobian.transpose() * residual;
                }
                if (edge.to > 0)
                {
                    addBlock(to_offset, to_offset,
                             to_jacobian.transpose() * to_jacobian);
                    gradient.segment<6>(to_offset) +=
                        to_jacobian.transpose() * residual;
                }
                if (edge.from > 0 && edge.to > 0)
                {
                    const Matrix6d cross =
                        from_jacobian.transpose() * to_jacobian;
                    addBlock(from_offset, to_offset, cross);
                    addBlock(to_offset, from_offset, cross.transpose());
                }
            }
            for (int index = 0; index < dimensions; ++index)
                triplets.emplace_back(index, index, 1e-6);
            Eigen::SparseMatrix<double> hessian(dimensions, dimensions);
            hessian.setFromTriplets(triplets.begin(), triplets.end());
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(hessian);
            if (solver.info() != Eigen::Success) return false;
            const Eigen::VectorXd step = solver.solve(-gradient);
            if (solver.info() != Eigen::Success || !step.allFinite()) return false;
            double maximum_step = 0.0;
            for (int index = 1; index < pose_count; ++index)
            {
                Vector6d delta = step.segment<6>(6 * (index - 1));
                const double translation_norm = delta.head<3>().norm();
                const double rotation_norm = delta.tail<3>().norm();
                if (translation_norm > 0.5)
                    delta.head<3>() *= 0.5 / translation_norm;
                if (rotation_norm > 0.15)
                    delta.tail<3>() *= 0.15 / rotation_norm;
                poses[index] = graphPerturbation(poses[index],delta,freeze_yaw);
                maximum_step = std::max(maximum_step, delta.norm());
            }
            if (maximum_step < 1e-5) break;
        }
        const double final_cost = cost(poses);
        if (!std::isfinite(final_cost) || final_cost > initial_cost+1e-8) return false;
        // Commit only after the candidate graph passes; failures leave all
        // optimized poses and map_to_raw unchanged.
        for (const Edge& edge : edges_) {
            if (!edge.loop) continue;
            const Vector6d residual = poseResidual(poses[edge.from],poses[edge.to],edge.measurement);
            if (residual.head<3>().norm() > graph_max_loop_residual_translation_ ||
                residual.tail<3>().norm() > graph_max_loop_residual_rotation_deg_*kPi/180.0)
                return false;
        }
        if (validate_graph_geometry_ && !edges_.empty() && edges_.back().loop) {
            const Edge& edge=edges_.back();
            CloudT::Ptr source=downsample(keyframes_[edge.to].local_cloud,fine_icp_voxel_size_);
            CloudT::Ptr target=buildCandidateSubmap(edge.from,edge.to);
            const auto old_relative=keyframes_[edge.from].optimized_pose.inverse()*keyframes_[edge.to].optimized_pose;
            const auto new_relative=poses[edge.from].inverse()*poses[edge.to];
            const Verification before=verifyGeometry(source,target,old_relative.matrix().cast<float>(),false);
            const Verification after=verifyGeometry(source,target,new_relative.matrix().cast<float>(),false);
            CloudT::Ptr previous(new CloudT), previous_after(new CloudT);
            const auto old_previous=keyframes_[edge.to].optimized_pose.inverse()*keyframes_[edge.to-1].optimized_pose;
            const auto new_previous=poses[edge.to].inverse()*poses[edge.to-1];
            pcl::transformPointCloud(*keyframes_[edge.to-1].local_cloud,*previous,old_previous.matrix().cast<float>());
            pcl::transformPointCloud(*keyframes_[edge.to-1].local_cloud,*previous_after,new_previous.matrix().cast<float>());
            const auto held_before=verifyGeometry(previous,target,old_relative.matrix().cast<float>(),false);
            const auto held_after=verifyGeometry(previous_after,target,new_relative.matrix().cast<float>(),false);
            last_graph_gain_=1.-after.clipped_mse/std::max(1e-12,before.clipped_mse);
            last_graph_holdout_gain_=1.-held_after.clipped_mse/std::max(1e-12,held_before.clipped_mse);
            if (!sufficientGeometry(after,false) || !sufficientGeometry(held_after,false) ||
                after.clipped_mse>before.clipped_mse*(1.+graph_geometry_max_regression_)+1e-12 ||
                held_after.clipped_mse>held_before.clipped_mse*(1.+graph_geometry_max_regression_)+1e-12) {
                ++rejected_graph_geometry_count_; return false;
            }
        }
        for (int index = 0; index < pose_count; ++index)
            keyframes_[index].optimized_pose = poses[index];
        map_to_raw_ = keyframes_.back().optimized_pose *
                      keyframes_.back().raw_pose.inverse();
        map_dirty_ = true;
        return true;
    }

    void loopTimer(const ros::TimerEvent&)
    {
        if (loop_search_running_.exchange(true)) return;
        std::unique_lock<std::mutex> lock(mutex_);
        if (keyframes_.size() <= static_cast<std::size_t>(min_loop_keyframe_gap_))
        {
            loop_search_running_ = false;
            return;
        }
        const int current = static_cast<int>(keyframes_.size()) - 1;
        if (looped_keyframes_.count(current) != 0U ||
            current == last_checked_keyframe_ ||
            (last_accepted_loop_keyframe_ >= 0 &&
             current - last_accepted_loop_keyframe_ < loop_cooldown_keyframes_))
        {
            loop_search_running_ = false;
            return;
        }
        last_checked_keyframe_ = current;
        const ros::WallTime search_started = ros::WallTime::now();
        const std::vector<int> spatial = spatialCandidates(current);
        last_candidate_diagnostics_ = CandidateDiagnostics();
        last_candidate_diagnostics_.current = current;
        last_candidate_diagnostics_.spatial_count = spatial.size();
        if (spatial.empty())
        {
            ++no_spatial_candidate_count_;
            last_candidate_diagnostics_.stage = "no_spatial_candidate";
            last_candidate_diagnostics_.processing_ms =
                (ros::WallTime::now() - search_started).toSec() * 1000.0;
            loop_search_running_ = false;
            lock.unlock();
            publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                               "no spatial loop candidate");
            return;
        }

        const std::vector<LoopCandidate> candidates =
            descriptorCandidates(current, spatial);
        if (!candidates.empty())
        {
            last_candidate_diagnostics_.candidate = candidates.front().index;
            last_candidate_diagnostics_.spatial_distance =
                candidates.front().spatial_distance;
            last_candidate_diagnostics_.descriptor_score =
                candidates.front().descriptor_score;
            last_candidate_diagnostics_.sector_shift_deg =
                candidates.front().sector_shift * scan_context_.sectorAngle() *
                180.0 / kPi;
        }

        int icp_attempts = 0;
        int candidate = -1;
        int sector_shift = 0;
        double descriptor_score = std::numeric_limits<double>::infinity();
        double spatial_distance = std::numeric_limits<double>::infinity();
        double fitness = std::numeric_limits<double>::infinity();
        double overlap = 0.0;
        double best_quality = std::numeric_limits<double>::infinity();
        Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
        Verification selected_verification, selected_holdout;
        bool selected_small=false;
        for (const LoopCandidate& proposal : candidates)
        {
            const bool geometric_fallback =
                proposal.spatial_distance <= geometric_fallback_radius_ &&
                proposal.descriptor_score <= scan_context_geometric_threshold_;
            if (!proposal.strict_descriptor_match && !geometric_fallback)
                continue;
            if (icp_attempts >= max_icp_candidates_) break;
            ++icp_attempts;
            ++icp_attempt_count_;
            Eigen::Isometry3d proposed_measurement = Eigen::Isometry3d::Identity();
            double proposed_fitness = std::numeric_limits<double>::infinity();
            double proposed_overlap = 0.0;
            if (!registerLoop(current, proposal.index, proposal.sector_shift,
                              &proposed_measurement, &proposed_fitness,
                              &proposed_overlap))
            {
                ++rejected_icp_count_;
                if (proposed_fitness < fitness)
                {
                    fitness = proposed_fitness;
                    overlap = proposed_overlap;
                }
                ROS_WARN("[LoopClosure] %d<->%d rejected by ICP "
                         "(distance=%.2f SC=%.3f fitness=%.3f overlap=%.2f).",
                         proposal.index, current, proposal.spatial_distance,
                         proposal.descriptor_score, proposed_fitness,
                         proposed_overlap);
                continue;
            }
            const double quality = proposed_fitness /
                std::max(0.05, proposed_overlap);
            if (quality < best_quality)
            {
                best_quality = quality;
                candidate = proposal.index;
                sector_shift = proposal.sector_shift;
                descriptor_score = proposal.descriptor_score;
                spatial_distance = proposal.spatial_distance;
                fitness = proposed_fitness;
                overlap = proposed_overlap;
                measurement = proposed_measurement;
                selected_verification = last_verification_;
                selected_holdout = last_holdout_;
                selected_small=last_registration_small_;
            }
        }

        if (icp_attempts == 0)
        {
            ++rejected_descriptor_count_;
            last_candidate_diagnostics_.stage = "scan_context_rejected";
            last_candidate_diagnostics_.processing_ms =
                (ros::WallTime::now() - search_started).toSec() * 1000.0;
            loop_search_running_ = false;
            lock.unlock();
            publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                               "Scan Context rejected all spatial candidates");
            return;
        }
        if (candidate < 0)
        {
            last_candidate_diagnostics_.stage = "icp_rejected";
            last_candidate_diagnostics_.fitness = fitness;
            last_candidate_diagnostics_.overlap = overlap;
            last_candidate_diagnostics_.processing_ms =
                (ros::WallTime::now() - search_started).toSec() * 1000.0;
            loop_search_running_ = false;
            lock.unlock();
            publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                               "ICP rejected all descriptor candidates");
            return;
        }

        // A single registration never changes the graph. Two distinct
        // keyframes must agree on the correction at the current body origin.
        const Eigen::Isometry3d proposed_correction =
            keyframes_[candidate].raw_pose*measurement*keyframes_[current].raw_pose.inverse();
        const Eigen::Vector3d body = keyframes_[current].raw_pose.translation();
        ++verified_revisit_count_;
        const bool strict_small=selected_small || pending_small_;
        const double confirm_translation=strict_small ?
            std::min(loop_confirmation_translation_,small_loop_confirmation_translation_) : loop_confirmation_translation_;
        const double confirm_rotation=strict_small ?
            std::min(loop_confirmation_rotation_deg_,small_loop_confirmation_rotation_deg_) : loop_confirmation_rotation_deg_;
        const bool consistent = pending_confirmation_keyframe_ >= 0 &&
            current>pending_confirmation_keyframe_ &&
            current-pending_confirmation_keyframe_ <= 3 &&
            (proposed_correction*body-pending_correction_*body).norm() <= confirm_translation &&
            rotationDistance(proposed_correction.rotation(),pending_correction_.rotation()) <=
                confirm_rotation*kPi/180.0;
        pending_confirmation_count_ = consistent ? pending_confirmation_count_+1 : 1;
        pending_small_ = consistent ? strict_small : selected_small;
        pending_confirmation_keyframe_ = current;
        pending_correction_ = proposed_correction;
        const int confirmation_needed=pending_small_ ?
            std::max(loop_confirmation_keyframes_,small_loop_confirmation_keyframes_) : loop_confirmation_keyframes_;
        if (pending_confirmation_count_ < std::max(1,confirmation_needed)) {
            ++awaiting_confirmation_count_;
            last_candidate_diagnostics_.stage = "awaiting_temporal_confirmation";
            last_candidate_diagnostics_.candidate = candidate;
            last_candidate_diagnostics_.fitness = fitness;
            last_candidate_diagnostics_.overlap = overlap;
            last_candidate_diagnostics_.processing_ms =
                (ros::WallTime::now()-search_started).toSec()*1000.0;
            loop_search_running_ = false;
            lock.unlock();
            publishDiagnostics(diagnostic_msgs::DiagnosticStatus::WARN,
                               "geometric match awaiting another independent keyframe");
            return;
        }
        const bool confirmed_small=pending_small_;
        pending_confirmation_count_ = 0;
        pending_confirmation_keyframe_ = -1; pending_small_=false;

        Edge loop_edge;
        last_verification_ = selected_verification;
        last_holdout_ = selected_holdout;
        loop_edge.from = candidate;
        loop_edge.to = current;
        loop_edge.measurement = measurement;
        // Base weights are already sqrt-information. Do not square a huge
        // fitness-derived multiplier and let one dense floor overpower odom.
        const double confidence = clampValue(
            overlap * verification_max_rmse_/std::max(0.04,std::sqrt(fitness)), 0.5, 1.5);
        loop_edge.translation_weight = loop_translation_weight_ * confidence;
        loop_edge.rotation_weight = loop_rotation_weight_ * confidence;
        if (confirmed_small) {
            loop_edge.translation_weight*=small_loop_weight_scale_;
            loop_edge.rotation_weight=0.; // Do not inject residual-scale yaw evidence.
        }
        loop_edge.loop = true;
        loop_edge.fitness = fitness;
        edges_.push_back(loop_edge);
        if (!optimizePoseGraph(confirmed_small))
        {
            edges_.pop_back();
            ++optimizer_failure_count_;
            last_candidate_diagnostics_.stage = "optimizer_failed";
            last_candidate_diagnostics_.candidate = candidate;
            last_candidate_diagnostics_.spatial_distance = spatial_distance;
            last_candidate_diagnostics_.descriptor_score = descriptor_score;
            last_candidate_diagnostics_.fitness = fitness;
            last_candidate_diagnostics_.overlap = overlap;
            last_candidate_diagnostics_.processing_ms =
                (ros::WallTime::now() - search_started).toSec() * 1000.0;
            loop_search_running_ = false;
            lock.unlock();
            publishDiagnostics(diagnostic_msgs::DiagnosticStatus::ERROR,
                               "optimizer failed; loop discarded");
            return;
        }
        looped_keyframes_.insert(current);
        last_accepted_loop_keyframe_ = current;
        ++accepted_loop_count_;
        if (confirmed_small) ++accepted_small_loop_count_;
        last_candidate_diagnostics_.stage = "accepted";
        last_candidate_diagnostics_.candidate = candidate;
        last_candidate_diagnostics_.spatial_distance = spatial_distance;
        last_candidate_diagnostics_.descriptor_score = descriptor_score;
        last_candidate_diagnostics_.sector_shift_deg =
            sector_shift * scan_context_.sectorAngle() * 180.0 / kPi;
        last_candidate_diagnostics_.fitness = fitness;
        last_candidate_diagnostics_.overlap = overlap;
        last_candidate_diagnostics_.processing_ms =
            (ros::WallTime::now() - search_started).toSec() * 1000.0;
        ROS_INFO("[LoopClosure] accepted %d<->%d SC=%.3f shift=%.1fdeg "
                 "distance=%.2f ICP=%.3f overlap=%.2f.",
                 candidate, current, descriptor_score,
                 sector_shift * scan_context_.sectorAngle() * 180.0 / kPi,
                 spatial_distance, fitness, overlap);
        loop_search_running_ = false;
        lock.unlock();
        publishPaths();
        publishLoopMarkers();
        publishDiagnostics(diagnostic_msgs::DiagnosticStatus::OK,
                           "loop accepted and graph optimized");
    }

    void publishPaths()
    {
        fastlio_bridge::LoopGraph graph;
        nav_msgs::Path optimized;
        nav_msgs::Path raw;
        optimized.header.stamp = ros::Time::now();
        raw.header.stamp = optimized.header.stamp;
        optimized.header.frame_id = world_frame_;
        raw.header.frame_id = world_frame_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            graph.revision=accepted_loop_count_;
            optimized.poses.reserve(keyframes_.size());
            raw.header.frame_id = raw_frame_;
            raw.poses.reserve(keyframes_.size());
            for (const Keyframe& keyframe : keyframes_)
            {
                geometry_msgs::PoseStamped optimized_pose;
                optimized_pose.header.stamp = keyframe.stamp;
                optimized_pose.header.frame_id = world_frame_;
                optimized_pose.pose = eigenPose(keyframe.optimized_pose);
                optimized.poses.push_back(optimized_pose);
                geometry_msgs::PoseStamped raw_pose = optimized_pose;
                raw_pose.header.frame_id = raw_frame_;
                raw_pose.pose = eigenPose(keyframe.raw_pose);
                raw.poses.push_back(raw_pose);
            }
        }
        optimized_path_pub_.publish(optimized);
        raw_path_pub_.publish(raw);
        graph.header=optimized.header;
        graph.raw=std::move(raw);
        graph.optimized=std::move(optimized);
        graph_pub_.publish(graph);
    }

    void publishLoopMarkers()
    {
        visualization_msgs::Marker nodes;
        visualization_msgs::Marker edges;
        nodes.header.frame_id = world_frame_;
        edges.header.frame_id = world_frame_;
        nodes.header.stamp = ros::Time::now();
        edges.header.stamp = nodes.header.stamp;
        nodes.ns = "loop_nodes";
        edges.ns = "loop_edges";
        nodes.id = 0;
        edges.id = 1;
        nodes.type = visualization_msgs::Marker::SPHERE_LIST;
        edges.type = visualization_msgs::Marker::LINE_LIST;
        nodes.action = visualization_msgs::Marker::ADD;
        edges.action = visualization_msgs::Marker::ADD;
        nodes.pose.orientation.w = 1.0;
        edges.pose.orientation.w = 1.0;
        nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.25;
        edges.scale.x = 0.08;
        nodes.color.r = 0.1F;
        nodes.color.g = 1.0F;
        nodes.color.b = 0.2F;
        nodes.color.a = 1.0F;
        edges.color.r = 1.0F;
        edges.color.g = 0.2F;
        edges.color.b = 0.1F;
        edges.color.a = 1.0F;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const Edge& edge : edges_)
            {
                if (!edge.loop) continue;
                geometry_msgs::Point from;
                geometry_msgs::Point to;
                const Eigen::Vector3d from_position =
                    keyframes_[edge.from].optimized_pose.translation();
                const Eigen::Vector3d to_position =
                    keyframes_[edge.to].optimized_pose.translation();
                from.x = from_position.x();
                from.y = from_position.y();
                from.z = from_position.z();
                to.x = to_position.x();
                to.y = to_position.y();
                to.z = to_position.z();
                nodes.points.push_back(from);
                nodes.points.push_back(to);
                edges.points.push_back(from);
                edges.points.push_back(to);
            }
        }
        visualization_msgs::MarkerArray array;
        array.markers.push_back(nodes);
        array.markers.push_back(edges);
        loop_markers_pub_.publish(array);
    }

    void mapTimer(const ros::TimerEvent&)
    {
        std::vector<Keyframe> snapshot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!map_dirty_ || keyframes_.empty()) return;
            snapshot = keyframes_;
            map_dirty_ = false;
        }
        CloudT::Ptr map(new CloudT);
        std::size_t expected_points = 0;
        for (const Keyframe& keyframe : snapshot)
            expected_points += keyframe.local_cloud->size();
        map->reserve(expected_points);
        for (const Keyframe& keyframe : snapshot)
        {
            CloudT transformed;
            pcl::transformPointCloud(*keyframe.local_cloud, transformed,
                                     keyframe.optimized_pose.matrix().cast<float>());
            *map += transformed;
        }
        map = downsample(map, map_voxel_size_);
        sensor_msgs::PointCloud2 message;
        pcl::toROSMsg(*map, message);
        message.header.stamp = ros::Time::now();
        message.header.frame_id = world_frame_;
        optimized_map_pub_.publish(message);
        ROS_INFO("[LoopClosure] rebuilt map: %zu keyframes, %zu points.",
                 snapshot.size(), map->size());
    }

    void publishDiagnostics(std::uint8_t level, const std::string& message)
    {
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus status;
        status.name = "AstraDrone/loop_closure_backend";
        status.hardware_id = "FAST-LIO2";
        status.level = level;
        status.message = message;
        auto add = [&status](const std::string& key, const std::string& value)
        {
            diagnostic_msgs::KeyValue item;
            item.key = key;
            item.value = value;
            status.values.push_back(item);
        };
        auto addCount = [&add](const std::string& key, std::size_t value)
        {
            add(key, std::to_string(value));
        };
        auto addDouble = [&add](const std::string& key, double value)
        {
            add(key, std::isfinite(value) ? std::to_string(value) : "nan");
        };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            addCount("keyframes", keyframes_.size());
            addCount("graph_edges", edges_.size());
            addCount("accepted_loops", accepted_loop_count_);
            addCount("verified_revisit_proposals",verified_revisit_count_);
            addCount("accepted_small_loops",accepted_small_loop_count_);
            addCount("rejected_small_evidence_seeds",rejected_small_evidence_count_);
            addCount("rejected_graph_geometry",rejected_graph_geometry_count_);
            addDouble("last_graph_gain",last_graph_gain_);
            addDouble("last_graph_holdout_gain",last_graph_holdout_gain_);
            addCount("no_spatial_candidates", no_spatial_candidate_count_);
            addCount("rejected_scan_context", rejected_descriptor_count_);
            addCount("icp_attempts", icp_attempt_count_);
            addCount("rejected_icp", rejected_icp_count_);
            addCount("optimizer_failures", optimizer_failure_count_);
            addCount("rejected_sync", rejected_sync_count_);
            addCount("rejected_degenerate_seeds", rejected_degenerate_count_);
            addCount("rejected_geometry_seeds", rejected_geometry_count_);
            addCount("rejected_holdout_seeds", rejected_holdout_count_);
            addCount("rejected_gain_seeds", rejected_gain_count_);
            addCount("rejected_tiny_correction_seeds", rejected_tiny_correction_count_);
            addCount("rejected_large_correction_seeds", rejected_large_correction_count_);
            addCount("awaiting_temporal_confirmation", awaiting_confirmation_count_);
            addDouble("last_verification_rmse_m", last_verification_.rmse);
            addDouble("last_verification_eigen_ratio", last_verification_.eigen_ratio);
            addDouble("last_holdout_overlap", last_holdout_.overlap);
            addDouble("last_holdout_rmse_m", last_holdout_.rmse);
            add("last_stage", last_candidate_diagnostics_.stage);
            add("last_current_keyframe",
                std::to_string(last_candidate_diagnostics_.current));
            add("last_candidate_keyframe",
                std::to_string(last_candidate_diagnostics_.candidate));
            addCount("last_spatial_candidate_count",
                     last_candidate_diagnostics_.spatial_count);
            addDouble("last_spatial_distance_m",
                      last_candidate_diagnostics_.spatial_distance);
            addDouble("last_scan_context_score",
                      last_candidate_diagnostics_.descriptor_score);
            addDouble("last_sector_shift_deg",
                      last_candidate_diagnostics_.sector_shift_deg);
            addDouble("last_icp_fitness",
                      last_candidate_diagnostics_.fitness);
            addDouble("last_icp_overlap",
                      last_candidate_diagnostics_.overlap);
            addDouble("last_processing_ms",
                      last_candidate_diagnostics_.processing_ms);
        }
        array.status.push_back(status);
        diagnostics_pub_.publish(array);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_filter_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_filter_sub_;
    message_filters::Synchronizer<SyncPolicy> synchronizer_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Publisher optimized_odom_pub_;
    ros::Publisher corrected_scan_pub_;
    ros::Publisher optimized_map_pub_;
    ros::Publisher optimized_path_pub_;
    ros::Publisher raw_path_pub_;
    ros::Publisher graph_pub_;
    ros::Publisher loop_markers_pub_;
    ros::Publisher correction_pub_;
    ros::Publisher diagnostics_pub_;
    ros::Publisher seed_evidence_pub_;
    ros::Publisher input_ack_pub_;
    ros::ServiceServer process_service_;
    ros::ServiceServer freeze_service_;
    std::atomic<bool> frozen_keyframes_{false};
    bool deterministic_replay_{false};
    ros::Timer loop_timer_;
    ros::Timer map_timer_;
    mutable std::mutex mutex_;
    std::atomic<bool> loop_search_running_{false};
    std::vector<Keyframe> keyframes_;
    std::vector<Edge> edges_;
    std::set<int> looped_keyframes_;
    Eigen::Isometry3d map_to_raw_{Eigen::Isometry3d::Identity()};
    ScanContextDescriptor scan_context_;
    bool map_dirty_{false};
    int last_checked_keyframe_{-1};
    int last_accepted_loop_keyframe_{-1};
    std::size_t accepted_loop_count_{0};
    std::size_t no_spatial_candidate_count_{0};
    std::size_t rejected_descriptor_count_{0};
    std::size_t icp_attempt_count_{0};
    std::size_t rejected_icp_count_{0};
    std::size_t optimizer_failure_count_{0};
    CandidateDiagnostics last_candidate_diagnostics_;

    std::string odom_topic_;
    std::string cloud_topic_;
    std::string world_frame_;
    std::string optimized_odom_topic_;
    std::string corrected_scan_topic_;
    std::string optimized_map_topic_;
    std::string optimized_path_topic_;
    std::string raw_path_topic_;
    std::string raw_frame_{"map"};
    double keyframe_distance_{1.0};
    double keyframe_angle_deg_{10.0};
    double keyframe_voxel_size_{0.25};
    int max_keyframe_points_{18000};
    int min_keyframe_points_{250};
    int max_keyframes_{3000};
    double loop_detection_rate_{0.5};
    double min_loop_time_{20.0};
    int min_loop_keyframe_gap_{25};
    int loop_cooldown_keyframes_{10};
    double loop_search_radius_{8.0};
    double max_loop_vertical_separation_{1.0};
    int max_spatial_candidates_{15};
    double scan_context_threshold_{0.22};
    double scan_context_geometric_threshold_{0.35};
    double geometric_fallback_radius_{2.5};
    int max_icp_candidates_{5};
    int descriptor_submap_keyframes_{4};
    double descriptor_voxel_size_{0.35};
    int descriptor_max_points_{40000};
    bool use_scan_context_yaw_seeds_{true};
    int submap_neighbors_{5};
    double icp_voxel_size_{0.30};
    double icp_max_correspondence_distance_{1.2};
    int icp_max_iterations_{60};
    double icp_fitness_threshold_{0.30};
    double icp_min_overlap_{0.35};
    double max_loop_translation_correction_{4.0};
    double max_loop_rotation_correction_deg_{35.0};
    int optimizer_iterations_{8};
    double optimizer_huber_delta_{2.5};
    double odom_translation_weight_{30.0};
    double odom_rotation_weight_{40.0};
    double loop_translation_weight_{12.0};
    double loop_rotation_weight_{18.0};
    double map_publish_rate_{0.2};
    double map_voxel_size_{0.20};
    bool publish_corrected_scan_{true};
    double max_sync_time_difference_{0.02};
    double verification_distance_{0.25};
    double verification_min_overlap_{0.60};
    double verification_max_rmse_{0.14};
    double verification_min_gain_{0.02};
    double verification_min_eigen_ratio_{0.0001};
    double fine_icp_voxel_size_{0.12};
    double fine_icp_correspondence_distance_{0.40};
    double min_loop_translation_correction_{0.04};
    double min_loop_rotation_correction_deg_{0.30};
    double graph_max_loop_residual_translation_{0.25};
    double graph_max_loop_residual_rotation_deg_{3.0};
    Verification last_verification_;
    Verification last_holdout_;
    std::atomic<std::size_t> rejected_sync_count_{0};
    std::size_t rejected_degenerate_count_{0}, rejected_geometry_count_{0};
    std::size_t rejected_holdout_count_{0}, rejected_gain_count_{0};
    std::size_t rejected_tiny_correction_count_{0}, rejected_large_correction_count_{0};
    int loop_confirmation_keyframes_{2};
    double loop_confirmation_translation_{0.10}, loop_confirmation_rotation_deg_{1.0};
    int pending_confirmation_keyframe_{-1}, pending_confirmation_count_{0};
    Eigen::Isometry3d pending_correction_{Eigen::Isometry3d::Identity()};
    std::size_t awaiting_confirmation_count_{0};
    bool gravity_aligned_graph_{true};
    double correction_noise_multiplier_{2.0};
    bool allow_verified_small_loops_{false}, last_registration_small_{false}, pending_small_{false};
    double small_loop_min_holdout_gain_{0.005},small_loop_weight_scale_{0.5};
    int small_loop_confirmation_keyframes_{3};
    double small_loop_confirmation_translation_{0.03},small_loop_confirmation_rotation_deg_{0.3};
    bool validate_graph_geometry_{true};
    double graph_geometry_max_regression_{0.01};
    std::size_t verified_revisit_count_{0},accepted_small_loop_count_{0};
    std::size_t rejected_small_evidence_count_{0},rejected_graph_geometry_count_{0};
    double last_graph_gain_{0.},last_graph_holdout_gain_{0.};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loop_closure_backend");
    LoopClosureBackend backend;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
