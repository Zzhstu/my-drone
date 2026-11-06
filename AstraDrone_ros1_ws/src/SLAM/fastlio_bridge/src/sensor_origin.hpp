#pragma once

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <vector>

// FAST-LIO publishes the IMU origin. Navigation/PX4 use the vehicle body
// origin. This translation-only option requires coincident IMU/body axes;
// its zero default preserves existing real-hardware launch behavior.
inline Eigen::Vector3d loadImuOriginInBody(ros::NodeHandle& pnh)
{
    std::vector<double> xyz;
    xyz = {0., 0., 0.};
    if (pnh.hasParam("imu_origin_in_body") && !pnh.getParam("imu_origin_in_body", xyz))
        throw std::invalid_argument("imu_origin_in_body must be a numeric ROS array");
    if (xyz.size() != 3 || !std::isfinite(xyz[0]) ||
        !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
        throw std::invalid_argument("imu_origin_in_body must contain three finite coordinates");
    return Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
}

inline Eigen::Isometry3d imuPoseToBody(const Eigen::Isometry3d& imu,
                                     const Eigen::Vector3d& imu_in_body)
{
    Eigen::Isometry3d body = imu;
    body.translation() -= imu.linear() * imu_in_body;
    return body;
}
