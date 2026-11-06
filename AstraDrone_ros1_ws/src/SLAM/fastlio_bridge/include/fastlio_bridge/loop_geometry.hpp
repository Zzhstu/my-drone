#pragma once

#include <fastlio_bridge/LoopGraph.h>
#include <fastlio_bridge/DynamicObjectArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace online_loop {

inline Eigen::Isometry3d poseMatrix(const geometry_msgs::Pose& pose)
{
    Eigen::Quaterniond q(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
    Eigen::Vector3d p(pose.position.x,pose.position.y,pose.position.z);
    if (!p.allFinite() || !q.coeffs().allFinite() || q.norm()<1e-8)
        throw std::invalid_argument("invalid graph/odometry pose");
    Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
    result.linear()=q.normalized().toRotationMatrix();result.translation()=p;return result;
}

inline geometry_msgs::Pose poseMessage(const Eigen::Isometry3d& pose)
{
    geometry_msgs::Pose result;
    result.position.x=pose.translation().x();result.position.y=pose.translation().y();result.position.z=pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());q.normalize();
    result.orientation.x=q.x();result.orientation.y=q.y();result.orientation.z=q.z();result.orientation.w=q.w();
    return result;
}

class Graph
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    fastlio_bridge::LoopGraph message;
    std::vector<double> stamps;
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> translations;
    std::vector<Eigen::Quaterniond,Eigen::aligned_allocator<Eigen::Quaterniond>> rotations;

    explicit Graph(const fastlio_bridge::LoopGraph& input) : message(input)
    {
        if (input.raw.header.frame_id!="map" || input.optimized.header.frame_id!="loop_map" ||
            input.raw.poses.empty() || input.raw.poses.size()!=input.optimized.poses.size())
            throw std::invalid_argument("unpaired global graph or wrong frame");
        for (std::size_t i=0;i<input.raw.poses.size();++i)
        {
            const auto& raw=input.raw.poses[i];const auto& opt=input.optimized.poses[i];
            const double stamp=raw.header.stamp.toSec();
            if (raw.header.stamp!=opt.header.stamp || stamp<=0 ||
                (!stamps.empty() && stamp<=stamps.back()))
                throw std::invalid_argument("graph acquisition stamps must strictly increase");
            const auto correction=poseMatrix(opt.pose)*poseMatrix(raw.pose).inverse();
            if (input.revision==0 && !correction.matrix().isApprox(Eigen::Matrix4d::Identity(),1e-8))
                throw std::invalid_argument("revision zero must be identity");
            stamps.push_back(stamp);translations.push_back(correction.translation());
            rotations.emplace_back(correction.rotation());rotations.back().normalize();
        }
    }

    Eigen::Isometry3d at(double stamp) const
    {
        // Retain exact raw XYZ/pose bytes when the graph has no accepted loop.
        if (!message.revision) return Eigen::Isometry3d::Identity();
        auto found=std::upper_bound(stamps.begin(),stamps.end(),stamp);
        const std::size_t b=std::distance(stamps.begin(),found);
        Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
        if (!b || b==stamps.size())
        {
            const std::size_t i=!b ? 0 : stamps.size()-1;
            result.translation()=translations[i];result.linear()=rotations[i].toRotationMatrix();
        }
        else
        {
            const double w=(stamp-stamps[b-1])/(stamps[b]-stamps[b-1]);
            result.translation()=(1-w)*translations[b-1]+w*translations[b];
            result.linear()=rotations[b-1].slerp(w,rotations[b]).normalized().toRotationMatrix();
        }
        return result;
    }
};

inline sensor_msgs::PointCloud2::ConstPtr transformCloud(
    const sensor_msgs::PointCloud2::ConstPtr& source,const Eigen::Isometry3d& transform)
{
    if (source->header.frame_id!="map") throw std::invalid_argument("cloud must be local map frame");
    sensor_msgs::PointCloud2::Ptr output(new sensor_msgs::PointCloud2(*source));
    output->header.frame_id="loop_map";
    if (transform.matrix().isApprox(Eigen::Matrix4d::Identity(),1e-12)) return output;
    if (source->is_bigendian) throw std::invalid_argument("big-endian online cloud unsupported");
    std::vector<std::uint32_t> offsets;
    for (const std::string name : {"x","y","z"})
    {
        auto f=std::find_if(source->fields.begin(),source->fields.end(),[&](const auto& field){return field.name==name;});
        if (f==source->fields.end() || f->datatype!=sensor_msgs::PointField::FLOAT32 ||
            f->offset+4>source->point_step) throw std::invalid_argument("online scan requires FLOAT32 XYZ");
        offsets.push_back(f->offset);
    }
    if (source->row_step<source->width*source->point_step ||
        source->data.size()<std::size_t(source->height)*source->row_step)
        throw std::invalid_argument("malformed online point cloud payload");
    for (std::uint32_t y=0;y<source->height;++y)
        for (std::uint32_t x=0;x<source->width;++x)
        {
            const std::size_t start=std::size_t(y)*source->row_step+x*source->point_step;
            Eigen::Vector3d p;
            for (int i=0;i<3;++i){float v;std::memcpy(&v,&source->data[start+offsets[i]],4);p[i]=v;}
            if (!p.allFinite()) continue;
            p=transform*p;
            for (int i=0;i<3;++i){const float v=float(p[i]);std::memcpy(&output->data[start+offsets[i]],&v,4);}
        }
    return output;
}

inline nav_msgs::Odometry::ConstPtr transformOdom(
    const nav_msgs::Odometry::ConstPtr& source,const Eigen::Isometry3d& transform)
{
    if (source->header.frame_id!="map") throw std::invalid_argument("odometry must be local map frame");
    nav_msgs::Odometry::Ptr output(new nav_msgs::Odometry(*source));
    output->header.frame_id="loop_map";
    if (!transform.matrix().isApprox(Eigen::Matrix4d::Identity(),1e-12))
        output->pose.pose=poseMessage(transform*poseMatrix(source->pose.pose));
    return output;
}

inline fastlio_bridge::DynamicObjectArray::ConstPtr transformObjects(
    const fastlio_bridge::DynamicObjectArray::ConstPtr& source,const Eigen::Isometry3d& transform)
{
    if (source->header.frame_id!="map") throw std::invalid_argument("objects must be local map frame");
    fastlio_bridge::DynamicObjectArray::Ptr output(new fastlio_bridge::DynamicObjectArray(*source));
    output->header.frame_id="loop_map";
    if (transform.matrix().isApprox(Eigen::Matrix4d::Identity(),1e-12)) return output;
    const Eigen::Matrix3d rotation=transform.rotation();
    for (auto& object : output->objects)
    {
        object.pose=poseMessage(transform*poseMatrix(object.pose));
        for (auto* vector : {&object.twist.linear,&object.twist.angular})
        {
            const Eigen::Vector3d value=rotation*Eigen::Vector3d(vector->x,vector->y,vector->z);
            vector->x=value.x();vector->y=value.y();vector->z=value.z();
        }
        const Eigen::Vector3d size=rotation.cwiseAbs()*Eigen::Vector3d(object.size.x,object.size.y,object.size.z);
        object.size.x=size.x();object.size.y=size.y();object.size.z=size.z();
    }
    return output;
}
} // namespace online_loop
