#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>

namespace astra {

// Candidate suppression only. This result NEVER authorizes occupancy deletion
// or a free-space update. Inputs must be freshly measured mature cells;
// the queried voxel and its immediate neighbors must be excluded by the caller.
struct SurfaceEvidence {
    bool supported{false};
    unsigned inliers{0};
    double distance{-1.0}, rms{-1.0}, minor_span{0.0};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()}, center{Eigen::Vector3d::Zero()};
};

inline SurfaceEvidence staticSurfaceEvidence(
    const Eigen::Vector3d& query, const std::vector<Eigen::Vector3d>& points,
    double max_distance, double plane_tolerance, unsigned min_points,
    double min_span, const std::vector<Eigen::Vector3d>* current=nullptr)
{
    SurfaceEvidence result;
    if (points.size() < min_points || !query.allFinite()) return result;
    // Deterministic bounded RANSAC tolerates floor/rack junctions. No world
    // model, semantic label, axis alignment, or assumed shelf pose is used.
    std::uint32_t seed = 0x83a19bU;
    auto index = [&](std::size_t count) { seed = 1664525U*seed+1013904223U; return seed%count; };
    std::vector<std::size_t> best;
    unsigned best_current=0;
    for (unsigned trial=0; trial<96; ++trial) {
        // Current returns propose a plane; EARLIER support must validate it.
        // A sparse new face otherwise has negligible three-point sampling
        // probability among a much denser opposite face in the old map.
        const auto& seeds=(current && current->size()>=3 && trial<48) ? *current : points;
        const auto i=index(seeds.size()), j=index(seeds.size()), k=index(seeds.size());
        Eigen::Vector3d normal=(seeds[j]-seeds[i]).cross(seeds[k]-seeds[i]);
        if (normal.norm()<1e-6) continue;
        normal.normalize();
        // This guard targets broad vertical structural faces. A floor or a
        // narrow person's silhouette is deliberately insufficient evidence.
        if (std::abs(normal.z())>.35 ||
            std::abs(normal.dot(query-seeds[i]))>max_distance) continue;
        std::vector<std::size_t> inliers;
        for (std::size_t p=0; p<points.size(); ++p)
            if (std::abs(normal.dot(points[p]-seeds[i]))<=plane_tolerance)
                inliers.push_back(p);
        if (inliers.size()<min_points) continue;
        unsigned matched=0;
        if (current) {
            for (const auto& point:*current)
                matched+=std::abs(normal.dot(point-seeds[i]))<=.04;
            if (matched<4) continue;
            if (inliers.size()<.15*points.size()) {
                // A newly revealed back face may be a small minority beside
                // a dense front/floor. Global density is not geometric
                // evidence against it. Admit it only when it explains most
                // CURRENT returns and has strong LOCAL prior consensus.
                // Oblique bridges between two real faces have off-plane
                // neighbors at their intersection and fail this check.
                if (matched<.80*current->size()) continue;
                unsigned nearby=0;
                for (const auto& point:points) {
                    bool close=false;
                    for (auto p:inliers)
                        if ((point-points[p]).squaredNorm()<=.18*.18) {close=true;break;}
                    nearby+=close;
                }
                if (inliers.size()<.70*nearby) continue;
            }
        }
        // The densest old face can be the OPPOSITE side of a shelf. Select
        // support for the actual foreground returns, rather than letting
        // that unrelated mature plane hide the newly observed back face.
        // A few samples on two separate faces can form a spurious oblique
        // bridge through a nearby person. Demand genuine prior consensus,
        // and rank current support together with that prior consensus.
        const auto score=std::uint64_t(matched)*inliers.size();
        const auto previous_score=std::uint64_t(best_current)*best.size();
        if ((current && score>previous_score) || (!current && inliers.size()>best.size())) {
            best=std::move(inliers);best_current=matched;
        }
    }
    if (best.size()<min_points) return result;
    Eigen::Vector3d mean=Eigen::Vector3d::Zero();
    for (auto i:best) mean+=points[i];
    mean/=double(best.size());
    Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
    for (auto i:best) { const Eigen::Vector3d d=points[i]-mean; covariance+=d*d.transpose(); }
    covariance/=double(best.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
    if (eig.info()!=Eigen::Success) return result;
    const Eigen::Vector3d normal=eig.eigenvectors().col(0);
    result.normal=normal;result.center=mean;
    result.inliers=best.size();
    result.distance=std::abs(normal.dot(query-mean));
    result.rms=std::sqrt(std::max(0.0,eig.eigenvalues()[0]));
    double span=1e9;
    for (int axis=1; axis<3; ++axis) {
        double low=1e9, high=-1e9;
        for (auto i:best) {
            const double value=eig.eigenvectors().col(axis).dot(points[i]-mean);
            low=std::min(low,value); high=std::max(high,value);
        }
        span=std::min(span,high-low);
    }
    result.minor_span=span;
    // The second eigenvalue additionally rejects a thin line with two distant
    // points that happens to have a large bounding box.
    result.supported=std::abs(normal.z())<=.35 && result.distance<=max_distance &&
        result.rms<=plane_tolerance && span>=min_span &&
        eig.eigenvalues()[1]>=.0256;
    return result;
}
}  // namespace astra
