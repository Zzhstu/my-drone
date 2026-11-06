#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace astra {

// A point that first appears in unknown space is not evidence of motion. Keep
// a short, revocable record of cells crossed by actual earlier measured rays.
// This is a motion-confirmation gate, NOT an occupancy/free-space map: it never
// removes a return, clears an obstacle, or authorizes navigation through a cell.
class ObservedFreeMotion {
public:
    struct Key {
        int x, y, z;
        bool operator==(const Key& other) const {
            return x==other.x && y==other.y && z==other.z;
        }
    };
    struct Hash {
        std::size_t operator()(const Key& key) const {
            std::size_t value=0;
            for (int part:{key.x,key.y,key.z})
                value^=std::hash<int>()(part)+0x9e3779b9U+(value<<6U)+(value>>2U);
            return value;
        }
    };
    struct Evidence {
        unsigned frames{0};
        double first{0.}, last{0.};
    };

    double resolution{.25}, lifetime{30.}, minimum_age{.15};
    unsigned minimum_frames{2}, max_rays{4096}, max_cells{200000};
    double endpoint_margin{.40}, max_range{6.0};

    bool previouslyFree(const Eigen::Vector3d& point, double stamp) const {
        auto found=free_.find(keyFor(point));
        return found!=free_.end() && found->second.frames>=minimum_frames &&
            stamp-found->second.first>=minimum_age &&
            stamp>found->second.last && stamp-found->second.last<=lifetime;
    }

    // Query previouslyFree BEFORE integrating this scan. A current ray is
    // never allowed to establish evidence for a return in its own scan.
    void integrate(const Eigen::Vector3d& origin,
                   const std::vector<Eigen::Vector3d>& endpoints, double stamp) {
        if (!origin.allFinite() || !std::isfinite(stamp) || stamp<=last_stamp_)
            return;
        if (last_stamp_>0. && stamp-last_stamp_>lifetime) free_.clear();
        last_stamp_=stamp;
        std::unordered_set<Key,Hash> occupied, measured, crossed;
        occupied.reserve(endpoints.size()*2);
        for (const auto& point:endpoints) {
            if (!point.allFinite() || (point-origin).norm()>max_range+resolution)
                continue;
            const Key key=keyFor(point);
            measured.insert(key);
            // Neighbor protection prevents endpoint/voxel boundaries, small
            // registration errors, and thin static surfaces from voting free.
            for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y)
                for (int z=-1;z<=1;++z)
                    occupied.insert({key.x+x,key.y+y,key.z+z});
        }
        const std::size_t stride=std::max<std::size_t>(1,
            (endpoints.size()+max_rays-1)/max_rays);
        for (std::size_t index=0;index<endpoints.size();index+=stride) {
            const Eigen::Vector3d ray=endpoints[index]-origin;
            const double length=ray.norm();
            if (!ray.allFinite() || length<=endpoint_margin) continue;
            const Eigen::Vector3d direction=ray/length;
            const double limit=std::min(max_range,length-endpoint_margin);
            for (double distance=resolution;distance<limit;distance+=resolution*.5) {
                const Key key=keyFor(origin+direction*distance);
                if (!occupied.count(key)) crossed.insert(key);
            }
        }
        // Dilated endpoints prevent NEW free votes. Revoke prior free only
        // where a return was actually measured: erasing the dilated halo would
        // destroy all evidence in front of an approaching real moving person
        // before that person entered the next cell.
        for (const Key& key:measured) free_.erase(key);
        for (auto it=free_.begin();it!=free_.end();) {
            if (stamp-it->second.last>lifetime) it=free_.erase(it);
            else ++it;
        }
        for (const Key& key:crossed) {
            auto found=free_.find(key);
            if (found==free_.end()) {
                if (free_.size()>=max_cells) continue;
                free_.emplace(key,Evidence{1,stamp,stamp});
            } else {
                found->second.frames=std::min(minimum_frames,found->second.frames+1);
                found->second.last=stamp;
            }
        }
    }

    std::size_t size() const { return free_.size(); }

private:
    Key keyFor(const Eigen::Vector3d& p) const {
        return {int(std::floor(p.x()/resolution)),int(std::floor(p.y()/resolution)),
                int(std::floor(p.z()/resolution))};
    }
    std::unordered_map<Key,Evidence,Hash> free_;
    double last_stamp_{0.};
};
}  // namespace astra
