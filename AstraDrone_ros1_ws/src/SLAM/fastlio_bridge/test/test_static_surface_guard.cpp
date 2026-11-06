#include <fastlio_bridge/static_surface_guard.hpp>
#include <cstdlib>

int main(int argc,char** argv) {
    if (argc!=2) return 2;
    const int scenario=std::atoi(argv[1]);
    std::vector<Eigen::Vector3d> points;
    for (int i=-6;i<=6;++i) for (int j=-6;j<=6;++j) {
        Eigen::Vector3d p(0.,.15*i,.15*j);
        if (p.norm()<.25 || p.norm()>.9) continue;
        if (scenario==2 && std::abs(p.y())>.2) continue; // narrow person
        if (scenario==3 && i!=0) continue;              // single line
        if (scenario==4) std::swap(p.x(),p.z());        // floor
        points.push_back(p);
    }
    Eigen::Vector3d query=Eigen::Vector3d::Zero();
    if (scenario==5) query.x()=.08;                    // nearby person
    if (scenario==6) points.resize(8);                 // sparse unknown
    if (scenario==7) {                                 // wall/floor junction
        for (int i=-5;i<=5;++i) for (int j=0;j<=5;++j)
            points.emplace_back(.15*j,.15*i,-.6);
    }
    // No alignment with map axes or map origin may be assumed.
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(.63,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Vector3d offset(17.2,-5.1,.8);
    for (auto& p:points) p=rotation*p+offset;
    query=rotation*query+offset;
    const auto r=astra::staticSurfaceEvidence(query,points,.04,.025,18,.7);
    const bool expected=scenario==1 || scenario==7;
    return r.supported==expected ? 0 : 1;
}
