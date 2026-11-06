#include <fastlio_bridge/observed_free_motion.hpp>
#include <cassert>
#include <iostream>

int main() {
    astra::ObservedFreeMotion evidence;
    const Eigen::Vector3d origin(0,0,1);
    std::vector<Eigen::Vector3d> wall;
    for (int y=-8;y<=8;++y) for (int z=-4;z<=4;++z)
        wall.emplace_back(4.13,y*.15,1+z*.15);
    const Eigen::Vector3d candidate(2.125,.125,1.125);
    assert(!evidence.previouslyFree(candidate,1)); // unknown is not free
    evidence.integrate(origin,wall,1);
    assert(!evidence.previouslyFree(candidate,1.1)); // one scan cannot vote twice
    evidence.integrate(origin,wall,1.1);
    evidence.integrate(origin,wall,1.2);
    assert(evidence.previouslyFree(candidate,1.3));
    for (const auto& p:wall) assert(!evidence.previouslyFree(p,1.3));
    for (const auto& p:wall) {
        Eigen::Vector3d boundary=p;boundary.x()-=.20;
        assert(!evidence.previouslyFree(boundary,1.3));
    }
    auto person=wall;person.push_back(candidate);
    evidence.integrate(origin,person,1.3);
    assert(!evidence.previouslyFree(candidate,1.4)); // measured occupancy revokes
    evidence.integrate(origin,wall,1.4);
    evidence.integrate(origin,wall,1.6);
    assert(evidence.previouslyFree(candidate,1.7)); // actual re-observation restores
    assert(!evidence.previouslyFree(candidate,40)); // evidence is bounded in time
    astra::ObservedFreeMotion duplicate;
    duplicate.integrate(origin,wall,1);
    duplicate.integrate(origin,wall,1);
    assert(!duplicate.previouslyFree(candidate,1.2));
    astra::ObservedFreeMotion bounded;bounded.max_cells=8;
    bounded.integrate(origin,wall,1);
    assert(bounded.size()<=8);
    // Unknown moving object: earlier rays behind its changing position can
    // establish evidence for its next position, without freezing it static.
    astra::ObservedFreeMotion moving;
    bool detected=false;
    for (int frame=0;frame<50;++frame) {
        const double t=1+frame*.1;
        const Eigen::Vector3d position(2.125,-1.125+frame*.045,1.125);
        if (moving.previouslyFree(position,t)) detected=true;
        auto scene=wall;scene.push_back(position);
        moving.integrate(origin,scene,t);
    }
    assert(detected);
    std::cout<<"observed-free geometry, temporal, endpoint, capacity and moving-object checks passed\n";
}
