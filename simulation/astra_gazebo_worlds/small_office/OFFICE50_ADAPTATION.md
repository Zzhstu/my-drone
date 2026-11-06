# 50.4m² office demo adaptation

Run `bash scripts/run_sh/office50_dual_map_demo.sh` from the project root.
This generates a separate world per run under `office50_dual_map_runs`.

The 7.2×7.0m room, closed walls/ceiling and furniture placement are a new local
layout, not a claim that upstream `office_small.world` has this floor area.
Ten furniture instances retain the original GitHub collision/visual meshes and
scale; two additional bookcases use box collision/visual geometry. Furniture
types: two desks, two chairs, four cabinets, one sofa, one coffee table and two
bookcases. Roof rendering is transparent for Gazebo inspection; its collision
geometry remains present for the ODE multi-ray LiDAR.

Source: [macc-n/gazebo_worlds](https://github.com/macc-n/gazebo_worlds), pinned
commit `989303039221f5f432e96f8ba2fdfb400db38f11`. The existing `LICENSE`, upstream
README, original world, assets and `SOURCE.json` remain intact. The generator
verifies original/mesh hashes. Adaptation source:
`AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/scripts/office50_scene.py`.

One person (Gazebo model `1`) moves along x=1.1m, y=-0.2..1.2m for 60 seconds of
simulation time after the UAV body first rises above 0.9m, then stops in place.
It uses the original project's standing mesh translated/turned by one pose
owner, not skeletal gait animation. Automatic motion and optional keyboard
commands use the same guarded actor controller. World geometry/truth is used
only by the simulator, actor controller and offline evaluator, not the UAV's
black-box exploration/SLAM input.

Scene-local footprint bounds are conservative mesh AABBs, computed from actual
COLLADA node matrices and units, for placement/actor guard and floor exclusion.
They are not an exact substitute for furniture mesh surfaces when evaluating
point accuracy. Room wall/floor reference-support metrics do not establish full
3D completeness, ghost labels or collision-free flight.

The existing dual-map exploration/risk/control parameters are inherited. The
first central-actor layout caused a blocked return and was safely stopped;
layout revision 2 leaves a west-side alternative path. Both trial evidence and
the DUFO same-frame classification/export fix are documented in
`evaluation/office50_20261030`; do not report the interrupted trial as success.
