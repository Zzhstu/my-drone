# Pinned small-office scene assets

Source: https://github.com/macc-n/gazebo_worlds

Commit: `989303039221f5f432e96f8ba2fdfb400db38f11`

The upstream repository declares GPL-3.0; its `LICENSE` and README are retained.
`office_small.original.world` and all 14 mesh files are unmodified source copies.
`SOURCE.json` records paths, hashes, selected models and omissions.

`small_indoor_scene.py` generates a runtime copy: removes saved simulator state
and tabletop clutter, retains the selected 19 top-level models' collision
geometry and poses, makes them static, resolves mesh paths to local files,
uses plain SDF colours and the project's existing 1ms/1000Hz physics settings.
The approximately 10x10m room has no roof in the original source. It is not a
newly claimed dataset or a complete reproduction of the furnished upstream world.

Run from the repository root:

```bash
bash scripts/run_sh/small_indoor_exploration_demo.sh
```

Instructions: `AstraDrone_ros1_ws/src/SLAM/fastlio_bridge/SMALL_INDOOR_DEMO_CN.md`.
