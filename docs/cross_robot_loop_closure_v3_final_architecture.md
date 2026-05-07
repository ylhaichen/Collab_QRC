# Cross-Robot Loop Closure v3 Final Architecture

## Claim Boundary

v3 keeps the validated v2 safety boundary:

- no runtime GT input in discovered mode
- robust inter-robot inlier selection gates alignment
- `/merged_map` opens only after final alignment status is `aligned`
- descriptor-only, rejected, and tentative matches never enter the team graph

The optimized PGO claim is allowed only when `optimization_backend` is
`gtsam_python` or `gtsam_cpp` and `optimization_success=true` in host runtime
validation. A repo-local ROS Humble C++ GTSAM package can now be used when
system `libgtsam-dev` is unavailable, but offline optimizer success alone is
not enough to claim runtime optimized multi-robot PGO.

## Data Flow

1. Each robot runs Fast-LIO / SC-PGO independently.
2. Optional `dynamic_scene_filter` publishes `/robot_*/cloud_static`.
3. `loop_keyframe_exporter_node` builds Scan Context descriptors from static
   cloud when enabled, otherwise from `/cloud_registered_body`.
4. `cross_robot_loop_matcher_node` performs Scan Context retrieval and
   self-contained `icp_2d` verification.
5. `robust_loop_selector_node` selects a PCM-style robust inlier set.
6. `team_pose_graph_node` exports `team_pose_graph_factors/v1` and publishes
   graph inputs on `/team_slam/team_pose_graph_factors`.
7. `team_pose_graph_optimizer_node` consumes those factors and runs C++ GTSAM
   `Pose2` optimization when available; otherwise it publishes an explicit
   `g2o_export_only` dependency blocker.
8. `relative_transform_manager_node` publishes alignment only after robust and
   graph gates pass.

## Backend Modes

- `auto`: Python GTSAM, then C++ GTSAM optimizer, then export-only fallback.
- `gtsam_python`: optimize in the Python node if `import gtsam` works.
- `gtsam_cpp`: C++ optimizer node using GTSAM `Pose2` factors generated from
  robust accepted inter-robot loop closures and consecutive odometry factors.
- `g2o_export_only`: write `team_pose_graph.g2o` and factors JSON without
  claiming optimization.

Use `scripts/setup/check_gtsam_backend.sh` before claiming optimized PGO.

## Current Host Status

Python `gtsam` is still unavailable. C++ GTSAM is available through the
repo-local extracted ROS Humble package under `.local_deps/gtsam_humble`, and
the offline optimizer path produces `optimization_backend=gtsam_cpp` with
`optimization_success=true` on the exported factor graph. Full Status A still
requires host runtime validation with:

```bash
START_BRIDGE=true TEAM_POSE_GRAPH_BACKEND=auto TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE=false \
  bash scripts/bench/run_cross_loop_runtime_validation.sh
```
