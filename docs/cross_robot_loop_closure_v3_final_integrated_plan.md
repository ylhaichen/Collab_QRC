# Cross-Robot Loop Closure v3 Final Integrated Plan

## Purpose

Build the final integrated version of the current Cross-Robot Loop Closure system in one coherent implementation pass.

The final version must integrate three capabilities:

1. **Optimized GTSAM backend** for actual pose graph optimization instead of only `g2o_export_only`.
2. **Dynamic object filtering / long-term obstacle cleanup** so moving humans or other moving objects do not remain as permanent obstacles.
3. **Decentralized two-Jetson deployment** so each robot can run the team-SLAM pipeline onboard without a central laptop.

This plan must be implemented as one final integrated change set. Do not split it into small easy stages and stop after partial patches.

---

## Current System Status

The current v2 system has passed runtime validation in `g2o_export_only` mode.

Validated behavior:

```text
Cross-Robot Loop Closure v2:
  robust inter-robot loop selection        PASS
  no-GT runtime                            PASS
  overlap alignment                        PASS
  no-overlap false-positive rejection      PASS
  /merged_map safety gating                PASS
  team pose graph g2o export               PASS
  optimized GTSAM backend                  NOT AVAILABLE
```

Current valid claim:

```text
independent per-robot SC-PGO loop closure
+ robust inter-robot loop selection
+ valid team pose graph g2o export path
+ safety-gated discovered map alignment architecture
```

Current invalid claim:

```text
optimized centralized multi-robot PGO
true optimized inter-robot pose graph correction
distributed PGO
```

Reason:

```text
backend=g2o_export_only
optimization_success=false
host Python has no gtsam
```

---

## Reference Projects

### 1. Swarm-LIO2

Repository:

```text
https://github.com/hku-mars/Swarm-LIO2
```

Use as architecture reference for:

```text
decentralized LiDAR-inertial swarm architecture
peer communication
mutual state estimation
bandwidth-efficient swarm operation
onboard deployment pattern
```

Do not directly port Swarm-LIO2.

### 2. Dynamic-LIO

Repository:

```text
https://github.com/ZikangYuan/dynamic_lio
```

Use as concept reference for:

```text
dynamic scene LiDAR-inertial odometry
label consistency
dynamic point filtering
preventing dynamic objects from polluting long-term maps
```

Do not replace Fast-LIO with Dynamic-LIO.

---

## Implementation Rules

### Required

```text
Do the final integrated version at once.
Do not stop after only installing GTSAM.
Do not stop after only adding dynamic filtering.
Do not stop after only adding decentralized launch files.
Do not stop after only adding docs.
Do not stop after only adding one benchmark.
```

The final output must include all three:

```text
1. optimized backend or explicit fallback
2. dynamic object filtering / map cleanup
3. decentralized deployment support
```

### Forbidden

Do not add:

```text
DPGO
KISS-Matcher
TEASER++
Quatro
Nano-GICP
VLM
3D scene graph
online 3DGS
unrelated planner behavior
unrelated navigation changes
```

Do not break the validated v2 behavior:

```text
overlap must still align
no-overlap must still reject
GT must not be used at runtime
/merged_map must only open after robust alignment
```

---

# Part A — Optimized GTSAM Backend

## Problem

Current system exports:

```text
logs/team_pose_graph.g2o
logs/team_pose_graph_factors.json
```

but does not optimize because host Python lacks `gtsam`:

```text
ModuleNotFoundError: No module named 'gtsam'
```

The ROS1 SC-PGO Docker image may have C++ GTSAM, but that does not make `import gtsam` available to the host ROS2 Python environment.

## Required Backend Modes

Implement backend selection:

```text
team_pose_graph_backend:
  auto
  gtsam_python
  gtsam_cpp
  g2o_export_only
```

Behavior:

```text
auto:
  try gtsam_python
  else try gtsam_cpp service/node if built
  else fallback to g2o_export_only
```

## Preferred Route

Because Python `gtsam` is missing on the host, implement a **C++ ROS2 GTSAM optimizer node** if system C++ GTSAM is available.

Add either:

```text
src/collaborative_exploration/team_pose_graph_optimizer/
```

or extend the existing package if cleaner:

```text
src/collaborative_exploration/team_loop_closure/
```

Executable:

```text
team_pose_graph_optimizer_node
```

## Inputs

```text
/team_slam/keyframes
/team_slam/robust_loop_inliers
/team_slam/cross_robot_matches
/team_slam/team_pose_graph_factors
```

## Outputs

```text
/team_slam/robot_a/corrected_odom_global
/team_slam/robot_b/corrected_odom_global
/team_slam/team_map_to_robot_a_map
/team_slam/team_map_to_robot_b_map
/team_slam/pose_graph_metrics
```

Still export:

```text
logs/team_pose_graph.g2o
logs/team_pose_graph_factors.json
logs/team_pose_graph_metrics.json
```

## Graph Design

Variables:

```text
X(robot_a, keyframe_id)
X(robot_b, keyframe_id)
```

Factors:

```text
odom factor:
  X(robot, k) -> X(robot, k+1)

inter-robot loop factor:
  X(robot_a, i) -> X(robot_b, j)
  only from robust accepted inliers

optional self-loop factor:
  only if SC-PGO loop event/corrected path can be reliably converted
```

Never add:

```text
descriptor-only candidates
rejected matches
tentative matches
GT factors
```

## Optimization Rules

Use one of:

```text
GTSAM LevenbergMarquardtOptimizer
GTSAM ISAM2
```

Use `Pose3` if practical. `Pose2` is acceptable for current ground robot SE(2) validation only if clearly documented.

Noise model:

```text
odom_noise:
  x/y small-medium
  yaw medium
  z/roll/pitch larger if Pose3

inter_robot_loop_noise:
  based on ICP rmse, inlier_ratio, transform_spread
```

## Metrics

Publish and write:

```json
{
  "optimization_backend": "gtsam_cpp|gtsam_python|g2o_export_only",
  "optimization_success": true,
  "num_keyframes_robot_a": 0,
  "num_keyframes_robot_b": 0,
  "num_odom_factors": 0,
  "num_inter_robot_factors_inlier": 0,
  "num_inter_robot_factors_rejected": 0,
  "pose_graph_error_before": null,
  "pose_graph_error_after": null,
  "gt_used_runtime": false
}
```

## Dependency Handling

Add:

```text
scripts/setup/check_gtsam_backend.sh
```

It should check:

```bash
python3 -c "import gtsam"
ldconfig -p | grep gtsam
cmake find_package(GTSAM)
```

If Python `gtsam` is unavailable but C++ GTSAM is available, use C++ node.

If neither is available:

```text
do not crash
fallback to g2o_export_only
write dependency blocker into validation note
```

---

# Part B — Dynamic Object Filtering / Long-Term Obstacle Cleanup

## Problem

Moving humans or other moving objects can appear in LiDAR/map data and remain as obstacles for too long.

This affects:

```text
Fast-LIO map
merged_map
Nav2 costmaps
Scan Context / ICP keyframes
loop closure verification
```

## Required Approach

Use **Dynamic-LIO** as a concept reference only.

Implement a ROS2 dynamic filtering layer based on:

```text
temporal voxel consistency
observation lifetime
hit/miss history
motion consistency
short-term dynamic obstacle TTL
```

Do not use semantic human detection as the core requirement.

## Package / Nodes

Add package:

```text
src/collaborative_exploration/dynamic_scene_filter/
```

or integrate into:

```text
src/collaborative_exploration/team_loop_closure/
```

Nodes:

```text
dynamic_obstacle_filter_node.py
dynamic_voxel_decay_map_node.py
```

## Inputs

Per robot:

```text
/<ns>/cloud_registered_body
/<ns>/Odometry or /<ns>/corrected_odom
```

Optional:

```text
/<ns>/velodyne_points
/merged_map
```

## Outputs

```text
/<ns>/cloud_static
/<ns>/cloud_dynamic
/<ns>/dynamic_voxel_markers
/<ns>/dynamic_obstacle_mask
/team_slam/static_keyframe_clouds
/team_slam/dynamic_filter_metrics
```

## Voxel Memory

For each robot maintain a rolling voxel memory:

```text
voxel_id
last_seen_time
first_seen_time
observation_count
hit_count
miss_count
last_position_centroid
velocity_estimate
static_score
dynamic_score
```

A voxel is likely static if:

```text
observed repeatedly over time
consistent position
seen from multiple frames
not only present in short interval
```

A voxel is likely dynamic if:

```text
appears briefly
moves across adjacent voxels
inconsistent with previous static map
seen once then disappears
high short-term velocity
```

## Parameters

```yaml
dynamic_filter_enabled: true
dynamic_voxel_size: 0.25
dynamic_static_min_observations: 3
dynamic_static_min_lifetime_sec: 2.0
dynamic_decay_time_sec: 5.0
dynamic_obstacle_ttl_sec: 2.0
dynamic_max_static_velocity: 0.15
dynamic_min_dynamic_velocity: 0.35
dynamic_near_robot_ignore_radius: 0.4
dynamic_publish_debug_clouds: true
```

## Integration Points

Use filtered static clouds for:

```text
Scan Context keyframe descriptor
icp_2d registration verification
team pose graph keyframe cloud export
optional map merge input if safe
```

Safer initial integration:

```text
raw cloud -> Fast-LIO remains
static cloud -> team_loop_closure / map merge / long-term obstacle cleanup
```

Do not replace raw Fast-LIO input unless tested.

## Nav2 / Costmap Cleanup

For the obstacle persistence issue, also tune costmap behavior:

```yaml
obstacle_layer:
  observation_persistence: 0.0 or low value
  clearing: true
  raytrace_max_range: valid
  obstacle_max_range: valid

voxel_layer:
  enabled: true if available
  mark_threshold: reasonable
  unknown_threshold: reasonable
```

If global map keeps human traces, add a decay overlay rather than corrupting the SLAM map:

```text
static SLAM map
+ rolling dynamic obstacle layer with TTL
+ clearing rays remove stale dynamic obstacles
```

## Metrics

Add benchmark metrics:

```json
{
  "dynamic_points_filtered": 0,
  "static_points_kept": 0,
  "dynamic_filter_ratio": 0.0,
  "stale_obstacle_decay_time_sec": null,
  "dynamic_false_static_count": 0,
  "static_false_dynamic_count": 0
}
```

## Validation Scene

Add or modify:

```text
moving_human_proxy_scene.xml
```

If MuJoCo animated human is too expensive, add synthetic dynamic point injection:

```text
dynamic_obstacle_injector_node.py
```

Pass condition:

```text
moving object appears in local costmap temporarily
moving object does not remain in long-term map after TTL
static walls remain
Scan Context / ICP use static cloud and do not match dynamic trace
```

---

# Part C — Decentralized Two-Jetson Deployment

## Goal

Support:

```text
Jetson Nano on robot_a
Jetson Nano on robot_b
no central laptop required
```

## Architecture

Implement **replicated decentralized team SLAM**, not DPGO.

Each Jetson runs:

```text
per-robot Fast-LIO / odom adapter
local keyframe exporter
dynamic obstacle filter
cross-robot matcher
robust loop selector
team_pose_graph_node
relative_transform_manager
local map merge gate
```

Both robots exchange compact peer data:

```text
keyframe descriptor
compact static keyframe cloud
verified match summary
robust inlier set
pose graph factor summary
alignment status
```

## Communication Node

Add:

```text
team_slam_peer_node.py
```

or equivalent.

Topics:

```text
/team_slam/local/keyframes
/team_slam/local/keyframe_clouds
/team_slam/local/robust_loop_inliers
/team_slam/local/pose_graph_metrics

/team_slam/peer/keyframes
/team_slam/peer/keyframe_clouds
/team_slam/peer/robust_loop_inliers
/team_slam/peer/pose_graph_metrics
```

## Supported Communication Modes

```text
decentralized_mode:=true
robot_id:=robot_a|robot_b
peer_robot_id:=robot_b|robot_a
team_comm_mode:=dds|udp_json
```

Start with DDS. Add UDP JSON only if DDS over the Jetson network is unreliable.

## Bandwidth Limits

Jetson Nano is weak, so limit communication:

```yaml
peer_keyframe_rate_hz: 0.5
peer_cloud_voxel_size: 0.4
peer_cloud_max_points: 2000
peer_descriptor_only_until_candidate: true
send_cloud_only_on_candidate: true
compress_peer_json: true
```

Communication policy:

```text
Always exchange:
  descriptors
  poses
  metadata

Only exchange compact cloud when:
  descriptor candidate passes threshold
```

## Launch Files

Add:

```text
launch/decentralized_robot.launch.py
```

Args:

```text
robot_id:=robot_a
peer_robot_id:=robot_b
decentralized_mode:=true
use_dynamic_filter:=true
team_pose_graph_backend:=auto
team_comm_mode:=dds
livox_topic:=...
odom_topic:=...
```

Robot A example:

```bash
ros2 launch go2_gazebo_sim decentralized_robot.launch.py \
  robot_id:=robot_a \
  peer_robot_id:=robot_b \
  decentralized_mode:=true \
  use_dynamic_filter:=true \
  team_pose_graph_backend:=auto
```

Robot B example:

```bash
ros2 launch go2_gazebo_sim decentralized_robot.launch.py \
  robot_id:=robot_b \
  peer_robot_id:=robot_a \
  decentralized_mode:=true \
  use_dynamic_filter:=true \
  team_pose_graph_backend:=auto
```

## Config Files

Add:

```text
config/decentralized/robot_a.yaml
config/decentralized/robot_b.yaml
config/decentralized/team_network.yaml
config/dynamic_filter.yaml
```

Example fields:

```yaml
robot_id: robot_a
peer_robot_id: robot_b
ros_domain_id: 42
peer_ip: ...
local_ip: ...
keyframe_exchange_rate_hz: 0.5
max_peer_cloud_points: 2000
```

## Decentralized Consistency Rule

Both robots should independently reach consistent alignment:

```text
robot_a accepted T_a_b
robot_b accepted inverse T_b_a
```

Add check:

```text
alignment_symmetry_error_translation
alignment_symmetry_error_yaw
```

Pass condition:

```text
translation symmetry error < 0.5 m
yaw symmetry error < 5 deg
```

---

# Final Validation Requirements

## 1. Existing v2 Regression

Run:

```bash
START_BRIDGE=true bash scripts/bench/run_cross_loop_runtime_validation.sh
```

Must still pass:

```text
overlap_pass=true
no_overlap_pass=true
gt_used_runtime=false
```

## 2. GTSAM Optimized Backend Validation

Run with:

```text
team_pose_graph_backend:=auto
```

Expected if GTSAM available:

```text
optimization_backend=gtsam_cpp or gtsam_python
optimization_success=true
pose_graph_inter_robot_factors > 0
pose_graph_error_after <= pose_graph_error_before
```

If unavailable:

```text
optimization_backend=g2o_export_only
optimization_success=false
dependency_blocker recorded
```

## 3. Dynamic Object Validation

Run dynamic obstacle test.

Pass:

```text
dynamic object temporarily appears
dynamic object decays / clears after TTL
long-term map does not retain human path as permanent obstacle
static walls remain stable
dynamic_filter_metrics generated
```

## 4. Decentralized Simulation Validation

Run two separate processes simulating two Jetsons:

```text
ROS_DOMAIN_ID=42 robot_a decentralized launch
ROS_DOMAIN_ID=42 robot_b decentralized launch
```

Pass:

```text
both robots publish local keyframes
peer descriptors received
peer compact clouds received only when needed
both robots compute consistent robust inliers
both robots export or optimize team graph
alignment symmetry error below threshold
no central node required
```

## 5. Real Jetson Readiness Check

Add:

```text
scripts/deploy/check_jetson_readiness.sh
```

Check:

```text
CPU arch
RAM
ROS2 sourced
Livox driver available
DDS discovery works
peer ping works
topic bandwidth below threshold
team_slam nodes launch
```

---

# Required Deliverables

Generate or update:

```text
logs/cross_loop_closure_final_eval.json
logs/cross_loop_closure_final_eval.md
logs/team_pose_graph_metrics.json
logs/dynamic_filter_validation.json
logs/dynamic_filter_validation.md
logs/decentralized_validation.json
logs/decentralized_validation.md
logs/jetson_readiness_report.md
```

Add docs:

```text
docs/cross_robot_loop_closure_v3_final_architecture.md
docs/decentralized_jetson_deployment.md
docs/dynamic_object_filtering.md
```

---

# Final Acceptance Status

The task is complete only if one of these statuses is explicitly reported.

## Status A — Full Optimized Version Passed

```text
GTSAM optimized backend passed
dynamic object filtering passed
decentralized two-robot deployment simulation passed
Jetson readiness scripts added
```

Valid claim:

```text
centralized or replicated-decentralized multi-robot pose graph correction with robust inter-robot factors, dynamic object filtering, and decentralized onboard deployment support
```

## Status B — GTSAM Blocked, Export Version Passed

```text
g2o_export_only passed
dynamic object filtering passed
decentralized deployment support passed
GTSAM dependency blocker recorded
```

Valid claim:

```text
robust inter-robot loop selection with valid pose graph export, dynamic object filtering, and decentralized deployment support
```

## Status C — External Blocker

If blocked by:

```text
missing GTSAM C++ / Python dependency
Jetson unavailable
Docker permission
ROS2 DDS network
Livox driver unavailable
```

then stop and record the exact blocker. Do not fake success.

---

# Prompt to Coding Agent

```text
Build the final integrated version at once: optimized GTSAM backend if available, Dynamic-LIO-inspired dynamic object filtering, and Swarm-LIO2-inspired decentralized two-Jetson deployment.

Do not split this into small stages. Deliver one coherent final change set with validation and explicit fallback if GTSAM or Jetson runtime is unavailable.

Use Swarm-LIO2 only as architecture reference for decentralized LiDAR-inertial swarm deployment. Do not port it directly.

Use Dynamic-LIO only as concept reference for dynamic object filtering / label consistency. Do not replace Fast-LIO.

The final output must include:
1. optimized GTSAM backend or explicit g2o_export_only fallback with dependency blocker
2. dynamic object filtering / long-term obstacle cleanup
3. decentralized two-Jetson deployment support
4. validation logs and docs

Do not add DPGO, KISS-Matcher, TEASER++, Quatro, Nano-GICP, VLM, 3D scene graph, online 3DGS, unrelated planner behavior, or unrelated navigation changes.

Do not break existing validated v2 behavior:
overlap must still align; no-overlap must still reject; GT must not be used at runtime; /merged_map must only open after robust alignment.
```
