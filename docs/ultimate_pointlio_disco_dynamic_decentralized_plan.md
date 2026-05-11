# Ultimate Implementation Plan: Dynamic-Aware Decentralized Multi-Robot Exploration

## Target Repository

```text
https://github.com/ylhaichen/Collab_QRC
```

Working branch requirement:

```text
feature/pointlio-disco-dynamic-decentralized
```

If this branch does not exist, create it from the latest stable working branch. Do not work directly on `main`.

Push policy:

```text
Push only to fork:
https://github.com/ylhaichen/Collab_QRC.git

Do not push to origin if origin points to HanshangZhu/Collab_QRC or any upstream repository.
```

---

## Ultimate Goal

Implement a complete dynamic-aware decentralized multi-robot exploration system for two Livox-equipped legged robots, Go2 and Go2W.

The final system should use:

```text
Primary local SLAM:
  Point-LIO for real robot robustness

Cross-robot loop closure:
  DiSCo-SLAM-style team_loop_closure
  Scan Context + KISS-Matcher / ICP
  PCM / GNC robust inlier selection

Dynamic map:
  online temporal / Dynamic-LIO-style filtering
  ERASOR or Removert asynchronous cleanup

Decentralization:
  descriptor-first peer exchange
  compact cloud on demand

Map merge:
  safety-gated only after robust evidence
```

Do not implement a toy prototype. Implement the final integrated architecture as one coherent change set. Internal dependency ordering is allowed, but the deliverable must be a complete end-to-end system, not a partial stage.

---

## Core System Philosophy

Do not make the system depend on any single upstream SLAM package for every capability.

The system must be modular:

```text
Local SLAM backend:
  Point-LIO primary
  Fast-LIO / SC-PGO fallback
  optional Swarm-LIO2 experimental backend

Cross-robot alignment:
  independent DiSCo-SLAM-style team_loop_closure

Dynamic object handling:
  online static/dynamic filtering
  asynchronous static map cleanup

Decentralization:
  descriptor-first peer exchange
  compact cloud on demand

Map merge:
  never automatic
  enabled only after robust alignment evidence
```

The cross-robot alignment authority must be the robust loop closure system, not ground truth, not manually configured initial pose, and not a single weak match.

---

## Non-Negotiable Safety Rules

The following rules are mandatory:

```text
1. Do not use ground truth for runtime alignment.
2. Do not use manually hardcoded robot_a_to_robot_b transform.
3. Do not open /merged_map from descriptor-only matches.
4. Do not open /merged_map from a single weak ICP match.
5. Do not open /merged_map from an unverified local SLAM mutual state.
6. Do not exchange raw dense point clouds continuously between robots.
7. Do not allow moving humans or dynamic objects to become permanent map obstacles.
8. Do not remove the validated Fast-LIO / SC-PGO fallback until Point-LIO primary validation passes.
9. Do not fake Status A. If a backend cannot run, record exact blocker.
10. Do not push to upstream origin.
```

---

## Architecture Overview

```text
Robot A                                                   Robot B
──────────────────────────────────────────────────────────────────────────────

Livox LiDAR + IMU                                        Livox LiDAR + IMU
   │                                                        │
   ▼                                                        ▼
Point-LIO local SLAM                                     Point-LIO local SLAM
   │                                                        │
   ├── /robot_a/Odometry                                   ├── /robot_b/Odometry
   ├── /robot_a/corrected_odom                             ├── /robot_b/corrected_odom
   ├── /robot_a/cloud_registered_body                      ├── /robot_b/cloud_registered_body
   ├── /robot_a/cloud_static                               ├── /robot_b/cloud_static
   └── /robot_a/cloud_dynamic                              └── /robot_b/cloud_dynamic
   │                                                        │
   └────────────── descriptor-first peer exchange ─────────┘
                            │
                            ▼
                  team_loop_closure
                            │
          ┌─────────────────┼─────────────────┐
          ▼                 ▼                 ▼
    Scan Context      KISS-Matcher / ICP   Robust selector
    retrieval         verification          PCM / GNC
          └─────────────────┼─────────────────┘
                            ▼
                    team_pose_graph
                    GTSAM / g2o export
                            │
                            ▼
                  relative_transform_manager
                            │
                            ▼
              /merged_map safety-gated output
                            │
                            ▼
                Nav2 / exploration allocator

Asynchronous map cleanup:

Point-LIO local maps / keyframes / poses
          │
          ▼
ERASOR or Removert cleanup backend
          │
          ├── cleaned_static_map
          ├── removed_dynamic_points
          └── map cleanup metrics
          │
          ▼
ROS2 cleaned map publisher
```

---

## Required Backend Modes

Add or formalize these modes:

```yaml
local_slam_backend:
  point_lio
  fast_lio_scpgo
  swarm_lio2_experimental

cross_loop_backend:
  disco_style_team_loop_closure

registration_backend:
  kiss_matcher
  icp_2d
  gicp_optional

robust_selection_backend:
  pcm
  gnc
  greedy_consistency_fallback

dynamic_filter_backend:
  temporal_voxel
  dynamic_lio_style
  dynamic_lio_wrapper_optional

static_map_cleanup_backend:
  none
  erasor
  removert
  temporal_voxel_fallback

team_comm_mode:
  dds
  udp_json
  descriptor_only
```

Default safe mode:

```yaml
local_slam_backend: fast_lio_scpgo
cross_loop_backend: disco_style_team_loop_closure
registration_backend: icp_2d
robust_selection_backend: greedy_consistency_fallback
dynamic_filter_backend: temporal_voxel
static_map_cleanup_backend: none
team_comm_mode: dds
```

Target production mode:

```yaml
local_slam_backend: point_lio
cross_loop_backend: disco_style_team_loop_closure
registration_backend: kiss_matcher
robust_selection_backend: pcm
dynamic_filter_backend: dynamic_lio_style
static_map_cleanup_backend: erasor
team_comm_mode: descriptor_only
```

---

# Part A — Primary Local SLAM: Point-LIO

## Objective

Use Point-LIO as the primary local SLAM backend for each robot because the real robots use Livox LiDAR and legged platforms have vibration / aggressive motion characteristics.

Point-LIO should provide stable per-robot odometry and local map data.

## Required Implementation

Add a Point-LIO backend integration layer.

Expected source:

```text
https://github.com/hku-mars/Point-LIO
```

Do not remove Fast-LIO / SC-PGO. Keep it as fallback.

Add:

```text
docker/point_lio/
scripts/setup/check_point_lio.sh
scripts/manual/run_point_lio_docker_build_and_test.sh
scripts/bench/run_point_lio_shadow_validation.sh
scripts/bench/run_point_lio_primary_validation.sh
```

If using native ROS1 on robot / Jetson, add:

```text
scripts/deploy/check_point_lio_real_robot.sh
```

## Required ROS2 Adapter Contract

Point-LIO output must be adapted to the same contract previously used by Fast-LIO:

```text
/<ns>/Odometry
/<ns>/corrected_odom
/<ns>/odom/nav
/<ns>/cloud_registered_body
/<ns>/cloud_static
/<ns>/cloud_dynamic
/tf
```

Add adapter:

```text
point_lio_ros2_adapter_node
```

The adapter must support:

```text
shadow mode:
  Point-LIO publishes /<ns>/point_lio/*
  Fast-LIO remains production

primary mode:
  Point-LIO owns /<ns>/Odometry, /<ns>/corrected_odom, /<ns>/odom/nav
```

## Required Validation

Shadow validation must pass first:

```text
Point-LIO receives Livox LiDAR + IMU input.
Point-LIO native odometry nonzero-rate.
ROS2 /<ns>/point_lio/Odometry nonzero-rate.
Fast-LIO production path remains unaffected.
```

Primary validation:

```text
/robot_a/Odometry from Point-LIO adapter
/robot_b/Odometry from Point-LIO adapter
/robot_a/corrected_odom from Point-LIO adapter
/robot_b/corrected_odom from Point-LIO adapter
/robot_a/odom/nav valid
/robot_b/odom/nav valid
Nav2 odom/tf valid
team_loop_closure receives keyframes
GT not used
```

Required logs:

```text
logs/point_lio_shadow_validation.json
logs/point_lio_shadow_validation.md
logs/point_lio_primary_validation.json
logs/point_lio_primary_validation.md
```

---

# Part B — Cross-Robot Loop Closure: DiSCo-SLAM-Style team_loop_closure

## Objective

Implement DiSCo-SLAM-style cross-robot loop closure as the authoritative cross-robot alignment module.

The system must handle unknown initial relative pose.

## Required Pipeline

```text
static keyframe cloud
  -> Scan Context descriptor
  -> descriptor exchange
  -> candidate retrieval
  -> KISS-Matcher / ICP verification
  -> PCM / GNC robust selection
  -> team pose graph factor generation
  -> discovered T_robot_a_map_robot_b_map
  -> safety-gated /merged_map
```

## Required Inputs

```text
/<ns>/Odometry
/<ns>/corrected_odom
/<ns>/cloud_static
fallback: /<ns>/cloud_registered_body
```

## Required Outputs

```text
/team_slam/keyframes
/team_slam/keyframe_clouds
/team_slam/cross_robot_candidates
/team_slam/cross_robot_matches
/team_slam/robust_loop_inliers
/team_slam/relative_transform
/team_slam/alignment_status
/team_slam/pose_graph_metrics
```

## Scan Context

Keep or improve the existing Scan Context implementation.

Requirements:

```text
descriptor non-empty
ring_key non-empty
sector_key non-empty
yaw shift estimated
same-robot matching disabled for cross-robot loop closure
```

## KISS-Matcher / ICP Registration

Add KISS-Matcher as preferred robust registration backend if buildable.

Expected source:

```text
https://github.com/MIT-SPARK/KISS-Matcher
```

Backend modes:

```yaml
registration_backend:
  kiss_matcher
  icp_2d
  gicp_optional
```

Rules:

```text
If KISS-Matcher builds and runs:
  use kiss_matcher for coarse registration
  optionally refine with ICP/GICP

If KISS-Matcher is blocked:
  use icp_2d fallback
  record blocker
```

A descriptor match must never be accepted without geometric verification.

## PCM / GNC Robust Selection

Implement or strengthen pairwise consistency logic.

Backend modes:

```yaml
robust_selection_backend:
  pcm
  gnc
  greedy_consistency_fallback
```

Required behavior:

```text
verified matches -> consistency graph
nodes = verified inter-robot matches
edges = pairwise consistency
select max consistent inlier set
reject ambiguous / inconsistent candidates
```

Acceptance requires:

```text
robust_inlier_set_size >= threshold
robust_inlier_ratio_eligible >= threshold
transform_spread_translation <= threshold
transform_spread_yaw_deg <= threshold
median_rmse <= threshold
gt_used_runtime=false
```

## Safety Gate

/merged_map may open only if:

```text
robust_loop_selector accepted
team pose graph has accepted inter-robot factors
relative transform finite
no-overlap rejection passes
GT not used
```

If a local SLAM backend provides mutual state, it may be used as optional consistency check, not as a hard requirement.

## Required Validation

Run:

```text
overlap scene:
  alignment accepted
  /merged_map opens after robust evidence

no-overlap scene:
  alignment rejected
  /merged_map stays closed

symmetric / repeated scene:
  false positives rejected
```

Required logs:

```text
logs/cross_loop_closure_final_eval.json
logs/cross_loop_closure_final_eval.md
logs/robust_loop_selection_eval.json
logs/robust_loop_selection_eval.md
```

---

# Part C — Dynamic Map Handling

## Objective

Moving humans and moving objects must not contaminate the long-term static map or cross-robot loop closure.

Use two layers:

```text
online dynamic filtering:
  temporal / Dynamic-LIO-style filtering

asynchronous static map cleanup:
  ERASOR or Removert
```

---

## C1 — Online Dynamic Filtering

Required outputs:

```text
/<ns>/cloud_static
/<ns>/cloud_dynamic
/<ns>/dynamic_filter_metrics
```

Possible backends:

```yaml
dynamic_filter_backend:
  temporal_voxel
  dynamic_lio_style
  dynamic_lio_wrapper_optional
```

Rules:

```text
cloud_static is used by:
  Point-LIO map export if applicable
  team_loop_closure Scan Context
  KISS-Matcher / ICP registration
  team pose graph keyframe cloud

cloud_dynamic is used by:
  local costmap dynamic obstacle layer
  TTL-based clearing
```

Dynamic object behavior:

```text
moving object appears as short-term obstacle
moving object does not remain permanent in static map
static walls remain stable
```

## C2 — ERASOR / Removert Async Cleanup

Backend modes:

```yaml
static_map_cleanup_backend:
  none
  erasor
  removert
  temporal_voxel_fallback
```

Expected sources:

```text
ERASOR:
  https://github.com/LimHyungTae/ERASOR

Removert:
  https://github.com/gisbi-kim/removert
```

Do not run ERASOR / Removert inside the real-time odometry loop.

Required export format:

```text
pcds/
dense_global_map.pcd
poses_lidar2body.csv
initial_naive_map.pcd
```

Required outputs:

```text
/team_slam/cleaned_static_map
/team_slam/removed_dynamic_points
/team_slam/map_cleanup_metrics
```

Required validation:

```text
naive map contains dynamic trace
cleanup removes dynamic trace
static walls preserved
cleaned map published to ROS2
robot control loop not blocked
```

Logs:

```text
logs/dynamic_filter_validation.json
logs/dynamic_filter_validation.md
logs/map_cleanup_validation.json
logs/map_cleanup_validation.md
```

---

# Part D — Decentralized Communication

## Objective

Implement low-bandwidth decentralized peer exchange suitable for Jetson Nano.

## Required Policy

Always exchange:

```text
robot_id
timestamp
keyframe_id
local pose
Scan Context descriptor
ring_key
sector_key
health metrics
alignment status
```

Only on demand exchange:

```text
compact static keyframe cloud
verified match summary
robust inlier set
pose graph factor summary
```

Never continuously exchange:

```text
raw LiDAR
dense map
full costmap
all point clouds
```

## Required Topics

Local:

```text
/team_slam/local/keyframes
/team_slam/local/descriptors
/team_slam/local/status
/team_slam/local/pose_graph_metrics
```

Peer:

```text
/team_slam/peer/keyframes
/team_slam/peer/descriptors
/team_slam/peer/status
/team_slam/peer/pose_graph_metrics
```

Cloud-on-demand:

```text
/team_slam/cloud_request
/team_slam/cloud_response
```

## Backend Modes

```yaml
team_comm_mode:
  dds
  udp_json
  descriptor_only
```

Default for real deployment:

```yaml
team_comm_mode: descriptor_only
```

Bandwidth limits:

```yaml
peer_descriptor_rate_hz: 0.5
peer_cloud_max_points: 2000
peer_cloud_voxel_size: 0.4
send_cloud_only_on_candidate: true
```

Required metrics:

```text
packets_sent
packets_received
bytes_sent
bytes_received
cloud_requests
cloud_responses
average_latency_ms
dropped_messages
```

Logs:

```text
logs/decentralized_comm_validation.json
logs/decentralized_comm_validation.md
```

---

# Part E — Exploration and Task Allocation

## Objective

Collaborative exploration should consider both coverage and map quality.

## Required Goal Types

```text
frontier_goal
loop_closure_goal
rendezvous_goal
dynamic_cleanup_goal
communication_recovery_goal
```

## Utility Function

Implement or update candidate utility:

```text
utility =
  coverage_gain
  + loop_closure_gain
  + map_cleanup_gain
  + communication_value
  - travel_cost
  - mobility_risk
  - duplicate_assignment_penalty
```

## Required Behavior

```text
If maps are not aligned:
  do not send robots to peer coordinates
  exchange descriptors
  seek high-probability loop closure opportunities only if transform is known or candidate is local

If robust alignment exists:
  allow inter-robot rendezvous and shared-map exploration

If dynamic contamination is high:
  prefer cleanup or revisit goal

If communication weak:
  prefer rendezvous / relay goal
```

Logs:

```text
logs/exploration_allocator_eval.json
logs/exploration_allocator_eval.md
```

---

# Part F — Validation Matrix

## F1 — Local SLAM Validation

Backends:

```text
fast_lio_scpgo
point_lio
```

Scenes:

```text
overlap
no-overlap
dynamic object
corridor / symmetric
```

Metrics:

```text
odom rate
cloud rate
Nav2 odom/tf validity
CPU/RAM
keyframe count
drift if GT eval available
```

## F2 — Cross-Robot Loop Closure Validation

Must test:

```text
overlap accepted
no-overlap rejected
symmetric false positives rejected
descriptor-only does not merge
single weak match does not merge
```

## F3 — Dynamic Map Validation

Must test:

```text
moving object appears
moving object clears
static wall remains
cloud_static excludes dynamic trace
cloud_dynamic includes moving object
cleanup removes residual map trace
```

## F4 — Decentralized Communication Validation

Must test:

```text
descriptor exchange
cloud-on-demand
bandwidth limit respected
no raw LiDAR continuous exchange
peer loss / reconnect
```

## F5 — Real Robot Validation

Check:

```text
Livox topics
IMU topics
Go2 / Go2W topics
Jetson CPU/RAM
network reachability
ROS1 / ROS2 bridge if used
Nav2 odom/tf
team_loop_closure keyframes
no GT runtime
```

---

# Part G — Required Logs

Generate or update:

```text
logs/local_slam_validation.json
logs/local_slam_validation.md
logs/point_lio_validation.json
logs/point_lio_validation.md
logs/cross_loop_closure_final_eval.json
logs/cross_loop_closure_final_eval.md
logs/robust_loop_selection_eval.json
logs/robust_loop_selection_eval.md
logs/dynamic_filter_validation.json
logs/dynamic_filter_validation.md
logs/map_cleanup_validation.json
logs/map_cleanup_validation.md
logs/decentralized_comm_validation.json
logs/decentralized_comm_validation.md
logs/exploration_allocator_eval.json
logs/exploration_allocator_eval.md
logs/final_system_validation.json
logs/final_system_validation.md
```

---

# Part H — Required Docs

Add or update:

```text
docs/final_system_architecture.md
docs/local_slam_backend_point_lio.md
docs/disco_style_cross_robot_loop_closure.md
docs/dynamic_map_filtering_and_cleanup.md
docs/decentralized_descriptor_exchange.md
docs/safety_gated_map_merge.md
docs/real_robot_deployment_go2_go2w.md
```

---

# Part I — Final Status Labels

Report exactly one:

## Status A — Full System Passed

```text
Point-LIO primary local SLAM passed.
DiSCo-SLAM-style cross-robot loop closure passed.
Dynamic filtering passed.
ERASOR/Removert cleanup passed or justified fallback passed.
Decentralized communication passed.
Safety-gated map merge passed.
Real robot validation passed.
Fast-LIO can be demoted to fallback.
```

## Status B — Simulation Passed, Real Blocked

```text
Simulation full system passed.
Real robot blocked by hardware / network / driver availability.
Fast-LIO remains fallback or production on real robot.
```

## Status C — Local SLAM Passed, Full Multi-Robot Blocked

```text
Point-LIO or selected local SLAM passed.
Cross-robot loop closure or decentralized communication blocked.
Fast-LIO remains production.
```

## Status D — External Blocker

```text
Point-LIO unavailable
Docker / ROS1 / ROS2 unavailable
Livox / IMU unavailable
Jetson unavailable
network unavailable
KISS-Matcher / ERASOR / Removert build blocked
```

Do not fake Status A.

---

# Part J — Coding Agent Prompt

```text
Implement the final redesigned system for Collab_QRC.

Do not continue the previous Swarm-LIO2-mutual-transform hard-gate approach.

The new target architecture is:

Primary local SLAM:
  Point-LIO for real robot robustness
  Fast-LIO / SC-PGO retained as fallback

Cross-robot loop closure:
  DiSCo-SLAM-style team_loop_closure
  Scan Context descriptor exchange
  KISS-Matcher / ICP geometric verification
  PCM / GNC robust inlier selection

Dynamic map:
  online temporal / Dynamic-LIO-style filtering
  ERASOR or Removert asynchronous cleanup

Decentralization:
  descriptor-first peer exchange
  compact cloud on demand

Map merge:
  safety-gated only after robust evidence

Do not break the current validated baseline.
Do not remove Fast-LIO until the new system passes validation.
Do not use GT for runtime alignment.
Do not open /merged_map without robust evidence.
Do not continuously exchange raw LiDAR or dense maps.

Work in one coherent long-running implementation task.
Do not stop after only adding scaffolding or docs.
If a backend is blocked, record exact blocker and continue with the remaining feasible system components.

Expected final result:
A decentralized dynamic-aware collaborative exploration system for two Livox-equipped legged robots under unknown initial relative pose.

Commit and push only to the user's fork.
Do not push origin.
```
