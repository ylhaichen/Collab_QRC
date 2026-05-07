# Cross-Robot Loop Closure Implementation Plan

## Project Context

This plan implements the LiDAR/odometry-based part of the Collab_QRC research project:

```text
Fast-LIO / SC-PGO
  -> pose graph health
  -> loop-closure candidates
  -> role-aware CFPA2 allocation
```

Current scope:

```text
independent per-robot loop closure
+ unknown-initial-pose inter-robot alignment
+ loop-closure-aware rendezvous assistance
```

Out of scope for this stage:

```text
VLM
3D scene graph
online 3DGS
ground-truth runtime alignment
true multi-robot pose graph correction before team_pose_graph_node exists
```

The current stage should **not** claim true cross-robot pose graph correction. That claim is only valid after a central or distributed `team_pose_graph_node` jointly optimizes both robots using inter-robot loop factors.

---

## Definition of Done

The stage is complete when the following launch works:

```bash
./scripts/launch/nav_test_demo3_mixed.sh \
  loop_closure:=true \
  loop_closure_backend:=ros1_bridge \
  relative_pose_source:=discovered \
  inter_robot_loop_closure:=true \
  team_alignment_min_matches:=2 \
  mujoco_cameras:=false
```

Expected runtime behavior:

```text
bootstrap_from_gt=false
no runtime /odom/ground_truth alignment
no /merged_map before verified alignment
real Scan Context descriptors published
cross-robot descriptor candidates found
bad candidates rejected by registration
alignment_status becomes aligned only after >=2 consistent matches
/merged_map enabled only after aligned
inter_robot_rendezvous candidates enabled only after aligned
```

---

## Step 0 — Audit and Freeze No-GT Runtime Mode

### Task

Verify that `relative_pose_source:=none|discovered` never uses ground truth for runtime alignment.

Audit:

```text
src/go2w/go2_gazebo_sim/launch/nav_test_mujoco_fastlio_mixed.launch.py
scripts/runtime/fast_lio_tf_adapter.py
scripts/runtime/bootstrap_map_merge_poses.py
src/vendor/multirobot_map_merge/
src/collaborative_exploration/team_loop_closure/
src/collaborative_exploration/reconstruction_awareness/
```

### Reference Project

**DiSCo-SLAM**  
Repo: https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM

### Main Concept

Distributed multi-robot LiDAR SLAM should not assume known relative initial poses. Robots exchange compact LiDAR descriptors and discover inter-robot loop closures from overlapping observations.

### Why Helpful

This matches the required behavior:

```text
Before alignment:
  robot_a/map and robot_b/map are independent
  no merged_map
  no inter-robot goal routing

After verified alignment:
  publish discovered T_robot_a_map_robot_b_map
  enable map merge
  enable inter_robot_rendezvous candidates
```

### Deliverable

Add or verify launch logs:

```text
relative_pose_source=none       -> bootstrap_from_gt=false, map_merge=false
relative_pose_source=discovered -> bootstrap_from_gt=false, map_merge gated by alignment_status
relative_pose_source=gt         -> legacy sim only
```

### Acceptance Test

```bash
grep -R "/odom/ground_truth\|bootstrap_from_gt\|bootstrap_map_merge" src scripts
```

For `relative_pose_source:=none|discovered`, no runtime alignment path should depend on GT.

---

## Step 1 — Replace Placeholder Descriptor with Real Scan Context

### Task

Upgrade:

```text
src/collaborative_exploration/team_loop_closure/team_loop_closure/loop_keyframe_exporter_node.py
src/collaborative_exploration/team_loop_closure/team_loop_closure/common.py
```

Each robot should publish real Scan Context keyframes:

```text
/team_slam/keyframes
```

Each keyframe should include:

```json
{
  "robot_id": "robot_a",
  "keyframe_id": 42,
  "stamp": 123.456,
  "local_pose": "Pose3 in robot_a/map",
  "descriptor_type": "scan_context",
  "scan_context": "...",
  "ring_key": "...",
  "sector_key": "...",
  "compact_cloud_topic": "/team_slam/keyframe_clouds/robot_a/42"
}
```

### Reference Project

**scancontext_ros2**  
Repo: https://github.com/aserbremen/scancontext_ros2

### Main Concept

Scan Context is a global LiDAR descriptor. Typical retrieval uses:

```text
Scan Context descriptor
  -> ring-key KD-tree candidate search
  -> full descriptor distance
  -> yaw alignment / sector shift
```

### Why Helpful

The current placeholder descriptor is not enough for real cross-robot place recognition. Real Scan Context gives:

```text
compact descriptor exchange
rotation-aware matching
candidate yaw prior
cross-robot place recognition before map alignment
```

### Params to Add

```yaml
scan_context_num_rings: 20
scan_context_num_sectors: 60
scan_context_max_radius: 80.0
keyframe_min_translation: 1.0
keyframe_min_yaw_deg: 10.0
keyframe_min_time_sec: 1.0
```

### Acceptance Test

```bash
ros2 topic echo /team_slam/keyframes
```

Expected:

```text
robot_a keyframes increasing
robot_b keyframes increasing
descriptor not empty
ring_key not empty
compact_cloud available
```

---

## Step 2 — Implement Cross-Robot Descriptor Retrieval

### Task

Upgrade:

```text
src/collaborative_exploration/team_loop_closure/team_loop_closure/cross_robot_loop_matcher_node.py
```

Only compare different robots:

```python
if query.robot_id == candidate.robot_id:
    continue
```

Pipeline:

```text
incoming keyframe
  -> query other robot descriptor database
  -> ring_key KD-tree top-K
  -> full Scan Context distance
  -> yaw shift estimate
  -> candidate_match
```

### Reference Project

**DiSCo-SLAM**  
Repo: https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM

### Main Concept

DiSCo-SLAM uses Scan Context as a lightweight descriptor for multi-robot LiDAR SLAM and data-efficient observation exchange.

### Why Helpful

This converts the system from planner-level rendezvous heuristics into real cross-robot place recognition.

### Deliverable

Publish:

```text
/team_slam/cross_robot_candidates
```

Candidate schema:

```json
{
  "query_robot": "robot_a",
  "query_keyframe": 42,
  "match_robot": "robot_b",
  "match_keyframe": 17,
  "descriptor_distance": 0.13,
  "yaw_shift_deg": -84.0,
  "stage": "descriptor_candidate"
}
```

### Acceptance Test

In a scene with shared overlap:

```text
robot_a visits corridor
robot_b later visits same corridor
```

Expected:

```text
cross_robot_candidates count > 0
same-robot candidates = 0
```

---

## Step 3 — Add Geometric Verification

### Task

Do not accept descriptor-only matches. Add point cloud registration:

```text
Scan Context candidate
  -> yaw prior
  -> coarse registration
  -> ICP / GICP refinement
  -> fitness / inlier ratio / residual check
  -> verified or rejected
```

Create or modify:

```text
src/collaborative_exploration/team_loop_closure/team_loop_closure/registration_backend.py
src/collaborative_exploration/team_loop_closure/team_loop_closure/cross_robot_loop_matcher_node.py
```

### Reference Project A

**FAST-LIO-SAM-SC-QN**  
Repo: https://github.com/engcang/FAST-LIO-SAM-SC-QN

### Main Concept

Combines:

```text
FAST-LIO2
+ ScanContext loop candidate detection
+ Quatro coarse registration
+ Nano-GICP refinement
+ GTSAM / iSAM2 pose graph optimization
```

### Why Helpful

This is close to the current stack because the project already uses Fast-LIO and SC-PGO-style loop closure.

### Reference Project B

**KISS-Matcher**  
Repo: https://github.com/MIT-SPARK/KISS-Matcher

### Main Concept

Fast, robust, scalable point cloud registration with ROS2 SLAM examples.

### Why Helpful

Good candidate for coarse registration before ICP/GICP, especially because it is ROS2-oriented.

### Reference Project C

**TEASER++**  
Repo: https://github.com/MIT-SPARK/TEASER-plusplus

### Main Concept

Certifiably robust 3D point cloud registration under high outlier rates.

### Why Helpful

Useful fallback if KISS-Matcher integration is too heavy or if repeated geometry creates many outlier correspondences.

### Params to Add

```yaml
registration_backend: "kiss_matcher"   # options: kiss_matcher, teaser, gicp_only
registration_voxel_size: 0.25
registration_max_corr_dist: 1.5
registration_min_inlier_ratio: 0.25
registration_max_fitness: 0.8
registration_max_rmse: 0.5
```

### Deliverable

Publish:

```text
/team_slam/cross_robot_matches
```

Verified match schema:

```json
{
  "accepted": true,
  "query_robot": "robot_a",
  "query_keyframe": 42,
  "match_robot": "robot_b",
  "match_keyframe": 17,
  "T_query_map_match_map": [ ... ],
  "descriptor_distance": 0.13,
  "yaw_shift_deg": -84.0,
  "fitness": 0.42,
  "inlier_ratio": 0.37,
  "rmse": 0.21,
  "stage": "geometrically_verified"
}
```

### Acceptance Test

In a symmetric or repeated scene:

```text
descriptor candidates may exist
bad candidates rejected by registration
accepted false positives should remain low or zero
```

---

## Step 4 — Add Consistency-Based Acceptance

### Task

Do not publish `aligned` after one match unless explicitly in debug mode.

Implement consistency graph:

```text
node = verified cross-robot match
edge = pairwise transform consistency
accept alignment if clique_size >= min_consistent_matches
```

Simplified consistency check:

```python
T_ab_i = transform estimated by match i
T_ab_j = transform estimated by match j

delta = inverse(T_ab_i) * T_ab_j

consistent = (
    translation_norm(delta) < 1.0 and
    yaw_error(delta) < 10_deg and
    roll_pitch_error(delta) < 5_deg
)
```

Modify:

```text
src/collaborative_exploration/team_loop_closure/team_loop_closure/relative_transform_manager_node.py
```

### Reference Project A

**robust_multirobot_map_merging**  
Repo: https://github.com/lajoiepy/robust_multirobot_map_merging

### Main Concept

Select an inlier subset from many inter-robot measurements using pairwise consistency.

### Why Helpful

One wrong inter-robot loop closure can corrupt the whole merged map. Pairwise consistency prevents single false positives from triggering map merge.

### Reference Project B

**Kimera-Multi**  
Repo: https://github.com/MIT-SPARK/Kimera-Multi

### Main Concept

Robust multi-robot loop closure handling with incorrect inter/intra-robot loop closure rejection under perceptual aliasing.

### Why Helpful

Use its robust acceptance logic as a design pattern even though its sensor stack is visual-inertial rather than Fast-LIO LiDAR.

### Params to Add

```yaml
team_alignment_min_matches: 2
team_alignment_max_translation_disagreement: 1.0
team_alignment_max_yaw_disagreement_deg: 10.0
team_alignment_single_match_debug: false
```

### States

```text
unaligned
tentative
aligned
rejected
```

### Acceptance Test

```text
one high-score match:
  alignment_status = tentative
  map_merge = disabled

two consistent matches:
  alignment_status = aligned
  map_merge = enabled if team_alignment_enable_map_merge=true

two inconsistent matches:
  alignment_status = rejected or tentative
  map_merge = disabled
```

---

## Step 5 — Publish Discovered Relative Transform and Gate Map Merge

### Task

After accepted alignment, publish:

```text
/team_slam/relative_transform
/team_slam/alignment_status
```

Frame convention:

```text
parent: robot_a/map
child: robot_b/map
transform: T_robot_a_map_robot_b_map
```

Do not publish conflicting global `map -> base_link` transforms.

Modify:

```text
src/collaborative_exploration/team_loop_closure/team_loop_closure/relative_transform_manager_node.py
src/collaborative_exploration/team_loop_closure/team_loop_closure/discovered_map_merge_bootstrap_node.py
src/vendor/multirobot_map_merge/
```

### Reference Project

**Multi-Robot-Graph-SLAM**  
Repo: https://github.com/aserbremen/Multi-Robot-Graph-SLAM

### Main Concept

ROS2 multi-robot 3D LiDAR graph SLAM with namespace-aware multi-robot graph sharing.

### Why Helpful

Use it as a ROS2 namespace, multi-robot graph, and map-sharing structure reference.

### Runtime Rules

```text
unaligned:
  no /merged_map
  no transform bridge between robot maps

tentative:
  publish diagnostics only
  no map merge

aligned:
  publish T_robot_a_map_robot_b_map
  allow discovered map merge
```

### Acceptance Test

Before alignment:

```bash
ros2 topic list | grep merged_map
```

Expected:

```text
no /merged_map
```

After alignment:

```text
/team_slam/alignment_status = aligned
/merged_map appears
```

---

## Step 6 — Feed Inter-Robot Loop Candidates into CFPA2 Only After Alignment

### Task

Modify:

```text
src/collaborative_exploration/reconstruction_awareness/reconstruction_awareness/loop_closure_candidate_node.py
src/collaborative_exploration/reconstruction_awareness/reconstruction_awareness/pose_graph_health_node.py
CFPA2 allocator integration
```

Before alignment:

```text
source=inter_robot_rendezvous disabled
```

After alignment:

```text
source=inter_robot_rendezvous enabled
coordinates transformed into target robot local frame
```

### Reference Project

Internal proposal: `Loop_Closure_Reconstruction_Grounded_Scene_Reports_Proposal.md`

### Main Concept

Keep CFPA2 as the stable planner backbone, then add loop-closure candidates and `loop_gain` as a planner utility term.

### Why Helpful

This keeps the contribution as loop-closure-aware collaborative exploration, not as a new SLAM backend.

### Candidate Schema

```json
{
  "id": "loop_inter_0031",
  "role": "loop_close",
  "source": "inter_robot_rendezvous",
  "target_robot": "robot_b",
  "x": 4.2,
  "y": -1.1,
  "loop_gain": 0.81,
  "alignment_confidence": 0.74,
  "mobility_risk_go2": 0.2,
  "mobility_risk_go2w": 0.6
}
```

### Acceptance Test

```bash
./scripts/launch/nav_test_demo3_mixed.sh \
  loop_closure:=true \
  loop_closure_backend:=ros1_bridge \
  relative_pose_source:=discovered \
  inter_robot_loop_closure:=true \
  team_alignment_min_matches:=2 \
  mujoco_cameras:=false
```

Expected:

```text
before aligned:
  no inter_robot_rendezvous candidates

after aligned:
  inter_robot_rendezvous candidates appear
  candidates are in target robot frame
```

---

## Step 7 — Add Benchmark Scripts

### Task

Add:

```text
scripts/bench/benchmark_cross_loop_closure.sh
scripts/bench/cross_loop_closure_reporter.py
```

### Reference Project

Internal proposal evaluation section.

### Main Concept

Evaluate loop closure not only by coverage, but by localization quality, inter-robot loop closure count, map consistency, and false-positive rejection.

### Metrics

```text
number of descriptor candidates
number of geometrically verified matches
number of accepted consistent matches
time to first alignment
alignment error vs GT, evaluation only
false-positive rejection count
merged_map enabled time
inter_robot_rendezvous candidate count
coverage before/after alignment
```

### Output

```text
logs/cross_loop_closure_summary.json
logs/cross_loop_closure_summary.md
```

Example JSON:

```json
{
  "relative_pose_source": "discovered",
  "gt_used_runtime": false,
  "descriptor_candidates": 18,
  "verified_matches": 5,
  "consistent_matches": 3,
  "alignment_status": "aligned",
  "time_to_alignment_sec": 74.2,
  "alignment_error_eval_only": {
    "translation_m": 0.42,
    "yaw_deg": 3.8
  },
  "inter_robot_rendezvous_candidates": 7
}
```

---

## Step 8 — Implement v2 Central `team_pose_graph_node`

### Task

Only start this after Steps 1–7 are stable.

Create:

```text
src/collaborative_exploration/team_loop_closure/team_loop_closure/team_pose_graph_node.py
```

or a C++ package if Python GTSAM performance is too weak.

Subscribe:

```text
/team_slam/keyframes
/team_slam/cross_robot_matches
/robot_a/Odometry
/robot_b/Odometry
/robot_a/corrected_odom
/robot_b/corrected_odom
```

Build graph:

```text
Variables:
  X(a, k)
  X(b, k)

Factors:
  odom factor:        X(a,k) -> X(a,k+1)
  odom factor:        X(b,k) -> X(b,k+1)
  self loop factor:   X(a,i) -> X(a,j)
  self loop factor:   X(b,i) -> X(b,j)
  inter loop factor:  X(a,i) -> X(b,j)
```

### Reference Project A

**GTSAM**  
Official: https://gtsam.org/  
GitHub: https://github.com/borglab/gtsam

### Main Concept

Factor graph optimization using `BetweenFactorPose3`, `Pose3`, and iSAM2.

### Why Helpful

This is the correct backend for true multi-robot pose graph correction:

```text
inter-robot match -> BetweenFactorPose3(X_a_i, X_b_j)
```

### Reference Project B

**FAST_LIO_SLAM**  
Repo: https://github.com/gisbi-kim/FAST_LIO_SLAM

### Main Concept

FAST-LIO2 and SC-PGO run separately. SC-PGO consumes odometry and LiDAR point cloud topics from FAST-LIO2 and produces optimized map output.

### Why Helpful

Use it to understand how Fast-LIO odometry and SC-PGO are wired in a single-robot setup.

### Reference Project C

**Kimera-Multi / DPGO**  
Repo: https://github.com/MIT-SPARK/Kimera-Multi

### Main Concept

Distributed robust multi-robot pose graph optimization.

### Why Helpful

Do not implement distributed optimization now. Use its architecture as future reference after centralized v2 works.

### Deliverable

Publish:

```text
/team_slam/robot_a/corrected_odom_global
/team_slam/robot_b/corrected_odom_global
/team_slam/team_map_to_robot_a_map
/team_slam/team_map_to_robot_b_map
/team_slam/pose_graph_metrics
```

Metrics schema:

```json
{
  "num_keyframes_robot_a": 120,
  "num_keyframes_robot_b": 118,
  "num_self_loop_factors": 6,
  "num_inter_robot_factors": 3,
  "latest_optimization_error": 12.4,
  "alignment_confidence": 0.83
}
```

### Acceptance Test

Use GT only for evaluation:

```text
ATE before team PGO
ATE after team PGO
map consistency before/after
```

Runtime graph must not consume GT.

---

## Implementation Priority

```text
P0: no-GT audit and gating
P1: real Scan Context descriptor
P2: cross-robot candidate retrieval
P3: registration verification
P4: consistency-based acceptance
P5: discovered transform + map merge gating
P6: CFPA2 inter_robot_rendezvous after alignment
P7: benchmark reporter
P8: central GTSAM team_pose_graph_node
```

Do not start with `team_pose_graph_node` before P1–P5 are stable. A pose graph with bad inter-robot factors is worse than no cross-loop closure.

---

## Prompt to Coding Agent

```text
You are working in the Collab_QRC repo. Implement the cross-robot LiDAR loop closure plan in docs/cross_robot_loop_closure_implementation_plan.md.

Scope:
- Focus only on LiDAR/odometry-based unknown-initial-pose cross-robot loop closure.
- Do not use VLM, 3DGS, scene graph, or ground-truth runtime alignment.
- Preserve current per-robot Fast-LIO + ROS1 SC-PGO path.
- Do not claim or implement true multi-robot pose graph correction until team_pose_graph_node is explicitly added.

Implementation order:
1. Audit and enforce no-GT behavior for relative_pose_source:=none|discovered.
2. Replace placeholder keyframe descriptor with real Scan Context in team_loop_closure.
3. Implement cross-robot-only descriptor retrieval using ring-key / descriptor distance / yaw shift.
4. Add geometric verification using KISS-Matcher, TEASER++, Quatro/Nano-GICP, or a GICP fallback.
5. Add consistency-based acceptance in relative_transform_manager_node. Do not publish aligned from a single match unless debug mode is enabled.
6. Publish discovered T_robot_a_map_robot_b_map only after >=2 consistent verified matches.
7. Gate /merged_map and inter_robot_rendezvous candidates until alignment_status == aligned.
8. Add benchmark scripts and JSON/Markdown reporter for descriptor candidates, verified matches, accepted matches, false positives, time-to-alignment, and GT evaluation-only alignment error.
9. Only after Steps 1–8 pass, implement central GTSAM team_pose_graph_node with per-robot odom factors, self-loop factors, and inter-robot loop factors.

Reference projects:
- DiSCo-SLAM: multi-robot LiDAR SLAM with Scan Context descriptor exchange.
- scancontext_ros2: ROS2 Scan Context descriptor and retrieval pattern.
- FAST-LIO-SAM-SC-QN: FAST-LIO2 + ScanContext + Quatro + Nano-GICP + GTSAM/iSAM2.
- KISS-Matcher: ROS2-friendly robust point cloud registration.
- TEASER++: robust 3D registration under high outlier rate.
- robust_multirobot_map_merging: pairwise consistency for rejecting bad inter-robot measurements.
- Kimera-Multi: robust multi-robot loop closure and distributed PGO design pattern.
- Multi-Robot-Graph-SLAM: ROS2 multi-robot graph / namespace structure reference.
- GTSAM: backend for v2 true multi-robot pose graph correction.

Acceptance criteria:
- bootstrap_from_gt=false for relative_pose_source:=none|discovered.
- No runtime /odom/ground_truth alignment in discovered mode.
- No /merged_map before verified alignment.
- /team_slam/keyframes publishes real Scan Context descriptors.
- /team_slam/cross_robot_candidates contains only cross-robot matches.
- /team_slam/cross_robot_matches rejects descriptor-only false positives through registration.
- /team_slam/alignment_status becomes aligned only after >=2 consistent verified matches.
- /merged_map and inter_robot_rendezvous candidates appear only after aligned.
- Benchmark reporter writes logs/cross_loop_closure_summary.json and logs/cross_loop_closure_summary.md.

Before editing, inspect the existing launch files and team_loop_closure package. Make the smallest robust changes first, run colcon build on affected packages, and provide a final summary with changed files, commands run, and pass/fail status.
```



P0-P7 implementation and discovered-mode core runtime smoke have passed. Do not add new features.

Your next task is validation only.

Do not implement:
- team_pose_graph_node
- GTSAM pose graph
- KISS-Matcher
- TEASER++
- Quatro
- Nano-GICP
- VLM
- 3D scene graph
- new planner behavior

Only do the following two validation tasks.

Task 1: Rerun fixed 180s benchmark

First clean old ROS1 bridge session if needed:

docker compose -f docker/ros1_scpgo/docker-compose.yml down

Then start SC-PGO bridge:

bash scripts/launch/scpgo_ros1_bridge.sh

Then run:

DURATION_SEC=180 scripts/bench/benchmark_cross_loop_closure.sh

Expected output:
- logs/cross_loop_closure_summary.json
- logs/cross_loop_closure_summary.md

The previous summary with keyframes=0 and descriptor_candidates=0 is invalid because it was generated before the recorder fix. Regenerate it.

Benchmark pass criteria:
- keyframes > 0
- descriptor_candidates > 0 in overlap scene
- cross_robot_matches > 0
- rejected_matches > 0 or false_positive_rejects recorded
- alignment_status is captured
- if aligned:
  - time_to_alignment_sec is non-null
  - merged_map_enabled_time_sec is non-null
  - consistent_matches >= team_alignment_min_matches
- GT alignment error is eval-only and not consumed by runtime nodes

Also verify:
- /robot_a/odom/ground_truth subscriber count = 0 during runtime
- /robot_b/odom/ground_truth subscriber count = 0 during runtime
- pose_graph_health_node does not subscribe to /odom/ground_truth when enable_gt_drift_metrics=false
- team_loop_closure nodes do not subscribe to /odom/ground_truth

Task 2: Run dedicated symmetric/repeated false-positive scene

Use a scene where robot_a and robot_b observe visually/geometrically similar but different corridor or room segments.

Expected behavior:
- /team_slam/keyframes still publishes valid Scan Context descriptors
- /team_slam/cross_robot_candidates may appear
- bad / ambiguous candidates are rejected by icp_2d registration
- rejected matches include reject_reason
- alignment_status must not become aligned from false positives
- /team_slam/relative_transform must not be published unless >=2 consistent verified matches exist
- /merged_map must stay disabled if alignment is not valid
- inter_robot_rendezvous candidates must stay disabled before valid alignment

If false alignment occurs:
Do not add a new backend.
Do not add KISS-Matcher yet.
First tune only these existing thresholds:
- registration_max_rmse
- registration_min_inlier_ratio
- registration_max_fitness
- registration_min_correspondences
- team_alignment_max_translation_disagreement
- team_alignment_max_yaw_disagreement_deg
- team_alignment_min_matches

Deliverables:
1. Updated logs/cross_loop_closure_summary.json
2. Updated logs/cross_loop_closure_summary.md
3. A short validation note containing:
   - overlap scene result
   - symmetric/repeated false-positive scene result
   - whether alignment was correct
   - whether /merged_map gating behaved correctly
   - whether GT runtime subscriber count stayed 0
   - any threshold changes made

Do not change the project claim boundary.

Current valid claim:
independent per-robot SC-PGO loop closure
+ discovered inter-robot map alignment
+ loop-closure-aware rendezvous assistance

Invalid claim:
true multi-robot pose graph correction