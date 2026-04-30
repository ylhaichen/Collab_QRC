# Sensing- and Mobility-Aware Collaborative Exploration for Asymmetric Quadruped Teams

## Research Proposal

**Project:** Collab_QRC heterogeneous quadruped collaborative exploration  
**Working title:** Sensing- and Mobility-Aware Collaborative Exploration for Asymmetric Quadruped Teams  
**Short title:** Asymmetric Quadruped Exploration  
**Date:** 2026-04-27  
**Platform:** Unitree Go2 + Unitree Go2W, ROS 2 Humble, MuJoCo, real-robot deployment  
**Compute:** RTX 4090, 4x A100, 2x RTX 4070  
**Core idea:** Go2 has better LiDAR and should act as a trusted mapper / validator / localization anchor. Go2W has weaker onboard LiDAR but better flat-ground speed and wheeled-legged mobility, so it should act as a fast scout / executor under map trust and safety constraints.

---

## Current Implementation Snapshot (2026-04-30)

The first runnable Loop+Risk stage now exists in the repo as an extension of the existing ROS 2 / MuJoCo / CFPA2 stack:

- `reconstruction_awareness` publishes `pose_graph_health`, `loop_candidates`, `morphology_risk`, and per-robot `peer_scan` inputs.
- CFPA2 consumes the awareness topics and can assign `role=loop_close` goals in addition to normal frontier exploration.
- `nav_test_mujoco_fastlio_mixed.launch.py` exposes launch switches for `role_awareness_enabled`, `loop_candidates_enabled`, `morphology_risk_enabled`, `peer_obstacle_enabled`, and `loop_risk_output_dir`.
- `benchmark_loop_risk_allocator.sh` runs `coverage_only_mppi`, `loop_only_mppi`, and `loop_risk_mppi` modes and now records loop-role assignment counts plus recovery counters in `loop_risk_summary.json`.

Runtime evidence from a 120 s headless MuJoCo smoke test on 2026-04-29:

```text
OUT_DIR=/tmp/loop_risk_smoke_role_20260429_2342
mode=loop_risk_mppi
coverage=78.55%
loop_candidate_count_max=20
pose_health_states_seen=["loop_needed"]
loop_close_assignment_seen=true
robot_b loop_close assignments=1
robot_b obstacle_contacts=213
```

The important positive result is that `loop_close` is no longer just produced by the candidate node; it reaches the CFPA2 assignment log. The first smoke exposed a narrow-passage `pivot-lock` mismatch where `_set_active_goal()` changed the accepted goal locally but callers still published the rejected candidate. `_set_active_goal()` now returns the accepted goal after pivot-lock arbitration, and all goal-publish call sites use that returned value.

Post-guard runtime evidence from a 120 s headless MuJoCo smoke test on 2026-04-30:

```text
OUT_DIR=/tmp/loop_risk_smoke_pivotguard_20260430_run
mode=loop_risk_mppi
coverage=59.83%
loop_candidate_count_max=20
loop_close_assignment_seen=true
loop_close assignments=2
first_loop_close_assignment=robot_a goal=(3.80,1.90)
robot_a obstacle_contacts=13
robot_b obstacle_contacts=372
```

This confirms the Loop+Risk role chain after the pivot-lock accepted-goal fix. The remaining failure mode is local execution recovery: `robot_b` can wedge near a Nav2/MPPI goal, repeatedly run `BackUp`, and republish the same goal without telling CFPA2 to rotate away from that target. The current code now closes that feedback loop by making `stuck_watchdog` publish `/{namespace}/frontier_replan`; CFPA2 receives that event, blacklists the current goal with the configured cluster radius, and the benchmark summary records `watchdog_frontier_replan` and `cfpa2_frontier_replan_blacklist` counts.

### Stage 1 verification — 180 s sweet-spot smoke (2026-04-30 02:17)

After the recovery-chain runtime confirmation, four follow-up tunings were applied to fix several Loop+Risk failure modes that the first smokes exposed:

1. **MPPI centerline preference** — SmacPlannerHybrid `cost_penalty: 2.0 → 5.0` (path through doorway centers, not wall-hug) and MPPI `PathAlignCritic.cost_weight: 14 → 20` (controller follows centerline tightly).
2. **Peer-obstacle dynamic clearing** — `peer_scan` layer was `clearing: false` so peer's historical positions accumulated in costmap as a "peer trail" → late-stage RViz clutter and coverage stall. Set `clearing: true` so peer history is raytrace-cleared each tick.
3. **`bt_navigator.odom_topic` typo** — `nav2_go2_full_stack.yaml` had `/robot_a/odom/nav` for robot_b's BT navigator (copy-paste from go2w yaml). Fixed to `/robot_b/odom/nav`. Symptom before fix: `bt_navigator` continuously aborted goal handles because it was reading robot_a's pose for robot_b's nav.
4. **Role utility weight sweep** — bisection over `role_w_loop` × `role_early_loop_scale` landed on `1.0 / 1.0` for occasional-priority loop_close (~30 % of ASSIGN events) without dominating.

180 s smoke evidence after all four tunings:

```text
OUT_DIR=/tmp/loop_risk_smoke_180s_021314
mode=loop_risk_mppi
coverage=73.1%
distance robot_a=209.08m  robot_b=68.78m
obstacle_contacts robot_a=0  robot_b=0
yaw_drift_peak robot_a=4.7°  robot_b=3.1°
loop_close_assignments=12 (a=8, b=4)
recovery: watchdog_replan=2  cfpa2_replan_bl=2
```

This is the first end-to-end run where all Stage 1 components fire correctly: pose_graph_health publishes `loop_needed`, loop_closure_candidate generates 20 candidates, peer_obstacle_scan keeps both costmaps populated with peer geometry, the recovery chain triggers without deadlock, and CFPA2 assigns `loop_close` in proportion to its declared utility.

### Stage 2 ablation — 3 modes × 3 trials × 180 s (2026-04-30 02:18, demo3_mixed, equal Livox)

```text
mode                | cov%   | Go2W_m  | Go2_m  | Go2W_obs | Go2_obs | Go2W_yaw | Go2_yaw | tip | loop a/b
--------------------+--------+---------+--------+----------+---------+----------+---------+-----+---------
coverage_only_mppi  | 83.2 % | 233.84  |  36.24 |    66    |   387   |   3.4°   |  4.2°   |  1  |  0/0
loop_only_mppi      | 77.3 % | 132.41  |  42.19 |    34    |   894   |   4.3°   |  3.6°   |  0  | 12/12
loop_risk_mppi      | 49.1 % | 110.78  | 168.11 |     1    |   626   | 180.0° * |  4.7°   |  0  | 16/6

* trial 3 single SLAM yaw-divergence event for robot_a; mean of trials 1+2 ≈ 3 °.
```

**Monotonic improvements** (coverage_only → loop_only → loop_risk):
- Go2W obstacle contacts: 66 → 34 → **1** (×66 reduction; peer_obstacle + loop revisit removes corner stamping)
- Go2W tip events: 1/3 → 0/3 → 0/3 (loop revisit avoids the bag-of-stamps tip-over pathway)
- Go2 distance: 36 → 42 → **168 m** (peer_obstacle lets Go2 actually traverse instead of pivot-locking near peer)
- loop_close balance: 0 → 24 (12+12) → 22 (16+6) — Loop+Risk shifts revisit toward Go2W where wheel mobility is cheaper

**Trade-offs**:
- Coverage cost: 83 → 77 → 49 % (loop revisit eats exploration time; trial-3 SLAM divergence drops the loop_risk mean further)
- Go2 obstacle contacts spike then partially recover: 387 → 894 → 626 (loop revisit pulls Go2 into tight corner geometry; peer_obstacle helps but does not eliminate)
- New failure mode in loop_risk: 1/3 trials had a catastrophic Fast-LIO yaw divergence (180 °) for robot_a, causing the trial to abort exploration. Sample size 3 is below the CLAUDE.md PASS-criterion 5-trial minimum; this single divergence dominates the mean.

The Loop+Risk pipeline runs end-to-end and the per-robot safety improvements are real, but n = 3 trials per mode is too small for paper-grade claims and 1/3 SLAM divergence is a real reliability gap that needs a separate fix before scaling up.

Artifacts: `results/ablation/20260430_021813/{coverage_only_mppi,loop_only_mppi,loop_risk_mppi}/`.

## 1. Executive Summary

This proposal targets a research gap in **multi-robot collaborative exploration**: most existing systems treat robots as roughly homogeneous agents, or only model heterogeneity at the high-level task-allocation layer. In our platform, the heterogeneity is fundamental and physically meaningful:

- **Go2** has a better LiDAR stack and can produce more reliable occupancy, traversability, and localization estimates.
- **Go2W** has weaker onboard LiDAR, but can move faster and more efficiently on flat or semi-structured ground due to its wheeled-legged morphology.
- Both robots operate as real quadruped systems, where planner decisions must respect contact risk, leg/body clearance, SLAM drift, narrow-space deadlock, and sensor failure.

The proposed thesis is:

> In heterogeneous quadruped teams, sensing asymmetry is not a nuisance to be hidden by map merging; it is a planning variable. A fast but weakly sensing robot becomes useful only when coordinated through trust-aware mapping, active validation, morphology-aware frontier allocation, and runtime safety verification.

The proposed system will jointly reason about:

- `information gain`
- `sensor confidence`
- `map trust`
- `SLAM health`
- `robot-specific mobility cost`
- `locomotion risk`
- `execution failure probability`
- `runtime safety rejection`

The expected outcome is a publishable system paper and algorithmic contribution for **heterogeneous quadruped collaborative exploration**, validated in MuJoCo and on real Go2 + Go2W hardware.

---

## 2. Available Assets

### 2.1 Hardware

| Asset | Role in this proposal |
|---|---|
| Unitree Go2 | High-quality sensing robot, trusted mapper, validator, loop-closure anchor |
| Unitree Go2W | Fast wheeled-legged scout, low-risk corridor sweeper, executor on trusted map |
| Better LiDAR on Go2 | Enables asymmetric observation model and trust-weighted map fusion |
| Weaker native LiDAR on Go2W | Creates real sensing asymmetry and measurable failure mode |
| RTX 4090 | Local training / evaluation / visualization |
| 4x A100 | Large-scale simulation rollout, supervised failure prediction, risk model training |
| 2x RTX 4070 | Parallel experiments, lightweight model inference, dev workloads |

### 2.2 Existing Repo Capabilities

The current `Collab_QRC` repo already provides most of the required system substrate:

- ROS 2 Humble multi-robot infrastructure.
- MuJoCo simulation with Go2 / Go2W assets.
- Real Go2 and Go2W bringup.
- Fast-LIO2 + SC-PGO path for high-quality LiDAR-inertial SLAM.
- Cartographer / 2D occupancy path for lighter mapping.
- FAR / TARE / CFPA2 / A* navigation and exploration backends.
- Contact and tip-over metrics from MuJoCo.
- `session_reporter.py` style benchmark reporting.
- `path_safety_filter` and `cmd_vel_safety_shield` for runtime safety.
- Door task and VLM exploration components, usable as optional semantic extension.

This means the proposal does not require building a full autonomy stack from scratch. The research work should focus on the **decision layer**, **map trust layer**, **evaluation protocol**, and **real-robot validation**.

---

## 3. Research Problem

### 3.1 Problem Statement

Given a team of quadruped robots with asymmetric sensing and mobility, explore an unknown environment efficiently while maintaining map reliability and physical safety.

Let the robot team be:

```text
R = {Go2, Go2W}
```

Each robot has a different sensor model:

```text
S_Go2  = high-quality LiDAR, higher observation confidence
S_Go2W = weak native LiDAR, lower observation confidence
```

and a different mobility model:

```text
M_Go2  = slower walking, better for careful validation / rough or narrow regions
M_Go2W = faster wheeled-legged motion, better for flat long-range traversal
```

The system must decide:

1. Which robot should visit each frontier?
2. Which robot's map updates should be trusted?
3. When should Go2 validate an uncertain area observed by Go2W?
4. When should Go2W exploit trusted Go2 map information for fast coverage?
5. When should either robot stop, replan, or reject unsafe commands?

### 3.2 Why Existing Formulations Are Insufficient

Classical multi-robot exploration often optimizes:

```text
frontier utility = information gain - travel cost
```

This is not enough for the current platform because:

- A Go2W frontier may look cheap by distance but be unreliable due to poor LiDAR.
- A Go2 frontier may look slow by travel time but produce high-value trusted map information.
- Naive map union can mark false free-space from weak sensing as safe, causing contact or stuck events.
- Frontier assignment that ignores `SLAM health` can amplify map drift.
- Planner success in occupancy space does not imply physical success for quadruped body and leg envelopes.

The proposal therefore treats exploration as a coupled problem:

```text
collaborative exploration = active mapping + heterogeneous task allocation
                          + sensing trust + morphology-aware motion cost
                          + failure memory + runtime safety assurance
```

---

## 4. Literature Landscape and Novelty Boundary

### 4.1 Exploration and Active SLAM

**TARE** introduced a hierarchical exploration framework for complex 3D environments, using dense local processing and sparse global planning to improve exploration efficiency. It reports faster exploration and lower computation than earlier methods in complex environments. This is a strong baseline, not something to re-claim.

**Efficient Multi-robot Active SLAM** integrates frontier sharing, pose graph uncertainty, and path entropy into a utility function for multi-robot exploration. This shows that uncertainty-aware exploration is already an active research direction.

**Learning-Based Multi-Robot Active SLAM survey** frames multi-robot AC-SLAM as a coupled system of decentralized decision-making and distributed factor graph estimation. This supports the need to jointly reason about planning and estimation.

**Gap left open:** these works do not deeply model **real sensor-asymmetric quadruped teams**, where one robot's map updates are more trustworthy than another's and mobility differs physically.

### 4.2 Multi-Robot Coordination Under Communication Limits

Recent **low-bandwidth decentralized multi-robot exploration** work proposes Cross-rank coordination, sharing only robot positions under severe bandwidth limits, and validates with real quadruped robots. This is highly relevant and strong.

However, that work primarily assumes homogeneous robots and similar sensing stacks. It also explicitly highlights issues when fast and slow robots are treated similarly.

**Gap left open:** low-bandwidth coordination has not fully addressed asymmetric sensing quality, map trust, and morphology-dependent frontier utility.

### 4.3 FAR / TARE / SubT-Style Navigation

**FAR Planner** uses a dynamically updated visibility graph for fast replanning in known and unknown environments, and was used in DARPA SubT-style systems. This is a strong route-planning baseline.

**Gap left open:** FAR is a path planner, not a sensing-trust-aware collaborative exploration policy. In this repo, FAR also has known edge cases, such as V-graph connections through sparse wall observations and unsafe pivot behaviour near obstacles.

### 4.4 VLM and Semantic Multi-Robot Navigation

**Co-NavGPT** and **COMRES-VLM** already use VLMs for multi-robot visual semantic navigation, frontier assignment, topological skeleton reasoning, and object search.

**Gap left open:** VLM-based coordination is not the main novelty here. This proposal focuses on physical robot asymmetry, map trust, sensor confidence, contact-aware safety, and real quadruped deployment. VLM can be used later as an optional semantic prior, not as the core claim.

### 4.5 Map Fusion and Heterogeneous Sensors

Map merging and heterogeneous sensor fusion have been studied in multi-robot SLAM. Existing reviews note that different sensor quality and map types make direct map fusion difficult.

**Gap left open:** much of this literature focuses on map merging accuracy, not on how sensor trust should change **frontier allocation**, **validation behaviour**, and **physical exploration strategy**.

### 4.6 Novelty Positioning

This project should not claim novelty in:

- frontier-based exploration itself
- hierarchical exploration itself
- multi-robot map sharing itself
- VLM frontier assignment itself
- generic safety filtering itself

The strongest novelty claim is:

> A unified system for heterogeneous quadruped collaborative exploration where asymmetric LiDAR quality and asymmetric mobility are explicitly modeled in map fusion, frontier allocation, active validation, and runtime safety.

---

## 5. Research Questions

### RQ1: Sensor-Asymmetric Planning

How should frontier allocation change when one robot has high-quality LiDAR and another has weak LiDAR?

Expected answer:

- Go2 should prioritize high-uncertainty, low-confidence, safety-critical, and validation-heavy frontiers.
- Go2W should prioritize low-risk, trusted, flat, high-throughput regions.

### RQ2: Trust-Aware Map Fusion

Can source-aware occupancy confidence reduce false-free regions and contact-inducing map errors compared with naive map fusion?

Expected answer:

- Confidence-weighted fusion should reduce false-free rate and unsafe frontier assignments.
- Conflicting observations should trigger validation instead of direct execution.

### RQ3: Active Validation

When Go2W observes a frontier with low confidence, is it better to send Go2 for validation before allowing Go2W to enter?

Expected answer:

- Validation adds time overhead, but reduces contact, stuck events, and map inconsistency.
- The best strategy should validate selectively, not always.

### RQ4: Mobility-Sensing Coupling

Does a joint sensing + mobility utility outperform sensing-only or mobility-only assignment?

Expected answer:

- Sensing-only underuses Go2W speed.
- Mobility-only overtrusts weak Go2W mapping.
- Joint utility should achieve better coverage per failure event.

### RQ5: Real-Quadruped Robustness

Do the benefits survive real robot deployment with LiDAR noise, drift, communication latency, and physical contact risk?

Expected answer:

- Real deployment should show fewer unsafe decisions and better map reliability than naive dual exploration, even if absolute coverage speed is lower than simulation.

---

## 6. Hypotheses

### H1: Trust-aware map fusion improves physical safety.

Compared with naive occupancy union, trust-aware map fusion will reduce:

- `false-free rate`
- `contact count`
- `leg scuff events`
- `unsafe path proposals`

### H2: Sensing- and mobility-aware assignment improves team efficiency.

Compared with distance-only, information-gain-only, or mobility-only assignment, the full method will improve:

- `coverage per minute`
- `coverage per meter`
- `coverage per contact`
- `successful frontier completion rate`

### H3: Active validation reduces weak-sensor failure modes.

When Go2W's sensor confidence is low, selective Go2 validation will reduce:

- repeated unreachable frontier attempts
- map conflicts
- SLAM drift propagation
- collisions caused by poor local occupancy

### H4: Realistic quadruped metrics change method ranking.

A method that performs well under simple coverage/time metrics may perform poorly when evaluated with:

- contact
- scuff
- tip-over
- stuck
- drift
- deadlock
- false-free map error

---

## 7. Proposed Method

### 7.1 System Overview

The proposed system has six modules:

```text
Per-Robot Sensing Model
        |
Trust-Aware Map Fusion
        |
Frontier / Topological Graph Extraction
        |
Sensing- and Mobility-Aware Assignment
        |
Runtime Safety Verification
        |
Failure Memory and Active Validation
```

The method should initially be implemented as a wrapper around the existing CFPA2 / TARE / FAR infrastructure rather than replacing the whole navigation stack.

### 7.2 Per-Robot Sensing Model

Each robot maintains a sensor confidence model:

```text
sensor_confidence_i(x, t) in [0, 1]
```

where `i` is robot identity and `x` is a map cell, voxel, frontier cluster, or local region.

Candidate features:

- LiDAR max range
- point density per sector
- vertical field-of-view coverage
- return intensity / dropout ratio if available
- scan age
- scan incidence angle
- registration quality
- local map entropy
- SLAM odometry health
- IMU inconsistency
- observed contact or slip event

Pragmatic first version:

```text
Go2 base sensor trust  = 1.0
Go2W base sensor trust = 0.45 to 0.65
```

Then modulate by local runtime health:

```text
trust_i = base_i
        * lidar_density_score
        * slam_health_score
        * recency_score
        * consistency_score
```

This allows the first paper version to avoid overfitting a learned model too early.

### 7.3 Trust-Aware Occupancy Map

Instead of a binary occupancy grid:

```text
cell ∈ {free, occupied, unknown}
```

use a source-aware belief representation:

```text
cell = {
  p_free,
  p_occupied,
  confidence,
  source_robot_ids,
  last_observed_time,
  conflict_score,
  validation_required
}
```

#### Fusion rule

For each observation `z_i` from robot `i`:

```text
log_odds(cell) <- log_odds(cell) + trust_i * inverse_sensor_model(z_i)
confidence(cell) <- update_confidence(confidence, trust_i, recency)
```

If Go2W says `free` but Go2 says `occupied`, mark:

```text
conflict_score ↑
validation_required = true
```

If Go2 and Go2W agree:

```text
confidence ↑
validation_required = false
```

### 7.4 Frontier Graph with Trust Attributes

Each frontier cluster should carry:

```text
frontier = {
  centroid,
  expected_information_gain,
  source_robot,
  map_confidence,
  conflict_score,
  validation_required,
  local_traversability_score_Go2,
  local_traversability_score_Go2W,
  slam_uncertainty,
  nearest_loop_closure_value,
  historical_failure_score
}
```

The frontier is not just geometry. It becomes a decision object that knows:

- who discovered it
- how reliable it is
- which robot can sense it better
- which robot can move there safely
- whether it needs validation

### 7.5 Sensing- and Mobility-Aware Utility

For robot `i` and frontier `f`:

```text
U(i, f) =
  w_info   * ExpectedInformationGain(i, f)
- w_travel * TravelCost(i, f)
- w_mob    * LocomotionRisk(i, f)
- w_sense  * SensingUncertainty(i, f)
- w_map    * MapTrustPenalty(f)
- w_fail   * PredictedExecutionFailure(i, f)
- w_slam   * LocalizationUncertainty(i, f)
- w_safe   * RuntimeSafetyCost(i, f)
```

Where:

```text
SensingUncertainty(i, f) = 1 - expected_sensor_confidence_i(f)
MapTrustPenalty(f)       = 1 - map_confidence(f)
```

The allocation policy can start with a simple auction:

```text
robot_i bids on frontier_f using U(i, f)
assign highest positive utility while enforcing robot separation and deadlock constraints
```

Then later extend to:

- Hungarian assignment
- MDVRP-style assignment
- receding-horizon graph assignment
- learned utility weight prediction

### 7.6 Active Validation Policy

When Go2W discovers or proposes a low-confidence frontier:

```text
if validation_required(frontier) and Go2_validation_cost < risk_threshold:
    assign Go2 -> validation viewpoint
    hold or redirect Go2W
else:
    allow Go2W if safety verifier accepts path
```

Validation outcomes:

- `confirmed_free`: Go2W can proceed quickly.
- `confirmed_blocked`: frontier is blacklisted or reclassified.
- `uncertain`: request closer viewpoint, slow mode, or stop.
- `unsafe`: safety layer rejects execution and updates failure memory.

This is the key behavioural novelty. The system does not just merge maps; it actively moves the better sensor to resolve uncertainty.

### 7.7 Cooperative Localization Anchor

When Go2W's localization quality degrades:

```text
if SLAM_health_Go2W < threshold:
    assign Go2 to loop-closure / rendezvous / shared-viewpoint region
    reduce Go2W speed or restrict it to trusted map
```

Possible implementation levels:

1. Simple heuristic using SLAM drift metrics and odom-map inconsistency.
2. Trigger rendezvous when Go2W map conflict rises.
3. Add active loop-closure viewpoint utility.

This module is optional for the first submission but valuable for a stronger paper.

### 7.8 Runtime Safety Verification

The planner can still be wrong. Every assigned path and velocity command should pass through a runtime assurance layer:

- footprint collision check
- oriented body envelope check
- yaw-pivot risk near walls
- peer robot prediction
- leg swing / scuff margin approximation
- low obstacle / pillar special case
- stuck and no-progress detector

Existing repo components:

- `path_safety_filter`
- `cmd_vel_safety_shield`
- CFPA2 pivot-lock
- waypoint watchdog
- contact monitoring

The proposed research contribution is not merely to add a safety patch. The important part is that safety rejection becomes feedback:

```text
safety_rejection(reason, pose, frontier) -> failure_memory -> future utility penalty
```

### 7.9 Failure Memory

Each failed execution updates a persistent region-level memory:

```text
failure = {
  frontier_id,
  robot_id,
  failure_type,
  pose,
  timestamp,
  local_map_patch,
  safety_status,
  contact_status,
  slam_health,
  retry_count
}
```

Failure types:

- `planner_no_path`
- `local_planner_stall`
- `cmd_vel_rejected`
- `contact`
- `leg_scuff`
- `tip_risk`
- `map_conflict`
- `low_lidar_confidence`
- `slam_degraded`
- `peer_blockage`

This feeds:

```text
PredictedExecutionFailure(i, f)
```

Initial version can be rule-based. Later version can be learned from MuJoCo rollouts.

---

## 8. Learning Component

The proposal should not begin as pure MARL. That would increase risk and reduce interpretability.

Recommended learning module:

### Supervised Frontier Failure Predictor

Use MuJoCo rollouts to train:

```text
P(success, contact, stuck, timeout, drift | local_map_patch, robot_type, frontier_features, path_features)
```

Inputs:

- local occupancy patch
- local point density / confidence patch
- robot type
- planned path length / curvature / clearance
- frontier trust features
- recent failure memory
- SLAM health

Outputs:

- success probability
- contact risk
- stuck risk
- timeout risk
- validation required probability

Training data:

- generate many rollouts in randomized MuJoCo scenes
- labels from `session_reporter`, contact monitor, watchdog, safety shield
- use Go2 and Go2W separately to learn morphology and sensor-specific failure modes

Deployment:

```text
PredictedExecutionFailure(i, f) = model(local_features, robot_i, frontier_f)
```

This gives a CoRL-style learning angle while keeping the system grounded in classical robotics.

---

## 9. Implementation Plan in This Repo

### 9.1 New Modules

Recommended package:

```text
src/exploration/asymmetric_collaborative_exploration/
```

Core nodes:

```text
sensor_trust_node.py
trust_map_fusion_node.py
frontier_trust_annotator.py
asymmetric_frontier_allocator.py
active_validation_manager.py
failure_memory_node.py
```

If keeping scope smaller, start with:

```text
asymmetric_frontier_allocator.py
trust_map_fusion_node.py
failure_memory_node.py
```

### 9.2 Integration Points

Existing systems to reuse:

- CFPA2 frontier detection and allocation as initial baseline.
- TARE frontiers / waypoints as alternative source.
- FAR / localPlanner / pathFollower for execution.
- `path_safety_filter` and `cmd_vel_safety_shield` for safety feedback.
- MuJoCo contact bridge for labels.
- `session_reporter.py` for experiment metrics.

### 9.3 Minimal Viable Implementation

The fastest publishable prototype:

1. Add per-robot static sensor trust:

   ```text
   Go2 = high trust
   Go2W = low trust
   ```

2. Annotate frontiers by source and confidence.
3. Modify assignment utility to include sensing trust and mobility cost.
4. Add Go2 validation behaviour for low-confidence Go2W frontiers.
5. Feed safety rejections into failure memory.
6. Benchmark against naive dual CFPA2 / TARE / FAR variants.

This avoids needing a full probabilistic SLAM rewrite.

---

## 10. Experimental Design

### 10.1 Simulation Environments

Use MuJoCo as the main simulator because physical contact matters.

Recommended scenes:

| Scene | Purpose |
|---|---|
| Flat maze | Go2W speed advantage should matter |
| Narrow corridor / doorway | deadlock and safety constraints |
| Low obstacle / pillar scene | weak LiDAR false-free and scuff failures |
| Long symmetric corridor | SLAM drift and loop-closure weakness |
| Mixed rough-flat terrain | morphology-aware assignment |
| Conflicting-sensor scene | Go2W sees free, Go2 sees obstacle |
| Optional door/button scene | interactive frontier extension |

### 10.2 Real-Robot Experiments

Minimum real validation:

1. Go2-only mapping baseline.
2. Go2W-only mapping baseline.
3. Naive Go2 + Go2W exploration.
4. Proposed asymmetric exploration.

Recommended physical layouts:

- indoor corridor with occlusions
- narrow doorway / corner
- mixed open + cluttered room
- low obstacle that weaker LiDAR struggles with

Real experiments do not need to be huge. The key is to show the same failure mode and recovery mechanism observed in simulation.

### 10.3 Baselines

Core baselines:

| Baseline | Purpose |
|---|---|
| Go2-only | high-quality sensing but slower single robot |
| Go2W-only | fast robot with weak sensing |
| naive dual exploration | treats both maps and robots equally |
| distance-only assignment | classical frontier allocation baseline |
| information-gain-only assignment | active exploration baseline |
| mobility-aware only | tests whether sensing trust is necessary |
| sensing-aware only | tests whether mobility model is necessary |
| naive map merge | tests trust-aware fusion |
| proposed full method | full system |

Optional stronger baselines:

- TARE single robot
- M-TARE-style multi-robot approximation
- FAR + CFPA2 dual robot
- low-bandwidth position-only coordination
- COMRES-style topological frontier assignment if feasible

### 10.4 Ablations

| Ablation | What it tests |
|---|---|
| no sensor trust | all robot observations equal |
| no map conflict handling | conflicts do not trigger validation |
| no active validation | Go2 never validates Go2W frontiers |
| no failure memory | repeated failed frontiers not penalized |
| no safety feedback | safety rejection not used in future planning |
| no mobility model | Go2 and Go2W have same motion cost |
| no SLAM health | localization uncertainty ignored |

### 10.5 Metrics

Coverage and efficiency:

- `coverage_percent`
- `time_to_coverage`
- `coverage_per_meter`
- `coverage_per_joule` if energy proxy is available
- `duplicate_exploration_ratio`

Map quality:

- `occupancy_F1`
- `false_free_rate`
- `false_occupied_rate`
- `map_conflict_count`
- `trusted_map_area`

Safety and physical realism:

- `contact_count`
- `leg_scuff_count`
- `body_collision_count`
- `tip_over`
- `near_wall_yaw_pivot_count`
- `cmd_vel_rejection_count`
- `path_safety_rejection_count`

Planning robustness:

- `frontier_completion_rate`
- `repeated_failed_frontier_count`
- `stuck_events`
- `replan_count`
- `validation_actions`
- `validation_success_rate`

SLAM / localization:

- `estimated_drift`
- `loop_closure_events`
- `pose_graph_uncertainty`
- `map_merge_error`

System cost:

- `CPU_usage`
- `GPU_usage`
- `communication_bytes`
- `latency`

### 10.6 Statistical Protocol

Minimum for simulation:

```text
5 scenes x 5 methods x 10 trials = 250 runs
```

Better:

```text
5 scenes x 8 methods x 20 trials = 800 runs
```

For real robots:

```text
3 scenes x 4 methods x 3-5 trials
```

Real trials can be fewer if each is well instrumented and supported by video, logs, and clear failure cases.

---

## 11. Expected Results

The ideal result table should show:

1. Go2W-only is fast but has worse map quality and more failures.
2. Go2-only is safer and more accurate but slower.
3. Naive dual exploration improves coverage speed but increases map conflicts and unsafe decisions.
4. Mobility-aware only improves speed but still overtrusts weak sensing.
5. Sensing-aware only improves safety but underuses Go2W mobility.
6. Full method gives the best trade-off:
   - near-Go2 map reliability
   - much better coverage speed than Go2-only
   - fewer contacts than naive dual
   - fewer repeated frontier failures
   - lower false-free rate

The most convincing plot:

```text
x-axis: coverage percent
y-axis: contact / false-free / stuck events
```

The proposed method should shift the Pareto frontier:

```text
more coverage at the same safety level
or
same coverage with much fewer failures
```

---

## 12. Novelty Evaluation

### 12.1 Novelty Score

Estimated novelty:

```text
Medium-high to high
```

This depends strongly on execution quality.

### 12.2 Why It Is Novel

The strongest novel aspects are:

- explicit treatment of **sensing asymmetry** in collaborative quadruped exploration
- coupling of `sensor trust` with `frontier allocation`
- active validation by the high-quality sensor robot
- source-aware and confidence-weighted map fusion for exploration decisions
- joint sensing + mobility + safety utility
- real Go2 + Go2W validation with asymmetric LiDAR quality
- physical metrics beyond coverage, including contact and scuff

### 12.3 Why It Could Be Seen as Incremental

Reviewers may consider the idea incremental if:

- the method is only a weighted sum of hand-tuned costs
- there is no clear comparison to strong baselines
- the real robot section is weak
- map trust is not quantitatively evaluated
- the system does not show a failure mode that existing methods cannot handle

### 12.4 How to Strengthen the Paper

The paper becomes significantly stronger if it includes:

1. Quantitative Go2 vs Go2W LiDAR quality analysis.
2. Real examples where Go2W produces false-free / low-confidence maps.
3. Proof that naive map fusion makes bad assignments.
4. Active validation examples where Go2 prevents Go2W failure.
5. Ablation proving sensing + mobility is better than either alone.
6. Contact-aware metrics showing physical realism.
7. At least one real dual-robot experiment.

---

## 13. Conference Targeting

### 13.1 CoRL 2026

**Fit:** medium, only strong if the learning component is emphasized.  
**Official deadline:** CoRL 2026 lists paper submission deadline as **May 28, 2026 AoE**.  
**Reality:** too soon for a full paper unless most experiments are already available.

Good CoRL angle:

- supervised frontier failure predictor
- learned sensor trust / risk model
- sim-to-real failure prediction
- uncertainty-aware robot learning for exploration

Recommendation:

```text
Use CoRL 2026 only for workshop / preliminary version, unless a strong learned-risk module and experiments are ready immediately.
```

### 13.2 IROS 2027

**Fit:** high.  
**Official deadline found:** IEEE RAS event page lists IROS 2027 paper deadline as **March 1, 2027**.  
**Why suitable:** IROS values complete robotics systems, real-robot validation, and field-ready autonomy.

Best IROS framing:

```text
Sensing- and Mobility-Aware Exploration for Heterogeneous Quadruped Teams
```

Required evidence:

- strong system implementation
- complete MuJoCo benchmark
- real Go2 + Go2W trials
- clear baselines and ablations

This is the most realistic main target.

### 13.3 ICRA 2027

**Fit:** high, but more competitive.  
**Official event dates:** IEEE RAS page lists ICRA 2027 as **May 24-28, 2027, Seoul**.  
**Submission deadline:** not reliably visible from official page at the time of writing; historically ICRA deadlines are much earlier than the conference.

Best ICRA framing:

```text
Trust-Aware Collaborative Exploration with Asymmetric Sensing and Mobility
```

To be competitive, the paper needs:

- strong novelty beyond engineering integration
- rigorous metric design
- real robot data
- clean algorithmic formulation

### 13.4 RSS 2027

**Fit:** medium-high if method is clean and evidence is strong.  
**Risk:** RSS expects a sharp scientific idea, not only a large system.  

RSS-level version should emphasize:

- formal problem definition
- information-theoretic or probabilistic map trust formulation
- active validation policy
- strong evidence that sensor asymmetry changes the optimal exploration strategy

### 13.5 RA-L with ICRA / IROS Option

**Fit:** high.  
This is a practical route if the system is solid but the novelty is more engineering-systems than theory.

Good RA-L framing:

```text
A real-time ROS 2 system for trust-aware heterogeneous quadruped exploration, validated in MuJoCo and on real Go2 / Go2W robots.
```

### 13.6 TRO / Field Robotics

**Fit:** later extension.  
These are better targets after collecting larger-scale real-world trials.

TRO / Field Robotics version should include:

- extensive deployment logs
- detailed failure taxonomy
- benchmark release
- generalization across environments
- long-duration robustness

---

## 14. Six-Month Roadmap

### Month 1: Sensor and Failure Characterization

Deliverables:

- quantify Go2 vs Go2W LiDAR differences
- record point density, range, dropout, map quality
- extend benchmark reporter for map quality and false-free metrics
- create first asymmetric-sensor MuJoCo scene

Success criterion:

```text
Demonstrate measurable sensing-quality gap between Go2 and Go2W.
```

### Month 2: Trust Map and Frontier Annotation

Deliverables:

- source-aware occupancy confidence
- frontier confidence annotation
- map conflict detection
- simple trust-weighted map fusion

Success criterion:

```text
Frontiers can be ranked by confidence and source robot.
```

### Month 3: Asymmetric Frontier Allocator

Deliverables:

- sensing + mobility utility
- robot-specific cost model
- assignment node connected to CFPA2 / TARE / FAR
- baseline comparison in simulation

Success criterion:

```text
Go2 and Go2W receive different roles naturally from utility, not hard-coded scripts.
```

### Month 4: Active Validation and Failure Memory

Deliverables:

- validation manager
- low-confidence frontier validation by Go2
- failure memory from stuck/contact/safety rejection
- first ablation plots

Success criterion:

```text
System prevents at least one repeatable Go2W weak-sensor failure case.
```

### Month 5: Real Robot Trials

Deliverables:

- real Go2 + Go2W trials in 2-3 scenes
- videos and logs
- hardware safety protocol
- real failure and recovery examples

Success criterion:

```text
Real dual-robot trial shows benefit over naive assignment.
```

### Month 6: Paper and Final Evaluation

Deliverables:

- final simulation benchmark
- real-robot result section
- ablation tables
- method diagrams
- paper draft
- supplementary video

Success criterion:

```text
Ready for IROS 2027 / ICRA 2027 submission cycle.
```

---

## 15. Key Risks and Mitigations

### Risk 1: Weighted utility looks too heuristic.

Mitigation:

- provide probabilistic interpretation of sensor confidence
- include ablation studies
- optionally learn weights from rollout data
- show qualitative failure cases that the heuristic captures

### Risk 2: Real robot trials are unstable.

Mitigation:

- keep real scenes small and controlled
- use simulation for statistical significance
- use real trials for validation of the core asymmetry claim
- prioritize safety and repeatable demonstrations

### Risk 3: Go2W weak LiDAR does not fail clearly enough.

Mitigation:

- design scenes that expose weak sensing: low obstacles, grazing-angle walls, reflective objects, narrow spaces
- artificially degrade Go2W LiDAR in simulation to match or amplify real limitations
- report sensor-quality measurements directly

### Risk 4: Method becomes too broad.

Mitigation:

- keep core paper focused on sensing + mobility asymmetry
- treat VLM, semantic search, and interactive door task as optional extensions
- do not add full MARL unless the core system is already strong

### Risk 5: Baselines are hard to reproduce.

Mitigation:

- use repo-native baselines first: Go2-only, Go2W-only, naive dual CFPA2/FAR/TARE
- add M-TARE-style and low-bandwidth baselines only if time allows
- clearly document configuration and random seeds

---

## 16. Paper Skeleton

### Abstract

Multi-robot exploration methods often assume homogeneous sensing and mobility, but real robot teams are frequently asymmetric. We present a trust-aware collaborative exploration system for heterogeneous quadruped teams consisting of a high-quality sensing Go2 and a faster but weakly sensing Go2W. The system models per-robot sensor confidence, fuses maps with source-aware trust, assigns frontiers using sensing- and mobility-aware utility, and actively dispatches the high-quality sensor robot to validate uncertain frontiers discovered by the weak-sensor teammate. A runtime safety layer feeds rejected paths and contact events back into persistent failure memory. In MuJoCo and real-robot experiments, the proposed system reduces false-free map errors, contact events, and repeated frontier failures while preserving the coverage advantage of the fast robot.

### Introduction

- motivate heterogeneous quadruped teams
- explain why Go2 + Go2W is not homogeneous
- show failure of naive map fusion and equal frontier assignment
- present thesis and contributions

### Related Work

- frontier exploration and active SLAM
- TARE / FAR / M-TARE
- low-bandwidth multi-robot exploration
- heterogeneous map fusion
- quadruped navigation and safety
- VLM semantic navigation as adjacent, not central

### Method

- problem formulation
- sensor confidence model
- trust-aware map fusion
- frontier trust graph
- assignment utility
- active validation policy
- runtime safety feedback and failure memory

### Experiments

- platform
- scenes
- baselines
- metrics
- simulation results
- real robot results
- ablations

### Discussion

- when sensing asymmetry helps
- when validation is worth the delay
- limitations
- future work

### Conclusion

- sensing asymmetry should be planned over
- trusted mapper + fast scout is a useful structure for heterogeneous quadruped teams

---

## 17. Minimal Publishable Claim

The minimum claim that should survive review:

> Modeling sensing asymmetry and map trust in heterogeneous quadruped exploration reduces unsafe frontier assignments and map-induced execution failures compared with naive multi-robot exploration, while preserving the coverage benefit of a faster weak-sensor teammate.

This claim is specific, testable, and aligned with the actual hardware.

---

## 18. Stronger Long-Term Claim

The stronger version for a top conference:

> Heterogeneous quadruped teams should assign exploration roles through a joint model of sensing quality, map trust, morphology-specific traversability, localization uncertainty, and runtime safety feedback. This produces a new class of collaborative exploration behaviour: trusted high-fidelity mappers actively validate uncertain regions, while fast weak-sensor robots exploit the trusted map for efficient coverage.

---

## 19. Recommended Next Action

The immediate next step should be experimental, not theoretical:

1. Quantify Go2 vs Go2W LiDAR quality in the same environment.
2. Construct one MuJoCo scene where Go2W's weak sensing causes a repeatable false-free or unsafe frontier.
3. Implement static trust-weighted frontier assignment.
4. Show Go2 validation prevents the Go2W failure.

This single demonstration will decide whether the topic is strong enough for IROS / ICRA.

---

## 20. References and Pointers

- TARE: A Hierarchical Framework for Efficiently Exploring Complex 3D Environments, RSS 2021. https://www.ri.cmu.edu/publications/tare-a-hierarchical-framework-for-efficiently-exploring-complex-3d-environments/
- FAR Planner: Fast, Attemptable Route Planner using Dynamic Visibility Update, IROS 2022. https://www.far-planner.com/far-planner
- Decentralized multi-robot exploration under low-bandwidth communications, Autonomous Robots 2026. https://link.springer.com/article/10.1007/s10514-025-10234-3
- Efficient Multi-robot Active SLAM, Journal of Intelligent & Robotic Systems 2025. https://link.springer.com/article/10.1007/s10846-025-02275-8
- Learning-Based Multi-Robot Active SLAM: A Conceptual Framework and Survey, Applied Sciences 2026. https://www.mdpi.com/2076-3417/16/3/1412
- Co-NavGPT: Multi-Robot Cooperative Visual Semantic Navigation Using Vision Language Models. https://arxiv.org/abs/2310.07937
- COMRES-VLM: Coordinated Multi-Robot Exploration and Search using Vision Language Models. https://arxiv.org/abs/2509.26324
- A Review on Map-Merging Methods for Typical Map Types in Multiple-Ground-Robot SLAM Solutions, Sensors 2020. https://pmc.ncbi.nlm.nih.gov/articles/PMC7730201/
- CoRL 2026 Call for Papers. https://www.corl.org/contributions/call-for-papers
- IROS 2027 IEEE RAS deadline page. https://www.ieee-ras.org/event/call-for-papers-paper-submission-deadline-iros-2027-ieee-rsj-international-conference-on-intelligent-robots-and-systems-iros-27403-0/
- ICRA 2027 IEEE RAS event page. https://www.ieee-ras.org/event/2027-ieee-international-conference-on-robotics-and-automation-icra-64212/
