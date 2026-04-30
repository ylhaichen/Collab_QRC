# Loop-Closure and Reconstruction-Aware Collaborative Exploration for Grounded Scene Reports

## Research Proposal

**Project:** Collab_QRC multi-robot quadruped collaborative exploration  
**Working title:** Loop-Closure and Reconstruction-Aware Collaborative Exploration for Grounded Scene Reports  
**Short title:** Grounded Scene Reports from Collaborative Quadruped Exploration  
**Date:** 2026-04-29  
**Platform:** Unitree Go2 + Unitree Go2W, ROS 2 Humble, MuJoCo, real-robot deployment  
**Sensor assumption:** both Go2 and Go2W carry Livox Mid-360 LiDAR + cameras  
**Compute:** RTX 4090, 4x A100, 2x RTX 4070  

---

## 1. Executive Summary

The project direction has changed because both robots will now carry Livox Mid-360 LiDAR. This removes the original main claim that Go2 is a strong-sensing robot while Go2W is a weak-sensing robot. The remaining and more robust heterogeneity is physical:

- **Go2** is legged-only, slower, and better suited for careful localization support, narrow-space validation, and stable loop-closure viewpoints.
- **Go2W** is wheeled-legged, faster on flat ground, but more vulnerable to wall proximity, pivot instability, residual drift, wheel scuff, and mode-switching failures.
- Both robots can produce high-quality LiDAR SLAM, but their **execution risk and best role in the exploration team differ**.

This proposal reframes the project around a stronger system-level question:

> Can a heterogeneous quadruped team actively decide when to explore new space, when to revisit loop-closure viewpoints, when to improve 3D reconstruction quality, and how to convert the resulting map into an evidence-grounded natural-language scene report?

The proposed system jointly optimizes:

```text
coverage gain
+ loop-closure / pose-graph gain
+ 3D reconstruction quality gain
+ semantic scene-report value
- travel cost
- robot-specific mobility risk
- contact / tip / stuck risk
```

The output is not only an occupancy map. The system produces:

- a 2D / 3D exploration map
- a pose graph with loop-closure evidence
- an accumulated 3D reconstruction
- a semantic 3D scene graph
- an evidence-grounded mission report with cited keyframes, map regions, robot poses, and uncertainty annotations

The thesis is:

> Collaborative quadruped exploration should be evaluated not only by area coverage, but by localization reliability, reconstruction completeness, physical safety, and the quality of grounded semantic reports produced from the explored scene.

This creates a publishable bridge between **multi-robot active SLAM**, **active 3D reconstruction**, **open-vocabulary 3D scene graphs**, and **VLM/LLM-based mission reporting**.

---

## 2. Why This Direction Fits the Current Hardware

### 2.1 Old Assumption

The previous proposal assumed:

```text
Go2  = strong LiDAR / trusted mapper
Go2W = weak native LiDAR / fast but low-trust scout
```

This supported a sensor-trust paper, but it becomes less compelling once both robots have Livox.

### 2.2 New Assumption

The new platform is:

```text
Go2  = Livox Mid-360 + legged-only mobility
Go2W = Livox Mid-360 + wheeled-legged mobility
```

Therefore, the interesting asymmetry is:

```text
same sensing, different embodiment
same SLAM backend, different execution risk
same reconstruction objective, different viewpoint cost
```

The new research problem is more robust because even identical sensors do not make the robots interchangeable. The Go2W may reach open frontiers faster, while Go2 may be safer for narrow loop closures, careful revisit poses, and validation around clutter.

---

## 3. Existing Repo Assets

The current `Collab_QRC` repository already provides a strong substrate:

- ROS 2 Humble multi-robot infrastructure.
- MuJoCo simulation with Go2 / Go2W assets.
- Real robot bringup for Go2 / Go2W.
- Livox Mid-360 integration.
- Fast-LIO2 LiDAR-inertial SLAM.
- SC-PGO / pose-graph correction path.
- FAR, TARE, CFPA2, and A* navigation / exploration backends.
- Contact, tip-over, stuck, SLAM drift, and coverage metrics via benchmark reporters.
- Dual-robot CFPA2 frontier allocation.
- Camera streams and VLM task infrastructure from `door_task` / `vlm_explorer`.
- Existing work on `frontier_trust`, `session_reporter`, and `collision.json` artifacts.

### 3.1 Current Repository Status After `origin/main` Merge

As of 2026-04-29, `origin/main` has been merged into the local branch. The upstream update is directly relevant to this new direction:

- `src/vendor/sc_pgo/fast_lio_sam/` is now present as a vendored pose-graph / loop-closure codebase. It is still not a finished ROS 2 Humble integration, but it gives a concrete implementation target for loop-closure experiments.
- `scripts/runtime/fast_lio_tf_adapter.py` already has logic to prefer corrected odometry when SC-PGO output becomes available.
- `src/go2w/go2w_config/config/nav/nav2_go2_full_stack.yaml` and `nav2_go2w_full_stack.yaml` add a more realistic Nav2 MPPI path for full-stack execution experiments.
- `scripts/runtime/stuck_watchdog.py`, `dual_robot_collision_monitor.py`, and `session_reporter.py` provide the safety and failure signals needed for mobility-risk modeling.
- The earlier `sensor_trust` implementation should now be treated as a legacy ablation, not the main claim, because both robots will mount Livox Mid-360.

The immediate engineering baseline should therefore be:

```text
Go2W / robot_a = Livox Mid-360 + Fast-LIO + Go2W mobility risk
Go2  / robot_b = Livox Mid-360 + Fast-LIO + Go2 mobility risk
sensor_trust_robot_a = 1.0
sensor_trust_robot_b = 1.0
```

Any performance difference after this point should be attributed to embodiment, controller behavior, loop-closure role allocation, reconstruction viewpoint cost, and physical execution risk, not LiDAR quality.

This proposal should avoid rebuilding autonomy from scratch. The research contribution should be implemented as a layer on top of the existing stack:

```text
Fast-LIO / SC-PGO
    -> pose graph health + loop closure candidates
    -> reconstruction quality map
    -> scene graph builder
    -> role-aware CFPA2 assignment
    -> grounded report generator
```

---

## 4. Research Problem

### 4.1 Problem Statement

Given a heterogeneous quadruped team with comparable LiDAR sensing but different locomotion capabilities, explore an unknown environment while producing a reliable, semantically meaningful, evidence-grounded scene report.

The system must decide:

1. Which robot should explore a new frontier?
2. Which robot should revisit a loop-closure viewpoint?
3. Which robot should improve low-quality reconstructed regions?
4. Which viewpoints are unsafe for Go2W but acceptable for Go2?
5. Which camera keyframes and 3D map regions support a natural-language report?
6. Which report statements are uncertain and require additional observations?

### 4.2 Why Coverage-Only Exploration Is Insufficient

Most exploration systems optimize:

```text
frontier utility = information gain - travel cost
```

This is insufficient for the target system because:

- high coverage can still produce a geometrically poor reconstruction
- high coverage can still produce a drifting or weakly constrained pose graph
- a robot may need to revisit known areas to reduce drift or close loops
- semantic summaries require object-level, room-level, and hazard-level evidence
- physical quadruped execution risk can dominate map-level path feasibility
- a good mission report must distinguish observed facts from inferred or uncertain claims

The proposed formulation is:

```text
collaborative exploration =
    active coverage
  + active loop closure
  + active 3D reconstruction
  + semantic scene graph construction
  + evidence-grounded report generation
  + morphology-aware safety and mobility risk
```

---

## 5. Literature Landscape and Novelty Boundary

### 5.1 Multi-Robot Active SLAM

Recent multi-robot active SLAM work already combines frontier sharing, pose graph uncertainty, and path entropy to balance exploration with mapping accuracy. For example, **Efficient Multi-robot Active SLAM** uses frontier management with pose-graph uncertainty and path entropy, with ROS validation in simulation and real-world experiments.

Source: https://link.springer.com/article/10.1007/s10846-025-02275-8

**Novelty boundary:** active SLAM and pose graph uncertainty are not new. The open gap is to combine active SLAM with reconstruction-aware viewpoint allocation, quadruped execution risk, and evidence-grounded scene reporting.

### 5.2 Active 3D Reconstruction with 3D Gaussian Splatting

**GS-Planner** introduced a 3DGS-based planning framework for active high-fidelity reconstruction, evaluating online reconstruction quality and completeness to guide robot data collection.

Source: https://arxiv.org/abs/2405.10142

**HGS-Planner** extended this direction with hierarchical global-local planning and reconstruction quality / completion gain. It was presented at ICRA 2025.

Source: https://arxiv.org/abs/2409.17624

**Multimodal LLM Guided Exploration and Active Mapping using Fisher Information** uses a 3DGS representation with multimodal LLM long-horizon planning and an information objective that considers localization uncertainty.

Source: https://arxiv.org/abs/2410.17422

**Novelty boundary:** active reconstruction with 3DGS is active and competitive. This project should not claim novelty in 3DGS reconstruction itself. The new contribution should be the integration of loop-closure assignment, robot-specific execution risk, multi-quadruped deployment, and grounded report generation.

### 5.3 Multi-Robot 3DGS Reconstruction

**Multi-robot autonomous 3D reconstruction using Gaussian splatting with Semantic guidance** proposes a centralized multi-robot 3DGS reconstruction framework, combining open-vocabulary semantic segmentation with surface uncertainty and multi-robot task assignment.

Source: https://arxiv.org/abs/2412.02249

**Novelty boundary:** multi-robot 3DGS reconstruction already exists. This project should not be framed as "first multi-robot 3DGS". The stronger gap is:

> Existing multi-robot reconstruction planners usually optimize view quality and planning efficiency, but do not deeply model quadruped locomotion risk, pose-graph loop-closure support, failure-memory feedback, and evidence-grounded mission reports.

### 5.4 Open-Vocabulary 3D Scene Graphs

**ConceptGraphs** builds open-vocabulary 3D scene graphs by fusing 2D foundation model outputs into 3D, creating object-centric scene representations useful for language-specified planning.

Source: https://arxiv.org/abs/2309.16650

**OVSG** supports context-aware entity grounding with open-vocabulary 3D scene graphs and free-form language queries.

Source: https://arxiv.org/abs/2309.15940

**HOV-SG** constructs hierarchical open-vocabulary 3D scene graphs with floor, room, and object levels for language-grounded robot navigation.

Source: https://arxiv.org/abs/2403.17846

**3DGraphLLM** combines semantic 3D scene graphs and LLMs for 3D scene understanding, showing that semantic relationships improve 3D vision-language tasks.

Source: https://arxiv.org/abs/2412.18450

**Novelty boundary:** 3D scene graphs + LLMs are not new. The gap is grounded, mission-level reporting from real multi-robot exploration logs, where every natural-language claim is linked to robot evidence, map uncertainty, reconstruction quality, and physical failure events.

### 5.5 VLM / LLM Robotics

VLMs can generate semantic summaries and high-level plans, but they are unreliable if asked to infer geometry from isolated images. The proposed system should therefore avoid raw-image-only summarization.

The correct architecture is:

```text
3D reconstruction + camera keyframes + robot trajectory + safety logs
    -> structured scene graph and evidence table
    -> VLM/LLM generates report with explicit citations
    -> verifier checks every claim against evidence
```

**Novelty boundary:** the system should not claim that LLMs can directly understand a 3D scene. The contribution is a grounded evidence pipeline that makes LLM reporting auditable.

---

## 6. Research Questions

### RQ1: Loop-Closure Role Allocation

When should a robot stop exploring new frontiers and instead revisit a loop-closure viewpoint?

Expected answer:

- If pose graph uncertainty or drift risk is high, revisiting a high-value loop closure can improve global map quality more than greedily exploring new space.
- Go2 may be better suited for careful loop-closure poses near narrow or cluttered areas.
- Go2W may be better suited for long open revisits if the path is flat and low-risk.

### RQ2: Reconstruction-Aware Exploration

Does adding reconstruction quality gain improve 3D scene completeness compared with coverage-only exploration?

Expected answer:

- Coverage-only exploration leaves sparse, single-view, or grazing-angle regions.
- Reconstruction-aware viewpoints improve geometric completeness and downstream scene graph quality.
- A proxy metric based on point density, view diversity, and incidence angle is enough for the first version.

### RQ3: Pose Graph Quality vs Coverage Trade-Off

Can the team improve localization reliability without sacrificing too much coverage?

Expected answer:

- Loop-closure-aware role allocation should reduce drift and map inconsistency.
- Some coverage speed is traded for better final reconstruction and report quality.
- A Pareto frontier should emerge between coverage and pose-graph quality.

### RQ4: Grounded Scene Reporting

Can a VLM/LLM generate useful exploration-area summaries if every claim is grounded in 3D scene graph nodes, keyframes, map regions, and robot logs?

Expected answer:

- Raw image-only summaries will hallucinate geometry and overstate confidence.
- Scene-graph-grounded summaries should be more accurate, more complete, and easier to verify.
- Evidence-linked reports are useful for downstream human operators.

### RQ5: Embodiment-Aware Viewpoint Assignment

Does robot-specific mobility risk improve the allocation of exploration, loop-closure, and reconstruction tasks?

Expected answer:

- Go2W should handle long, open, low-curvature routes.
- Go2 should handle tight loop closures, high-curvature validation viewpoints, and narrow reconstruction poses.
- Ignoring morphology should increase contact, stuck events, or failed viewpoints.

---

## 7. Hypotheses

### H1: Loop-closure-aware allocation improves map quality.

Compared with coverage-only CFPA2, the proposed method will reduce:

- global pose drift
- map inconsistency
- failed map merging cases
- false frontier generation from drifted maps

while maintaining acceptable coverage.

### H2: Reconstruction-aware allocation improves 3D scene quality.

Compared with area-coverage exploration, adding reconstruction-quality gain will improve:

- voxel / point cloud completeness
- surface point density
- multi-view coverage
- object-level scene graph recall
- offline 3DGS / mesh quality if used

### H3: Evidence-grounded scene reports are more reliable than image-only VLM summaries.

Compared with VLM summaries from raw camera frames, scene-graph-grounded reports will have:

- fewer unsupported claims
- better spatial correctness
- better uncertainty calibration
- more actionable next-step recommendations

### H4: Morphology-aware role allocation reduces physical failure.

Compared with treating Go2 and Go2W as interchangeable Livox robots, robot-specific mobility risk will reduce:

- contacts
- tip / degraded tilt
- planner stuck
- near-wall oscillation
- Go2W wheel scuff and pivot failures

---

## 8. Proposed System

### 8.1 High-Level Architecture

```text
Per-Robot Livox + Camera Streams
        |
Fast-LIO2 / SC-PGO
        |
Pose Graph Health Monitor
        |
Loop-Closure Candidate Generator
        |
3D Reconstruction Quality Map
        |
Semantic Scene Graph Builder
        |
Role-Aware CFPA2 Allocator
        |
Evidence-Grounded Report Generator
```

The allocator produces three classes of goals:

```text
Explore goals       -> expand unknown space
Loop-closure goals  -> improve pose graph / localization
Reconstruction goals-> improve surface / semantic completeness
```

### 8.2 Role Types

Each candidate goal has a role:

```text
role ∈ {explore, loop_close, reconstruct, validate, report_refine}
```

#### Explore

Standard frontier expansion. The objective is to maximize unknown-space coverage.

#### Loop Close

Revisit a location that creates a strong intra-robot or inter-robot loop-closure opportunity.

Candidate sources:

- robot trajectory crossing points
- previously visited nodes with high visual / LiDAR overlap potential
- areas near large pose graph uncertainty
- symmetric corridors where drift risk is high
- rendezvous regions between robot trajectories

#### Reconstruct

Visit a viewpoint that improves 3D scene quality.

Candidate sources:

- low point-density voxels
- high map uncertainty voxels
- single-view surfaces
- grazing-angle surfaces
- poorly observed object candidates
- report-critical unknown regions

#### Validate

Visit a viewpoint to confirm a hazard, obstacle, doorway, or uncertain semantic object.

#### Report Refine

Visit a viewpoint because the current mission report contains a low-confidence statement.

Example:

```text
"There may be a blocked doorway in the northeast room."
```

The system can generate a `report_refine` goal to collect a better camera view.

---

## 9. Candidate Utility

For robot `i` and candidate `c`:

```text
U(i, c) =
  w_cov   * CoverageGain(c)
+ w_loop  * LoopClosureGain(i, c)
+ w_rec   * ReconstructionGain(c)
+ w_sem   * SemanticReportGain(c)
- w_dist  * TravelCost(i, c)
- w_mob   * MobilityRisk(i, c)
- w_fail  * HistoricalFailureRisk(i, c)
- w_pose  * PoseUncertaintyCost(i, c)
- w_peer  * TeamInterferenceCost(i, c)
```

### 9.1 Coverage Gain

Can start with existing frontier `information_gain`.

### 9.2 Loop-Closure Gain

Pragmatic first version:

```text
LoopClosureGain = overlap_potential
                * pose_uncertainty_reduction_proxy
                * loop_candidate_age
                / travel_cost
```

Possible features:

- distance to previous trajectory node
- angle diversity relative to previous scan
- predicted LiDAR overlap
- current pose graph drift estimate
- SC-PGO correction magnitude
- time since last loop closure
- distance since last loop closure

### 9.3 Reconstruction Gain

First version should use geometry proxies, not online 3DGS:

```text
ReconstructionGain =
    unobserved_surface_score
  + low_density_voxel_score
  + single_view_penalty
  + semantic_object_uncertainty
```

Later version can use:

- TSDF uncertainty
- 3DGS Gaussian uncertainty
- rendered view error
- semantic instance uncertainty

### 9.4 Semantic Report Gain

A candidate has high report gain if it helps answer unresolved report questions:

```text
SemanticReportGain =
    unresolved_object_score
  + room_boundary_uncertainty
  + hazard_uncertainty
  + missing_keyframe_evidence
```

Example report questions:

- What rooms or corridors exist?
- What hazards were observed?
- Which regions are poorly reconstructed?
- Which areas remain unexplored?
- Which robot should continue exploration?

### 9.5 Mobility Risk

Use robot-specific risk:

```text
MobilityRisk(Go2W, c) high when:
  near wall
  narrow corridor
  high curvature route
  pivot-in-place required
  low obstacle / pillar nearby
  previous Go2W contact nearby

MobilityRisk(Go2, c) high when:
  long flat traversal
  time budget tight
  repeated slow-progress region
```

This keeps the equal-Livox platform scientifically meaningful.

---

## 10. 3D Reconstruction Layer

### 10.1 Level 1: Geometry-First Reconstruction

This should be the first implementation.

Inputs:

- `/robot_a/cloud_registered`
- `/robot_b/cloud_registered`
- Fast-LIO odometry
- corrected odometry if SC-PGO is available
- camera keyframes
- robot trajectories

Outputs:

- accumulated global point cloud
- voxel density map
- view count per voxel
- observed normal / incidence proxy
- reconstruction completeness score
- low-quality reconstruction regions

This is enough to implement reconstruction-aware planning without online neural rendering.

### 10.2 Level 2: Offline 3DGS Evaluation

After robot runs, use keyframes and poses to train or update a 3DGS model offline.

Use it for:

- visual report images
- reconstruction quality evaluation
- ablation against geometry-only reconstruction
- supplementary videos

Do not make online 3DGS required for the first paper. It increases engineering risk and competes directly with GS-Planner / HGS-Planner.

### 10.3 Level 3: Online 3DGS Planning

This is optional for a later paper.

If implemented, online 3DGS should be used only as an additional reconstruction quality estimator:

```text
Gaussian uncertainty -> reconstruction gain map -> candidate viewpoint utility
```

The main contribution should remain the collaborative quadruped system and evidence-grounded reporting.

---

## 11. Scene Graph Layer

### 11.1 Input Evidence

Each scene graph node should retain evidence:

```text
node = {
  node_id,
  label,
  type,
  3d_bbox,
  centroid,
  confidence,
  source_robot_ids,
  keyframe_ids,
  pointcloud_region_ids,
  first_seen_time,
  last_seen_time,
  uncertainty,
  report_relevance
}
```

Node types:

```text
room
corridor
doorway
obstacle
hazard
frontier
unexplored_region
low_quality_reconstruction_region
robot_failure_event
```

Edges:

```text
edge = {
  source,
  target,
  relation,
  confidence,
  evidence_ids
}
```

Relations:

```text
connected_to
inside
near
blocks
left_of / right_of
observed_from
reconstructed_by
unsafe_for
needs_revisit
```

### 11.2 Semantic Extraction

First implementation:

- use camera keyframe captions from VLM
- use object detection / segmentation where available
- project detections into 3D using robot pose and depth if available
- associate repeated observations into object nodes

Pragmatic VLM path:

```text
keyframe image + local map crop + robot pose
    -> VLM object/hazard/room caption
    -> structured JSON
    -> scene graph update
```

Do not let the VLM invent coordinates. Coordinates come from SLAM / projection.

---

## 12. Grounded Report Generator

### 12.1 Report Contract

The report generator outputs both Markdown and JSON:

```text
mission_summary.md
mission_summary.json
evidence_index.json
```

Every claim must have:

```json
{
  "claim": "The northeast corridor remains partially unexplored.",
  "type": "unexplored_region",
  "confidence": 0.78,
  "evidence": [
    "frontier_cluster_12",
    "map_region_ne_corridor",
    "keyframe_robot_b_0041"
  ],
  "uncertainty_reason": "Only one camera view and low voxel density behind doorway.",
  "recommended_action": "Send Go2 to loop-close near node 34, then send Go2W down the open corridor."
}
```

### 12.2 Report Sections

The report should contain:

1. Executive summary.
2. Explored topology.
3. Rooms / corridors / connected areas.
4. Important objects and landmarks.
5. Hazards and robot failure events.
6. Reconstruction quality.
7. Localization / loop-closure quality.
8. Unexplored or uncertain regions.
9. Recommended next actions.
10. Evidence appendix.

### 12.3 LLM Prompting Rule

The LLM receives only structured evidence:

```text
scene_graph.json
coverage_summary.json
pose_graph_health.json
reconstruction_quality.json
collision_report.json
selected_keyframe_captions.json
```

It should not receive raw unbounded image streams as the only context.

### 12.4 Verification

A `report_verifier` checks:

- each claim has evidence IDs
- each spatial statement references map / scene graph coordinates
- each hazard claim links to contact, image, or object evidence
- each uncertainty statement links to low coverage, low density, or missing views
- no claim refers to a non-existent node

Unsupported claims are either removed or marked as `unverified`.

---

## 13. Implementation Plan

### 13.1 New Modules

Recommended file layout:

```text
src/collaborative_exploration/
  reconstruction_awareness/
    reconstruction_quality_node.py
    loop_closure_candidate_node.py
    pose_graph_health_node.py
    scene_graph_builder_node.py
    grounded_report_generator.py
    report_verifier.py

scripts/bench/
  benchmark_loop_reconstruction_reports.sh
  reconstruction_reporter.py
  scene_graph_reporter.py
  report_quality_evaluator.py
```

### 13.2 Minimal First Implementation

Do not start with full 3DGS. Start with:

1. Accumulated point cloud / voxel density.
2. Pose graph health proxy.
3. Loop closure candidate scoring.
4. Reconstruction candidate scoring.
5. CFPA2 utility extension.
6. Offline scene graph from keyframes.
7. Evidence-grounded report generation.

### 13.3 Integration with Existing CFPA2

Add candidate types:

```text
frontier_candidate
loop_candidate
reconstruction_candidate
report_refine_candidate
```

Each candidate publishes:

```json
{
  "id": "candidate_17",
  "role": "loop_close",
  "x": 4.3,
  "y": -1.2,
  "coverage_gain": 0.1,
  "loop_gain": 0.8,
  "reconstruction_gain": 0.2,
  "semantic_report_gain": 0.1,
  "mobility_risk_go2": 0.2,
  "mobility_risk_go2w": 0.7
}
```

The CFPA2 allocator scores all candidates but can cap ratios:

```text
at least 60% explore goals before coverage reaches 70%
at most 30% loop-close goals unless drift is high
at most 30% reconstruction/report-refine goals unless coverage is already high
```

This prevents the system from spending all time polishing the map.

---

## 14. Evaluation Metrics

### 14.1 Exploration Metrics

- coverage ratio
- time to 50 / 70 / 90% coverage
- explored area per meter
- duplicate exploration ratio
- frontier completion rate

### 14.2 Localization Metrics

- SLAM drift vs ground truth in simulation
- pose graph correction magnitude
- loop closure count
- inter-robot loop closure count
- distance since last loop closure
- map consistency after PGO

### 14.3 Reconstruction Metrics

Geometry-first:

- voxel completeness
- mean point density
- low-density region count
- multi-view voxel ratio
- surface coverage proxy
- Chamfer / F-score if ground-truth mesh is available

3DGS optional:

- PSNR
- SSIM
- LPIPS
- rendering completeness
- novel-view quality

### 14.4 Semantic Scene Graph Metrics

- object / landmark recall
- room / corridor segmentation correctness
- relation correctness
- evidence completeness
- duplicate object merge rate
- unresolved semantic region count

### 14.5 Report Metrics

Automatic:

- number of claims
- percentage of evidence-linked claims
- unsupported claim rate
- uncertainty calibration
- next-action validity

Human evaluation:

- factual correctness
- spatial correctness
- usefulness to operator
- clarity of uncertainty
- usefulness of recommended next action

### 14.6 Physical Safety Metrics

- contact count
- wall / obstacle contact count
- tip-over
- degraded tilt
- stuck events
- planner failures
- Go2W mode switches
- near-wall pivot events

---

## 15. Baselines

### 15.1 Exploration Baselines

| Baseline | Purpose |
|---|---|
| CFPA2 coverage-only | current multi-robot baseline |
| CFPA2 + loop closure | tests localization term |
| CFPA2 + reconstruction | tests reconstruction term |
| CFPA2 + report-refine | tests report-driven perception |
| full proposed method | all terms combined |

### 15.2 Reconstruction Baselines

| Baseline | Purpose |
|---|---|
| passive reconstruction from coverage-only trajectories | default outcome |
| single robot reconstruction | measures multi-robot advantage |
| reconstruction-only NBV | tests whether reconstruction hurts coverage/safety |
| proposed coverage + loop + reconstruction | target method |

### 15.3 Reporting Baselines

| Baseline | Purpose |
|---|---|
| image-only VLM summary | tests hallucination / weak grounding |
| map-only textual summary | tests geometry without semantics |
| scene-graph-only LLM summary | tests structured semantics |
| proposed evidence-grounded report | target method |

---

## 16. Experimental Scenes

### 16.1 Simulation Scenes

Recommended MuJoCo scenes:

| Scene | Purpose |
|---|---|
| loop corridor | tests active loop closure |
| symmetric corridor | tests drift and ambiguity |
| multi-room indoor | tests report topology |
| pillar / occlusion maze | tests reconstruction holes |
| object-rich room | tests scene graph and VLM report |
| narrow doorway / corridor | tests Go2 vs Go2W mobility risk |
| long flat corridor | tests Go2W speed advantage |

### 16.2 Real Scenes

Start small:

1. lab corridor with a loop
2. two-room scene with doorway and obstacles
3. object-rich inspection area with shelves / boxes / chairs

The real experiments should validate:

- loop-closure behaviour
- reconstruction-quality improvement
- grounded report usefulness
- safe robot role allocation

They do not need to cover a huge area.

---

## 17. Roadmap

### Month 1: Reframe and Baseline

Deliverables:

- update proposal and thesis
- set both robots to Livox Mid-360 in simulation
- rerun Go2 vs Go2W with equal sensing
- characterize morphology-specific failures
- collect baseline coverage / drift / contact data

Success criterion:

- demonstrate that with equal sensors, robot type still changes failure profile and best role

### Month 2: Pose Graph Health and Loop-Closure Candidates

Deliverables:

- `pose_graph_health_node.py`
- `loop_closure_candidate_node.py`
- loop-closure candidate markers
- CFPA2 utility term for loop closure

Success criterion:

- robot can be assigned to revisit a useful loop-closure region
- drift or pose graph correction improves compared with coverage-only

### Month 3: Reconstruction Quality Map

Deliverables:

- accumulated point cloud quality map
- voxel density / view count / low-quality region detection
- reconstruction candidate generation
- benchmark reconstruction reporter

Success criterion:

- reconstruction-aware policy improves point density / completeness over coverage-only

### Month 4: Scene Graph and Evidence Index

Deliverables:

- camera keyframe logger
- object / landmark extraction pipeline
- initial 3D semantic scene graph
- evidence index linking claims to keyframes and map regions

Success criterion:

- scene graph can describe rooms, corridors, landmarks, hazards, and unexplored regions

### Month 5: Grounded Report Generator

Deliverables:

- `grounded_report_generator.py`
- `report_verifier.py`
- Markdown and JSON mission reports
- image-only vs scene-graph-grounded report ablation

Success criterion:

- generated reports have low unsupported-claim rate and useful next-action recommendations

### Month 6: Full Integrated Benchmark

Deliverables:

- coverage + loop + reconstruction + report integrated planner
- 3-5 simulation scenes
- 3-5 methods
- 5-10 trials per method
- preliminary paper figures

Success criterion:

- proposed method improves reconstruction/report quality while maintaining coverage and reducing physical failures

### Month 7-8: Real Robot Validation

Deliverables:

- real Go2 + Go2W Livox setup
- 2-3 controlled real scenes
- videos, logs, reports
- sim-to-real analysis

Success criterion:

- real trials show the same qualitative benefit: better loop closure, better scene report, safer role allocation

---

## 18. Expected Contributions

### Contribution 1: Multi-objective collaborative exploration.

A planner that allocates exploration, loop-closure, reconstruction, and report-refinement roles across a heterogeneous quadruped team.

### Contribution 2: Reconstruction-aware frontier assignment.

A practical reconstruction-quality map based on point density, view diversity, and semantic uncertainty, usable online before full neural reconstruction.

### Contribution 3: Loop-closure-aware role allocation.

A mechanism for assigning one robot to reduce localization uncertainty while the other continues coverage.

### Contribution 4: Evidence-grounded scene reports.

A report generator that converts multi-robot exploration logs, 3D reconstruction, camera keyframes, scene graph nodes, and failure events into auditable natural-language summaries.

### Contribution 5: Real quadruped evaluation.

Evaluation on Go2 and Go2W with identical Livox sensing, showing that same-sensor multi-robot exploration still requires morphology-aware planning.

---

## 19. Target Venues

### Strong targets

| Venue | Fit |
|---|---|
| ICRA | strongest fit for system + real robot + active mapping |
| IROS | strong fit, especially if implementation is broad |
| RA-L + ICRA/IROS option | good if method and experiments are mature |

### Ambitious targets

| Venue | Fit |
|---|---|
| RSS | possible if loop-closure/reconstruction planning has a clean algorithmic contribution |
| CoRL | possible if learned risk / report policy becomes central |

### Workshops

Good workshop targets before full paper:

- ICRA / IROS active SLAM workshop
- robot learning for semantic mapping workshop
- embodied AI / foundation models for robotics workshop
- field robotics / legged autonomy workshop

---

## 20. Paper Positioning

### What not to claim

Do not claim:

- first 3DGS active reconstruction
- first multi-robot 3DGS reconstruction
- first 3D scene graph for robots
- first LLM scene summary
- first active SLAM with loop closure
- first quadruped exploration system

### What to claim

Claim:

> We present a collaborative quadruped exploration system that jointly allocates coverage, loop-closure, reconstruction, and report-refinement goals, and produces evidence-grounded scene reports from 3D reconstruction, scene graphs, camera keyframes, pose graph health, and physical failure logs.

### One-sentence abstract

```text
We propose a multi-robot quadruped exploration system that treats localization support, 3D reconstruction quality, and grounded semantic reporting as first-class planning objectives, producing safer exploration and more reliable scene reports than coverage-only or image-only baselines.
```

---

## 21. Risks and Mitigations

### Risk 1: Online 3DGS is too heavy.

Mitigation:

- use geometry-first reconstruction quality for planning
- keep 3DGS offline for evaluation and visualization

### Risk 2: Loop closure signals are hard to expose from SC-PGO.

Mitigation:

- start with proxies: distance since last closure, drift estimate, revisit candidates, trajectory crossing
- add true loop closure events later

### Risk 3: VLM reports hallucinate.

Mitigation:

- evidence-only prompting
- JSON claim format
- report verifier
- unsupported claim metric

### Risk 4: Scene graph quality is poor in simulation.

Mitigation:

- start with simple labels: room, corridor, obstacle, hazard, unexplored
- object-level semantics can be added later

### Risk 5: Multi-objective planner becomes too complex.

Mitigation:

- implement one term at a time
- use ablations to prove each term
- keep CFPA2 assignment as the stable backbone

---

## 22. Immediate Next Steps

The first actionable sequence should be:

1. Update simulation so both Go2 and Go2W use Livox Mid-360.
2. Disable or neutralize old sensor-trust differences:

```text
sensor_trust_robot_a = 1.0
sensor_trust_robot_b = 1.0
```

3. Run morphology characterization:

```text
Go2 vs Go2W, same sensor, same scene, same planner
metrics = coverage, contacts, stuck, drift, distance, mode switches
```

4. Implement `pose_graph_health_node.py`.
5. Implement simple `loop_closure_candidate_node.py`.
6. Add `loop_gain` to CFPA2 utility.
7. Implement `reconstruction_quality_node.py` with voxel density.
8. Add `reconstruction_gain` to CFPA2 utility.
9. Add offline `mission_summary.md` generation from existing logs.
10. Only then add VLM-based keyframe semantics and 3D scene graph.

### 22.1 Immediate Code Checkpoint After Merge

The first implementation checkpoint should be small and measurable:

```text
Checkpoint A: equal-Livox substrate
  - MuJoCo LiDAR plugin prefers livox_mid360 over unitree_l1
  - robot_a Fast-LIO config uses pointlio_gazebo_mid360.yaml
  - robot_b Fast-LIO config uses pointlio_gazebo_mid360.yaml
  - CFPA2 sensor_trust_by_namespace = ["robot_a=1.00", "robot_b=1.00"]
  - launch default sensor_trust_robot_a = 1.00
```

Validation command:

```bash
./scripts/launch/nav_test_demo3_mixed.sh \
  slam_only:=true \
  gui:=false \
  rviz:=false \
  sensor_trust_robot_a:=1.00 \
  sensor_trust_robot_b:=1.00
```

Expected topics:

```text
/mujoco_sim/mujoco_lidar_sensor/registered_scan       frame=livox_mid360
/mujoco_sim/b_mujoco_lidar_sensor/registered_scan     frame=b_livox_mid360
/robot_a/velodyne_points                              frame=livox_mid360
/robot_b/velodyne_points                              frame=b_livox_mid360
/robot_a/Odometry
/robot_b/Odometry
```

Once this works for 60 s, the project should stop running L1-vs-Mid360 trust benchmarks by default and move to:

```text
same sensor, different embodiment:
  coverage
  drift / loop closure
  contact / stuck / tip
  reconstruction density
  report quality
```

---

## 23. Minimal Viable Paper

If scope must be reduced, the minimum publishable version is:

```text
coverage + loop closure + reconstruction quality + safety logs
```

with an offline report generator.

Minimum experiments:

- 3 simulation scenes
- 4 methods
- 5 trials per method
- 1 real scene

Methods:

1. coverage-only CFPA2
2. coverage + loop closure
3. coverage + reconstruction
4. full method

Metrics:

- coverage
- drift
- loop closure count
- reconstruction completeness
- contact / stuck / tip
- report supported-claim rate

This would already be a strong IROS / RA-L style paper if the real demo is clean.

---

## 24. Stronger Full Paper

The full version adds:

- semantic scene graph
- VLM keyframe captions
- evidence-grounded report generation
- report-refinement goals
- real Go2 + Go2W evaluation in multiple scenes
- optional offline 3DGS reconstruction visualization

This has a credible ICRA target if implemented cleanly.

---

## 25. Revised Project Thesis

The revised project should no longer be framed around weak vs strong LiDAR. The stronger thesis is:

> Same-sensor quadruped teams are still heterogeneous at the level that matters for deployment: locomotion, contact risk, localization support, and viewpoint cost. A useful collaborative exploration system must therefore allocate not just frontiers, but also loop-closure, reconstruction, and report-refinement roles, and must produce evidence-grounded scene summaries whose claims are traceable to robot observations.

---

## 26. Reference Boundary

These are the closest recent work clusters and the boundary they impose on novelty:

| Area | Representative work | Boundary for this project |
|---|---|---|
| Multi-robot active SLAM | Efficient Multi-robot Active SLAM, 2025. https://link.springer.com/article/10.1007/s10846-025-02275-8 | Pose-graph uncertainty and frontier utility are already known; novelty must include quadruped embodiment, loop-closure role allocation, reconstruction/report objectives, and real deployment evidence. |
| Active loop-closure exploration | Loop-Aware Exploration Graph, 2022. https://doi.org/10.1016/j.robot.2022.104179 | Active loop closure is not new; the contribution should be multi-robot role allocation with physical viewpoint cost. |
| Active 3DGS reconstruction | GS-Planner, 2024. https://arxiv.org/abs/2405.10142 | Do not claim novelty in 3DGS planning itself. Use reconstruction quality as one planning objective. |
| Hierarchical active reconstruction | HGS-Planner, ICRA 2025. https://arxiv.org/abs/2409.17624 | Completion/quality gain planning already exists; focus on collaborative quadruped execution and loop-closure coupling. |
| LLM-guided active mapping | Multimodal LLM Guided Exploration and Active Mapping using Fisher Information, 2024. https://arxiv.org/abs/2410.17422 | LLM semantic goal selection already exists; use LLMs for grounded reports and report-refinement goals rather than raw planner authority. |
| Multi-robot 3DGS | Multi-Robot Autonomous 3D Reconstruction using Gaussian Splatting with Semantic Guidance, 2024/2025. https://arxiv.org/abs/2412.02249 | Multi-robot 3DGS exists; avoid claiming "first multi-robot 3DGS". |
| Open-vocabulary 3D scene graphs | ConceptGraphs, ICRA 2024. https://concept-graphs.github.io/ | Open-vocabulary scene graphs exist; this project should emphasize evidence-grounded reporting from exploration logs. |
| Hierarchical scene graphs | HOV-SG, RSS 2024. https://hovsg.github.io/ | Room/floor/object graph hierarchy exists; use it as a design pattern, not as the main novelty. |
| Point-cloud scene graphs | Point2Graph. https://point2graph.github.io/ | Point-cloud-only scene graph generation is emerging; useful if camera calibration is weak, but not the central claim. |

The defensible paper claim is therefore:

> We do not introduce a new SLAM backend, a new 3DGS optimizer, or a new VLM. We introduce a real multi-quadruped exploration system that decides when to explore, loop-close, reconstruct, and refine a report, and evaluates the result using coverage, pose-graph health, reconstruction quality, physical safety, and evidence-linked report accuracy.
