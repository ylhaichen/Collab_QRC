# DiSCo-Style Cross-Robot Loop Closure

`team_loop_closure` is the cross-robot alignment authority. It does not depend on Swarm-LIO2 mutual transform initialization, manually configured robot-to-robot transforms, or ground truth.

## Pipeline

```text
static keyframe cloud
  -> Scan Context descriptor
  -> descriptor exchange
  -> candidate retrieval
  -> KISS-Matcher / ICP verification
  -> PCM / GNC consistency selection
  -> team pose graph factor
  -> relative transform gate
```

Descriptor candidates are published to `/team_slam/cross_robot_candidates`. Geometrically verified matches are published to `/team_slam/cross_robot_matches`. Robust inliers are published to `/team_slam/robust_loop_inliers`.

## Registration Backends

- `kiss_matcher`: preferred target backend. If unavailable or not wired to a validated CLI/library, the result records a dependency blocker and falls back to `icp_2d`.
- `icp_2d`: self-contained fallback used for geometric verification.
- `gicp_optional`: optional mode that currently records a blocker and falls back to `icp_2d`.

A descriptor-only match is never accepted. Same-robot matching is disabled for cross-robot candidate retrieval.

## Robust Selection

Supported mode labels:

- `pcm`
- `gnc`
- `greedy_consistency_fallback`

The selector builds a consistency graph over verified inter-robot matches, selects the largest consistent set, trims spread outliers, and accepts only when inlier count, eligible inlier ratio, transform spread, yaw spread, median RMSE, and `gt_used_runtime=false` pass.

## Current Runtime Result

The Point-LIO-primary simulation hardening passed three overlap trials and three no-overlap trials with the current validated registration backend `icp_2d`; KISS-Matcher is still a preferred target backend but is not runtime-validated on this branch.

Overlap trials accepted `aligned` with robust inlier counts `12`, `7`, and `45`, and inter-robot pose graph factors `7`, `7`, and `45`. No-overlap trials were rejected with zero accepted inter-robot pose graph factors and no `/merged_map` opening. All six trials reported `gt_used_runtime=false`, nonzero Point-LIO odom/cloud rate, valid Nav2 odom/tf, and nonzero team keyframes.

Descriptor-only and single weak match merge blocking are covered by the robust loop selector contract tests and the repeated no-overlap runtime result.
