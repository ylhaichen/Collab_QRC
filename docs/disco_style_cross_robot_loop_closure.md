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
