# Safety-Gated Map Merge

`/merged_map` must remain closed until the alignment gate accepts robust evidence.

## Gate Conditions

The gate accepts only when all conditions are true:

- robust loop selector accepted
- robust inlier set is stronger than a single weak match
- team pose graph has accepted inter-robot factors
- relative transform is finite
- no-overlap rejection has passed
- `gt_used_runtime=false`

Descriptor-only candidates, single weak ICP matches, export-only pose graph output, and optional local SLAM mutual state are insufficient.

## Runtime Surface

`relative_transform_manager_node` publishes:

- `/team_slam/relative_transform`
- `/team_slam/alignment_status`
- `/team_slam/local/status`

`discovered_map_merge_bootstrap_node` waits for `status=aligned` before writing map-merge parameters. The generated parameters are derived from discovered alignment, not MuJoCo ground truth or hardcoded `robot_a_to_robot_b`.

## Current Runtime Result

The safety gate passed in Point-LIO-primary simulation hardening. Across three overlap trials, `/merged_map` opened only after robust selection accepted, the team pose graph had accepted inter-robot factors, the relative transform was finite, paired no-overlap rejection had passed, and `gt_used_runtime=false`.

Across three no-overlap trials, `/merged_map` stayed closed and inter-robot pose graph factors remained zero. Descriptor-only candidates and a single weak match remain insufficient to open map merge.
