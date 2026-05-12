# Final System Architecture

This branch rebuilds the multi-robot SLAM stack around a modular, safety-gated architecture for two Livox-equipped legged robots, `robot_a` and `robot_b`.

## Runtime Shape

Local SLAM is selected by `local_slam_backend`:

- `fast_lio_scpgo`: default safe production path.
- `point_lio`: primary candidate through `point_lio_ros2_adapter_node`.
- `swarm_lio2_experimental`: optional consistency input only, not an alignment authority.

Cross-robot alignment authority is `team_loop_closure`:

1. `loop_keyframe_exporter_node` builds static-cloud keyframes and Scan Context descriptors.
2. `cross_robot_loop_matcher_node` retrieves cross-robot candidates and requires geometric verification.
3. `robust_loop_selector_node` applies PCM/GNC-style consistency selection.
4. `team_pose_graph_node` exports or optimizes accepted inter-robot factors.
5. `relative_transform_manager_node` opens alignment only when robust inliers, pose graph factors, finite transform, no-overlap rejection, and `gt_used_runtime=false` all pass.

Dynamic map handling uses `dynamic_scene_filter` online and `map_cleanup` asynchronously. Decentralized exchange uses descriptor-first DDS envelopes, with compact keyframe clouds sent only after a candidate or explicit request.

## Safety Defaults

The default configuration keeps `fast_lio_scpgo` as production. `/merged_map` remains closed unless the safety gate publishes `status=aligned`. Point-LIO, KISS-Matcher, ERASOR, and Removert are never claimed runtime-ready unless their build/run validation logs prove it.

## Runtime Validation Status

Current branch validation reached `Status B`, not Status A. Point-LIO shadow and primary simulation validation passed with native odometry on `/aft_mapped_to_init` and native cloud on `/cloud_registered_body`. The Point-LIO primary launch produced `/robot_a/Odometry`, `/robot_b/Odometry`, corrected odometry, `/odom/nav`, Nav2 odom/tf, and team keyframes without ground truth runtime alignment.

The DiSCo-style cross-robot runtime validation passed overlap and no-overlap scenes with `gtsam_cpp` pose graph optimization, robust inlier gating, and safety-gated `/merged_map`. Real Go2/Go2W validation is still blocked by missing robot network and Livox/IMU topics. Fast-LIO / SC-PGO cannot be demoted because the fallback regression did not produce nonzero odometry topics in the latest run.

## Primary Launch

```bash
ros2 launch team_loop_closure disco_dynamic_decentralized_team_slam.launch.py \
  local_slam_backend:=fast_lio_scpgo \
  registration_backend:=icp_2d \
  robust_selection_backend:=greedy_consistency_fallback \
  team_comm_mode:=dds
```

Target mode after validation:

```bash
ros2 launch team_loop_closure disco_dynamic_decentralized_team_slam.launch.py \
  local_slam_backend:=point_lio \
  registration_backend:=kiss_matcher \
  robust_selection_backend:=pcm \
  static_map_cleanup_backend:=erasor \
  team_comm_mode:=descriptor_only
```
