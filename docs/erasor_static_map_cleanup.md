# ERASOR Static Map Cleanup

ERASOR is integrated as an asynchronous cleanup backend, never inside the real-time odometry loop.

## Implemented Scaffolding

`erasor_adapter_node` subscribes to `/<ns>/cloud_static` and `/<ns>/cloud_dynamic`, exports PCD artifacts, and publishes:

- `/team_slam/cleaned_static_map`
- `/team_slam/erasor_removed_dynamic_cloud`
- `/team_slam/erasor_metrics`

Export artifacts are runtime products and must stay out of git tracking:

- `pcds/`
- `dense_global_map.pcd`
- `poses_lidar2body.csv`
- `initial_naive_map.pcd`
- `cleaned_static_map.pcd`
- `removed_dynamic_points.pcd`
- `erasor_metrics.json`

`static_map_cleanup_backend` supports `none`, `erasor_wrapper`, and `temporal_voxel_fallback`. `erasor_trigger_mode` supports `manual`, `periodic`, and `benchmark`.

## Mock / Synthetic Validation

Synthetic tests validate PCD export paths, metrics JSON, cleaned map topic, removed dynamic cloud topic, and `control_loop_blocked=false`.

## Docker Runtime Validation

Run on a Docker-enabled simulation host:

```bash
bash scripts/manual/run_erasor_docker_build_and_test.sh
bash scripts/bench/run_erasor_map_cleanup_validation.sh
```

## Real Robot Validation

Run after onboard ROS1 Noetic/catkin and map export paths are available:

```bash
bash scripts/setup/check_erasor.sh --host
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh
```

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Docker run/catkin runtime is blocked on this host.
- Real robot map cleanup cannot be marked passed without naive map, cleaned map, static wall preservation, and non-blocking runtime evidence.
