# Dynamic-LIO Filtering Integration

Dynamic-LIO must not publish primary odometry. Swarm-LIO2 remains the primary SLAM candidate, while Dynamic-LIO contributes only static/dynamic point filtering.

## Implemented Scaffolding

`dynamic_lio_filtering_node` supports:

- `dynamic_lio_port`: reserved for source-level filtering port into Swarm-LIO2 preprocessing/map update.
- `dynamic_lio_wrapper`: forwards `/<ns>/dynamic_lio/cloud_static` and `/<ns>/dynamic_lio/cloud_dynamic`.
- `temporal_voxel_fallback`: publishes `/<ns>/cloud_static`, `/<ns>/cloud_dynamic`, and `/team_slam/dynamic_filter_metrics`.
- `none`: disables the migration filtering path.

Metrics use schema `team_dynamic_filter_metrics/v1` and include backend, point counts, fallback state, blocker, and `gt_used_runtime=false`.

## Mock / Synthetic Validation

Synthetic tests validate wrapper static/dynamic cloud forwarding and metrics contract without requiring ROS1 Dynamic-LIO runtime.

## Docker Runtime Validation

Run on a Docker-enabled simulation host:

```bash
bash scripts/manual/run_dynamic_lio_docker_build_and_test.sh
bash scripts/bench/run_dynamic_lio_filter_validation.sh
```

## Real Robot Validation

Run on the onboard host or Jetson after ROS1 Noetic/catkin is available:

```bash
bash scripts/setup/check_dynamic_lio.sh --host
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh
```

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Docker run/catkin runtime is blocked on this host.
- Real onboard ROS1 Noetic/catkin/rospack and live LiDAR/IMU topics are unavailable here.
- Until Dynamic-LIO runtime is built, only `temporal_voxel_fallback` can be used as validated fallback.
