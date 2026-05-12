# Dynamic Map Filtering And Cleanup

Dynamic handling is split into online filtering and asynchronous cleanup.

## Online Filtering

`dynamic_obstacle_filter_node` consumes `/<ns>/cloud_registered_body` and publishes:

- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`
- `/<ns>/dynamic_filter_metrics`
- `/team_slam/dynamic_filter_metrics`

`cloud_static` is used for Scan Context, registration, and team keyframe clouds. `cloud_dynamic` is intended for short-term local obstacle layers and TTL-based clearing.

## Cleanup

`map_cleanup_backend_node` wraps asynchronous cleanup backends:

- `none`
- `erasor`
- `removert`
- `temporal_voxel_fallback`

Required export contract:

- `pcds/`
- `dense_global_map.pcd`
- `poses_lidar2body.csv`
- `initial_naive_map.pcd`

Outputs:

- `/team_slam/cleaned_static_map`
- `/team_slam/removed_dynamic_points`
- `/team_slam/map_cleanup_metrics`

ERASOR/Removert are not run in the real-time odometry loop. If their source or runtime is unavailable, logs must record the blocker and the fallback must not be used to claim ERASOR/Removert success.

## Current Runtime Result

The online temporal voxel filter stress check passed as an explicitly labeled synthetic moving-cluster contract: moving points appeared in `cloud_dynamic`, the moving trace was excluded from `cloud_static`, static wall points remained stable, and the dynamic voxel cleared after TTL. The Point-LIO cross-loop runtime also reported dynamic filter metrics and static-cloud keyframe flow, so Scan Context and registration used the static cloud path rather than raw dynamic traces.

ERASOR source is present under `external/ERASOR`, but ERASOR runtime is not claimed ready. A ROS Noetic Docker build probe with ROS/PCL/OpenCV dependencies failed at `jsk_recognition_msgs/PolygonArray.h`. The validated cleanup backend for this branch is therefore `temporal_voxel_fallback`, which produced `logs/map_cleanup_output/cleaned_static_map.pcd` and `logs/map_cleanup_output/removed_dynamic_points.pcd` from the required export format without blocking odometry/Nav2.
