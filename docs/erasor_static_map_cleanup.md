# ERASOR Static Map Cleanup

ERASOR is integrated as an asynchronous cleanup backend, not as part of the real-time odometry loop.

`erasor_adapter_node` subscribes to:

- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`

It exports:

- `pcds/`
- `dense_global_map.pcd`
- `poses_lidar2body.csv`
- `initial_naive_map.pcd`

It publishes:

- `/team_slam/cleaned_static_map`
- `/team_slam/erasor_removed_dynamic_cloud`
- `/team_slam/erasor_metrics`

`static_map_cleanup_backend` supports `none`, `erasor_wrapper`, and `temporal_voxel_fallback`. `erasor_trigger_mode` supports `manual`, `periodic`, and `benchmark`.

Current blocker: ERASOR source is available under `external/ERASOR`, but upstream package is ROS1/catkin and this host does not expose `catkin_make`/`rospack`. ERASOR cleanup cannot be validated yet.
