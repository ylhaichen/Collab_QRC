# Dynamic-LIO Filtering Integration

Dynamic-LIO must not publish primary odometry in this architecture.

The integration node `dynamic_lio_filtering_node` supports:

- `dynamic_lio_port`: reserved for a future source-level port into Swarm-LIO2 preprocessing/map update.
- `dynamic_lio_wrapper`: forwards `/<ns>/dynamic_lio/cloud_static` and `/<ns>/dynamic_lio/cloud_dynamic` into the project topic contract.
- `temporal_voxel_fallback`: uses the existing temporal voxel filter to publish `/<ns>/cloud_static`, `/<ns>/cloud_dynamic`, and `/team_slam/dynamic_filter_metrics`.
- `none`: disables the migration filtering path.

The required metrics fields are published with schema `team_dynamic_filter_metrics/v1`, including `dynamic_filter_backend`, point counts, fallback state, and blocker text.

Current blocker: Dynamic-LIO source is available under `external/dynamic_lio`, but the native filtering runtime artifact is not installed. Only the temporal voxel fallback is validated.
