# SLAM Backend Migration to Swarm-LIO2

## Runtime Modes

`slam_backend` supports:

- `fast_lio_scpgo`: default validated production baseline.
- `swarm_lio2_shadow`: Fast-LIO / SC-PGO still drives Nav2 and `team_loop_closure`; Swarm-LIO2 adapter publishes isolated `/robot_*/swarm_lio2/*` metrics.
- `swarm_lio2_primary`: Swarm-LIO2 adapter owns the former Fast-LIO topic contract. This mode is not production-valid until all required runtime validations pass.

## Topic Contract

Shadow mode publishes:

- `/<ns>/swarm_lio2/Odometry`
- `/<ns>/swarm_lio2/cloud_static`
- `/<ns>/swarm_lio2/cloud_map`
- `/<ns>/swarm_lio2/mutual_state`
- `/<ns>/swarm_lio2/relative_transform`
- `/team_slam/swarm_lio2_metrics`

Primary mode publishes:

- `/<ns>/Odometry`
- `/<ns>/corrected_odom`
- `/<ns>/cloud_registered_body`
- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`
- `/team_slam/swarm_lio2_relative_transform`
- `/tf`

## Current Status

The ROS2 adapter, launch args, and configs are implemented. External Swarm-LIO2 source/runtime is not available in this workspace, so `swarm_lio2_primary` is blocked and Fast-LIO remains the production backend.
