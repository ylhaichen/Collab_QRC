# SLAM Backend Migration to Swarm-LIO2

## Runtime Modes

`slam_backend` supports:

- `fast_lio_scpgo`: default validated production baseline.
- `swarm_lio2_shadow`: Fast-LIO / SC-PGO still drives Nav2 and `team_loop_closure`; Swarm-LIO2 adapter publishes isolated `/robot_*/swarm_lio2/*` metrics.
- `swarm_lio2_primary`: Swarm-LIO2 adapter owns the former Fast-LIO topic contract. This mode is not production-valid until all required runtime validations pass.

`deployment_mode` supports:

- `sim_ros2`: default ROS2 simulation path.
- `sim_hybrid_ros1_slam_ros2_nav`: ROS2 MuJoCo/Nav2 plus Dockerized ROS1/Noetic SLAM side.
- `real_hybrid_ros1_slam_ros2_nav`: onboard ROS1/Noetic SLAM side plus ROS2 high-level Nav2/team safety layer.
- `real_ros1_only_experimental`: onboard-only ROS1 SLAM bringup, not a production navigation mode.

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
- `/<ns>/odom/nav`
- `/<ns>/cloud_registered_body`
- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`
- `/team_slam/swarm_lio2_relative_transform`
- `/tf`

## Current Status

The ROS2 adapter, launch args, configs, hybrid bridge scaffolding, and validation scripts are implemented. External sources are present under `external/`, but ROS1/Noetic runtime build is blocked on this host. Fast-LIO remains the production backend.

Status labels now used by the migration report:

- Status A: sim and real hybrid passed.
- Status B: sim hybrid passed, real hybrid blocked.
- Status C: Swarm-LIO2 shadow passed, primary blocked.
- Status D: external/runtime blocker.
