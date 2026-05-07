# SLAM Backend Migration to Swarm-LIO2

## Implemented Scaffolding

`slam_backend` supports `fast_lio_scpgo`, `swarm_lio2_shadow`, and `swarm_lio2_primary`. `fast_lio_scpgo` remains the default production backend.

`deployment_mode` supports `sim_ros2`, `sim_hybrid_ros1_slam_ros2_nav`, `real_hybrid_ros1_slam_ros2_nav`, and `real_ros1_only_experimental`. `sim_ros2` remains the default deployment path.

The ROS2 adapter, launch args, configs, Docker/catkin bridge scaffolding, backend availability scripts, and validation summaries are implemented. Swarm-LIO2 primary is not a replacement claim until all required runtime validations pass.

## Topic Contract

Shadow mode publishes only isolated Swarm-LIO2 outputs:

- `/<ns>/swarm_lio2/Odometry`
- `/<ns>/swarm_lio2/cloud_static`
- `/<ns>/swarm_lio2/cloud_map`
- `/<ns>/swarm_lio2/mutual_state`
- `/<ns>/swarm_lio2/relative_transform`
- `/team_slam/swarm_lio2_metrics`

Primary candidate mode maps Swarm-LIO2 outputs into the existing ROS2 contract:

- `/<ns>/Odometry`
- `/<ns>/corrected_odom`
- `/<ns>/odom/nav`
- `/<ns>/cloud_registered_body`
- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`
- `/team_slam/swarm_lio2_relative_transform`
- `/tf`

## Mock / Synthetic Validation

Synthetic tests validate the ROS2-side adapter contract, Dynamic-LIO cloud forwarding contract, ERASOR cleanup topic/metrics contract, and Swarm-loop agreement gate math. These tests do not prove ROS1 backend runtime or real robot readiness.

## Docker Runtime Validation

Run on a Docker-enabled simulation host:

```bash
bash scripts/manual/run_sim_hybrid_full_validation.sh
```

Primary replacement still requires overlap, no-overlap, dynamic-object, ERASOR cleanup, Nav2 runtime, and loop-closure agreement validation.

## Real Robot Validation

Run on the robot/Jetson or validated field computer:

```bash
bash scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh --host
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_shadow_validation.sh
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh
```

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Docker run/catkin runtime is blocked in this host session.
- Real robot LiDAR/IMU/Unitree topics and peer network are unavailable here.
- Fast-LIO remains production backend.

Status labels:

- Status A: sim and real hybrid passed.
- Status B: sim hybrid passed, real hybrid blocked.
- Status C: Swarm-LIO2 shadow passed, primary blocked.
- Status D: external/runtime blocker.
