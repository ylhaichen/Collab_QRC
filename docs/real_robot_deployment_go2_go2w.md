# Real Robot Deployment: Go2 And Go2W

This branch is prepared for two Livox-equipped legged robots with unknown initial relative pose.

## Required Runtime Inputs

Per robot:

- Livox point cloud topic
- Livox IMU topic
- local SLAM odometry
- local registered cloud
- Nav2 odom/tf
- network reachability to peer

Point-LIO validation requires native ROS 1 backend build/run, ROS 1/ROS 2 bridge or equivalent topic transport, and nonzero-rate ROS 2 adapter topics.

## Safe Deployment Order

1. Run `fast_lio_scpgo` default and confirm baseline Nav2 odom/tf.
2. Run Point-LIO shadow mode and confirm native + adapter odometry rates.
3. Switch Point-LIO primary only after shadow validation passes.
4. Enable descriptor-first peer exchange.
5. Enable robust loop closure and keep `/merged_map` closed until safety gate accepts.
6. Enable asynchronous cleanup outside the odometry loop.

## Status Boundary

Without physical Go2/Go2W hardware, Livox/IMU drivers, network access, Docker permission, and ROS runtime logs, real robot validation remains blocked. Do not report Status A until those logs pass.

## Current Runtime Result

The latest real robot deployment check did not pass. `robot_a_network_reachable=false` and `robot_b_network_reachable=false`; required runtime topics `/livox/lidar`, `/livox/imu`, `/robot_a/Odometry`, `/robot_b/Odometry`, `/robot_a/odom/nav`, `/robot_b/odom/nav`, and `/team_slam/keyframes` were unavailable.

Point-LIO is validated as a simulation primary path, not as a real robot primary path. Fast-LIO / SC-PGO must remain the production-safe robot path until the real Go2/Go2W Point-LIO run and fallback regression both produce nonzero-rate odom/cloud/Nav2/tf evidence.
