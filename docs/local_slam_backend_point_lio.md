# Local SLAM Backend: Point-LIO

Point-LIO is integrated as a primary candidate, not as the default production backend.

## Components

- `docker/point_lio/`: ROS 1 Noetic build image for upstream `hku-mars/Point-LIO`.
- `point_lio_ros2_adapter_node`: ROS 2 adapter that republishes native Point-LIO output into the existing Collab_QRC local SLAM contract.
- `scripts/setup/check_point_lio.sh`: availability check.
- `scripts/manual/run_point_lio_docker_build_and_test.sh`: Docker build/runtime validation.
- `scripts/bench/run_point_lio_shadow_validation.sh`: shadow topic-rate validation.
- `scripts/bench/run_point_lio_primary_validation.sh`: primary-mode local SLAM validation.

## Topic Contract

The adapter publishes:

- `/<ns>/Odometry`
- `/<ns>/corrected_odom`
- `/<ns>/odom/nav`
- `/<ns>/cloud_registered_body`
- `/<ns>/cloud_static`
- `/<ns>/cloud_dynamic`
- `/tf`

Native Point-LIO inputs remain under:

- `/<ns>/point_lio/Odometry`
- `/<ns>/point_lio/cloud_registered_body`
- `/<ns>/point_lio/cloud_static`
- `/<ns>/point_lio/cloud_dynamic`

## Validation Boundary

Static adapter contract tests do not prove Point-LIO runtime readiness. Point-LIO may become primary only after Docker build, Livox/IMU input, shadow odometry rate, adapter odometry rate, Nav2 odom/tf, and real robot validation pass without ground truth.

## Current Runtime Result

Point-LIO shadow and primary simulation validation passed. Native Point-LIO odometry was discovered as `/aft_mapped_to_init`; native registered cloud was discovered as `/cloud_registered_body`. The ROS 2 adapter published nonzero-rate shadow odometry under `/robot_a/point_lio/Odometry` and `/robot_b/point_lio/Odometry`, then primary-mode `/robot_a/Odometry`, `/robot_b/Odometry`, corrected odometry, `/odom/nav`, clouds, and TF.

The adapter publishes `map->odom` as an identity compatibility transform and `odom->base_link` / `odom->b_base_link` from Point-LIO odometry. An odometry jump guard rejects implausible Point-LIO pose jumps without using GT or a hardcoded inter-robot transform.

Fast-LIO / SC-PGO remains the real robot production-safe path until real robot Point-LIO validation and fallback regression both pass. The latest Fast-LIO fallback regression was blocked because `/robot_a/Odometry`, `/robot_b/Odometry`, `/robot_a/odom/nav`, `/robot_b/odom/nav`, and registered cloud topics did not become nonzero-rate before launch exit.
