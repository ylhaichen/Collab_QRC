# Final System Validation

Final status label: **Status B - Point-LIO/DiSCo simulation core passed; real robot/network and fallback runtime blocked**

- status_a_claimed: `false`
- Point-LIO native odometry topic: `/aft_mapped_to_init`
- Point-LIO native cloud topic: `/cloud_registered_body`
- Point-LIO shadow validation: `passed`
- Point-LIO primary validation: `passed`
- Nav2 odom/tf: `passed`
- team_loop_closure keyframes: `passed`
- cross-loop overlap: `passed`, `aligned`, robust inliers `11`, inter-robot factors `11`
- cross-loop no-overlap: `passed`, `rejected`, inter-robot factors `0`
- registration backend runtime: `icp_2d`; KISS-Matcher runtime: `not_validated`
- robust selection backend runtime: `greedy_consistency_fallback`
- optimized pose graph backend: `gtsam_cpp`
- dynamic filtering: `synthetic_temporal_voxel_contract passed`; cross-loop runtime reported static-cloud metrics
- map cleanup: `temporal_voxel_fallback passed`; ERASOR runtime blocked by missing `jsk_recognition_msgs/PolygonArray.h`
- decentralized communication: `descriptor_only contract passed`; two-Jetson runtime not validated
- real robot validation: `blocked`, robot network/Livox/IMU topics unavailable
- Fast-LIO / SC-PGO regression: `blocked`; Fast-LIO cannot be demoted
- gt_used_runtime: `false`

BLOCKED_VALIDATION:
  validation_name: real_robot_deployment_go2_go2w
  blocked_command: bash scripts/deploy/check_real_robot_deployment_go2_go2w.sh
  blocker_type: network_unavailable
  exact_error: robot_a_network_reachable=false; robot_b_network_reachable=false; required Livox/IMU/Nav2/team topics missing.
  current_status: Status B
  claim_allowed: Point-LIO simulation primary, DiSCo-style cross-loop simulation runtime, temporal voxel fallback cleanup, descriptor-only communication contract.
  claim_not_allowed: Status A, real robot pass, two-Jetson communication pass, Fast-LIO demotion.

BLOCKED_VALIDATION:
  validation_name: fast_lio_fallback_regression
  blocked_command: bash scripts/bench/run_fast_lio_fallback_regression.sh
  blocker_type: ros_runtime
  exact_error: /robot_a/Odometry, /robot_b/Odometry, /robot_a/odom/nav, /robot_b/odom/nav, /robot_a/cloud_registered_body, and /robot_b/cloud_registered_body did not become nonzero-rate before launch exit.
  current_status: Status B
  claim_allowed: Point-LIO primary simulation path passed.
  claim_not_allowed: Fast-LIO demotion or fallback runtime-ready claim.

BLOCKED_VALIDATION:
  validation_name: erasor_runtime_cleanup
  blocked_command: docker run --rm -v "$PWD":/ws -w /ws ros:noetic-ros-base bash -lc 'apt-get update && apt-get install ROS1 deps && catkin_make -DCMAKE_BUILD_TYPE=Release'
  blocker_type: ros_runtime
  exact_error: fatal error: jsk_recognition_msgs/PolygonArray.h: No such file or directory
  current_status: Status B
  claim_allowed: temporal_voxel_fallback cleanup benchmark passed.
  claim_not_allowed: ERASOR runtime-ready claim.
