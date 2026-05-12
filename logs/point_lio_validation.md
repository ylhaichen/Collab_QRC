# Point-LIO Validation

- native_odom_topic: `/aft_mapped_to_init`
- native_cloud_topic: `/cloud_registered_body`
- shadow_validation_passed: `true`
- primary_validation_passed: `true`
- native_odometry_nonzero_rate: `true`
- native_cloud_nonzero_rate: `true`
- ros2_shadow_odometry_nonzero_rate: `true`
- ros2_primary_odometry_nonzero_rate: `true`
- nav2_odom_tf_validated: `true`
- team_loop_closure_keyframes_received: `true`
- gt_used_runtime: `false`
- real_robot_validation_passed: `false`

Point-LIO is runtime-valid in the local simulation path and as the selected primary local SLAM backend for the Point-LIO runtime validation. Real robot validation remains blocked by missing Go2/Go2W network and Livox/IMU topics.
