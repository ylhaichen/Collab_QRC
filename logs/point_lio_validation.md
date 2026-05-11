# Point-LIO Validation

- docker_build_passed: `true`
- docker_smoke_test_passed: `true`
- shadow_validation_passed: `false`
- primary_validation_passed: `false`
- backend_runtime_ready: `false`
- gt_used_runtime: `false`

BLOCKED_VALIDATION:
  validation_name: point_lio_shadow_validation
  blocked_command: timeout 12s ros2 topic hz /robot_a/point_lio/Odometry
  blocker_type: ros_runtime
  exact_error: 2026-05-12 00:19:43.722 [TRANSPORT_UDP Error] Error creating socket: Operation not permitted -> Function init; 2026-05-12 00:19:43.722 [RTPS_PARTICIPANT Error] User transport failed to register. -> Function RTPSParticipantImpl; WARNING: topic [/robot_a/point_lio/Odometry] does not appear to be published yet
  current_status: Status D
  claim_allowed: Point-LIO Docker build and package discovery pass; ROS2 adapter/config/scripts and static contract tests pass.
  claim_not_allowed: Point-LIO shadow odometry nonzero-rate, Point-LIO primary local SLAM, Nav2 odom/tf validity, real robot validation, or Status A.
