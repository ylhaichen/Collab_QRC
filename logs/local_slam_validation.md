# Local SLAM Validation

BLOCKED_VALIDATION:
  validation_name: point_lio_primary_validation
  blocked_command: timeout 12s ros2 topic hz /robot_a/Odometry
  blocker_type: ros_runtime
  exact_error: 2026-05-12 00:19:43.722 [TRANSPORT_UDP Error] Error creating socket: Operation not permitted -> Function init; 2026-05-12 00:19:43.722 [RTPS_PARTICIPANT Error] User transport failed to register. -> Function RTPSParticipantImpl; WARNING: topic [/robot_a/Odometry] does not appear to be published yet
  current_status: Status D
  claim_allowed: Point-LIO primary launch/config exists and static adapter tests pass.
  claim_not_allowed: Point-LIO primary local SLAM, Nav2 odom/tf validity, or Status A.

Fast-LIO / SC-PGO remains production safe mode.
