# Real Robot Deployment Validation

- runtime_valid: `False`
- robot_a_network_reachable: `False`
- robot_b_network_reachable: `False`
- blocker_type: `network_unavailable`
- required_topics_missing: `['/livox/lidar', '/livox/imu', '/robot_a/Odometry', '/robot_b/Odometry', '/robot_a/odom/nav', '/robot_b/odom/nav', '/team_slam/keyframes']`
- required_topics_nonzero_rate: `[]`
- gt_used_runtime: `False`

BLOCKED_VALIDATION:
  validation_name: real_robot_deployment_go2_go2w
  blocked_command: bash scripts/deploy/check_real_robot_deployment_go2_go2w.sh
  blocker_type: network_unavailable
  current_status: Status B
  claim_not_allowed: Real robot validation, Status A, or Fast-LIO demotion on hardware.
