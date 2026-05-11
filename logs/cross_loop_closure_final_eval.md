# Cross-Loop Runtime Validation

BLOCKED_VALIDATION:
  validation_name: cross_loop_closure_runtime_validation
  blocked_command: TOPIC_WAIT_SEC=3 OVERLAP_DURATION_SEC=3 NO_OVERLAP_DURATION_SEC=3 TEAM_POSE_GRAPH_BACKEND=g2o_export_only TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE=false USE_DYNAMIC_FILTER=true timeout 75s bash scripts/bench/run_cross_loop_runtime_validation.sh
  blocker_type: ros_runtime
  exact_error: ros_participant_socket_permission_denied; ROS2 DDS participant could not create UDP sockets in this sandbox, so no keyframes/candidates/matches were produced.
  current_status: Status D
  claim_allowed: Static/unit DiSCo-style contracts pass; descriptor-only and single weak match do not open merged map; no GT runtime path is used.
  claim_not_allowed: Live cross-robot loop closure, optimized PGO, safety-gated merged map runtime, simulation pass, real robot pass, or Status A.

- overlap_pass: `false`
- no_overlap_pass: `false`
- robust_inlier_set_size: `0`
- pose_graph_inter_robot_factors: `0`
- merged_map_enabled_time_sec: `null`
- gt_used_runtime: `false`
