# Swarm-LIO2 Shadow Validation

- schema: `swarm_lio2_shadow_validation/v4`
- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- slam_backend: `swarm_lio2_shadow`
- swarm_lio2_source_available: `True`
- swarm_lio2_buildable: `True`
- swarm_lio2_runtime_ready: `True`
- swarm_lio2_started: `True`
- ros1_launch_smoke_passed: `True`
- swarm_lio2_odometry_valid: `False`
- swarm_lio2_relative_state_valid: `False`
- ros2_receives_shadow_odometry: `False`
- fast_lio_baseline_still_runs: `True`
- production_downstream_depends_on_swarm: `False`
- metrics_recorded: `True`
- gt_used_runtime: `False`
- pass: `False`
- blocker: `missing_ros1_topics:/robot_a/swarm_lio2_raw/Odometry,/robot_b/swarm_lio2_raw/Odometry,/robot_a/swarm_lio2_raw/cloud_static,/robot_b/swarm_lio2_raw/cloud_static,/robot_a/swarm_lio2_raw/cloud_map,/robot_b/swarm_lio2_raw/cloud_map;/robot_a/swarm_lio2/Odometry:rate<0.1;/robot_b/swarm_lio2/Odometry:rate<0.1;/robot_a/swarm_lio2/cloud_static:rate<0.1;/robot_b/swarm_lio2/cloud_static:rate<0.1;/robot_a/swarm_lio2/cloud_map:rate<0.1;/robot_b/swarm_lio2/cloud_map:rate<0.1;/robot_a/swarm_lio2/Odometry:header.frame_id_empty;/robot_a/swarm_lio2/Odometry:child_frame_id_empty;/robot_b/swarm_lio2/Odometry:header.frame_id_empty;/robot_b/swarm_lio2/Odometry:child_frame_id_empty`
