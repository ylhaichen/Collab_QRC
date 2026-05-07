# ERASOR Map Cleanup Validation

- schema: `erasor_map_cleanup_validation/v4`
- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- static_map_cleanup_backend: `temporal_voxel_fallback`
- erasor_source_available: `True`
- erasor_buildable: `True`
- erasor_docker_catkin_build_passed: `True`
- erasor_runtime_ready: `False`
- naive_map_contains_dynamic_trace: `False`
- cleaned_map_removes_dynamic_trace: `False`
- static_walls_preserved: `False`
- cleaned_map_published: `False`
- control_loop_blocked: `False`
- fallback_used: `True`
- gt_used_runtime: `False`
- pass: `False`
- blocker: `missing_ros1_topics:/robot_a/swarm_lio2_raw/Odometry,/robot_b/swarm_lio2_raw/Odometry,/robot_a/swarm_lio2_raw/cloud_static,/robot_b/swarm_lio2_raw/cloud_static,/robot_a/swarm_lio2_raw/cloud_map,/robot_b/swarm_lio2_raw/cloud_map;/robot_a/swarm_lio2/Odometry:rate<0.1;/robot_b/swarm_lio2/Odometry:rate<0.1;/robot_a/swarm_lio2/cloud_static:rate<0.1;/robot_b/swarm_lio2/cloud_static:rate<0.1;/robot_a/swarm_lio2/cloud_map:rate<0.1;/robot_b/swarm_lio2/cloud_map:rate<0.1;/robot_a/swarm_lio2/Odometry:header.frame_id_empty;/robot_a/swarm_lio2/Odometry:child_frame_id_empty;/robot_b/swarm_lio2/Odometry:header.frame_id_empty;/robot_b/swarm_lio2/Odometry:child_frame_id_empty`
