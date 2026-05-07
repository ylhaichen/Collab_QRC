# Dynamic-LIO Filter Integration

- schema: `dynamic_lio_filter_integration/v4`
- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- dynamic_lio_source_available: `True`
- dynamic_lio_buildable: `True`
- dynamic_lio_docker_catkin_build_passed: `True`
- dynamic_lio_runtime_ready: `False`
- dynamic_filter_backend: `temporal_voxel_fallback`
- dynamic_points_filtered: `1`
- static_points_kept: `6`
- dynamic_filter_ratio: `0.14286`
- stale_obstacle_decay_time_sec: `1.5`
- temporal_voxel_fallback_passed: `True`
- fallback_used: `True`
- gt_used_runtime: `False`
- pass: `False`
- blocker: `missing_ros1_topics:/robot_a/swarm_lio2_raw/Odometry,/robot_b/swarm_lio2_raw/Odometry,/robot_a/swarm_lio2_raw/cloud_static,/robot_b/swarm_lio2_raw/cloud_static,/robot_a/swarm_lio2_raw/cloud_map,/robot_b/swarm_lio2_raw/cloud_map;/robot_a/swarm_lio2/Odometry:rate<0.1;/robot_b/swarm_lio2/Odometry:rate<0.1;/robot_a/swarm_lio2/cloud_static:rate<0.1;/robot_b/swarm_lio2/cloud_static:rate<0.1;/robot_a/swarm_lio2/cloud_map:rate<0.1;/robot_b/swarm_lio2/cloud_map:rate<0.1;/robot_a/swarm_lio2/Odometry:header.frame_id_empty;/robot_a/swarm_lio2/Odometry:child_frame_id_empty;/robot_b/swarm_lio2/Odometry:header.frame_id_empty;/robot_b/swarm_lio2/Odometry:child_frame_id_empty`
