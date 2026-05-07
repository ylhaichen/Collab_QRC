# ROS1 / ROS2 SLAM Bridge Topic Contract

- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- slam_backend: `swarm_lio2_shadow`
- pass: `False`
- ros1_topic_list_available: `True`
- ros2_topic_list_available: `True`
- ros1_missing_topics: `/robot_a/swarm_lio2_raw/Odometry,/robot_b/swarm_lio2_raw/Odometry,/robot_a/swarm_lio2_raw/cloud_static,/robot_b/swarm_lio2_raw/cloud_static,/robot_a/swarm_lio2_raw/cloud_map,/robot_b/swarm_lio2_raw/cloud_map`
- ros2_missing_topics: ``
- message_rates_nonzero: `False`
- frames_valid: `False`
- gt_used_runtime: `False`
- blocker: `missing_ros1_topics:/robot_a/swarm_lio2_raw/Odometry,/robot_b/swarm_lio2_raw/Odometry,/robot_a/swarm_lio2_raw/cloud_static,/robot_b/swarm_lio2_raw/cloud_static,/robot_a/swarm_lio2_raw/cloud_map,/robot_b/swarm_lio2_raw/cloud_map;/robot_a/swarm_lio2/Odometry:rate<0.1;/robot_b/swarm_lio2/Odometry:rate<0.1;/robot_a/swarm_lio2/cloud_static:rate<0.1;/robot_b/swarm_lio2/cloud_static:rate<0.1;/robot_a/swarm_lio2/cloud_map:rate<0.1;/robot_b/swarm_lio2/cloud_map:rate<0.1;/robot_a/swarm_lio2/Odometry:header.frame_id_empty;/robot_a/swarm_lio2/Odometry:child_frame_id_empty;/robot_b/swarm_lio2/Odometry:header.frame_id_empty;/robot_b/swarm_lio2/Odometry:child_frame_id_empty`
- recommended_next_action: `Start ROS1 hybrid SLAM container, ros1_bridge, ROS2 adapter, and sim sensor publishers; rerun this script.`
