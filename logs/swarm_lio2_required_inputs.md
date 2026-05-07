# Swarm-LIO2 Required Inputs

- mode: `host`
- lidar_type: `SIM=6`
- lidar_message_type: `sensor_msgs/PointCloud2`
- imu_message_type: `sensor_msgs/Imu`
- robot_a_lidar_input: `/quad1_pcl_render_node/sensor_cloud`
- robot_a_imu_input: `/quad_1/imu`
- robot_b_lidar_input: `/quad2_pcl_render_node/sensor_cloud`
- robot_b_imu_input: `/quad_2/imu`
- ros2_imu_source: `/robot_*/imu/data`
- ros2_lidar_raw_fallback_source: `/mujoco_sim/*/registered_scan`
- native_odom_outputs: `/quad1/lidar_slam/odom,/quad2/lidar_slam/odom`
- native_cloud_outputs: `/quad*/cloud_registered,/quad*/cloud_registered_body`
- runtime_inspection_available: `False`
- blocker: `rosnode list:exit=127;rosnode info /laserMapping_quad1:exit=127;rosnode info /laserMapping_quad2:exit=127;rostopic info /quad1_pcl_render_node/sensor_cloud:exit=127;rostopic info /quad2_pcl_render_node/sensor_cloud:exit=127;rostopic info /quad_1/imu:exit=127;rostopic info /quad_2/imu:exit=127`
- recommended_next_action: `Start ros1_hybrid_slam runtime and rerun with --docker.`
