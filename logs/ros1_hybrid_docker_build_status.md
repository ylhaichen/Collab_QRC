# ROS1 Hybrid Docker Build Status

- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- docker_image_build_success: `true`
- catkin_workspace_build_success: `true`
- ros1_launch_smoke_passed: `true`
- runtime_ready_scope: `Docker/catkin plus Swarm-LIO2 ROS1 wrapper launch smoke only`
- launch_smoke_log: `logs/manual/swarm_lio2_launch_smoke_20260507T193931Z.log`
- wrapper_topic_relays: `robot_a/robot_b LiDAR+IMU inputs and Swarm-LIO2 raw odom/cloud outputs`
- blocker: ``
- remaining_validation_blocker: `Swarm-LIO2 Docker image, ROS1 catkin workspace, and ROS1 launch smoke passed. Full sim_hybrid_ros1_slam_ros2_nav is still blocked: ROS1/ROS2 bridge topic contract has no nonzero Swarm-LIO2 odometry/cloud rates, primary Nav2/tf ownership and team_loop_closure keyframes are not validated, Dynamic-LIO static/dynamic cloud runtime output is not validated, and ERASOR cleanup output is not validated.`
