# Real Hybrid ROS1 SLAM / ROS2 Nav Validation

- schema: `real_hybrid_ros1_slam_ros2_nav_validation/v1`
- deployment_mode: `real_hybrid_ros1_slam_ros2_nav`
- pass: `False`
- final_status: `Status D — External Blocker`
- blocker: `ROS1 Noetic not available;catkin_make/catkin not available;rospack not available;LiDAR driver package not available in ROS2 environment;LiDAR topic /livox/lidar not available;IMU topic /livox/imu not available;Unitree topic /sportmodestate not available;peer robot unreachable or PEER_ROBOT_IP not set;DDS/bridge communication not observed;Swarm-LIO2: ros1_noetic_not_available;Dynamic-LIO: ros1_noetic_not_available;ERASOR: ros1_noetic_not_available`
- claim: `Fast-LIO remains production backend on real robot; real hybrid Swarm-LIO2 primary is not validated.`
- implemented_scaffolding: `True`
- real_robot_available: `False`
- docker_run_blocked: `False`
