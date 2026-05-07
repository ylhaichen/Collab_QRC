# Real Hybrid ROS1 SLAM / ROS2 Nav Check

- status: `Status D -- External Blocker`
- mode: `host`
- strategy: `host_noetic_catkin`
- pass: `false`
- runtime_ready: `false`
- ROS1 Noetic: `false`
- catkin: `false`
- rospack: `false`
- ROS2 Humble: `true`
- Docker daemon: `false`
- Docker run ready: `false`
- LiDAR topic /livox/lidar: `false`
- IMU topic /livox/imu: `false`
- Unitree topic /sportmodestate: `false`
- peer unset: `false`
- DDS/bridge observed: `false`
- blocker: `ROS1 Noetic not available;catkin_make/catkin not available;rospack not available;LiDAR driver package not available in ROS2 environment;LiDAR topic /livox/lidar not available;IMU topic /livox/imu not available;Unitree topic /sportmodestate not available;peer robot unreachable or PEER_ROBOT_IP not set;DDS/bridge communication not observed;Swarm-LIO2: ros1_noetic_not_available;Dynamic-LIO: ros1_noetic_not_available;ERASOR: ros1_noetic_not_available`
- recommended_next_action: `run on real robot or Jetson with ROS1 Noetic, live LiDAR/IMU/Unitree topics, peer network, and backend workspace`
