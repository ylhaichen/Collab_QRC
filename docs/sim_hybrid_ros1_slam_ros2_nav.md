# Sim Hybrid ROS1 SLAM / ROS2 Nav

`sim_hybrid_ros1_slam_ros2_nav` keeps MuJoCo, Nav2, exploration, and `team_loop_closure` in ROS2/Humble while running Swarm-LIO2, Dynamic-LIO, and ERASOR in a Dockerized ROS1/Noetic catkin side.

Default safety posture:

- `deployment_mode:=sim_ros2` remains the project default.
- `slam_backend:=fast_lio_scpgo` remains the production baseline.
- `swarm_lio2_shadow` may run beside Fast-LIO without downstream ownership.
- `swarm_lio2_primary` is not a replacement claim until overlap, no-overlap, dynamic-object, ERASOR, Nav2 runtime, and Swarm agreement validations pass.

Key entrypoints:

- `ros2 launch go2_gazebo_sim sim_hybrid_ros1_slam_ros2_nav.launch.py`
- `bash scripts/launch/ros1_hybrid_slam_bridge.sh mode=sim`
- `bash scripts/bench/run_sim_hybrid_ros1_slam_ros2_nav_validation.sh`

Current blocker on this host: Docker is installed, but the current shell cannot access the Docker daemon. The external sources are present under `external/`, but the ROS1 catkin runtime is not built.
