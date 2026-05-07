# Real Hybrid ROS1 SLAM / ROS2 Nav

`real_hybrid_ros1_slam_ros2_nav` keeps the ROS2/Humble high-level stack on the laptop or team computer while each robot runs the ROS1/Noetic SLAM layer onboard.

ROS1 onboard side:

- Unitree / Go2 / Go2W sensor interfaces.
- LiDAR and IMU drivers.
- Swarm-LIO2 shadow first, primary only after validation.
- Dynamic-LIO filtering wrapper or fallback.
- ERASOR asynchronous cleanup, never in the realtime control loop.

ROS2 high-level side:

- Nav2 and exploration allocator.
- `team_loop_closure`, `robust_loop_selector`, `team_pose_graph_node`.
- `relative_transform_manager` and `/merged_map` safety gate.
- Validation scripts and logs.

The real hybrid launch can include the existing `go2w_real_bringup` Nav2 stack with
`onboard_slam:=true`. In `swarm_lio2_primary`, the adapter republishes Swarm-LIO2
odometry to `/<ns>/odom/nav` so the existing real Nav2 contract does not need to
understand Swarm-LIO2 internal topics.

Key entrypoints:

- `scripts/real/real_autonomy.sh deployment_mode=real_hybrid_ros1_slam_ros2_nav slam_backend=swarm_lio2_shadow`
- `scripts/real/onboard_ros1_slam.sh slam_backend=swarm_lio2_shadow`
- `ros2 launch go2_gazebo_sim real_hybrid_ros1_slam_ros2_nav.launch.py`
- `bash scripts/deploy/check_real_hybrid_ros1_slam_ros2_nav.sh`

Current blocker on this host: ROS1 Noetic, `catkin_make`, and `rospack` are unavailable locally, and live real robot LiDAR/IMU/Unitree topics are not present. This is recorded as Status D, not a replacement pass.
