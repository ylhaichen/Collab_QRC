# ROS1 Hybrid Docker Build Status

- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- docker_image_build_success: `true`
- catkin_workspace_build_success: `true`
- ros1_launch_smoke_passed: `true`
- runtime_ready_scope: `Docker/catkin plus Swarm-LIO2 ROS1 launch smoke only`
- blocker: ``
- remaining_validation_blocker: `Swarm-LIO2 Docker image, ROS1 catkin workspace, and ROS1 launch smoke passed. Full sim_hybrid_ros1_slam_ros2_nav is still blocked: ROS1/ROS2 bridge odometry reception, Swarm-LIO2 primary Nav2/tf ownership, team_loop_closure keyframes, optimized overlap/no-overlap baseline rerun, Dynamic-LIO static/dynamic cloud runtime output, and ERASOR cleanup output were not validated in one completed runtime pass.`
