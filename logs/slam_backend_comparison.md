# SLAM Backend Comparison

- schema: `slam_backend_comparison/v3`
- default_deployment_mode: `sim_ros2`
- default_slam_backend: `fast_lio_scpgo`
- final_status: `Status D — External Blocker`
- production_backend: `fast_lio_scpgo`
- fast_lio_demotable: `False`
- swarm_lio2_docker_catkin_build: `True`
- swarm_lio2_ros1_launch_smoke: `True`
- swarm_lio2_primary_pass: `False`
- dynamic_lio_runtime_output_pass: `False`
- erasor_cleanup_output_pass: `False`
- real_hybrid_pass: `False`
- blocker: `Swarm-LIO2 Docker image, ROS1 catkin workspace, and ROS1 launch smoke passed. Full sim_hybrid_ros1_slam_ros2_nav is still blocked: ROS1/ROS2 bridge odometry reception, Swarm-LIO2 primary Nav2/tf ownership, team_loop_closure keyframes, optimized overlap/no-overlap baseline rerun, Dynamic-LIO static/dynamic cloud runtime output, and ERASOR cleanup output were not validated in one completed runtime pass. Required fresh baseline rerun with START_BRIDGE=true was not completed after the script default was corrected to TEAM_POSE_GRAPH_BACKEND=auto and TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE=false. The escalated Docker/bridge rerun was rejected by the approval reviewer due usage limit; retry was deferred by the environment until 8:06 PM or explicit approval. The last completed rerun used g2o_export_only, so optimized overlap_pass is false and the required Fast-LIO baseline regression cannot be claimed fresh-passing in this pass. ROS1 Noetic not available;catkin_make/catkin not available;rospack not available;LiDAR driver package not available in ROS2 environment;LiDAR topic /livox/lidar not available;IMU topic /livox/imu not available;Unitree topic /sportmodestate not available;peer robot unreachable or PEER_ROBOT_IP not set;DDS/bridge communication not observed;Swarm-LIO2: ros1_noetic_not_available;Dynamic-LIO: ros1_noetic_not_available;ERASOR: ros1_noetic_not_available`
- origin_push_policy: `origin skipped; fork-only push policy active.`
