# Sim Hybrid ROS1 SLAM / ROS2 Nav Validation

- status: `Status D — External Blocker`
- docker_image_build_passed: `True`
- catkin_workspace_build_passed: `True`
- swarm_lio2_ros1_launch_smoke_passed: `True`
- shadow_pass: `False`
- primary_pass: `False`
- dynamic_lio_runtime_output_pass: `False`
- erasor_cleanup_output_pass: `False`
- fresh_fast_lio_baseline_rerun_passed: `False`
- blocker: `Swarm-LIO2 Docker image, ROS1 catkin workspace, and ROS1 launch smoke passed. Full sim_hybrid_ros1_slam_ros2_nav is still blocked: ROS1/ROS2 bridge odometry reception, Swarm-LIO2 primary Nav2/tf ownership, team_loop_closure keyframes, optimized overlap/no-overlap baseline rerun, Dynamic-LIO static/dynamic cloud runtime output, and ERASOR cleanup output were not validated in one completed runtime pass. Required fresh baseline rerun with START_BRIDGE=true was not completed after the script default was corrected to TEAM_POSE_GRAPH_BACKEND=auto and TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE=false. The escalated Docker/bridge rerun was rejected by the approval reviewer due usage limit; retry was deferred by the environment until 8:06 PM or explicit approval. The last completed rerun used g2o_export_only, so optimized overlap_pass is false and the required Fast-LIO baseline regression cannot be claimed fresh-passing in this pass.`
- claim: `Fast-LIO remains production backend; Swarm-LIO2 primary is not validated.`
