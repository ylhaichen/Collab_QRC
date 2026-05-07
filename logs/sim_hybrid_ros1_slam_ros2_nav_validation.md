# Sim Hybrid ROS1 SLAM / ROS2 Nav Validation

- schema: `sim_hybrid_ros1_slam_ros2_nav_validation/v1`
- deployment_mode: `sim_hybrid_ros1_slam_ros2_nav`
- pass: `False`
- final_status: `Status D — External Blocker`
- blocker: `ROS1 hybrid Docker image build passed, but container catkin build could not be executed because the escalated Docker compose run was rejected by approval reviewer: Automatic approval review failed: usage limit hit; try again after 6:46 PM or with explicit user approval. Swarm-LIO2/Dynamic-LIO/ERASOR source-level build and runtime are not validated.`
- docker_backend_build_blocker: `ROS1 hybrid Docker image build passed, but container catkin build could not be executed because the escalated Docker compose run was rejected by approval reviewer: Automatic approval review failed: usage limit hit; try again after 6:46 PM or with explicit user approval. Swarm-LIO2/Dynamic-LIO/ERASOR source-level build and runtime are not validated.`
- claim: `Fast-LIO remains production backend; sim hybrid Swarm-LIO2 primary is not validated.`
