# Sim Hybrid ROS1 SLAM / ROS2 Nav

`sim_hybrid_ros1_slam_ros2_nav` keeps MuJoCo, Nav2, exploration, and `team_loop_closure` in ROS2/Humble while Swarm-LIO2, Dynamic-LIO, and ERASOR run in a Dockerized ROS1/Noetic catkin side.

## Implemented Scaffolding

- Default remains `deployment_mode:=sim_ros2` and `slam_backend:=fast_lio_scpgo`.
- ROS1 hybrid Docker files, bridge config, launch config, backend check scripts, and ROS2 adapter nodes are present.
- Swarm-LIO2 shadow mode remains isolated under `/<ns>/swarm_lio2/*`.
- Swarm-LIO2 primary mode is only a configured candidate topic contract, not a replacement claim.

## Mock / Synthetic Validation

- ROS2-side adapter tests cover synthetic Swarm-LIO2 odometry forwarding into `/<ns>/Odometry`, `/<ns>/corrected_odom`, and `/<ns>/odom/nav`.
- Dynamic-LIO wrapper tests cover static/dynamic cloud forwarding and metrics schema.
- ERASOR adapter tests cover synthetic PCD export, metrics JSON, and ROS2 cleanup topics.
- Swarm-loop agreement tests cover accept/reject threshold behavior and prevent Swarm mutual state alone from opening the merge gate.

## Docker Runtime Validation

Run on a host with Docker daemon permission:

```bash
bash scripts/manual/run_swarm_lio2_docker_build_and_test.sh
bash scripts/manual/run_dynamic_lio_docker_build_and_test.sh
bash scripts/manual/run_erasor_docker_build_and_test.sh
bash scripts/manual/run_sim_hybrid_full_validation.sh
```

For direct launch inspection:

```bash
ros2 launch go2_gazebo_sim sim_hybrid_ros1_slam_ros2_nav.launch.py --show-args
bash scripts/launch/ros1_hybrid_slam_bridge.sh mode=sim
bash scripts/bench/run_sim_hybrid_ros1_slam_ros2_nav_validation.sh
```

## Real Robot Validation

This mode is simulation-only. Real robot validation must use `real_hybrid_ros1_slam_ros2_nav` and must run on the robot/Jetson or validated field computer.

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Docker image build was previously recorded as passed, but Docker run/catkin runtime is still blocked in this host session.
- Fast-LIO remains the production backend until sim and real primary validations pass.
- Do not claim Swarm-LIO2 replacement from this runbook alone.
