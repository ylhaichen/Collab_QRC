# SLAM Backend Comparison

- default_slam_backend: `fast_lio_scpgo`
- fast_lio_overlap_pass: `True`
- fast_lio_no_overlap_pass: `True`
- swarm_lio2_shadow_blocker: `Swarm-LIO2 source exists under external/Swarm-LIO2, but upstream packages are ROS1/catkin and this host does not expose catkin_make/rospack; shadow odometry cannot be validated`
- swarm_lio2_primary_blocker: `Swarm-LIO2 source exists under external/Swarm-LIO2, but upstream packages are ROS1/catkin and this host does not expose catkin_make/rospack; primary mode cannot be validated`
- dynamic_filter_backend: `temporal_voxel_fallback`
- erasor_blocker: `ERASOR source exists under external/ERASOR, but upstream package is ROS1/catkin and this host does not expose catkin_make/rospack; asynchronous cleanup cannot be validated`

Final status: `Status C — External Blocker`.
