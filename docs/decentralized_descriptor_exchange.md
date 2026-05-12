# Decentralized Descriptor Exchange

The peer exchange is descriptor-first and bandwidth-aware for Jetson-class deployment.

## Always Exchanged

- `robot_id`
- timestamp
- keyframe id
- local pose
- Scan Context descriptor
- `ring_key`
- `sector_key`
- health metrics
- alignment status

## On Demand

- compact static keyframe cloud
- verified match summary
- robust inlier set
- pose graph factor summary

## Never Continuous

- raw LiDAR
- dense global map
- full costmap
- all point clouds

## Topics

Local topics:

- `/team_slam/local/keyframes`
- `/team_slam/local/descriptors`
- `/team_slam/local/status`
- `/team_slam/local/pose_graph_metrics`

Peer topics:

- `/team_slam/peer/keyframes`
- `/team_slam/peer/descriptors`
- `/team_slam/peer/status`
- `/team_slam/peer/pose_graph_metrics`

Cloud-on-demand:

- `/team_slam/cloud_request`
- `/team_slam/cloud_response`

Target deployment uses `team_comm_mode:=descriptor_only`, `peer_descriptor_rate_hz:=0.5`, `peer_cloud_max_points:=2000`, `peer_cloud_voxel_size:=0.4`, and `send_cloud_only_on_candidate:=true`.

## Current Runtime Result

The descriptor-first simulation stress check passed as an explicitly labeled synthetic contract for `team_comm_mode:=descriptor_only`: compact descriptors are exchanged in both directions, compact static keyframe cloud exchange is allowed only on candidate/request, bytes sent/received are recorded, and continuous raw LiDAR/dense map/full costmap exchange is blocked.

Peer loss/reconnect is recorded as not implemented in this simulation pass; it did not open `/merged_map`. Two-Jetson or equivalent physical peer-network validation is intentionally not claimed in this run. Do not claim decentralized real deployment bandwidth or reconnect behavior until `logs/decentralized_comm_validation.json` is produced from that network run.
