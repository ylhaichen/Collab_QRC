# Swarm-LIO2 Mutual State Debug

- mode: `docker`
- udp_bridge_running: `False`
- udp_bridge_required: `False`
- ros_direct_peer_subscription: `True`
- teammate_state_received: `True`
- teammate_array_length: `0`
- extrinsic_array_length: `0`
- quadstate_drone_ids: `[1, 2]`
- global_extrinsic_drone_ids: `[1, 2]`
- connected_teammate_ids: `{'quad1': [], 'quad2': []}`
- connected_teammate_num: `{'quad1': [0, 0, 0], 'quad2': [0, 0, 0]}`
- traj_matching_ids: `{'quad1': [], 'quad2': []}`
- mutual_observation_triggered: `False`
- global_extrinsic_initialized: `False`
- observation_topic_nonzero: `True`
- blocker: `mutual_observation_not_triggered`
- blocker_detail: `ROS direct peer state is present, but Swarm-LIO2 did not populate teammate[]; current sim did not trigger mutual observation / trajectory matching.`
