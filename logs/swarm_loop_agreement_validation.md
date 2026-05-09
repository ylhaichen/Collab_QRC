# Swarm-Loop Agreement Validation

- source: `sim_bridge`
- native_mutual_topic_used: `/global_extrinsic_to_teammate`
- native_mutual_topic_rate_hz: `20.764`
- native_global_extrinsic_has_entries: `False`
- native_quadstate_has_teammate_entries: `False`
- ros2_swarm_relative_transform_rate_hz: `0.0`
- t_swarm_a_b_available: `False`
- t_loop_a_b_available: `False`
- swarm_loop_agreement_gate_pass: `False`
- swarm_loop_translation_error_m: `None`
- swarm_loop_yaw_error_deg: `None`
- overlap_pass: `False`
- no_overlap_pass: `False`
- merged_map_agreement_gated: `True`
- gt_used_runtime: `False`
- blocker: `/team_slam/swarm_lio2_relative_transform:rate<0.1;swarm_lio2_global_extrinsic_status_empty;swarm_lio2_quadstate_teammate_empty;overlap_alignment_not_accepted_with_swarm_agreement;no_overlap_scene_not_run_after_primary_blocker`
