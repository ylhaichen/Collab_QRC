# Swarm-LIO2 Loop Closure Agreement Gate

`team_loop_closure` remains the independent safety gate. In `swarm_lio2_primary` mode, map merge requires agreement between:

- Swarm-LIO2 mutual transform: `T_swarm_a_b`
- robust loop closure transform: `T_loop_a_b`

The gate computes:

- `translation_error = ||translation(T_swarm_a_b^-1 * T_loop_a_b)||`
- `yaw_error = yaw(T_swarm_a_b^-1 * T_loop_a_b)`

Default limits:

- `swarm_loop_agreement_max_translation = 0.5`
- `swarm_loop_agreement_max_yaw_deg = 5.0`

`relative_transform_manager_node` only publishes `aligned` when robust loop closure, pose graph acceptance, and the Swarm agreement gate pass. Descriptor-only matches, weak single matches, Swarm mutual state alone, ERASOR cleaned maps alone, and runtime GT are not allowed to open `/merged_map`.
