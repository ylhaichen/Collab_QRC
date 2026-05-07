# Swarm-LIO2 Loop Closure Agreement Gate

`team_loop_closure` remains the independent safety gate. Swarm-LIO2 mutual state alone never opens `/merged_map`.

## Implemented Scaffolding

In `swarm_lio2_primary`, map merge requires agreement between:

- Swarm-LIO2 mutual transform: `T_swarm_a_b`
- robust loop closure transform: `T_loop_a_b`

The gate computes:

- `translation_error = ||translation(T_swarm_a_b^-1 * T_loop_a_b)||`
- `yaw_error = yaw(T_swarm_a_b^-1 * T_loop_a_b)`

Default limits:

- `swarm_loop_agreement_max_translation = 0.5`
- `swarm_loop_agreement_max_yaw_deg = 5.0`

`relative_transform_manager_node` publishes `aligned` only after robust loop closure, pose graph acceptance, and Swarm agreement pass.

## Mock / Synthetic Validation

Synthetic tests validate:

- agreement accepts errors below threshold
- translation error above `0.5 m` rejects
- yaw error above `5 deg` rejects
- Swarm-LIO2 mutual state alone does not align the team map

## Docker Runtime Validation

Run after Swarm-LIO2 Docker/catkin runtime is available:

```bash
bash scripts/manual/run_sim_hybrid_full_validation.sh
```

## Real Robot Validation

Run only after shadow mode, peer communication, and real Nav2 odometry are valid:

```bash
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh
```

## Current Blockers

- Current valid status is `Status D -- External Blocker`.
- Swarm-LIO2 runtime transform and team robust transform agreement have not been validated in sim or real runtime.
- Descriptor-only matches, weak single matches, ERASOR-only cleanup, Swarm-only mutual state, and runtime GT remain forbidden merge triggers.
