# Swarm-LIO2 Loop Closure Agreement Gate

`team_loop_closure` remains the independent safety gate. Swarm-LIO2 mutual state alone never opens `/merged_map`.

## Implemented Scaffolding

In `swarm_lio2_primary`, cross-robot alignment is policy-driven:

- `cross_robot_alignment_source = team_loop_closure | swarm_lio2_mutual | hybrid`
- `swarm_agreement_mode = required | optional_if_available | disabled_for_debug`

Default sim-hybrid policy:

- `cross_robot_alignment_source:=team_loop_closure`
- `swarm_agreement_mode:=optional_if_available`

Agreement math (when evaluated):

- `translation_error = ||translation(T_swarm_a_b^-1 * T_loop_a_b)||`
- `yaw_error = yaw(T_swarm_a_b^-1 * T_loop_a_b)`

Default limits:

- `swarm_loop_agreement_max_translation = 0.5`
- `swarm_loop_agreement_max_yaw_deg = 5.0`

`relative_transform_manager_node` publishes `aligned` only after robust loop closure and pose graph acceptance.
When `swarm_agreement_mode=optional_if_available`, Swarm agreement is enforced only if `T_swarm_a_b` is available; otherwise it records `swarm_loop_agreement_status=optional_unavailable` and keeps `team_loop_closure` authoritative.

## Mock / Synthetic Validation

Synthetic tests validate:

- agreement accepts errors below threshold
- translation error above `0.5 m` rejects
- yaw error above `5 deg` rejects
- Swarm-LIO2 mutual state alone does not align the team map

## Docker Runtime Validation

Swarm-LIO2 Docker/catkin and ROS1 launch smoke are now available, but agreement-gated map merge still requires full hybrid runtime evidence:

```bash
bash scripts/manual/run_sim_hybrid_full_validation.sh
START_BRIDGE=true bash scripts/bench/run_cross_loop_runtime_validation.sh
```

## Real Robot Validation

Run only after shadow mode, peer communication, and real Nav2 odometry are valid:

```bash
CONFIRM_REAL_ROBOT=1 bash scripts/manual/run_real_robot_primary_validation.sh
```

## Current Blockers

- Current valid status is `Status C -- Shadow Passed, Primary Blocked`.
- Swarm-LIO2 local odometry/cloud and ROS2 primary adapter path are validated in sim-bridge shadow/primary runs.
- In the current MuJoCo scene, Swarm mutual/extrinsic payload remains empty (`teammate[]=[]`, `extrinsic[]=[]`), so `T_swarm_a_b` is unavailable.
- Under the new policy, this condition is treated as `optional_unavailable` (not a standalone merge-authority source), while map merge still depends on robust `team_loop_closure` acceptance and pose-graph checks.
- The latest primary rerun on this host is externally blocked by Docker permission (`docker_socket_permission_denied`), so no fresh full-runtime pass claim is made.
- Descriptor-only matches, weak single matches, ERASOR-only cleanup, Swarm-only mutual state, and runtime GT remain forbidden merge triggers.
