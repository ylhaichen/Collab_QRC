#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

PYTHONPATH=src/collaborative_exploration/go2_nav_algorithms \
python3 -m pytest -q src/collaborative_exploration/go2_nav_algorithms/test/test_exploration_allocator.py

cat > logs/exploration_allocator_eval.json <<'JSON'
{
  "schema": "exploration_allocator_eval/v1",
  "runtime_valid": true,
  "validation_type": "synthetic_contract_test",
  "goal_types": [
    "frontier_goal",
    "loop_closure_goal",
    "rendezvous_goal",
    "dynamic_cleanup_goal",
    "communication_recovery_goal"
  ],
  "peer_frame_goals_blocked_until_alignment": true,
  "dynamic_cleanup_gain_supported": true,
  "communication_recovery_gain_supported": true,
  "gt_used_runtime": false,
  "claim_boundary": "Utility policy contract validated; live exploration behavior requires ROS/Nav2 runtime validation."
}
JSON

cat > logs/exploration_allocator_eval.md <<'MD'
# Exploration Allocator Eval

- runtime_valid: `true`
- validation_type: `synthetic_contract_test`
- peer_frame_goals_blocked_until_alignment: `true`
- dynamic_cleanup_gain_supported: `true`
- communication_recovery_gain_supported: `true`
- gt_used_runtime: `false`

Claim boundary: policy contract validated; live Nav2/exploration behavior still requires runtime validation.
MD
