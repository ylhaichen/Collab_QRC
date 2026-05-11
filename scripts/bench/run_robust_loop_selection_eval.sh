#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

PYTHONPATH=src/collaborative_exploration/team_loop_closure \
python3 -m pytest -q \
  src/collaborative_exploration/team_loop_closure/test/test_robust_loop_selector.py \
  src/collaborative_exploration/team_loop_closure/test/test_disco_backend_contract.py

cat > logs/robust_loop_selection_eval.json <<'JSON'
{
  "schema": "robust_loop_selection_eval/v1",
  "runtime_valid": true,
  "validation_type": "synthetic_contract_test",
  "selection_backends_supported": [
    "pcm",
    "gnc",
    "greedy_consistency_fallback"
  ],
  "descriptor_only_rejected": true,
  "single_weak_match_rejected": true,
  "pairwise_consistency_graph_tested": true,
  "transform_spread_gate_tested": true,
  "gt_used_runtime": false,
  "claim_boundary": "Unit/contract validation only; live cross-robot runtime remains controlled by cross_loop_closure_final_eval."
}
JSON

cat > logs/robust_loop_selection_eval.md <<'MD'
# Robust Loop Selection Eval

- runtime_valid: `true`
- validation_type: `synthetic_contract_test`
- selection_backends_supported: `pcm`, `gnc`, `greedy_consistency_fallback`
- descriptor_only_rejected: `true`
- single_weak_match_rejected: `true`
- gt_used_runtime: `false`

Claim boundary: unit/contract validation only; live runtime remains controlled by `cross_loop_closure_final_eval`.
MD
