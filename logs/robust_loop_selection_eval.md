# Robust Loop Selection Eval

- runtime_valid: `true`
- validation_type: `synthetic_contract_test`
- selection_backends_supported: `pcm`, `gnc`, `greedy_consistency_fallback`
- descriptor_only_rejected: `true`
- single_weak_match_rejected: `true`
- gt_used_runtime: `false`

Claim boundary: unit/contract validation only; live runtime remains controlled by `cross_loop_closure_final_eval`.
