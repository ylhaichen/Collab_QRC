# Simulation Hardening Eval

- final_status_label: `Simulation Hardening Passed`
- confidence_complete: `True`
- point_lio_primary_simulation_runtime_passed: `True`
- overlap_multi_run_passed: `True`
- no_overlap_multi_run_passed: `True`
- merged_map_safety_gate_passed: `True`
- gt_used_runtime_any: `False`
- nav2_odom_tf_valid_all_trials: `True`
- team_loop_closure_keyframes_valid_all_trials: `True`
- dynamic_object_stress_passed: `True`
- decentralized_comm_stress_passed: `True`
- fast_lio_fallback_regression_passed: `True`
- visualized_demo_script_generated: `True`

## Claim Boundary

Simulation hardening only. Real robot validation and Status A are not claimed by this report.

## Backends

- registration_backend: `icp_2d`
- robust_selection_backend: `greedy_consistency_fallback`
- kiss_matcher_runtime_validated: `False`
- erasor_removert_runtime_validated: `False`

## Blockers

- none

## Overlap Trials

| trial | status | pass | inliers | factors | merged_map_sec | odom_hz | cloud_hz | nav2_tf |
|---:|---|---|---:|---:|---:|---:|---:|---|
| 1 | aligned | True | 12 | 7 | 31.11 | 5.875 | 6.441 | True |
| 2 | aligned | True | 7 | 7 | 36.926 | 6.678 | 6.841 | True |
| 3 | aligned | True | 45 | 45 | 29.17 | 8.022 | 8.253 | True |

## No-Overlap Trials

| trial | status | pass | false_alignment | factors | merged_map_sec | odom_hz | cloud_hz | nav2_tf |
|---:|---|---|---|---:|---:|---:|---:|---|
| 1 | rejected | True | False | 0 | None | 5.683 | 5.643 | True |
| 2 | rejected | True | False | 0 | None | 7.377 | 7.502 | True |
| 3 | rejected | True | False | 0 | None | 4.105 | 4.235 | True |
