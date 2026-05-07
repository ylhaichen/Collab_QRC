# Baseline Regression Rerun Status

- command: `START_BRIDGE=true bash scripts/bench/run_cross_loop_runtime_validation.sh`
- attempted: `true`
- passed: `false`
- existing_baseline_log_used: `true`
- existing_baseline_overlap_pass: `true`
- existing_baseline_no_overlap_pass: `true`
- existing_baseline_gt_used_runtime: `false`
- blocker: `Baseline rerun requires Docker bridge access. Non-escalated run failed with Docker socket permission denied: connect /var/run/docker.sock operation not permitted. Escalated rerun was rejected by approval reviewer: Automatic approval review failed: usage limit hit; try again after 6:46 PM or with explicit user approval.`
