# Baseline Regression Rerun Status

- command: `START_BRIDGE=true bash scripts/bench/run_cross_loop_runtime_validation.sh`
- attempted: `True`
- passed: `False`
- completed_rerun_backend: `g2o_export_only`
- script_default_now: `TEAM_POSE_GRAPH_BACKEND=auto`
- export_only_gate_default_now: `false`
- blocker: `Required fresh baseline rerun with START_BRIDGE=true was not completed after the script default was corrected to TEAM_POSE_GRAPH_BACKEND=auto and TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE=false. The escalated Docker/bridge rerun was rejected by the approval reviewer due usage limit; retry was deferred by the environment until 8:06 PM or explicit approval. The last completed rerun used g2o_export_only, so optimized overlap_pass is false and the required Fast-LIO baseline regression cannot be claimed fresh-passing in this pass.`
