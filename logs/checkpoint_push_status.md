# Checkpoint Push Status

- original checkpoint: `checkpoint/v3-status-b-stable` / `v3-status-b-stable` at `f8fa9f8`.
- original fork branch/tag push: failed with GitHub `GH001 Large files detected`; `logs/cross_loop_overlap_runtime_pass/robust_loop_inliers.jsonl` is `963.36 MB` and `logs/cross_loop_no_overlap_runtime_pass/robust_loop_inliers.jsonl` is `950.38 MB`, above GitHub's `100.00 MB` limit.
- clean checkpoint: `checkpoint/v3-status-b-stable-clean` / `v3-status-b-stable-clean` at `9fabffd`.
- clean feature: `feature/swarm-lio2-primary-dynamiclio-erasor-clean` at `5c50235`.
- fork clean checkpoint branch push: succeeded.
- fork clean checkpoint tag push: succeeded.
- fork clean feature branch push: succeeded.
- origin clean checkpoint branch/tag/feature push: failed with `remote: Permission to HanshangZhu/Collab_QRC.git denied to ylhaichen` and `The requested URL returned error: 403`.

The clean checkpoint was not modified after creation. The push failure remaining on `origin` is remote permission, not large Git history.
