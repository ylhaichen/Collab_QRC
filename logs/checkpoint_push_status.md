# Checkpoint Push Status

- original checkpoint: `checkpoint/v3-status-b-stable` / `v3-status-b-stable` at `f8fa9f8`.
- original fork branch/tag push: failed with GitHub `GH001 Large files detected`; `logs/cross_loop_overlap_runtime_pass/robust_loop_inliers.jsonl` is `963.36 MB` and `logs/cross_loop_no_overlap_runtime_pass/robust_loop_inliers.jsonl` is `950.38 MB`, above GitHub's `100.00 MB` limit.
- clean checkpoint: `checkpoint/v3-status-b-stable-clean` / `v3-status-b-stable-clean` at `9fabffd`.
- clean feature: `feature/swarm-lio2-primary-dynamiclio-erasor-clean`; infrastructure recovery commit `207d056`, push-status record commit `ac74ea7`.
- fork clean checkpoint branch push: succeeded; latest confirmation returned `Everything up-to-date`.
- fork clean checkpoint tag push: succeeded; latest confirmation returned `Everything up-to-date`.
- fork clean feature branch push: succeeded; latest confirmation returned `Everything up-to-date`.
- origin push intentionally skipped in the latest validation because `origin` points to upstream `https://github.com/HanshangZhu/Collab_QRC.git` and the user only wants pushes to `https://github.com/ylhaichen/Collab_QRC.git`.

The clean checkpoint was not modified after creation. Current valid pushed targets are `fork/checkpoint/v3-status-b-stable-clean`, `fork/feature/swarm-lio2-primary-dynamiclio-erasor-clean`, and fork tag `v3-status-b-stable-clean`.
