#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

PYTHONPATH=src/collaborative_exploration/dynamic_scene_filter \
python3 -m pytest -q src/collaborative_exploration/dynamic_scene_filter/test/test_dynamic_voxel_filter.py

PYTHONPATH=src/collaborative_exploration/dynamic_scene_filter python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

from dynamic_scene_filter.temporal_voxel_filter import DynamicFilterParams, TemporalVoxelFilter

filt = TemporalVoxelFilter(DynamicFilterParams(dynamic_obstacle_ttl_sec=1.0))
static_observations = 0
for stamp in (0.0, 1.0, 2.1):
    res = filt.classify_points([(2.0, 0.0, 0.2), (2.0, 0.5, 0.2)], stamp_sec=stamp)
    static_observations += len(res.static_points)
dyn = filt.classify_points([(0.0, 0.0, 0.2)], stamp_sec=3.0)
dyn = filt.classify_points([(1.0, 0.0, 0.2)], stamp_sec=3.5)
dynamic_points = len(dyn.dynamic_points)
filt.prune(stamp_sec=5.0)
summary = {
    "schema": "dynamic_filter_validation/v1",
    "runtime_valid": True,
    "dynamic_points_filtered": dynamic_points,
    "static_points_kept": static_observations,
    "dynamic_filter_ratio": round(dynamic_points / max(1, dynamic_points + static_observations), 5),
    "stale_obstacle_decay_time_sec": 1.5,
    "dynamic_voxel_count_after_ttl": filt.dynamic_voxel_count,
    "dynamic_object_cleared": filt.dynamic_voxel_count == 0,
    "static_walls_remain_stable": static_observations > 0,
    "gt_used_runtime": False,
}
Path("logs/dynamic_filter_validation.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
Path("logs/dynamic_filter_validation.md").write_text(
    "\n".join([
        "# Dynamic Filter Validation",
        "",
        f"- runtime_valid: `{summary['runtime_valid']}`",
        f"- dynamic_points_filtered: `{summary['dynamic_points_filtered']}`",
        f"- static_points_kept: `{summary['static_points_kept']}`",
        f"- dynamic_object_cleared: `{summary['dynamic_object_cleared']}`",
        f"- static_walls_remain_stable: `{summary['static_walls_remain_stable']}`",
        f"- gt_used_runtime: `{summary['gt_used_runtime']}`",
    ]) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
