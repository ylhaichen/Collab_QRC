#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"
mkdir -p logs

PYTHONPATH=src/collaborative_exploration/map_cleanup \
python3 -m pytest -q src/collaborative_exploration/map_cleanup/test/test_map_cleanup_contract.py

PYTHONPATH=src/collaborative_exploration/map_cleanup python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

from map_cleanup.cleanup_contracts import CleanupBackendStatus, RequiredCleanupExport, backend_available

requested = "erasor"
available, blocker = backend_available(requested)
status = CleanupBackendStatus(
    backend=requested,
    backend_available=available,
    fallback_backend="temporal_voxel_fallback",
    dependency_blocker=blocker,
)
export = RequiredCleanupExport.from_root("logs/map_cleanup_export")
summary = status.to_payload()
summary.update({
    "schema": "map_cleanup_validation/v1",
    "runtime_valid": available and not blocker,
    "required_export_missing_paths": export.missing_paths(),
    "cleaned_static_map_topic": "/team_slam/cleaned_static_map",
    "removed_dynamic_points_topic": "/team_slam/removed_dynamic_points",
    "map_cleanup_metrics_topic": "/team_slam/map_cleanup_metrics",
})
Path("logs/map_cleanup_validation.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
Path("logs/map_cleanup_validation.md").write_text(
    "\n".join([
        "# Map Cleanup Validation",
        "",
        f"- backend: `{summary['backend']}`",
        f"- runtime_valid: `{summary['runtime_valid']}`",
        f"- selected_runtime_backend: `{summary['selected_runtime_backend']}`",
        f"- dependency_blocker: `{summary['dependency_blocker']}`",
        "- realtime_odometry_loop: `false`",
        "- gt_used_runtime: `false`",
    ]) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
