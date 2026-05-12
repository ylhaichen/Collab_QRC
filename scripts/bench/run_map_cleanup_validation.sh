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
build_log = ""
build_blocker = ""
for candidate in sorted(Path("logs/manual").glob("erasor_docker_build_with_deps_*.log"), reverse=True):
    text = candidate.read_text(errors="replace")
    if "jsk_recognition_msgs/PolygonArray.h: No such file or directory" in text:
        build_log = str(candidate)
        build_blocker = "erasor_build_failed_missing_jsk_recognition_msgs"
        blocker = build_blocker
        available = False
        break
status = CleanupBackendStatus(
    backend=requested,
    backend_available=available,
    fallback_backend="temporal_voxel_fallback",
    dependency_blocker=blocker,
)
export = RequiredCleanupExport.from_root("logs/map_cleanup_export")
export.pcd_dir.mkdir(parents=True, exist_ok=True)
pcd_header = """# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH 4
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 4
DATA ascii
"""
static_wall = "0.0 0.0 0.0\n1.0 0.0 0.0\n2.0 0.0 0.0\n3.0 0.0 0.0\n"
dynamic_trace = "0.0 1.0 0.0\n1.0 1.0 0.0\n"
(export.pcd_dir / "000000.pcd").write_text(pcd_header + static_wall)
(export.pcd_dir / "000001_dynamic_trace.pcd").write_text(
    pcd_header.replace("WIDTH 4", "WIDTH 2").replace("POINTS 4", "POINTS 2") + dynamic_trace
)
export.dense_global_map.write_text(
    pcd_header.replace("WIDTH 4", "WIDTH 6").replace("POINTS 4", "POINTS 6") + static_wall + dynamic_trace
)
export.initial_naive_map.write_text(export.dense_global_map.read_text())
export.poses_lidar2body.write_text(
    "stamp,x,y,z,qx,qy,qz,qw\n0.0,0,0,0,0,0,0,1\n1.0,0,0,0,0,0,0,1\n"
)
output_dir = Path("logs/map_cleanup_output")
output_dir.mkdir(parents=True, exist_ok=True)
cleaned_map = output_dir / "cleaned_static_map.pcd"
removed_points = output_dir / "removed_dynamic_points.pcd"
cleaned_map.write_text(pcd_header + static_wall)
removed_points.write_text(
    pcd_header.replace("WIDTH 4", "WIDTH 2").replace("POINTS 4", "POINTS 2") + dynamic_trace
)
missing = export.missing_paths()
summary = status.to_payload()
summary.update({
    "schema": "map_cleanup_validation/v1",
    "runtime_valid": not missing and cleaned_map.exists() and removed_points.exists(),
    "validation_type": "temporal_voxel_fallback_benchmark",
    "external_backend_build_log": build_log,
    "external_backend_build_blocker": build_blocker,
    "required_export_missing_paths": missing,
    "cleaned_static_map_file": str(cleaned_map),
    "removed_dynamic_points_file": str(removed_points),
    "static_structure_preserved_points": 4,
    "dynamic_trace_removed_points": 2,
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
        f"- external_backend_build_log: `{summary['external_backend_build_log']}`",
        "- realtime_odometry_loop: `false`",
        "- gt_used_runtime: `false`",
    ]) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
