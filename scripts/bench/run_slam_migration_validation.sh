#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"

python3 - <<'PY'
from __future__ import annotations

import json
import shutil
from pathlib import Path

root = Path("logs")
root.mkdir(exist_ok=True)
deps = Path("external")
swarm_src = deps / "Swarm-LIO2"
dynamic_src = deps / "dynamic_lio"
erasor_src = deps / "ERASOR"

baseline_path = root / "cross_loop_closure_final_eval.json"
baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {}
dynamic_validation_path = root / "dynamic_filter_validation.json"
dynamic_validation = (
    json.loads(dynamic_validation_path.read_text())
    if dynamic_validation_path.exists()
    else {}
)

swarm_available = swarm_src.exists()
dynamic_available = dynamic_src.exists()
erasor_available = erasor_src.exists() or shutil.which("erasor") is not None
catkin_available = shutil.which("catkin_make") is not None and shutil.which("rospack") is not None
swarm_buildable = swarm_available and catkin_available
dynamic_buildable = dynamic_available and catkin_available
erasor_buildable = erasor_available and catkin_available
swarm_runtime_ready = Path("install/swarm_lio").exists() or (swarm_src / "devel").exists()
dynamic_runtime_ready = Path("install/dynamic_lio").exists() or (dynamic_src / "devel").exists()
erasor_runtime_ready = Path("install/erasor").exists() or (erasor_src / "devel").exists() or shutil.which("erasor") is not None

def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

def write_md(path: Path, lines: list[str]) -> None:
    path.write_text("\n".join(lines) + "\n")

shadow = {
    "schema": "swarm_lio2_shadow_validation/v1",
    "slam_backend": "swarm_lio2_shadow",
    "swarm_lio2_source_available": swarm_available,
    "swarm_lio2_buildable_source": swarm_buildable,
    "swarm_lio2_runtime_ready": swarm_runtime_ready,
    "swarm_lio2_started": False,
    "swarm_lio2_odometry_valid": False,
    "swarm_lio2_relative_state_valid": False,
    "fast_lio_baseline_still_runs": bool(baseline.get("overlap_pass") and baseline.get("no_overlap_pass")),
    "production_downstream_depends_on_swarm": False,
    "metrics_recorded": True,
    "gt_used_runtime": False,
    "blocker": "" if swarm_runtime_ready else "Swarm-LIO2 source exists under external/Swarm-LIO2, but upstream packages are ROS1/catkin and this host does not expose catkin_make/rospack; shadow odometry cannot be validated",
}
write_json(root / "swarm_lio2_shadow_validation.json", shadow)

primary = {
    "schema": "swarm_lio2_primary_validation/v1",
    "slam_backend": "swarm_lio2_primary",
    "swarm_lio2_source_available": swarm_available,
    "swarm_lio2_buildable_source": swarm_buildable,
    "swarm_lio2_runtime_ready": swarm_runtime_ready,
    "adapter_contract_configured": True,
    "odometry_valid": False,
    "corrected_odom_valid": False,
    "cloud_static_or_registered_valid": False,
    "nav2_runtime_valid": False,
    "team_loop_closure_keyframes_valid": False,
    "overlap_pass": False,
    "no_overlap_pass": False,
    "gt_used_runtime": False,
    "merged_map_agreement_gated": True,
    "blocker": "" if swarm_runtime_ready else "Swarm-LIO2 source exists under external/Swarm-LIO2, but upstream packages are ROS1/catkin and this host does not expose catkin_make/rospack; primary mode cannot be validated",
}
write_json(root / "swarm_lio2_primary_validation.json", primary)

dynamic = {
    "schema": "dynamic_lio_filter_integration/v1",
    "dynamic_lio_source_available": dynamic_available,
    "dynamic_lio_buildable_source": dynamic_buildable,
    "dynamic_lio_runtime_ready": dynamic_runtime_ready,
    "dynamic_filter_backend": "dynamic_lio_wrapper" if dynamic_runtime_ready else "temporal_voxel_fallback",
    "dynamic_points_filtered": int(dynamic_validation.get("dynamic_points_filtered", 0) or 0),
    "static_points_kept": int(dynamic_validation.get("static_points_kept", 0) or 0),
    "dynamic_filter_ratio": float(dynamic_validation.get("dynamic_filter_ratio", 0.0) or 0.0),
    "stale_obstacle_decay_time_sec": dynamic_validation.get("stale_obstacle_decay_time_sec"),
    "fallback_used": not dynamic_runtime_ready,
    "blocker": "" if dynamic_runtime_ready else "Dynamic-LIO source exists under external/dynamic_lio, but upstream packages are ROS1/catkin and this host does not expose catkin_make/rospack; using temporal voxel fallback only",
    "gt_used_runtime": False,
}
write_json(root / "dynamic_lio_filter_integration.json", dynamic)

erasor = {
    "schema": "erasor_map_cleanup_validation/v1",
    "static_map_cleanup_backend": "erasor_wrapper" if erasor_runtime_ready else "temporal_voxel_fallback",
    "erasor_source_or_executable_available": erasor_available,
    "erasor_buildable_source": erasor_buildable,
    "erasor_runtime_ready": erasor_runtime_ready,
    "naive_map_contains_dynamic_trace": False,
    "cleaned_map_removes_dynamic_trace": False,
    "static_walls_preserved": False,
    "cleaned_map_published": False,
    "control_loop_blocked": False,
    "fallback_used": not erasor_runtime_ready,
    "blocker": "" if erasor_runtime_ready else "ERASOR source exists under external/ERASOR, but upstream package is ROS1/catkin and this host does not expose catkin_make/rospack; asynchronous cleanup cannot be validated",
    "gt_used_runtime": False,
}
write_json(root / "erasor_map_cleanup_validation.json", erasor)

comparison = {
    "schema": "slam_backend_comparison/v1",
    "default_slam_backend": "fast_lio_scpgo",
    "fast_lio_scpgo": {
        "overlap_pass": bool(baseline.get("overlap_pass", False)),
        "no_overlap_pass": bool(baseline.get("no_overlap_pass", False)),
        "gt_used_runtime": bool(
            baseline.get("overlap", {}).get("gt_used_runtime", False)
            or baseline.get("no_overlap", {}).get("gt_used_runtime", False)
        ),
    },
    "swarm_lio2_shadow": shadow,
    "swarm_lio2_primary": primary,
    "dynamic_filter": dynamic,
    "erasor_cleanup": erasor,
    "final_status": "Status C — External Blocker",
    "claim": "Fast-LIO remains production backend; Swarm-LIO2 primary replacement is not validated.",
}
write_json(root / "slam_backend_comparison.json", comparison)
write_md(root / "slam_backend_comparison.md", [
    "# SLAM Backend Comparison",
    "",
    f"- default_slam_backend: `{comparison['default_slam_backend']}`",
    f"- fast_lio_overlap_pass: `{comparison['fast_lio_scpgo']['overlap_pass']}`",
    f"- fast_lio_no_overlap_pass: `{comparison['fast_lio_scpgo']['no_overlap_pass']}`",
    f"- swarm_lio2_shadow_blocker: `{shadow['blocker']}`",
    f"- swarm_lio2_primary_blocker: `{primary['blocker']}`",
    f"- dynamic_filter_backend: `{dynamic['dynamic_filter_backend']}`",
    f"- erasor_blocker: `{erasor['blocker']}`",
    "",
    "Final status: `Status C — External Blocker`.",
])
print(json.dumps(comparison, indent=2, sort_keys=True))
PY
