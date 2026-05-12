#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${WS_DIR}"

START_BRIDGE="${START_BRIDGE:-false}"
PROFILE="${PROFILE:-robust}"
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC:-180}"
OVERLAP_DURATION_SEC="${OVERLAP_DURATION_SEC:-180}"
NO_OVERLAP_DURATION_SEC="${NO_OVERLAP_DURATION_SEC:-90}"
TEAM_POSE_GRAPH_BACKEND="${TEAM_POSE_GRAPH_BACKEND:-g2o_export_only}"
TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE="${TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE:-true}"
USE_DYNAMIC_FILTER="${USE_DYNAMIC_FILTER:-false}"

for arg in "$@"; do
  case "${arg}" in
    team_pose_graph_backend:=*)
      TEAM_POSE_GRAPH_BACKEND="${arg#team_pose_graph_backend:=}"
      ;;
    team_alignment_allow_export_only_gate:=*)
      TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE="${arg#team_alignment_allow_export_only_gate:=}"
      ;;
    use_dynamic_filter:=*)
      USE_DYNAMIC_FILTER="${arg#use_dynamic_filter:=}"
      ;;
  esac
done

if [[ "${START_BRIDGE}" == "true" ]]; then
  docker compose -f docker/ros1_scpgo/docker-compose.yml down
  bash scripts/launch/scpgo_ros1_bridge.sh -d
fi

OUT_DIR=logs/cross_loop_no_overlap_runtime_final \
DURATION_SEC="${NO_OVERLAP_DURATION_SEC}" \
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC}" \
PROFILE="${PROFILE}" \
SCENE_NAME=no_overlap_dual_scene.xml \
SCENE_HAS_OVERLAP=false \
scripts/bench/benchmark_cross_loop_closure.sh \
  "team_pose_graph_backend:=${TEAM_POSE_GRAPH_BACKEND}" \
  "team_alignment_allow_export_only_gate:=${TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE}" \
  "no_overlap_rejection_passed:=false" \
  "use_dynamic_filter:=${USE_DYNAMIC_FILTER}" \
  "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/no_overlap_dual_scene.xml"

NO_OVERLAP_REJECTION_PASSED="$(python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

summary = json.loads(
    Path("logs/cross_loop_no_overlap_runtime_final/cross_loop_closure_summary.json").read_text()
)
passed = (
    summary.get("runtime_valid") is True
    and summary.get("gt_used_runtime") is False
    and summary.get("alignment_status") in {"rejected", "tentative"}
    and summary.get("false_alignment") is False
    and summary.get("merged_map_enabled_time_sec") is None
    and int(summary.get("pose_graph_inter_robot_factors") or 0) == 0
)
print("true" if passed else "false")
PY
)"

OUT_DIR=logs/cross_loop_overlap_runtime_final \
DURATION_SEC="${OVERLAP_DURATION_SEC}" \
TOPIC_WAIT_SEC="${TOPIC_WAIT_SEC}" \
PROFILE="${PROFILE}" \
SCENE_NAME=demo3_mixed.xml \
SCENE_HAS_OVERLAP=true \
scripts/bench/benchmark_cross_loop_closure.sh \
  "team_pose_graph_backend:=${TEAM_POSE_GRAPH_BACKEND}" \
  "team_alignment_allow_export_only_gate:=${TEAM_ALIGNMENT_ALLOW_EXPORT_ONLY_GATE}" \
  "no_overlap_rejection_passed:=${NO_OVERLAP_REJECTION_PASSED}" \
  "use_dynamic_filter:=${USE_DYNAMIC_FILTER}" \
  "mujoco_model_path:=${WS_DIR}/src/go2w/go2_gazebo_sim/mujoco/demo3_mixed.xml"

python3 - <<'PY'
from __future__ import annotations

import json
from pathlib import Path

root = Path("logs")
overlap = json.loads((root / "cross_loop_overlap_runtime_final" / "cross_loop_closure_summary.json").read_text())
no_overlap = json.loads((root / "cross_loop_no_overlap_runtime_final" / "cross_loop_closure_summary.json").read_text())
backend = str(overlap.get("pose_graph_optimization_backend", ""))
error_before = overlap.get("pose_graph_error_before")
error_after = overlap.get("pose_graph_error_after")
optimized_overlap = (
    backend in {"gtsam_cpp", "gtsam_python"}
    and overlap.get("pose_graph_optimization_success") is True
    and int(overlap.get("pose_graph_inter_robot_factors") or 0) > 0
    and error_before is not None
    and error_after is not None
    and float(error_after) <= float(error_before)
)

overlap_pass = (
    overlap.get("runtime_valid") is True
    and overlap.get("gt_used_runtime") is False
    and overlap.get("alignment_status") == "aligned"
    and int(overlap.get("robust_inlier_set_size") or 0) >= 7
    and float(overlap.get("robust_inlier_ratio_eligible") or 0.0) >= 0.25
    and int(overlap.get("pose_graph_inter_robot_factors") or 0) > 0
    and optimized_overlap
    and overlap.get("merged_map_enabled_time_sec") is not None
)
no_overlap_pass = (
    no_overlap.get("runtime_valid") is True
    and no_overlap.get("gt_used_runtime") is False
    and no_overlap.get("alignment_status") in {"rejected", "tentative"}
    and no_overlap.get("false_alignment") is False
    and no_overlap.get("merged_map_enabled_time_sec") is None
    and int(no_overlap.get("pose_graph_inter_robot_factors") or 0) == 0
)
combined = {
    "schema": "cross_loop_closure_runtime_validation/v2",
    "claim_boundary": "Optimized PGO claim is valid only when overlap_pass and no_overlap_pass are true with a gtsam_cpp or gtsam_python backend.",
    "final_status": (
        "Status A - GTSAM optimized backend passed"
        if overlap_pass and no_overlap_pass and optimized_overlap
        else "Status B - optimized backend not validated; keep export/safety claim only"
    ),
    "overlap_pass": overlap_pass,
    "no_overlap_pass": no_overlap_pass,
    "optimized_overlap": optimized_overlap,
    "overlap": overlap,
    "no_overlap": no_overlap,
}
(root / "cross_loop_closure_final_eval.json").write_text(json.dumps(combined, indent=2, sort_keys=True) + "\n")
(root / "cross_loop_closure_final_eval.md").write_text(
    "\n".join([
        "# Cross-Loop Runtime Validation",
        "",
        f"- overlap_pass: `{overlap_pass}`",
        f"- no_overlap_pass: `{no_overlap_pass}`",
        f"- overlap_alignment_status: `{overlap.get('alignment_status')}`",
        f"- overlap_inliers: `{overlap.get('robust_inlier_set_size')}`",
        f"- overlap_pose_graph_inter_robot_factors: `{overlap.get('pose_graph_inter_robot_factors')}`",
        f"- overlap_pose_graph_backend: `{overlap.get('pose_graph_optimization_backend')}`",
        f"- overlap_pose_graph_optimization_success: `{overlap.get('pose_graph_optimization_success')}`",
        f"- overlap_pose_graph_error_before: `{overlap.get('pose_graph_error_before')}`",
        f"- overlap_pose_graph_error_after: `{overlap.get('pose_graph_error_after')}`",
        f"- no_overlap_alignment_status: `{no_overlap.get('alignment_status')}`",
        f"- no_overlap_false_alignment: `{no_overlap.get('false_alignment')}`",
        f"- gt_used_runtime_overlap: `{overlap.get('gt_used_runtime')}`",
        f"- gt_used_runtime_no_overlap: `{no_overlap.get('gt_used_runtime')}`",
        "",
        (
            "Claim: optimized centralized multi-robot pose graph correction with robust accepted "
            "inter-robot factors is validated for this run."
            if overlap_pass and no_overlap_pass and optimized_overlap
            else "Claim: export/safety architecture only; optimized runtime PGO is not validated."
        ),
    ]) + "\n"
)

overlap_artifacts = root / "cross_loop_overlap_runtime_final" / "artifacts"
metrics_path = overlap_artifacts / "team_pose_graph_metrics.json"
graph_dir = overlap_artifacts / "team_pose_graph"
if optimized_overlap and metrics_path.exists():
    (root / "team_pose_graph_metrics.json").write_text(metrics_path.read_text())
if optimized_overlap and (graph_dir / "team_pose_graph.g2o").exists():
    (root / "team_pose_graph.g2o").write_text((graph_dir / "team_pose_graph.g2o").read_text())
if optimized_overlap and (graph_dir / "team_pose_graph_factors.json").exists():
    (root / "team_pose_graph_factors.json").write_text((graph_dir / "team_pose_graph_factors.json").read_text())
print(json.dumps(combined, indent=2, sort_keys=True))
PY
