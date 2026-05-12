#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _load(path: Path, default: Any) -> Any:
    if not path.exists():
        return default
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return default


def _rows(root: Path, kind: str) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for path in sorted((root / kind).glob("trial_*/cross_loop_closure_summary.json")):
        obj = _load(path, {})
        if isinstance(obj, dict):
            out.append(obj)
    out.sort(key=lambda row: int(row.get("trial_id", 0) or 0))
    return out


def _overlap_trial_pass(row: dict[str, Any]) -> bool:
    merged_time = row.get("merged_map_enabled_time_sec")
    aligned_time = row.get("time_to_alignment_sec")
    merged_after_alignment = (
        merged_time is not None
        and float(merged_time) >= 0.0
        and (aligned_time is None or float(merged_time) + 1.0 >= float(aligned_time))
    )
    return (
        row.get("runtime_valid") is True
        and row.get("gt_used_runtime") is False
        and row.get("alignment_status") == "aligned"
        and row.get("alignment_success") is True
        and row.get("false_alignment") is False
        and int(row.get("robust_inlier_count", row.get("robust_inlier_set_size", 0)) or 0) >= 7
        and float(row.get("robust_inlier_ratio_eligible") or 0.0) >= 0.25
        and int(row.get("pose_graph_inter_robot_factors") or 0) > 0
        and merged_after_alignment
        and float(row.get("point_lio_odom_rate") or 0.0) > 0.0
        and float(row.get("point_lio_cloud_rate") or 0.0) > 0.0
        and row.get("nav2_odom_tf_valid") is True
        and int(row.get("keyframe_count", row.get("keyframes", 0)) or 0) > 0
        and int(row.get("descriptor_candidate_count", row.get("descriptor_candidates", 0)) or 0) > 0
        and int(row.get("verified_match_count", row.get("verified_matches", 0)) or 0) > 0
    )


def _no_overlap_trial_pass(row: dict[str, Any]) -> bool:
    return (
        row.get("runtime_valid") is True
        and row.get("gt_used_runtime") is False
        and row.get("alignment_status") in {"rejected", "tentative"}
        and row.get("alignment_success") is False
        and row.get("false_alignment") is False
        and row.get("merged_map_enabled_time_sec") is None
        and int(row.get("pose_graph_inter_robot_factors") or 0) == 0
        and float(row.get("point_lio_odom_rate") or 0.0) > 0.0
        and float(row.get("point_lio_cloud_rate") or 0.0) > 0.0
        and row.get("nav2_odom_tf_valid") is True
        and int(row.get("keyframe_count", row.get("keyframes", 0)) or 0) > 0
    )


def _dynamic_pass(summary: dict[str, Any]) -> bool:
    return (
        summary.get("runtime_valid") is True
        and summary.get("gt_used_runtime") is False
        and summary.get("moving_object_appears_in_cloud_dynamic") is True
        and summary.get("cloud_static_excludes_moving_trace") is True
        and summary.get("static_walls_remain_in_cloud_static") is True
        and summary.get("dynamic_obstacle_layer_clears_after_ttl") is True
        and summary.get("team_loop_closure_uses_cloud_static") is True
        and summary.get("moving_object_permanent_merged_map_obstacle") is False
    )


def _comm_pass(summary: dict[str, Any]) -> bool:
    return (
        summary.get("runtime_valid") is True
        and summary.get("gt_used_runtime") is False
        and summary.get("descriptors_exchanged_between_robot_a_and_robot_b") is True
        and summary.get("compact_cloud_requested_only_on_candidate") is True
        and int(summary.get("cloud_on_demand_request_count") or 0) > 0
        and int(summary.get("bytes_sent") or 0) > 0
        and int(summary.get("bytes_received") or 0) > 0
        and summary.get("continuous_raw_lidar_exchange") is False
        and summary.get("continuous_dense_map_exchange") is False
        and summary.get("continuous_full_costmap_exchange") is False
        and summary.get("peer_loss_does_not_open_merged_map") is True
    )


def _fast_lio_pass(summary: dict[str, Any]) -> bool:
    return (
        summary.get("runtime_valid") is True
        and summary.get("odom_topics_nonzero_rate") is True
        and summary.get("cloud_topics_nonzero_rate") is True
        and summary.get("fast_lio_scpgo_fallback_available") is True
        and summary.get("gt_used_runtime") is False
    )


def _required_logs_exist(repo: Path) -> tuple[bool, list[str]]:
    required = [
        "logs/local_slam_validation.json",
        "logs/local_slam_validation.md",
        "logs/point_lio_validation.json",
        "logs/point_lio_validation.md",
        "logs/dynamic_filter_validation.json",
        "logs/dynamic_filter_validation.md",
        "logs/decentralized_comm_validation.json",
        "logs/decentralized_comm_validation.md",
        "logs/fast_lio_fallback_regression.json",
        "logs/fast_lio_fallback_regression.md",
    ]
    missing = [item for item in required if not (repo / item).exists()]
    return len(missing) == 0, missing


def _trial_projection(row: dict[str, Any], passed: bool) -> dict[str, Any]:
    keys = [
        "scene_name",
        "trial_id",
        "local_slam_backend",
        "registration_backend",
        "robust_selection_backend",
        "alignment_status",
        "alignment_success",
        "false_alignment",
        "missed_alignment",
        "time_to_alignment_sec",
        "robust_inlier_count",
        "robust_inlier_ratio_eligible",
        "pose_graph_inter_robot_factors",
        "merged_map_enabled_time_sec",
        "gt_used_runtime",
        "point_lio_odom_rate",
        "point_lio_cloud_rate",
        "nav2_odom_tf_valid",
        "keyframe_count",
        "descriptor_candidate_count",
        "verified_match_count",
        "rejected_match_count",
    ]
    out = {key: row.get(key) for key in keys}
    out["passed"] = passed
    return out


def _write_md(path: Path, payload: dict[str, Any]) -> None:
    lines = [
        "# Simulation Hardening Eval",
        "",
        f"- final_status_label: `{payload['final_status_label']}`",
        f"- confidence_complete: `{payload['confidence_complete']}`",
        f"- point_lio_primary_simulation_runtime_passed: `{payload['point_lio_primary_simulation_runtime_passed']}`",
        f"- overlap_multi_run_passed: `{payload['overlap_multi_run']['passed']}`",
        f"- no_overlap_multi_run_passed: `{payload['no_overlap_multi_run']['passed']}`",
        f"- merged_map_safety_gate_passed: `{payload['merged_map_safety_gate_passed']}`",
        f"- gt_used_runtime_any: `{payload['gt_used_runtime_any']}`",
        f"- nav2_odom_tf_valid_all_trials: `{payload['nav2_odom_tf_valid_all_trials']}`",
        f"- team_loop_closure_keyframes_valid_all_trials: `{payload['team_loop_closure_keyframes_valid_all_trials']}`",
        f"- dynamic_object_stress_passed: `{payload['dynamic_object_stress']['passed']}`",
        f"- decentralized_comm_stress_passed: `{payload['decentralized_comm_stress']['passed']}`",
        f"- fast_lio_fallback_regression_passed: `{payload['fast_lio_fallback_regression']['passed']}`",
        f"- visualized_demo_script_generated: `{payload['visualized_demo_script_generated']}`",
        "",
        "## Claim Boundary",
        "",
        payload["claim_boundary"],
        "",
        "## Backends",
        "",
        f"- registration_backend: `{payload['registration_backend']}`",
        f"- robust_selection_backend: `{payload['robust_selection_backend']}`",
        f"- kiss_matcher_runtime_validated: `{payload['kiss_matcher_runtime_validated']}`",
        f"- erasor_removert_runtime_validated: `{payload['erasor_removert_runtime_validated']}`",
        "",
        "## Blockers",
        "",
    ]
    if payload["blockers"]:
        for blocker in payload["blockers"]:
            lines.append(f"- `{blocker}`")
    else:
        lines.append("- none")
    lines.extend([
        "",
        "## Overlap Trials",
        "",
        "| trial | status | pass | inliers | factors | merged_map_sec | odom_hz | cloud_hz | nav2_tf |",
        "|---:|---|---|---:|---:|---:|---:|---:|---|",
    ])
    for row in payload["overlap_multi_run"]["trials"]:
        lines.append(
            f"| {row.get('trial_id')} | {row.get('alignment_status')} | {row.get('passed')} | "
            f"{row.get('robust_inlier_count')} | {row.get('pose_graph_inter_robot_factors')} | "
            f"{row.get('merged_map_enabled_time_sec')} | {row.get('point_lio_odom_rate')} | "
            f"{row.get('point_lio_cloud_rate')} | {row.get('nav2_odom_tf_valid')} |"
        )
    lines.extend([
        "",
        "## No-Overlap Trials",
        "",
        "| trial | status | pass | false_alignment | factors | merged_map_sec | odom_hz | cloud_hz | nav2_tf |",
        "|---:|---|---|---|---:|---:|---:|---:|---|",
    ])
    for row in payload["no_overlap_multi_run"]["trials"]:
        lines.append(
            f"| {row.get('trial_id')} | {row.get('alignment_status')} | {row.get('passed')} | "
            f"{row.get('false_alignment')} | {row.get('pose_graph_inter_robot_factors')} | "
            f"{row.get('merged_map_enabled_time_sec')} | {row.get('point_lio_odom_rate')} | "
            f"{row.get('point_lio_cloud_rate')} | {row.get('nav2_odom_tf_valid')} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo-root", default=".")
    ap.add_argument("--hardening-root", default="logs/simulation_hardening")
    ap.add_argument("--required-trials", type=int, default=3)
    args = ap.parse_args()

    repo = Path(args.repo_root).resolve()
    root = (repo / args.hardening_root).resolve()
    logs = repo / "logs"
    logs.mkdir(exist_ok=True)

    overlap = _rows(root, "overlap")
    no_overlap = _rows(root, "no_overlap")
    overlap_trials = [_trial_projection(row, _overlap_trial_pass(row)) for row in overlap]
    no_overlap_trials = [_trial_projection(row, _no_overlap_trial_pass(row)) for row in no_overlap]
    dynamic = _load(logs / "dynamic_filter_validation.json", {})
    comm = _load(logs / "decentralized_comm_validation.json", {})
    fast = _load(logs / "fast_lio_fallback_regression.json", {})
    point = _load(logs / "point_lio_validation.json", {})
    local = _load(logs / "local_slam_validation.json", {})

    overlap_pass = len(overlap_trials) >= args.required_trials and all(row["passed"] for row in overlap_trials[: args.required_trials])
    no_overlap_pass = len(no_overlap_trials) >= args.required_trials and all(
        row["passed"] for row in no_overlap_trials[: args.required_trials]
    )
    all_rows = overlap + no_overlap
    gt_any = any(bool(row.get("gt_used_runtime")) for row in all_rows)
    merged_map_gate = (
        all(_overlap_trial_pass(row) for row in overlap)
        and all(row.get("merged_map_enabled_time_sec") is None for row in no_overlap)
        and all(int(row.get("pose_graph_inter_robot_factors") or 0) > 0 for row in overlap if row.get("merged_map_enabled_time_sec") is not None)
    )
    nav2_all = len(all_rows) >= args.required_trials * 2 and all(bool(row.get("nav2_odom_tf_valid")) for row in all_rows)
    keyframes_all = len(all_rows) >= args.required_trials * 2 and all(
        int(row.get("keyframe_count", row.get("keyframes", 0)) or 0) > 0 for row in all_rows
    )
    point_primary_passed = (
        local.get("point_lio_primary_passed") is True
        and local.get("nav2_odom_tf_validated") is True
        and point.get("shadow_validation_passed") is True
        and point.get("gt_used_runtime") is False
    )
    dynamic_passed = _dynamic_pass(dynamic)
    comm_passed = _comm_pass(comm)
    fast_passed = _fast_lio_pass(fast)

    required_logs_exist, missing_logs = _required_logs_exist(repo)
    registration_backend = next((str(row.get("registration_backend")) for row in all_rows if row.get("registration_backend")), "icp_2d")
    robust_backend = next(
        (str(row.get("robust_selection_backend")) for row in all_rows if row.get("robust_selection_backend")),
        "greedy_consistency_fallback",
    )
    blockers: list[str] = []
    if not point_primary_passed:
        blockers.append(f"Point-LIO primary/shadow simulation validation not fully passed: local={local.get('exact_error', '')} point={point.get('exact_error', '')}")
    if not overlap_pass:
        blockers.append("Overlap multi-run did not pass all required trials.")
    if not no_overlap_pass:
        blockers.append("No-overlap multi-run did not reject all required trials.")
    if gt_any:
        blockers.append("At least one trial reported gt_used_runtime=true.")
    if not merged_map_gate:
        blockers.append("/merged_map safety gate evidence incomplete or failed.")
    if not nav2_all:
        blockers.append("Nav2 odom/tf was not valid in every hardening trial.")
    if not keyframes_all:
        blockers.append("team_loop_closure keyframes were not valid in every hardening trial.")
    if not dynamic_passed:
        blockers.append(f"Dynamic object stress failed or incomplete: {dynamic.get('claim_boundary', '')}")
    if not comm_passed:
        blockers.append(f"Decentralized communication stress failed or incomplete: {comm.get('claim_boundary', '')}")
    if not fast_passed:
        blockers.append(f"Fast-LIO fallback regression failed: {fast.get('exact_error', '')}")
    if missing_logs:
        blockers.append("Missing required logs: " + ", ".join(missing_logs))

    confidence = (
        point_primary_passed
        and overlap_pass
        and no_overlap_pass
        and not gt_any
        and merged_map_gate
        and nav2_all
        and keyframes_all
        and dynamic_passed
        and comm_passed
        and fast_passed
        and required_logs_exist
    )
    visual_script = repo / "scripts/manual/run_visualized_pointlio_disco_demo.sh"
    payload = {
        "schema": "simulation_hardening_eval/v1",
        "final_status_label": "Simulation Hardening Passed" if confidence else "Simulation Hardening Incomplete",
        "confidence_complete": confidence,
        "claim_boundary": "Simulation hardening only. Real robot validation and Status A are not claimed by this report.",
        "required_trial_count": args.required_trials,
        "point_lio_primary_simulation_runtime_passed": point_primary_passed,
        "point_lio_validation": {
            "native_odom_topic": point.get("native_odom_topic"),
            "native_cloud_topic": point.get("native_cloud_topic"),
            "shadow_validation_passed": point.get("shadow_validation_passed"),
            "primary_validation_passed": local.get("point_lio_primary_passed"),
        },
        "registration_backend": registration_backend,
        "robust_selection_backend": robust_backend,
        "kiss_matcher_runtime_validated": registration_backend == "kiss_matcher",
        "kiss_matcher_note": "KISS-Matcher runtime not validated in this simulation-hardening pass." if registration_backend != "kiss_matcher" else "",
        "erasor_removert_runtime_validated": False,
        "map_cleanup_note": "ERASOR/Removert runtime not validated; temporal_voxel_fallback remains the simulation cleanup contract.",
        "overlap_multi_run": {
            "passed": overlap_pass,
            "required_trials": args.required_trials,
            "observed_trials": len(overlap_trials),
            "trials": overlap_trials,
        },
        "no_overlap_multi_run": {
            "passed": no_overlap_pass,
            "required_trials": args.required_trials,
            "observed_trials": len(no_overlap_trials),
            "trials": no_overlap_trials,
        },
        "dynamic_object_stress": {
            "passed": dynamic_passed,
            "source": dynamic.get("source", dynamic.get("validation_type")),
            "synthetic_contract": str(dynamic.get("validation_type", "")).startswith("synthetic"),
            "summary": dynamic,
        },
        "decentralized_comm_stress": {
            "passed": comm_passed,
            "summary": comm,
        },
        "fast_lio_fallback_regression": {
            "passed": fast_passed,
            "summary": fast,
        },
        "merged_map_safety_gate_passed": merged_map_gate,
        "gt_used_runtime_any": gt_any,
        "nav2_odom_tf_valid_all_trials": nav2_all,
        "team_loop_closure_keyframes_valid_all_trials": keyframes_all,
        "all_required_logs_generated": required_logs_exist,
        "missing_required_logs": missing_logs,
        "visualized_demo_script_generated": confidence and visual_script.exists(),
        "visualized_demo_command": "bash scripts/manual/run_visualized_pointlio_disco_demo.sh" if confidence and visual_script.exists() else "",
        "blockers": blockers,
    }
    (logs / "simulation_hardening_eval.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    _write_md(logs / "simulation_hardening_eval.md", payload)

    cross_payload = {
        "schema": "cross_loop_closure_simulation_hardening/v1",
        "overlap_pass": overlap_pass,
        "no_overlap_pass": no_overlap_pass,
        "merged_map_safety_gate_passed": merged_map_gate,
        "gt_used_runtime_any": gt_any,
        "overlap_trials": overlap_trials,
        "no_overlap_trials": no_overlap_trials,
    }
    (logs / "cross_loop_closure_final_eval.json").write_text(
        json.dumps(cross_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (logs / "cross_loop_closure_final_eval.md").write_text(
        "\n".join(
            [
                "# Cross-Loop Simulation Hardening",
                "",
                f"- overlap_pass: `{overlap_pass}`",
                f"- no_overlap_pass: `{no_overlap_pass}`",
                f"- merged_map_safety_gate_passed: `{merged_map_gate}`",
                f"- gt_used_runtime_any: `{gt_any}`",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    final_payload = {
        "schema": "final_system_validation/v2",
        "final_status_label": payload["final_status_label"],
        "confidence_complete": confidence,
        "status_a_claimed": False,
        "real_robot_validation_claimed": False,
        "fast_lio_fallback_regression_passed": fast_passed,
        "fast_lio_can_be_demoted": False,
        "fast_lio_global_demotion_allowed": False,
        "fast_lio_retained_as_fallback": True,
        "fast_lio_must_remain_production": True,
        "simulation_hardening_eval": payload,
    }
    (logs / "final_system_validation.json").write_text(
        json.dumps(final_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (logs / "final_system_validation.md").write_text(
        "\n".join(
            [
                "# Final System Validation",
                "",
                f"- final_status_label: `{payload['final_status_label']}`",
                f"- confidence_complete: `{confidence}`",
                "- status_a_claimed: `false`",
                "- real_robot_validation_claimed: `false`",
                f"- fast_lio_fallback_regression_passed: `{fast_passed}`",
                "- fast_lio_can_be_demoted: `false`",
                "- fast_lio_global_demotion_allowed: `false`",
                "- fast_lio_retained_as_fallback: `true`",
                "- fast_lio_must_remain_production: `true`",
                "",
                "See `logs/simulation_hardening_eval.json` for trial-level evidence.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if confidence else 1


if __name__ == "__main__":
    raise SystemExit(main())
