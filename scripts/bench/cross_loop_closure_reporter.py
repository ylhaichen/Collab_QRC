#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    if not path.exists():
        return out
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("data:"):
            line = line.split("data:", 1)[1].strip().strip("'\"")
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            out.append(obj)
    return out


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _first_stamp(items: list[dict[str, Any]], pred) -> float | None:
    stamps = [
        float(x.get("stamp_sec", 0.0))
        for x in items
        if pred(x) and x.get("stamp_sec") is not None
    ]
    return min(stamps) if stamps else None


def _latest(items: list[dict[str, Any]]) -> dict[str, Any]:
    return items[-1] if items else {}


def _alignment_error(statuses: list[dict[str, Any]], gt_path: Path | None) -> dict[str, float] | None:
    if gt_path is None or not gt_path.exists():
        return None
    aligned = [
        s for s in statuses
        if s.get("status") == "aligned" and isinstance(s.get("transform"), dict)
    ]
    if not aligned:
        return None
    try:
        gt = json.loads(gt_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None
    tf = aligned[-1]["transform"]
    dx = float(tf.get("x", 0.0)) - float(gt.get("x", 0.0))
    dy = float(tf.get("y", 0.0)) - float(gt.get("y", 0.0))
    dyaw = _wrap_pi(float(tf.get("yaw", 0.0)) - float(gt.get("yaw", 0.0)))
    return {
        "translation_m": round(math.hypot(dx, dy), 4),
        "yaw_deg": round(abs(math.degrees(dyaw)), 3),
    }


def _count_inter_robot_rendezvous(loop_payloads: list[dict[str, Any]]) -> int:
    count = 0
    for payload in loop_payloads:
        for cand in payload.get("candidates", []):
            if isinstance(cand, dict) and cand.get("source") == "inter_robot_rendezvous":
                count += 1
    return count


def _first_numeric_line(path: Path) -> float | None:
    if not path.exists():
        return None
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw.strip()
        if not line or line == "---":
            continue
        try:
            return float(line)
        except ValueError:
            continue
    return None


def _runtime_error(root: Path) -> str:
    launch_log = root / "launch.log"
    if launch_log.exists():
        text = launch_log.read_text(encoding="utf-8", errors="replace")
        if "Error creating socket: Operation not permitted" in text or "getifaddrs: Operation not permitted" in text:
            return "ros_participant_socket_permission_denied"
    timeout_count = 0
    for path in root.glob("*.err"):
        if "Timed out waiting for topic" in path.read_text(encoding="utf-8", errors="replace"):
            timeout_count += 1
    if timeout_count >= 4:
        return "ros_topic_recorders_timed_out"
    required_recordings = [
        root / "keyframes.jsonl",
        root / "cross_robot_matches.jsonl",
        root / "robust_loop_inliers.jsonl",
        root / "alignment_status.jsonl",
        root / "pose_graph_metrics.jsonl",
    ]
    if launch_log.exists() and all(
        (not path.exists()) or path.stat().st_size == 0
        for path in required_recordings
    ):
        text = launch_log.read_text(encoding="utf-8", errors="replace")
        if "loop_keyframe_exporter_node up" in text or "published robot_" in text:
            return "ros_topic_recorders_empty"
    return ""


def _first_seen(*groups: list[dict[str, Any]]) -> float:
    stamps = [
        float(x.get("stamp_sec", 0.0))
        for group in groups
        for x in group
        if x.get("stamp_sec") is not None
    ]
    return min(stamps) if stamps else 0.0


def _write_markdown(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# Cross-Robot Loop Closure v2 Summary",
        "",
        f"- scene_name: `{summary['scene_name']}`",
        f"- trial_id: `{summary['trial_id']}`",
        f"- profile: `{summary['profile']}`",
        f"- relative_pose_source: `{summary['relative_pose_source']}`",
        f"- gt_used_runtime: `{summary['gt_used_runtime']}`",
        f"- runtime_valid: `{summary['runtime_valid']}`",
        f"- runtime_error: `{summary['runtime_error']}`",
        f"- keyframes: `{summary['keyframes']}`",
        f"- descriptor_candidates: `{summary['descriptor_candidates']}`",
        f"- cross_robot_matches_total: `{summary['cross_robot_matches_total']}`",
        f"- verified_matches: `{summary['verified_matches']}`",
        f"- rejected_matches: `{summary['rejected_matches']}`",
        f"- robust_inlier_set_size: `{summary['robust_inlier_set_size']}`",
        f"- robust_inlier_ratio: `{summary['robust_inlier_ratio']}`",
        f"- robust_inlier_ratio_raw: `{summary['robust_inlier_ratio_raw']}`",
        f"- robust_inlier_ratio_eligible: `{summary['robust_inlier_ratio_eligible']}`",
        f"- prefiltered_matches: `{summary['prefiltered_matches']}`",
        f"- eligible_matches: `{summary['eligible_matches']}`",
        f"- deduplicated_matches: `{summary['deduplicated_matches']}`",
        f"- raw_consistent_matches: `{summary['raw_consistent_matches']}`",
        f"- robust_rejected_matches: `{summary['robust_rejected_matches']}`",
        f"- median_rmse: `{summary['median_rmse']}`",
        f"- median_descriptor_distance: `{summary['median_descriptor_distance']}`",
        f"- median_inlier_ratio: `{summary['median_inlier_ratio']}`",
        f"- transform_spread_translation: `{summary['transform_spread_translation']}`",
        f"- transform_spread_yaw_deg: `{summary['transform_spread_yaw_deg']}`",
        f"- reject_reason: `{summary['reject_reason']}`",
        f"- alignment_status: `{summary['alignment_status']}`",
        f"- time_to_alignment_sec: `{summary['time_to_alignment_sec']}`",
        f"- merged_map_enabled_time_sec: `{summary['merged_map_enabled_time_sec']}`",
        f"- pose_graph_optimization_success: `{summary['pose_graph_optimization_success']}`",
        f"- dependency_blocker: `{summary.get('dependency_blocker', '')}`",
        f"- pose_graph_num_factors: `{summary['pose_graph_num_factors']}`",
        f"- dynamic_points_filtered: `{summary.get('dynamic_points_filtered', 0)}`",
        f"- static_points_kept: `{summary.get('static_points_kept', 0)}`",
        f"- dynamic_filter_ratio: `{summary.get('dynamic_filter_ratio', 0.0)}`",
        f"- peer_envelopes: `{summary.get('peer_envelopes', 0)}`",
        f"- false_alignment: `{summary['false_alignment']}`",
        f"- missed_alignment: `{summary['missed_alignment']}`",
        f"- inter_robot_rendezvous_candidates: `{summary['inter_robot_rendezvous_candidates']}`",
    ]
    err = summary.get("gt_eval_only_alignment_error")
    if err:
        lines.extend([
            "",
            "## Eval-Only GT Alignment Error",
            "",
            f"- translation_m: `{err['translation_m']}`",
            f"- yaw_deg: `{err['yaw_deg']}`",
        ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def summarize_run(args: argparse.Namespace) -> dict[str, Any]:
    root = Path(args.log_dir)
    keyframes = _load_jsonl(root / "keyframes.jsonl")
    candidates = _load_jsonl(root / "cross_robot_candidates.jsonl")
    matches = _load_jsonl(root / "cross_robot_matches.jsonl")
    robust = _load_jsonl(root / "robust_loop_inliers.jsonl")
    statuses = _load_jsonl(root / "alignment_status.jsonl")
    pose_metrics = _load_jsonl(root / "pose_graph_metrics.jsonl")
    dynamic_metrics = _load_jsonl(root / "dynamic_filter_metrics.jsonl")
    peer_statuses = _load_jsonl(root / "peer_status.jsonl")
    peer_envelopes = _load_jsonl(root / "peer_envelopes.jsonl")
    loop_candidates = _load_jsonl(root / "loop_candidates.jsonl")

    accepted = [m for m in matches if bool(m.get("accepted", False))]
    rejected = [m for m in matches if not bool(m.get("accepted", False))]
    latest_robust = _latest(robust)
    latest_status = _latest(statuses)
    latest_pose = _latest(pose_metrics)
    latest_dynamic = _latest(dynamic_metrics)
    latest_peer = _latest(peer_statuses)
    final_status = str(latest_status.get("status", "unknown")) if latest_status else "unknown"
    runtime_error = _runtime_error(root)
    runtime_valid = runtime_error == ""
    first_seen = _first_seen(keyframes, candidates, matches, robust, statuses, pose_metrics)
    aligned_stamp = _first_stamp(statuses, lambda x: x.get("status") == "aligned")
    ever_aligned = aligned_stamp is not None
    time_to_alignment = None if aligned_stamp is None else round(aligned_stamp - first_seen, 3)
    merged_map_stamp = _first_numeric_line(root / "merged_map_stamps.txt")
    if merged_map_stamp is None:
        merged_map_stamp = _first_stamp(statuses, lambda x: x.get("status") == "aligned")
    merged_time = None if merged_map_stamp is None else round(merged_map_stamp - first_seen, 3)
    has_overlap = bool(args.scene_has_overlap)
    false_alignment = runtime_valid and (not has_overlap) and ever_aligned
    missed_alignment = runtime_valid and has_overlap and final_status != "aligned"
    gt_used_runtime = bool(args.gt_used_runtime) or bool(latest_robust.get("gt_used_runtime", False)) or bool(
        latest_pose.get("gt_used_runtime", False)
    ) or bool(latest_status.get("gt_used_runtime", False))
    inferred_keyframes: set[tuple[str, str]] = set()
    for item in latest_robust.get("inliers", []) + latest_robust.get("rejected", []):
        if not isinstance(item, dict):
            continue
        for robot_key, frame_key in (
            ("source_robot", "source_keyframe"),
            ("target_robot", "target_keyframe"),
            ("query_robot", "query_keyframe"),
            ("match_robot", "match_keyframe"),
        ):
            robot = str(item.get(robot_key, ""))
            frame = str(item.get(frame_key, ""))
            if robot and frame:
                inferred_keyframes.add((robot, frame))
    inferred_verified = int(latest_robust.get("raw_verified_matches", 0) or 0)
    inferred_rejected = int(latest_robust.get("raw_rejected_matches", 0) or 0)
    keyframe_count = max(len(keyframes), len(inferred_keyframes))
    verified_count = max(len(accepted), inferred_verified)
    rejected_count = max(len(rejected), inferred_rejected)
    match_total = max(len(matches), verified_count + rejected_count)
    descriptor_candidate_count = max(len(candidates), match_total if len(candidates) == 0 else len(candidates))
    pose_graph_num_factors = int(latest_pose.get("pose_graph_num_factors", 0) or 0)
    if pose_graph_num_factors <= 0:
        pose_graph_num_factors = int(latest_pose.get("num_odom_factors", 0) or 0) + int(
            latest_pose.get("num_inter_robot_factors_inlier", 0) or 0
        )

    gt_path = Path(args.gt_transform_json) if args.gt_transform_json else None
    summary: dict[str, Any] = {
        "schema": "cross_loop_closure_summary/v2",
        "scene_name": args.scene_name,
        "trial_id": int(args.trial_id),
        "profile": args.profile,
        "relative_pose_source": args.relative_pose_source,
        "gt_used_runtime": gt_used_runtime,
        "runtime_valid": runtime_valid,
        "runtime_error": runtime_error,
        "keyframes": keyframe_count,
        "descriptor_candidates": descriptor_candidate_count,
        "verified_matches": verified_count,
        "cross_robot_matches_total": match_total,
        "rejected_matches": rejected_count,
        "false_positive_rejection_count": rejected_count,
        "recorder_inferred_counts": {
            "keyframes": keyframe_count > len(keyframes),
            "matches": match_total > len(matches),
            "descriptor_candidates": descriptor_candidate_count > len(candidates),
        },
        "robust_inlier_set_size": int(latest_robust.get("robust_inlier_set_size", 0) or 0),
        "robust_inlier_ratio": float(latest_robust.get("robust_inlier_ratio", 0.0) or 0.0),
        "robust_inlier_ratio_raw": float(
            latest_robust.get("robust_inlier_ratio_raw", latest_robust.get("robust_inlier_ratio", 0.0)) or 0.0
        ),
        "robust_inlier_ratio_eligible": float(
            latest_robust.get("robust_inlier_ratio_eligible", latest_robust.get("robust_inlier_ratio", 0.0)) or 0.0
        ),
        "eligible_matches": int(latest_robust.get("eligible_matches", inferred_verified) or 0),
        "prefiltered_matches": int(
            latest_robust.get("prefiltered_matches", latest_robust.get("eligible_matches", inferred_verified)) or 0
        ),
        "deduplicated_matches": int(latest_robust.get("deduplicated_matches", inferred_verified) or 0),
        "raw_consistent_matches": int(latest_robust.get("raw_consistent_matches", 0) or 0),
        "robust_rejected_matches": int(latest_robust.get("robust_rejected_matches", 0) or 0),
        "median_rmse": float(latest_robust.get("median_rmse", 999.0) or 999.0),
        "median_descriptor_distance": float(
            latest_robust.get("median_descriptor_distance", 999.0) or 999.0
        ),
        "median_inlier_ratio": float(latest_robust.get("median_inlier_ratio", 0.0) or 0.0),
        "transform_spread_translation": float(
            latest_robust.get(
                "transform_spread_translation",
                latest_robust.get("translation_spread_m", 999.0),
            ) or 999.0
        ),
        "transform_spread_yaw_deg": float(
            latest_robust.get(
                "transform_spread_yaw_deg",
                latest_robust.get("yaw_spread_deg", 999.0),
            ) or 999.0
        ),
        "reject_reason": str(
            latest_robust.get("reject_reason", latest_status.get("reject_reason", latest_status.get("reason", "")))
        ),
        "consistent_matches": int(latest_robust.get("robust_inlier_set_size", 0) or 0),
        "alignment_status": final_status,
        "time_to_alignment_sec": time_to_alignment,
        "merged_map_enabled_time_sec": merged_time,
        "pose_graph_num_factors": pose_graph_num_factors,
        "pose_graph_optimization_success": bool(latest_pose.get("optimization_success", False)),
        "pose_graph_optimization_backend": latest_pose.get("optimization_backend", "unknown"),
        "pose_graph_error_before": latest_pose.get("pose_graph_error_before"),
        "pose_graph_error_after": latest_pose.get("pose_graph_error_after"),
        "pose_graph_inter_robot_factors": int(
            latest_pose.get("num_inter_robot_factors_inlier", 0) or 0
        ),
        "dependency_blocker": str(latest_pose.get("dependency_blocker", "")),
        "dynamic_points_filtered": int(latest_dynamic.get("dynamic_points_filtered", 0) or 0),
        "static_points_kept": int(latest_dynamic.get("static_points_kept", 0) or 0),
        "dynamic_filter_ratio": float(latest_dynamic.get("dynamic_filter_ratio", 0.0) or 0.0),
        "dynamic_filter_metrics_count": len(dynamic_metrics),
        "peer_status": latest_peer,
        "peer_envelopes": len(peer_envelopes),
        "alignment_symmetry_error_translation": latest_peer.get("alignment_symmetry_error_translation"),
        "alignment_symmetry_error_yaw": latest_peer.get("alignment_symmetry_error_yaw"),
        "false_alignment": false_alignment,
        "missed_alignment": missed_alignment,
        "inter_robot_rendezvous_candidates": _count_inter_robot_rendezvous(loop_candidates),
        "gt_eval_only_alignment_error": _alignment_error(statuses, gt_path),
    }
    (root / "cross_loop_closure_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_markdown(root / "cross_loop_closure_summary.md", summary)
    (root / "cross_loop_closure_final_eval.json").write_text(
        json.dumps([summary], indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_final_eval_markdown(root / "cross_loop_closure_final_eval.md", [summary])
    return summary


def _write_final_eval_markdown(path: Path, rows: list[dict[str, Any]]) -> None:
    false_count = sum(1 for r in rows if r.get("false_alignment"))
    missed_count = sum(1 for r in rows if r.get("missed_alignment"))
    lines = [
        "# Cross-Robot Loop Closure v2 Final Eval",
        "",
        f"- runs: `{len(rows)}`",
        f"- false_alignment_count: `{false_count}`",
        f"- missed_alignment_count: `{missed_count}`",
        f"- gt_used_runtime_any: `{any(bool(r.get('gt_used_runtime')) for r in rows)}`",
        f"- runtime_invalid_count: `{sum(1 for r in rows if not bool(r.get('runtime_valid', True)))}`",
        "",
        "| scene | trial | profile | status | robust_inliers | pose_graph | false | missed |",
        "|---|---:|---|---|---:|---|---|---|",
    ]
    for row in rows:
        lines.append(
            "| "
            f"{row.get('scene_name')} | {row.get('trial_id')} | {row.get('profile')} | "
            f"{row.get('alignment_status')} | {row.get('robust_inlier_set_size')} | "
            f"{row.get('pose_graph_optimization_backend')}:{row.get('pose_graph_optimization_success')} | "
            f"{row.get('false_alignment')} | {row.get('missed_alignment')} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def aggregate_runs(root: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(root.rglob("cross_loop_closure_summary.json")):
        if path.parent == root:
            continue
        try:
            obj = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            continue
        if isinstance(obj, dict):
            rows.append(obj)
    rows.sort(key=lambda r: (str(r.get("scene_name")), str(r.get("profile")), int(r.get("trial_id", 0))))
    (root / "cross_loop_closure_final_eval.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    _write_final_eval_markdown(root / "cross_loop_closure_final_eval.md", rows)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", required=True)
    ap.add_argument("--relative-pose-source", default="discovered")
    ap.add_argument("--gt-used-runtime", action="store_true")
    ap.add_argument("--gt-transform-json", default="")
    ap.add_argument("--scene-name", default="demo3_mixed.xml")
    ap.add_argument("--scene-has-overlap", action="store_true")
    ap.add_argument("--trial-id", default="1")
    ap.add_argument("--profile", default="robust")
    ap.add_argument("--aggregate-root", action="store_true")
    args = ap.parse_args()

    root = Path(args.log_dir)
    if args.aggregate_root:
        rows = aggregate_runs(root)
        print(json.dumps(rows, indent=2, sort_keys=True))
    else:
        summary = summarize_run(args)
        print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
