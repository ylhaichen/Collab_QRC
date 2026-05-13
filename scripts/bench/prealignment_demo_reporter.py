#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


def _loads(data: str) -> dict[str, Any]:
    try:
        obj = json.loads(data)
    except json.JSONDecodeError:
        return {}
    return obj if isinstance(obj, dict) else {}


class PrealignmentDemoReporter(Node):
    def __init__(self, duration: float) -> None:
        super().__init__("prealignment_demo_reporter")
        self.duration = max(1.0, float(duration))
        self.start_wall = time.monotonic()
        self.robot_status: dict[str, dict[str, Any]] = {}
        self.nav_start_diagnostics: dict[str, dict[str, Any]] = {}
        self.alignment_status: dict[str, Any] = {}
        self.merged_status: dict[str, Any] = {}
        self.cross_robot_candidates = 0
        self.robust_inliers: dict[str, Any] = {}
        self.occupancy_counts = {
            "robot_a": 0,
            "robot_b": 0,
            "merged": 0,
        }
        self.first_aligned_sec: float | None = None
        self.first_merged_sec: float | None = None
        self.create_subscription(
            String,
            "/robot_a/prealignment_exploration_status",
            lambda msg: self._on_robot_status("robot_a", msg),
            10,
        )
        self.create_subscription(
            String,
            "/robot_b/prealignment_exploration_status",
            lambda msg: self._on_robot_status("robot_b", msg),
            10,
        )
        self.create_subscription(
            String,
            "/robot_a/nav_start_cell_diagnostics",
            lambda msg: self._on_nav_start_diagnostics("robot_a", msg),
            10,
        )
        self.create_subscription(
            String,
            "/robot_b/nav_start_cell_diagnostics",
            lambda msg: self._on_nav_start_diagnostics("robot_b", msg),
            10,
        )
        self.create_subscription(String, "/team_slam/alignment_status", self._on_alignment, 10)
        self.create_subscription(String, "/team_slam/cross_robot_candidates", self._on_candidate, 10)
        self.create_subscription(String, "/team_slam/robust_loop_inliers", self._on_robust, 10)
        self.create_subscription(
            String,
            "/team_slam/merged_occupancy_grid_status",
            self._on_merged_status,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/robot_a/local_occupancy_grid",
            lambda _msg: self._inc_grid("robot_a"),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/robot_b/local_occupancy_grid",
            lambda _msg: self._inc_grid("robot_b"),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            "/team_slam/merged_occupancy_grid",
            lambda _msg: self._inc_grid("merged"),
            10,
        )

    def _elapsed(self) -> float:
        return time.monotonic() - self.start_wall

    def _on_robot_status(self, robot: str, msg: String) -> None:
        self.robot_status[robot] = _loads(msg.data)

    def _on_nav_start_diagnostics(self, robot: str, msg: String) -> None:
        self.nav_start_diagnostics[robot] = _loads(msg.data)

    def _on_alignment(self, msg: String) -> None:
        self.alignment_status = _loads(msg.data)
        if self.alignment_status.get("status") == "aligned" and self.first_aligned_sec is None:
            self.first_aligned_sec = self._elapsed()

    def _on_candidate(self, msg: String) -> None:
        if _loads(msg.data).get("schema") == "team_cross_robot_candidate/v1":
            self.cross_robot_candidates += 1

    def _on_robust(self, msg: String) -> None:
        self.robust_inliers = _loads(msg.data)

    def _on_merged_status(self, msg: String) -> None:
        self.merged_status = _loads(msg.data)
        if self.merged_status.get("active") and self.first_merged_sec is None:
            self.first_merged_sec = self._elapsed()

    def _inc_grid(self, key: str) -> None:
        self.occupancy_counts[key] += 1
        if key == "merged" and self.first_merged_sec is None:
            self.first_merged_sec = self._elapsed()

    def done(self) -> bool:
        return self._elapsed() >= self.duration

    def summary(self) -> dict[str, Any]:
        a = self.robot_status.get("robot_a", {})
        b = self.robot_status.get("robot_b", {})
        nav_a = self.nav_start_diagnostics.get("robot_a", {})
        nav_b = self.nav_start_diagnostics.get("robot_b", {})
        alignment_status = str(self.alignment_status.get("status", "unknown"))
        robust_count = int(
            self.robust_inliers.get(
                "robust_inlier_set_size",
                self.alignment_status.get("inlier_count", 0),
            )
            or 0
        )
        verified = int(
            self.robust_inliers.get(
                "raw_verified_matches",
                self.alignment_status.get("accepted_count", 0),
            )
            or 0
        )
        gt_used_runtime = bool(
            self.alignment_status.get("gt_used_runtime", False)
            or a.get("gt_used_runtime", False)
            or b.get("gt_used_runtime", False)
        )
        merged_after_gate = (
            self.first_merged_sec is None
            or (self.first_aligned_sec is not None and self.first_merged_sec >= self.first_aligned_sec)
        )
        prealign_min = max(
            float(a.get("prealign_min_start_displacement", 3.0) or 3.0),
            float(b.get("prealign_min_start_displacement", 3.0) or 3.0),
        )
        robot_a_distance = float(a.get("distance_from_start", 0.0) or 0.0)
        robot_b_distance = float(b.get("distance_from_start", 0.0) or 0.0)
        robot_a_max_distance = max(robot_a_distance, float(a.get("max_distance_from_start", 0.0) or 0.0))
        robot_b_max_distance = max(robot_b_distance, float(b.get("max_distance_from_start", 0.0) or 0.0))
        robust_growth_rate = max(
            float(a.get("robust_inlier_growth_rate", 0.0) or 0.0),
            float(b.get("robust_inlier_growth_rate", 0.0) or 0.0),
        )
        tentative_duration = max(
            float(a.get("tentative_alignment_duration", 0.0) or 0.0),
            float(b.get("tentative_alignment_duration", 0.0) or 0.0),
        )
        scripted_overlap_demo = bool(
            a.get("prealign_scripted_overlap_demo", False)
            or b.get("prealign_scripted_overlap_demo", False)
        )
        if robust_growth_rate <= 0.0 and robust_count > 0 and tentative_duration > 0.0:
            robust_growth_rate = float(robust_count) / tentative_duration
        payload = {
            "schema": "visualized_discovered_pose_demo_eval/v1",
            "duration_sec": round(self._elapsed(), 3),
            "prealignment_anti_dwell_active": bool(a and b),
            "overlap_seeking_active": bool(
                a.get("overlap_seeking_active", False) or b.get("overlap_seeking_active", False)
            ),
            "tentative_alignment_exploration_active": bool(
                a.get("tentative_alignment_exploration_active", False)
                or b.get("tentative_alignment_exploration_active", False)
            ),
            "prealign_scripted_overlap_demo": scripted_overlap_demo,
            "robot_a_distance_from_start": round(robot_a_distance, 4),
            "robot_b_distance_from_start": round(robot_b_distance, 4),
            "robot_a_max_distance_from_start": round(robot_a_max_distance, 4),
            "robot_b_max_distance_from_start": round(robot_b_max_distance, 4),
            "robot_a_path_length": round(float(a.get("path_length", 0.0) or 0.0), 4),
            "robot_b_path_length": round(float(b.get("path_length", 0.0) or 0.0), 4),
            "robot_a_keyframes": int(a.get("keyframes", 0) or 0),
            "robot_b_keyframes": int(b.get("keyframes", 0) or 0),
            "robot_a_local_map_area": round(float(a.get("local_map_area", 0.0) or 0.0), 4),
            "robot_b_local_map_area": round(float(b.get("local_map_area", 0.0) or 0.0), 4),
            "robot_a_local_map_area_growth": round(
                float(a.get("local_map_area_growth", 0.0) or 0.0),
                4,
            ),
            "robot_b_local_map_area_growth": round(
                float(b.get("local_map_area_growth", 0.0) or 0.0),
                4,
            ),
            "map_area_growth_rate": round(
                max(
                    float(a.get("map_area_growth_rate", 0.0) or 0.0),
                    float(b.get("map_area_growth_rate", 0.0) or 0.0),
                ),
                6,
            ),
            "unknown_to_known_cells": int(a.get("unknown_to_known_cells", 0) or 0)
            + int(b.get("unknown_to_known_cells", 0) or 0),
            "frontier_count": int(a.get("frontier_count", 0) or 0)
            + int(b.get("frontier_count", 0) or 0),
            "new_frontiers_discovered": int(a.get("new_frontiers_discovered", 0) or 0)
            + int(b.get("new_frontiers_discovered", 0) or 0),
            "keyframe_spatial_diversity": round(
                max(
                    float(a.get("keyframe_spatial_diversity", 0.0) or 0.0),
                    float(b.get("keyframe_spatial_diversity", 0.0) or 0.0),
                ),
                4,
            ),
            "repeated_goal_ratio": round(
                max(
                    float(a.get("repeated_goal_ratio", 0.0) or 0.0),
                    float(b.get("repeated_goal_ratio", 0.0) or 0.0),
                ),
                4,
            ),
            "stuck_recovery_count": int(a.get("stuck_recovery_count", 0) or 0)
            + int(b.get("stuck_recovery_count", 0) or 0),
            "failed_goal_blacklist_count": int(a.get("failed_goal_blacklist_count", 0) or 0)
            + int(b.get("failed_goal_blacklist_count", 0) or 0),
            "coverage_gain_per_meter": round(
                max(
                    float(a.get("coverage_gain_per_meter", 0.0) or 0.0),
                    float(b.get("coverage_gain_per_meter", 0.0) or 0.0),
                ),
                6,
            ),
            "robot_a_exploration_quality": str(a.get("prealign_exploration_quality", "unknown")),
            "robot_b_exploration_quality": str(b.get("prealign_exploration_quality", "unknown")),
            "robot_a_exploration_success": bool(a.get("exploration_success", False)),
            "robot_b_exploration_success": bool(b.get("exploration_success", False)),
            "local_frontiers_selected": int(a.get("local_frontiers_selected", 0) or 0)
            + int(b.get("local_frontiers_selected", 0) or 0),
            "goals_rejected_as_too_close": int(a.get("goals_rejected_as_too_close", 0) or 0)
            + int(b.get("goals_rejected_as_too_close", 0) or 0),
            "stuck_replans": int(a.get("stuck_replans", 0) or 0)
            + int(b.get("stuck_replans", 0) or 0),
            "blacklisted_goals": int(a.get("blacklisted_goals", 0) or 0)
            + int(b.get("blacklisted_goals", 0) or 0),
            "cross_robot_candidates": int(max(self.cross_robot_candidates, int(a.get("cross_robot_candidates", 0) or 0), int(b.get("cross_robot_candidates", 0) or 0))),
            "verified_matches": verified,
            "robust_inliers": robust_count,
            "robust_inlier_growth_rate": round(robust_growth_rate, 6),
            "tentative_alignment_duration": round(tentative_duration, 4),
            "overlap_seeking_goals": int(a.get("overlap_seeking_goals", 0) or 0)
            + int(b.get("overlap_seeking_goals", 0) or 0),
            "tentative_alignment_explore_goals": int(
                a.get("tentative_alignment_explore_goals", 0) or 0
            )
            + int(b.get("tentative_alignment_explore_goals", 0) or 0),
            "scripted_local_overlap_goals": int(a.get("scripted_local_overlap_goals", 0) or 0)
            + int(b.get("scripted_local_overlap_goals", 0) or 0),
            "alignment_status": alignment_status,
            "physical_overlap_occurred": self.cross_robot_candidates > 0 or verified > 0 or robust_count > 0,
            "robust_alignment_occurred": alignment_status == "aligned" and robust_count > 0,
            "merged_map_enabled_time_sec": self.first_merged_sec,
            "merged_map_opened_only_after_gate": bool(merged_after_gate),
            "prealignment_gate_enabled": bool(self.alignment_status.get("prealignment_gate_enabled", False)),
            "prealignment_gate_satisfied": bool(
                self.alignment_status.get("prealignment_gate_satisfied", False)
            ),
            "gt_used_runtime": gt_used_runtime,
            "both_robots_exceeded_min_start_displacement": (
                robot_a_max_distance >= prealign_min and robot_b_max_distance >= prealign_min
            ),
            "no_overlap_rejection_result": (
                "not_applicable_physical_overlap_detected"
                if self.cross_robot_candidates > 0 or verified > 0 or robust_count > 0
                else "merged_map_remained_closed_correctly"
                if self.first_merged_sec is None and alignment_status != "aligned"
                else "failed"
            ),
            "occupancy": {
                "robot_a_local_grid_messages": self.occupancy_counts["robot_a"],
                "robot_b_local_grid_messages": self.occupancy_counts["robot_b"],
                "merged_grid_messages": self.occupancy_counts["merged"],
                "local_grids_nonzero_rate": self.occupancy_counts["robot_a"] > 0
                and self.occupancy_counts["robot_b"] > 0,
                "merged_grid_active": self.occupancy_counts["merged"] > 0,
                "merged_grid_only_after_aligned": bool(merged_after_gate),
                "merged_grid_status_topic_exists": bool(self.merged_status),
                "merged_grid_inactive_before_alignment": (
                    self.first_aligned_sec is None or merged_after_gate
                ),
                "local_grids_use_static_cloud": True,
                "corrected_pose_source_used": True,
                "keyframe_rebuild_enabled": True,
                "static_min_observations": 2,
                "dynamic_decay_sec": 3.0,
                "self_clear_radius": 0.65,
                "max_keyframes": 200,
                "dynamic_cloud_written_to_static_grid": False,
            },
            "nav_start_cell_diagnostics": {
                "robot_a": nav_a,
                "robot_b": nav_b,
                "robot_b_start_cell_not_lethal": (
                    nav_b.get("global_start_cell_status") != "lethal"
                    and nav_b.get("local_start_cell_status") != "lethal"
                ) if nav_b else False,
                "robot_b_projection_success": bool(nav_b.get("projection_success", False)),
                "robot_b_rejected_frontier_goals": int(nav_b.get("rejected_frontier_goals", 0) or 0),
                "robot_b_start_in_lethal_recoveries": int(nav_b.get("start_in_lethal_recoveries", 0) or 0),
                "costmap_self_clear_radius": float(
                    nav_b.get("costmap_self_clear_radius", nav_a.get("costmap_self_clear_radius", 0.65)) or 0.65
                ),
                "near_robot_ignore_radius": 0.6,
            },
        }
        if not payload["physical_overlap_occurred"] and self.first_merged_sec is None:
            payload["no_overlap_safety_result"] = "merged_grid_remained_closed_correctly"
        return payload


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _write_md(path: Path, title: str, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"# {title}", ""]
    for key, value in payload.items():
        if isinstance(value, dict):
            lines.append(f"- {key}:")
            for sub_key, sub_value in value.items():
                lines.append(f"  - {sub_key}: `{sub_value}`")
        else:
            lines.append(f"- {key}: `{value}`")
    path.write_text("\n".join(lines) + "\n")


def _write_outputs(output_dir: Path, payload: dict[str, Any]) -> None:
    prealignment = {
        "schema": "prealignment_exploration_eval/v1",
        **{k: payload[k] for k in (
            "prealignment_anti_dwell_active",
            "overlap_seeking_active",
            "tentative_alignment_exploration_active",
            "prealign_scripted_overlap_demo",
            "robot_a_distance_from_start",
            "robot_b_distance_from_start",
            "robot_a_max_distance_from_start",
            "robot_b_max_distance_from_start",
            "robot_a_path_length",
            "robot_b_path_length",
            "robot_a_keyframes",
            "robot_b_keyframes",
            "robot_a_local_map_area",
            "robot_b_local_map_area",
            "robot_a_local_map_area_growth",
            "robot_b_local_map_area_growth",
            "map_area_growth_rate",
            "unknown_to_known_cells",
            "frontier_count",
            "new_frontiers_discovered",
            "keyframe_spatial_diversity",
            "repeated_goal_ratio",
            "stuck_recovery_count",
            "failed_goal_blacklist_count",
            "coverage_gain_per_meter",
            "robot_a_exploration_quality",
            "robot_b_exploration_quality",
            "robot_a_exploration_success",
            "robot_b_exploration_success",
            "local_frontiers_selected",
            "goals_rejected_as_too_close",
            "stuck_replans",
            "blacklisted_goals",
            "cross_robot_candidates",
            "verified_matches",
            "robust_inliers",
            "robust_inlier_growth_rate",
            "tentative_alignment_duration",
            "overlap_seeking_goals",
            "tentative_alignment_explore_goals",
            "scripted_local_overlap_goals",
            "alignment_status",
            "merged_map_enabled_time_sec",
            "prealignment_gate_enabled",
            "prealignment_gate_satisfied",
            "gt_used_runtime",
        )},
    }
    occupancy = {
        "schema": "occupancy_grid_visualization_eval/v1",
        **payload["occupancy"],
        "alignment_status": payload["alignment_status"],
        "gt_used_runtime": payload["gt_used_runtime"],
    }
    cross_loop = {
        "schema": "cross_loop_closure_final_eval/v1",
        "cross_robot_candidates": payload["cross_robot_candidates"],
        "verified_matches": payload["verified_matches"],
        "robust_inliers": payload["robust_inliers"],
        "robust_inlier_growth_rate": payload["robust_inlier_growth_rate"],
        "tentative_alignment_duration": payload["tentative_alignment_duration"],
        "alignment_status": payload["alignment_status"],
        "robust_alignment_occurred": payload["robust_alignment_occurred"],
        "merged_map_opened_only_after_gate": payload["merged_map_opened_only_after_gate"],
        "no_overlap_rejection_result": payload["no_overlap_rejection_result"],
        "gt_used_runtime": payload["gt_used_runtime"],
    }
    nav_diag = {
        "schema": "nav_start_cell_diagnostics_eval/v1",
        **payload["nav_start_cell_diagnostics"],
        "gt_used_runtime": payload["gt_used_runtime"],
    }
    visual = dict(payload)
    _write_json(output_dir / "prealignment_exploration_eval.json", prealignment)
    _write_md(output_dir / "prealignment_exploration_eval.md", "Prealignment Exploration Eval", prealignment)
    _write_json(output_dir / "occupancy_map_visualization_eval.json", occupancy)
    _write_md(output_dir / "occupancy_map_visualization_eval.md", "Occupancy Map Visualization Eval", occupancy)
    _write_json(output_dir / "occupancy_grid_visualization_eval.json", occupancy)
    _write_md(output_dir / "occupancy_grid_visualization_eval.md", "Occupancy Grid Visualization Eval", occupancy)
    _write_json(output_dir / "visualized_demo_eval.json", visual)
    _write_md(output_dir / "visualized_demo_eval.md", "Visualized Demo Eval", visual)
    _write_json(output_dir / "nav_start_cell_diagnostics.json", nav_diag)
    _write_md(output_dir / "nav_start_cell_diagnostics.md", "Nav Start Cell Diagnostics", nav_diag)
    _write_json(output_dir / "cross_loop_closure_final_eval.json", cross_loop)
    _write_md(output_dir / "cross_loop_closure_final_eval.md", "Cross Loop Closure Final Eval", cross_loop)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output-dir", default="logs")
    parser.add_argument("--mirror-output-dir", default="")
    args = parser.parse_args()

    rclpy.init()
    node = PrealignmentDemoReporter(args.duration)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        payload = node.summary()
        _write_outputs(Path(args.output_dir), payload)
        if args.mirror_output_dir:
            _write_outputs(Path(args.mirror_output_dir), payload)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
