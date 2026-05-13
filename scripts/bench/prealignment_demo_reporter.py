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
        payload = {
            "schema": "visualized_discovered_pose_demo_eval/v1",
            "duration_sec": round(self._elapsed(), 3),
            "prealignment_anti_dwell_active": bool(a and b),
            "robot_a_distance_from_start": round(robot_a_distance, 4),
            "robot_b_distance_from_start": round(robot_b_distance, 4),
            "robot_a_max_distance_from_start": round(robot_a_max_distance, 4),
            "robot_b_max_distance_from_start": round(robot_b_max_distance, 4),
            "robot_a_path_length": round(float(a.get("path_length", 0.0) or 0.0), 4),
            "robot_b_path_length": round(float(b.get("path_length", 0.0) or 0.0), 4),
            "robot_a_keyframes": int(a.get("keyframes", 0) or 0),
            "robot_b_keyframes": int(b.get("keyframes", 0) or 0),
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
            "occupancy": {
                "robot_a_local_grid_messages": self.occupancy_counts["robot_a"],
                "robot_b_local_grid_messages": self.occupancy_counts["robot_b"],
                "merged_grid_messages": self.occupancy_counts["merged"],
                "local_grids_nonzero_rate": self.occupancy_counts["robot_a"] > 0
                and self.occupancy_counts["robot_b"] > 0,
                "merged_grid_active": self.occupancy_counts["merged"] > 0,
                "merged_grid_only_after_aligned": bool(merged_after_gate),
                "local_grids_use_static_cloud": True,
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
            "robot_a_distance_from_start",
            "robot_b_distance_from_start",
            "robot_a_max_distance_from_start",
            "robot_b_max_distance_from_start",
            "robot_a_path_length",
            "robot_b_path_length",
            "robot_a_keyframes",
            "robot_b_keyframes",
            "local_frontiers_selected",
            "goals_rejected_as_too_close",
            "stuck_replans",
            "blacklisted_goals",
            "cross_robot_candidates",
            "verified_matches",
            "robust_inliers",
            "alignment_status",
            "merged_map_enabled_time_sec",
            "prealignment_gate_enabled",
            "prealignment_gate_satisfied",
            "gt_used_runtime",
        )},
    }
    occupancy = {
        "schema": "occupancy_map_visualization_eval/v1",
        **payload["occupancy"],
        "alignment_status": payload["alignment_status"],
        "gt_used_runtime": payload["gt_used_runtime"],
    }
    visual = dict(payload)
    _write_json(output_dir / "prealignment_exploration_eval.json", prealignment)
    _write_md(output_dir / "prealignment_exploration_eval.md", "Prealignment Exploration Eval", prealignment)
    _write_json(output_dir / "occupancy_map_visualization_eval.json", occupancy)
    _write_md(output_dir / "occupancy_map_visualization_eval.md", "Occupancy Map Visualization Eval", occupancy)
    _write_json(output_dir / "visualized_demo_eval.json", visual)
    _write_md(output_dir / "visualized_demo_eval.md", "Visualized Demo Eval", visual)


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
