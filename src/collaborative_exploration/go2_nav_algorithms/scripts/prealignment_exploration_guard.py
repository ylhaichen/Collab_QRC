#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, String

from go2_nav_algorithms.prealignment_policy import (
    GoalSample,
    PrealignmentConfig,
    PrealignmentPolicy,
)


def _stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _loads(data: str) -> dict[str, Any]:
    try:
        obj = json.loads(data)
    except json.JSONDecodeError:
        return {}
    return obj if isinstance(obj, dict) else {}


class PrealignmentExplorationGuard(Node):
    """Local-goal gate for unknown-pose exploration before team alignment."""

    def __init__(self) -> None:
        super().__init__("prealignment_exploration_guard")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("input_goal_topic", "way_point_coord_raw")
        self.declare_parameter("output_goal_topic", "way_point_coord")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("frontier_replan_topic", "frontier_replan")
        self.declare_parameter("nav_status_topic", "nav_status")
        self.declare_parameter("alignment_status_topic", "/team_slam/alignment_status")
        self.declare_parameter("keyframe_topic", "/team_slam/keyframes")
        self.declare_parameter("candidate_topic", "/team_slam/cross_robot_candidates")
        self.declare_parameter("robust_inliers_topic", "/team_slam/robust_loop_inliers")
        self.declare_parameter("status_topic", "prealignment_exploration_status")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("frontier_stride", 3)
        self.declare_parameter("max_frontiers", 80)
        self.declare_parameter("prealign_goal_hold_sec", 5.0)

        self.declare_parameter("prealign_min_goal_distance", 2.0)
        self.declare_parameter("prealign_min_start_displacement", 3.0)
        self.declare_parameter("prealign_dwell_timeout_sec", 20.0)
        self.declare_parameter("prealign_stuck_replan_limit", 3)
        self.declare_parameter("prealign_goal_blacklist_radius", 1.0)
        self.declare_parameter("prealign_overlap_timeout_sec", 60.0)
        self.declare_parameter("prealign_min_keyframes_before_alignment", 5)
        self.declare_parameter("prealign_exploration_radius_growth", 1.5)
        self.declare_parameter("prealign_far_frontier_bonus", 1.0)
        self.declare_parameter("prealign_corridor_frontier_bonus", 0.5)
        self.declare_parameter("prealign_keyframe_gain_bonus", 0.5)

        ns = str(self.get_parameter("namespace").value).strip().strip("/")
        self.ns = ns or "robot"
        cfg = PrealignmentConfig(
            min_goal_distance=float(self.get_parameter("prealign_min_goal_distance").value),
            min_start_displacement=float(self.get_parameter("prealign_min_start_displacement").value),
            dwell_timeout_sec=float(self.get_parameter("prealign_dwell_timeout_sec").value),
            stuck_replan_limit=int(self.get_parameter("prealign_stuck_replan_limit").value),
            goal_blacklist_radius=float(self.get_parameter("prealign_goal_blacklist_radius").value),
            overlap_timeout_sec=float(self.get_parameter("prealign_overlap_timeout_sec").value),
            min_keyframes_before_alignment=int(
                self.get_parameter("prealign_min_keyframes_before_alignment").value
            ),
            exploration_radius_growth=float(
                self.get_parameter("prealign_exploration_radius_growth").value
            ),
            far_frontier_bonus=float(self.get_parameter("prealign_far_frontier_bonus").value),
            corridor_frontier_bonus=float(self.get_parameter("prealign_corridor_frontier_bonus").value),
            keyframe_gain_bonus=float(self.get_parameter("prealign_keyframe_gain_bonus").value),
        )
        self.policy = PrealignmentPolicy(robot_id=self.ns, config=cfg)

        in_goal = self._ns_topic(str(self.get_parameter("input_goal_topic").value))
        out_goal = self._ns_topic(str(self.get_parameter("output_goal_topic").value))
        odom_topic = self._ns_topic(str(self.get_parameter("odom_topic").value))
        map_topic = self._ns_topic(str(self.get_parameter("map_topic").value))
        replan_topic = self._ns_topic(str(self.get_parameter("frontier_replan_topic").value))
        nav_status_topic = self._ns_topic(str(self.get_parameter("nav_status_topic").value))
        status_topic = self._ns_topic(str(self.get_parameter("status_topic").value))
        self.output_frame_id = str(self.get_parameter("output_frame_id").value).strip() or "map"
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_frontiers = max(1, int(self.get_parameter("max_frontiers").value))
        self.prealign_goal_hold_sec = max(0.0, float(self.get_parameter("prealign_goal_hold_sec").value))

        self.latest_odom: Odometry | None = None
        self.latest_map: OccupancyGrid | None = None
        self.latest_goal: GoalSample | None = None
        self.keyframes_by_robot: dict[str, int] = {}
        self.cross_robot_candidates = 0
        self.verified_matches = 0
        self.robust_inliers = 0
        self.alignment_status = "unaligned"
        self.gt_used_runtime = False
        self.last_publish_sec = 0.0
        self.last_output_goal: GoalSample | None = None

        self.create_subscription(PointStamped, in_goal, self._on_goal, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 2)
        self.create_subscription(Empty, replan_topic, self._on_replan, 10)
        self.create_subscription(String, nav_status_topic, self._on_nav_status, 10)
        self.create_subscription(String, str(self.get_parameter("alignment_status_topic").value), self._on_alignment_status, 10)
        self.create_subscription(String, str(self.get_parameter("keyframe_topic").value), self._on_keyframe, 10)
        self.create_subscription(String, str(self.get_parameter("candidate_topic").value), self._on_candidate, 10)
        self.create_subscription(String, str(self.get_parameter("robust_inliers_topic").value), self._on_robust, 10)

        self.goal_pub = self.create_publisher(PointStamped, out_goal, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "prealignment_exploration_guard active: "
            f"ns={self.ns} {in_goal} -> {out_goal} frame={self.output_frame_id} "
            f"min_goal={cfg.min_goal_distance:.2f}m min_start={cfg.min_start_displacement:.2f}m"
        )

    def _ns_topic(self, suffix: str) -> str:
        clean = str(suffix or "").strip()
        if clean.startswith("/"):
            return clean
        return f"/{self.ns}/{clean.lstrip('/')}"

    def _now_sec(self) -> float:
        msg = self.get_clock().now().to_msg()
        return _stamp_to_sec(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        p = msg.pose.pose.position
        self.policy.update_pose(float(p.x), float(p.y), stamp_sec=self._now_sec())

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        known = sum(1 for v in msg.data if int(v) >= 0)
        self.policy.update_map_coverage_cells(known)

    def _on_goal(self, msg: PointStamped) -> None:
        incoming = GoalSample(
            float(msg.point.x),
            float(msg.point.y),
            frame_id=msg.header.frame_id or self.output_frame_id,
        )
        self.latest_goal = incoming
        self._publish_decision(incoming, force=True)

    def _on_replan(self, _: Empty) -> None:
        if self.latest_goal is not None:
            self.policy.mark_goal_failed(self.latest_goal, stamp_sec=self._now_sec())
        self._publish_decision(None, force=True, override_hold=True)

    def _on_nav_status(self, msg: String) -> None:
        text = str(msg.data).lower()
        if not any(token in text for token in ("failed", "stuck", "no_plan", "out_of_bounds")):
            return
        if self.latest_goal is not None:
            self.policy.mark_goal_failed(self.latest_goal, stamp_sec=self._now_sec())
        self._publish_decision(None, force=True, override_hold=True)

    def _on_alignment_status(self, msg: String) -> None:
        payload = _loads(msg.data)
        status = str(payload.get("status", "unaligned"))
        self.alignment_status = status
        self.gt_used_runtime = bool(payload.get("gt_used_runtime", False))
        self.verified_matches = max(
            self.verified_matches,
            int(payload.get("accepted_count", payload.get("verified_matches", 0)) or 0),
        )
        self.robust_inliers = max(
            self.robust_inliers,
            int(payload.get("inlier_count", payload.get("robust_inliers", 0)) or 0),
        )
        self.policy.update_alignment_status(
            status=status,
            cross_robot_candidates=self.cross_robot_candidates,
            verified_matches=self.verified_matches,
            robust_inliers=self.robust_inliers,
            stamp_sec=self._now_sec(),
        )

    def _on_keyframe(self, msg: String) -> None:
        payload = _loads(msg.data)
        robot = str(payload.get("robot", payload.get("robot_id", ""))).strip().strip("/")
        if not robot:
            return
        current = self.keyframes_by_robot.get(robot, 0) + 1
        self.keyframes_by_robot[robot] = current
        if robot == self.ns:
            self.policy.update_keyframe_count(current)

    def _on_candidate(self, msg: String) -> None:
        payload = _loads(msg.data)
        if payload.get("schema") == "team_cross_robot_candidate/v1":
            self.cross_robot_candidates += 1

    def _on_robust(self, msg: String) -> None:
        payload = _loads(msg.data)
        self.verified_matches = max(
            self.verified_matches,
            int(payload.get("raw_verified_matches", 0) or 0),
        )
        self.robust_inliers = max(
            self.robust_inliers,
            int(payload.get("robust_inlier_set_size", 0) or 0),
        )

    def _tick(self) -> None:
        self._publish_status()
        if self.latest_odom is None:
            return
        now = self._now_sec()
        if now - self.last_publish_sec < 2.0:
            return
        if (
            self.policy.phase.value
            in {"unaligned_local_explore", "overlap_seeking", "tentative_alignment", "rejected_recover"}
            and self.policy.metrics.distance_from_start < self.policy.config.min_start_displacement
        ):
            self._publish_decision(self.latest_goal, force=False)

    def _publish_decision(
        self,
        incoming: GoalSample | None,
        *,
        force: bool,
        override_hold: bool = False,
    ) -> None:
        if self.latest_odom is None:
            return
        now = self._now_sec()
        if (
            not override_hold
            and self.last_output_goal is not None
            and self.policy.phase.value != "aligned_shared_explore"
            and now - self.last_publish_sec < self.prealign_goal_hold_sec
        ):
            return
        frontiers = self._extract_local_frontiers()
        decision = self.policy.choose_goal(
            incoming=incoming,
            local_frontiers=frontiers,
            stamp_sec=now,
        )
        if decision.goal is None:
            return
        if not force and decision.reason == "local_goal_allowed":
            return
        self._publish_goal(decision.goal)
        self.get_logger().info(
            f"{self.ns}: prealign_goal reason={decision.reason} "
            f"phase={decision.phase.value} goal=({decision.goal.x:+.2f},{decision.goal.y:+.2f})"
        )

    def _publish_goal(self, goal: GoalSample) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.point.x = float(goal.x)
        msg.point.y = float(goal.y)
        msg.point.z = 0.0
        self.goal_pub.publish(msg)
        self.last_publish_sec = self._now_sec()
        self.latest_goal = goal
        self.last_output_goal = goal

    def _extract_local_frontiers(self) -> list[GoalSample]:
        msg = self.latest_map
        odom = self.latest_odom
        if msg is None or odom is None or msg.info.resolution <= 0.0:
            return []
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 2 or height <= 2:
            return []
        data = list(msg.data)
        rx = float(odom.pose.pose.position.x)
        ry = float(odom.pose.pose.position.y)
        out: list[GoalSample] = []
        for gy in range(1, height - 1, self.frontier_stride):
            row = gy * width
            for gx in range(1, width - 1, self.frontier_stride):
                value = int(data[row + gx])
                if value != 0:
                    continue
                has_unknown = False
                occupied_near = False
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        n = int(data[(gy + dy) * width + gx + dx])
                        if n < 0:
                            has_unknown = True
                        if n >= 50:
                            occupied_near = True
                if not has_unknown or occupied_near:
                    continue
                wx = float(msg.info.origin.position.x) + (gx + 0.5) * float(msg.info.resolution)
                wy = float(msg.info.origin.position.y) + (gy + 0.5) * float(msg.info.resolution)
                distance = math.hypot(wx - rx, wy - ry)
                if distance < self.policy.config.min_goal_distance:
                    continue
                corridor_score = self._corridor_score(data, width, height, gx, gy)
                out.append(
                    GoalSample(
                        wx,
                        wy,
                        frame_id=f"{self.ns}/map",
                        frontier_size=1.0,
                        corridor_score=corridor_score,
                        keyframe_gain=distance,
                    )
                )
        return sorted(out, key=lambda g: math.hypot(g.x - rx, g.y - ry), reverse=True)[: self.max_frontiers]

    @staticmethod
    def _corridor_score(data: list[int], width: int, height: int, gx: int, gy: int) -> float:
        free_dirs = 0
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            seen_free = False
            for step in range(1, 5):
                nx = gx + dx * step
                ny = gy + dy * step
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    break
                if int(data[ny * width + nx]) == 0:
                    seen_free = True
            if seen_free:
                free_dirs += 1
        return float(free_dirs)

    def _publish_status(self) -> None:
        m = self.policy.metrics
        payload = {
            "schema": "prealignment_exploration_status/v1",
            "robot_id": self.ns,
            "alignment_state": self.policy.phase.value,
            "alignment_status": self.alignment_status,
            "prealign_min_start_displacement": float(self.policy.config.min_start_displacement),
            "distance_from_start": round(float(m.distance_from_start), 4),
            "max_distance_from_start": round(float(m.max_distance_from_start), 4),
            "path_length": round(float(m.path_length), 4),
            "keyframes": int(m.keyframes),
            "map_coverage_cells": int(m.map_coverage_cells),
            "local_frontiers_selected": int(m.local_frontiers_selected),
            "goals_rejected_as_too_close": int(m.goals_rejected_as_too_close),
            "stuck_replans": int(m.stuck_replans),
            "blacklisted_goals": int(m.blacklisted_goals),
            "goal_failure_count": int(m.goal_failure_count),
            "peer_frame_goals_blocked": int(m.peer_frame_goals_blocked),
            "cross_robot_candidates": int(self.cross_robot_candidates),
            "verified_matches": int(self.verified_matches),
            "robust_inliers": int(self.robust_inliers),
            "gt_used_runtime": bool(self.gt_used_runtime),
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PrealignmentExplorationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
