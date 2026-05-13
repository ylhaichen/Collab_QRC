#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray

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
        self.declare_parameter("quality_map_topic", "")
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
        self.declare_parameter("frontier_marker_topic", "frontiers")
        self.declare_parameter("frontier_obstacle_clearance_cells", 2)
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
        self.declare_parameter("prealign_robust_acceptance_min_inliers", 7)
        self.declare_parameter("prealign_scripted_overlap_demo", False)
        self.declare_parameter("prealign_min_path_length", 4.0)
        self.declare_parameter("prealign_min_local_map_area_growth", 1.0)
        self.declare_parameter("prealign_min_keyframe_spatial_diversity", 0.0)
        self.declare_parameter("prealign_max_repeated_goal_ratio", 0.5)

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
            robust_acceptance_min_inliers=int(
                self.get_parameter("prealign_robust_acceptance_min_inliers").value
            ),
            scripted_overlap_demo=bool(self.get_parameter("prealign_scripted_overlap_demo").value),
            min_path_length=float(self.get_parameter("prealign_min_path_length").value),
            min_local_map_area_growth=float(
                self.get_parameter("prealign_min_local_map_area_growth").value
            ),
            min_keyframe_spatial_diversity=float(
                self.get_parameter("prealign_min_keyframe_spatial_diversity").value
            ),
            max_repeated_goal_ratio=float(
                self.get_parameter("prealign_max_repeated_goal_ratio").value
            ),
        )
        self.policy = PrealignmentPolicy(robot_id=self.ns, config=cfg)

        in_goal = self._ns_topic(str(self.get_parameter("input_goal_topic").value))
        out_goal = self._ns_topic(str(self.get_parameter("output_goal_topic").value))
        odom_topic = self._ns_topic(str(self.get_parameter("odom_topic").value))
        map_topic = self._ns_topic(str(self.get_parameter("map_topic").value))
        quality_map_raw = str(self.get_parameter("quality_map_topic").value).strip()
        quality_map_topic = self._ns_topic(quality_map_raw) if quality_map_raw else ""
        replan_topic = self._ns_topic(str(self.get_parameter("frontier_replan_topic").value))
        nav_status_topic = self._ns_topic(str(self.get_parameter("nav_status_topic").value))
        status_topic = self._ns_topic(str(self.get_parameter("status_topic").value))
        self.output_frame_id = str(self.get_parameter("output_frame_id").value).strip() or "map"
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_frontiers = max(1, int(self.get_parameter("max_frontiers").value))
        self.frontier_obstacle_clearance_cells = max(
            1,
            int(self.get_parameter("frontier_obstacle_clearance_cells").value),
        )
        self.prealign_goal_hold_sec = max(0.0, float(self.get_parameter("prealign_goal_hold_sec").value))
        frontier_marker_topic = self._ns_topic(str(self.get_parameter("frontier_marker_topic").value))

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
        self.last_goal_reason = "none"
        self.last_goal_source = "none"
        self.initial_known_cells: int | None = None
        self.keyframe_positions: list[tuple[float, float]] = []

        self.create_subscription(PointStamped, in_goal, self._on_goal, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, 2)
        if quality_map_topic and quality_map_topic != map_topic:
            self.create_subscription(OccupancyGrid, quality_map_topic, self._on_quality_map, 2)
        self.create_subscription(Empty, replan_topic, self._on_replan, 10)
        self.create_subscription(String, nav_status_topic, self._on_nav_status, 10)
        self.create_subscription(String, str(self.get_parameter("alignment_status_topic").value), self._on_alignment_status, 10)
        self.create_subscription(String, str(self.get_parameter("keyframe_topic").value), self._on_keyframe, 10)
        self.create_subscription(String, str(self.get_parameter("candidate_topic").value), self._on_candidate, 10)
        self.create_subscription(String, str(self.get_parameter("robust_inliers_topic").value), self._on_robust, 10)

        self.goal_pub = self.create_publisher(PointStamped, out_goal, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.frontier_pub = self.create_publisher(MarkerArray, frontier_marker_topic, 10)
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "prealignment_exploration_guard active: "
            f"ns={self.ns} {in_goal} -> {out_goal} frame={self.output_frame_id} "
            f"min_goal={cfg.min_goal_distance:.2f}m min_start={cfg.min_start_displacement:.2f}m "
            f"scripted_overlap_demo={cfg.scripted_overlap_demo}"
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
        self._update_map_quality(msg, publish_frontiers=True)

    def _on_quality_map(self, msg: OccupancyGrid) -> None:
        self._update_map_quality(msg, publish_frontiers=False)

    def _update_map_quality(self, msg: OccupancyGrid, *, publish_frontiers: bool) -> None:
        known = sum(1 for v in msg.data if int(v) >= 0)
        self.policy.update_map_coverage_cells(known)
        if self.initial_known_cells is None:
            self.initial_known_cells = known
        frontiers = self._extract_local_frontiers()
        local_map_area = float(known) * float(msg.info.resolution) * float(msg.info.resolution)
        self.policy.update_map_quality(
            local_map_area=local_map_area,
            frontier_count=len(frontiers),
            keyframe_spatial_diversity=self._keyframe_spatial_diversity(),
            stamp_sec=self._now_sec(),
            unknown_to_known_cells=max(0, known - int(self.initial_known_cells or 0)),
        )
        if publish_frontiers:
            self._publish_frontier_markers(frontiers)

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
            status=self._policy_status_for_exploration(status),
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
            pose = payload.get("pose", {})
            if isinstance(pose, dict) and {"x", "y"}.issubset(pose):
                try:
                    self.keyframe_positions.append((float(pose["x"]), float(pose["y"])))
                    if len(self.keyframe_positions) > 200:
                        self.keyframe_positions = self.keyframe_positions[-200:]
                except (TypeError, ValueError):
                    pass

    def _on_candidate(self, msg: String) -> None:
        payload = _loads(msg.data)
        if payload.get("schema") == "team_cross_robot_candidate/v1":
            self.cross_robot_candidates += 1
            self.policy.update_alignment_status(
                status=self._policy_status_for_exploration(self.alignment_status),
                cross_robot_candidates=self.cross_robot_candidates,
                verified_matches=self.verified_matches,
                robust_inliers=self.robust_inliers,
                stamp_sec=self._now_sec(),
            )

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
        self.policy.update_alignment_status(
            status=self._policy_status_for_exploration(str(payload.get("status", self.alignment_status))),
            cross_robot_candidates=self.cross_robot_candidates,
            verified_matches=self.verified_matches,
            robust_inliers=self.robust_inliers,
            stamp_sec=self._now_sec(),
        )

    def _policy_status_for_exploration(self, status: str) -> str:
        clean = str(status or "unaligned").strip().lower()
        if (
            clean == "rejected"
            and self.cross_robot_candidates > 0
            and self.verified_matches > 0
            and 0 < self.robust_inliers < self.policy.config.robust_acceptance_min_inliers
        ):
            return "tentative"
        return clean

    def _tick(self) -> None:
        self._publish_status()
        if self.latest_odom is None:
            return
        now = self._now_sec()
        if now - self.last_publish_sec < 2.0:
            return
        prealign_phase = self.policy.phase.value in {
            "unaligned_local_explore",
            "overlap_seeking",
            "tentative_alignment",
            "rejected_recover",
        }
        if not prealign_phase:
            return
        needs_start_escape = (
            self.policy.metrics.distance_from_start
            < self.policy.config.min_start_displacement
        )
        needs_multiview_overlap = (
            self.policy.phase.value in {"overlap_seeking", "tentative_alignment", "rejected_recover"}
            and self.policy.metrics.robust_inliers < self.policy.config.robust_acceptance_min_inliers
        )
        scripted_demo = self.policy.config.scripted_overlap_demo
        if needs_start_escape or needs_multiview_overlap or scripted_demo:
            incoming = None if needs_multiview_overlap or scripted_demo else self.latest_goal
            self._publish_decision(incoming, force=False)

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
        self.last_goal_reason = decision.reason
        self.last_goal_source = self._decision_source(decision.reason)
        self.get_logger().info(
            f"{self.ns}: prealign_goal reason={decision.reason} "
            f"phase={decision.phase.value} source={self.last_goal_source} "
            f"goal=({decision.goal.x:+.2f},{decision.goal.y:+.2f})"
        )

    @staticmethod
    def _decision_source(reason: str) -> str:
        if reason == "scripted_local_overlap_demo":
            return "scripted_local_overlap_demo"
        if "frontier" in reason:
            return "local_frontier"
        if "primitive" in reason:
            return "local_exploration_primitive"
        if reason == "local_goal_allowed":
            return "local_planner_goal"
        if reason == "peer_frame_goal_blocked_until_alignment":
            return "peer_frame_blocked_local_recovery"
        return "local_policy"

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
                occupied_near = self._occupied_near(data, width, height, gx, gy)
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        n = int(data[(gy + dy) * width + gx + dx])
                        if n < 0:
                            has_unknown = True
                if not has_unknown or occupied_near:
                    continue
                wx = float(msg.info.origin.position.x) + (gx + 0.5) * float(msg.info.resolution)
                wy = float(msg.info.origin.position.y) + (gy + 0.5) * float(msg.info.resolution)
                distance = math.hypot(wx - rx, wy - ry)
                if distance < self.policy.config.min_goal_distance:
                    continue
                corridor_score = self._corridor_score(data, width, height, gx, gy)
                expected_gain = self._expected_coverage_gain(data, width, height, gx, gy)
                out.append(
                    GoalSample(
                        wx,
                        wy,
                        frame_id=f"{self.ns}/map",
                        frontier_size=1.0,
                        corridor_score=corridor_score,
                        keyframe_gain=distance,
                        expected_coverage_gain=expected_gain,
                    )
                )
        return sorted(out, key=lambda g: math.hypot(g.x - rx, g.y - ry), reverse=True)[: self.max_frontiers]

    def _occupied_near(self, data: list[int], width: int, height: int, gx: int, gy: int) -> bool:
        r = self.frontier_obstacle_clearance_cells
        for ny in range(max(0, gy - r), min(height, gy + r + 1)):
            for nx in range(max(0, gx - r), min(width, gx + r + 1)):
                if int(data[ny * width + nx]) >= 50:
                    return True
        return False

    @staticmethod
    def _expected_coverage_gain(data: list[int], width: int, height: int, gx: int, gy: int) -> float:
        unknown = 0
        free = 0
        radius = 5
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                value = int(data[ny * width + nx])
                if value < 0:
                    unknown += 1
                elif value == 0:
                    free += 1
        return float(unknown) + 0.1 * float(free)

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

    def _keyframe_spatial_diversity(self) -> float:
        if len(self.keyframe_positions) < 2:
            return 0.0
        first = self.keyframe_positions[0]
        return max(math.hypot(x - first[0], y - first[1]) for x, y in self.keyframe_positions)

    def _publish_frontier_markers(self, frontiers: list[GoalSample]) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.header.frame_id = f"{self.ns}/map"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker = Marker()
        marker.header.stamp = clear.header.stamp
        marker.header.frame_id = f"{self.ns}/map"
        marker.ns = f"{self.ns}_prealign_frontiers"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9
        for goal in frontiers[: self.max_frontiers]:
            p = Point()
            p.x = float(goal.x)
            p.y = float(goal.y)
            p.z = 0.08
            marker.points.append(p)
        markers.markers.append(marker)
        self.frontier_pub.publish(markers)

    def _publish_status(self) -> None:
        m = self.policy.metrics
        payload = {
            "schema": "prealignment_exploration_status/v1",
            "robot_id": self.ns,
            "alignment_state": self.policy.phase.value,
            "alignment_status": self.alignment_status,
            "prealign_min_start_displacement": float(self.policy.config.min_start_displacement),
            "prealign_scripted_overlap_demo": bool(self.policy.config.scripted_overlap_demo),
            "prealign_robust_acceptance_min_inliers": int(
                self.policy.config.robust_acceptance_min_inliers
            ),
            "distance_from_start": round(float(m.distance_from_start), 4),
            "max_distance_from_start": round(float(m.max_distance_from_start), 4),
            "path_length": round(float(m.path_length), 4),
            "keyframes": int(m.keyframes),
            "map_coverage_cells": int(m.map_coverage_cells),
            "local_map_area": round(float(m.local_map_area), 4),
            "local_map_area_growth": round(float(m.local_map_area_growth), 4),
            "map_area_growth_rate": round(float(m.map_area_growth_rate), 6),
            "unknown_to_known_cells": int(m.unknown_to_known_cells),
            "frontier_count": int(m.frontier_count),
            "new_frontiers_discovered": int(m.new_frontiers_discovered),
            "keyframe_spatial_diversity": round(float(m.keyframe_spatial_diversity), 4),
            "repeated_goal_ratio": round(float(m.repeated_goal_ratio), 4),
            "stuck_recovery_count": int(m.stuck_recovery_count),
            "failed_goal_blacklist_count": int(m.failed_goal_blacklist_count),
            "coverage_gain_per_meter": round(float(m.coverage_gain_per_meter), 6),
            "prealign_exploration_quality": m.prealign_exploration_quality.value,
            "exploration_success": bool(m.exploration_success),
            "local_frontiers_selected": int(m.local_frontiers_selected),
            "goals_rejected_as_too_close": int(m.goals_rejected_as_too_close),
            "stuck_replans": int(m.stuck_replans),
            "blacklisted_goals": int(m.blacklisted_goals),
            "goal_failure_count": int(m.goal_failure_count),
            "peer_frame_goals_blocked": int(m.peer_frame_goals_blocked),
            "cross_robot_candidates": int(self.cross_robot_candidates),
            "verified_matches": int(self.verified_matches),
            "robust_inliers": int(self.robust_inliers),
            "robust_inlier_growth_rate": round(float(m.robust_inlier_growth_rate), 6),
            "tentative_alignment_duration": round(float(m.tentative_alignment_duration), 4),
            "overlap_seeking_active": bool(
                self.policy.phase.value in {"overlap_seeking", "rejected_recover"}
            ),
            "tentative_alignment_exploration_active": bool(
                self.policy.phase.value == "tentative_alignment"
                and m.robust_inliers < self.policy.config.robust_acceptance_min_inliers
            ),
            "overlap_seeking_goals": int(m.overlap_seeking_goals),
            "tentative_alignment_explore_goals": int(m.tentative_alignment_explore_goals),
            "scripted_local_overlap_goals": int(m.scripted_local_overlap_goals),
            "last_goal_reason": self.last_goal_reason,
            "source": self.last_goal_source,
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
