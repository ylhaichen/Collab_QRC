from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable


class AlignmentPhase(str, Enum):
    UNALIGNED_LOCAL_EXPLORE = "unaligned_local_explore"
    OVERLAP_SEEKING = "overlap_seeking"
    TENTATIVE_ALIGNMENT = "tentative_alignment"
    ALIGNED_SHARED_EXPLORE = "aligned_shared_explore"
    REJECTED_RECOVER = "rejected_recover"


@dataclass(frozen=True)
class PrealignmentConfig:
    min_goal_distance: float = 2.0
    min_start_displacement: float = 3.0
    dwell_timeout_sec: float = 20.0
    stuck_replan_limit: int = 3
    goal_blacklist_radius: float = 1.0
    goal_blacklist_ttl_sec: float = 120.0
    overlap_timeout_sec: float = 60.0
    min_keyframes_before_alignment: int = 5
    exploration_radius_growth: float = 1.5
    far_frontier_bonus: float = 1.0
    corridor_frontier_bonus: float = 0.5
    keyframe_gain_bonus: float = 0.5
    last_goal_memory: int = 8
    primitive_step_m: float = 3.5
    recent_goal_radius: float = 0.8
    robust_acceptance_min_inliers: int = 7
    scripted_overlap_demo: bool = False


@dataclass(frozen=True)
class GoalSample:
    x: float
    y: float
    frame_id: str = ""
    frontier_size: float = 0.0
    corridor_score: float = 0.0
    keyframe_gain: float = 0.0


@dataclass
class PrealignmentMetrics:
    distance_from_start: float = 0.0
    max_distance_from_start: float = 0.0
    path_length: float = 0.0
    keyframes: int = 0
    map_coverage_cells: int = 0
    local_frontiers_selected: int = 0
    goals_rejected_as_too_close: int = 0
    stuck_replans: int = 0
    blacklisted_goals: int = 0
    goal_failure_count: int = 0
    peer_frame_goals_blocked: int = 0
    cross_robot_candidates: int = 0
    verified_matches: int = 0
    robust_inliers: int = 0
    robust_inlier_growth_rate: float = 0.0
    tentative_alignment_duration: float = 0.0
    overlap_seeking_goals: int = 0
    tentative_alignment_explore_goals: int = 0
    scripted_local_overlap_goals: int = 0


@dataclass(frozen=True)
class GoalDecision:
    goal: GoalSample | None
    phase: AlignmentPhase
    reason: str
    incoming_allowed: bool


@dataclass
class _BlacklistedGoal:
    goal: GoalSample
    until_sec: float


def frame_robot(frame_id: str) -> str:
    clean = str(frame_id or "").strip().strip("/")
    if not clean:
        return ""
    return clean.split("/", 1)[0]


class PrealignmentPolicy:
    """Local-only exploration policy that prevents pre-alignment start dwell."""

    def __init__(self, *, robot_id: str, config: PrealignmentConfig | None = None) -> None:
        self.robot_id = str(robot_id).strip().strip("/") or "robot"
        self.config = config or PrealignmentConfig()
        self.phase = AlignmentPhase.UNALIGNED_LOCAL_EXPLORE
        self.metrics = PrealignmentMetrics()
        self.start_xy: tuple[float, float] | None = None
        self.last_xy: tuple[float, float] | None = None
        self.first_pose_stamp_sec: float | None = None
        self.last_pose_stamp_sec: float | None = None
        self.last_goals: deque[GoalSample] = deque(maxlen=max(1, self.config.last_goal_memory))
        self.blacklist: list[_BlacklistedGoal] = []
        self._primitive_index = 0
        self._scripted_index = 0
        self._last_alignment_status = "unknown"
        self._phase_entered_sec: float | None = None
        self._tentative_started_sec: float | None = None
        self._robust_history: deque[tuple[float, int]] = deque(maxlen=12)

    @property
    def local_frame_id(self) -> str:
        return f"{self.robot_id}/map"

    def update_pose(self, x: float, y: float, *, stamp_sec: float) -> None:
        xy = (float(x), float(y))
        if self.start_xy is None:
            self.start_xy = xy
            self.first_pose_stamp_sec = float(stamp_sec)
        if self.last_xy is not None:
            step = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            if math.isfinite(step):
                self.metrics.path_length += step
        self.last_xy = xy
        self.last_pose_stamp_sec = float(stamp_sec)
        if self.start_xy is not None:
            self.metrics.distance_from_start = math.hypot(
                xy[0] - self.start_xy[0],
                xy[1] - self.start_xy[1],
            )
            self.metrics.max_distance_from_start = max(
                self.metrics.max_distance_from_start,
                self.metrics.distance_from_start,
            )

    def update_keyframe_count(self, count: int) -> None:
        self.metrics.keyframes = max(self.metrics.keyframes, int(count))

    def update_map_coverage_cells(self, known_cells: int) -> None:
        self.metrics.map_coverage_cells = max(self.metrics.map_coverage_cells, int(known_cells))

    def update_alignment_status(
        self,
        *,
        status: str,
        cross_robot_candidates: int,
        verified_matches: int,
        robust_inliers: int,
        stamp_sec: float,
    ) -> None:
        stamp = float(stamp_sec)
        status_clean = str(status or "").strip().lower()
        self._last_alignment_status = status_clean
        candidates = int(cross_robot_candidates)
        verified = int(verified_matches)
        inliers = int(robust_inliers)
        self.metrics.cross_robot_candidates = max(self.metrics.cross_robot_candidates, candidates)
        self.metrics.verified_matches = max(self.metrics.verified_matches, verified)
        self.metrics.robust_inliers = max(self.metrics.robust_inliers, inliers)
        self._update_robust_growth(stamp, self.metrics.robust_inliers)
        if status_clean == "aligned":
            self._set_phase(AlignmentPhase.ALIGNED_SHARED_EXPLORE, stamp)
        elif status_clean == "rejected":
            self._set_phase(AlignmentPhase.REJECTED_RECOVER, stamp)
        elif status_clean == "tentative" or candidates > 0 or verified > 0 or inliers > 0:
            self._set_phase(AlignmentPhase.TENTATIVE_ALIGNMENT, stamp)
        elif self.phase not in {
            AlignmentPhase.OVERLAP_SEEKING,
            AlignmentPhase.REJECTED_RECOVER,
        }:
            self._set_phase(AlignmentPhase.UNALIGNED_LOCAL_EXPLORE, stamp)
        if self._tentative_started_sec is not None:
            self.metrics.tentative_alignment_duration = max(
                self.metrics.tentative_alignment_duration,
                max(0.0, stamp - self._tentative_started_sec),
            )

    def mark_goal_failed(self, goal: GoalSample, *, stamp_sec: float) -> None:
        self.metrics.goal_failure_count += 1
        self._add_blacklist(goal, stamp_sec)
        if self.metrics.goal_failure_count >= self.config.stuck_replan_limit:
            self.metrics.stuck_replans += 1
            if self.phase != AlignmentPhase.ALIGNED_SHARED_EXPLORE:
                self._set_phase(AlignmentPhase.OVERLAP_SEEKING, stamp_sec)

    def is_blacklisted(self, goal: GoalSample, *, stamp_sec: float) -> bool:
        self._prune_blacklist(stamp_sec)
        radius = max(0.0, self.config.goal_blacklist_radius)
        return any(self._dist(goal, item.goal) <= radius for item in self.blacklist)

    def choose_goal(
        self,
        *,
        incoming: GoalSample | None,
        local_frontiers: Iterable[GoalSample],
        stamp_sec: float,
    ) -> GoalDecision:
        self._refresh_phase_for_progress(stamp_sec)
        self._prune_blacklist(stamp_sec)

        if self.phase == AlignmentPhase.ALIGNED_SHARED_EXPLORE:
            if incoming is None:
                return GoalDecision(None, self.phase, "aligned_no_goal", True)
            self._remember_goal(incoming)
            return GoalDecision(incoming, self.phase, "aligned_passthrough", True)

        if incoming is not None and self._is_peer_frame(incoming):
            self.metrics.peer_frame_goals_blocked += 1
            fallback, _reason = self._choose_local_overlap_goal(local_frontiers, stamp_sec)
            self._remember_goal(fallback)
            return GoalDecision(
                fallback,
                self.phase,
                "peer_frame_goal_blocked_until_alignment",
                False,
            )

        incoming_allowed = (
            incoming is not None and self._goal_allowed(incoming, stamp_sec, count_rejection=True)
        )
        if incoming_allowed and self._should_prefer_overlap_goal_over_incoming():
            fallback, reason = self._choose_local_overlap_goal(local_frontiers, stamp_sec)
            self._remember_goal(fallback)
            return GoalDecision(fallback, self.phase, reason, False)

        if incoming is not None and incoming_allowed:
            self._remember_goal(incoming)
            return GoalDecision(incoming, self.phase, "local_goal_allowed", True)

        if incoming is None and self._should_prefer_overlap_goal_over_incoming():
            fallback, reason = self._choose_local_overlap_goal(local_frontiers, stamp_sec)
            self._remember_goal(fallback)
            return GoalDecision(fallback, self.phase, reason, True)

        fallback = self._select_far_local_goal(local_frontiers, stamp_sec)
        if fallback is not None:
            self.metrics.local_frontiers_selected += 1
            self._remember_goal(fallback)
            return GoalDecision(
                fallback,
                self.phase,
                "selected_far_local_frontier",
                incoming is None,
            )

        primitive = self._next_exploration_primitive()
        self._remember_goal(primitive)
        return GoalDecision(
            primitive,
            self.phase,
            "fallback_exploration_primitive",
            incoming is None,
        )

    def _refresh_phase_for_progress(self, stamp_sec: float) -> None:
        if self.phase in {
            AlignmentPhase.ALIGNED_SHARED_EXPLORE,
            AlignmentPhase.TENTATIVE_ALIGNMENT,
        }:
            return
        if self.phase == AlignmentPhase.REJECTED_RECOVER:
            self._set_phase(AlignmentPhase.OVERLAP_SEEKING, stamp_sec)
            return
        if self.first_pose_stamp_sec is None:
            return
        elapsed = float(stamp_sec) - self.first_pose_stamp_sec
        dwell = (
            elapsed >= self.config.dwell_timeout_sec
            and self.metrics.distance_from_start < self.config.min_start_displacement
        )
        timed_out = elapsed >= self.config.overlap_timeout_sec
        enough_keyframes = self.metrics.keyframes >= self.config.min_keyframes_before_alignment
        if dwell or timed_out or enough_keyframes:
            self._set_phase(AlignmentPhase.OVERLAP_SEEKING, stamp_sec)

    def _set_phase(self, phase: AlignmentPhase, stamp_sec: float) -> None:
        if self.phase != phase:
            self.phase = phase
            self._phase_entered_sec = float(stamp_sec)
        if phase == AlignmentPhase.TENTATIVE_ALIGNMENT and self._tentative_started_sec is None:
            self._tentative_started_sec = float(stamp_sec)

    def _update_robust_growth(self, stamp_sec: float, robust_inliers: int) -> None:
        if not math.isfinite(float(stamp_sec)):
            return
        self._robust_history.append((float(stamp_sec), int(robust_inliers)))
        if len(self._robust_history) < 2:
            return
        first_t, first_v = self._robust_history[0]
        last_t, last_v = self._robust_history[-1]
        dt = last_t - first_t
        if dt > 1e-6:
            self.metrics.robust_inlier_growth_rate = max(
                0.0,
                float(last_v - first_v) / dt,
            )

    def _should_prefer_overlap_goal_over_incoming(self) -> bool:
        if self.phase == AlignmentPhase.ALIGNED_SHARED_EXPLORE:
            return False
        if self.config.scripted_overlap_demo:
            return True
        if self.phase not in {
            AlignmentPhase.OVERLAP_SEEKING,
            AlignmentPhase.TENTATIVE_ALIGNMENT,
            AlignmentPhase.REJECTED_RECOVER,
        }:
            return False
        return self.metrics.robust_inliers < int(self.config.robust_acceptance_min_inliers)

    def _choose_local_overlap_goal(
        self,
        local_frontiers: Iterable[GoalSample],
        stamp_sec: float,
    ) -> tuple[GoalSample, str]:
        frontiers = list(local_frontiers)
        scripted_frontier = self._select_scripted_local_frontier(frontiers, stamp_sec)
        if scripted_frontier is not None:
            self.metrics.local_frontiers_selected += 1
            self.metrics.scripted_local_overlap_goals += 1
            if self.phase == AlignmentPhase.TENTATIVE_ALIGNMENT:
                self.metrics.tentative_alignment_explore_goals += 1
            else:
                self.metrics.overlap_seeking_goals += 1
            return scripted_frontier, "scripted_local_overlap_demo"

        scripted = self._next_scripted_overlap_goal(stamp_sec)
        if scripted is not None:
            self.metrics.scripted_local_overlap_goals += 1
            return scripted, "scripted_local_overlap_demo"

        fallback = self._select_far_local_goal(frontiers, stamp_sec)
        if fallback is not None:
            self.metrics.local_frontiers_selected += 1
            if self.phase == AlignmentPhase.TENTATIVE_ALIGNMENT:
                self.metrics.tentative_alignment_explore_goals += 1
                return fallback, "tentative_alignment_explore"
            self.metrics.overlap_seeking_goals += 1
            return fallback, "overlap_seeking_far_frontier"

        primitive = self._next_exploration_primitive()
        if self.phase == AlignmentPhase.TENTATIVE_ALIGNMENT:
            self.metrics.tentative_alignment_explore_goals += 1
            return primitive, "tentative_alignment_explore"
        self.metrics.overlap_seeking_goals += 1
        return primitive, "fallback_exploration_primitive"

    def _select_scripted_local_frontier(
        self,
        local_frontiers: Iterable[GoalSample],
        stamp_sec: float,
    ) -> GoalSample | None:
        if not self.config.scripted_overlap_demo:
            return None
        candidates = [
            g for g in local_frontiers
            if not self._is_peer_frame(g)
            and self._goal_allowed(g, stamp_sec, count_rejection=False)
            and not self._recently_used(g)
        ]
        if not candidates:
            return None
        rx, ry = self.last_xy or self.start_xy or (0.0, 0.0)
        sx, sy = self.start_xy or (rx, ry)
        waypoints = self._scripted_local_overlap_waypoints()
        dx, dy = waypoints[self._scripted_index % len(waypoints)]
        preferred_angle = math.atan2((sy + dy) - ry, (sx + dx) - rx)

        def score(goal: GoalSample) -> float:
            distance = math.hypot(goal.x - rx, goal.y - ry)
            start_distance = math.hypot(goal.x - sx, goal.y - sy)
            angle = math.atan2(goal.y - ry, goal.x - rx)
            angle_bonus = math.cos(angle - preferred_angle)
            return (
                distance * (1.0 + self.config.far_frontier_bonus)
                + start_distance * self.config.far_frontier_bonus
                + float(goal.corridor_score) * self.config.corridor_frontier_bonus
                + float(goal.keyframe_gain) * self.config.keyframe_gain_bonus
                + angle_bonus * max(0.5, self.config.far_frontier_bonus)
            )

        self._scripted_index += 1
        return max(candidates, key=score)

    def _select_far_local_goal(
        self,
        local_frontiers: Iterable[GoalSample],
        stamp_sec: float,
    ) -> GoalSample | None:
        candidates = [
            g for g in local_frontiers
            if not self._is_peer_frame(g)
            and self._goal_allowed(g, stamp_sec, count_rejection=False)
            and not self._recently_used(g)
        ]
        if not candidates:
            return None
        rx, ry = self.last_xy or self.start_xy or (0.0, 0.0)
        sx, sy = self.start_xy or (rx, ry)

        def score(goal: GoalSample) -> float:
            distance = math.hypot(goal.x - rx, goal.y - ry)
            start_distance = math.hypot(goal.x - sx, goal.y - sy)
            return (
                distance * (1.0 + self.config.far_frontier_bonus)
                + start_distance * self.config.far_frontier_bonus
                + float(goal.corridor_score) * self.config.corridor_frontier_bonus
                + float(goal.keyframe_gain) * self.config.keyframe_gain_bonus
                + float(goal.frontier_size) * 0.01
            )

        return max(candidates, key=score)

    def _goal_allowed(self, goal: GoalSample, stamp_sec: float, *, count_rejection: bool) -> bool:
        if not self._finite_goal(goal):
            return False
        if self.is_blacklisted(goal, stamp_sec=stamp_sec):
            return False
        rx, ry = self.last_xy or self.start_xy or (0.0, 0.0)
        current_min = self.config.min_goal_distance
        if self.phase in {
            AlignmentPhase.OVERLAP_SEEKING,
            AlignmentPhase.TENTATIVE_ALIGNMENT,
            AlignmentPhase.REJECTED_RECOVER,
        }:
            current_min *= max(1.0, self.config.exploration_radius_growth)
        too_close_current = math.hypot(goal.x - rx, goal.y - ry) < current_min
        too_close_start = False
        if self.start_xy is not None:
            too_close_start = (
                math.hypot(goal.x - self.start_xy[0], goal.y - self.start_xy[1])
                < self.config.min_start_displacement
            )
        if too_close_current or too_close_start:
            if count_rejection:
                self.metrics.goals_rejected_as_too_close += 1
                self._add_blacklist(goal, stamp_sec)
            return False
        return True

    def _next_exploration_primitive(self) -> GoalSample:
        rx, ry = self.last_xy or self.start_xy or (0.0, 0.0)
        sx, sy = self.start_xy or (rx, ry)
        radius = max(
            self.config.primitive_step_m,
            self.config.min_start_displacement,
            self.config.min_goal_distance * max(1.0, self.config.exploration_radius_growth),
        )
        golden = math.pi * (3.0 - math.sqrt(5.0))
        angle = self._primitive_index * golden
        start_dx = rx - sx
        start_dy = ry - sy
        start_distance = math.hypot(start_dx, start_dy)
        if 1e-3 < start_distance < self.config.min_start_displacement:
            angle = math.atan2(start_dy, start_dx)
        self._primitive_index += 1
        return GoalSample(
            rx + radius * math.cos(angle),
            ry + radius * math.sin(angle),
            frame_id=self.local_frame_id,
        )

    def _next_scripted_overlap_goal(self, stamp_sec: float) -> GoalSample | None:
        if not self.config.scripted_overlap_demo:
            return None
        waypoints = self._scripted_local_overlap_waypoints()
        sx, sy = self.start_xy or (0.0, 0.0)
        for _ in range(len(waypoints)):
            dx, dy = waypoints[self._scripted_index % len(waypoints)]
            self._scripted_index += 1
            goal = GoalSample(
                sx + dx,
                sy + dy,
                frame_id=self.local_frame_id,
                frontier_size=1.0,
                corridor_score=2.0,
                keyframe_gain=math.hypot(dx, dy),
            )
            if self._goal_allowed(goal, stamp_sec, count_rejection=False) and not self._recently_used(goal):
                return goal
        return None

    def _scripted_local_overlap_waypoints(self) -> tuple[tuple[float, float], ...]:
        robot = self.robot_id.lower()
        if robot.endswith("b") or robot == "robot_b":
            return (
                (4.5, -1.2),
                (5.5, 1.4),
                (3.8, 3.2),
                (1.5, 4.8),
                (-2.5, 3.8),
                (5.8, -0.4),
                (4.8, 4.2),
            )
        return (
            (4.5, 1.2),
            (5.5, -1.2),
            (3.8, 3.2),
            (1.5, 4.8),
            (-2.5, 3.8),
            (5.8, 0.4),
            (4.8, -1.8),
        )

    def _is_peer_frame(self, goal: GoalSample) -> bool:
        robot = frame_robot(goal.frame_id)
        return bool(robot and robot not in {self.robot_id, "map", "odom", "team_map", "world"})

    def _recently_used(self, goal: GoalSample) -> bool:
        radius = max(0.0, self.config.recent_goal_radius)
        return any(self._dist(goal, prev) < radius for prev in self.last_goals)

    def _remember_goal(self, goal: GoalSample) -> None:
        self.last_goals.append(goal)

    def _add_blacklist(self, goal: GoalSample, stamp_sec: float) -> None:
        self.blacklist.append(
            _BlacklistedGoal(goal=goal, until_sec=float(stamp_sec) + self.config.goal_blacklist_ttl_sec)
        )
        self.metrics.blacklisted_goals = len(self.blacklist)

    def _prune_blacklist(self, stamp_sec: float) -> None:
        self.blacklist = [item for item in self.blacklist if item.until_sec >= float(stamp_sec)]
        self.metrics.blacklisted_goals = len(self.blacklist)

    @staticmethod
    def _dist(a: GoalSample, b: GoalSample) -> float:
        return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))

    @staticmethod
    def _finite_goal(goal: GoalSample) -> bool:
        return math.isfinite(float(goal.x)) and math.isfinite(float(goal.y))
