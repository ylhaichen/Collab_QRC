from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class GoalType(str, Enum):
    FRONTIER = "frontier_goal"
    LOOP_CLOSURE = "loop_closure_goal"
    RENDEZVOUS = "rendezvous_goal"
    DYNAMIC_CLEANUP = "dynamic_cleanup_goal"
    COMMUNICATION_RECOVERY = "communication_recovery_goal"


@dataclass(frozen=True)
class TeamAlignmentState:
    aligned: bool
    local_robot_id: str
    dynamic_contamination_high: bool = False
    communication_weak: bool = False


@dataclass(frozen=True)
class GoalCandidate:
    goal_type: GoalType
    frame_id: str
    coverage_gain: float
    loop_closure_gain: float
    map_cleanup_gain: float
    communication_value: float
    travel_cost: float
    mobility_risk: float
    duplicate_assignment_penalty: float


@dataclass(frozen=True)
class GoalScore:
    allowed: bool
    utility: float
    reason: str


def _frame_robot(frame_id: str) -> str:
    return str(frame_id or "").strip().strip("/").split("/", 1)[0]


def score_goal_candidate(candidate: GoalCandidate, state: TeamAlignmentState) -> GoalScore:
    frame_robot = _frame_robot(candidate.frame_id)
    local_robot = str(state.local_robot_id).strip().strip("/")
    if frame_robot and frame_robot != local_robot and not state.aligned:
        return GoalScore(
            allowed=False,
            utility=float("-inf"),
            reason="peer_frame_goal_blocked_until_robust_alignment",
        )

    cleanup_gain = float(candidate.map_cleanup_gain)
    comm_value = float(candidate.communication_value)
    loop_gain = float(candidate.loop_closure_gain)
    if state.dynamic_contamination_high and candidate.goal_type == GoalType.DYNAMIC_CLEANUP:
        cleanup_gain *= 1.35
    if state.communication_weak and candidate.goal_type in {
        GoalType.RENDEZVOUS,
        GoalType.COMMUNICATION_RECOVERY,
    }:
        comm_value *= 1.35
    if not state.aligned and candidate.goal_type == GoalType.LOOP_CLOSURE:
        loop_gain *= 1.20

    utility = (
        float(candidate.coverage_gain)
        + loop_gain
        + cleanup_gain
        + comm_value
        - float(candidate.travel_cost)
        - float(candidate.mobility_risk)
        - float(candidate.duplicate_assignment_penalty)
    )
    return GoalScore(allowed=True, utility=utility, reason="utility_scored")
