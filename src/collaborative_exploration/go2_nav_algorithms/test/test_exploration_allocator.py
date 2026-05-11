from go2_nav_algorithms.exploration_allocator import (
    GoalCandidate,
    GoalType,
    TeamAlignmentState,
    score_goal_candidate,
)


def test_allocator_goal_types_cover_loop_cleanup_rendezvous_and_comm_recovery() -> None:
    assert {goal.value for goal in GoalType} >= {
        "frontier_goal",
        "loop_closure_goal",
        "rendezvous_goal",
        "dynamic_cleanup_goal",
        "communication_recovery_goal",
    }


def test_unaligned_maps_reject_peer_frame_goals() -> None:
    candidate = GoalCandidate(
        goal_type=GoalType.RENDEZVOUS,
        frame_id="robot_b/map",
        coverage_gain=1.0,
        loop_closure_gain=3.0,
        map_cleanup_gain=0.0,
        communication_value=2.0,
        travel_cost=0.2,
        mobility_risk=0.1,
        duplicate_assignment_penalty=0.0,
    )
    state = TeamAlignmentState(aligned=False, local_robot_id="robot_a")

    score = score_goal_candidate(candidate, state)

    assert score.allowed is False
    assert score.reason == "peer_frame_goal_blocked_until_robust_alignment"


def test_dynamic_contamination_prioritizes_cleanup_when_alignment_is_local() -> None:
    candidate = GoalCandidate(
        goal_type=GoalType.DYNAMIC_CLEANUP,
        frame_id="robot_a/map",
        coverage_gain=0.5,
        loop_closure_gain=0.0,
        map_cleanup_gain=4.0,
        communication_value=0.0,
        travel_cost=0.8,
        mobility_risk=0.2,
        duplicate_assignment_penalty=0.0,
    )
    state = TeamAlignmentState(aligned=False, local_robot_id="robot_a", dynamic_contamination_high=True)

    score = score_goal_candidate(candidate, state)

    assert score.allowed is True
    assert score.utility > 3.0
    assert score.reason == "utility_scored"
