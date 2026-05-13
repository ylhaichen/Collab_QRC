import math

from go2_nav_algorithms.prealignment_policy import (
    AlignmentPhase,
    GoalSample,
    PrealignmentConfig,
    PrealignmentPolicy,
)


def test_unaligned_policy_rejects_goals_inside_start_bubble_and_chooses_far_frontier() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(
            min_goal_distance=2.0,
            min_start_displacement=3.0,
            dwell_timeout_sec=20.0,
            goal_blacklist_radius=1.0,
        ),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(0.1, 0.0, stamp_sec=21.0)

    decision = policy.choose_goal(
        incoming=GoalSample(0.5, 0.0, frame_id="robot_a/map"),
        local_frontiers=[
            GoalSample(0.7, 0.1, frame_id="robot_a/map"),
            GoalSample(4.0, 0.0, frame_id="robot_a/map"),
        ],
        stamp_sec=21.0,
    )

    assert decision.goal is not None
    assert decision.goal.x == 4.0
    assert decision.goal.y == 0.0
    assert decision.phase == AlignmentPhase.OVERLAP_SEEKING
    assert decision.reason == "selected_far_local_frontier"
    assert policy.metrics.goals_rejected_as_too_close == 1
    assert policy.metrics.local_frontiers_selected == 1


def test_unaligned_policy_never_routes_peer_frame_goal_before_alignment() -> None:
    policy = PrealignmentPolicy(robot_id="robot_a")
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)

    decision = policy.choose_goal(
        incoming=GoalSample(5.0, 0.0, frame_id="robot_b/map"),
        local_frontiers=[],
        stamp_sec=5.0,
    )

    assert decision.goal is not None
    assert decision.goal.frame_id == "robot_a/map"
    assert decision.reason == "peer_frame_goal_blocked_until_alignment"
    assert policy.metrics.peer_frame_goals_blocked == 1


def test_state_machine_enters_tentative_aligned_and_rejected_recover() -> None:
    policy = PrealignmentPolicy(robot_id="robot_a")

    policy.update_alignment_status(
        status="unaligned",
        cross_robot_candidates=0,
        verified_matches=0,
        robust_inliers=0,
        stamp_sec=0.0,
    )
    assert policy.phase == AlignmentPhase.UNALIGNED_LOCAL_EXPLORE

    policy.update_alignment_status(
        status="tentative",
        cross_robot_candidates=3,
        verified_matches=1,
        robust_inliers=0,
        stamp_sec=10.0,
    )
    assert policy.phase == AlignmentPhase.TENTATIVE_ALIGNMENT

    policy.update_alignment_status(
        status="aligned",
        cross_robot_candidates=8,
        verified_matches=7,
        robust_inliers=7,
        stamp_sec=20.0,
    )
    assert policy.phase == AlignmentPhase.ALIGNED_SHARED_EXPLORE

    policy.update_alignment_status(
        status="rejected",
        cross_robot_candidates=8,
        verified_matches=7,
        robust_inliers=0,
        stamp_sec=30.0,
    )
    assert policy.phase == AlignmentPhase.REJECTED_RECOVER


def test_policy_tracks_path_length_keyframes_and_blacklists_failed_goal() -> None:
    policy = PrealignmentPolicy(robot_id="robot_a")
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(1.0, 0.0, stamp_sec=1.0)
    policy.update_pose(1.0, 2.0, stamp_sec=2.0)
    policy.update_keyframe_count(6)

    policy.mark_goal_failed(GoalSample(3.0, 0.0, frame_id="robot_a/map"), stamp_sec=3.0)

    assert math.isclose(policy.metrics.distance_from_start, math.sqrt(5.0), rel_tol=1e-6)
    assert math.isclose(policy.metrics.max_distance_from_start, math.sqrt(5.0), rel_tol=1e-6)
    assert math.isclose(policy.metrics.path_length, 3.0, rel_tol=1e-6)
    assert policy.metrics.keyframes == 6
    assert policy.metrics.goal_failure_count == 1
    assert policy.metrics.blacklisted_goals == 1
    assert policy.is_blacklisted(GoalSample(3.4, 0.0, frame_id="robot_a/map"), stamp_sec=4.0)


def test_fallback_primitive_moves_outward_when_robot_is_still_inside_start_radius() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_b",
        config=PrealignmentConfig(min_start_displacement=3.0, primitive_step_m=3.5),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(1.0, 0.0, stamp_sec=21.0)

    decision = policy.choose_goal(incoming=None, local_frontiers=[], stamp_sec=21.0)

    assert decision.goal is not None
    assert decision.reason == "fallback_exploration_primitive"
    assert decision.goal.x > 4.0
    assert math.hypot(decision.goal.x, decision.goal.y) >= 4.0
