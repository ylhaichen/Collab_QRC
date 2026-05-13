import math

from go2_nav_algorithms.prealignment_policy import (
    AlignmentPhase,
    GoalSample,
    PrealignExplorationQuality,
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


def test_tentative_alignment_keeps_local_anti_dwell_goal_selection() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_b",
        config=PrealignmentConfig(
            min_goal_distance=2.0,
            min_start_displacement=3.0,
            goal_blacklist_radius=1.0,
        ),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(0.2, 0.0, stamp_sec=5.0)
    policy.update_alignment_status(
        status="tentative",
        cross_robot_candidates=4,
        verified_matches=2,
        robust_inliers=1,
        stamp_sec=5.0,
    )

    decision = policy.choose_goal(
        incoming=GoalSample(0.5, 0.0, frame_id="robot_b/map"),
        local_frontiers=[
            GoalSample(1.0, 0.0, frame_id="robot_b/map"),
            GoalSample(5.0, 0.0, frame_id="robot_b/map"),
        ],
        stamp_sec=5.0,
    )

    assert decision.phase == AlignmentPhase.TENTATIVE_ALIGNMENT
    assert decision.reason == "selected_far_local_frontier"
    assert decision.goal is not None
    assert math.isclose(decision.goal.x, 5.0)


def test_tentative_alignment_prefers_multiview_frontier_over_valid_near_local_goal() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(
            min_goal_distance=1.0,
            min_start_displacement=2.0,
            robust_acceptance_min_inliers=7,
        ),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(3.5, 0.0, stamp_sec=10.0)
    policy.update_alignment_status(
        status="tentative",
        cross_robot_candidates=80,
        verified_matches=20,
        robust_inliers=3,
        stamp_sec=10.0,
    )

    decision = policy.choose_goal(
        incoming=GoalSample(5.0, 0.0, frame_id="robot_a/map", keyframe_gain=0.5),
        local_frontiers=[
            GoalSample(5.0, 0.0, frame_id="robot_a/map", keyframe_gain=0.5),
            GoalSample(8.0, 1.5, frame_id="robot_a/map", corridor_score=2.0, keyframe_gain=4.8),
        ],
        stamp_sec=11.0,
    )

    assert decision.phase == AlignmentPhase.TENTATIVE_ALIGNMENT
    assert decision.reason == "tentative_alignment_explore"
    assert decision.goal is not None
    assert math.isclose(decision.goal.x, 8.0)
    assert policy.metrics.tentative_alignment_explore_goals == 1


def test_scripted_overlap_demo_generates_local_only_robot_specific_multiview_goals() -> None:
    robot_a = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(
            min_goal_distance=1.0,
            min_start_displacement=2.0,
            scripted_overlap_demo=True,
        ),
    )
    robot_b = PrealignmentPolicy(
        robot_id="robot_b",
        config=PrealignmentConfig(
            min_goal_distance=1.0,
            min_start_displacement=2.0,
            scripted_overlap_demo=True,
        ),
    )
    for policy in (robot_a, robot_b):
        policy.update_pose(0.0, 0.0, stamp_sec=0.0)
        policy.update_pose(0.2, 0.0, stamp_sec=5.0)
        policy.update_alignment_status(
            status="tentative",
            cross_robot_candidates=40,
            verified_matches=12,
            robust_inliers=3,
            stamp_sec=5.0,
        )

    a_decision = robot_a.choose_goal(incoming=None, local_frontiers=[], stamp_sec=6.0)
    b_decision = robot_b.choose_goal(incoming=None, local_frontiers=[], stamp_sec=6.0)

    assert a_decision.reason == "scripted_local_overlap_demo"
    assert b_decision.reason == "scripted_local_overlap_demo"
    assert a_decision.goal is not None
    assert b_decision.goal is not None
    assert a_decision.goal.frame_id == "robot_a/map"
    assert b_decision.goal.frame_id == "robot_b/map"
    assert (a_decision.goal.x, a_decision.goal.y) != (b_decision.goal.x, b_decision.goal.y)
    assert robot_a.metrics.scripted_local_overlap_goals == 1
    assert robot_b.metrics.scripted_local_overlap_goals == 1


def test_scripted_overlap_demo_prefers_real_local_frontier_over_fixed_waypoint() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(
            min_goal_distance=1.0,
            min_start_displacement=2.0,
            scripted_overlap_demo=True,
        ),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(0.5, 0.0, stamp_sec=3.0)
    policy.update_alignment_status(
        status="tentative",
        cross_robot_candidates=20,
        verified_matches=8,
        robust_inliers=3,
        stamp_sec=3.0,
    )

    decision = policy.choose_goal(
        incoming=None,
        local_frontiers=[
            GoalSample(2.2, -1.0, frame_id="robot_a/map", keyframe_gain=1.0),
            GoalSample(3.2, 1.0, frame_id="robot_a/map", corridor_score=2.0, keyframe_gain=3.0),
        ],
        stamp_sec=4.0,
    )

    assert decision.reason == "scripted_local_overlap_demo"
    assert decision.goal is not None
    assert decision.goal.x == 3.2
    assert decision.goal.y == 1.0
    assert policy.metrics.local_frontiers_selected == 1
    assert policy.metrics.tentative_alignment_explore_goals == 1
    assert policy.metrics.scripted_local_overlap_goals == 1


def test_policy_tracks_robust_growth_rate_and_tentative_duration() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(robust_acceptance_min_inliers=7),
    )
    policy.update_alignment_status(
        status="tentative",
        cross_robot_candidates=10,
        verified_matches=5,
        robust_inliers=1,
        stamp_sec=10.0,
    )
    policy.update_alignment_status(
        status="tentative",
        cross_robot_candidates=30,
        verified_matches=18,
        robust_inliers=4,
        stamp_sec=25.0,
    )

    assert policy.phase == AlignmentPhase.TENTATIVE_ALIGNMENT
    assert math.isclose(policy.metrics.robust_inlier_growth_rate, 0.2, rel_tol=1e-6)
    assert math.isclose(policy.metrics.tentative_alignment_duration, 15.0, rel_tol=1e-6)


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


def test_policy_does_not_count_motion_as_exploration_without_map_growth() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(
            min_start_displacement=3.0,
            min_path_length=4.0,
            min_local_map_area_growth=1.0,
            max_repeated_goal_ratio=0.5,
        ),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(4.5, 0.0, stamp_sec=8.0)
    policy.update_keyframe_count(8)
    policy.update_map_quality(
        local_map_area=2.0,
        frontier_count=6,
        keyframe_spatial_diversity=3.0,
        stamp_sec=1.0,
    )
    policy.update_map_quality(
        local_map_area=2.0,
        frontier_count=6,
        keyframe_spatial_diversity=3.0,
        stamp_sec=8.0,
    )

    assert policy.metrics.distance_from_start > policy.config.min_start_displacement
    assert policy.metrics.path_length > policy.config.min_path_length
    assert policy.metrics.prealign_exploration_quality == PrealignExplorationQuality.MOVING_ONLY
    assert not policy.metrics.exploration_success

    policy.update_map_quality(
        local_map_area=4.0,
        frontier_count=11,
        keyframe_spatial_diversity=4.2,
        stamp_sec=16.0,
    )

    assert policy.metrics.local_map_area_growth >= 2.0
    assert policy.metrics.map_area_growth_rate > 0.0
    assert policy.metrics.new_frontiers_discovered >= 5
    assert policy.metrics.coverage_gain_per_meter > 0.0
    assert policy.metrics.prealign_exploration_quality == PrealignExplorationQuality.EXPLORING
    assert policy.metrics.exploration_success


def test_policy_tracks_repeated_goal_ratio_and_blocks_exploration_success() -> None:
    policy = PrealignmentPolicy(
        robot_id="robot_a",
        config=PrealignmentConfig(
            min_goal_distance=0.1,
            min_start_displacement=1.0,
            min_path_length=1.0,
            min_local_map_area_growth=0.5,
            max_repeated_goal_ratio=0.25,
            recent_goal_radius=0.8,
        ),
    )
    policy.update_pose(0.0, 0.0, stamp_sec=0.0)
    policy.update_pose(2.0, 0.0, stamp_sec=5.0)
    policy.update_keyframe_count(5)
    policy.update_map_quality(
        local_map_area=1.0,
        frontier_count=4,
        keyframe_spatial_diversity=2.0,
        stamp_sec=0.0,
    )
    policy.update_map_quality(
        local_map_area=2.0,
        frontier_count=8,
        keyframe_spatial_diversity=3.0,
        stamp_sec=5.0,
    )

    for _ in range(4):
        policy._remember_goal(GoalSample(3.0, 0.0, frame_id="robot_a/map"))

    assert policy.metrics.repeated_goal_ratio > policy.config.max_repeated_goal_ratio
    assert policy.metrics.prealign_exploration_quality == PrealignExplorationQuality.LOW_COVERAGE_GROWTH
    assert not policy.metrics.exploration_success
