import math

import numpy as np

from team_loop_closure.common import se2_from_xyyaw
from team_loop_closure.registration_backend import register_keyframe_clouds
from team_loop_closure.relative_transform_manager_node import (
    AlignmentGateInputs,
    evaluate_alignment_gate,
)
from team_loop_closure.robust_loop_selector import (
    RobustSelectorParams,
    VerifiedMatch,
    select_robust_inliers,
)


def _cloud(dx: float = 0.0, dy: float = 0.0) -> np.ndarray:
    pts = []
    for x in range(5):
        for y in range(5):
            pts.append((float(x) * 0.4 + dx, float(y) * 0.4 + dy))
    return np.asarray(pts, dtype=np.float64)


def test_kiss_matcher_mode_falls_back_to_icp_with_explicit_blocker() -> None:
    result = register_keyframe_clouds(
        _cloud(),
        _cloud(dx=0.1, dy=-0.05),
        backend="kiss_matcher",
        initial_yaw=0.0,
        yaw_search_sectors=1,
        sector_count=60,
        max_iterations=8,
        max_corr_dist_m=0.6,
        kiss_matcher_available=False,
    )

    assert result.requested_backend == "kiss_matcher"
    assert result.backend == "icp_2d"
    assert result.dependency_blocker == "kiss_matcher_not_available"
    assert result.geometric_verification_required is True
    assert math.isfinite(result.fitness_m)


def test_registration_rejects_tiny_clouds_without_crashing() -> None:
    result = register_keyframe_clouds(
        np.empty((0, 2), dtype=np.float64),
        _cloud(),
        backend="icp_2d",
        initial_yaw=0.3,
        yaw_search_sectors=1,
        sector_count=60,
        max_iterations=8,
        max_corr_dist_m=0.6,
    )

    assert result.backend == "icp_2d"
    assert math.isinf(result.fitness_m)
    assert result.inlier_ratio == 0.0
    assert result.num_correspondences == 0


def test_robust_selector_reports_pcm_backend_and_rejects_single_weak_match() -> None:
    params = RobustSelectorParams(
        robust_min_inliers=3,
        robust_min_inlier_ratio=0.5,
        robust_max_median_rmse=0.45,
    )
    weak = VerifiedMatch(
        match_id="weak",
        payload={"query_robot": "robot_a", "match_robot": "robot_b"},
        transform=se2_from_xyyaw(2.0, 0.0, 0.1),
        rmse=0.8,
        fitness=0.8,
        inlier_ratio=0.2,
        num_correspondences=7,
        descriptor_score=0.4,
    )

    result = select_robust_inliers([weak], params, selection_backend="pcm")

    assert result.selection_backend == "pcm"
    assert result.accepted is False
    assert result.reason in {"no_eligible_verified_matches", "insufficient_robust_inliers"}


def test_safety_gate_never_opens_from_descriptor_only_or_single_weak_match() -> None:
    descriptor_only = evaluate_alignment_gate(
        AlignmentGateInputs(
            robust_accepted=False,
            robust_status="tentative",
            robust_inlier_set_size=0,
            pose_graph_inter_robot_factors=0,
            relative_transform_finite=False,
            no_overlap_rejection_passed=False,
            gt_used_runtime=False,
        )
    )
    weak = evaluate_alignment_gate(
        AlignmentGateInputs(
            robust_accepted=False,
            robust_status="rejected",
            robust_inlier_set_size=1,
            pose_graph_inter_robot_factors=0,
            relative_transform_finite=True,
            no_overlap_rejection_passed=False,
            gt_used_runtime=False,
        )
    )

    assert descriptor_only.status == "tentative"
    assert descriptor_only.open_merged_map is False
    assert descriptor_only.reason == "robust_loop_selector_not_accepted"
    assert weak.status == "rejected"
    assert weak.open_merged_map is False


def test_safety_gate_opens_only_after_robust_factor_and_no_overlap_gates() -> None:
    decision = evaluate_alignment_gate(
        AlignmentGateInputs(
            robust_accepted=True,
            robust_status="accepted",
            robust_inlier_set_size=7,
            pose_graph_inter_robot_factors=7,
            relative_transform_finite=True,
            no_overlap_rejection_passed=True,
            gt_used_runtime=False,
        )
    )

    assert decision.status == "aligned"
    assert decision.open_merged_map is True
    assert decision.reason == "robust_alignment_pose_graph_and_no_overlap_gate_accepted"
