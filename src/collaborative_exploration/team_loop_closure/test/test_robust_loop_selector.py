import math

import numpy as np

from team_loop_closure.common import se2_from_xyyaw
from team_loop_closure.robust_loop_selector import (
    RobustSelectorParams,
    VerifiedMatch,
    pairwise_consistent,
    select_robust_inliers,
)


def _match(idx: int, x: float, y: float, yaw: float, *, rmse: float = 0.18) -> VerifiedMatch:
    return VerifiedMatch(
        match_id=f"m{idx}",
        payload={"match_id": f"m{idx}"},
        transform=se2_from_xyyaw(x, y, yaw),
        rmse=rmse,
        fitness=rmse,
        inlier_ratio=0.72,
        num_correspondences=120,
        descriptor_score=0.22,
    )


def test_pairwise_consistency_uses_relative_transform_delta() -> None:
    params = RobustSelectorParams(
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
    )
    a = _match(0, 2.0, -1.0, math.radians(5.0))
    b = _match(1, 2.35, -0.82, math.radians(9.0))
    c = _match(2, 5.5, 3.0, math.radians(50.0))

    assert pairwise_consistent(a, b, params)
    assert not pairwise_consistent(a, c, params)


def test_select_robust_inliers_accepts_largest_pairwise_consistent_set() -> None:
    params = RobustSelectorParams(
        robust_min_inliers=4,
        robust_min_inlier_ratio=0.5,
        robust_max_median_rmse=0.45,
        robust_max_translation_spread_m=1.0,
        robust_max_yaw_spread_rad=math.radians(10.0),
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
    )
    matches = [
        _match(0, 1.0, 2.0, 0.01),
        _match(1, 1.2, 2.1, 0.03),
        _match(2, 0.8, 1.9, -0.02),
        _match(3, 1.1, 2.2, 0.04),
        _match(4, 7.0, -2.0, 1.2),
        _match(5, -6.0, 4.0, -1.1),
    ]

    result = select_robust_inliers(matches, params)

    assert result.accepted
    assert result.inlier_count == 4
    assert result.rejected_count == 2
    assert result.reason == "robust_inlier_set_accepted"
    assert np.allclose(result.transform[:2, 2], [1.025, 2.05], atol=0.2)


def test_select_robust_inliers_rejects_alias_when_ratio_or_spread_fails() -> None:
    params = RobustSelectorParams(
        robust_min_inliers=7,
        robust_min_inlier_ratio=0.35,
        robust_max_median_rmse=0.45,
        robust_max_translation_spread_m=1.0,
        robust_max_yaw_spread_rad=math.radians(10.0),
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
    )
    matches = [_match(i, 3.0 + 0.05 * i, -1.0, 0.01 * i) for i in range(6)]

    result = select_robust_inliers(matches, params)

    assert not result.accepted
    assert result.inlier_count == 6
    assert result.reason == "insufficient_robust_inliers"


def test_select_robust_inliers_uses_eligible_component_ratio_not_raw_verified_ratio() -> None:
    params = RobustSelectorParams(
        robust_min_inliers=7,
        robust_min_inlier_ratio=0.25,
        robust_max_median_rmse=0.45,
        robust_max_translation_spread_m=1.0,
        robust_max_yaw_spread_rad=math.radians(10.0),
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
        robust_prefilter_max_rmse=0.45,
        robust_prefilter_min_inlier_ratio=0.35,
    )
    true_cluster = [_match(i, 1.0 + 0.04 * i, 2.0, 0.01 * i) for i in range(7)]
    aliases = [
        _match(100 + i, 10.0 + 2.0 * i, -5.0 - i, 1.0 + 0.1 * i, rmse=0.2)
        for i in range(40)
    ]

    result = select_robust_inliers(true_cluster + aliases, params)

    assert result.accepted
    assert result.raw_verified_count == 47
    assert result.eligible_count == 7
    assert result.inlier_count == 7
    assert result.inlier_ratio_raw < params.robust_min_inlier_ratio
    assert result.inlier_ratio_eligible == 1.0


def test_select_robust_inliers_accepts_component_that_is_not_full_clique() -> None:
    params = RobustSelectorParams(
        robust_min_inliers=7,
        robust_min_inlier_ratio=0.25,
        robust_max_median_rmse=0.45,
        robust_max_translation_spread_m=1.0,
        robust_max_yaw_spread_rad=math.radians(10.0),
        translation_threshold_m=1.0,
        yaw_threshold_rad=math.radians(10.0),
    )
    # This component is connected by pairwise-consistent edges, but endpoints
    # are not all mutually connected. The final spread gate still keeps it
    # compact enough to accept as one robust transform hypothesis.
    component = [
        _match(i, 1.0 + 0.15 * i, 2.0, math.radians(i * 1.2))
        for i in range(7)
    ]
    aliases = [
        _match(100 + i, -8.0 - i, 4.0 + i, -1.5)
        for i in range(3)
    ]

    result = select_robust_inliers(component + aliases, params)

    assert result.accepted
    assert result.inlier_count == 7
    assert result.reason == "robust_inlier_set_accepted"
    assert result.translation_spread_m <= params.robust_max_translation_spread_m
