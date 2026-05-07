from __future__ import annotations

import math
from dataclasses import dataclass, field
from statistics import median
from typing import Any

import numpy as np

from .common import circular_mean, invert_se2, wrap_pi, xyyaw_from_se2


@dataclass(frozen=True)
class RobustSelectorParams:
    translation_threshold_m: float = 1.0
    yaw_threshold_rad: float = math.radians(10.0)
    robust_min_inliers: int = 7
    robust_min_inlier_ratio: float = 0.25
    robust_max_median_rmse: float = 0.45
    robust_max_translation_spread_m: float = 1.0
    robust_max_yaw_spread_rad: float = math.radians(12.0)
    max_pairwise_rmse_disagreement: float = 0.35
    min_pairwise_inlier_ratio: float = 0.0
    min_pairwise_correspondences: int = 0
    robust_prefilter_max_rmse: float = 0.45
    robust_prefilter_min_inlier_ratio: float = 0.35
    robust_prefilter_min_correspondences: int = 0
    robust_prefilter_max_descriptor_distance: float = math.inf
    deduplicate_by_query_keyframe: bool = False
    deduplicate_by_match_keyframe: bool = False
    deduplicate_transform_bin_translation_m: float = 0.0
    deduplicate_transform_bin_yaw_rad: float = 0.0


@dataclass
class VerifiedMatch:
    match_id: str
    payload: dict[str, Any]
    transform: np.ndarray
    rmse: float
    fitness: float
    inlier_ratio: float
    num_correspondences: int
    descriptor_score: float


@dataclass
class RobustSelectionResult:
    accepted: bool
    reason: str
    inliers: list[VerifiedMatch] = field(default_factory=list)
    rejected: list[VerifiedMatch] = field(default_factory=list)
    transform: np.ndarray | None = None
    inlier_ratio: float = 0.0
    median_rmse: float = math.inf
    translation_spread_m: float = math.inf
    yaw_spread_rad: float = math.inf
    raw_consistent_matches: int = 0
    raw_verified_count: int = 0
    prefiltered_count: int = 0
    eligible_count: int = 0
    deduplicated_count: int = 0
    inlier_ratio_raw: float = 0.0
    inlier_ratio_eligible: float = 0.0
    median_descriptor_distance: float = math.inf
    median_inlier_ratio: float = 0.0

    @property
    def inlier_count(self) -> int:
        return len(self.inliers)

    @property
    def rejected_count(self) -> int:
        return len(self.rejected)


def pairwise_consistent(a: VerifiedMatch, b: VerifiedMatch, params: RobustSelectorParams) -> bool:
    delta = invert_se2(a.transform) @ b.transform
    dx, dy, dyaw = xyyaw_from_se2(delta)
    if math.hypot(dx, dy) > params.translation_threshold_m:
        return False
    if abs(wrap_pi(dyaw)) > params.yaw_threshold_rad:
        return False
    if abs(float(a.rmse) - float(b.rmse)) > params.max_pairwise_rmse_disagreement:
        return False
    if min(float(a.inlier_ratio), float(b.inlier_ratio)) < params.min_pairwise_inlier_ratio:
        return False
    if min(int(a.num_correspondences), int(b.num_correspondences)) < params.min_pairwise_correspondences:
        return False
    return True


def _greedy_maximum_consistent_set(
    matches: list[VerifiedMatch],
    adjacency: list[set[int]],
) -> list[int]:
    if not matches:
        return []
    best: list[int] = []
    order = sorted(range(len(matches)), key=lambda i: len(adjacency[i]), reverse=True)
    for seed in order:
        clique = [seed]
        candidates = [i for i in order if i != seed and i in adjacency[seed]]
        while candidates:
            nxt = max(candidates, key=lambda i: sum(1 for j in clique if i in adjacency[j]))
            if all(nxt in adjacency[j] for j in clique):
                clique.append(nxt)
            candidates = [i for i in candidates if i != nxt and all(i in adjacency[j] for j in clique)]
        if len(clique) > len(best):
            best = clique
    return sorted(best)


def _safe_float(value: Any, default: float = math.inf) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(out):
        return default
    return out


def _quality_key(match: VerifiedMatch) -> tuple[float, float, float, int]:
    return (
        _safe_float(match.rmse),
        _safe_float(match.descriptor_score),
        -_safe_float(match.inlier_ratio, 0.0),
        -int(match.num_correspondences),
    )


def _keyframe_identity(match: VerifiedMatch, prefix: str) -> tuple[str, str]:
    payload = match.payload
    if prefix == "query":
        robot = payload.get("query_robot", payload.get("source_robot", ""))
        keyframe = payload.get("query_keyframe", payload.get("source_keyframe", ""))
    else:
        robot = payload.get("match_robot", payload.get("target_robot", ""))
        keyframe = payload.get("match_keyframe", payload.get("target_keyframe", ""))
    if not robot or not keyframe:
        return "__match_id__", match.match_id
    return str(robot), str(keyframe)


def _has_descriptor_distance(match: VerifiedMatch) -> bool:
    return "descriptor_score" in match.payload or "descriptor_distance" in match.payload


def _eligible_match(match: VerifiedMatch, params: RobustSelectorParams) -> bool:
    if _safe_float(match.rmse) > params.robust_prefilter_max_rmse:
        return False
    if _safe_float(match.inlier_ratio, 0.0) < params.robust_prefilter_min_inlier_ratio:
        return False
    if int(match.num_correspondences) < params.robust_prefilter_min_correspondences:
        return False
    if (
        math.isfinite(params.robust_prefilter_max_descriptor_distance)
        and _has_descriptor_distance(match)
        and _safe_float(match.descriptor_score) > params.robust_prefilter_max_descriptor_distance
    ):
        return False
    return True


def _best_by_key(matches: list[VerifiedMatch], key_fn) -> list[VerifiedMatch]:
    best: dict[Any, VerifiedMatch] = {}
    for match in matches:
        key = key_fn(match)
        old = best.get(key)
        if old is None or _quality_key(match) < _quality_key(old):
            best[key] = match
    return sorted(best.values(), key=lambda m: m.match_id)


def _deduplicate_transform_bins(
    matches: list[VerifiedMatch],
    translation_bin_m: float,
    yaw_bin_rad: float,
) -> list[VerifiedMatch]:
    if translation_bin_m <= 0.0 and yaw_bin_rad <= 0.0:
        return matches

    def _bin(match: VerifiedMatch) -> tuple[int, int, int]:
        x, y, yaw = xyyaw_from_se2(match.transform)
        tx = math.floor(x / translation_bin_m) if translation_bin_m > 0.0 else 0
        ty = math.floor(y / translation_bin_m) if translation_bin_m > 0.0 else 0
        yw = math.floor(wrap_pi(yaw) / yaw_bin_rad) if yaw_bin_rad > 0.0 else 0
        return int(tx), int(ty), int(yw)

    return _best_by_key(matches, _bin)


def _prepare_match_pool(
    matches: list[VerifiedMatch],
    params: RobustSelectorParams,
) -> tuple[list[VerifiedMatch], list[VerifiedMatch]]:
    eligible = [match for match in matches if _eligible_match(match, params)]
    deduped = eligible
    if params.deduplicate_by_query_keyframe:
        deduped = _best_by_key(deduped, lambda m: _keyframe_identity(m, "query"))
    if params.deduplicate_by_match_keyframe:
        deduped = _best_by_key(deduped, lambda m: _keyframe_identity(m, "match"))
    deduped = _deduplicate_transform_bins(
        deduped,
        params.deduplicate_transform_bin_translation_m,
        params.deduplicate_transform_bin_yaw_rad,
    )
    return eligible, deduped


def _largest_component_indices(adjacency: list[set[int]]) -> list[int]:
    if not adjacency:
        return []
    seen: set[int] = set()
    best: list[int] = []
    for seed in range(len(adjacency)):
        if seed in seen:
            continue
        stack = [seed]
        seen.add(seed)
        component: list[int] = []
        while stack:
            idx = stack.pop()
            component.append(idx)
            for nb in adjacency[idx]:
                if nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        if len(component) > len(best):
            best = component
    return sorted(best)


def _average_transform(matches: list[VerifiedMatch]) -> np.ndarray | None:
    if not matches:
        return None
    xs: list[float] = []
    ys: list[float] = []
    yaws: list[float] = []
    for match in matches:
        x, y, yaw = xyyaw_from_se2(match.transform)
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
    out = np.eye(3, dtype=np.float64)
    cyaw = circular_mean(yaws)
    c = math.cos(cyaw)
    s = math.sin(cyaw)
    out[0, 0] = c
    out[0, 1] = -s
    out[1, 0] = s
    out[1, 1] = c
    out[0, 2] = float(np.mean(xs))
    out[1, 2] = float(np.mean(ys))
    return out


def _spread(matches: list[VerifiedMatch], transform: np.ndarray | None) -> tuple[float, float]:
    if not matches or transform is None:
        return math.inf, math.inf
    cx, cy, cyaw = xyyaw_from_se2(transform)
    trans = []
    yaws = []
    for match in matches:
        x, y, yaw = xyyaw_from_se2(match.transform)
        trans.append(math.hypot(x - cx, y - cy))
        yaws.append(abs(wrap_pi(yaw - cyaw)))
    return max(trans, default=math.inf), max(yaws, default=math.inf)


def _trim_to_spread_gate(
    matches: list[VerifiedMatch],
    params: RobustSelectorParams,
) -> list[VerifiedMatch]:
    trimmed = list(matches)
    while len(trimmed) > params.robust_min_inliers:
        transform = _average_transform(trimmed)
        trans_spread, yaw_spread = _spread(trimmed, transform)
        if (
            trans_spread <= params.robust_max_translation_spread_m
            and yaw_spread <= params.robust_max_yaw_spread_rad
        ):
            break
        if transform is None:
            break
        cx, cy, cyaw = xyyaw_from_se2(transform)

        def _outlier_key(match: VerifiedMatch) -> tuple[float, float, float, float, int]:
            x, y, yaw = xyyaw_from_se2(match.transform)
            trans_norm = math.hypot(x - cx, y - cy) / max(
                1e-6, params.robust_max_translation_spread_m
            )
            yaw_norm = abs(wrap_pi(yaw - cyaw)) / max(1e-6, params.robust_max_yaw_spread_rad)
            rmse, desc, neg_inlier, neg_corr = _quality_key(match)
            return max(trans_norm, yaw_norm), trans_norm, yaw_norm, rmse + desc - neg_inlier, neg_corr

        worst = max(trimmed, key=_outlier_key)
        trimmed.remove(worst)
    return trimmed


def select_robust_inliers(
    matches: list[VerifiedMatch],
    params: RobustSelectorParams,
) -> RobustSelectionResult:
    if not matches:
        return RobustSelectionResult(
            accepted=False,
            reason="no_verified_matches",
            inliers=[],
            rejected=[],
            transform=None,
            raw_consistent_matches=0,
            raw_verified_count=0,
            prefiltered_count=0,
            eligible_count=0,
            deduplicated_count=0,
        )

    raw_verified_count = len(matches)
    eligible, pool = _prepare_match_pool(matches, params)
    if not pool:
        return RobustSelectionResult(
            accepted=False,
            reason="no_eligible_verified_matches",
            inliers=[],
            rejected=list(matches),
            transform=None,
            raw_consistent_matches=0,
            raw_verified_count=raw_verified_count,
            prefiltered_count=len(eligible),
            eligible_count=len(eligible),
            deduplicated_count=0,
        )

    adjacency: list[set[int]] = [set() for _ in pool]
    edge_count = 0
    for i in range(len(pool)):
        for j in range(i + 1, len(pool)):
            if pairwise_consistent(pool[i], pool[j], params):
                adjacency[i].add(j)
                adjacency[j].add(i)
                edge_count += 1

    component_indices = _largest_component_indices(adjacency)
    consensus_pool = [pool[i] for i in component_indices]
    # Use the maximum pairwise-consistent component as the robust candidate set,
    # then trim geometric outliers until the final spread gates can evaluate a
    # compact transform hypothesis. This preserves real revisits where not every
    # match is mutually clique-connected, while rejecting loose alias chains.
    inliers = _trim_to_spread_gate(consensus_pool, params)
    inlier_ids = {m.match_id for m in inliers}
    rejected = [m for m in matches if m.match_id not in inlier_ids]
    transform = _average_transform(inliers)
    trans_spread, yaw_spread = _spread(inliers, transform)
    inlier_ratio_raw = float(len(inliers)) / float(max(1, raw_verified_count))
    inlier_ratio_eligible = float(len(inliers)) / float(max(1, len(consensus_pool)))
    med_rmse = float(median([m.rmse for m in inliers])) if inliers else math.inf
    med_desc = (
        float(median([m.descriptor_score for m in inliers if _has_descriptor_distance(m)]))
        if any(_has_descriptor_distance(m) for m in inliers)
        else math.inf
    )
    med_inlier_ratio = float(median([m.inlier_ratio for m in inliers])) if inliers else 0.0

    if len(inliers) < params.robust_min_inliers:
        reason = "insufficient_robust_inliers"
        accepted = False
    elif inlier_ratio_eligible < params.robust_min_inlier_ratio:
        reason = "insufficient_robust_inlier_ratio"
        accepted = False
    elif med_rmse > params.robust_max_median_rmse:
        reason = "median_registration_rmse_too_high"
        accepted = False
    elif trans_spread > params.robust_max_translation_spread_m:
        reason = "transform_translation_spread_too_high"
        accepted = False
    elif yaw_spread > params.robust_max_yaw_spread_rad:
        reason = "transform_yaw_spread_too_high"
        accepted = False
    else:
        reason = "robust_inlier_set_accepted"
        accepted = True

    return RobustSelectionResult(
        accepted=accepted,
        reason=reason,
        inliers=inliers,
        rejected=rejected,
        transform=transform,
        inlier_ratio=inlier_ratio_eligible,
        median_rmse=med_rmse,
        translation_spread_m=trans_spread,
        yaw_spread_rad=yaw_spread,
        raw_consistent_matches=edge_count,
        raw_verified_count=raw_verified_count,
        prefiltered_count=len(eligible),
        eligible_count=len(consensus_pool),
        deduplicated_count=len(pool),
        inlier_ratio_raw=inlier_ratio_raw,
        inlier_ratio_eligible=inlier_ratio_eligible,
        median_descriptor_distance=med_desc,
        median_inlier_ratio=med_inlier_ratio,
    )
