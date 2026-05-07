from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .common import icp_2d, wrap_pi


@dataclass(frozen=True)
class RegistrationResult:
    transform: np.ndarray
    fitness_m: float
    inlier_ratio: float
    num_correspondences: int
    backend: str


def register_keyframe_clouds(
    source_xy: np.ndarray,
    target_xy: np.ndarray,
    *,
    backend: str,
    initial_yaw: float,
    yaw_search_sectors: int,
    sector_count: int,
    max_iterations: int,
    max_corr_dist_m: float,
) -> RegistrationResult:
    """Self-contained registration hook for v1.

    `icp_2d` is the only implemented backend in this stage. Other backend
    names are accepted as aliases so launch files can keep a stable interface
    while KISS-Matcher/TEASER++ remain future optional plugins.
    """

    backend_name = str(backend or "icp_2d").strip().lower()
    if backend_name not in {"icp_2d", "gicp_only", "kiss_matcher", "teaser"}:
        backend_name = "icp_2d"

    yaw_step = 2.0 * np.pi / float(max(1, sector_count))
    best_t = None
    best_fit = float("inf")
    best_inlier = 0.0
    source_count = int(source_xy.shape[0]) if source_xy.ndim == 2 else 0
    for delta in range(-yaw_search_sectors, yaw_search_sectors + 1):
        t, fit, inlier = icp_2d(
            source_xy,
            target_xy,
            initial_yaw=wrap_pi(initial_yaw + delta * yaw_step),
            max_iterations=max_iterations,
            max_corr_dist=max_corr_dist_m,
        )
        if fit < best_fit:
            best_t = t
            best_fit = fit
            best_inlier = inlier

    assert best_t is not None
    return RegistrationResult(
        transform=best_t,
        fitness_m=best_fit,
        inlier_ratio=best_inlier,
        num_correspondences=int(round(best_inlier * float(max(0, source_count)))),
        backend="icp_2d",
    )
