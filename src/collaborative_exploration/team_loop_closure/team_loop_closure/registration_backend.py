from __future__ import annotations

from dataclasses import dataclass
import shutil

import numpy as np

from .common import icp_2d, wrap_pi


@dataclass(frozen=True)
class RegistrationResult:
    transform: np.ndarray
    fitness_m: float
    inlier_ratio: float
    num_correspondences: int
    backend: str
    requested_backend: str = "icp_2d"
    dependency_blocker: str = ""
    geometric_verification_required: bool = True


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
    kiss_matcher_available: bool | None = None,
) -> RegistrationResult:
    """Register compact static keyframe clouds.

    KISS-Matcher is the preferred production backend when it is available as
    an external executable/library. This Python path does not fake that
    backend. If KISS-Matcher is requested but unavailable, it records the exact
    blocker and falls back to ICP 2D for geometric verification.
    """

    requested_backend = str(backend or "icp_2d").strip().lower()
    if requested_backend in {"gicp_only", "gicp_optional", "teaser"}:
        requested_backend = "gicp_optional"
    if requested_backend not in {"icp_2d", "gicp_optional", "kiss_matcher"}:
        requested_backend = "icp_2d"
    dependency_blocker = ""
    backend_name = requested_backend
    if requested_backend == "kiss_matcher":
        available = bool(shutil.which("kiss_matcher")) if kiss_matcher_available is None else kiss_matcher_available
        backend_name = "icp_2d"
        if available:
            backend_name = "icp_2d"
            dependency_blocker = "kiss_matcher_cli_integration_not_configured"
        else:
            dependency_blocker = "kiss_matcher_not_available"
    elif requested_backend == "gicp_optional":
        backend_name = "icp_2d"
        dependency_blocker = "gicp_optional_not_available"

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
        backend=backend_name,
        requested_backend=requested_backend,
        dependency_blocker=dependency_blocker,
        geometric_verification_required=True,
    )
