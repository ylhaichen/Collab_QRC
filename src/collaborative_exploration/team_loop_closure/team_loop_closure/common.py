from __future__ import annotations

import json
import math
from typing import Any, Iterable

import numpy as np


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def se2_from_xyyaw(x: float, y: float, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    out = np.eye(3, dtype=np.float64)
    out[0, 0] = c
    out[0, 1] = -s
    out[1, 0] = s
    out[1, 1] = c
    out[0, 2] = x
    out[1, 2] = y
    return out


def xyyaw_from_se2(t: np.ndarray) -> tuple[float, float, float]:
    return (
        float(t[0, 2]),
        float(t[1, 2]),
        math.atan2(float(t[1, 0]), float(t[0, 0])),
    )


def invert_se2(t: np.ndarray) -> np.ndarray:
    r = t[:2, :2]
    p = t[:2, 2]
    out = np.eye(3, dtype=np.float64)
    out[:2, :2] = r.T
    out[:2, 2] = -r.T @ p
    return out


def transform_points_xy(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 2)
    return (points[:, :2] @ t[:2, :2].T) + t[:2, 2]


def pose_dict_from_msg(msg: Any) -> dict[str, float]:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {
        "x": float(p.x),
        "y": float(p.y),
        "z": float(p.z),
        "yaw": float(yaw_from_quat(q)),
    }


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def dumps_compact(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def loads_dict(data: str) -> dict[str, Any] | None:
    try:
        obj = json.loads(data)
    except json.JSONDecodeError:
        return None
    return obj if isinstance(obj, dict) else None


def scan_context(
    points_xyz: np.ndarray,
    *,
    rings: int,
    sectors: int,
    max_radius: float,
) -> np.ndarray:
    """Build a compact Scan Context descriptor from a local LiDAR keyframe.

    The descriptor follows the standard polar ring/sector layout. Each bin
    stores normalized max relative height, which is closer to canonical Scan
    Context than the previous occupancy-count proxy while still staying fully
    self-contained in Python/Numpy.
    """
    desc = np.zeros((rings, sectors), dtype=np.float32)
    if points_xyz.size == 0 or max_radius <= 1e-6:
        return desc
    xy = points_xyz[:, :2]
    radii = np.linalg.norm(xy, axis=1)
    mask = (radii > 0.15) & (radii <= max_radius)
    if not np.any(mask):
        return desc
    pts = points_xyz[mask]
    radii = radii[mask]
    angles = np.arctan2(pts[:, 1], pts[:, 0])
    ring_idx = np.minimum((radii / max_radius * rings).astype(np.int32), rings - 1)
    sector_idx = np.floor((angles + math.pi) / (2.0 * math.pi) * sectors).astype(np.int32)
    sector_idx = np.clip(sector_idx, 0, sectors - 1)
    heights = pts[:, 2] - float(np.min(pts[:, 2]))
    np.maximum.at(desc, (ring_idx, sector_idx), heights.astype(np.float32))
    if desc.max() > 0.0:
        desc /= desc.max()
    return desc


def scan_context_ring_key(desc: np.ndarray) -> np.ndarray:
    if desc.ndim != 2 or desc.size == 0:
        return np.empty((0,), dtype=np.float32)
    return np.mean(desc, axis=1).astype(np.float32)


def scan_context_sector_key(desc: np.ndarray) -> np.ndarray:
    if desc.ndim != 2 or desc.size == 0:
        return np.empty((0,), dtype=np.float32)
    return np.mean(desc, axis=0).astype(np.float32)


def ring_key_distance(a: np.ndarray, b: np.ndarray) -> float:
    if a.shape != b.shape or a.size == 0:
        return float("inf")
    return float(np.linalg.norm(a - b) / math.sqrt(float(max(1, a.size))))


def descriptor_distance_and_shift(a: np.ndarray, b: np.ndarray) -> tuple[float, int]:
    if a.shape != b.shape or a.size == 0:
        return (float("inf"), 0)
    best = float("inf")
    best_shift = 0
    sectors = a.shape[1]
    an = np.linalg.norm(a)
    if an < 1e-6:
        return (float("inf"), 0)
    for shift in range(sectors):
        rb = np.roll(b, shift, axis=1)
        denom = an * max(np.linalg.norm(rb), 1e-6)
        # Cosine distance in [0, 2]. Lower is better.
        dist = 1.0 - float(np.sum(a * rb) / denom)
        if dist < best:
            best = dist
            best_shift = shift
    return (best, best_shift)


def sector_shift_to_yaw(shift: int, sectors: int) -> float:
    # Positive roll of source descriptor roughly means source needs negative
    # yaw rotation to align to target.
    return wrap_pi(-2.0 * math.pi * float(shift) / float(max(1, sectors)))


def rigid_transform_2d(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    if src.shape[0] < 3 or dst.shape[0] < 3:
        return np.eye(3, dtype=np.float64)
    cs = np.mean(src, axis=0)
    cd = np.mean(dst, axis=0)
    xs = src - cs
    xd = dst - cd
    h = xs.T @ xd
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = cd - r @ cs
    out = np.eye(3, dtype=np.float64)
    out[:2, :2] = r
    out[:2, 2] = t
    return out


def nearest_neighbor_indices(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # Brute-force is intentional: clouds are compact keyframe samples.
    diff = src[:, None, :] - dst[None, :, :]
    dist2 = np.sum(diff * diff, axis=2)
    idx = np.argmin(dist2, axis=1)
    d = np.sqrt(dist2[np.arange(src.shape[0]), idx])
    return idx, d


def icp_2d(
    src_xy: np.ndarray,
    dst_xy: np.ndarray,
    *,
    initial_yaw: float,
    max_iterations: int,
    max_corr_dist: float,
) -> tuple[np.ndarray, float, float]:
    if src_xy.shape[0] < 12 or dst_xy.shape[0] < 12:
        return (se2_from_xyyaw(0.0, 0.0, initial_yaw), float("inf"), 0.0)
    total = se2_from_xyyaw(0.0, 0.0, initial_yaw)
    prev = float("inf")
    for _ in range(max_iterations):
        moved = transform_points_xy(src_xy, total)
        idx, dist = nearest_neighbor_indices(moved, dst_xy)
        mask = dist <= max_corr_dist
        if int(np.count_nonzero(mask)) < 8:
            break
        delta = rigid_transform_2d(moved[mask], dst_xy[idx[mask]])
        total = delta @ total
        mean = float(np.mean(dist[mask]))
        if abs(prev - mean) < 1e-4:
            prev = mean
            break
        prev = mean
    moved = transform_points_xy(src_xy, total)
    _idx, dist = nearest_neighbor_indices(moved, dst_xy)
    mask = dist <= max_corr_dist
    inlier_ratio = float(np.count_nonzero(mask)) / float(max(1, src_xy.shape[0]))
    fitness = float(np.mean(dist[mask])) if np.any(mask) else float("inf")
    return (total, fitness, inlier_ratio)


def circular_mean(angles: Iterable[float]) -> float:
    angles = list(angles)
    if not angles:
        return 0.0
    return math.atan2(
        sum(math.sin(a) for a in angles),
        sum(math.cos(a) for a in angles),
    )
