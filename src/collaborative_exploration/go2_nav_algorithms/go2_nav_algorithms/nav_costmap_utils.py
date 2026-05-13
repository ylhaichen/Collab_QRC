from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


@dataclass(frozen=True)
class CellCost:
    cost: int | None
    status: str
    grid_x: int | None
    grid_y: int | None


@dataclass(frozen=True)
class ProjectionResult:
    raw_goal: tuple[float, float]
    projected_goal: tuple[float, float] | None
    projected_goal_cost: int | None
    projection_success: bool
    reason: str


def classify_cost(
    cost: int | None,
    *,
    lethal_threshold: int = 90,
    inflated_threshold: int = 1,
) -> str:
    if cost is None:
        return "outside"
    value = int(cost)
    if value < 0:
        return "unknown"
    if value >= int(lethal_threshold):
        return "lethal"
    if value >= int(inflated_threshold):
        return "inflated"
    return "free"


def world_to_grid(spec: GridSpec, x: float, y: float) -> tuple[int, int] | None:
    if spec.width <= 0 or spec.height <= 0 or spec.resolution <= 0.0:
        return None
    if not (math.isfinite(float(x)) and math.isfinite(float(y))):
        return None
    gx = int(math.floor((float(x) - float(spec.origin_x)) / float(spec.resolution)))
    gy = int(math.floor((float(y) - float(spec.origin_y)) / float(spec.resolution)))
    if gx < 0 or gy < 0 or gx >= int(spec.width) or gy >= int(spec.height):
        return None
    return gx, gy


def grid_to_world(spec: GridSpec, gx: int, gy: int) -> tuple[float, float]:
    return (
        float(spec.origin_x) + (float(gx) + 0.5) * float(spec.resolution),
        float(spec.origin_y) + (float(gy) + 0.5) * float(spec.resolution),
    )


def cell_cost(
    data: Sequence[int],
    spec: GridSpec,
    x: float,
    y: float,
    *,
    lethal_threshold: int = 90,
) -> CellCost:
    cell = world_to_grid(spec, x, y)
    if cell is None:
        return CellCost(None, "outside", None, None)
    gx, gy = cell
    idx = gy * int(spec.width) + gx
    if idx < 0 or idx >= len(data):
        return CellCost(None, "outside", gx, gy)
    cost = int(data[idx])
    return CellCost(cost, classify_cost(cost, lethal_threshold=lethal_threshold), gx, gy)


def nearest_free_distance(
    data: Sequence[int],
    spec: GridSpec,
    x: float,
    y: float,
    *,
    max_radius_m: float = 1.5,
    lethal_threshold: int = 90,
) -> float | None:
    center = world_to_grid(spec, x, y)
    if center is None:
        return None
    cx, cy = center
    max_cells = max(0, int(math.ceil(float(max_radius_m) / float(spec.resolution))))
    best: float | None = None
    for gy in range(max(0, cy - max_cells), min(spec.height, cy + max_cells + 1)):
        for gx in range(max(0, cx - max_cells), min(spec.width, cx + max_cells + 1)):
            idx = gy * spec.width + gx
            if idx >= len(data):
                continue
            if classify_cost(int(data[idx]), lethal_threshold=lethal_threshold) != "free":
                continue
            wx, wy = grid_to_world(spec, gx, gy)
            dist = math.hypot(wx - float(x), wy - float(y))
            if dist <= max_radius_m and (best is None or dist < best):
                best = dist
    if best is None:
        return None
    return round(float(best), 4)


def _point_in_poly(x: float, y: float, poly: Sequence[tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_at_y = (xj - xi) * (y - yi) / max(1e-12, yj - yi) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def _world_footprint(
    footprint: Sequence[tuple[float, float]],
    *,
    pose_x: float,
    pose_y: float,
    pose_yaw: float,
) -> list[tuple[float, float]]:
    c = math.cos(float(pose_yaw))
    s = math.sin(float(pose_yaw))
    return [
        (
            float(pose_x) + c * float(px) - s * float(py),
            float(pose_y) + s * float(px) + c * float(py),
        )
        for px, py in footprint
    ]


def footprint_collision_status(
    data: Sequence[int],
    spec: GridSpec,
    *,
    pose_x: float,
    pose_y: float,
    pose_yaw: float = 0.0,
    footprint: Sequence[tuple[float, float]] = (),
    lethal_threshold: int = 90,
    unknown_is_collision: bool = False,
) -> str:
    if not footprint:
        return cell_cost(data, spec, pose_x, pose_y, lethal_threshold=lethal_threshold).status
    poly = _world_footprint(footprint, pose_x=pose_x, pose_y=pose_y, pose_yaw=pose_yaw)
    min_x = min(p[0] for p in poly)
    max_x = max(p[0] for p in poly)
    min_y = min(p[1] for p in poly)
    max_y = max(p[1] for p in poly)
    lo = world_to_grid(spec, min_x, min_y) or (
        max(0, int(math.floor((min_x - spec.origin_x) / spec.resolution))),
        max(0, int(math.floor((min_y - spec.origin_y) / spec.resolution))),
    )
    hi = world_to_grid(spec, max_x, max_y) or (
        min(spec.width - 1, int(math.floor((max_x - spec.origin_x) / spec.resolution))),
        min(spec.height - 1, int(math.floor((max_y - spec.origin_y) / spec.resolution))),
    )
    saw_unknown = False
    for gy in range(max(0, lo[1]), min(spec.height - 1, hi[1]) + 1):
        for gx in range(max(0, lo[0]), min(spec.width - 1, hi[0]) + 1):
            wx, wy = grid_to_world(spec, gx, gy)
            if not _point_in_poly(wx, wy, poly):
                continue
            idx = gy * spec.width + gx
            if idx >= len(data):
                continue
            status = classify_cost(int(data[idx]), lethal_threshold=lethal_threshold)
            if status == "lethal":
                return "collision"
            if status == "unknown":
                saw_unknown = True
    if saw_unknown and unknown_is_collision:
        return "unknown_collision"
    if saw_unknown:
        return "unknown"
    return "free"


def _failed_near(goal: tuple[float, float], failed_goals: Iterable[tuple[float, float]], radius: float) -> bool:
    r = max(0.0, float(radius))
    return any(math.hypot(goal[0] - fx, goal[1] - fy) <= r for fx, fy in failed_goals)


def project_frontier_goal(
    data: Sequence[int],
    spec: GridSpec,
    *,
    raw_x: float,
    raw_y: float,
    current_x: float,
    current_y: float,
    start_x: float,
    start_y: float,
    min_current_distance: float,
    min_start_distance: float,
    failed_goals: Iterable[tuple[float, float]],
    failed_goal_radius: float,
    max_search_radius_m: float = 1.5,
    lethal_threshold: int = 90,
) -> ProjectionResult:
    raw = (float(raw_x), float(raw_y))
    center = world_to_grid(spec, raw_x, raw_y)
    if center is None:
        return ProjectionResult(raw, None, None, False, "raw_goal_outside_costmap")
    cx, cy = center
    max_cells = max(1, int(math.ceil(float(max_search_radius_m) / float(spec.resolution))))
    candidates: list[tuple[float, float, int, float, float]] = []
    for gy in range(max(0, cy - max_cells), min(spec.height, cy + max_cells + 1)):
        for gx in range(max(0, cx - max_cells), min(spec.width, cx + max_cells + 1)):
            idx = gy * spec.width + gx
            if idx >= len(data):
                continue
            cost = int(data[idx])
            if classify_cost(cost, lethal_threshold=lethal_threshold) != "free":
                continue
            wx, wy = grid_to_world(spec, gx, gy)
            raw_dist = math.hypot(wx - raw[0], wy - raw[1])
            if raw_dist > max_search_radius_m:
                continue
            if math.hypot(wx - float(current_x), wy - float(current_y)) < min_current_distance:
                continue
            if math.hypot(wx - float(start_x), wy - float(start_y)) < min_start_distance:
                continue
            if _failed_near((wx, wy), failed_goals, failed_goal_radius):
                continue
            current_dist = math.hypot(wx - float(current_x), wy - float(current_y))
            candidates.append((raw_dist, -current_dist, cost, wx, wy))
    if not candidates:
        return ProjectionResult(raw, None, None, False, "no_reachable_free_approach_pose")
    raw_dist, _neg_current_dist, cost, wx, wy = min(candidates)
    reason = "raw_goal_free" if raw_dist <= spec.resolution * 0.51 else "projected_to_free_approach_pose"
    return ProjectionResult(raw, (wx, wy), cost, True, reason)


def make_start_cell_diagnostics(
    *,
    local_data: Sequence[int] | None,
    local_spec: GridSpec | None,
    global_data: Sequence[int] | None,
    global_spec: GridSpec | None,
    pose_x: float,
    pose_y: float,
    footprint: Sequence[tuple[float, float]],
    local_inflation_radius: float,
    global_inflation_radius: float,
    robot_radius: float,
    lethal_threshold: int = 90,
    pose_yaw: float = 0.0,
) -> dict[str, object]:
    local = (
        cell_cost(local_data, local_spec, pose_x, pose_y, lethal_threshold=lethal_threshold)
        if local_data is not None and local_spec is not None
        else CellCost(None, "unavailable", None, None)
    )
    global_cell = (
        cell_cost(global_data, global_spec, pose_x, pose_y, lethal_threshold=lethal_threshold)
        if global_data is not None and global_spec is not None
        else CellCost(None, "unavailable", None, None)
    )
    nearest = None
    if global_data is not None and global_spec is not None:
        nearest = nearest_free_distance(
            global_data,
            global_spec,
            pose_x,
            pose_y,
            max_radius_m=max(1.5, float(global_inflation_radius) + float(robot_radius) + 0.5),
            lethal_threshold=lethal_threshold,
        )
    footprint_status = "unavailable"
    if global_data is not None and global_spec is not None:
        footprint_status = footprint_collision_status(
            global_data,
            global_spec,
            pose_x=pose_x,
            pose_y=pose_y,
            pose_yaw=pose_yaw,
            footprint=footprint,
            lethal_threshold=lethal_threshold,
        )
    start_cell_lethal = local.status == "lethal" or global_cell.status == "lethal"
    return {
        "local_start_cell_cost": local.cost,
        "local_start_cell_status": local.status,
        "global_start_cell_cost": global_cell.cost,
        "global_start_cell_status": global_cell.status,
        "start_cell_lethal": bool(start_cell_lethal),
        "nearest_free_cell_distance_m": nearest,
        "footprint_collision_status": footprint_status,
        "inflation_radius": {
            "local": float(local_inflation_radius),
            "global": float(global_inflation_radius),
        },
        "robot_radius": float(robot_radius),
        "robot_footprint": [[float(x), float(y)] for x, y in footprint],
    }
