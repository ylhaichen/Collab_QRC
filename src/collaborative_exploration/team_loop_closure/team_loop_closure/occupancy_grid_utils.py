from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class LocalGridSpec:
    resolution: float = 0.1
    size_x: float = 40.0
    size_y: float = 40.0
    height_min: float = -0.2
    height_max: float = 1.5

    @property
    def width(self) -> int:
        return max(1, int(round(self.size_x / self.resolution)))

    @property
    def height(self) -> int:
        return max(1, int(round(self.size_y / self.resolution)))

    @property
    def origin_x(self) -> float:
        return -0.5 * self.size_x

    @property
    def origin_y(self) -> float:
        return -0.5 * self.size_y


@dataclass
class SimpleOccupancyGrid:
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: list[int]

    @classmethod
    def empty(cls, frame_id: str, spec: LocalGridSpec) -> "SimpleOccupancyGrid":
        return cls(
            frame_id=frame_id,
            resolution=float(spec.resolution),
            width=spec.width,
            height=spec.height,
            origin_x=spec.origin_x,
            origin_y=spec.origin_y,
            data=[-1] * (spec.width * spec.height),
        )

    def cell_index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int] | None:
        if self.resolution <= 0.0:
            return None
        fx = (float(wx) - self.origin_x) / self.resolution
        fy = (float(wy) - self.origin_y) / self.resolution
        eps = 1e-9
        if abs(fx - float(self.width)) <= eps:
            fx = float(self.width - 1)
        if abs(fy - float(self.height)) <= eps:
            fy = float(self.height - 1)
        gx = int(math.floor(fx))
        gy = int(math.floor(fy))
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gx, gy

    def cell_center(self, gx: int, gy: int) -> tuple[float, float]:
        return (
            self.origin_x + (float(gx) + 0.5) * self.resolution,
            self.origin_y + (float(gy) + 0.5) * self.resolution,
        )

    def set_world_occupied(self, wx: float, wy: float) -> None:
        cell = self.world_to_cell(wx, wy)
        if cell is None:
            return
        gx, gy = cell
        self.data[self.cell_index(gx, gy)] = 100

    def value_at_world(self, wx: float, wy: float) -> int | None:
        cell = self.world_to_cell(wx, wy)
        if cell is None:
            return None
        return self.data[self.cell_index(cell[0], cell[1])]


@dataclass(frozen=True)
class OccupancyKeyframe:
    keyframe_id: str
    corrected_pose_xyyaw: tuple[float, float, float]
    points_xyz: list[tuple[float, float, float]]
    stamp_sec: float
    source: str = "cloud_static"


@dataclass(frozen=True)
class AlignmentSnapshot:
    status: str
    gt_used_runtime: bool
    transform_xyyaw: tuple[float, float, float] | None


def project_static_points_to_grid(
    *,
    points_xyz: Iterable[tuple[float, float, float]],
    robot_pose_xyyaw: tuple[float, float, float],
    frame_id: str,
    spec: LocalGridSpec,
) -> SimpleOccupancyGrid:
    grid = SimpleOccupancyGrid.empty(frame_id, spec)
    rx, ry, yaw = robot_pose_xyyaw
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    for px, py, pz in points_xyz:
        z = float(pz)
        if z < spec.height_min or z > spec.height_max:
            continue
        wx = float(rx) + c * float(px) - s * float(py)
        wy = float(ry) + s * float(px) + c * float(py)
        grid.set_world_occupied(wx, wy)
    return grid


def build_static_grid_from_keyframes(
    keyframes: Iterable[OccupancyKeyframe],
    *,
    frame_id: str,
    spec: LocalGridSpec,
    static_min_observations: int = 2,
    self_clear_radius: float = 0.45,
) -> SimpleOccupancyGrid:
    grid = SimpleOccupancyGrid.empty(frame_id, spec)
    occupied_counts = [0] * (grid.width * grid.height)
    free_counts = [0] * (grid.width * grid.height)
    poses: list[tuple[float, float, float]] = []
    min_obs = max(1, int(static_min_observations))
    clear_radius = max(0.0, float(self_clear_radius))

    for keyframe in keyframes:
        rx, ry, yaw = keyframe.corrected_pose_xyyaw
        if not all(math.isfinite(float(v)) for v in (rx, ry, yaw)):
            continue
        poses.append((float(rx), float(ry), float(yaw)))
        robot_cell = grid.world_to_cell(rx, ry)
        c = math.cos(float(yaw))
        s = math.sin(float(yaw))
        for px, py, pz in keyframe.points_xyz:
            z = float(pz)
            if z < spec.height_min or z > spec.height_max:
                continue
            body_range = math.hypot(float(px), float(py))
            if body_range <= clear_radius:
                continue
            wx = float(rx) + c * float(px) - s * float(py)
            wy = float(ry) + s * float(px) + c * float(py)
            target_cell = grid.world_to_cell(wx, wy)
            if target_cell is None:
                continue
            if robot_cell is not None:
                for gx, gy in _bresenham_cells(robot_cell, target_cell)[:-1]:
                    free_counts[grid.cell_index(gx, gy)] += 1
            occupied_counts[grid.cell_index(target_cell[0], target_cell[1])] += 1

    for idx, count in enumerate(occupied_counts):
        if count >= min_obs:
            grid.data[idx] = 100
        elif count > 0 or free_counts[idx] > 0:
            grid.data[idx] = 0

    if clear_radius > 0.0:
        for rx, ry, _yaw in poses:
            _clear_disc(grid, rx, ry, clear_radius)
    return grid


def merge_local_grids_if_aligned(
    grid_a: SimpleOccupancyGrid,
    grid_b: SimpleOccupancyGrid,
    alignment: AlignmentSnapshot,
    *,
    output_frame_id: str,
) -> SimpleOccupancyGrid | None:
    if str(alignment.status).strip().lower() != "aligned":
        return None
    if alignment.gt_used_runtime:
        return None
    if alignment.transform_xyyaw is None:
        return None
    tx, ty, yaw = alignment.transform_xyyaw
    if not all(math.isfinite(v) for v in (tx, ty, yaw)):
        return None

    spec = LocalGridSpec(
        resolution=grid_a.resolution,
        size_x=grid_a.width * grid_a.resolution,
        size_y=grid_a.height * grid_a.resolution,
    )
    merged = SimpleOccupancyGrid.empty(output_frame_id, spec)
    merged.origin_x = grid_a.origin_x
    merged.origin_y = grid_a.origin_y

    _copy_occupied_cells(grid_a, merged)

    c = math.cos(yaw)
    s = math.sin(yaw)
    for gy in range(grid_b.height):
        for gx in range(grid_b.width):
            value = grid_b.data[grid_b.cell_index(gx, gy)]
            if value < 50:
                continue
            for bx, by in _cell_samples(grid_b, gx, gy):
                ax = float(tx) + c * bx - s * by
                ay = float(ty) + s * bx + c * by
                merged.set_world_occupied(ax, ay)
    return merged


def merged_grid_status_payload(
    *,
    alignment_status: str,
    gt_used_runtime: bool,
    robot_a_grid_received: bool,
    robot_b_grid_received: bool,
    relative_transform_received: bool,
    merged_map_enabled_time_sec: float | None,
    merged_grid_published: bool,
) -> dict[str, object]:
    status = str(alignment_status or "unknown").strip().lower()
    active = (
        status == "aligned"
        and not bool(gt_used_runtime)
        and bool(robot_a_grid_received)
        and bool(robot_b_grid_received)
        and bool(relative_transform_received)
        and bool(merged_grid_published)
    )
    if active:
        reason = "active"
    elif bool(gt_used_runtime):
        reason = "gt_runtime_blocked"
    elif status != "aligned":
        reason = "alignment_status_not_aligned"
    elif not bool(robot_a_grid_received):
        reason = "waiting_for_robot_a_grid"
    elif not bool(robot_b_grid_received):
        reason = "waiting_for_robot_b_grid"
    elif not bool(relative_transform_received):
        reason = "waiting_for_relative_transform"
    else:
        reason = "waiting_to_publish"
    return {
        "schema": "merged_occupancy_grid_status/v1",
        "active": bool(active),
        "alignment_status": status,
        "reason": reason,
        "robot_a_grid_received": bool(robot_a_grid_received),
        "robot_b_grid_received": bool(robot_b_grid_received),
        "relative_transform_received": bool(relative_transform_received),
        "merged_grid_published": bool(merged_grid_published),
        "gt_used_runtime": bool(gt_used_runtime),
        "merged_map_enabled_time_sec": merged_map_enabled_time_sec,
        # Back-compatible aliases used by older logs/scripts.
        "has_robot_a_grid": bool(robot_a_grid_received),
        "has_robot_b_grid": bool(robot_b_grid_received),
        "has_relative_transform": bool(relative_transform_received),
    }


def grid_has_occupied_cells(grid: SimpleOccupancyGrid) -> bool:
    return any(v >= 50 for v in grid.data)


def _copy_occupied_cells(src: SimpleOccupancyGrid, dst: SimpleOccupancyGrid) -> None:
    for gy in range(src.height):
        for gx in range(src.width):
            value = src.data[src.cell_index(gx, gy)]
            if value < 50:
                continue
            for wx, wy in _cell_samples(src, gx, gy):
                dst.set_world_occupied(wx, wy)


def _cell_samples(grid: SimpleOccupancyGrid, gx: int, gy: int) -> list[tuple[float, float]]:
    x0 = grid.origin_x + float(gx) * grid.resolution
    y0 = grid.origin_y + float(gy) * grid.resolution
    x1 = x0 + grid.resolution
    y1 = y0 + grid.resolution
    cx, cy = grid.cell_center(gx, gy)
    return [(cx, cy), (x0, y0), (x1, y0), (x0, y1), (x1, y1)]


def _bresenham_cells(
    start: tuple[int, int],
    end: tuple[int, int],
) -> list[tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return cells


def _clear_disc(grid: SimpleOccupancyGrid, cx: float, cy: float, radius: float) -> None:
    if grid.resolution <= 0.0:
        return
    cell_radius = int(math.ceil(radius / grid.resolution))
    center = grid.world_to_cell(cx, cy)
    if center is None:
        return
    cgx, cgy = center
    r2 = radius * radius
    for gy in range(max(0, cgy - cell_radius), min(grid.height, cgy + cell_radius + 1)):
        for gx in range(max(0, cgx - cell_radius), min(grid.width, cgx + cell_radius + 1)):
            wx, wy = grid.cell_center(gx, gy)
            if (wx - cx) * (wx - cx) + (wy - cy) * (wy - cy) <= r2:
                grid.data[grid.cell_index(gx, gy)] = 0
