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
