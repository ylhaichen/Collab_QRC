import importlib.util
import math
from pathlib import Path

import numpy as np


def _load_map_augmenter():
    path = Path(__file__).resolve().parents[1] / "runtime" / "map_augmenter.py"
    spec = importlib.util.spec_from_file_location("map_augmenter", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_clear_robot_footprint_cells_frees_only_current_body_region() -> None:
    mod = _load_map_augmenter()
    width = 30
    height = 30
    resolution = 0.1
    origin_x = -1.5
    origin_y = -1.5
    data = np.full(width * height, -1, dtype=np.int8)

    center = 15 * width + 15
    far = 25 * width + 25
    data[center] = 100
    data[far] = 100

    cleared = mod.clear_robot_footprint_cells(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        footprint_length_m=0.70,
        footprint_width_m=0.40,
        padding_m=0.04,
    )

    assert cleared > 0
    assert data[center] == 0
    assert data[far] == 100


def test_clear_robot_footprint_cells_respects_robot_yaw() -> None:
    mod = _load_map_augmenter()
    width = 40
    height = 40
    resolution = 0.1
    origin_x = -2.0
    origin_y = -2.0
    data = np.full(width * height, -1, dtype=np.int8)

    # With yaw=90deg, the long footprint axis lies along world Y. This cell
    # is inside the rotated body rectangle, while the same X offset is outside.
    inside_y = int((0.30 - origin_y) / resolution) * width + int((0.00 - origin_x) / resolution)
    outside_x = int((0.00 - origin_y) / resolution) * width + int((0.30 - origin_x) / resolution)
    data[inside_y] = 100
    data[outside_x] = 100

    mod.clear_robot_footprint_cells(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=math.pi / 2.0,
        footprint_length_m=0.70,
        footprint_width_m=0.20,
        padding_m=0.0,
    )

    assert data[inside_y] == 0
    assert data[outside_x] == 100


def test_clear_robot_footprint_cells_supports_costmap_self_clear_radius() -> None:
    mod = _load_map_augmenter()
    width = 50
    height = 50
    resolution = 0.05
    origin_x = -1.25
    origin_y = -1.25
    data = np.full(width * height, -1, dtype=np.int8)

    # This cell is outside the rectangular body footprint but inside the
    # requested circular costmap self-clear radius. It represents a leg or
    # near-base return that can poison Nav2's start cell neighborhood.
    near_leg = int((0.0 - origin_y) / resolution) * width + int((0.55 - origin_x) / resolution)
    far_wall = int((0.0 - origin_y) / resolution) * width + int((0.90 - origin_x) / resolution)
    data[near_leg] = 100
    data[far_wall] = 100

    cleared = mod.clear_robot_footprint_cells(
        data,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        footprint_length_m=0.65,
        footprint_width_m=0.30,
        padding_m=0.0,
        self_clear_radius_m=0.65,
    )

    assert cleared > 0
    assert data[near_leg] == 0
    assert data[far_wall] == 100
