from nav_msgs.msg import OccupancyGrid

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import (
    shared_map_quality_summary,
    shared_map_quality_usable,
)


def _grid(values: list[int]) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.info.width = len(values)
    msg.info.height = 1
    msg.info.resolution = 0.05
    msg.data = values
    return msg


def test_shared_map_quality_rejects_occupied_heavy_grid() -> None:
    msg = _grid([100] * 94 + [0] * 3 + [-1] * 3)

    summary = shared_map_quality_summary(msg, occ_threshold=50, unknown_value=-1)

    assert summary["occupied_ratio"] == 0.94
    assert not shared_map_quality_usable(
        msg,
        occ_threshold=50,
        unknown_value=-1,
        max_occupied_ratio=0.70,
        min_free_ratio=0.01,
        min_known_cells=20,
    )


def test_shared_map_quality_accepts_balanced_grid() -> None:
    msg = _grid([100] * 10 + [0] * 50 + [-1] * 40)

    summary = shared_map_quality_summary(msg, occ_threshold=50, unknown_value=-1)

    assert summary["free_ratio"] == 0.50
    assert shared_map_quality_usable(
        msg,
        occ_threshold=50,
        unknown_value=-1,
        max_occupied_ratio=0.70,
        min_free_ratio=0.01,
        min_known_cells=20,
    )
