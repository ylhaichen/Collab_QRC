from team_loop_closure.occupancy_grid_utils import (
    AlignmentSnapshot,
    LocalGridSpec,
    SimpleOccupancyGrid,
    grid_has_occupied_cells,
    merge_local_grids_if_aligned,
    project_static_points_to_grid,
)


def test_static_cloud_projection_uses_height_window_and_local_pose() -> None:
    spec = LocalGridSpec(resolution=0.5, size_x=4.0, size_y=4.0, height_min=-0.2, height_max=1.5)

    grid = project_static_points_to_grid(
        points_xyz=[(1.0, 0.0, 0.3), (1.0, 0.0, 2.0), (-1.0, 0.0, -0.4)],
        robot_pose_xyyaw=(1.0, 1.0, 0.0),
        frame_id="robot_a/map",
        spec=spec,
    )

    assert grid.frame_id == "robot_a/map"
    assert grid_has_occupied_cells(grid)
    assert sum(1 for v in grid.data if v == 100) == 1


def test_merged_grid_is_absent_before_alignment() -> None:
    spec = LocalGridSpec(resolution=1.0, size_x=4.0, size_y=4.0)
    grid_a = SimpleOccupancyGrid.empty("robot_a/map", spec)
    grid_b = SimpleOccupancyGrid.empty("robot_b/map", spec)

    merged = merge_local_grids_if_aligned(
        grid_a,
        grid_b,
        AlignmentSnapshot(status="tentative", gt_used_runtime=False, transform_xyyaw=(0.0, 0.0, 0.0)),
        output_frame_id="team_map",
    )

    assert merged is None


def test_merged_grid_rejects_gt_runtime_even_with_aligned_status() -> None:
    spec = LocalGridSpec(resolution=1.0, size_x=4.0, size_y=4.0)
    grid_a = SimpleOccupancyGrid.empty("robot_a/map", spec)
    grid_b = SimpleOccupancyGrid.empty("robot_b/map", spec)

    merged = merge_local_grids_if_aligned(
        grid_a,
        grid_b,
        AlignmentSnapshot(status="aligned", gt_used_runtime=True, transform_xyyaw=(0.0, 0.0, 0.0)),
        output_frame_id="team_map",
    )

    assert merged is None


def test_merged_grid_transforms_robot_b_after_alignment() -> None:
    spec = LocalGridSpec(resolution=1.0, size_x=6.0, size_y=6.0)
    grid_a = SimpleOccupancyGrid.empty("robot_a/map", spec)
    grid_b = SimpleOccupancyGrid.empty("robot_b/map", spec)
    grid_a.set_world_occupied(0.0, 0.0)
    grid_b.set_world_occupied(1.0, 0.0)

    merged = merge_local_grids_if_aligned(
        grid_a,
        grid_b,
        AlignmentSnapshot(status="aligned", gt_used_runtime=False, transform_xyyaw=(2.0, 0.0, 0.0)),
        output_frame_id="team_map",
    )

    assert merged is not None
    assert merged.frame_id == "team_map"
    assert merged.value_at_world(0.0, 0.0) == 100
    assert merged.value_at_world(3.0, 0.0) == 100
