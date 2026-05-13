from team_loop_closure.occupancy_grid_utils import (
    AlignmentSnapshot,
    OccupancyKeyframe,
    LocalGridSpec,
    SimpleOccupancyGrid,
    build_static_grid_from_keyframes,
    grid_has_occupied_cells,
    merge_local_grids_if_aligned,
    merged_grid_status_payload,
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


def test_keyframe_rebuild_requires_repeated_static_observations_and_clears_self_footprint() -> None:
    spec = LocalGridSpec(resolution=0.5, size_x=6.0, size_y=6.0, height_min=-0.2, height_max=1.5)
    repeated = [
        OccupancyKeyframe(
            keyframe_id="kf0",
            corrected_pose_xyyaw=(0.0, 0.0, 0.0),
            points_xyz=[(2.0, 0.0, 0.2), (0.1, 0.0, 0.2)],
            stamp_sec=0.0,
            source="cloud_static",
        ),
        OccupancyKeyframe(
            keyframe_id="kf1",
            corrected_pose_xyyaw=(0.0, 0.0, 0.0),
            points_xyz=[(2.0, 0.0, 0.2), (0.1, 0.0, 0.2)],
            stamp_sec=1.0,
            source="cloud_static",
        ),
    ]

    single_grid = build_static_grid_from_keyframes(
        repeated[:1],
        frame_id="robot_a/map",
        spec=spec,
        static_min_observations=2,
        self_clear_radius=0.45,
    )
    rebuilt = build_static_grid_from_keyframes(
        repeated,
        frame_id="robot_a/map",
        spec=spec,
        static_min_observations=2,
        self_clear_radius=0.45,
    )

    assert single_grid.value_at_world(2.0, 0.0) != 100
    assert rebuilt.value_at_world(2.0, 0.0) == 100
    assert rebuilt.value_at_world(0.0, 0.0) == 0
    assert rebuilt.value_at_world(0.1, 0.0) == 0


def test_keyframe_rebuild_uses_corrected_pose_source() -> None:
    spec = LocalGridSpec(resolution=0.5, size_x=8.0, size_y=8.0, height_min=-0.2, height_max=1.5)
    keyframes = [
        OccupancyKeyframe(
            keyframe_id="kf0",
            corrected_pose_xyyaw=(1.0, 0.0, 0.0),
            points_xyz=[(1.0, 0.0, 0.2)],
            stamp_sec=0.0,
            source="cloud_static",
        ),
        OccupancyKeyframe(
            keyframe_id="kf1",
            corrected_pose_xyyaw=(1.0, 0.0, 0.0),
            points_xyz=[(1.0, 0.0, 0.2)],
            stamp_sec=1.0,
            source="cloud_static",
        ),
    ]

    rebuilt = build_static_grid_from_keyframes(
        keyframes,
        frame_id="robot_a/map",
        spec=spec,
        static_min_observations=2,
        self_clear_radius=0.45,
    )

    assert rebuilt.value_at_world(2.0, 0.0) == 100
    assert rebuilt.value_at_world(1.0, 0.0) == 0


def test_merged_grid_status_payload_reports_inactive_reason_before_alignment() -> None:
    payload = merged_grid_status_payload(
        alignment_status="tentative",
        gt_used_runtime=False,
        robot_a_grid_received=True,
        robot_b_grid_received=True,
        relative_transform_received=True,
        merged_map_enabled_time_sec=None,
        merged_grid_published=False,
    )

    assert payload["active"] is False
    assert payload["reason"] == "alignment_status_not_aligned"
    assert payload["robot_a_grid_received"] is True
    assert payload["robot_b_grid_received"] is True
    assert payload["relative_transform_received"] is True
    assert payload["merged_grid_published"] is False
