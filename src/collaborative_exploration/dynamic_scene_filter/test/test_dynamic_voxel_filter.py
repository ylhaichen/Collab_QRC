from dynamic_scene_filter.temporal_voxel_filter import (
    DynamicFilterParams,
    TemporalVoxelFilter,
)
from dynamic_scene_filter.frame_transform import (
    Pose2D,
    split_original_points_by_labels,
    transform_body_points_to_odom,
)


def test_repeated_stationary_voxel_becomes_static() -> None:
    filt = TemporalVoxelFilter(
        DynamicFilterParams(
            voxel_size=0.5,
            static_min_observations=3,
            static_min_lifetime_sec=1.0,
            max_static_velocity=0.15,
            min_dynamic_velocity=0.35,
        )
    )

    labels = []
    for i, t in enumerate((0.0, 0.6, 1.2)):
        result = filt.classify_points([(1.0, 0.0, 0.2)], stamp_sec=t)
        labels.append(result.labels[0])

    assert labels[-1] == "static"
    assert result.static_points == [(1.0, 0.0, 0.2)]
    assert result.dynamic_points == []


def test_fast_moving_voxel_is_dynamic_and_decays() -> None:
    filt = TemporalVoxelFilter(
        DynamicFilterParams(
            voxel_size=0.25,
            static_min_observations=3,
            static_min_lifetime_sec=2.0,
            dynamic_obstacle_ttl_sec=1.0,
            max_static_velocity=0.15,
            min_dynamic_velocity=0.35,
            track_new_voxel_motion=True,
        )
    )

    filt.classify_points([(0.0, 0.0, 0.1)], stamp_sec=0.0)
    result = filt.classify_points([(1.0, 0.0, 0.1)], stamp_sec=0.5)

    assert result.labels[0] == "dynamic"
    assert result.dynamic_points == [(1.0, 0.0, 0.1)]

    filt.prune(stamp_sec=2.0)
    assert filt.dynamic_voxel_count == 0


def test_body_frame_static_wall_is_classified_in_odom_frame() -> None:
    filt = TemporalVoxelFilter(
        DynamicFilterParams(
            voxel_size=0.5,
            static_min_observations=3,
            static_min_lifetime_sec=1.0,
            max_static_velocity=0.15,
            min_dynamic_velocity=0.35,
        )
    )

    result = None
    latest_body_points = []
    # The same world point at odom x=3.0 appears to move in the robot body
    # frame as the robot drives forward. Classification must use odom points,
    # while the published cloud keeps body-frame points for Scan Context.
    for robot_x, stamp_sec in ((0.0, 0.0), (1.0, 0.6), (2.0, 1.2)):
        latest_body_points = [(3.0 - robot_x, 0.0, 0.2)]
        odom_points = transform_body_points_to_odom(
            latest_body_points,
            Pose2D(x=robot_x, y=0.0, z=0.0, yaw=0.0),
        )
        result = filt.classify_points(odom_points, stamp_sec=stamp_sec)

    assert result is not None
    assert result.labels == ["static"]

    static_points, dynamic_points = split_original_points_by_labels(latest_body_points, result.labels)
    assert static_points == [(1.0, 0.0, 0.2)]
    assert dynamic_points == []


def test_near_robot_points_are_excluded_from_static_cloud() -> None:
    filt = TemporalVoxelFilter(
        DynamicFilterParams(
            near_robot_ignore_radius=0.6,
            static_min_observations=1,
            static_min_lifetime_sec=0.0,
        )
    )

    result = filt.classify_points([(0.35, 0.0, 0.2), (1.2, 0.0, 0.2)], stamp_sec=3.0)

    assert result.labels == ["ignored_near_robot", "static"]
    static_points, dynamic_points = split_original_points_by_labels(
        [(0.35, 0.0, 0.2), (1.2, 0.0, 0.2)],
        result.labels,
    )
    assert static_points == [(1.2, 0.0, 0.2)]
    assert dynamic_points == []
