from go2_nav_algorithms.nav_costmap_utils import (
    GridSpec,
    classify_cost,
    make_start_cell_diagnostics,
    project_frontier_goal,
)


def test_start_cell_diagnostics_classifies_lethal_start_and_nearest_free_distance() -> None:
    spec = GridSpec(width=7, height=7, resolution=0.1, origin_x=-0.35, origin_y=-0.35)
    data = [0] * (spec.width * spec.height)
    center = 3 * spec.width + 3
    data[center] = 100

    diag = make_start_cell_diagnostics(
        local_data=data,
        local_spec=spec,
        global_data=data,
        global_spec=spec,
        pose_x=0.0,
        pose_y=0.0,
        footprint=[(0.25, 0.12), (0.25, -0.12), (-0.25, -0.12), (-0.25, 0.12)],
        local_inflation_radius=0.22,
        global_inflation_radius=0.20,
        robot_radius=0.35,
        lethal_threshold=90,
    )

    assert diag["local_start_cell_cost"] == 100
    assert diag["global_start_cell_cost"] == 100
    assert diag["local_start_cell_status"] == "lethal"
    assert diag["global_start_cell_status"] == "lethal"
    assert diag["start_cell_lethal"] is True
    assert diag["nearest_free_cell_distance_m"] == 0.1
    assert diag["footprint_collision_status"] == "collision"


def test_project_frontier_goal_returns_nearby_free_approach_pose_not_raw_obstacle() -> None:
    spec = GridSpec(width=11, height=11, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [-1] * (spec.width * spec.height)
    raw_idx = 5 * spec.width + 5
    approach_idx = 5 * spec.width + 4
    data[raw_idx] = 100
    data[approach_idx] = 0

    result = project_frontier_goal(
        data,
        spec,
        raw_x=5.5,
        raw_y=5.5,
        current_x=1.5,
        current_y=5.5,
        start_x=1.5,
        start_y=5.5,
        min_current_distance=1.0,
        min_start_distance=2.0,
        failed_goals=[],
        failed_goal_radius=1.0,
        max_search_radius_m=2.0,
        lethal_threshold=90,
    )

    assert result.projection_success is True
    assert result.raw_goal == (5.5, 5.5)
    assert result.projected_goal == (4.5, 5.5)
    assert result.projected_goal_cost == 0
    assert classify_cost(result.projected_goal_cost) == "free"


def test_project_frontier_goal_rejects_blacklisted_projected_pose() -> None:
    spec = GridSpec(width=11, height=11, resolution=1.0, origin_x=0.0, origin_y=0.0)
    data = [-1] * (spec.width * spec.height)
    data[5 * spec.width + 5] = 100
    data[5 * spec.width + 4] = 0

    result = project_frontier_goal(
        data,
        spec,
        raw_x=5.5,
        raw_y=5.5,
        current_x=1.5,
        current_y=5.5,
        start_x=1.5,
        start_y=5.5,
        min_current_distance=1.0,
        min_start_distance=2.0,
        failed_goals=[(4.5, 5.5)],
        failed_goal_radius=1.0,
        max_search_radius_m=2.0,
        lethal_threshold=90,
    )

    assert result.projection_success is False
    assert result.reason == "no_reachable_free_approach_pose"
