from nav_msgs.msg import OccupancyGrid, Odometry

from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import odom_inside_map_bounds
from cfpa2_collaborative_autonomy.cfpa2_coordinator_node import goal_within_robot_distance


def _map() -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.info.width = 20
    msg.info.height = 20
    msg.info.resolution = 0.5
    msg.info.origin.position.x = -5.0
    msg.info.origin.position.y = -5.0
    msg.data = [0] * 400
    return msg


def _odom(x: float, y: float) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_cfpa2_accepts_start_pose_inside_planning_map() -> None:
    assert odom_inside_map_bounds(_odom(0.0, 0.0), _map(), margin_m=0.0)


def test_cfpa2_rejects_start_pose_outside_planning_map() -> None:
    assert not odom_inside_map_bounds(_odom(11.66, -99.40), _map(), margin_m=1.0)


def test_cfpa2_rejects_far_fallback_frontier_goal() -> None:
    assert not goal_within_robot_distance(
        robot_xy=(2.5, 2.8),
        goal_xy=(45.72, -0.16),
        max_distance_m=20.0,
    )


def test_cfpa2_accepts_near_fallback_frontier_goal() -> None:
    assert goal_within_robot_distance(
        robot_xy=(2.5, 2.8),
        goal_xy=(12.0, 4.0),
        max_distance_m=20.0,
    )
