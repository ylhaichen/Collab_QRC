from nav_msgs.msg import OccupancyGrid, Odometry

from point_lio_ros2_adapter.point_lio_ros2_adapter_node import odom_within_map_bounds


def _map() -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.info.width = 10
    msg.info.height = 10
    msg.info.resolution = 1.0
    msg.info.origin.position.x = -5.0
    msg.info.origin.position.y = -5.0
    msg.data = [0] * 100
    return msg


def _odom(x: float, y: float) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_odom_within_map_bounds_accepts_pose_inside_grid() -> None:
    assert odom_within_map_bounds(_odom(0.0, 0.0), _map(), margin_m=0.0)


def test_odom_within_map_bounds_rejects_pose_outside_grid() -> None:
    assert not odom_within_map_bounds(_odom(11.66, -99.40), _map(), margin_m=1.0)
