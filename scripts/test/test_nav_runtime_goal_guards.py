from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from nav_msgs.msg import OccupancyGrid, Odometry


ROOT = Path(__file__).resolve().parents[1]


def _load_script(name: str):
    path = ROOT / "runtime" / f"{name}.py"
    spec = spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def test_cfpa2_to_nav2_bridge_rejects_goal_forwarding_when_pose_outside_map() -> None:
    bridge = _load_script("cfpa2_to_nav2_bridge")

    assert not bridge.odom_pose_inside_map(_odom(11.66, -99.40), _map(), margin_m=1.0)


def test_stuck_watchdog_rejects_republish_when_pose_outside_map() -> None:
    watchdog = _load_script("stuck_watchdog")

    assert not watchdog.odom_pose_inside_map(_odom(11.66, -99.40), _map(), margin_m=1.0)
