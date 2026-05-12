#!/usr/bin/env python3
"""cfpa2_to_nav2_bridge — translate CFPA2 way_point into Nav2 goal_pose.

CFPA2 publishes /<ns>/way_point as PointStamped (just an XY target with
no orientation). Nav2's bt_navigator subscribes to /<ns>/goal_pose as
PoseStamped (full pose with orientation). This bridge:

  - subscribes /<ns>/way_point (BEST_EFFORT to match CFPA2's QoS)
  - synthesizes orientation = atan2(goal - robot_pose) so the planner
    has a sensible terminal heading
  - publishes /<ns>/goal_pose (BEST_EFFORT to match Nav2's QoS) only
    when the goal *changed* — re-publishing identical goals at 2 Hz
    would force Nav2 to abort + replan every tick

Run alongside:
  - the sim (any backend including 'none')
  - cfpa2_coordinator (with explore:=true)
  - nav2_robot_a.launch.py (the Nav2 stack)
"""
from __future__ import annotations

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


def odom_pose_inside_map(odom: Odometry, map_msg: OccupancyGrid | None, *, margin_m: float) -> bool:
    if map_msg is None:
        return True
    x = float(odom.pose.pose.position.x)
    y = float(odom.pose.pose.position.y)
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)
    resolution = float(map_msg.info.resolution)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False
    margin = max(0.0, float(margin_m))
    min_x = float(map_msg.info.origin.position.x) - margin
    min_y = float(map_msg.info.origin.position.y) - margin
    max_x = float(map_msg.info.origin.position.x) + float(width) * resolution + margin
    max_y = float(map_msg.info.origin.position.y) + float(height) * resolution + margin
    return min_x <= x <= max_x and min_y <= y <= max_y


class Cfpa2ToNav2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("cfpa2_to_nav2_bridge")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("waypoint_topic", "way_point")
        self.declare_parameter("goal_pose_topic", "goal_pose")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("reject_when_odom_outside_map", True)
        self.declare_parameter("map_bounds_margin_m", 2.0)
        # Skip republishing if new goal is within this distance of last
        # published goal — CFPA2 republishes its current goal at 2 Hz to
        # keep the channel alive, we don't want Nav2 to restart every tick.
        self.declare_parameter("goal_change_min_m", 0.30)

        ns = str(self.get_parameter("namespace").value)
        wp_topic = f"/{ns}/{self.get_parameter('waypoint_topic').value}"
        goal_topic = f"/{ns}/{self.get_parameter('goal_pose_topic').value}"
        odom_topic = f"/{ns}/{self.get_parameter('odom_topic').value}"
        map_topic = f"/{ns}/{self.get_parameter('map_topic').value}"
        self.goal_change_min_m = float(
            self.get_parameter("goal_change_min_m").value
        )
        self.reject_when_odom_outside_map = bool(
            self.get_parameter("reject_when_odom_outside_map").value
        )
        self.map_bounds_margin_m = max(0.0, float(self.get_parameter("map_bounds_margin_m").value))

        # CFPA2's way_point_coord publishes RELIABLE; odom_relay also reliable.
        cfpa_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Nav2's bt_navigator subscribes goal_pose with rclcpp::SystemDefaultsQoS,
        # which is RELIABLE/VOLATILE/KEEP_LAST(10). Earlier comment claiming
        # BEST_EFFORT was wrong — published goals were silently dropped on the
        # wire (publisher logs "forwarded goal" but bt_navigator never received
        # them). Confirmed via the runtime warning:
        #   "New publisher discovered on '/<ns>/goal_pose'... incompatible
        #    QoS. Last incompatible policy: RELIABILITY".
        nav2_goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._last_pose_x: float | None = None
        self._last_pose_y: float | None = None
        self._last_odom: Odometry | None = None
        self._latest_map: OccupancyGrid | None = None
        self._last_goal_x: float | None = None
        self._last_goal_y: float | None = None
        self._last_invalid_odom_warn_ns = 0

        self.create_subscription(Odometry, odom_topic, self._on_odom, cfpa_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, cfpa_qos)
        self.create_subscription(
            PointStamped, wp_topic, self._on_waypoint, cfpa_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, goal_topic, nav2_goal_qos
        )

        self.get_logger().info(
            f"bridge armed. {wp_topic} → {goal_topic}; pose from {odom_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_waypoint(self, msg: PointStamped) -> None:
        gx, gy = float(msg.point.x), float(msg.point.y)
        if (
            self.reject_when_odom_outside_map
            and self._last_odom is not None
            and not odom_pose_inside_map(
                self._last_odom,
                self._latest_map,
                margin_m=self.map_bounds_margin_m,
            )
        ):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_invalid_odom_warn_ns > int(2e9):
                pos = self._last_odom.pose.pose.position
                self.get_logger().warn(
                    "dropping CFPA2 waypoint because odom/nav is outside map bounds "
                    f"pose=({float(pos.x):+.2f},{float(pos.y):+.2f})"
                )
                self._last_invalid_odom_warn_ns = now_ns
            return
        # Suppress duplicate / sub-threshold-change goals.
        if (
            self._last_goal_x is not None
            and math.hypot(gx - self._last_goal_x, gy - self._last_goal_y)
            < self.goal_change_min_m
        ):
            return

        # Synthesize orientation pointing from current robot pose to goal.
        # If we have no odom yet, default to facing +x (yaw=0).
        yaw = 0.0
        if self._last_pose_x is not None:
            dx = gx - self._last_pose_x
            dy = gy - self._last_pose_y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or "map"
        out.pose.position.x = gx
        out.pose.position.y = gy
        out.pose.position.z = 0.0
        # quaternion from yaw alone (z-axis rotation).
        out.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.orientation.w = math.cos(0.5 * yaw)
        self._goal_pub.publish(out)

        self.get_logger().info(
            f"forwarded goal ({gx:+.2f}, {gy:+.2f}) yaw={math.degrees(yaw):+.1f}°"
        )
        self._last_goal_x = gx
        self._last_goal_y = gy


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    try:
        node = Cfpa2ToNav2Bridge()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
