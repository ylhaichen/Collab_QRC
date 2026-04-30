#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .common import now_sec_from_node, wrap_pi, yaw_from_quat


class PeerObstacleScanNode(Node):
    """Publish a synthetic LaserScan sector for the teammate body.

    robot_self_filter keeps peer geometry out of the SLAM cloud, which is
    correct for mapping. Nav2 still needs a local obstacle for execution, so
    this node adds only the peer footprint back into the local costmap path.
    """

    def __init__(self) -> None:
        super().__init__("peer_obstacle_scan_node")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("peer_namespace", "robot_b")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("peer_radius_m", 0.55)
        self.declare_parameter("range_cap_m", 6.0)
        self.declare_parameter("stale_timeout_sec", 0.5)
        self.declare_parameter("angle_increment_deg", 1.0)

        self.ns = str(self.get_parameter("namespace").value).strip("/")
        self.peer_ns = str(self.get_parameter("peer_namespace").value).strip("/")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.peer_radius_m = max(0.05, float(self.get_parameter("peer_radius_m").value))
        self.range_cap_m = max(0.5, float(self.get_parameter("range_cap_m").value))
        self.stale_timeout_sec = max(0.05, float(self.get_parameter("stale_timeout_sec").value))
        angle_inc = math.radians(max(0.25, float(self.get_parameter("angle_increment_deg").value)))
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = angle_inc
        self.count = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1

        self.self_odom: Odometry | None = None
        self.peer_odom: Odometry | None = None
        self.self_rx_sec = 0.0
        self.peer_rx_sec = 0.0

        self.create_subscription(Odometry, f"/{self.ns}/odom/nav", self._on_self_odom, 10)
        self.create_subscription(Odometry, f"/{self.peer_ns}/odom/nav", self._on_peer_odom, 10)
        self.pub = self.create_publisher(LaserScan, f"/{self.ns}/peer_scan", qos_profile_sensor_data)

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"peer_obstacle_scan_node up: /{self.ns}/peer_scan sees /{self.peer_ns} "
            f"radius={self.peer_radius_m:.2f}m frame={self.base_frame}"
        )

    def _on_self_odom(self, msg: Odometry) -> None:
        self.self_odom = msg
        self.self_rx_sec = now_sec_from_node(self)

    def _on_peer_odom(self, msg: Odometry) -> None:
        self.peer_odom = msg
        self.peer_rx_sec = now_sec_from_node(self)

    def _empty_scan(self) -> LaserScan:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.05
        msg.range_max = self.range_cap_m
        msg.ranges = [float("inf")] * self.count
        return msg

    def _tick(self) -> None:
        scan = self._empty_scan()
        now = now_sec_from_node(self)
        if (
            self.self_odom is None
            or self.peer_odom is None
            or (now - self.self_rx_sec) > self.stale_timeout_sec
            or (now - self.peer_rx_sec) > self.stale_timeout_sec
        ):
            self.pub.publish(scan)
            return

        sp = self.self_odom.pose.pose.position
        pp = self.peer_odom.pose.pose.position
        yaw = yaw_from_quat(self.self_odom.pose.pose.orientation)
        dx = float(pp.x) - float(sp.x)
        dy = float(pp.y) - float(sp.y)
        center_dist = math.hypot(dx, dy)
        if center_dist <= 1e-3 or center_dist > self.range_cap_m + self.peer_radius_m:
            self.pub.publish(scan)
            return

        bearing = wrap_pi(math.atan2(dy, dx) - yaw)
        half_width = math.asin(min(0.95, self.peer_radius_m / max(center_dist, self.peer_radius_m)))
        hit_range = max(scan.range_min, min(self.range_cap_m, center_dist - self.peer_radius_m))
        i0 = max(0, int(math.floor((bearing - half_width - self.angle_min) / self.angle_increment)))
        i1 = min(self.count - 1, int(math.ceil((bearing + half_width - self.angle_min) / self.angle_increment)))
        for i in range(i0, i1 + 1):
            angle = self.angle_min + i * self.angle_increment
            if abs(wrap_pi(angle - bearing)) <= half_width:
                scan.ranges[i] = hit_range
        self.pub.publish(scan)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = PeerObstacleScanNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
