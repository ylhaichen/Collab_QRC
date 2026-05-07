#!/usr/bin/env python3
"""Convert ROS 1 SC-PGO PoseStamped output into corrected Odometry.

The vendored ROS 1 FAST-LIO-SAM/SC-PGO node publishes its continuously
corrected pose on `/pose_stamped`. Its `/corrected_odom` topic is a
PointCloud2 trajectory visualization, not nav_msgs/Odometry. The ROS 2
fast_lio_tf_adapter expects `/<ns>/corrected_odom` to be Odometry, so this
small adapter bridges the semantic gap after ros1_bridge has carried
`/<ns>/sc_pgo/pose_stamped` into the ROS 2 graph.
"""
from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class ScPgoPoseToOdomAdapter(Node):
    def __init__(self) -> None:
        super().__init__("scpgo_pose_to_odom_adapter")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("pose_topic", "sc_pgo/pose_stamped")
        self.declare_parameter("raw_odom_topic", "Odometry")
        self.declare_parameter("output_topic", "corrected_odom")
        self.declare_parameter("child_frame_id", "")
        self.declare_parameter("raw_odom_staleness_sec", 1.0)

        ns = str(self.get_parameter("namespace").value).strip().strip("/")
        self.ns = ns or "robot"
        self.raw_staleness = float(self.get_parameter("raw_odom_staleness_sec").value)
        self.child_frame_override = str(self.get_parameter("child_frame_id").value).strip()

        def qualify(topic: str) -> str:
            topic = topic.strip()
            return topic if topic.startswith("/") else f"/{self.ns}/{topic}"

        self.pose_topic = qualify(str(self.get_parameter("pose_topic").value))
        self.raw_odom_topic = qualify(str(self.get_parameter("raw_odom_topic").value))
        self.output_topic = qualify(str(self.get_parameter("output_topic").value))

        self.latest_raw: Odometry | None = None
        self.latest_raw_wall_t = 0.0
        self.pose_count = 0

        self.create_subscription(Odometry, self.raw_odom_topic, self._on_raw_odom, 20)
        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 20)
        self.pub = self.create_publisher(Odometry, self.output_topic, 20)

        self.get_logger().info(
            "scpgo_pose_to_odom_adapter up: "
            f"{self.pose_topic} + {self.raw_odom_topic} -> {self.output_topic}"
        )

    @staticmethod
    def _now_wall() -> float:
        return time.monotonic()

    def _on_raw_odom(self, msg: Odometry) -> None:
        self.latest_raw = msg
        self.latest_raw_wall_t = self._now_wall()

    def _fresh_raw(self) -> Odometry | None:
        if self.latest_raw is None:
            return None
        if self._now_wall() - self.latest_raw_wall_t > self.raw_staleness:
            return None
        return self.latest_raw

    def _on_pose(self, msg: PoseStamped) -> None:
        raw = self._fresh_raw()
        out = Odometry()
        out.header = msg.header
        out.pose.pose = msg.pose

        if raw is not None:
            if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
                out.header.stamp = raw.header.stamp
            if not out.header.frame_id:
                out.header.frame_id = raw.header.frame_id
            out.child_frame_id = raw.child_frame_id
            out.pose.covariance = raw.pose.covariance
            out.twist = raw.twist
        else:
            out.child_frame_id = "body"

        if self.child_frame_override:
            out.child_frame_id = self.child_frame_override

        self.pub.publish(out)
        self.pose_count += 1
        if self.pose_count == 1 or self.pose_count % 100 == 0:
            p = out.pose.pose.position
            self.get_logger().info(
                f"published {self.pose_count} corrected odom msgs "
                f"pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) "
                f"frame={out.header.frame_id or '<empty>'} child={out.child_frame_id}"
            )


def main(argv=None) -> None:
    _user_argv, ros_argv = _split_ros_argv(sys.argv if argv is None else argv)
    rclpy.init(args=ros_argv)
    node = ScPgoPoseToOdomAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
