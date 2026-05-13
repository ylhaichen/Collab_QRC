from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .common import yaw_from_quat
from .occupancy_grid_utils import AlignmentSnapshot, SimpleOccupancyGrid, merge_local_grids_if_aligned


class MergedOccupancyGridNode(Node):
    def __init__(self) -> None:
        super().__init__("merged_occupancy_grid_node")
        self.declare_parameter("robot_a_grid_topic", "/robot_a/local_occupancy_grid")
        self.declare_parameter("robot_b_grid_topic", "/robot_b/local_occupancy_grid")
        self.declare_parameter("alignment_status_topic", "/team_slam/alignment_status")
        self.declare_parameter("relative_transform_topic", "/team_slam/relative_transform")
        self.declare_parameter("output_topic", "/team_slam/merged_occupancy_grid")
        self.declare_parameter("status_topic", "/team_slam/merged_occupancy_grid_status")
        self.declare_parameter("output_frame_id", "team_map")
        self.declare_parameter("publish_rate_hz", 1.0)

        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.alignment_status = "unaligned"
        self.gt_used_runtime = False
        self.relative_xyyaw: tuple[float, float, float] | None = None
        self.grid_a: OccupancyGrid | None = None
        self.grid_b: OccupancyGrid | None = None
        self.merged_map_enabled_time_sec: float | None = None

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("output_topic").value),
            qos_latched,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("robot_a_grid_topic").value),
            lambda msg: setattr(self, "grid_a", msg),
            qos_latched,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("robot_b_grid_topic").value),
            lambda msg: setattr(self, "grid_b", msg),
            qos_latched,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("alignment_status_topic").value),
            self._on_alignment,
            10,
        )
        self.create_subscription(
            TransformStamped,
            str(self.get_parameter("relative_transform_topic").value),
            self._on_transform,
            10,
        )
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "merged_occupancy_grid_node up: publishes only after robust aligned status and relative transform"
        )

    def _now_sec(self) -> float:
        msg = self.get_clock().now().to_msg()
        return float(msg.sec) + float(msg.nanosec) * 1e-9

    def _on_alignment(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {}
        if not isinstance(payload, dict):
            payload = {}
        self.alignment_status = str(payload.get("status", self.alignment_status)).lower()
        self.gt_used_runtime = bool(payload.get("gt_used_runtime", False))
        tf = payload.get("transform", {})
        if isinstance(tf, dict) and {"x", "y"}.issubset(tf):
            try:
                self.relative_xyyaw = (
                    float(tf.get("x", 0.0)),
                    float(tf.get("y", 0.0)),
                    float(tf.get("yaw", 0.0)),
                )
            except (TypeError, ValueError):
                pass

    def _on_transform(self, msg: TransformStamped) -> None:
        self.relative_xyyaw = (
            float(msg.transform.translation.x),
            float(msg.transform.translation.y),
            float(yaw_from_quat(msg.transform.rotation)),
        )

    def _tick(self) -> None:
        status_payload: dict[str, Any] = {
            "schema": "merged_occupancy_grid_status/v1",
            "alignment_status": self.alignment_status,
            "active": False,
            "gt_used_runtime": bool(self.gt_used_runtime),
            "has_robot_a_grid": self.grid_a is not None,
            "has_robot_b_grid": self.grid_b is not None,
            "has_relative_transform": self.relative_xyyaw is not None,
            "merged_map_enabled_time_sec": self.merged_map_enabled_time_sec,
        }
        if self.grid_a is None or self.grid_b is None:
            self.status_pub.publish(String(data=json.dumps(status_payload, sort_keys=True)))
            return
        merged = merge_local_grids_if_aligned(
            self._from_msg(self.grid_a),
            self._from_msg(self.grid_b),
            AlignmentSnapshot(
                status=self.alignment_status,
                gt_used_runtime=self.gt_used_runtime,
                transform_xyyaw=self.relative_xyyaw,
            ),
            output_frame_id=self.output_frame_id,
        )
        if merged is None:
            self.status_pub.publish(String(data=json.dumps(status_payload, sort_keys=True)))
            return
        if self.merged_map_enabled_time_sec is None:
            self.merged_map_enabled_time_sec = self._now_sec()
        out = self._to_msg(merged)
        self.pub.publish(out)
        status_payload["active"] = True
        status_payload["merged_map_enabled_time_sec"] = self.merged_map_enabled_time_sec
        self.status_pub.publish(String(data=json.dumps(status_payload, sort_keys=True)))

    @staticmethod
    def _from_msg(msg: OccupancyGrid) -> SimpleOccupancyGrid:
        return SimpleOccupancyGrid(
            frame_id=msg.header.frame_id,
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            data=[int(v) for v in msg.data],
        )

    def _to_msg(self, grid: SimpleOccupancyGrid) -> OccupancyGrid:
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = grid.frame_id
        out.info.resolution = float(grid.resolution)
        out.info.width = int(grid.width)
        out.info.height = int(grid.height)
        out.info.origin.position.x = float(grid.origin_x)
        out.info.origin.position.y = float(grid.origin_y)
        out.info.origin.orientation.w = 1.0
        out.data = list(grid.data)
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MergedOccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
