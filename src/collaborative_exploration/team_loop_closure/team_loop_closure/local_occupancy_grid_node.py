from __future__ import annotations

from collections import deque
from typing import Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from .common import yaw_from_quat
from .occupancy_grid_utils import (
    LocalGridSpec,
    OccupancyKeyframe,
    SimpleOccupancyGrid,
    build_static_grid_from_keyframes,
    project_static_points_to_grid,
)


class LocalOccupancyGridNode(Node):
    def __init__(self) -> None:
        super().__init__("local_occupancy_grid_node")
        self.declare_parameter("robot_namespace", "robot_a")
        self.declare_parameter("cloud_static_topic", "cloud_static")
        self.declare_parameter("fallback_cloud_topic", "cloud_registered_body")
        self.declare_parameter("odom_topic", "Odometry")
        self.declare_parameter("corrected_odom_topic", "corrected_odom")
        self.declare_parameter("output_topic", "local_occupancy_grid")
        self.declare_parameter("frame_id", "")
        self.declare_parameter("occupancy_resolution", 0.1)
        self.declare_parameter("occupancy_size_x", 40.0)
        self.declare_parameter("occupancy_size_y", 40.0)
        self.declare_parameter("occupancy_height_min", -0.2)
        self.declare_parameter("occupancy_height_max", 1.5)
        self.declare_parameter("occupancy_decay_sec", 0.0)
        self.declare_parameter("use_cloud_static", True)
        self.declare_parameter("cloud_static_stale_sec", 2.0)
        self.declare_parameter("max_points_per_cloud", 60000)
        self.declare_parameter("occupancy_use_corrected_pose", True)
        self.declare_parameter("occupancy_rebuild_from_keyframes", True)
        self.declare_parameter("occupancy_static_min_observations", 2)
        self.declare_parameter("occupancy_dynamic_decay_sec", 3.0)
        self.declare_parameter("occupancy_self_clear_radius", 0.45)
        self.declare_parameter("occupancy_max_keyframes", 200)
        self.declare_parameter("occupancy_rebuild_period_sec", 2.0)

        self.ns = str(self.get_parameter("robot_namespace").value).strip().strip("/") or "robot"
        self.frame_id = str(self.get_parameter("frame_id").value).strip() or f"{self.ns}/map"
        self.spec = LocalGridSpec(
            resolution=float(self.get_parameter("occupancy_resolution").value),
            size_x=float(self.get_parameter("occupancy_size_x").value),
            size_y=float(self.get_parameter("occupancy_size_y").value),
            height_min=float(self.get_parameter("occupancy_height_min").value),
            height_max=float(self.get_parameter("occupancy_height_max").value),
        )
        self.use_cloud_static = bool(self.get_parameter("use_cloud_static").value)
        self.cloud_static_stale_sec = max(0.0, float(self.get_parameter("cloud_static_stale_sec").value))
        self.max_points = max(1, int(self.get_parameter("max_points_per_cloud").value))
        self.use_corrected_pose = bool(self.get_parameter("occupancy_use_corrected_pose").value)
        self.rebuild_from_keyframes = bool(
            self.get_parameter("occupancy_rebuild_from_keyframes").value
        )
        self.static_min_observations = max(
            1,
            int(self.get_parameter("occupancy_static_min_observations").value),
        )
        self.dynamic_decay_sec = max(
            0.0,
            float(self.get_parameter("occupancy_dynamic_decay_sec").value),
        )
        self.self_clear_radius = max(
            0.0,
            float(self.get_parameter("occupancy_self_clear_radius").value),
        )
        self.max_keyframes = max(1, int(self.get_parameter("occupancy_max_keyframes").value))
        self.rebuild_period_sec = max(
            0.0,
            float(self.get_parameter("occupancy_rebuild_period_sec").value),
        )
        self.latest_raw_odom: Odometry | None = None
        self.latest_corrected_odom: Odometry | None = None
        self.static_last_sec: float | None = None
        self.grid = SimpleOccupancyGrid.empty(self.frame_id, self.spec)
        self.keyframes: deque[OccupancyKeyframe] = deque(maxlen=self.max_keyframes)
        self.keyframe_seq = 0
        self.last_rebuild_sec: float | None = None
        self.last_pose_source = "none"

        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        out_topic = self._ns_topic(str(self.get_parameter("output_topic").value))
        self.pub = self.create_publisher(OccupancyGrid, out_topic, qos_latched)
        self.create_subscription(
            PointCloud2,
            self._ns_topic(str(self.get_parameter("cloud_static_topic").value)),
            lambda msg: self._on_cloud(msg, source="static"),
            5,
        )
        self.create_subscription(
            PointCloud2,
            self._ns_topic(str(self.get_parameter("fallback_cloud_topic").value)),
            lambda msg: self._on_cloud(msg, source="fallback"),
            5,
        )
        self.create_subscription(
            Odometry,
            self._ns_topic(str(self.get_parameter("odom_topic").value)),
            lambda msg: self._set_odom(msg, corrected=False),
            20,
        )
        self.create_subscription(
            Odometry,
            self._ns_topic(str(self.get_parameter("corrected_odom_topic").value)),
            lambda msg: self._set_odom(msg, corrected=True),
            20,
        )
        self.get_logger().info(
            "local_occupancy_grid_node up: "
            f"ns={self.ns} output={out_topic} frame={self.frame_id} "
            f"resolution={self.spec.resolution:.3f} static_cloud={self.use_cloud_static} "
            f"corrected_pose={self.use_corrected_pose} rebuild_from_keyframes={self.rebuild_from_keyframes}"
        )

    def _ns_topic(self, suffix: str) -> str:
        clean = str(suffix or "").strip()
        if clean.startswith("/"):
            return clean
        return f"/{self.ns}/{clean.lstrip('/')}"

    def _now_sec(self) -> float:
        msg = self.get_clock().now().to_msg()
        return float(msg.sec) + float(msg.nanosec) * 1e-9

    def _set_odom(self, msg: Odometry, *, corrected: bool) -> None:
        if corrected:
            self.latest_corrected_odom = msg
        else:
            self.latest_raw_odom = msg

    def _latest_odom(self) -> tuple[Odometry | None, str]:
        if self.use_corrected_pose and self.latest_corrected_odom is not None:
            return self.latest_corrected_odom, "corrected_odom"
        if self.latest_raw_odom is not None:
            return self.latest_raw_odom, "Odometry"
        return None, "none"

    def _on_cloud(self, msg: PointCloud2, *, source: str) -> None:
        if source == "static":
            self.static_last_sec = self._now_sec()
        elif self.use_cloud_static and self.static_last_sec is not None:
            if self._now_sec() - self.static_last_sec <= self.cloud_static_stale_sec:
                return
        odom, pose_source = self._latest_odom()
        if odom is None:
            return
        points = self._cloud_points(msg)
        if not points:
            return
        if self.rebuild_from_keyframes:
            self._append_keyframe(points, odom, source=f"cloud_{source}", stamp_sec=self._now_sec())
            if self._should_rebuild_grid(pose_source):
                self._rebuild_and_publish(msg.header.stamp, pose_source=pose_source)
            return
        projected = project_static_points_to_grid(
            points_xyz=points,
            robot_pose_xyyaw=self._pose_xyyaw(odom),
            frame_id=self.frame_id,
            spec=self.spec,
        )
        for i, value in enumerate(projected.data):
            if value >= 50:
                self.grid.data[i] = 100
        out = self._to_msg(self.grid)
        out.header.stamp = msg.header.stamp
        self.pub.publish(out)

    def _append_keyframe(
        self,
        points: list[tuple[float, float, float]],
        odom: Odometry,
        *,
        source: str,
        stamp_sec: float,
    ) -> None:
        self.keyframes.append(
            OccupancyKeyframe(
                keyframe_id=f"{self.ns}_occ_kf_{self.keyframe_seq:06d}",
                corrected_pose_xyyaw=self._pose_xyyaw(odom),
                points_xyz=points,
                stamp_sec=float(stamp_sec),
                source=source,
            )
        )
        self.keyframe_seq += 1

    def _should_rebuild_grid(self, pose_source: str) -> bool:
        now = self._now_sec()
        if self.last_rebuild_sec is None:
            return True
        if self.last_pose_source != pose_source and pose_source == "corrected_odom":
            return True
        return now - self.last_rebuild_sec >= self.rebuild_period_sec

    def _rebuild_and_publish(self, stamp, *, pose_source: str) -> None:
        now = self._now_sec()
        keyframes = list(self.keyframes)
        if self.dynamic_decay_sec > 0.0:
            keyframes = [
                kf for kf in keyframes
                if kf.source == "cloud_static" or now - float(kf.stamp_sec) <= self.dynamic_decay_sec
            ]
        self.grid = build_static_grid_from_keyframes(
            keyframes,
            frame_id=self.frame_id,
            spec=self.spec,
            static_min_observations=self.static_min_observations,
            self_clear_radius=self.self_clear_radius,
        )
        out = self._to_msg(self.grid)
        out.header.stamp = stamp
        self.pub.publish(out)
        self.last_rebuild_sec = now
        self.last_pose_source = pose_source

    def _cloud_points(self, msg: PointCloud2) -> list[tuple[float, float, float]]:
        pts: list[tuple[float, float, float]] = []
        for idx, p in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if idx >= self.max_points:
                break
            pts.append((float(p[0]), float(p[1]), float(p[2])))
        return pts

    @staticmethod
    def _pose_xyyaw(msg: Odometry) -> tuple[float, float, float]:
        p = msg.pose.pose.position
        return (float(p.x), float(p.y), float(yaw_from_quat(msg.pose.pose.orientation)))

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
    node = LocalOccupancyGridNode()
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
