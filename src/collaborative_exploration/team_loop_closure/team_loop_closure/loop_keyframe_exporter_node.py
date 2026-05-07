from __future__ import annotations

import math
import time
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import String

from .common import (
    dumps_compact,
    pose_dict_from_msg,
    scan_context,
    scan_context_ring_key,
    scan_context_sector_key,
    stamp_to_sec,
    wrap_pi,
)


class LoopKeyframeExporter(Node):
    """Publish compact LiDAR keyframes for cross-robot place recognition."""

    def __init__(self) -> None:
        super().__init__("loop_keyframe_exporter_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("output_topic", "/team_slam/keyframes")
        self.declare_parameter("keyframe_cloud_topic", "/team_slam/keyframe_clouds")
        self.declare_parameter("raw_odom_topic", "Odometry")
        self.declare_parameter("corrected_odom_topic", "corrected_odom")
        self.declare_parameter("cloud_topic", "cloud_registered_body")
        self.declare_parameter("keyframe_distance_m", 1.5)
        self.declare_parameter("keyframe_yaw_deg", 20.0)
        self.declare_parameter("keyframe_min_translation", 1.0)
        self.declare_parameter("keyframe_min_yaw_deg", 10.0)
        self.declare_parameter("keyframe_min_time_sec", 1.0)
        self.declare_parameter("corrected_staleness_sec", 2.0)
        self.declare_parameter("corrected_max_raw_translation_delta_m", 25.0)
        self.declare_parameter("corrected_max_raw_yaw_delta_deg", 90.0)
        self.declare_parameter("min_points", 80)
        self.declare_parameter("max_points", 900)
        self.declare_parameter("descriptor_rings", 20)
        self.declare_parameter("descriptor_sectors", 60)
        self.declare_parameter("descriptor_max_radius_m", 18.0)
        self.declare_parameter("scan_context_num_rings", 20)
        self.declare_parameter("scan_context_num_sectors", 60)
        self.declare_parameter("scan_context_max_radius", 80.0)

        raw_ns = self.get_parameter("namespaces").value
        self.namespaces = [str(ns).strip().strip("/") for ns in raw_ns if str(ns).strip()]
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.keyframe_cloud_topic = str(self.get_parameter("keyframe_cloud_topic").value)
        self.raw_odom_topic = str(self.get_parameter("raw_odom_topic").value)
        self.corrected_odom_topic = str(self.get_parameter("corrected_odom_topic").value)
        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.keyframe_distance = float(self.get_parameter("keyframe_min_translation").value)
        if self.keyframe_distance <= 0.0:
            self.keyframe_distance = float(self.get_parameter("keyframe_distance_m").value)
        self.keyframe_yaw = math.radians(float(self.get_parameter("keyframe_min_yaw_deg").value))
        if self.keyframe_yaw <= 0.0:
            self.keyframe_yaw = math.radians(float(self.get_parameter("keyframe_yaw_deg").value))
        self.keyframe_min_time = float(self.get_parameter("keyframe_min_time_sec").value)
        self.corrected_staleness = float(self.get_parameter("corrected_staleness_sec").value)
        self.corrected_max_raw_translation_delta = float(
            self.get_parameter("corrected_max_raw_translation_delta_m").value
        )
        self.corrected_max_raw_yaw_delta = math.radians(
            float(self.get_parameter("corrected_max_raw_yaw_delta_deg").value)
        )
        self.min_points = int(self.get_parameter("min_points").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.rings = int(self.get_parameter("scan_context_num_rings").value)
        self.sectors = int(self.get_parameter("scan_context_num_sectors").value)
        self.max_radius = float(self.get_parameter("scan_context_max_radius").value)
        if self.rings <= 0:
            self.rings = int(self.get_parameter("descriptor_rings").value)
        if self.sectors <= 0:
            self.sectors = int(self.get_parameter("descriptor_sectors").value)
        if self.max_radius <= 0.0:
            self.max_radius = float(self.get_parameter("descriptor_max_radius_m").value)

        self._raw_pose: dict[str, dict[str, Any]] = {}
        self._corrected_pose: dict[str, dict[str, Any]] = {}
        self._last_keyframe_pose: dict[str, dict[str, float]] = {}
        self._last_keyframe_wall_sec: dict[str, float] = {}
        self._counts: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self._subs = []
        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, self.keyframe_cloud_topic, 10)

        for ns in self.namespaces:
            self._subs.append(self.create_subscription(
                Odometry, f"/{ns}/{self.raw_odom_topic.lstrip('/')}",
                lambda msg, n=ns: self._on_odom(n, msg, corrected=False), 20))
            self._subs.append(self.create_subscription(
                Odometry, f"/{ns}/{self.corrected_odom_topic.lstrip('/')}",
                lambda msg, n=ns: self._on_odom(n, msg, corrected=True), 20))
            self._subs.append(self.create_subscription(
                PointCloud2, f"/{ns}/{self.cloud_topic.lstrip('/')}",
                lambda msg, n=ns: self._on_cloud(n, msg), 5))

        self.get_logger().info(
            "loop_keyframe_exporter_node up: "
            f"robots={self.namespaces} output={self.output_topic} "
            f"clouds={self.keyframe_cloud_topic} scan_context={self.rings}x{self.sectors}"
        )

    @staticmethod
    def _mono() -> float:
        return time.monotonic()

    def _on_odom(self, ns: str, msg: Odometry, *, corrected: bool) -> None:
        pose = pose_dict_from_msg(msg)
        pose["stamp_sec"] = stamp_to_sec(msg.header.stamp)
        pose["wall_sec"] = self._mono()
        if corrected:
            self._corrected_pose[ns] = pose
        else:
            self._raw_pose[ns] = pose

    def _current_pose(self, ns: str) -> tuple[dict[str, float] | None, str]:
        corrected = self._corrected_pose.get(ns)
        raw = self._raw_pose.get(ns)
        if (
            corrected is not None
            and self._mono() - float(corrected["wall_sec"]) <= self.corrected_staleness
            and self._corrected_is_sane(corrected, raw)
        ):
            return corrected, "corrected_odom"
        return (raw, "Odometry") if raw is not None else (None, "none")

    def _corrected_is_sane(
        self,
        corrected: dict[str, float],
        raw: dict[str, float] | None,
    ) -> bool:
        values = [corrected.get(k, float("nan")) for k in ("x", "y", "yaw")]
        if not all(math.isfinite(float(v)) for v in values):
            return False
        if raw is None:
            return True
        dx = float(corrected["x"]) - float(raw["x"])
        dy = float(corrected["y"]) - float(raw["y"])
        dyaw = abs(wrap_pi(float(corrected["yaw"]) - float(raw["yaw"])))
        if math.hypot(dx, dy) > self.corrected_max_raw_translation_delta:
            return False
        if dyaw > self.corrected_max_raw_yaw_delta:
            return False
        return True

    def _cloud_to_points(self, msg: PointCloud2) -> np.ndarray:
        pts: list[tuple[float, float, float]] = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            r2 = x * x + y * y
            if 0.04 <= r2 <= self.max_radius * self.max_radius and -2.0 <= z <= 3.0:
                pts.append((x, y, z))
        if not pts:
            return np.empty((0, 3), dtype=np.float32)
        if len(pts) > self.max_points:
            stride = max(1, len(pts) // self.max_points)
            pts = pts[::stride][: self.max_points]
        return np.asarray(pts, dtype=np.float32)

    def _should_publish(self, ns: str, pose: dict[str, float]) -> bool:
        last = self._last_keyframe_pose.get(ns)
        if last is None:
            return True
        last_wall = self._last_keyframe_wall_sec.get(ns, 0.0)
        if self.keyframe_min_time > 0.0 and self._mono() - last_wall < self.keyframe_min_time:
            return False
        dx = float(pose["x"]) - float(last["x"])
        dy = float(pose["y"]) - float(last["y"])
        dyaw = abs(wrap_pi(float(pose["yaw"]) - float(last["yaw"])))
        return math.hypot(dx, dy) >= self.keyframe_distance or dyaw >= self.keyframe_yaw

    def _on_cloud(self, ns: str, msg: PointCloud2) -> None:
        pose, pose_source = self._current_pose(ns)
        if pose is None or not self._should_publish(ns, pose):
            return
        points = self._cloud_to_points(msg)
        if int(points.shape[0]) < self.min_points:
            return
        desc = scan_context(points, rings=self.rings, sectors=self.sectors, max_radius=self.max_radius)
        ring_key = scan_context_ring_key(desc)
        sector_key = scan_context_sector_key(desc)
        kid = f"{ns}_kf_{self._counts[ns]:06d}"
        self._counts[ns] += 1
        self._last_keyframe_pose[ns] = dict(pose)
        self._last_keyframe_wall_sec[ns] = self._mono()
        cloud_header = Header()
        cloud_header.stamp = msg.header.stamp
        cloud_header.frame_id = f"team_slam_keyframe_cloud/{ns}/{kid}"
        cloud_msg = point_cloud2.create_cloud_xyz32(cloud_header, np.round(points, 3).tolist())
        self.cloud_pub.publish(cloud_msg)
        payload = {
            "schema": "team_loop_keyframe/v1",
            "id": kid,
            "keyframe_id": self._counts[ns] - 1,
            "robot": ns,
            "robot_id": ns,
            "stamp_sec": round(stamp_to_sec(msg.header.stamp), 6),
            "cloud_frame_id": msg.header.frame_id,
            "pose_source": pose_source,
            "descriptor_type": "scan_context",
            "pose": {
                "x": round(float(pose["x"]), 4),
                "y": round(float(pose["y"]), 4),
                "z": round(float(pose.get("z", 0.0)), 4),
                "yaw": round(float(pose["yaw"]), 6),
            },
            "descriptor": {
                "type": "scan_context",
                "rings": self.rings,
                "sectors": self.sectors,
                "max_radius_m": self.max_radius,
                "values": np.round(desc.reshape(-1), 5).tolist(),
                "scan_context": np.round(desc.reshape(-1), 5).tolist(),
                "ring_key": np.round(ring_key, 5).tolist(),
                "sector_key": np.round(sector_key, 5).tolist(),
            },
            "scan_context": np.round(desc.reshape(-1), 5).tolist(),
            "ring_key": np.round(ring_key, 5).tolist(),
            "sector_key": np.round(sector_key, 5).tolist(),
            "compact_cloud_topic": self.keyframe_cloud_topic,
            "compact_cloud_key": kid,
            "compact_cloud_frame_id": cloud_header.frame_id,
            "cloud": {
                "frame": "body",
                "point_count": int(points.shape[0]),
            },
        }
        self.pub.publish(String(data=dumps_compact(payload)))
        if self._counts[ns] == 1 or self._counts[ns] % 10 == 0:
            self.get_logger().info(
                f"published {kid} robot={ns} points={points.shape[0]} pose_source={pose_source}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoopKeyframeExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
