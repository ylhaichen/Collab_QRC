from __future__ import annotations

import json
import math
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .frame_transform import Pose2D, split_original_points_by_labels, transform_body_points_to_odom
from .temporal_voxel_filter import DynamicFilterParams, Point3, TemporalVoxelFilter


def _stamp_to_sec(msg) -> float:
    return float(msg.sec) + float(msg.nanosec) * 1e-9


def _cloud_points(msg: PointCloud2) -> list[Point3]:
    return [
        (float(p[0]), float(p[1]), float(p[2]))
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    ]


class DynamicObstacleFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_filter_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("dynamic_filter_enabled", True)
        self.declare_parameter("dynamic_voxel_size", 0.25)
        self.declare_parameter("dynamic_static_min_observations", 3)
        self.declare_parameter("dynamic_static_min_lifetime_sec", 2.0)
        self.declare_parameter("dynamic_decay_time_sec", 5.0)
        self.declare_parameter("dynamic_obstacle_ttl_sec", 2.0)
        self.declare_parameter("dynamic_max_static_velocity", 0.15)
        self.declare_parameter("dynamic_min_dynamic_velocity", 0.35)
        self.declare_parameter("dynamic_near_robot_ignore_radius", 0.4)
        self.declare_parameter("dynamic_track_new_voxel_motion", False)
        self.declare_parameter("dynamic_publish_debug_clouds", True)
        self.declare_parameter("input_cloud_topic", "cloud_registered_body")
        self.declare_parameter("odom_topic", "corrected_odom")
        self.declare_parameter("raw_odom_topic", "Odometry")
        self.declare_parameter("classify_in_odom_frame", True)

        raw_namespaces = self.get_parameter("namespaces").value
        self.namespaces = [str(ns).strip().strip("/") for ns in raw_namespaces if str(ns).strip()]
        self.enabled = bool(self.get_parameter("dynamic_filter_enabled").value)
        self.debug_clouds = bool(self.get_parameter("dynamic_publish_debug_clouds").value)
        self.input_cloud_topic = str(self.get_parameter("input_cloud_topic").value).strip().strip("/")
        self.odom_topic = str(self.get_parameter("odom_topic").value).strip().strip("/")
        self.raw_odom_topic = str(self.get_parameter("raw_odom_topic").value).strip().strip("/")
        self.classify_in_odom_frame = bool(self.get_parameter("classify_in_odom_frame").value)
        params = DynamicFilterParams(
            voxel_size=float(self.get_parameter("dynamic_voxel_size").value),
            static_min_observations=int(self.get_parameter("dynamic_static_min_observations").value),
            static_min_lifetime_sec=float(self.get_parameter("dynamic_static_min_lifetime_sec").value),
            decay_time_sec=float(self.get_parameter("dynamic_decay_time_sec").value),
            dynamic_obstacle_ttl_sec=float(self.get_parameter("dynamic_obstacle_ttl_sec").value),
            max_static_velocity=float(self.get_parameter("dynamic_max_static_velocity").value),
            min_dynamic_velocity=float(self.get_parameter("dynamic_min_dynamic_velocity").value),
            near_robot_ignore_radius=float(self.get_parameter("dynamic_near_robot_ignore_radius").value),
            track_new_voxel_motion=bool(self.get_parameter("dynamic_track_new_voxel_motion").value),
        )
        self.filters = {ns: TemporalVoxelFilter(params) for ns in self.namespaces}
        self.latest_corrected_odom: dict[str, Odometry] = {}
        self.latest_raw_odom: dict[str, Odometry] = {}

        self.static_pubs = {
            ns: self.create_publisher(PointCloud2, f"/{ns}/cloud_static", 5) for ns in self.namespaces
        }
        self.dynamic_pubs = {
            ns: self.create_publisher(PointCloud2, f"/{ns}/cloud_dynamic", 5) for ns in self.namespaces
        }
        self.mask_pubs = {
            ns: self.create_publisher(String, f"/{ns}/dynamic_obstacle_mask", 5) for ns in self.namespaces
        }
        self.marker_pubs = {
            ns: self.create_publisher(MarkerArray, f"/{ns}/dynamic_voxel_markers", 5)
            for ns in self.namespaces
        }
        self.metrics_pub = self.create_publisher(String, "/team_slam/dynamic_filter_metrics", 10)
        self.robot_metrics_pubs = {
            ns: self.create_publisher(String, f"/{ns}/dynamic_filter_metrics", 10)
            for ns in self.namespaces
        }

        for ns in self.namespaces:
            self.create_subscription(
                PointCloud2,
                f"/{ns}/{self.input_cloud_topic}",
                lambda msg, n=ns: self._on_cloud(n, msg),
                5,
            )
            self.create_subscription(
                Odometry,
                f"/{ns}/{self.odom_topic}",
                lambda msg, n=ns: self.latest_corrected_odom.__setitem__(n, msg),
                5,
            )
            if self.raw_odom_topic and self.raw_odom_topic != self.odom_topic:
                self.create_subscription(
                    Odometry,
                    f"/{ns}/{self.raw_odom_topic}",
                    lambda msg, n=ns: self.latest_raw_odom.__setitem__(n, msg),
                    5,
                )
        self.get_logger().info(
            "dynamic_obstacle_filter_node up: "
            f"robots={self.namespaces} enabled={self.enabled} "
            f"classify_in_odom_frame={self.classify_in_odom_frame}"
        )

    def _publish_cloud(self, pub, header: Header, points: Iterable[Point3]) -> None:
        out_header = Header()
        out_header.stamp = header.stamp
        out_header.frame_id = header.frame_id
        pub.publish(point_cloud2.create_cloud_xyz32(out_header, list(points)))

    def _publish_markers(self, ns: str, msg: PointCloud2, dynamic_points: list[Point3]) -> None:
        markers = MarkerArray()
        marker = Marker()
        marker.header = msg.header
        marker.ns = f"{ns}_dynamic_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.0
        marker.color.a = 0.55
        for x, y, z in dynamic_points[:2000]:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)
        markers.markers.append(marker)
        self.marker_pubs[ns].publish(markers)

    @staticmethod
    def _pose_from_odom(msg: Odometry) -> Pose2D:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        return Pose2D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=math.atan2(siny_cosp, cosy_cosp),
        )

    def _latest_odom(self, ns: str) -> tuple[Odometry | None, str]:
        corrected = self.latest_corrected_odom.get(ns)
        if corrected is not None:
            return corrected, "corrected_odom"
        raw = self.latest_raw_odom.get(ns)
        if raw is not None:
            return raw, "Odometry"
        return None, "none"

    def _classification_points(self, ns: str, points: list[Point3]) -> tuple[list[Point3], str]:
        if not self.classify_in_odom_frame:
            return points, "body_disabled"
        odom, source = self._latest_odom(ns)
        if odom is None:
            return points, "body_no_odom"
        return transform_body_points_to_odom(points, self._pose_from_odom(odom)), source

    def _on_cloud(self, ns: str, msg: PointCloud2) -> None:
        stamp_sec = _stamp_to_sec(msg.header.stamp)
        points = _cloud_points(msg)
        if not self.enabled:
            result_static = points
            result_dynamic: list[Point3] = []
            ratio = 0.0
        else:
            classification_points, classification_frame = self._classification_points(ns, points)
            result = self.filters[ns].classify_points(classification_points, stamp_sec=stamp_sec)
            result_static, result_dynamic = split_original_points_by_labels(points, result.labels)
            ratio = result.dynamic_filter_ratio
        self._publish_cloud(self.static_pubs[ns], msg.header, result_static)
        if self.debug_clouds:
            self._publish_cloud(self.dynamic_pubs[ns], msg.header, result_dynamic)
            self._publish_markers(ns, msg, result_dynamic)
        mask = {
            "schema": "dynamic_obstacle_mask/v1",
            "robot_id": ns,
            "stamp_sec": round(stamp_sec, 6),
            "dynamic_points_filtered": len(result_dynamic),
            "static_points_kept": len(result_static),
            "dynamic_filter_ratio": round(float(ratio), 5),
            "dynamic_voxel_count": self.filters[ns].dynamic_voxel_count,
            "classification_frame": classification_frame if self.enabled else "disabled",
            "gt_used_runtime": False,
        }
        self.mask_pubs[ns].publish(String(data=json.dumps(mask, sort_keys=True)))
        metrics = dict(mask)
        metrics["schema"] = "team_dynamic_filter_metrics/v1"
        metrics_msg = String(data=json.dumps(metrics, sort_keys=True))
        self.metrics_pub.publish(metrics_msg)
        self.robot_metrics_pubs[ns].publish(metrics_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
