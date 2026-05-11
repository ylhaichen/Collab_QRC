from __future__ import annotations

import json
from pathlib import Path
from typing import Any

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
except ModuleNotFoundError:
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[misc,assignment]
    PointCloud2 = Any  # type: ignore[misc,assignment]
    String = Any  # type: ignore[misc,assignment]

from .cleanup_contracts import (
    CleanupBackendStatus,
    RequiredCleanupExport,
    backend_available,
    build_cleanup_command,
)


class MapCleanupBackendNode(Node):
    """Publish asynchronous map-cleanup readiness and fallback metrics.

    External ERASOR/Removert execution is intentionally kept outside the
    odometry loop. This node reports command contracts and publishes fallback
    outputs when only temporal voxel cleanup is available.
    """

    def __init__(self) -> None:
        super().__init__("map_cleanup_backend_node")
        self.declare_parameter("static_map_cleanup_backend", "none")
        self.declare_parameter("fallback_backend", "temporal_voxel_fallback")
        self.declare_parameter("export_root", "logs/map_cleanup_export")
        self.declare_parameter("output_dir", "logs/map_cleanup_output")
        self.declare_parameter("input_static_map_topic", "/team_slam/static_map")
        self.backend = str(self.get_parameter("static_map_cleanup_backend").value).strip().lower()
        self.fallback = str(self.get_parameter("fallback_backend").value).strip().lower()
        self.export = RequiredCleanupExport.from_root(str(self.get_parameter("export_root").value))
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        available, blocker = backend_available(self.backend)
        self.status = CleanupBackendStatus(
            backend=self.backend,
            backend_available=available,
            fallback_backend=self.fallback,
            dependency_blocker=blocker,
        )
        self.cleaned_pub = self.create_publisher(PointCloud2, "/team_slam/cleaned_static_map", 5)
        self.removed_pub = self.create_publisher(PointCloud2, "/team_slam/removed_dynamic_points", 5)
        self.metrics_pub = self.create_publisher(String, "/team_slam/map_cleanup_metrics", 10)
        self.status_pub = self.create_publisher(String, "/team_slam/map_cleanup_status", 10)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_static_map_topic").value),
            self._on_static_map,
            5,
        )
        self.create_timer(2.0, self._publish_status)

    def _payload(self) -> dict[str, Any]:
        command = build_cleanup_command(self.status.selected_runtime_backend(), self.export, output_dir=self.output_dir)
        payload = self.status.to_payload()
        payload.update(
            {
                "schema": "team_map_cleanup_metrics/v1",
                "required_export_missing_paths": self.export.missing_paths(),
                "cleanup_command": command.argv,
                "realtime_odometry_loop": command.realtime_odometry_loop,
            }
        )
        return payload

    def _publish_status(self) -> None:
        payload = self._payload()
        self.metrics_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.status_pub.publish(String(data=json.dumps(self.status.to_payload(), sort_keys=True)))

    def _on_static_map(self, msg: PointCloud2) -> None:
        if self.status.selected_runtime_backend() == "temporal_voxel_fallback":
            self.cleaned_pub.publish(msg)
        self._publish_status()


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run map_cleanup_backend_node")
    rclpy.init(args=args)
    node = MapCleanupBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
