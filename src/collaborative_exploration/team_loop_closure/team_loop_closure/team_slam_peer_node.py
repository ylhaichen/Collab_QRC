from __future__ import annotations

import base64
import gzip
import json
import time
from dataclasses import dataclass
from typing import Any

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
except ModuleNotFoundError:  # Allows pure policy tests without sourcing ROS 2.
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[misc,assignment]
    PointCloud2 = Any  # type: ignore[misc,assignment]
    String = Any  # type: ignore[misc,assignment]

from .common import dumps_compact, loads_dict


@dataclass(frozen=True)
class PeerBridgePolicy:
    descriptor_only_until_candidate: bool = True
    send_cloud_only_on_candidate: bool = True
    peer_cloud_max_points: int = 2000


def build_peer_envelope(
    *,
    topic: str,
    robot_id: str,
    peer_robot_id: str,
    payload: dict[str, Any],
    compress: bool,
) -> dict[str, Any]:
    raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    encoded = gzip.compress(raw) if compress else raw
    return {
        "schema": "team_slam_peer_envelope/v1",
        "transport": "dds",
        "topic": topic,
        "source_robot": robot_id,
        "target_robot": peer_robot_id,
        "compressed": bool(compress),
        "payload_encoding": "base64+gzip+json" if compress else "base64+json",
        "payload_b64": base64.b64encode(encoded).decode("ascii"),
    }


def should_forward_keyframe_cloud(
    policy: PeerBridgePolicy,
    keyframe_id: str,
    candidate_keyframes: set[str],
) -> bool:
    if not policy.descriptor_only_until_candidate and not policy.send_cloud_only_on_candidate:
        return True
    return str(keyframe_id) in candidate_keyframes


class TeamSlamPeerNode(Node):
    """Bandwidth-aware DDS peer bridge for replicated team-SLAM nodes.

    DDS mode intentionally forwards normal ROS messages; the JSON envelope is
    emitted only on the diagnostics topic so local consumers can keep using the
    existing schemas.
    """

    def __init__(self) -> None:
        super().__init__("team_slam_peer_node")
        self.declare_parameter("robot_id", "robot_a")
        self.declare_parameter("peer_robot_id", "robot_b")
        self.declare_parameter("team_comm_mode", "dds")
        self.declare_parameter("peer_keyframe_rate_hz", 0.5)
        self.declare_parameter("peer_descriptor_only_until_candidate", True)
        self.declare_parameter("send_cloud_only_on_candidate", True)
        self.declare_parameter("peer_cloud_max_points", 2000)
        self.declare_parameter("compress_peer_json", True)

        self.robot_id = str(self.get_parameter("robot_id").value).strip().strip("/")
        self.peer_robot_id = str(self.get_parameter("peer_robot_id").value).strip().strip("/")
        self.mode = str(self.get_parameter("team_comm_mode").value).strip().lower() or "dds"
        self.compress = bool(self.get_parameter("compress_peer_json").value)
        self.rate_sec = 1.0 / max(0.05, float(self.get_parameter("peer_keyframe_rate_hz").value))
        self.policy = PeerBridgePolicy(
            descriptor_only_until_candidate=bool(
                self.get_parameter("peer_descriptor_only_until_candidate").value
            ),
            send_cloud_only_on_candidate=bool(self.get_parameter("send_cloud_only_on_candidate").value),
            peer_cloud_max_points=int(self.get_parameter("peer_cloud_max_points").value),
        )
        self.last_keyframe_sent = 0.0
        self.candidate_keyframes: set[str] = set()

        self.peer_keyframes_pub = self.create_publisher(String, "/team_slam/peer/keyframes", 10)
        self.peer_robust_pub = self.create_publisher(String, "/team_slam/peer/robust_loop_inliers", 10)
        self.peer_metrics_pub = self.create_publisher(String, "/team_slam/peer/pose_graph_metrics", 10)
        self.peer_cloud_pub = self.create_publisher(PointCloud2, "/team_slam/peer/keyframe_clouds", 10)
        self.envelope_pub = self.create_publisher(String, "/team_slam/peer/envelopes", 10)
        self.status_pub = self.create_publisher(String, "/team_slam/peer/status", 10)

        self.create_subscription(String, "/team_slam/local/keyframes", self._on_local_keyframe, 10)
        self.create_subscription(String, "/team_slam/local/robust_loop_inliers", self._on_local_robust, 10)
        self.create_subscription(String, "/team_slam/local/pose_graph_metrics", self._on_local_metrics, 10)
        self.create_subscription(String, "/team_slam/cross_robot_candidates", self._on_candidate, 20)
        self.create_subscription(PointCloud2, "/team_slam/local/keyframe_clouds", self._on_local_cloud, 10)

        self.create_timer(2.0, self._publish_status)
        if self.mode != "dds":
            self.get_logger().warn(
                f"team_comm_mode:={self.mode} requested; UDP JSON is not enabled in this build, "
                "falling back to DDS topic forwarding with explicit status."
            )
            self.mode = "dds"
        self.get_logger().info(
            f"team_slam_peer_node up: robot={self.robot_id} peer={self.peer_robot_id} mode={self.mode}"
        )

    def _stamp(self) -> float:
        now = self.get_clock().now().to_msg()
        return float(now.sec) + float(now.nanosec) * 1e-9

    def _emit_envelope(self, topic: str, payload: dict[str, Any]) -> None:
        env = build_peer_envelope(
            topic=topic,
            robot_id=self.robot_id,
            peer_robot_id=self.peer_robot_id,
            payload=payload,
            compress=self.compress,
        )
        env["stamp_sec"] = round(self._stamp(), 6)
        self.envelope_pub.publish(String(data=dumps_compact(env)))

    def _on_candidate(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if not payload:
            return
        for key in ("query_keyframe", "match_keyframe", "source_keyframe", "target_keyframe"):
            value = str(payload.get(key, ""))
            if value:
                self.candidate_keyframes.add(value)

    def _on_local_keyframe(self, msg: String) -> None:
        now = time.monotonic()
        if now - self.last_keyframe_sent < self.rate_sec:
            return
        payload = loads_dict(msg.data)
        if not payload:
            return
        robot = str(payload.get("robot", payload.get("robot_id", ""))).strip().strip("/")
        if robot and robot != self.robot_id:
            return
        self.last_keyframe_sent = now
        self._emit_envelope("/team_slam/local/keyframes", payload)
        self.peer_keyframes_pub.publish(msg)

    def _on_local_robust(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if payload:
            self._emit_envelope("/team_slam/local/robust_loop_inliers", payload)
        self.peer_robust_pub.publish(msg)

    def _on_local_metrics(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if payload:
            self._emit_envelope("/team_slam/local/pose_graph_metrics", payload)
        self.peer_metrics_pub.publish(msg)

    def _on_local_cloud(self, msg: PointCloud2) -> None:
        kid = str(msg.header.frame_id).split("/")[-1]
        if should_forward_keyframe_cloud(self.policy, kid, self.candidate_keyframes):
            self.peer_cloud_pub.publish(msg)

    def _publish_status(self) -> None:
        payload = {
            "schema": "team_slam_peer_status/v1",
            "stamp_sec": round(self._stamp(), 6),
            "robot_id": self.robot_id,
            "peer_robot_id": self.peer_robot_id,
            "team_comm_mode": self.mode,
            "candidate_keyframes": len(self.candidate_keyframes),
            "dependency_blocker": "" if self.mode == "dds" else "udp_json_not_enabled",
            "gt_used_runtime": False,
        }
        self.status_pub.publish(String(data=dumps_compact(payload)))


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run team_slam_peer_node")
    rclpy.init(args=args)
    node = TeamSlamPeerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
