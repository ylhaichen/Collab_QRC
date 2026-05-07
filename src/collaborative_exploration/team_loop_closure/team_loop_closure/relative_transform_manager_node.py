from __future__ import annotations

import math
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from .common import dumps_compact, loads_dict, quat_from_yaw, se2_from_xyyaw, wrap_pi, xyyaw_from_se2
from .swarm_loop_agreement import (
    SwarmLoopAgreementResult,
    evaluate_swarm_loop_agreement,
    se2_from_transform_msg,
)


class RelativeTransformManager(Node):
    """Final safety gate for discovered inter-robot map-frame alignment."""

    def __init__(self) -> None:
        super().__init__("relative_transform_manager_node")
        self.declare_parameter("robust_inliers_topic", "/team_slam/robust_loop_inliers")
        self.declare_parameter("pose_graph_metrics_topic", "/team_slam/pose_graph_metrics")
        self.declare_parameter("status_topic", "/team_slam/alignment_status")
        self.declare_parameter("relative_transform_topic", "/team_slam/relative_transform")
        self.declare_parameter("parent_frame", "robot_a/map")
        self.declare_parameter("child_frame", "robot_b/map")
        self.declare_parameter("team_alignment_allow_export_only_gate", False)
        self.declare_parameter("require_swarm_loop_agreement", False)
        self.declare_parameter(
            "swarm_loop_relative_transform_topic",
            "/team_slam/swarm_lio2_relative_transform",
        )
        self.declare_parameter("swarm_loop_agreement_max_translation", 0.5)
        self.declare_parameter("swarm_loop_agreement_max_yaw_deg", 5.0)
        self.declare_parameter("alignment_reject_timeout_sec", 60.0)
        self.declare_parameter("alignment_reject_min_verified_matches", 7)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("publish_tf", False)

        self.robust_topic = str(self.get_parameter("robust_inliers_topic").value)
        self.metrics_topic = str(self.get_parameter("pose_graph_metrics_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.relative_topic = str(self.get_parameter("relative_transform_topic").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value).strip().strip("/")
        self.child_frame = str(self.get_parameter("child_frame").value).strip().strip("/")
        self.allow_export_only = bool(
            self.get_parameter("team_alignment_allow_export_only_gate").value
        )
        self.require_swarm_agreement = bool(
            self.get_parameter("require_swarm_loop_agreement").value
        )
        self.swarm_relative_topic = str(
            self.get_parameter("swarm_loop_relative_transform_topic").value
        )
        self.swarm_max_translation = float(
            self.get_parameter("swarm_loop_agreement_max_translation").value
        )
        self.swarm_max_yaw_rad = math.radians(
            float(self.get_parameter("swarm_loop_agreement_max_yaw_deg").value)
        )
        self.reject_timeout_sec = float(self.get_parameter("alignment_reject_timeout_sec").value)
        self.reject_min_verified_matches = int(
            self.get_parameter("alignment_reject_min_verified_matches").value
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.robust_payload: dict[str, Any] | None = None
        self.metrics_payload: dict[str, Any] | None = None
        self.swarm_transform: np.ndarray | None = None
        self.swarm_agreement: SwarmLoopAgreementResult | None = None
        self.first_robust_stamp_sec: float | None = None
        self.status = "unaligned"
        self.last_reason = "waiting_for_robust_loop_inliers"
        self.current_transform: np.ndarray | None = None

        self.create_subscription(String, self.robust_topic, self._on_robust, 10)
        self.create_subscription(String, self.metrics_topic, self._on_metrics, 10)
        self.create_subscription(
            TransformStamped,
            self.swarm_relative_topic,
            self._on_swarm_relative_transform,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.tf_pub = self.create_publisher(TransformStamped, self.relative_topic, 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "relative_transform_manager_node up as robust PGO gate: "
            f"{self.parent_frame} -> {self.child_frame}"
        )

    def _on_robust(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if payload and payload.get("schema") == "team_robust_loop_inliers/v1":
            self.robust_payload = payload
            if self.first_robust_stamp_sec is None:
                try:
                    self.first_robust_stamp_sec = float(payload.get("stamp_sec", 0.0))
                except (TypeError, ValueError):
                    self.first_robust_stamp_sec = None

    def _on_metrics(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if payload and payload.get("schema") == "team_pose_graph_metrics/v1":
            self.metrics_payload = payload

    def _on_swarm_relative_transform(self, msg: TransformStamped) -> None:
        self.swarm_transform = se2_from_transform_msg(msg)

    def _transform_from_robust(self) -> np.ndarray | None:
        payload = self.robust_payload or {}
        tf = payload.get("transform", {})
        try:
            return se2_from_xyyaw(float(tf["x"]), float(tf["y"]), float(tf.get("yaw", 0.0)))
        except Exception:
            return None

    def _update_status(self) -> None:
        robust = self.robust_payload or {}
        metrics = self.metrics_payload or {}
        self.current_transform = self._transform_from_robust()
        self.swarm_agreement = None
        robust_accepted = bool(robust.get("accepted", False)) and robust.get("status") == "accepted"
        gt_used = bool(robust.get("gt_used_runtime", False)) or bool(metrics.get("gt_used_runtime", False))
        backend = str(metrics.get("optimization_backend", "unknown"))
        optimization_success = bool(metrics.get("optimization_success", False))
        export_only_allowed = self.allow_export_only and backend == "g2o_export_only"
        graph_ready = optimization_success or export_only_allowed

        if not robust:
            self.status = "unaligned"
            self.last_reason = "waiting_for_robust_loop_inliers"
        elif not robust_accepted:
            robust_status = str(robust.get("status", "tentative"))
            raw_verified = int(robust.get("raw_verified_matches", 0) or 0)
            now_msg = self.get_clock().now().to_msg()
            now_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
            elapsed = 0.0
            if self.first_robust_stamp_sec is not None:
                elapsed = max(0.0, now_sec - self.first_robust_stamp_sec)
            enough_evidence = raw_verified >= self.reject_min_verified_matches
            timed_out = self.reject_timeout_sec > 0.0 and elapsed >= self.reject_timeout_sec
            if robust_status == "rejected" or enough_evidence or timed_out:
                self.status = "rejected"
                self.last_reason = str(
                    robust.get("reject_reason")
                    or robust.get("reason")
                    or "insufficient_robust_consensus"
                )
                if self.last_reason in (
                    "",
                    "robust_inliers_not_accepted",
                    "insufficient_robust_inliers",
                    "no_eligible_verified_matches",
                ):
                    self.last_reason = "insufficient_robust_consensus"
            else:
                self.status = "tentative"
                self.last_reason = str(
                    robust.get("reject_reason")
                    or robust.get("reason")
                    or "robust_inliers_not_accepted"
                )
        elif gt_used:
            self.status = "rejected"
            self.last_reason = "gt_used_runtime_forbidden"
        elif not metrics:
            self.status = "tentative"
            self.last_reason = "waiting_for_team_pose_graph_metrics"
        elif not graph_ready:
            self.status = "tentative"
            self.last_reason = (
                "pose_graph_export_only_gate_disabled"
                if backend == "g2o_export_only"
                else "pose_graph_optimization_not_successful"
            )
        elif self.require_swarm_agreement and self.current_transform is None:
            self.status = "tentative"
            self.last_reason = "waiting_for_team_loop_transform"
        elif self.require_swarm_agreement and self.swarm_transform is None:
            self.status = "tentative"
            self.last_reason = "waiting_for_swarm_lio2_relative_transform"
        else:
            if self.require_swarm_agreement:
                self.swarm_agreement = evaluate_swarm_loop_agreement(
                    self.swarm_transform,
                    self.current_transform,
                    max_translation_m=self.swarm_max_translation,
                    max_yaw_rad=self.swarm_max_yaw_rad,
                )
                if not self.swarm_agreement.accepted:
                    self.status = "rejected"
                    self.last_reason = self.swarm_agreement.reason
                    return
            self.status = "aligned"
            self.last_reason = (
                "robust_alignment_pose_graph_and_swarm_agreement_accepted"
                if self.require_swarm_agreement
                else (
                    "robust_alignment_and_pose_graph_accepted"
                    if optimization_success
                    else "robust_alignment_export_only_gate_accepted"
                )
            )

    def _transform_msg(self, transform: np.ndarray) -> TransformStamped:
        x, y, yaw = xyyaw_from_se2(transform)
        qx, qy, qz, qw = quat_from_yaw(yaw)
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.parent_frame
        out.child_frame_id = self.child_frame
        out.transform.translation.x = x
        out.transform.translation.y = y
        out.transform.translation.z = 0.0
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        return out

    def _tick(self) -> None:
        self._update_status()
        now_msg = self.get_clock().now().to_msg()
        now_sec = float(now_msg.sec) + float(now_msg.nanosec) * 1e-9
        robust = self.robust_payload or {}
        metrics = self.metrics_payload or {}
        confidence = float(robust.get("alignment_confidence", 0.0) or 0.0)
        payload: dict[str, Any] = {
            "schema": "team_alignment_status/v1",
            "stamp_sec": round(now_sec, 6),
            "status": self.status,
            "confidence": round(confidence, 4) if self.status == "aligned" else 0.0,
            "inlier_count": int(robust.get("robust_inlier_set_size", 0) or 0),
            "accepted_count": int(robust.get("raw_verified_matches", 0) or 0),
            "rejected_count": int(robust.get("robust_rejected_matches", 0) or 0),
            "reason": self.last_reason,
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "robust_status": robust.get("status", "unknown"),
            "robust_inlier_ratio": robust.get("robust_inlier_ratio", 0.0),
            "robust_inlier_ratio_raw": robust.get("robust_inlier_ratio_raw", 0.0),
            "robust_inlier_ratio_eligible": robust.get("robust_inlier_ratio_eligible", 0.0),
            "eligible_matches": int(robust.get("eligible_matches", 0) or 0),
            "deduplicated_matches": int(robust.get("deduplicated_matches", 0) or 0),
            "reject_reason": robust.get("reject_reason", ""),
            "pose_graph_backend": metrics.get("optimization_backend", "unknown"),
            "pose_graph_optimization_success": bool(metrics.get("optimization_success", False)),
            "swarm_loop_agreement_required": self.require_swarm_agreement,
            "swarm_loop_agreement_accepted": (
                bool(self.swarm_agreement.accepted) if self.swarm_agreement is not None else False
            ),
            "swarm_loop_agreement_reason": (
                self.swarm_agreement.reason if self.swarm_agreement is not None else ""
            ),
            "swarm_loop_translation_error_m": (
                round(float(self.swarm_agreement.translation_error_m), 5)
                if self.swarm_agreement is not None else None
            ),
            "swarm_loop_yaw_error_deg": (
                round(float(self.swarm_agreement.yaw_error_deg), 5)
                if self.swarm_agreement is not None else None
            ),
            "gt_used_runtime": bool(robust.get("gt_used_runtime", False))
            or bool(metrics.get("gt_used_runtime", False)),
        }
        if self.current_transform is not None:
            x, y, yaw = xyyaw_from_se2(self.current_transform)
            payload["transform"] = {
                "x": round(float(x), 5),
                "y": round(float(y), 5),
                "yaw": round(float(wrap_pi(yaw)), 6),
            }
            if self.status == "aligned":
                tf_msg = self._transform_msg(self.current_transform)
                self.tf_pub.publish(tf_msg)
                if self.tf_br is not None:
                    self.tf_br.sendTransform(tf_msg)
        self.status_pub.publish(String(data=dumps_compact(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeTransformManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
