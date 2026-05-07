from __future__ import annotations

import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import dumps_compact, invert_se2, loads_dict, se2_from_xyyaw, wrap_pi, xyyaw_from_se2
from .robust_loop_selector import (
    RobustSelectionResult,
    RobustSelectorParams,
    VerifiedMatch,
    select_robust_inliers,
)


class RobustLoopSelectorNode(Node):
    """PCM-style robust selector over verified inter-robot loop matches."""

    def __init__(self) -> None:
        super().__init__("robust_loop_selector_node")
        self.declare_parameter("match_topic", "/team_slam/cross_robot_matches")
        self.declare_parameter("robust_inliers_topic", "/team_slam/robust_loop_inliers")
        self.declare_parameter("reference_robot", "robot_a")
        self.declare_parameter("target_robot", "robot_b")
        self.declare_parameter("parent_frame", "robot_a/map")
        self.declare_parameter("child_frame", "robot_b/map")
        self.declare_parameter("max_verified_matches", 240)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("team_alignment_min_matches", 2)
        self.declare_parameter("team_alignment_max_translation_disagreement", 1.0)
        self.declare_parameter("team_alignment_max_yaw_disagreement_deg", 10.0)
        self.declare_parameter("robust_min_inliers", 7)
        self.declare_parameter("robust_min_inlier_ratio", 0.25)
        self.declare_parameter("robust_max_median_rmse", 0.45)
        self.declare_parameter("robust_max_translation_spread_m", 1.0)
        self.declare_parameter("robust_max_yaw_spread_deg", 12.0)
        self.declare_parameter("robust_max_pairwise_rmse_disagreement", 0.35)
        self.declare_parameter("robust_min_pairwise_inlier_ratio", 0.0)
        self.declare_parameter("robust_min_pairwise_correspondences", 0)
        self.declare_parameter("robust_prefilter_max_rmse", 0.45)
        self.declare_parameter("robust_prefilter_min_inlier_ratio", 0.35)
        self.declare_parameter("robust_prefilter_min_correspondences", 0)
        self.declare_parameter("robust_prefilter_max_descriptor_distance", 0.45)
        self.declare_parameter("robust_deduplicate_by_query_keyframe", False)
        self.declare_parameter("robust_deduplicate_by_match_keyframe", False)
        self.declare_parameter("robust_deduplicate_transform_bin_translation_m", 0.0)
        self.declare_parameter("robust_deduplicate_transform_bin_yaw_deg", 0.0)

        self.match_topic = str(self.get_parameter("match_topic").value)
        self.out_topic = str(self.get_parameter("robust_inliers_topic").value)
        self.reference_robot = str(self.get_parameter("reference_robot").value).strip().strip("/")
        self.target_robot = str(self.get_parameter("target_robot").value).strip().strip("/")
        self.parent_frame = str(self.get_parameter("parent_frame").value).strip().strip("/")
        self.child_frame = str(self.get_parameter("child_frame").value).strip().strip("/")
        self.max_matches = int(self.get_parameter("max_verified_matches").value)
        backup_min = int(self.get_parameter("team_alignment_min_matches").value)
        robust_min = int(self.get_parameter("robust_min_inliers").value)
        self.params = RobustSelectorParams(
            translation_threshold_m=float(
                self.get_parameter("team_alignment_max_translation_disagreement").value
            ),
            yaw_threshold_rad=math.radians(
                float(self.get_parameter("team_alignment_max_yaw_disagreement_deg").value)
            ),
            robust_min_inliers=max(backup_min, robust_min),
            robust_min_inlier_ratio=float(self.get_parameter("robust_min_inlier_ratio").value),
            robust_max_median_rmse=float(self.get_parameter("robust_max_median_rmse").value),
            robust_max_translation_spread_m=float(
                self.get_parameter("robust_max_translation_spread_m").value
            ),
            robust_max_yaw_spread_rad=math.radians(
                float(self.get_parameter("robust_max_yaw_spread_deg").value)
            ),
            max_pairwise_rmse_disagreement=float(
                self.get_parameter("robust_max_pairwise_rmse_disagreement").value
            ),
            min_pairwise_inlier_ratio=float(
                self.get_parameter("robust_min_pairwise_inlier_ratio").value
            ),
            min_pairwise_correspondences=int(
                self.get_parameter("robust_min_pairwise_correspondences").value
            ),
            robust_prefilter_max_rmse=float(
                self.get_parameter("robust_prefilter_max_rmse").value
            ),
            robust_prefilter_min_inlier_ratio=float(
                self.get_parameter("robust_prefilter_min_inlier_ratio").value
            ),
            robust_prefilter_min_correspondences=int(
                self.get_parameter("robust_prefilter_min_correspondences").value
            ),
            robust_prefilter_max_descriptor_distance=float(
                self.get_parameter("robust_prefilter_max_descriptor_distance").value
            ),
            deduplicate_by_query_keyframe=bool(
                self.get_parameter("robust_deduplicate_by_query_keyframe").value
            ),
            deduplicate_by_match_keyframe=bool(
                self.get_parameter("robust_deduplicate_by_match_keyframe").value
            ),
            deduplicate_transform_bin_translation_m=float(
                self.get_parameter("robust_deduplicate_transform_bin_translation_m").value
            ),
            deduplicate_transform_bin_yaw_rad=math.radians(
                float(self.get_parameter("robust_deduplicate_transform_bin_yaw_deg").value)
            ),
        )

        self.verified: list[VerifiedMatch] = []
        self.seen_match_ids: set[str] = set()
        self.raw_rejected_count = 0
        self.last_result: RobustSelectionResult | None = None

        self.create_subscription(String, self.match_topic, self._on_match, 50)
        self.pub = self.create_publisher(String, self.out_topic, 10)
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "robust_loop_selector_node up: "
            f"matches={self.match_topic} out={self.out_topic} "
            f"robust_min_inliers={self.params.robust_min_inliers}"
        )

    def _stamp_sec(self) -> float:
        now = self.get_clock().now().to_msg()
        return float(now.sec) + float(now.nanosec) * 1e-9

    def _normalize_transform(self, payload: dict[str, Any]) -> np.ndarray | None:
        tf = payload.get("transform", {})
        parent = str(tf.get("parent_frame", "")).strip().strip("/")
        child = str(tf.get("child_frame", "")).strip().strip("/")
        try:
            t = se2_from_xyyaw(float(tf["x"]), float(tf["y"]), float(tf.get("yaw", 0.0)))
        except Exception:
            return None
        if parent == self.parent_frame and child == self.child_frame:
            return t
        if parent == self.child_frame and child == self.parent_frame:
            return invert_se2(t)
        return None

    @staticmethod
    def _match_id(payload: dict[str, Any]) -> str:
        raw = payload.get("match_id")
        if raw:
            return str(raw)
        return "|".join([
            str(payload.get("query_robot", payload.get("source_robot", ""))),
            str(payload.get("query_keyframe", payload.get("source_keyframe", ""))),
            str(payload.get("match_robot", payload.get("target_robot", ""))),
            str(payload.get("match_keyframe", payload.get("target_keyframe", ""))),
            str(payload.get("stamp_sec", "")),
        ])

    def _on_match(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if not payload or payload.get("schema") != "team_cross_robot_match/v1":
            return
        if not bool(payload.get("accepted", False)):
            self.raw_rejected_count += 1
            return
        if str(payload.get("stage", "")) == "descriptor_candidate":
            self.raw_rejected_count += 1
            return
        transform = self._normalize_transform(payload)
        if transform is None:
            return
        mid = self._match_id(payload)
        if mid in self.seen_match_ids:
            return
        self.seen_match_ids.add(mid)
        match = VerifiedMatch(
            match_id=mid,
            payload=payload,
            transform=transform,
            rmse=float(payload.get("rmse", payload.get("icp_fitness_m", payload.get("fitness", 999.0)))),
            fitness=float(payload.get("fitness", payload.get("icp_fitness_m", 999.0))),
            inlier_ratio=float(payload.get("inlier_ratio", payload.get("icp_inlier_ratio", 0.0))),
            num_correspondences=int(payload.get("num_correspondences", 0)),
            descriptor_score=float(payload.get("descriptor_score", payload.get("descriptor_distance", 999.0))),
        )
        self.verified.append(match)
        if len(self.verified) > self.max_matches:
            removed = self.verified[: len(self.verified) - self.max_matches]
            del self.verified[: len(self.verified) - self.max_matches]
            for old in removed:
                self.seen_match_ids.discard(old.match_id)

    @staticmethod
    def _match_summary(match: VerifiedMatch) -> dict[str, Any]:
        x, y, yaw = xyyaw_from_se2(match.transform)
        payload = match.payload
        return {
            "match_id": match.match_id,
            "query_robot": payload.get("query_robot", payload.get("source_robot", "")),
            "query_keyframe": payload.get("query_keyframe", payload.get("source_keyframe", "")),
            "match_robot": payload.get("match_robot", payload.get("target_robot", "")),
            "match_keyframe": payload.get("match_keyframe", payload.get("target_keyframe", "")),
            "source_robot": payload.get("source_robot", payload.get("query_robot", "")),
            "source_keyframe": payload.get("source_keyframe", payload.get("query_keyframe", "")),
            "target_robot": payload.get("target_robot", payload.get("match_robot", "")),
            "target_keyframe": payload.get("target_keyframe", payload.get("match_keyframe", "")),
            "rmse": round(float(match.rmse), 5),
            "fitness": round(float(match.fitness), 5),
            "inlier_ratio": round(float(match.inlier_ratio), 5),
            "num_correspondences": int(match.num_correspondences),
            "descriptor_score": round(float(match.descriptor_score), 5),
            "transform": {
                "x": round(float(x), 5),
                "y": round(float(y), 5),
                "yaw": round(float(wrap_pi(yaw)), 6),
                "parent_frame": str(payload.get("transform", {}).get("parent_frame", "")),
                "child_frame": str(payload.get("transform", {}).get("child_frame", "")),
            },
            "T_query_to_match": payload.get("T_query_to_match", []),
            "payload": payload,
        }

    def _payload_from_result(self, result: RobustSelectionResult) -> dict[str, Any]:
        status = "accepted" if result.accepted else ("tentative" if result.inliers else "unaligned")
        if not result.accepted and len(self.verified) >= self.params.robust_min_inliers:
            status = "rejected"
        confidence = 0.0
        if result.inliers:
            rmse_quality = 1.0 - min(1.0, result.median_rmse / max(1e-6, self.params.robust_max_median_rmse))
            confidence = max(0.0, min(1.0, 0.65 * result.inlier_ratio + 0.35 * rmse_quality))
        payload: dict[str, Any] = {
            "schema": "team_robust_loop_inliers/v1",
            "stamp_sec": round(self._stamp_sec(), 6),
            "status": status,
            "accepted": bool(result.accepted),
            "reason": result.reason,
            "reject_reason": "" if result.accepted else result.reason,
            "robust_inlier_set_size": result.inlier_count,
            "robust_inlier_ratio": round(float(result.inlier_ratio), 5),
            "robust_inlier_ratio_raw": round(float(result.inlier_ratio_raw), 5),
            "robust_inlier_ratio_eligible": round(float(result.inlier_ratio_eligible), 5),
            "raw_verified_matches": int(result.raw_verified_count),
            "prefiltered_matches": int(result.prefiltered_count),
            "eligible_matches": int(result.eligible_count),
            "deduplicated_matches": int(result.deduplicated_count),
            "raw_rejected_matches": self.raw_rejected_count,
            "raw_consistent_matches": int(result.raw_consistent_matches),
            "robust_rejected_matches": result.rejected_count,
            "median_rmse": round(float(result.median_rmse), 5) if math.isfinite(result.median_rmse) else 999.0,
            "median_descriptor_distance": round(float(result.median_descriptor_distance), 5)
            if math.isfinite(result.median_descriptor_distance) else 999.0,
            "median_inlier_ratio": round(float(result.median_inlier_ratio), 5),
            "translation_spread_m": round(float(result.translation_spread_m), 5)
            if math.isfinite(result.translation_spread_m) else 999.0,
            "transform_spread_translation": round(float(result.translation_spread_m), 5)
            if math.isfinite(result.translation_spread_m) else 999.0,
            "yaw_spread_deg": round(math.degrees(float(result.yaw_spread_rad)), 3)
            if math.isfinite(result.yaw_spread_rad) else 999.0,
            "transform_spread_yaw_deg": round(math.degrees(float(result.yaw_spread_rad)), 3)
            if math.isfinite(result.yaw_spread_rad) else 999.0,
            "alignment_confidence": round(confidence, 5),
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "gt_used_runtime": False,
            "inliers": [self._match_summary(m) for m in result.inliers],
            "rejected": [self._match_summary(m) for m in result.rejected],
        }
        if result.transform is not None:
            x, y, yaw = xyyaw_from_se2(result.transform)
            payload["transform"] = {
                "x": round(float(x), 5),
                "y": round(float(y), 5),
                "yaw": round(float(wrap_pi(yaw)), 6),
            }
        return payload

    def _tick(self) -> None:
        self.last_result = select_robust_inliers(list(self.verified), self.params)
        self.pub.publish(String(data=dumps_compact(self._payload_from_result(self.last_result))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobustLoopSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
