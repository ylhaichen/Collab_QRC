from __future__ import annotations

import math
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

from .common import (
    descriptor_distance_and_shift,
    dumps_compact,
    invert_se2,
    loads_dict,
    ring_key_distance,
    scan_context_ring_key,
    sector_shift_to_yaw,
    se2_from_xyyaw,
    wrap_pi,
    xyyaw_from_se2,
)
from .registration_backend import register_keyframe_clouds


class CrossRobotLoopMatcher(Node):
    """Descriptor retrieval plus self-contained geometric verification."""

    def __init__(self) -> None:
        super().__init__("cross_robot_loop_matcher_node")
        self.declare_parameter("keyframe_topic", "/team_slam/keyframes")
        self.declare_parameter("keyframe_cloud_topic", "/team_slam/keyframe_clouds")
        self.declare_parameter("additional_keyframe_topics", [])
        self.declare_parameter("additional_keyframe_cloud_topics", [])
        self.declare_parameter("candidate_topic", "/team_slam/cross_robot_candidates")
        self.declare_parameter("match_topic", "/team_slam/cross_robot_matches")
        self.declare_parameter("reference_robot", "robot_a")
        self.declare_parameter("target_robot", "robot_b")
        self.declare_parameter("max_keyframes_per_robot", 160)
        self.declare_parameter("ring_key_top_k", 5)
        self.declare_parameter("descriptor_candidate_threshold", 0.55)
        self.declare_parameter("descriptor_accept_threshold", 0.42)
        self.declare_parameter("registration_backend", "icp_2d")
        self.declare_parameter("registration_max_points", 260)
        self.declare_parameter("registration_max_iterations", 18)
        self.declare_parameter("registration_max_corr_dist", 0.75)
        self.declare_parameter("registration_max_fitness", 0.45)
        self.declare_parameter("registration_min_inlier_ratio", 0.35)
        self.declare_parameter("yaw_search_sectors", 2)

        self.keyframe_topic = str(self.get_parameter("keyframe_topic").value)
        self.keyframe_cloud_topic = str(self.get_parameter("keyframe_cloud_topic").value)
        self.additional_keyframe_topics = [
            str(t) for t in self.get_parameter("additional_keyframe_topics").value if str(t).strip()
        ]
        self.additional_keyframe_cloud_topics = [
            str(t) for t in self.get_parameter("additional_keyframe_cloud_topics").value if str(t).strip()
        ]
        self.candidate_topic = str(self.get_parameter("candidate_topic").value)
        self.match_topic = str(self.get_parameter("match_topic").value)
        self.reference_robot = str(self.get_parameter("reference_robot").value).strip().strip("/")
        self.target_robot = str(self.get_parameter("target_robot").value).strip().strip("/")
        self.max_keyframes = int(self.get_parameter("max_keyframes_per_robot").value)
        self.ring_top_k = int(self.get_parameter("ring_key_top_k").value)
        self.candidate_thr = float(self.get_parameter("descriptor_candidate_threshold").value)
        self.desc_thr = float(self.get_parameter("descriptor_accept_threshold").value)
        self.registration_backend = str(self.get_parameter("registration_backend").value)
        self.registration_max_points = int(self.get_parameter("registration_max_points").value)
        self.registration_iters = int(self.get_parameter("registration_max_iterations").value)
        self.registration_corr = float(self.get_parameter("registration_max_corr_dist").value)
        self.registration_fitness_thr = float(self.get_parameter("registration_max_fitness").value)
        self.registration_inlier_thr = float(self.get_parameter("registration_min_inlier_ratio").value)
        self.yaw_search_sectors = int(self.get_parameter("yaw_search_sectors").value)

        self.keyframes: dict[str, list[dict[str, Any]]] = {}
        self.clouds: dict[str, np.ndarray] = {}
        self.pending_by_cloud: dict[str, dict[str, Any]] = {}
        self.seen_pairs: set[tuple[str, str]] = set()
        self.create_subscription(String, self.keyframe_topic, self._on_keyframe, 10)
        self.create_subscription(PointCloud2, self.keyframe_cloud_topic, self._on_keyframe_cloud, 10)
        for topic in self.additional_keyframe_topics:
            self.create_subscription(String, topic, self._on_keyframe, 10)
        for topic in self.additional_keyframe_cloud_topics:
            self.create_subscription(PointCloud2, topic, self._on_keyframe_cloud, 10)
        self.candidate_pub = self.create_publisher(String, self.candidate_topic, 10)
        self.match_pub = self.create_publisher(String, self.match_topic, 10)
        self.get_logger().info(
            "cross_robot_loop_matcher_node up: "
            f"keyframes={self.keyframe_topic} clouds={self.keyframe_cloud_topic} "
            f"candidates={self.candidate_topic} matches={self.match_topic} "
            f"backend={self.registration_backend}"
        )

    def _stamp_sec(self) -> float:
        now = self.get_clock().now().to_msg()
        return float(now.sec) + float(now.nanosec) * 1e-9

    @staticmethod
    def _descriptor(kf: dict[str, Any]) -> np.ndarray:
        d = kf.get("descriptor", {})
        rings = int(d.get("rings", 0))
        sectors = int(d.get("sectors", 0))
        values = d.get("scan_context", d.get("values", kf.get("scan_context", [])))
        if rings <= 0 or sectors <= 0 or len(values) != rings * sectors:
            return np.empty((0, 0), dtype=np.float32)
        return np.asarray(values, dtype=np.float32).reshape(rings, sectors)

    @classmethod
    def _ring_key(cls, kf: dict[str, Any]) -> np.ndarray:
        d = kf.get("descriptor", {})
        values = d.get("ring_key", kf.get("ring_key", []))
        if values:
            return np.asarray(values, dtype=np.float32)
        return scan_context_ring_key(cls._descriptor(kf))

    def _cloud_xy(self, kf: dict[str, Any]) -> np.ndarray:
        kid = str(kf.get("id", kf.get("compact_cloud_key", "")))
        pts = self.clouds.get(kid)
        if pts is None:
            pts = np.asarray(kf.get("cloud", {}).get("points_xyz", []), dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 2:
            return np.empty((0, 2), dtype=np.float32)
        xy = pts[:, :2]
        if xy.shape[0] > self.registration_max_points:
            stride = max(1, xy.shape[0] // self.registration_max_points)
            xy = xy[::stride][: self.registration_max_points]
        return np.asarray(xy, dtype=np.float64)

    @staticmethod
    def _pose_se2(kf: dict[str, Any]) -> np.ndarray:
        p = kf.get("pose", {})
        return se2_from_xyyaw(float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0)))

    def _relative_transform_reference_to_target(
        self,
        *,
        query: dict[str, Any],
        match: dict[str, Any],
        match_from_query_body: np.ndarray,
    ) -> tuple[str, str, np.ndarray]:
        query_robot = str(query["robot"])
        match_robot = str(match["robot"])
        t_match_map_match_body = self._pose_se2(match)
        t_query_map_query_body = self._pose_se2(query)
        t_match_map_query_map = (
            t_match_map_match_body
            @ match_from_query_body
            @ invert_se2(t_query_map_query_body)
        )
        parent = f"{match_robot}/map"
        child = f"{query_robot}/map"
        if parent == f"{self.reference_robot}/map" and child == f"{self.target_robot}/map":
            return parent, child, t_match_map_query_map
        if parent == f"{self.target_robot}/map" and child == f"{self.reference_robot}/map":
            return f"{self.reference_robot}/map", f"{self.target_robot}/map", invert_se2(t_match_map_query_map)
        return parent, child, t_match_map_query_map

    def _publish_candidate(
        self,
        *,
        query: dict[str, Any],
        match: dict[str, Any],
        ring_distance: float,
        descriptor_distance: float,
        descriptor_shift: int,
        yaw_prior: float,
    ) -> None:
        payload = {
            "schema": "team_cross_robot_candidate/v1",
            "stage": "descriptor_candidate",
            "stamp_sec": round(self._stamp_sec(), 6),
            "query_robot": str(query["robot"]),
            "query_keyframe": str(query["id"]),
            "match_robot": str(match["robot"]),
            "match_keyframe": str(match["id"]),
            "ring_key_distance": round(float(ring_distance), 5),
            "descriptor_distance": round(float(descriptor_distance), 5),
            "descriptor_score": round(float(descriptor_distance), 5),
            "descriptor_shift": int(descriptor_shift),
            "yaw_shift_deg": round(math.degrees(yaw_prior), 3),
        }
        self.candidate_pub.publish(String(data=dumps_compact(payload)))

    def _publish_match(
        self,
        *,
        query: dict[str, Any],
        match: dict[str, Any],
        accepted: bool,
        reason: str,
        descriptor_distance: float,
        descriptor_shift: int,
        yaw_prior: float,
        registration_transform: np.ndarray,
        registration_fitness: float,
        inlier_ratio: float,
        registration_backend: str,
    ) -> None:
        parent, child, rel = self._relative_transform_reference_to_target(
            query=query,
            match=match,
            match_from_query_body=registration_transform,
        )
        x, y, yaw = xyyaw_from_se2(rel)
        ix, iy, iyaw = xyyaw_from_se2(registration_transform)
        payload = {
            "schema": "team_cross_robot_match/v1",
            "stamp_sec": round(self._stamp_sec(), 6),
            "accepted": bool(accepted),
            "reason": reason,
            "reject_reason": "" if accepted else reason,
            "stage": "geometrically_verified" if accepted else "geometrically_rejected",
            "query_keyframe": str(query["id"]),
            "query_robot": str(query["robot"]),
            "match_keyframe": str(match["id"]),
            "match_robot": str(match["robot"]),
            "source_keyframe": str(query["id"]),
            "source_robot": str(query["robot"]),
            "target_keyframe": str(match["id"]),
            "target_robot": str(match["robot"]),
            "descriptor_distance": round(float(descriptor_distance), 5),
            "descriptor_score": round(float(descriptor_distance), 5),
            "descriptor_shift": int(descriptor_shift),
            "yaw_shift_deg": round(math.degrees(yaw_prior), 3),
            "registration_backend": registration_backend,
            "fitness": round(float(registration_fitness), 5) if math.isfinite(registration_fitness) else 999.0,
            "inlier_ratio": round(float(inlier_ratio), 5),
            "rmse": round(float(registration_fitness), 5) if math.isfinite(registration_fitness) else 999.0,
            "num_correspondences": int(max(0, round(inlier_ratio * float(max(1, self._cloud_xy(query).shape[0]))))),
            "icp_fitness_m": round(float(registration_fitness), 5) if math.isfinite(registration_fitness) else 999.0,
            "icp_inlier_ratio": round(float(inlier_ratio), 5),
            "icp_target_from_source": {
                "x": round(ix, 5),
                "y": round(iy, 5),
                "yaw": round(wrap_pi(iyaw), 6),
            },
            "transform": {
                "parent_frame": parent,
                "child_frame": child,
                "x": round(x, 5),
                "y": round(y, 5),
                "yaw": round(wrap_pi(yaw), 6),
            },
            "T_query_map_match_map": [
                round(float(x), 5),
                round(float(y), 5),
                round(float(wrap_pi(yaw)), 6),
            ],
            "T_query_to_match": [
                round(float(ix), 5),
                round(float(iy), 5),
                round(float(wrap_pi(iyaw)), 6),
            ],
        }
        self.match_pub.publish(String(data=dumps_compact(payload)))
        if accepted:
            self.get_logger().info(
                f"accepted cross match {query['id']} -> {match['id']} "
                f"desc={descriptor_distance:.3f} fitness={registration_fitness:.3f} "
                f"inliers={inlier_ratio:.2f}"
            )

    def _candidate_pool(self, query: dict[str, Any]) -> list[tuple[float, dict[str, Any]]]:
        query_key = self._ring_key(query)
        ranked: list[tuple[float, dict[str, Any]]] = []
        for other_robot, other_keyframes in self.keyframes.items():
            if other_robot == str(query["robot"]):
                continue
            for other in other_keyframes:
                d = ring_key_distance(query_key, self._ring_key(other))
                if math.isfinite(d):
                    ranked.append((d, other))
        ranked.sort(key=lambda item: item[0])
        return ranked[: max(1, self.ring_top_k)]

    def _match_pair(self, query: dict[str, Any], match: dict[str, Any], ring_distance: float) -> None:
        qid = str(query["id"])
        mid = str(match["id"])
        pair = tuple(sorted((qid, mid)))
        if pair in self.seen_pairs:
            return
        self.seen_pairs.add(pair)

        qd = self._descriptor(query)
        md = self._descriptor(match)
        descriptor_distance, shift = descriptor_distance_and_shift(md, qd)
        if not math.isfinite(descriptor_distance):
            return
        sectors = max(1, int(query.get("descriptor", {}).get("sectors", qd.shape[1] if qd.ndim == 2 else 1)))
        yaw_prior = sector_shift_to_yaw(shift, sectors)
        if descriptor_distance > self.candidate_thr:
            return

        self._publish_candidate(
            query=query,
            match=match,
            ring_distance=ring_distance,
            descriptor_distance=descriptor_distance,
            descriptor_shift=shift,
            yaw_prior=yaw_prior,
        )

        query_xy = self._cloud_xy(query)
        match_xy = self._cloud_xy(match)
        result = register_keyframe_clouds(
            query_xy,
            match_xy,
            backend=self.registration_backend,
            initial_yaw=yaw_prior,
            yaw_search_sectors=self.yaw_search_sectors,
            sector_count=sectors,
            max_iterations=self.registration_iters,
            max_corr_dist_m=self.registration_corr,
        )
        accepted = (
            descriptor_distance <= self.desc_thr
            and result.fitness_m <= self.registration_fitness_thr
            and result.inlier_ratio >= self.registration_inlier_thr
        )
        if accepted:
            reason = "accepted"
        elif descriptor_distance > self.desc_thr:
            reason = "descriptor_below_threshold"
        elif result.fitness_m > self.registration_fitness_thr:
            reason = "registration_fitness_below_threshold"
        else:
            reason = "registration_inlier_ratio_below_threshold"
        self._publish_match(
            query=query,
            match=match,
            accepted=accepted,
            reason=reason,
            descriptor_distance=descriptor_distance,
            descriptor_shift=shift,
            yaw_prior=yaw_prior,
            registration_transform=result.transform,
            registration_fitness=result.fitness_m,
            inlier_ratio=result.inlier_ratio,
            registration_backend=result.backend,
        )

    def _try_match_keyframe(self, kf: dict[str, Any]) -> None:
        kid = str(kf.get("id", ""))
        if kid not in self.clouds and not kf.get("cloud", {}).get("points_xyz"):
            self.pending_by_cloud[kid] = kf
            return
        for ring_distance, other in self._candidate_pool(kf):
            oid = str(other.get("id", ""))
            if oid not in self.clouds and not other.get("cloud", {}).get("points_xyz"):
                continue
            self._match_pair(kf, other, ring_distance)

    def _on_keyframe_cloud(self, msg: PointCloud2) -> None:
        parts = str(msg.header.frame_id).split("/")
        if len(parts) < 3 or parts[0] != "team_slam_keyframe_cloud":
            return
        kid = parts[2]
        pts = np.asarray(
            [
                (float(p[0]), float(p[1]), float(p[2]))
                for p in point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            ],
            dtype=np.float32,
        )
        if kid and pts.ndim == 2 and pts.shape[1] >= 3:
            self.clouds[kid] = pts[:, :3]
            pending = self.pending_by_cloud.pop(kid, None)
            if pending is not None:
                self._try_match_keyframe(pending)

    def _on_keyframe(self, msg: String) -> None:
        kf = loads_dict(msg.data)
        if not kf or kf.get("schema") != "team_loop_keyframe/v1":
            return
        robot = str(kf.get("robot", kf.get("robot_id", ""))).strip().strip("/")
        kid = str(kf.get("id", ""))
        if not robot or not kid:
            return
        kf["robot"] = robot
        existing = self.keyframes.setdefault(robot, [])
        self._try_match_keyframe(kf)
        existing.append(kf)
        if len(existing) > self.max_keyframes:
            removed = existing[: len(existing) - self.max_keyframes]
            del existing[: len(existing) - self.max_keyframes]
            for old in removed:
                self.clouds.pop(str(old.get("id", "")), None)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrossRobotLoopMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
