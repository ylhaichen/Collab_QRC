#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .common import atomic_write_json, clamp01, nearest_occupied_distance, now_sec_from_node, point_reachable_enough, wrap_pi


class LoopClosureCandidateNode(Node):
    """Generate revisit candidates from pose-graph health proxies."""

    def __init__(self) -> None:
        super().__init__("loop_closure_candidate_node")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("max_candidates", 20)
        self.declare_parameter("min_current_distance_m", 0.70)
        self.declare_parameter("candidate_time_gap_sec", 30.0)
        self.declare_parameter("candidate_path_gap_m", 6.0)
        self.declare_parameter("self_loop_radius_m", 1.8)
        self.declare_parameter("inter_robot_radius_m", 2.0)
        self.declare_parameter("reachability_clearance_m", 0.22)
        self.declare_parameter("travel_cost_scale_m", 12.0)
        self.declare_parameter("map_topic", "/merged_map")
        self.declare_parameter("inter_robot_enabled", True)
        self.declare_parameter("require_team_alignment_for_inter_robot", True)
        self.declare_parameter("alignment_status_topic", "/team_slam/alignment_status")

        self.output_path = str(self.get_parameter("output_path").value)
        self.max_candidates = int(self.get_parameter("max_candidates").value)
        self.min_current_distance_m = float(self.get_parameter("min_current_distance_m").value)
        self.candidate_time_gap_sec = float(self.get_parameter("candidate_time_gap_sec").value)
        self.candidate_path_gap_m = float(self.get_parameter("candidate_path_gap_m").value)
        self.self_loop_radius_m = float(self.get_parameter("self_loop_radius_m").value)
        self.inter_robot_radius_m = float(self.get_parameter("inter_robot_radius_m").value)
        self.reachability_clearance_m = float(self.get_parameter("reachability_clearance_m").value)
        self.travel_cost_scale_m = float(self.get_parameter("travel_cost_scale_m").value)
        self.inter_robot_enabled = bool(self.get_parameter("inter_robot_enabled").value)
        self.require_team_alignment = bool(
            self.get_parameter("require_team_alignment_for_inter_robot").value
        )
        self._team_aligned = False
        self._team_tf_parent = "robot_a/map"
        self._team_tf_child = "robot_b/map"
        self._team_tf = (0.0, 0.0, 0.0)

        self.health: dict[str, Any] = {}
        self.map_msg: OccupancyGrid | None = None
        self._pub = self.create_publisher(String, "/cfpa2/loop_candidates", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/cfpa2/loop_candidate_markers", 10
        )
        self.create_subscription(String, "/cfpa2/pose_graph_health", self._on_health, 10)
        self.create_subscription(
            String,
            str(self.get_parameter("alignment_status_topic").value),
            self._on_alignment_status,
            10,
        )
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"loop_closure_candidate_node up: max={self.max_candidates} output={self.output_path or '<none>'}"
        )

    def _on_health(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.health = payload

    def _on_alignment_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict) or payload.get("schema") != "team_alignment_status/v1":
            return
        self._team_aligned = str(payload.get("status", "")) == "aligned"
        self._team_tf_parent = str(payload.get("parent_frame", "robot_a/map")).strip().strip("/")
        self._team_tf_child = str(payload.get("child_frame", "robot_b/map")).strip().strip("/")
        tf = payload.get("transform", {})
        if isinstance(tf, dict):
            self._team_tf = (
                float(tf.get("x", 0.0)),
                float(tf.get("y", 0.0)),
                float(tf.get("yaw", 0.0)),
            )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _candidate_allowed_by_map(self, x: float, y: float) -> bool:
        if self.map_msg is None:
            return True
        return point_reachable_enough(self.map_msg, x, y, self.reachability_clearance_m)

    def _clearance_risk(self, x: float, y: float, robot: str) -> float:
        if self.map_msg is None:
            return 0.3
        clearance = nearest_occupied_distance(self.map_msg, x, y, 1.5)
        if not math.isfinite(clearance):
            return 0.0
        body_margin = 0.45 if robot == "robot_a" else 0.28
        return clamp01((body_margin - clearance) / max(body_margin, 0.05))

    def _score_candidate(
        self,
        *,
        target_robot: str,
        source: str,
        current: dict[str, Any],
        keyframe: dict[str, Any],
        robot_state: dict[str, Any],
        evidence_extra: list[str] | None = None,
        require_path_gap: bool = True,
    ) -> dict[str, Any] | None:
        cx = float(current["x"])
        cy = float(current["y"])
        cyaw = float(current.get("yaw", 0.0))
        kx = float(keyframe["x"])
        ky = float(keyframe["y"])
        kyaw = float(keyframe.get("yaw", 0.0))
        travel = math.hypot(kx - cx, ky - cy)
        if travel < self.min_current_distance_m:
            return None
        if not self._candidate_allowed_by_map(kx, ky):
            return None

        now = float(self.health.get("stamp_sec", now_sec_from_node(self)))
        age = max(0.0, now - float(keyframe.get("t_sec", now)))
        path_gap = max(0.0, float(robot_state.get("path_len_m", 0.0)) - float(keyframe.get("path_len_m", 0.0)))
        if age < self.candidate_time_gap_sec:
            return None
        if require_path_gap and path_gap < self.candidate_path_gap_m:
            return None

        proximity_score = clamp01(1.0 - min(travel, self.travel_cost_scale_m) / self.travel_cost_scale_m)
        heading_diversity = clamp01(abs(wrap_pi(cyaw - kyaw)) / math.pi)
        age_score = clamp01(age / 180.0)
        drift_yaw = float(robot_state.get("drift_yaw_peak_deg", 0.0))
        drift_trans = float(robot_state.get("drift_trans_peak_m", 0.0))
        drift_risk_score = clamp01(max(drift_yaw / 12.0, drift_trans / 0.30))
        inter_robot_score = 1.0 if source == "inter_robot_rendezvous" else 0.0
        travel_penalty = clamp01(travel / self.travel_cost_scale_m)

        loop_gain = (
            0.35 * proximity_score
            + 0.20 * heading_diversity
            + 0.15 * age_score
            + 0.20 * drift_risk_score
            + 0.20 * inter_robot_score
            - 0.25 * travel_penalty
        )
        loop_gain = clamp01(loop_gain)
        mobility_go2w = self._clearance_risk(kx, ky, "robot_a")
        mobility_go2 = self._clearance_risk(kx, ky, "robot_b")
        evidence = [str(keyframe.get("id", "keyframe_unknown"))]
        if evidence_extra:
            evidence.extend(evidence_extra)
        cid = f"loop_{target_robot}_{source}_{abs(hash((target_robot, source, round(kx, 2), round(ky, 2)))) % 100000:05d}"
        return {
            "id": cid,
            "role": "loop_close",
            "x": round(kx, 3),
            "y": round(ky, 3),
            "target_robot": target_robot,
            "source": source,
            "loop_gain": round(loop_gain, 4),
            "travel_cost_m": round(travel, 3),
            "mobility_risk_go2": round(mobility_go2, 4),
            "mobility_risk_go2w": round(mobility_go2w, 4),
            "evidence": evidence,
            "features": {
                "proximity": round(proximity_score, 4),
                "heading_diversity": round(heading_diversity, 4),
                "age": round(age_score, 4),
                "drift": round(drift_risk_score, 4),
                "inter_robot": round(inter_robot_score, 4),
                "path_gap_m": round(path_gap, 3),
            },
        }

    def _build_candidates(self) -> list[dict[str, Any]]:
        robots = self.health.get("robots", {})
        if not isinstance(robots, dict):
            return []
        out: list[dict[str, Any]] = []
        for ns, robot_state in robots.items():
            if not isinstance(robot_state, dict):
                continue
            current = robot_state.get("current_pose")
            if not isinstance(current, dict):
                continue
            keyframes = robot_state.get("keyframes", [])
            if not isinstance(keyframes, list):
                continue
            state = str(robot_state.get("state", "good"))
            distance_since_revisit = float(robot_state.get("distance_since_revisit_m", 0.0) or 0.0)
            drift_revisit_allowed = (
                state in ("drift_risk", "loop_needed", "weak")
                or distance_since_revisit >= self.candidate_path_gap_m
            )
            for keyframe in keyframes[:-1]:
                if not isinstance(keyframe, dict):
                    continue
                travel = math.hypot(float(current["x"]) - float(keyframe["x"]), float(current["y"]) - float(keyframe["y"]))
                source = "self_crossing" if travel <= self.self_loop_radius_m else "drift_revisit"
                if source == "drift_revisit" and not drift_revisit_allowed:
                    continue
                cand = self._score_candidate(
                    target_robot=ns,
                    source=source,
                    current=current,
                    keyframe=keyframe,
                    robot_state=robot_state,
                )
                if cand is not None:
                    out.append(cand)

            for other_ns, other_state in robots.items():
                if not self.inter_robot_enabled:
                    continue
                if self.require_team_alignment and not self._team_aligned:
                    continue
                if other_ns == ns or not isinstance(other_state, dict):
                    continue
                other_keyframes = other_state.get("keyframes", [])
                if not isinstance(other_keyframes, list):
                    continue
                for keyframe in other_keyframes[:-1]:
                    if not isinstance(keyframe, dict):
                        continue
                    kx, ky, kyaw = self._transform_between_robot_maps(
                        float(keyframe["x"]),
                        float(keyframe["y"]),
                        float(keyframe.get("yaw", 0.0)),
                        str(other_ns),
                        str(ns),
                    )
                    transformed_keyframe = dict(keyframe)
                    transformed_keyframe["x"] = kx
                    transformed_keyframe["y"] = ky
                    transformed_keyframe["yaw"] = kyaw
                    travel = math.hypot(float(current["x"]) - kx, float(current["y"]) - ky)
                    if travel > self.inter_robot_radius_m:
                        continue
                    cand = self._score_candidate(
                        target_robot=ns,
                        source="inter_robot_rendezvous",
                        current=current,
                        keyframe=transformed_keyframe,
                        robot_state=robot_state,
                        evidence_extra=[f"source_robot={other_ns}"],
                        require_path_gap=False,
                    )
                    if cand is not None:
                        out.append(cand)

        dedup: dict[tuple[str, int, int], dict[str, Any]] = {}
        for cand in out:
            key = (
                str(cand["target_robot"]),
                int(round(float(cand["x"]) / 0.5)),
                int(round(float(cand["y"]) / 0.5)),
            )
            old = dedup.get(key)
            if old is None or float(cand["loop_gain"]) > float(old["loop_gain"]):
                dedup[key] = cand
        return sorted(dedup.values(), key=lambda c: float(c["loop_gain"]), reverse=True)[: self.max_candidates]

    def _transform_between_robot_maps(
        self, x: float, y: float, yaw_in: float, source_ns: str, target_ns: str
    ) -> tuple[float, float, float]:
        if source_ns == target_ns:
            return x, y, yaw_in
        parent_robot = self._team_tf_parent.split("/")[0]
        child_robot = self._team_tf_child.split("/")[0]
        tx, ty, yaw = self._team_tf
        c = math.cos(yaw)
        s = math.sin(yaw)
        if source_ns == child_robot and target_ns == parent_robot:
            return (
                c * x - s * y + tx,
                s * x + c * y + ty,
                wrap_pi(yaw_in + yaw),
            )
        if source_ns == parent_robot and target_ns == child_robot:
            dx = x - tx
            dy = y - ty
            return (
                c * dx + s * dy,
                -s * dx + c * dy,
                wrap_pi(yaw_in - yaw),
            )
        return x, y, yaw_in

    def _payload(self) -> dict[str, Any]:
        candidates = self._build_candidates()
        return {
            "schema": "loop_candidates/v1",
            "stamp_sec": round(now_sec_from_node(self), 3),
            "candidate_count": len(candidates),
            "candidates": candidates,
        }

    def _publish_markers(self, candidates: list[dict[str, Any]]) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, cand in enumerate(candidates):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "loop_candidates"
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(cand["x"])
            m.pose.position.y = float(cand["y"])
            m.pose.position.z = 0.35
            scale = 0.25 + 0.35 * clamp01(float(cand.get("loop_gain", 0.0)))
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            if cand.get("source") == "inter_robot_rendezvous":
                m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.2, 1.0, 0.9
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.9
            markers.markers.append(m)
            txt = Marker()
            txt.header = m.header
            txt.ns = "loop_candidate_labels"
            txt.id = 1000 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = m.pose.position.x
            txt.pose.position.y = m.pose.position.y
            txt.pose.position.z = 0.75
            txt.scale.z = 0.22
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"{cand.get('target_robot')} loop={float(cand.get('loop_gain', 0.0)):.2f}"
            markers.markers.append(txt)
        self._marker_pub.publish(markers)

    def _tick(self) -> None:
        payload = self._payload()
        self._pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self._publish_markers(payload["candidates"])
        atomic_write_json(self.output_path, payload)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = LoopClosureCandidateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
