#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .common import atomic_write_json, now_sec_from_node, wrap_pi, yaw_from_quat


@dataclass
class Keyframe:
    keyframe_id: str
    robot: str
    t_sec: float
    x: float
    y: float
    yaw: float
    path_len_m: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.keyframe_id,
            "robot": self.robot,
            "t_sec": round(self.t_sec, 3),
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 4),
            "path_len_m": round(self.path_len_m, 3),
        }


@dataclass
class RobotState:
    ns: str
    keyframes: list[Keyframe] = field(default_factory=list)
    current_xy: tuple[float, float] | None = None
    current_yaw: float = 0.0
    current_t_sec: float = 0.0
    path_len_m: float = 0.0
    last_odom_xy: tuple[float, float] | None = None
    raw_anchor: tuple[float, float, float] | None = None
    gt_anchor: tuple[float, float, float] | None = None
    latest_gt: tuple[float, float, float] | None = None
    latest_corrected: tuple[float, float, float] | None = None
    corrected_delta_trans_m: float = 0.0
    corrected_delta_yaw_deg: float = 0.0
    drift_trans_m: float = 0.0
    drift_yaw_deg: float = 0.0
    drift_trans_peak_m: float = 0.0
    drift_yaw_peak_deg: float = 0.0
    last_loop_candidate_t_sec: float = 0.0
    self_crossings: int = 0
    inter_robot_opportunities: int = 0


class PoseGraphHealthNode(Node):
    """Proxy pose-graph health monitor.

    This is deliberately independent of SC-PGO. It turns odom history into
    keyframes, loop-revisit opportunities, and drift-risk proxies that CFPA2
    can consume before a true ROS 2 PGO port exists.
    """

    def __init__(self) -> None:
        super().__init__("pose_graph_health_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("keyframe_distance_m", 0.75)
        self.declare_parameter("keyframe_yaw_deg", 20.0)
        self.declare_parameter("candidate_time_gap_sec", 30.0)
        self.declare_parameter("candidate_path_gap_m", 6.0)
        self.declare_parameter("loop_candidate_radius_m", 1.5)
        self.declare_parameter("inter_robot_radius_m", 2.0)
        self.declare_parameter("distance_since_revisit_warn_m", 18.0)
        self.declare_parameter("drift_yaw_warn_deg", 8.0)
        self.declare_parameter("drift_trans_warn_m", 0.20)
        self.declare_parameter("max_keyframes_per_robot", 240)

        raw_namespaces = self.get_parameter("namespaces").value
        self.namespaces = [str(ns).strip().strip("/") for ns in raw_namespaces if str(ns).strip()]
        self.output_path = str(self.get_parameter("output_path").value)
        self.keyframe_distance_m = float(self.get_parameter("keyframe_distance_m").value)
        self.keyframe_yaw_rad = math.radians(float(self.get_parameter("keyframe_yaw_deg").value))
        self.candidate_time_gap_sec = float(self.get_parameter("candidate_time_gap_sec").value)
        self.candidate_path_gap_m = float(self.get_parameter("candidate_path_gap_m").value)
        self.loop_candidate_radius_m = float(self.get_parameter("loop_candidate_radius_m").value)
        self.inter_robot_radius_m = float(self.get_parameter("inter_robot_radius_m").value)
        self.distance_since_revisit_warn_m = float(
            self.get_parameter("distance_since_revisit_warn_m").value
        )
        self.drift_yaw_warn_deg = float(self.get_parameter("drift_yaw_warn_deg").value)
        self.drift_trans_warn_m = float(self.get_parameter("drift_trans_warn_m").value)
        self.max_keyframes = int(self.get_parameter("max_keyframes_per_robot").value)

        self.states = {ns: RobotState(ns=ns) for ns in self.namespaces}
        self._pub = self.create_publisher(String, "/cfpa2/pose_graph_health", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/cfpa2/pose_graph_health_markers", 10
        )
        for ns in self.namespaces:
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._on_odom(m, n), 20)
            self.create_subscription(Odometry, f"/{ns}/odom/ground_truth", lambda m, n=ns: self._on_gt(m, n), 20)
            self.create_subscription(Odometry, f"/{ns}/corrected_odom", lambda m, n=ns: self._on_corrected(m, n), 10)

        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "pose_graph_health_node up: "
            f"robots={self.namespaces} keyframe={self.keyframe_distance_m:.2f}m/"
            f"{math.degrees(self.keyframe_yaw_rad):.0f}deg output={self.output_path or '<none>'}"
        )

    def _pose_tuple(self, msg: Odometry) -> tuple[float, float, float]:
        return (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quat(msg.pose.pose.orientation),
        )

    def _on_gt(self, msg: Odometry, ns: str) -> None:
        self.states[ns].latest_gt = self._pose_tuple(msg)

    def _on_corrected(self, msg: Odometry, ns: str) -> None:
        st = self.states[ns]
        st.latest_corrected = self._pose_tuple(msg)
        if st.current_xy is None:
            return
        cx, cy = st.current_xy
        cyaw = st.current_yaw
        ox, oy, oyaw = st.latest_corrected
        st.corrected_delta_trans_m = math.hypot(ox - cx, oy - cy)
        st.corrected_delta_yaw_deg = abs(math.degrees(wrap_pi(oyaw - cyaw)))

    def _on_odom(self, msg: Odometry, ns: str) -> None:
        st = self.states[ns]
        now = now_sec_from_node(self)
        x, y, yaw = self._pose_tuple(msg)
        if st.last_odom_xy is not None:
            st.path_len_m += math.hypot(x - st.last_odom_xy[0], y - st.last_odom_xy[1])
        st.last_odom_xy = (x, y)
        st.current_xy = (x, y)
        st.current_yaw = yaw
        st.current_t_sec = now

        if st.raw_anchor is None:
            st.raw_anchor = (x, y, yaw)
        if st.latest_gt is not None and st.gt_anchor is None:
            st.gt_anchor = st.latest_gt
        self._update_drift(st)
        self._maybe_add_keyframe(st, now, x, y, yaw)

    def _update_drift(self, st: RobotState) -> None:
        if st.raw_anchor is None or st.gt_anchor is None or st.latest_gt is None or st.current_xy is None:
            return
        rx0, ry0, ryaw0 = st.raw_anchor
        gx0, gy0, gyaw0 = st.gt_anchor
        gx, gy, gyaw = st.latest_gt
        rx, ry = st.current_xy
        raw_dx, raw_dy = rx - rx0, ry - ry0
        gt_dx, gt_dy = gx - gx0, gy - gy0
        st.drift_trans_m = math.hypot(raw_dx - gt_dx, raw_dy - gt_dy)
        st.drift_yaw_deg = abs(math.degrees(wrap_pi((st.current_yaw - ryaw0) - (gyaw - gyaw0))))
        st.drift_trans_peak_m = max(st.drift_trans_peak_m, st.drift_trans_m)
        st.drift_yaw_peak_deg = max(st.drift_yaw_peak_deg, st.drift_yaw_deg)

    def _maybe_add_keyframe(self, st: RobotState, now: float, x: float, y: float, yaw: float) -> None:
        if not st.keyframes:
            self._add_keyframe(st, now, x, y, yaw)
            return
        last = st.keyframes[-1]
        dist = math.hypot(x - last.x, y - last.y)
        dyaw = abs(wrap_pi(yaw - last.yaw))
        if dist >= self.keyframe_distance_m or dyaw >= self.keyframe_yaw_rad:
            self._add_keyframe(st, now, x, y, yaw)

    def _add_keyframe(self, st: RobotState, now: float, x: float, y: float, yaw: float) -> None:
        kid = f"keyframe_{st.ns}_{len(st.keyframes):04d}"
        st.keyframes.append(Keyframe(kid, st.ns, now, x, y, yaw, st.path_len_m))
        if len(st.keyframes) > self.max_keyframes:
            st.keyframes = st.keyframes[-self.max_keyframes:]

    def _candidate_counts(self, st: RobotState) -> tuple[int, float]:
        if st.current_xy is None:
            return 0, 0.0
        count = 0
        last_candidate_t = st.last_loop_candidate_t_sec
        for kf in st.keyframes[:-1]:
            time_gap = st.current_t_sec - kf.t_sec
            path_gap = st.path_len_m - kf.path_len_m
            if time_gap < self.candidate_time_gap_sec or path_gap < self.candidate_path_gap_m:
                continue
            if math.hypot(st.current_xy[0] - kf.x, st.current_xy[1] - kf.y) <= self.loop_candidate_radius_m:
                count += 1
                last_candidate_t = max(last_candidate_t, st.current_t_sec)
        if count:
            st.last_loop_candidate_t_sec = last_candidate_t
        distance_since = st.path_len_m
        if st.last_loop_candidate_t_sec > 0.0:
            # Approximate since last candidate by distance from newest eligible frame.
            eligible = [
                k.path_len_m for k in st.keyframes
                if k.t_sec <= st.last_loop_candidate_t_sec
            ]
            if eligible:
                distance_since = max(0.0, st.path_len_m - max(eligible))
        return count, distance_since

    def _inter_robot_opportunities(self, ns: str) -> int:
        st = self.states[ns]
        if st.current_xy is None:
            return 0
        count = 0
        for other_ns, other in self.states.items():
            if other_ns == ns:
                continue
            for kf in other.keyframes:
                time_gap = st.current_t_sec - kf.t_sec
                if time_gap < self.candidate_time_gap_sec:
                    continue
                if math.hypot(st.current_xy[0] - kf.x, st.current_xy[1] - kf.y) <= self.inter_robot_radius_m:
                    count += 1
        return count

    def _health_state(self, st: RobotState, self_crossings: int, inter_ops: int, distance_since: float) -> str:
        if st.drift_yaw_peak_deg >= self.drift_yaw_warn_deg or st.drift_trans_peak_m >= self.drift_trans_warn_m:
            return "drift_risk"
        if self_crossings > 0 or inter_ops > 0 or distance_since >= self.distance_since_revisit_warn_m:
            return "loop_needed"
        if len(st.keyframes) < 3:
            return "weak"
        return "good"

    def _payload(self) -> dict[str, Any]:
        now = now_sec_from_node(self)
        robots: dict[str, Any] = {}
        summary = {"worst_state": "good", "loop_needed_count": 0, "drift_risk_count": 0}
        order = {"good": 0, "weak": 1, "loop_needed": 2, "drift_risk": 3}
        for ns, st in self.states.items():
            self_crossings, distance_since = self._candidate_counts(st)
            inter_ops = self._inter_robot_opportunities(ns)
            st.self_crossings = self_crossings
            st.inter_robot_opportunities = inter_ops
            state = self._health_state(st, self_crossings, inter_ops, distance_since)
            if order[state] > order[str(summary["worst_state"])]:
                summary["worst_state"] = state
            if state == "loop_needed":
                summary["loop_needed_count"] += 1
            if state == "drift_risk":
                summary["drift_risk_count"] += 1
            robots[ns] = {
                "state": state,
                "keyframe_count": len(st.keyframes),
                "path_len_m": round(st.path_len_m, 3),
                "distance_since_revisit_m": round(distance_since, 3),
                "self_crossing_count": self_crossings,
                "inter_robot_opportunity_count": inter_ops,
                "drift_trans_m": round(st.drift_trans_m, 4),
                "drift_trans_peak_m": round(st.drift_trans_peak_m, 4),
                "drift_yaw_deg": round(st.drift_yaw_deg, 3),
                "drift_yaw_peak_deg": round(st.drift_yaw_peak_deg, 3),
                "corrected_delta_trans_m": round(st.corrected_delta_trans_m, 4),
                "corrected_delta_yaw_deg": round(st.corrected_delta_yaw_deg, 3),
                "current_pose": (
                    None if st.current_xy is None else {
                        "x": round(st.current_xy[0], 3),
                        "y": round(st.current_xy[1], 3),
                        "yaw": round(st.current_yaw, 4),
                    }
                ),
                "keyframes": [kf.as_dict() for kf in st.keyframes],
            }
        return {
            "schema": "pose_graph_health/v1",
            "stamp_sec": round(now, 3),
            "summary": summary,
            "robots": robots,
        }

    def _publish_markers(self, payload: dict[str, Any]) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1
        for idx, (ns, robot) in enumerate(payload.get("robots", {}).items()):
            color = (0.1, 0.7, 1.0, 0.9) if idx == 0 else (1.0, 0.6, 0.1, 0.9)
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = self.get_clock().now().to_msg()
            line.ns = f"{ns}_pose_graph"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.035
            line.color.r, line.color.g, line.color.b, line.color.a = color
            for kf in robot.get("keyframes", []):
                p = Point()
                p.x = float(kf["x"])
                p.y = float(kf["y"])
                p.z = 0.12
                line.points.append(p)
            markers.markers.append(line)
        self._marker_pub.publish(markers)

    def _tick(self) -> None:
        payload = self._payload()
        self._pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self._publish_markers(payload)
        atomic_write_json(self.output_path, payload)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = PoseGraphHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
