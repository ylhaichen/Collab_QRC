#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from collections import deque
from typing import Any

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .common import atomic_write_json, clamp01, nearest_occupied_distance, now_sec_from_node, yaw_from_quat


def _tilt_deg_from_quat(q: Any) -> float:
    # Body-Z expressed in world frame. acos(z_axis.z) is robust enough for
    # the sim-risk signal; the collision monitor remains the authoritative
    # tip-over reporter for benchmark JSON.
    z_axis_z = 1.0 - 2.0 * (float(q.x) * float(q.x) + float(q.y) * float(q.y))
    return math.degrees(math.acos(max(-1.0, min(1.0, z_axis_z))))


class MorphologyRiskNode(Node):
    """Publish lightweight robot-specific execution risk for CFPA2.

    This node intentionally works as a proxy layer. It converts observed
    contacts, slow progress, peer proximity, and map clearance into a compact
    JSON signal; it does not modify SLAM or local planner internals.
    """

    def __init__(self) -> None:
        super().__init__("morphology_risk_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("output_path", "")
        self.declare_parameter("map_topic", "/merged_map")
        self.declare_parameter("clearance_probe_radius_m", 1.5)
        self.declare_parameter("contact_heat_resolution_m", 0.5)
        self.declare_parameter("contact_decay_sec", 90.0)
        self.declare_parameter("stuck_window_sec", 10.0)
        self.declare_parameter("stuck_progress_m", 0.12)
        self.declare_parameter("goal_reached_radius_m", 0.45)
        self.declare_parameter("peer_near_radius_m", 1.4)
        self.declare_parameter("peer_crossing_radius_m", 1.0)

        self.namespaces = [str(x).strip("/") for x in self.get_parameter("namespaces").value]
        self.output_path = str(self.get_parameter("output_path").value)
        self.clearance_probe_radius_m = float(self.get_parameter("clearance_probe_radius_m").value)
        self.contact_heat_resolution_m = max(0.1, float(self.get_parameter("contact_heat_resolution_m").value))
        self.contact_decay_sec = max(1.0, float(self.get_parameter("contact_decay_sec").value))
        self.stuck_window_sec = max(2.0, float(self.get_parameter("stuck_window_sec").value))
        self.stuck_progress_m = max(0.0, float(self.get_parameter("stuck_progress_m").value))
        self.goal_reached_radius_m = max(0.05, float(self.get_parameter("goal_reached_radius_m").value))
        self.peer_near_radius_m = max(0.2, float(self.get_parameter("peer_near_radius_m").value))
        self.peer_crossing_radius_m = max(0.2, float(self.get_parameter("peer_crossing_radius_m").value))

        self.map_msg: OccupancyGrid | None = None
        self.odom: dict[str, Odometry] = {}
        self.goals: dict[str, tuple[float, float]] = {}
        self.nav_status: dict[str, dict[str, Any]] = {}
        self.odom_hist: dict[str, deque[tuple[float, float, float]]] = {
            ns: deque() for ns in self.namespaces
        }
        self.events: dict[str, deque[dict[str, Any]]] = {ns: deque(maxlen=500) for ns in self.namespaces}
        self.heat: dict[str, dict[tuple[int, int], float]] = {ns: {} for ns in self.namespaces}
        self.last_payload: dict[str, Any] = {}

        self.pub = self.create_publisher(String, "/cfpa2/morphology_risk", 10)
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
        self.create_subscription(String, "/collision_events", self._on_collision_event, 50)
        for ns in self.namespaces:
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._on_odom(n, m), 10)
            self.create_subscription(PointStamped, f"/{ns}/way_point_coord", lambda m, n=ns: self._on_goal(n, m), 10)
            self.create_subscription(String, f"/{ns}/nav_status", lambda m, n=ns: self._on_nav_status(n, m), 10)

        rate = max(0.2, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"morphology_risk_node up: robots={self.namespaces} output={self.output_path or '<none>'}"
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_odom(self, ns: str, msg: Odometry) -> None:
        self.odom[ns] = msg
        p = msg.pose.pose.position
        t = now_sec_from_node(self)
        hist = self.odom_hist.setdefault(ns, deque())
        hist.append((t, float(p.x), float(p.y)))
        while hist and (t - hist[0][0]) > (self.stuck_window_sec + 1.0):
            hist.popleft()

    def _on_goal(self, ns: str, msg: PointStamped) -> None:
        self.goals[ns] = (float(msg.point.x), float(msg.point.y))

    def _on_nav_status(self, ns: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"state": msg.data}
        if isinstance(payload, dict):
            self.nav_status[ns] = payload

    def _on_collision_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        ns = str(payload.get("robot", "")).strip("/")
        if ns not in self.events:
            return
        t = float(payload.get("t_sim", now_sec_from_node(self)))
        kind = str(payload.get("kind", "unknown"))
        pos = payload.get("pos") if isinstance(payload.get("pos"), list) else None
        event = {
            "t_sec": round(t, 3),
            "kind": kind,
            "other": str(payload.get("other", "")),
            "tilt_deg": float(payload.get("tilt_deg", 0.0) or 0.0),
        }
        if pos and len(pos) >= 2:
            x = float(pos[0])
            y = float(pos[1])
            event["x"] = round(x, 3)
            event["y"] = round(y, 3)
            key = (
                int(round(x / self.contact_heat_resolution_m)),
                int(round(y / self.contact_heat_resolution_m)),
            )
            self.heat[ns][key] = self.heat[ns].get(key, 0.0) + 1.0
        self.events[ns].append(event)

    def _pose_xy_yaw_tilt(self, ns: str) -> tuple[float, float, float, float] | None:
        msg = self.odom.get(ns)
        if msg is None:
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return float(p.x), float(p.y), yaw_from_quat(q), _tilt_deg_from_quat(q)

    def _goal_distance(self, ns: str) -> float | None:
        pose = self._pose_xy_yaw_tilt(ns)
        goal = self.goals.get(ns)
        if pose is None or goal is None:
            return None
        return math.hypot(goal[0] - pose[0], goal[1] - pose[1])

    def _slow_progress_risk(self, ns: str) -> tuple[float, bool]:
        hist = self.odom_hist.get(ns)
        d_goal = self._goal_distance(ns)
        if not hist or len(hist) < 2 or d_goal is None or d_goal < self.goal_reached_radius_m:
            return 0.0, False
        now = hist[-1][0]
        old = None
        for sample in hist:
            if now - sample[0] >= self.stuck_window_sec * 0.8:
                old = sample
                break
        if old is None:
            return 0.0, False
        moved = math.hypot(hist[-1][1] - old[1], hist[-1][2] - old[2])
        risk = clamp01((self.stuck_progress_m - moved) / max(self.stuck_progress_m, 0.01))
        nav_state = str(self.nav_status.get(ns, {}).get("state", "")).lower()
        if any(tok in nav_state for tok in ("stuck", "unreachable", "failed", "blocked")):
            risk = max(risk, 0.8)
        return risk, risk >= 0.7

    def _peer_features(self, ns: str) -> tuple[float, float, float]:
        pose = self._pose_xy_yaw_tilt(ns)
        if pose is None:
            return 0.0, 0.0, float("inf")
        best_peer_dist = float("inf")
        crossing = 0.0
        for other in self.namespaces:
            if other == ns:
                continue
            opose = self._pose_xy_yaw_tilt(other)
            if opose is None:
                continue
            dist = math.hypot(opose[0] - pose[0], opose[1] - pose[1])
            best_peer_dist = min(best_peer_dist, dist)
            goal = self.goals.get(ns)
            ogoal = self.goals.get(other)
            if goal is not None and ogoal is not None:
                # Cheap predicted-crossing proxy: if each robot's goal is
                # close to the other robot's current line of travel, raise risk.
                d_goal_to_peer = math.hypot(goal[0] - opose[0], goal[1] - opose[1])
                d_other_goal_to_self = math.hypot(ogoal[0] - pose[0], ogoal[1] - pose[1])
                crossing = max(
                    crossing,
                    clamp01((self.peer_crossing_radius_m - min(d_goal_to_peer, d_other_goal_to_self))
                            / self.peer_crossing_radius_m),
                )
        peer_risk = 0.0 if not math.isfinite(best_peer_dist) else clamp01(
            (self.peer_near_radius_m - best_peer_dist) / self.peer_near_radius_m
        )
        return peer_risk, crossing, best_peer_dist

    def _top_heat(self, ns: str, limit: int = 8) -> list[dict[str, Any]]:
        items = sorted(self.heat.get(ns, {}).items(), key=lambda kv: kv[1], reverse=True)[:limit]
        out = []
        for (ix, iy), count in items:
            out.append({
                "x": round(ix * self.contact_heat_resolution_m, 3),
                "y": round(iy * self.contact_heat_resolution_m, 3),
                "count": round(float(count), 3),
            })
        return out

    def _robot_payload(self, ns: str) -> dict[str, Any]:
        pose = self._pose_xy_yaw_tilt(ns)
        clearance = float("inf")
        if pose is not None and self.map_msg is not None:
            clearance = nearest_occupied_distance(self.map_msg, pose[0], pose[1], self.clearance_probe_radius_m)
        body_margin = 0.52 if ns == "robot_a" else 0.28
        near_wall_risk = 0.0 if not math.isfinite(clearance) else clamp01((body_margin - clearance) / max(body_margin, 0.05))
        slow_risk, stuck = self._slow_progress_risk(ns)
        peer_risk, crossing_risk, peer_dist = self._peer_features(ns)
        tilt = 0.0 if pose is None else pose[3]
        tip_risk = clamp01((tilt - 25.0) / 55.0)
        recent_contacts = [
            e for e in self.events.get(ns, [])
            if now_sec_from_node(self) - float(e.get("t_sec", 0.0)) <= self.contact_decay_sec
        ]
        contact_rate = len(recent_contacts) / self.contact_decay_sec
        contact_risk = clamp01(contact_rate / 1.5)
        multiplier = {
            "wall_end": 1.25 if ns == "robot_a" else 1.00,
            "tip": 1.40 if ns == "robot_a" else 0.85,
            "slow_progress": 0.85 if ns == "robot_a" else 1.35,
            "corner_stamping": 0.95 if ns == "robot_a" else 1.35,
        }
        risk_score = clamp01(
            0.25 * near_wall_risk * multiplier["wall_end"]
            + 0.25 * contact_risk * multiplier["corner_stamping"]
            + 0.20 * slow_risk * multiplier["slow_progress"]
            + 0.15 * tip_risk * multiplier["tip"]
            + 0.10 * peer_risk
            + 0.05 * crossing_risk
        )
        return {
            "risk_score": round(risk_score, 4),
            "near_wall_clearance_m": None if not math.isfinite(clearance) else round(clearance, 3),
            "near_wall_risk": round(near_wall_risk, 4),
            "contact_rate_hz": round(contact_rate, 4),
            "contact_risk": round(contact_risk, 4),
            "stuck_risk": round(slow_risk, 4),
            "stuck_event": bool(stuck),
            "tilt_deg": round(tilt, 3),
            "tip_risk": round(tip_risk, 4),
            "peer_distance_m": None if not math.isfinite(peer_dist) else round(peer_dist, 3),
            "peer_risk": round(peer_risk, 4),
            "predicted_crossing_risk": round(crossing_risk, 4),
            "multipliers": multiplier,
            "contact_heat_top": self._top_heat(ns),
            "recent_contact_count": len(recent_contacts),
        }

    def _payload(self) -> dict[str, Any]:
        robots = {ns: self._robot_payload(ns) for ns in self.namespaces}
        return {
            "schema": "morphology_risk/v1",
            "stamp_sec": round(now_sec_from_node(self), 3),
            "robots": robots,
        }

    def _tick(self) -> None:
        payload = self._payload()
        self.last_payload = payload
        self.pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        atomic_write_json(self.output_path, payload)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = MorphologyRiskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
