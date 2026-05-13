#!/usr/bin/env python3
"""cfpa2_to_nav2_bridge — translate CFPA2 way_point into Nav2 goal_pose.

CFPA2 publishes /<ns>/way_point as PointStamped (just an XY target with
no orientation). Nav2's bt_navigator subscribes to /<ns>/goal_pose as
PoseStamped (full pose with orientation). This bridge:

  - subscribes /<ns>/way_point (BEST_EFFORT to match CFPA2's QoS)
  - synthesizes orientation = atan2(goal - robot_pose) so the planner
    has a sensible terminal heading
  - publishes /<ns>/goal_pose (BEST_EFFORT to match Nav2's QoS) only
    when the goal *changed* — re-publishing identical goals at 2 Hz
    would force Nav2 to abort + replan every tick

Run alongside:
  - the sim (any backend including 'none')
  - cfpa2_coordinator (with explore:=true)
  - nav2_robot_a.launch.py (the Nav2 stack)
"""
from __future__ import annotations

import ast
import json
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import ClearCostmapAroundRobot
from std_msgs.msg import Empty, String

from go2_nav_algorithms.nav_costmap_utils import (
    GridSpec,
    cell_cost,
    make_start_cell_diagnostics,
    project_frontier_goal,
)


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


def odom_pose_inside_map(odom: Odometry, map_msg: OccupancyGrid | None, *, margin_m: float) -> bool:
    if map_msg is None:
        return True
    x = float(odom.pose.pose.position.x)
    y = float(odom.pose.pose.position.y)
    if not math.isfinite(x) or not math.isfinite(y):
        return False
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)
    resolution = float(map_msg.info.resolution)
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False
    margin = max(0.0, float(margin_m))
    min_x = float(map_msg.info.origin.position.x) - margin
    min_y = float(map_msg.info.origin.position.y) - margin
    max_x = float(map_msg.info.origin.position.x) + float(width) * resolution + margin
    max_y = float(map_msg.info.origin.position.y) + float(height) * resolution + margin
    return min_x <= x <= max_x and min_y <= y <= max_y


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def _grid_spec(msg: OccupancyGrid | None) -> GridSpec | None:
    if msg is None:
        return None
    return GridSpec(
        width=int(msg.info.width),
        height=int(msg.info.height),
        resolution=float(msg.info.resolution),
        origin_x=float(msg.info.origin.position.x),
        origin_y=float(msg.info.origin.position.y),
    )


def _parse_footprint(raw: object) -> list[tuple[float, float]]:
    raw_text = str(raw or "").strip()
    if raw_text and not raw_text.startswith("["):
        out: list[tuple[float, float]] = []
        for pair in raw_text.split(";"):
            if not pair.strip():
                continue
            try:
                x_text, y_text = pair.split(",", 1)
                out.append((float(x_text), float(y_text)))
            except ValueError:
                return []
        return out
    try:
        obj = ast.literal_eval(raw_text)
    except (SyntaxError, ValueError):
        return []
    out: list[tuple[float, float]] = []
    if not isinstance(obj, (list, tuple)):
        return out
    for item in obj:
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            continue
        try:
            out.append((float(item[0]), float(item[1])))
        except (TypeError, ValueError):
            continue
    return out


class Cfpa2ToNav2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("cfpa2_to_nav2_bridge")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("waypoint_topic", "way_point")
        self.declare_parameter("goal_pose_topic", "goal_pose")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("map_topic", "map")
        self.declare_parameter("global_costmap_topic", "global_costmap/costmap")
        self.declare_parameter("local_costmap_topic", "local_costmap/costmap")
        self.declare_parameter("frontier_replan_topic", "frontier_replan")
        self.declare_parameter("diagnostics_topic", "nav_start_cell_diagnostics")
        self.declare_parameter("reject_when_odom_outside_map", True)
        self.declare_parameter("map_bounds_margin_m", 2.0)
        self.declare_parameter("goal_projection_enabled", True)
        self.declare_parameter("goal_projection_search_radius_m", 1.5)
        self.declare_parameter("goal_cost_lethal_threshold", 90)
        self.declare_parameter("failed_goal_blacklist_radius", 1.0)
        self.declare_parameter("failed_goal_blacklist_ttl_sec", 45.0)
        self.declare_parameter("costmap_self_clear_radius", 0.65)
        self.declare_parameter("start_lethal_recovery_cooldown_sec", 1.5)
        self.declare_parameter("local_inflation_radius", 0.22)
        self.declare_parameter("global_inflation_radius", 0.20)
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("footprint", "0.325,0.15;0.325,-0.15;-0.325,-0.15;-0.325,0.15")
        # Skip republishing if new goal is within this distance of last
        # published goal — CFPA2 republishes its current goal at 2 Hz to
        # keep the channel alive, we don't want Nav2 to restart every tick.
        self.declare_parameter("goal_change_min_m", 0.30)

        ns = str(self.get_parameter("namespace").value)
        wp_topic = f"/{ns}/{self.get_parameter('waypoint_topic').value}"
        goal_topic = f"/{ns}/{self.get_parameter('goal_pose_topic').value}"
        odom_topic = f"/{ns}/{self.get_parameter('odom_topic').value}"
        map_topic = f"/{ns}/{self.get_parameter('map_topic').value}"
        global_costmap_topic = f"/{ns}/{self.get_parameter('global_costmap_topic').value}"
        local_costmap_topic = f"/{ns}/{self.get_parameter('local_costmap_topic').value}"
        frontier_replan_topic = f"/{ns}/{self.get_parameter('frontier_replan_topic').value}"
        diagnostics_topic = f"/{ns}/{self.get_parameter('diagnostics_topic').value}"
        self.goal_change_min_m = float(
            self.get_parameter("goal_change_min_m").value
        )
        self.reject_when_odom_outside_map = bool(
            self.get_parameter("reject_when_odom_outside_map").value
        )
        self.map_bounds_margin_m = max(0.0, float(self.get_parameter("map_bounds_margin_m").value))
        self.goal_projection_enabled = bool(self.get_parameter("goal_projection_enabled").value)
        self.goal_projection_search_radius_m = max(
            0.1, float(self.get_parameter("goal_projection_search_radius_m").value)
        )
        self.goal_cost_lethal_threshold = int(self.get_parameter("goal_cost_lethal_threshold").value)
        self.failed_goal_blacklist_radius = max(
            0.0, float(self.get_parameter("failed_goal_blacklist_radius").value)
        )
        self.failed_goal_blacklist_ttl_sec = max(
            1.0, float(self.get_parameter("failed_goal_blacklist_ttl_sec").value)
        )
        self.costmap_self_clear_radius = max(
            0.0, float(self.get_parameter("costmap_self_clear_radius").value)
        )
        self.start_lethal_recovery_cooldown_sec = max(
            0.1, float(self.get_parameter("start_lethal_recovery_cooldown_sec").value)
        )
        self.local_inflation_radius = float(self.get_parameter("local_inflation_radius").value)
        self.global_inflation_radius = float(self.get_parameter("global_inflation_radius").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.footprint = _parse_footprint(str(self.get_parameter("footprint").value))

        # CFPA2's way_point_coord publishes RELIABLE; odom_relay also reliable.
        cfpa_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Nav2's bt_navigator subscribes goal_pose with rclcpp::SystemDefaultsQoS,
        # which is RELIABLE/VOLATILE/KEEP_LAST(10). Earlier comment claiming
        # BEST_EFFORT was wrong — published goals were silently dropped on the
        # wire (publisher logs "forwarded goal" but bt_navigator never received
        # them). Confirmed via the runtime warning:
        #   "New publisher discovered on '/<ns>/goal_pose'... incompatible
        #    QoS. Last incompatible policy: RELIABILITY".
        nav2_goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._last_pose_x: float | None = None
        self._last_pose_y: float | None = None
        self._last_odom: Odometry | None = None
        self._latest_map: OccupancyGrid | None = None
        self._latest_global_costmap: OccupancyGrid | None = None
        self._latest_local_costmap: OccupancyGrid | None = None
        self._last_goal_x: float | None = None
        self._last_goal_y: float | None = None
        self._last_invalid_odom_warn_ns = 0
        self._failed_goals: deque[tuple[float, float, float]] = deque(maxlen=64)
        self._last_recovery_wall_sec = 0.0
        self._rejected_frontier_goals = 0
        self._projection_successes = 0
        self._start_in_lethal_recoveries = 0
        self._last_projection_payload: dict | None = None

        self.create_subscription(Odometry, odom_topic, self._on_odom, cfpa_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, cfpa_qos)
        self.create_subscription(OccupancyGrid, global_costmap_topic, self._on_global_costmap, cfpa_qos)
        self.create_subscription(OccupancyGrid, local_costmap_topic, self._on_local_costmap, cfpa_qos)
        self.create_subscription(
            PointStamped, wp_topic, self._on_waypoint, cfpa_qos
        )
        self._goal_pub = self.create_publisher(
            PoseStamped, goal_topic, nav2_goal_qos
        )
        self._frontier_replan_pub = self.create_publisher(Empty, frontier_replan_topic, 10)
        self._diagnostics_pub = self.create_publisher(String, diagnostics_topic, 10)
        self._local_clear = self.create_client(
            ClearCostmapAroundRobot,
            f"/{ns}/local_costmap/clear_around_local_costmap",
        )
        self._global_clear = self.create_client(
            ClearCostmapAroundRobot,
            f"/{ns}/global_costmap/clear_around_global_costmap",
        )
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"bridge armed. {wp_topic} → {goal_topic}; pose from {odom_topic}; "
            f"projection={self.goal_projection_enabled} diagnostics={diagnostics_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_pose_x = msg.pose.pose.position.x
        self._last_pose_y = msg.pose.pose.position.y

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._latest_global_costmap = msg

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._latest_local_costmap = msg

    def _on_waypoint(self, msg: PointStamped) -> None:
        raw_gx, raw_gy = float(msg.point.x), float(msg.point.y)
        if (
            self.reject_when_odom_outside_map
            and self._last_odom is not None
            and not odom_pose_inside_map(
                self._last_odom,
                self._latest_map,
                margin_m=self.map_bounds_margin_m,
            )
        ):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_invalid_odom_warn_ns > int(2e9):
                pos = self._last_odom.pose.pose.position
                self.get_logger().warn(
                    "dropping CFPA2 waypoint because odom/nav is outside map bounds "
                    f"pose=({float(pos.x):+.2f},{float(pos.y):+.2f})"
                )
                self._last_invalid_odom_warn_ns = now_ns
            return
        if self._is_goal_blacklisted(raw_gx, raw_gy):
            self._rejected_frontier_goals += 1
            self._publish_replan()
            self._publish_diagnostics(
                extra={
                    "raw_frontier_goal": [round(raw_gx, 4), round(raw_gy, 4)],
                    "projection_success": False,
                    "reason": "goal_blacklisted_after_failed_plan",
                }
            )
            return

        start_diag = self._diagnostics_payload()
        start_blocked = bool(start_diag.get("start_cell_lethal")) or (
            start_diag.get("footprint_collision_status") == "collision"
        )
        if start_blocked:
            self._rejected_frontier_goals += 1
            self._blacklist_goal(raw_gx, raw_gy)
            self._recover_start_in_lethal(start_diag)
            self._publish_diagnostics(
                extra={
                    "raw_frontier_goal": [round(raw_gx, 4), round(raw_gy, 4)],
                    "projection_success": False,
                    "reason": "start_cell_or_footprint_blocked",
                }
            )
            return

        gx, gy = raw_gx, raw_gy
        projection_payload: dict = {
            "raw_frontier_goal": [round(raw_gx, 4), round(raw_gy, 4)],
            "projected_approach_goal": None,
            "projected_goal_cost": None,
            "projection_success": False,
            "reason": "projection_disabled",
        }
        if self.goal_projection_enabled and self._last_odom is not None:
            projected = self._project_goal(raw_gx, raw_gy)
            projection_payload.update({
                "projection_success": projected.projection_success,
                "reason": projected.reason,
                "projected_goal_cost": projected.projected_goal_cost,
            })
            if not projected.projection_success or projected.projected_goal is None:
                self._rejected_frontier_goals += 1
                self._blacklist_goal(raw_gx, raw_gy)
                self._publish_replan()
                self._last_projection_payload = projection_payload
                self._publish_diagnostics(extra=projection_payload)
                return
            gx, gy = projected.projected_goal
            projection_payload["projected_approach_goal"] = [round(gx, 4), round(gy, 4)]
            self._projection_successes += 1
        self._last_projection_payload = projection_payload
        self._publish_diagnostics(extra=projection_payload)
        # Suppress duplicate / sub-threshold-change goals.
        if (
            self._last_goal_x is not None
            and math.hypot(gx - self._last_goal_x, gy - self._last_goal_y)
            < self.goal_change_min_m
        ):
            return

        # Synthesize orientation pointing from current robot pose to goal.
        # If we have no odom yet, default to facing +x (yaw=0).
        yaw = 0.0
        if self._last_pose_x is not None:
            dx = gx - self._last_pose_x
            dy = gy - self._last_pose_y
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or "map"
        out.pose.position.x = gx
        out.pose.position.y = gy
        out.pose.position.z = 0.0
        # quaternion from yaw alone (z-axis rotation).
        out.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.orientation.w = math.cos(0.5 * yaw)
        self._goal_pub.publish(out)

        self.get_logger().info(
            f"forwarded goal ({gx:+.2f}, {gy:+.2f}) yaw={math.degrees(yaw):+.1f}° "
            f"raw=({raw_gx:+.2f},{raw_gy:+.2f}) projection={projection_payload['reason']}"
        )
        self._last_goal_x = gx
        self._last_goal_y = gy

    def _project_goal(self, raw_gx: float, raw_gy: float):
        odom = self._last_odom
        costmap = self._latest_global_costmap or self._latest_map
        spec = _grid_spec(costmap)
        if odom is None or costmap is None or spec is None:
            from go2_nav_algorithms.nav_costmap_utils import ProjectionResult
            return ProjectionResult((raw_gx, raw_gy), (raw_gx, raw_gy), None, True, "projection_unavailable_passthrough")
        pose = odom.pose.pose.position
        return project_frontier_goal(
            list(costmap.data),
            spec,
            raw_x=raw_gx,
            raw_y=raw_gy,
            current_x=float(pose.x),
            current_y=float(pose.y),
            start_x=float(pose.x),
            start_y=float(pose.y),
            min_current_distance=0.35,
            min_start_distance=0.35,
            failed_goals=[(x, y) for x, y, _until in self._failed_goals],
            failed_goal_radius=self.failed_goal_blacklist_radius,
            max_search_radius_m=self.goal_projection_search_radius_m,
            lethal_threshold=self.goal_cost_lethal_threshold,
        )

    def _diagnostics_payload(self) -> dict:
        odom = self._last_odom
        payload = {
            "schema": "nav_start_cell_diagnostics/v1",
            "active": odom is not None,
            "local_costmap_received": self._latest_local_costmap is not None,
            "global_costmap_received": self._latest_global_costmap is not None,
            "rejected_frontier_goals": int(self._rejected_frontier_goals),
            "projection_success_count": int(self._projection_successes),
            "start_in_lethal_recoveries": int(self._start_in_lethal_recoveries),
            "costmap_self_clear_radius": float(self.costmap_self_clear_radius),
        }
        if odom is None:
            return payload
        pose = odom.pose.pose
        local_spec = _grid_spec(self._latest_local_costmap)
        global_spec = _grid_spec(self._latest_global_costmap)
        diag = make_start_cell_diagnostics(
            local_data=list(self._latest_local_costmap.data) if self._latest_local_costmap else None,
            local_spec=local_spec,
            global_data=list(self._latest_global_costmap.data) if self._latest_global_costmap else None,
            global_spec=global_spec,
            pose_x=float(pose.position.x),
            pose_y=float(pose.position.y),
            pose_yaw=_yaw_from_quaternion(pose.orientation),
            footprint=self.footprint,
            local_inflation_radius=self.local_inflation_radius,
            global_inflation_radius=self.global_inflation_radius,
            robot_radius=self.robot_radius,
            lethal_threshold=self.goal_cost_lethal_threshold,
        )
        payload.update(diag)
        if self._latest_global_costmap is not None and global_spec is not None and self._last_goal_x is not None:
            goal_cell = cell_cost(
                list(self._latest_global_costmap.data),
                global_spec,
                self._last_goal_x,
                self._last_goal_y,
                lethal_threshold=self.goal_cost_lethal_threshold,
            )
            payload["last_sent_goal_cost"] = goal_cell.cost
            payload["last_sent_goal_status"] = goal_cell.status
        if self._last_projection_payload:
            payload.update(self._last_projection_payload)
        return payload

    def _publish_diagnostics(self, *, extra: dict | None = None) -> None:
        payload = self._diagnostics_payload()
        if extra:
            payload.update(extra)
        self._diagnostics_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _recover_start_in_lethal(self, diag: dict) -> None:
        now = time.monotonic()
        if now - self._last_recovery_wall_sec < self.start_lethal_recovery_cooldown_sec:
            return
        self._last_recovery_wall_sec = now
        self._start_in_lethal_recoveries += 1
        self._publish_replan()
        for client, label in ((self._local_clear, "local"), (self._global_clear, "global")):
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=0.05)
            if client.service_is_ready():
                req = ClearCostmapAroundRobot.Request()
                req.reset_distance = float(max(self.costmap_self_clear_radius, self.robot_radius + 0.2))
                client.call_async(req)
        self.get_logger().warn(
            "start cell blocked before Nav2 goal; clearing costmaps around robot "
            f"and requesting replan diag={json.dumps(diag, sort_keys=True)}"
        )

    def _publish_replan(self) -> None:
        self._frontier_replan_pub.publish(Empty())

    def _blacklist_goal(self, x: float, y: float) -> None:
        self._failed_goals.append((float(x), float(y), time.monotonic() + self.failed_goal_blacklist_ttl_sec))

    def _is_goal_blacklisted(self, x: float, y: float) -> bool:
        now = time.monotonic()
        while self._failed_goals and self._failed_goals[0][2] < now:
            self._failed_goals.popleft()
        return any(
            math.hypot(float(x) - gx, float(y) - gy) <= self.failed_goal_blacklist_radius
            for gx, gy, until in self._failed_goals
            if until >= now
        )


def main(argv=None) -> int:
    _, ros_argv = _split_ros_argv(argv if argv else sys.argv[1:])
    rclpy.init(args=([sys.argv[0]] + ros_argv) if ros_argv else None)
    node = None
    try:
        node = Cfpa2ToNav2Bridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    main()
