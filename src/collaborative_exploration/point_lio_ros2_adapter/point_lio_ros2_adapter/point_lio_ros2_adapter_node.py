from __future__ import annotations

import copy
import json
import math
import time
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
except ModuleNotFoundError:  # Pure contract tests can run without ROS 2 sourced.
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[misc,assignment]
    Odometry = Any  # type: ignore[misc,assignment]
    OccupancyGrid = Any  # type: ignore[misc,assignment]
    PointCloud2 = Any  # type: ignore[misc,assignment]
    Imu = Any  # type: ignore[misc,assignment]
    String = Any  # type: ignore[misc,assignment]
    TransformStamped = Any  # type: ignore[misc,assignment]
    TransformBroadcaster = None  # type: ignore[assignment]

from .contracts import PointLioAdapterStatus, build_point_lio_contract


def _yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def odom_within_map_bounds(msg: Odometry, map_msg: OccupancyGrid | None, *, margin_m: float) -> bool:
    if map_msg is None:
        return True
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
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


class _RateCounter:
    def __init__(self) -> None:
        self.times: list[float] = []

    def mark(self) -> None:
        now = time.monotonic()
        self.times.append(now)
        cutoff = now - 5.0
        self.times = [t for t in self.times if t >= cutoff]

    def rate_hz(self) -> float:
        if len(self.times) < 2:
            return 0.0
        span = max(1e-6, self.times[-1] - self.times[0])
        return float(len(self.times) - 1) / span


class PointLioRos2AdapterNode(Node):
    """Expose Point-LIO output through the existing Collab_QRC local SLAM topics."""

    def __init__(self) -> None:
        super().__init__("point_lio_ros2_adapter_node")
        self.declare_parameter("robot_namespace", "robot_a")
        self.declare_parameter("mode", "shadow")
        self.declare_parameter("publish_primary_contract", False)
        self.declare_parameter("livox_topic", "livox/lidar")
        self.declare_parameter("imu_topic", "livox/imu")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("status_topic", "/team_slam/local/status")
        self.declare_parameter("output_frame_id", "odom")
        self.declare_parameter("output_child_frame_id", "base_link")
        self.declare_parameter("output_cloud_frame_id", "")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("publish_map_to_odom_tf", False)
        self.declare_parameter("align_cloud_stamp_to_odom", True)
        self.declare_parameter("odom_jump_guard_enabled", True)
        self.declare_parameter("max_odom_translation_step_m", 1.0)
        self.declare_parameter("max_odom_yaw_step_deg", 60.0)
        self.declare_parameter("max_odom_speed_mps", 3.0)
        self.declare_parameter("map_bounds_guard_enabled", False)
        self.declare_parameter("map_bounds_topic", "map")
        self.declare_parameter("map_bounds_margin_m", 2.0)

        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip().strip("/")
        self.mode = str(self.get_parameter("mode").value).strip().lower() or "shadow"
        self.publish_primary = bool(self.get_parameter("publish_primary_contract").value)
        if self.mode == "primary":
            self.publish_primary = True
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value).strip() or "odom"
        self.output_child_frame_id = str(self.get_parameter("output_child_frame_id").value).strip() or "base_link"
        self.output_cloud_frame_id = (
            str(self.get_parameter("output_cloud_frame_id").value).strip() or self.output_child_frame_id
        )
        self.map_frame_id = str(self.get_parameter("map_frame_id").value).strip() or "map"
        self.publish_map_to_odom_tf = bool(self.get_parameter("publish_map_to_odom_tf").value)
        self.align_cloud_stamp_to_odom = bool(self.get_parameter("align_cloud_stamp_to_odom").value)
        self.odom_jump_guard_enabled = bool(self.get_parameter("odom_jump_guard_enabled").value)
        self.max_odom_translation_step_m = float(self.get_parameter("max_odom_translation_step_m").value)
        self.max_odom_yaw_step_rad = math.radians(float(self.get_parameter("max_odom_yaw_step_deg").value))
        self.max_odom_speed_mps = float(self.get_parameter("max_odom_speed_mps").value)
        self.map_bounds_guard_enabled = bool(self.get_parameter("map_bounds_guard_enabled").value)
        self.map_bounds_topic = str(self.get_parameter("map_bounds_topic").value).strip().strip("/") or "map"
        self.map_bounds_margin_m = max(0.0, float(self.get_parameter("map_bounds_margin_m").value))
        self.contract = build_point_lio_contract(self.robot_namespace)
        self.odom_rate = _RateCounter()
        self.adapter_odom_rate = _RateCounter()
        self.cloud_rate = _RateCounter()
        self.livox_input_seen = False
        self.imu_input_seen = False
        self.last_odom_stamp = None
        self.last_native_odom: Odometry | None = None
        self.last_published_odom: Odometry | None = None
        self.last_map_valid_odom: Odometry | None = None
        self.latest_bounds_map: OccupancyGrid | None = None
        self.rejected_odom_jumps = 0
        self.rejected_odom_bounds = 0
        self._last_reject_reason = ""

        self.odom_pub = self.create_publisher(Odometry, self.contract.outputs["odometry"], 20)
        self.corrected_pub = self.create_publisher(Odometry, self.contract.outputs["corrected_odom"], 20)
        self.nav_pub = self.create_publisher(Odometry, self.contract.outputs["nav_odom"], 20)
        self.body_cloud_pub = self.create_publisher(PointCloud2, self.contract.outputs["registered_body"], 5)
        self.static_cloud_pub = self.create_publisher(PointCloud2, self.contract.outputs["static_cloud"], 5)
        self.dynamic_cloud_pub = self.create_publisher(PointCloud2, self.contract.outputs["dynamic_cloud"], 5)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf and TransformBroadcaster else None

        self.create_subscription(Odometry, self.contract.native_odom_topic, self._on_native_odom, 20)
        if self.map_bounds_guard_enabled:
            map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                OccupancyGrid,
                f"/{self.robot_namespace}/{self.map_bounds_topic}",
                self._on_bounds_map,
                map_qos,
            )
        self.create_subscription(PointCloud2, self.contract.native_cloud_topic, self._on_native_cloud, 5)
        self.create_subscription(PointCloud2, self.contract.native_static_topic, self._on_native_static, 5)
        self.create_subscription(PointCloud2, self.contract.native_dynamic_topic, self._on_native_dynamic, 5)
        self.create_subscription(
            PointCloud2,
            f"/{self.robot_namespace}/{str(self.get_parameter('livox_topic').value).strip().strip('/')}",
            self._on_livox,
            5,
        )
        self.create_subscription(
            Imu,
            f"/{self.robot_namespace}/{str(self.get_parameter('imu_topic').value).strip().strip('/')}",
            self._on_imu,
            20,
        )
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            f"Point-LIO adapter up robot={self.robot_namespace} mode={self.mode} "
            f"publish_primary_contract={self.publish_primary} "
            f"frame={self.output_frame_id} child={self.output_child_frame_id} "
            f"map_to_odom_tf={self.publish_map_to_odom_tf} "
            f"odom_jump_guard={self.odom_jump_guard_enabled} "
            f"map_bounds_guard={self.map_bounds_guard_enabled}"
        )

    def _on_livox(self, _msg: PointCloud2) -> None:
        self.livox_input_seen = True

    def _on_imu(self, _msg: Imu) -> None:
        self.imu_input_seen = True

    def _on_bounds_map(self, msg: OccupancyGrid) -> None:
        self.latest_bounds_map = msg
        if self.last_published_odom is not None and odom_within_map_bounds(
            self.last_published_odom,
            msg,
            margin_m=self.map_bounds_margin_m,
        ):
            self.last_map_valid_odom = copy.deepcopy(self.last_published_odom)

    def _retarget_odom(self, msg: Odometry) -> Odometry:
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.child_frame_id = self.output_child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        return out

    @staticmethod
    def _stamp_sec(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _native_odom_is_plausible(self, msg: Odometry) -> bool:
        self._last_reject_reason = ""
        if self.map_bounds_guard_enabled and not odom_within_map_bounds(
            msg,
            self.latest_bounds_map,
            margin_m=self.map_bounds_margin_m,
        ):
            self._last_reject_reason = "map_bounds"
            self.rejected_odom_bounds += 1
            return False
        if not self.odom_jump_guard_enabled or self.last_native_odom is None:
            return True
        prev = self.last_native_odom.pose.pose.position
        cur = msg.pose.pose.position
        dist = math.sqrt(
            (float(cur.x) - float(prev.x)) ** 2
            + (float(cur.y) - float(prev.y)) ** 2
            + (float(cur.z) - float(prev.z)) ** 2
        )
        yaw_delta = abs(_wrap_pi(_yaw_from_odom(msg) - _yaw_from_odom(self.last_native_odom)))
        dt = max(0.0, self._stamp_sec(msg) - self._stamp_sec(self.last_native_odom))
        speed_gate = self.max_odom_speed_mps * dt + 0.15 if dt > 0.0 else 0.0
        trans_gate = max(self.max_odom_translation_step_m, speed_gate)
        ok = dist <= trans_gate and yaw_delta <= self.max_odom_yaw_step_rad
        if not ok:
            self._last_reject_reason = "jump"
        return ok

    def _publish_tf(self, odom: Odometry) -> None:
        if self.tf_br is None:
            return
        transforms = []
        if self.publish_map_to_odom_tf and self.map_frame_id != self.output_frame_id:
            map_tf = TransformStamped()
            map_tf.header.stamp = odom.header.stamp
            map_tf.header.frame_id = self.map_frame_id
            map_tf.child_frame_id = self.output_frame_id
            map_tf.transform.rotation.w = 1.0
            transforms.append(map_tf)
        tf_msg = TransformStamped()
        tf_msg.header = odom.header
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation
        transforms.append(tf_msg)
        self.tf_br.sendTransform(transforms)

    def _on_native_odom(self, msg: Odometry) -> None:
        self.odom_rate.mark()
        if not self.publish_primary:
            return
        if self._native_odom_is_plausible(msg):
            self.last_native_odom = copy.deepcopy(msg)
            odom = self._retarget_odom(msg)
            self.last_published_odom = copy.deepcopy(odom)
            if odom_within_map_bounds(
                odom,
                self.latest_bounds_map,
                margin_m=self.map_bounds_margin_m,
            ):
                self.last_map_valid_odom = copy.deepcopy(odom)
        elif self.last_published_odom is not None:
            self.rejected_odom_jumps += 1
            if self.rejected_odom_jumps <= 5 or self.rejected_odom_jumps % 50 == 0:
                self.get_logger().warn(
                    "Rejecting implausible Point-LIO odometry; "
                    f"robot={self.robot_namespace} reason={self._last_reject_reason or 'unknown'} "
                    f"rejected_count={self.rejected_odom_jumps}"
                )
            hold = self.last_published_odom
            if self.map_bounds_guard_enabled and self.latest_bounds_map is not None:
                if self.last_map_valid_odom is not None:
                    hold = self.last_map_valid_odom
                elif not odom_within_map_bounds(
                    hold,
                    self.latest_bounds_map,
                    margin_m=self.map_bounds_margin_m,
                ):
                    return
            odom = copy.deepcopy(hold)
            odom.header.stamp = msg.header.stamp
        else:
            self.rejected_odom_jumps += 1
            return
        self.last_odom_stamp = odom.header.stamp
        self.odom_pub.publish(odom)
        self.corrected_pub.publish(odom)
        self.nav_pub.publish(odom)
        self._publish_tf(odom)
        self.adapter_odom_rate.mark()

    def _retarget_cloud(self, msg: PointCloud2) -> PointCloud2:
        if self.align_cloud_stamp_to_odom and self.last_odom_stamp is not None:
            msg.header.stamp = self.last_odom_stamp
        msg.header.frame_id = self.output_cloud_frame_id
        return msg

    def _on_native_cloud(self, msg: PointCloud2) -> None:
        self.cloud_rate.mark()
        if self.publish_primary:
            out = self._retarget_cloud(msg)
            self.body_cloud_pub.publish(out)
            self.static_cloud_pub.publish(out)

    def _on_native_static(self, msg: PointCloud2) -> None:
        if self.publish_primary:
            self.static_cloud_pub.publish(self._retarget_cloud(msg))

    def _on_native_dynamic(self, msg: PointCloud2) -> None:
        if self.publish_primary:
            self.dynamic_cloud_pub.publish(self._retarget_cloud(msg))

    def _publish_status(self) -> None:
        status = PointLioAdapterStatus(
            robot_namespace=self.robot_namespace,
            mode=self.mode,
            native_odom_rate_hz=self.odom_rate.rate_hz(),
            adapter_odom_rate_hz=self.adapter_odom_rate.rate_hz(),
            cloud_rate_hz=self.cloud_rate.rate_hz(),
            livox_input_seen=self.livox_input_seen,
            imu_input_seen=self.imu_input_seen,
            dependency_blocker="",
            primary_enabled=self.publish_primary,
        )
        payload = status.to_payload()
        payload["last_pose_yaw_source"] = "native_point_lio_quaternion"
        payload["odom_jump_guard_enabled"] = bool(self.odom_jump_guard_enabled)
        payload["rejected_odom_jumps"] = int(self.rejected_odom_jumps)
        payload["map_bounds_guard_enabled"] = bool(self.map_bounds_guard_enabled)
        payload["rejected_odom_bounds"] = int(self.rejected_odom_bounds)
        payload["last_reject_reason"] = str(self._last_reject_reason)
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run point_lio_ros2_adapter_node")
    rclpy.init(args=args)
    node = PointLioRos2AdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
