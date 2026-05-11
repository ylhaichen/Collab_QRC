from __future__ import annotations

import json
import math
import time
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
except ModuleNotFoundError:  # Pure contract tests can run without ROS 2 sourced.
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[misc,assignment]
    Odometry = Any  # type: ignore[misc,assignment]
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

        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip().strip("/")
        self.mode = str(self.get_parameter("mode").value).strip().lower() or "shadow"
        self.publish_primary = bool(self.get_parameter("publish_primary_contract").value)
        if self.mode == "primary":
            self.publish_primary = True
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.contract = build_point_lio_contract(self.robot_namespace)
        self.odom_rate = _RateCounter()
        self.adapter_odom_rate = _RateCounter()
        self.cloud_rate = _RateCounter()
        self.livox_input_seen = False
        self.imu_input_seen = False

        self.odom_pub = self.create_publisher(Odometry, self.contract.outputs["odometry"], 20)
        self.corrected_pub = self.create_publisher(Odometry, self.contract.outputs["corrected_odom"], 20)
        self.nav_pub = self.create_publisher(Odometry, self.contract.outputs["nav_odom"], 20)
        self.body_cloud_pub = self.create_publisher(PointCloud2, self.contract.outputs["registered_body"], 5)
        self.static_cloud_pub = self.create_publisher(PointCloud2, self.contract.outputs["static_cloud"], 5)
        self.dynamic_cloud_pub = self.create_publisher(PointCloud2, self.contract.outputs["dynamic_cloud"], 5)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf and TransformBroadcaster else None

        self.create_subscription(Odometry, self.contract.native_odom_topic, self._on_native_odom, 20)
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
            f"publish_primary_contract={self.publish_primary}"
        )

    def _on_livox(self, _msg: PointCloud2) -> None:
        self.livox_input_seen = True

    def _on_imu(self, _msg: Imu) -> None:
        self.imu_input_seen = True

    def _retarget_odom(self, msg: Odometry) -> Odometry:
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = f"{self.robot_namespace}/odom"
        out.child_frame_id = f"{self.robot_namespace}/base_link"
        out.pose = msg.pose
        out.twist = msg.twist
        return out

    def _publish_tf(self, odom: Odometry) -> None:
        if self.tf_br is None:
            return
        tf_msg = TransformStamped()
        tf_msg.header = odom.header
        tf_msg.child_frame_id = odom.child_frame_id
        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self.tf_br.sendTransform(tf_msg)

    def _on_native_odom(self, msg: Odometry) -> None:
        self.odom_rate.mark()
        if not self.publish_primary:
            return
        odom = self._retarget_odom(msg)
        self.odom_pub.publish(odom)
        self.corrected_pub.publish(odom)
        self.nav_pub.publish(odom)
        self._publish_tf(odom)
        self.adapter_odom_rate.mark()

    def _retarget_cloud(self, msg: PointCloud2) -> PointCloud2:
        msg.header.frame_id = f"{self.robot_namespace}/base_link"
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
