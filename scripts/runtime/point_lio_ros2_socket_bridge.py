#!/usr/bin/env python3
"""ROS 2 side of the explicit Point-LIO Docker runtime bridge."""
from __future__ import annotations

import argparse
import base64
import json
import socket
import threading
import time
from typing import Any

import math
import numpy as np
import rclpy
from geometry_msgs.msg import Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header, String


def _stamp_to_dict(stamp: Any) -> dict[str, int]:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def _stamp_from_dict(payload: dict[str, Any]) -> Any:
    from builtin_interfaces.msg import Time

    stamp = Time()
    stamp.sec = int(payload.get("sec", 0))
    stamp.nanosec = int(payload.get("nanosec", 0))
    return stamp


def _header_to_dict(header: Header) -> dict[str, Any]:
    return {"stamp": _stamp_to_dict(header.stamp), "frame_id": str(header.frame_id or "")}


def _header_from_dict(payload: dict[str, Any]) -> Header:
    header = Header()
    header.stamp = _stamp_from_dict(payload.get("stamp", {}))
    header.frame_id = str(payload.get("frame_id", ""))
    return header


def _fields_to_dict(fields: list[PointField]) -> list[dict[str, Any]]:
    return [
        {
            "name": str(field.name),
            "offset": int(field.offset),
            "datatype": int(field.datatype),
            "count": int(field.count),
        }
        for field in fields
    ]


def _fields_from_dict(fields: list[dict[str, Any]]) -> list[PointField]:
    out: list[PointField] = []
    for item in fields:
        field = PointField()
        field.name = str(item.get("name", ""))
        field.offset = int(item.get("offset", 0))
        field.datatype = int(item.get("datatype", 0))
        field.count = int(item.get("count", 1))
        out.append(field)
    return out


def _cloud_to_dict(msg: PointCloud2) -> dict[str, Any]:
    return {
        "type": "pointcloud2",
        "header": _header_to_dict(msg.header),
        "height": int(msg.height),
        "width": int(msg.width),
        "fields": _fields_to_dict(msg.fields),
        "is_bigendian": bool(msg.is_bigendian),
        "point_step": int(msg.point_step),
        "row_step": int(msg.row_step),
        "data_b64": base64.b64encode(bytes(msg.data)).decode("ascii"),
        "is_dense": bool(msg.is_dense),
    }


def _cloud_from_dict(payload: dict[str, Any]) -> PointCloud2:
    msg = PointCloud2()
    msg.header = _header_from_dict(payload.get("header", {}))
    msg.height = int(payload.get("height", 1))
    msg.width = int(payload.get("width", 0))
    msg.fields = _fields_from_dict(payload.get("fields", []))
    msg.is_bigendian = bool(payload.get("is_bigendian", False))
    msg.point_step = int(payload.get("point_step", 0))
    msg.row_step = int(payload.get("row_step", 0))
    msg.data = base64.b64decode(str(payload.get("data_b64", "")))
    msg.is_dense = bool(payload.get("is_dense", True))
    return msg


def _vector_to_dict(v: Vector3) -> dict[str, float]:
    return {"x": float(v.x), "y": float(v.y), "z": float(v.z)}


def _vector_from_dict(payload: dict[str, Any]) -> Vector3:
    v = Vector3()
    v.x = float(payload.get("x", 0.0))
    v.y = float(payload.get("y", 0.0))
    v.z = float(payload.get("z", 0.0))
    return v


def _quat_to_dict(q: Quaternion) -> dict[str, float]:
    return {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)}


def _quat_from_dict(payload: dict[str, Any]) -> Quaternion:
    q = Quaternion()
    q.x = float(payload.get("x", 0.0))
    q.y = float(payload.get("y", 0.0))
    q.z = float(payload.get("z", 0.0))
    q.w = float(payload.get("w", 1.0))
    return q


def _point_from_dict(payload: dict[str, Any]) -> Point:
    p = Point()
    p.x = float(payload.get("x", 0.0))
    p.y = float(payload.get("y", 0.0))
    p.z = float(payload.get("z", 0.0))
    return p


def _imu_to_dict(msg: Imu) -> dict[str, Any]:
    return {
        "type": "imu",
        "header": _header_to_dict(msg.header),
        "orientation": _quat_to_dict(msg.orientation),
        "orientation_covariance": [float(v) for v in msg.orientation_covariance],
        "angular_velocity": _vector_to_dict(msg.angular_velocity),
        "angular_velocity_covariance": [float(v) for v in msg.angular_velocity_covariance],
        "linear_acceleration": _vector_to_dict(msg.linear_acceleration),
        "linear_acceleration_covariance": [float(v) for v in msg.linear_acceleration_covariance],
    }


def _odom_from_dict(payload: dict[str, Any], fallback_frame: str, fallback_child: str) -> Odometry:
    msg = Odometry()
    msg.header = _header_from_dict(payload.get("header", {}))
    if not msg.header.frame_id:
        msg.header.frame_id = fallback_frame
    msg.child_frame_id = str(payload.get("child_frame_id", "")) or fallback_child
    pose = payload.get("pose", {})
    twist = payload.get("twist", {})
    msg.pose.pose.position = _point_from_dict(pose.get("position", {}))
    msg.pose.pose.orientation = _quat_from_dict(pose.get("orientation", {}))
    msg.pose.covariance = [float(v) for v in pose.get("covariance", [0.0] * 36)]
    msg.twist.twist.linear = _vector_from_dict(twist.get("linear", {}))
    msg.twist.twist.angular = _vector_from_dict(twist.get("angular", {}))
    msg.twist.covariance = [float(v) for v in twist.get("covariance", [0.0] * 36)]
    return msg


class PointLioRos2SocketBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__(f"{args.robot_namespace}_point_lio_ros2_socket_bridge")
        self.robot_namespace = args.robot_namespace.strip().strip("/")
        self.host = args.host
        self.port = int(args.port)
        self.max_cloud_hz = float(args.max_cloud_hz)
        self.adapt_mid360 = bool(args.adapt_mid360)
        self.num_rings = int(args.num_rings)
        self.min_vert_angle = math.radians(float(args.min_vert_angle_deg))
        self.max_vert_angle = math.radians(float(args.max_vert_angle_deg))
        self.ring_step = (self.max_vert_angle - self.min_vert_angle) / max(self.num_rings - 1, 1)
        self.last_cloud_sent = 0.0
        self.socket: socket.socket | None = None
        self.socket_lock = threading.Lock()
        self.rx_count = 0
        self.tx_count = 0
        self.connected = False

        qos_cloud = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT if self.adapt_mid360 else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, args.cloud_topic, self._on_cloud, qos_cloud)
        self.create_subscription(Imu, args.imu_topic, self._on_imu, 100)
        self.odom_pub = self.create_publisher(Odometry, f"/{self.robot_namespace}/point_lio/Odometry", 20)
        self.cloud_pub = self.create_publisher(
            PointCloud2, f"/{self.robot_namespace}/point_lio/cloud_registered_body", 5
        )
        self.status_pub = self.create_publisher(String, f"/{self.robot_namespace}/point_lio/socket_bridge_status", 10)
        self.create_timer(1.0, self._publish_status)
        self.reader_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.reader_thread.start()
        self.get_logger().info(
            f"Point-LIO ROS2 socket bridge robot={self.robot_namespace} "
            f"cloud={args.cloud_topic} imu={args.imu_topic} target={self.host}:{self.port}"
        )

    def _connection_loop(self) -> None:
        while rclpy.ok():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=2.0)
                sock.settimeout(None)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self.socket_lock:
                    self.socket = sock
                    self.connected = True
                self.get_logger().info(f"Connected to Point-LIO ROS1 bridge at {self.host}:{self.port}")
                self._read_loop(sock)
            except OSError as exc:
                self.get_logger().warn(f"Point-LIO bridge connect/read failed: {exc}")
                time.sleep(1.0)
            finally:
                with self.socket_lock:
                    if self.socket is not None:
                        try:
                            self.socket.close()
                        except OSError:
                            pass
                    self.socket = None
                    self.connected = False

    def _read_loop(self, sock: socket.socket) -> None:
        stream = sock.makefile("rb")
        while rclpy.ok():
            line = stream.readline()
            if not line:
                return
            try:
                payload = json.loads(line.decode("utf-8"))
                kind = payload.get("type")
                if kind == "odometry":
                    msg = _odom_from_dict(
                        payload,
                        fallback_frame=f"{self.robot_namespace}/point_lio_map",
                        fallback_child=f"{self.robot_namespace}/point_lio_body",
                    )
                    self.odom_pub.publish(msg)
                elif kind == "pointcloud2":
                    msg = _cloud_from_dict(payload)
                    if not msg.header.frame_id:
                        msg.header.frame_id = f"{self.robot_namespace}/point_lio_body"
                    self.cloud_pub.publish(msg)
                self.rx_count += 1
            except Exception as exc:  # noqa: BLE001 - runtime bridge should stay up.
                self.get_logger().warn(f"Point-LIO ROS2 bridge decode failed: {exc}")

    def _send(self, payload: dict[str, Any]) -> None:
        data = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        with self.socket_lock:
            sock = self.socket
            if sock is None:
                return
            try:
                sock.sendall(data)
                self.tx_count += 1
            except OSError:
                try:
                    sock.close()
                except OSError:
                    pass
                if self.socket is sock:
                    self.socket = None
                    self.connected = False

    def _on_cloud(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if self.max_cloud_hz > 0.0 and now - self.last_cloud_sent < 1.0 / self.max_cloud_hz:
            return
        self.last_cloud_sent = now
        if self.adapt_mid360:
            msg = self._adapt_mid360_cloud(msg)
        self._send(_cloud_to_dict(msg))

    def _adapt_mid360_cloud(self, msg: PointCloud2) -> PointCloud2:
        field_map = {f.name: f for f in msg.fields}
        if not {"x", "y", "z"}.issubset(field_map):
            return msg
        n_points = int(msg.width * msg.height)
        if n_points <= 0 or msg.point_step <= 0:
            return msg
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n_points, int(msg.point_step))
        x_off = int(field_map["x"].offset)
        y_off = int(field_map["y"].offset)
        z_off = int(field_map["z"].offset)
        i_off = int(field_map["intensity"].offset) if "intensity" in field_map else None
        x = np.frombuffer(raw[:, x_off:x_off + 4].tobytes(), dtype=np.float32)
        y = np.frombuffer(raw[:, y_off:y_off + 4].tobytes(), dtype=np.float32)
        z = np.frombuffer(raw[:, z_off:z_off + 4].tobytes(), dtype=np.float32)
        if i_off is None:
            intensity = np.zeros(n_points, dtype=np.float32)
        else:
            intensity = np.frombuffer(raw[:, i_off:i_off + 4].tobytes(), dtype=np.float32)
        xy_range = np.sqrt(x * x + y * y)
        vert_angle = np.arctan2(z, np.maximum(xy_range, 1e-6))
        ring_float = np.rint((vert_angle - self.min_vert_angle) / self.ring_step)
        ring = np.clip(ring_float, 0, self.num_rings - 1).astype(np.uint16)
        azimuth = np.arctan2(y, x)
        scan_period_us = 1_000_000.0 / max(self.max_cloud_hz, 1e-3)
        time_offset = (((azimuth + math.pi) / (2.0 * math.pi)) * scan_period_us).astype(np.float32)

        out_point_step = 24
        out_data = np.zeros((n_points, out_point_step), dtype=np.uint8)
        out_data[:, 0:4] = np.frombuffer(x.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 4:8] = np.frombuffer(y.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 8:12] = np.frombuffer(z.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 12:16] = np.frombuffer(intensity.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 16:20] = np.frombuffer(time_offset.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 20:22] = np.frombuffer(ring.tobytes(), dtype=np.uint8).reshape(n_points, 2)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n_points
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
        ]
        out.is_bigendian = False
        out.point_step = out_point_step
        out.row_step = out_point_step * n_points
        out.data = out_data.tobytes()
        out.is_dense = True
        return out

    def _on_imu(self, msg: Imu) -> None:
        self._send(_imu_to_dict(msg))

    def _publish_status(self) -> None:
        payload = {
            "schema": "point_lio_socket_bridge_status/v1",
            "robot_id": self.robot_namespace,
            "connected": bool(self.connected),
            "rx_count": int(self.rx_count),
            "tx_count": int(self.tx_count),
            "gt_used_runtime": False,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-namespace", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--cloud-topic", required=True)
    parser.add_argument("--imu-topic", required=True)
    parser.add_argument("--max-cloud-hz", type=float, default=10.0)
    parser.add_argument("--adapt-mid360", action="store_true")
    parser.add_argument("--num-rings", type=int, default=20)
    parser.add_argument("--min-vert-angle-deg", type=float, default=-7.0)
    parser.add_argument("--max-vert-angle-deg", type=float, default=52.0)
    args = parser.parse_args()
    rclpy.init()
    node = PointLioRos2SocketBridge(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
