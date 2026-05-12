#!/usr/bin/env python3
"""ROS 1 side of the explicit Point-LIO runtime bridge.

This process runs inside the Point-LIO Docker container. It receives ROS 2
sensor messages over a local TCP socket and republishes them into the ROS 1
Point-LIO graph; it also forwards Point-LIO odometry/cloud outputs back to
the host ROS 2 bridge.
"""
from __future__ import annotations

import argparse
import base64
import json
import socket
import threading
import time
from typing import Any

import rospy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header


def _stamp_to_dict(stamp: rospy.Time) -> dict[str, int]:
    return {"sec": int(stamp.secs), "nanosec": int(stamp.nsecs)}


def _stamp_from_dict(payload: dict[str, Any]) -> rospy.Time:
    return rospy.Time(int(payload.get("sec", 0)), int(payload.get("nanosec", 0)))


def _header_to_dict(header: Header) -> dict[str, Any]:
    return {
        "stamp": _stamp_to_dict(header.stamp),
        "frame_id": str(header.frame_id or ""),
    }


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


def _point_to_dict(p: Point) -> dict[str, float]:
    return {"x": float(p.x), "y": float(p.y), "z": float(p.z)}


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


def _imu_from_dict(payload: dict[str, Any]) -> Imu:
    msg = Imu()
    msg.header = _header_from_dict(payload.get("header", {}))
    msg.orientation = _quat_from_dict(payload.get("orientation", {}))
    msg.orientation_covariance = [float(v) for v in payload.get("orientation_covariance", [0.0] * 9)]
    msg.angular_velocity = _vector_from_dict(payload.get("angular_velocity", {}))
    msg.angular_velocity_covariance = [
        float(v) for v in payload.get("angular_velocity_covariance", [0.0] * 9)
    ]
    msg.linear_acceleration = _vector_from_dict(payload.get("linear_acceleration", {}))
    msg.linear_acceleration_covariance = [
        float(v) for v in payload.get("linear_acceleration_covariance", [0.0] * 9)
    ]
    return msg


def _odom_to_dict(msg: Odometry) -> dict[str, Any]:
    return {
        "type": "odometry",
        "header": _header_to_dict(msg.header),
        "child_frame_id": str(msg.child_frame_id or ""),
        "pose": {
            "position": _point_to_dict(msg.pose.pose.position),
            "orientation": _quat_to_dict(msg.pose.pose.orientation),
            "covariance": [float(v) for v in msg.pose.covariance],
        },
        "twist": {
            "linear": _vector_to_dict(msg.twist.twist.linear),
            "angular": _vector_to_dict(msg.twist.twist.angular),
            "covariance": [float(v) for v in msg.twist.covariance],
        },
    }


class PointLioRos1SocketBridge:
    def __init__(self, port: int) -> None:
        self.port = int(port)
        self.client: socket.socket | None = None
        self.client_lock = threading.Lock()
        self.cloud_pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=5)
        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=100)
        self.odom_sub = rospy.Subscriber("/aft_mapped_to_init", Odometry, self._on_odom, queue_size=20)
        self.cloud_sub = rospy.Subscriber("/cloud_registered_body", PointCloud2, self._on_cloud, queue_size=5)
        self.tx_count = 0
        self.rx_count = 0

    def serve(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("127.0.0.1", self.port))
        server.listen(1)
        rospy.loginfo("Point-LIO ROS1 socket bridge listening on 127.0.0.1:%d", self.port)
        while not rospy.is_shutdown():
            conn, addr = server.accept()
            rospy.loginfo("Point-LIO ROS1 socket bridge accepted %s", addr)
            with self.client_lock:
                if self.client is not None:
                    try:
                        self.client.close()
                    except OSError:
                        pass
                self.client = conn
            try:
                self._read_loop(conn)
            finally:
                with self.client_lock:
                    if self.client is conn:
                        self.client = None
                try:
                    conn.close()
                except OSError:
                    pass

    def _read_loop(self, conn: socket.socket) -> None:
        stream = conn.makefile("rb")
        while not rospy.is_shutdown():
            line = stream.readline()
            if not line:
                return
            try:
                payload = json.loads(line.decode("utf-8"))
                kind = payload.get("type")
                if kind == "pointcloud2":
                    self.cloud_pub.publish(_cloud_from_dict(payload))
                elif kind == "imu":
                    self.imu_pub.publish(_imu_from_dict(payload))
                self.rx_count += 1
            except Exception as exc:  # noqa: BLE001 - keep bridge alive for runtime validation.
                rospy.logwarn("Point-LIO ROS1 bridge decode failed: %s", exc)

    def _send(self, payload: dict[str, Any]) -> None:
        data = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        with self.client_lock:
            client = self.client
            if client is None:
                return
            try:
                client.sendall(data)
                self.tx_count += 1
            except OSError:
                try:
                    client.close()
                except OSError:
                    pass
                if self.client is client:
                    self.client = None

    def _on_odom(self, msg: Odometry) -> None:
        self._send(_odom_to_dict(msg))

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._send(_cloud_to_dict(msg))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    args = parser.parse_args()
    rospy.init_node("point_lio_ros1_socket_bridge", anonymous=False)
    bridge = PointLioRos1SocketBridge(port=args.port)
    thread = threading.Thread(target=bridge.serve, daemon=True)
    thread.start()
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        rospy.loginfo_throttle(
            5.0,
            "Point-LIO ROS1 bridge rx=%d tx=%d client=%s",
            bridge.rx_count,
            bridge.tx_count,
            bridge.client is not None,
        )
        rate.sleep()


if __name__ == "__main__":
    main()
