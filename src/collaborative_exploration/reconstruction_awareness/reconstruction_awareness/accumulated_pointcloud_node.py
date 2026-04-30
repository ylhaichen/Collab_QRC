#!/usr/bin/env python3
"""Accumulated point-cloud node (proposal §10.1, output 1+2).

Subscribes per-robot `/<ns>/registered_scan_map` PointCloud2 streams
(map frame, published by Fast-LIO + pointcloud_frame_bridge), voxel-
downsamples online, keeps a global accumulated voxel set, and:

- publishes /cfpa2/accumulated_cloud      (PointCloud2, 1 Hz)
- publishes /cfpa2/reconstruction_voxels  (alias of accumulated_cloud,
                                           fed into scene_graph 3D
                                           bbox computation)
- on shutdown saves accumulated_cloud.pcd + .ply + JSON summary

Voxel resolution is independent of reconstruction_quality_node's
quality grid; the goal here is "smallest reconstruction we can keep
in memory and dump as a paper figure", not view-diversity scoring.

Frame: every input cloud must already be in map frame (the frame_id is
verified each tick; clouds with a different frame_id are dropped with
a logged warning).
"""
from __future__ import annotations

import json
import math
import os
import struct
import time
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .common import atomic_write_json, now_sec_from_node


def _iter_xyz(cloud: PointCloud2):
    fields = {f.name: f for f in cloud.fields}
    if "x" not in fields or "y" not in fields or "z" not in fields:
        return
    fx, fy, fz = fields["x"], fields["y"], fields["z"]
    point_step = cloud.point_step
    data = bytes(cloud.data)
    n = cloud.width * cloud.height
    if (
        fx.offset == 0 and fy.offset == 4 and fz.offset == 8
        and fx.datatype == 7 and fy.datatype == 7 and fz.datatype == 7
        and point_step >= 12
        and len(data) >= n * point_step
    ):
        for i in range(n):
            base = i * point_step
            x, y, z = struct.unpack_from("<fff", data, base)
            yield x, y, z
        return
    for i in range(n):
        base = i * point_step
        x = struct.unpack_from("<f", data, base + fx.offset)[0]
        y = struct.unpack_from("<f", data, base + fy.offset)[0]
        z = struct.unpack_from("<f", data, base + fz.offset)[0]
        yield x, y, z


class AccumulatedPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("accumulated_pointcloud_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("voxel_size_m", 0.10)
        self.declare_parameter("z_min_m", 0.05)
        self.declare_parameter("z_max_m", 1.40)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("expected_frame", "map")
        self.declare_parameter("output_dir", "")           # save .pcd / .ply / summary here
        self.declare_parameter("output_basename", "accumulated_cloud")
        self.declare_parameter("ply_enabled", True)
        # Memory cap: stop adding new voxels past this many unique
        # cells so the node survives long trials.
        self.declare_parameter("max_voxels", 1_000_000)

        nss = self.get_parameter("namespaces").value or []
        self.namespaces = [str(n).strip("/") for n in nss if str(n).strip("/")]
        self.voxel = max(0.02, float(self.get_parameter("voxel_size_m").value))
        self.z_min = float(self.get_parameter("z_min_m").value)
        self.z_max = float(self.get_parameter("z_max_m").value)
        self.expected_frame = str(self.get_parameter("expected_frame").value).strip()
        self.output_dir = str(self.get_parameter("output_dir").value).strip()
        self.output_basename = str(self.get_parameter("output_basename").value).strip()
        self.ply_enabled = bool(self.get_parameter("ply_enabled").value)
        self.max_voxels = max(1, int(self.get_parameter("max_voxels").value))
        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))

        # Voxel state. Key = (ix, iy, iz). Value = {count_total, per_robot{ns:int}}.
        self._voxels: dict[tuple[int, int, int], dict[str, Any]] = {}
        self._raw_points_seen = 0
        self._dropped_wrong_frame = 0
        self._dropped_z_filter = 0
        self._unique_added = 0
        self._per_robot_points: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self._first_t = time.time()

        for ns in self.namespaces:
            self.create_subscription(
                PointCloud2,
                f"/{ns}/registered_scan_map",
                self._make_cloud_cb(ns),
                10,
            )

        self._cloud_pub = self.create_publisher(PointCloud2, "/cfpa2/accumulated_cloud", 5)
        self._voxel_pub = self.create_publisher(PointCloud2, "/cfpa2/reconstruction_voxels", 5)
        self._summary_pub = self.create_publisher(String, "/cfpa2/accumulated_cloud_summary", 5)
        self._timer = self.create_timer(1.0 / rate, self._tick)

        if self.output_dir:
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            f"accumulated_pointcloud_node up: voxel={self.voxel:.2f}m z=[{self.z_min:.2f},{self.z_max:.2f}]m "
            f"namespaces={self.namespaces} expected_frame={self.expected_frame} "
            f"output_dir={self.output_dir or 'none'}"
        )

    # ── ingest ────────────────────────────────────────────────────────

    def _make_cloud_cb(self, ns: str):
        def _cb(msg: PointCloud2) -> None:
            self._ingest_cloud(ns, msg)
        return _cb

    def _ingest_cloud(self, ns: str, msg: PointCloud2) -> None:
        if self.expected_frame and msg.header.frame_id != self.expected_frame:
            self._dropped_wrong_frame += 1
            if self._dropped_wrong_frame in (1, 100, 1000):
                self.get_logger().warn(
                    f"{ns} cloud frame={msg.header.frame_id!r} != expected "
                    f"{self.expected_frame!r}; dropping (count={self._dropped_wrong_frame})"
                )
            return
        added_unique = 0
        added_total = 0
        for x, y, z in _iter_xyz(msg):
            self._raw_points_seen += 1
            if z < self.z_min or z > self.z_max:
                self._dropped_z_filter += 1
                continue
            ix = int(math.floor(x / self.voxel))
            iy = int(math.floor(y / self.voxel))
            iz = int(math.floor(z / self.voxel))
            key = (ix, iy, iz)
            entry = self._voxels.get(key)
            if entry is None:
                if len(self._voxels) >= self.max_voxels:
                    continue  # cap reached; drop new voxels (existing ones still update count)
                entry = {"count": 0, "per_robot": {}, "centroid": [0.0, 0.0, 0.0]}
                self._voxels[key] = entry
                added_unique += 1
            entry["count"] += 1
            entry["per_robot"][ns] = entry["per_robot"].get(ns, 0) + 1
            # Running sum for centroid; output average on save.
            entry["centroid"][0] += x
            entry["centroid"][1] += y
            entry["centroid"][2] += z
            added_total += 1
        self._unique_added += added_unique
        self._per_robot_points[ns] = self._per_robot_points.get(ns, 0) + added_total

    # ── publish ──────────────────────────────────────────────────────

    def _make_xyz_cloud(self, frame: str = "map") -> PointCloud2:
        msg = PointCloud2()
        msg.header.frame_id = frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(self._voxels)
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        buf = bytearray(msg.row_step)
        for i, (key, entry) in enumerate(self._voxels.items()):
            ix, iy, iz = key
            cnt = max(1, entry["count"])
            cx = entry["centroid"][0] / cnt
            cy = entry["centroid"][1] / cnt
            cz = entry["centroid"][2] / cnt
            struct.pack_into("<fff", buf, i * msg.point_step, float(cx), float(cy), float(cz))
        msg.data = bytes(buf)
        return msg

    def _summary(self) -> dict[str, Any]:
        return {
            "stamp_sec": now_sec_from_node(self),
            "schema": "accumulated_cloud_summary/v1",
            "voxel_size_m": round(self.voxel, 3),
            "z_filter_range_m": [round(self.z_min, 3), round(self.z_max, 3)],
            "raw_points_seen": int(self._raw_points_seen),
            "occupied_voxels": int(len(self._voxels)),
            "voxels_added_unique": int(self._unique_added),
            "dropped_wrong_frame": int(self._dropped_wrong_frame),
            "dropped_z_filter": int(self._dropped_z_filter),
            "per_robot_point_count": dict(self._per_robot_points),
            "namespaces": list(self.namespaces),
            "elapsed_wall_sec": round(time.time() - self._first_t, 3),
            "max_voxels_cap": int(self.max_voxels),
            "max_voxels_reached": bool(len(self._voxels) >= self.max_voxels),
        }

    def _tick(self) -> None:
        if not self._voxels:
            # Still publish an empty summary so downstream nodes know we are alive.
            sm = String()
            sm.data = json.dumps(self._summary(), separators=(",", ":"))
            self._summary_pub.publish(sm)
            return
        cloud = self._make_xyz_cloud()
        self._cloud_pub.publish(cloud)
        self._voxel_pub.publish(cloud)  # alias topic
        sm = String()
        sm.data = json.dumps(self._summary(), separators=(",", ":"))
        self._summary_pub.publish(sm)

    # ── shutdown / persistence ───────────────────────────────────────

    def save_artifacts(self) -> dict[str, Any]:
        if not self.output_dir:
            self.get_logger().info("accumulated_pointcloud_node: output_dir empty; not saving")
            return self._summary()
        out_dir = Path(self.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        pcd_path = out_dir / f"{self.output_basename}.pcd"
        ply_path = out_dir / f"{self.output_basename}.ply"
        summary_path = out_dir / f"{self.output_basename}_summary.json"
        # Compute centroids once, reuse for both writers.
        points: list[tuple[float, float, float]] = []
        for entry in self._voxels.values():
            cnt = max(1, entry["count"])
            cx = entry["centroid"][0] / cnt
            cy = entry["centroid"][1] / cnt
            cz = entry["centroid"][2] / cnt
            points.append((float(cx), float(cy), float(cz)))
        n = len(points)
        # PCD ASCII per spec v0.7.
        try:
            with pcd_path.open("w", encoding="utf-8") as fh:
                fh.write(
                    "# .PCD v0.7 - Point Cloud Data file format\n"
                    "VERSION 0.7\n"
                    "FIELDS x y z\n"
                    "SIZE 4 4 4\n"
                    "TYPE F F F\n"
                    "COUNT 1 1 1\n"
                    f"WIDTH {n}\nHEIGHT 1\n"
                    "VIEWPOINT 0 0 0 1 0 0 0\n"
                    f"POINTS {n}\nDATA ascii\n"
                )
                for x, y, z in points:
                    fh.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
        except OSError as exc:
            self.get_logger().warn(f"failed to write {pcd_path}: {exc}")
        # PLY ASCII (optional, easier for paper figures).
        if self.ply_enabled:
            try:
                with ply_path.open("w", encoding="utf-8") as fh:
                    fh.write(
                        "ply\nformat ascii 1.0\n"
                        f"element vertex {n}\n"
                        "property float x\nproperty float y\nproperty float z\n"
                        "end_header\n"
                    )
                    for x, y, z in points:
                        fh.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
            except OSError as exc:
                self.get_logger().warn(f"failed to write {ply_path}: {exc}")
        summary = self._summary()
        summary["pcd_path"] = str(pcd_path)
        if self.ply_enabled:
            summary["ply_path"] = str(ply_path)
        summary["summary_path"] = str(summary_path)
        atomic_write_json(str(summary_path), summary)
        self.get_logger().info(
            f"accumulated_pointcloud_node: saved {n} voxels → {pcd_path}, "
            f"summary → {summary_path}"
        )
        return summary


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AccumulatedPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:  # pragma: no cover
        node.get_logger().exception("accumulated_pointcloud_node crashed")
    finally:
        try:
            node.save_artifacts()
        except Exception:  # pragma: no cover
            node.get_logger().exception("save_artifacts failed")
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
