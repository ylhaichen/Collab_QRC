#!/usr/bin/env python3
"""Geometry-first reconstruction quality node.

Aggregates per-robot registered point clouds into a 2D voxel grid (XY,
ignoring Z height for navigation purposes) and tracks per-voxel point
count + viewpoint diversity. Voxels that are observed but with low
point density or low view diversity become `role=reconstruct`
candidates for the CFPA2 allocator.

This is the Level-1 (geometry-only) reconstruction quality layer of
the Loop+Risk+Reconstruction proposal §10. Voxel density and view
diversity are computed online; no neural rendering or 3DGS dependency.
A future Level-2 layer can replace the proxies with rendered-view
quality scores; this node's interface (`/cfpa2/reconstruction
_candidates`) mirrors loop_closure_candidate_node so the CFPA2
allocator does not care which proxy is published.

Subscribes
----------
/<ns>/registered_scan_map         (PointCloud2, map frame)
/<ns>/odom/nav                    (Odometry, robot pose for view diversity)
/merged_map                       (OccupancyGrid, scope mask: only score
                                   voxels inside known free area)

Publishes
---------
/cfpa2/reconstruction_candidates  (std_msgs/String JSON,  ~1 Hz)
/cfpa2/reconstruction_quality_markers (visualization_msgs/MarkerArray)

The candidate payload mirrors loop_candidates so CFPA2 can dispatch
either via the same role-utility code path. Each candidate dict:

    {
      "x": ...,
      "y": ...,
      "target_robot": "robot_a" | "robot_b",
      "recon_gain": float in [0,1],          # 1 - density / target
      "view_diversity": int,                  # distinct yaw bins
      "voxel_count": int,                     # raw points in the voxel
      "kind": "reconstruct"
    }
"""
from __future__ import annotations

import json
import math
import struct
from typing import Any

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .common import atomic_write_json, clamp01, now_sec_from_node, yaw_from_quat


def _iter_xyz(cloud: PointCloud2):
    """Yield (x, y, z) tuples from a PointCloud2 message.

    Avoids the optional sensor_msgs_py dependency by parsing the binary
    payload directly. Requires the cloud to have float32 x/y/z fields
    starting at offsets 0/4/8 (the layout fast_lio publishes); falls
    back to a slower struct-unpack scan otherwise.
    """
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
        # Fast path: contiguous float32 xyz at start of point.
        for i in range(n):
            base = i * point_step
            x, y, z = struct.unpack_from("<fff", data, base)
            yield x, y, z
        return
    # Slow path: per-field unpack.
    for i in range(n):
        base = i * point_step
        x = struct.unpack_from("<f", data, base + fx.offset)[0]
        y = struct.unpack_from("<f", data, base + fy.offset)[0]
        z = struct.unpack_from("<f", data, base + fz.offset)[0]
        yield x, y, z


class ReconstructionQualityNode(Node):
    """Track per-voxel point density + view diversity, emit recon goals."""

    def __init__(self) -> None:
        super().__init__("reconstruction_quality_node")
        # Voxel grid params.
        self.declare_parameter("voxel_size_m", 0.30)
        self.declare_parameter("z_min_m", 0.10)
        self.declare_parameter("z_max_m", 1.20)
        self.declare_parameter("cloud_subsample_stride", 4)
        self.declare_parameter("yaw_bin_deg", 45.0)
        # Quality thresholds.
        self.declare_parameter("density_target_pts", 30)
        self.declare_parameter("view_diversity_target", 3)
        # Candidate selection.
        self.declare_parameter("max_candidates", 16)
        self.declare_parameter("candidate_min_separation_m", 0.80)
        self.declare_parameter("candidate_min_distance_from_robot_m", 0.80)
        self.declare_parameter("candidate_max_distance_from_robot_m", 8.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        # Identity / output.
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("map_topic", "/merged_map")
        self.declare_parameter("output_path", "")

        self.voxel = max(0.05, float(self.get_parameter("voxel_size_m").value))
        self.z_min = float(self.get_parameter("z_min_m").value)
        self.z_max = float(self.get_parameter("z_max_m").value)
        self.subsample = max(1, int(self.get_parameter("cloud_subsample_stride").value))
        self.yaw_bin = max(5.0, float(self.get_parameter("yaw_bin_deg").value))
        self.density_target = max(1, int(self.get_parameter("density_target_pts").value))
        self.view_target = max(1, int(self.get_parameter("view_diversity_target").value))
        self.max_candidates = max(1, int(self.get_parameter("max_candidates").value))
        self.cand_sep = max(self.voxel, float(self.get_parameter("candidate_min_separation_m").value))
        self.cand_min_dist = max(0.0, float(self.get_parameter("candidate_min_distance_from_robot_m").value))
        self.cand_max_dist = max(self.cand_min_dist + 0.1, float(self.get_parameter("candidate_max_distance_from_robot_m").value))
        rate = max(0.05, float(self.get_parameter("publish_rate_hz").value))
        nss = self.get_parameter("namespaces").value or []
        self.namespaces = [str(n).strip("/") for n in nss if str(n).strip("/")]
        self.map_topic = str(self.get_parameter("map_topic").value).strip()
        self.output_path = str(self.get_parameter("output_path").value).strip()

        # Per-voxel state. Key = (ix, iy). Value = {count:int, yaw_bins:set, last_t:float}.
        self._voxels: dict[tuple[int, int], dict[str, Any]] = {}
        self._cloud_count = 0
        self._point_count = 0

        # Per-namespace pose for view-diversity binning + candidate routing.
        self._poses: dict[str, tuple[float, float, float]] = {}

        # /merged_map (or fallback /robot_a/map) acts as a scope mask:
        # only score voxels that fall inside known-free cells, so we
        # don't try to "reconstruct" outside the explored area.
        self._map_msg: OccupancyGrid | None = None
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)

        # Per-robot point cloud + odom subscribers.
        for ns in self.namespaces:
            self.create_subscription(
                PointCloud2,
                f"/{ns}/registered_scan_map",
                self._make_cloud_cb(ns),
                10,
            )
            self.create_subscription(
                Odometry,
                f"/{ns}/odom/nav",
                self._make_odom_cb(ns),
                10,
            )

        self._pub = self.create_publisher(String, "/cfpa2/reconstruction_candidates", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/cfpa2/reconstruction_quality_markers", 10
        )
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"reconstruction_quality_node up: voxel={self.voxel:.2f}m z=[{self.z_min:.2f},{self.z_max:.2f}]m "
            f"target_density={self.density_target}pts view_target={self.view_target} "
            f"map_topic={self.map_topic} output={self.output_path or 'none'}"
        )

    # ── subscriber callbacks ──────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_msg = msg

    def _make_odom_cb(self, ns: str):
        def _cb(msg: Odometry) -> None:
            try:
                p = msg.pose.pose.position
                yaw = yaw_from_quat(msg.pose.pose.orientation)
            except Exception:  # pragma: no cover
                return
            self._poses[ns] = (float(p.x), float(p.y), float(yaw))
        return _cb

    def _make_cloud_cb(self, ns: str):
        def _cb(msg: PointCloud2) -> None:
            self._ingest_cloud(ns, msg)
        return _cb

    def _ingest_cloud(self, ns: str, msg: PointCloud2) -> None:
        pose = self._poses.get(ns)
        if pose is None:
            return
        view_yaw_bin = int(math.degrees(pose[2]) // self.yaw_bin) % int(360.0 // self.yaw_bin)
        ns_bin = (ns, view_yaw_bin)  # (robot, yaw_bin) tuple identifies a viewpoint slot
        now = now_sec_from_node(self)
        added = 0
        for i, (x, y, z) in enumerate(_iter_xyz(msg)):
            if (i % self.subsample) != 0:
                continue
            if z < self.z_min or z > self.z_max:
                continue
            ix = int(math.floor(x / self.voxel))
            iy = int(math.floor(y / self.voxel))
            entry = self._voxels.get((ix, iy))
            if entry is None:
                entry = {"count": 0, "yaw_bins": set(), "last_t": now}
                self._voxels[(ix, iy)] = entry
            entry["count"] += 1
            entry["yaw_bins"].add(ns_bin)
            entry["last_t"] = now
            added += 1
        self._cloud_count += 1
        self._point_count += added

    # ── candidate selection ──────────────────────────────────────────

    def _voxel_in_known_free(self, ix: int, iy: int) -> bool:
        if self._map_msg is None:
            return True  # no map yet — accept everything
        msg = self._map_msg
        # Voxel center in world coords.
        wx = (ix + 0.5) * self.voxel
        wy = (iy + 0.5) * self.voxel
        gx = int(math.floor((wx - msg.info.origin.position.x) / max(1e-6, msg.info.resolution)))
        gy = int(math.floor((wy - msg.info.origin.position.y) / max(1e-6, msg.info.resolution)))
        if gx < 0 or gx >= msg.info.width or gy < 0 or gy >= msg.info.height:
            return False
        v = msg.data[gy * msg.info.width + gx]
        # 0 = free, 100 = occupied, -1 = unknown.
        return 0 <= v < 50

    def _voxel_quality(self, entry: dict[str, Any]) -> float:
        """Return reconstruction quality in [0,1]. 1 = perfectly observed."""
        density_q = clamp01(entry["count"] / float(self.density_target))
        view_q = clamp01(len(entry["yaw_bins"]) / float(self.view_target))
        # Geometric mean: a voxel with 100 points from one viewpoint is
        # NOT well reconstructed (single-view bias).
        return math.sqrt(density_q * view_q)

    def _pick_target_robot(self, wx: float, wy: float) -> str:
        # Route candidate to the robot closest to the voxel; falls back
        # to the first ns if no pose data yet.
        best_ns = self.namespaces[0] if self.namespaces else "robot_a"
        best_d = float("inf")
        for ns in self.namespaces:
            p = self._poses.get(ns)
            if p is None:
                continue
            d = math.hypot(wx - p[0], wy - p[1])
            if d < best_d:
                best_d = d
                best_ns = ns
        return best_ns

    def _build_candidates(self) -> list[dict[str, Any]]:
        if not self._voxels:
            return []
        # Score every observed voxel that's inside known-free area.
        scored: list[tuple[float, int, int, dict[str, Any]]] = []
        for (ix, iy), entry in self._voxels.items():
            if not self._voxel_in_known_free(ix, iy):
                continue
            q = self._voxel_quality(entry)
            if q >= 0.95:
                continue  # already well-reconstructed; not a candidate
            scored.append((q, ix, iy, entry))
        if not scored:
            return []
        # Lowest quality first.
        scored.sort(key=lambda t: t[0])

        cands: list[dict[str, Any]] = []
        accepted_xy: list[tuple[float, float]] = []
        for q, ix, iy, entry in scored:
            wx = (ix + 0.5) * self.voxel
            wy = (iy + 0.5) * self.voxel
            # Spatial separation between selected candidates so the list
            # is not 16 cells of the same wall edge.
            too_close = any(
                math.hypot(wx - sx, wy - sy) < self.cand_sep
                for sx, sy in accepted_xy
            )
            if too_close:
                continue
            ns_target = self._pick_target_robot(wx, wy)
            pose = self._poses.get(ns_target)
            if pose is not None:
                d = math.hypot(wx - pose[0], wy - pose[1])
                if d < self.cand_min_dist or d > self.cand_max_dist:
                    continue
            cands.append({
                "x": round(wx, 3),
                "y": round(wy, 3),
                "target_robot": ns_target,
                "recon_gain": round(1.0 - q, 4),
                "view_diversity": len(entry["yaw_bins"]),
                "voxel_count": int(entry["count"]),
                "voxel_size_m": round(self.voxel, 3),
                "kind": "reconstruct",
            })
            accepted_xy.append((wx, wy))
            if len(cands) >= self.max_candidates:
                break
        return cands

    # ── publish ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        cands = self._build_candidates()
        payload: dict[str, Any] = {
            "stamp_sec": now_sec_from_node(self),
            "voxel_size_m": round(self.voxel, 3),
            "voxel_count": len(self._voxels),
            "point_count": self._point_count,
            "cloud_count": self._cloud_count,
            "density_target_pts": self.density_target,
            "view_diversity_target": self.view_target,
            "candidates": cands,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._pub.publish(msg)
        self._publish_markers(cands)
        if self.output_path:
            atomic_write_json(self.output_path, payload)

    def _publish_markers(self, cands: list[dict[str, Any]]) -> None:
        ma = MarkerArray()
        # Clear previous markers.
        clear = Marker()
        clear.header.frame_id = "map"
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, c in enumerate(cands):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "reconstruction_candidates"
            m.id = i + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(c["x"])
            m.pose.position.y = float(c["y"])
            m.pose.position.z = 0.10
            m.pose.orientation.w = 1.0
            m.scale.x = max(0.20, self.voxel * 1.2)
            m.scale.y = max(0.20, self.voxel * 1.2)
            m.scale.z = 0.05
            # Colour by recon_gain: blue (low gain) → magenta (high gain).
            g = float(c.get("recon_gain", 0.0))
            m.color.r = 0.4 + 0.6 * g
            m.color.g = 0.1
            m.color.b = 0.6 + 0.4 * (1.0 - g)
            m.color.a = 0.75
            ma.markers.append(m)
        self._marker_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReconstructionQualityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
