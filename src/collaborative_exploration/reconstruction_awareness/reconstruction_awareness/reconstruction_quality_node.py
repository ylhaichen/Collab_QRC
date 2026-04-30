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
    """3D voxel reconstruction-quality tracker.

    Voxel key is (ix, iy, iz). Per voxel:
        point_count           int
        source_robot_ids      set[str]
        yaw_bins              set[(ns, yaw_bin)]
        elev_bins             set[(ns, elev_bin)]
        first_seen_time       float
        last_seen_time        float
        sum_x, sum_y, sum_z   float (running sum for centroid_xyz)
        reconstruction_score  float in [0, 1]   (computed lazily)
        low_quality           bool              (computed lazily)
    """

    def __init__(self) -> None:
        super().__init__("reconstruction_quality_node")
        # Voxel grid params (3D).
        self.declare_parameter("xy_resolution_m", 0.30)
        self.declare_parameter("z_resolution_m", 0.20)
        # Backward-compat: old single-param name; if set, overrides both.
        self.declare_parameter("voxel_size_m", 0.0)
        self.declare_parameter("z_min_m", 0.10)
        self.declare_parameter("z_max_m", 1.20)
        self.declare_parameter("cloud_subsample_stride", 4)
        self.declare_parameter("yaw_bin_deg", 45.0)
        self.declare_parameter("elev_bin_deg", 30.0)
        # Quality thresholds.
        self.declare_parameter("density_target_pts", 20)
        self.declare_parameter("view_diversity_target", 3)
        self.declare_parameter("low_quality_score_threshold", 0.5)
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
        self.declare_parameter("voxels_output_path", "")
        self.declare_parameter("summary_output_path", "")
        # Cap how many full voxel records get serialised in
        # reconstruction_voxels.json — full grids hit 30 k+ entries on
        # demo3 and the json read time dominates mission_summary.
        self.declare_parameter("max_voxels_serialised", 8000)

        legacy = float(self.get_parameter("voxel_size_m").value)
        self.xy_voxel = max(0.05, float(self.get_parameter("xy_resolution_m").value))
        self.z_voxel = max(0.05, float(self.get_parameter("z_resolution_m").value))
        if legacy > 0.0:
            # Honour the old 2D param if explicitly set; same value for z.
            self.xy_voxel = legacy
            self.z_voxel = legacy
        self.z_min = float(self.get_parameter("z_min_m").value)
        self.z_max = float(self.get_parameter("z_max_m").value)
        self.subsample = max(1, int(self.get_parameter("cloud_subsample_stride").value))
        self.yaw_bin = max(5.0, float(self.get_parameter("yaw_bin_deg").value))
        self.elev_bin = max(5.0, float(self.get_parameter("elev_bin_deg").value))
        self.density_target = max(1, int(self.get_parameter("density_target_pts").value))
        self.view_target = max(1, int(self.get_parameter("view_diversity_target").value))
        self.low_quality_threshold = max(
            0.0, min(1.0, float(self.get_parameter("low_quality_score_threshold").value))
        )
        self.max_candidates = max(1, int(self.get_parameter("max_candidates").value))
        self.cand_sep = max(self.xy_voxel, float(self.get_parameter("candidate_min_separation_m").value))
        self.cand_min_dist = max(0.0, float(self.get_parameter("candidate_min_distance_from_robot_m").value))
        self.cand_max_dist = max(self.cand_min_dist + 0.1, float(self.get_parameter("candidate_max_distance_from_robot_m").value))
        rate = max(0.05, float(self.get_parameter("publish_rate_hz").value))
        nss = self.get_parameter("namespaces").value or []
        self.namespaces = [str(n).strip("/") for n in nss if str(n).strip("/")]
        self.map_topic = str(self.get_parameter("map_topic").value).strip()
        self.output_path = str(self.get_parameter("output_path").value).strip()
        self.voxels_output_path = str(self.get_parameter("voxels_output_path").value).strip()
        self.summary_output_path = str(self.get_parameter("summary_output_path").value).strip()
        self.max_voxels_serialised = max(0, int(self.get_parameter("max_voxels_serialised").value))

        # Per-voxel state. Key = (ix, iy, iz).
        self._voxels: dict[tuple[int, int, int], dict[str, Any]] = {}
        self._cloud_count = 0
        self._point_count = 0
        self._per_robot_points: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self._z_seen_min = float("inf")
        self._z_seen_max = float("-inf")

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
            f"reconstruction_quality_node up: xy={self.xy_voxel:.2f}m z={self.z_voxel:.2f}m "
            f"z_range=[{self.z_min:.2f},{self.z_max:.2f}]m "
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
            # Track full (x, y, z, yaw); z used for elevation-bin computation.
            self._poses[ns] = (float(p.x), float(p.y), float(yaw), float(p.z))
        return _cb

    def _make_cloud_cb(self, ns: str):
        def _cb(msg: PointCloud2) -> None:
            self._ingest_cloud(ns, msg)
        return _cb

    def _ingest_cloud(self, ns: str, msg: PointCloud2) -> None:
        pose = self._poses.get(ns)
        if pose is None:
            return
        rx, ry, ryaw, rz = pose if len(pose) == 4 else (pose[0], pose[1], pose[2], 0.40)
        yaw_slots = max(1, int(360.0 // self.yaw_bin))
        yaw_view_bin = int(math.degrees(ryaw) // self.yaw_bin) % yaw_slots
        ns_yaw_bin = (ns, yaw_view_bin)
        now = now_sec_from_node(self)
        added = 0
        per_robot_added = 0
        z_min_local = float("inf")
        z_max_local = float("-inf")
        for i, (x, y, z) in enumerate(_iter_xyz(msg)):
            if (i % self.subsample) != 0:
                continue
            if z < self.z_min or z > self.z_max:
                continue
            ix = int(math.floor(x / self.xy_voxel))
            iy = int(math.floor(y / self.xy_voxel))
            iz = int(math.floor(z / self.z_voxel))
            entry = self._voxels.get((ix, iy, iz))
            if entry is None:
                entry = {
                    "count": 0,
                    "source_robot_ids": set(),
                    "yaw_bins": set(),
                    "elev_bins": set(),
                    "first_t": now,
                    "last_t": now,
                    "sum_x": 0.0,
                    "sum_y": 0.0,
                    "sum_z": 0.0,
                }
                self._voxels[(ix, iy, iz)] = entry
            entry["count"] += 1
            entry["source_robot_ids"].add(ns)
            entry["yaw_bins"].add(ns_yaw_bin)
            # Elevation bin: angle from robot pose to voxel center.
            vcx = (ix + 0.5) * self.xy_voxel
            vcy = (iy + 0.5) * self.xy_voxel
            vcz = (iz + 0.5) * self.z_voxel
            horiz = math.hypot(rx - vcx, ry - vcy)
            elev_deg = math.degrees(math.atan2(vcz - rz, max(0.05, horiz)))
            elev_slots = max(1, int(180.0 // self.elev_bin))
            elev_idx = int((elev_deg + 90.0) // self.elev_bin) % elev_slots
            entry["elev_bins"].add((ns, elev_idx))
            entry["last_t"] = now
            entry["sum_x"] += float(x)
            entry["sum_y"] += float(y)
            entry["sum_z"] += float(z)
            added += 1
            per_robot_added += 1
            if z < z_min_local:
                z_min_local = z
            if z > z_max_local:
                z_max_local = z
        self._cloud_count += 1
        self._point_count += added
        self._per_robot_points[ns] = self._per_robot_points.get(ns, 0) + per_robot_added
        if z_min_local < self._z_seen_min:
            self._z_seen_min = z_min_local
        if z_max_local > self._z_seen_max:
            self._z_seen_max = z_max_local

    # ── candidate selection ──────────────────────────────────────────

    def _voxel_xy_in_known_free(self, ix: int, iy: int) -> bool:
        """Project the (ix, iy) column of voxels onto /merged_map.

        We accept a 3D voxel as candidate-eligible only if its XY
        projection is inside the known-free region of the navigation
        map; this keeps recon goals within the explored area.
        """
        if self._map_msg is None:
            return True
        msg = self._map_msg
        wx = (ix + 0.5) * self.xy_voxel
        wy = (iy + 0.5) * self.xy_voxel
        gx = int(math.floor((wx - msg.info.origin.position.x) / max(1e-6, msg.info.resolution)))
        gy = int(math.floor((wy - msg.info.origin.position.y) / max(1e-6, msg.info.resolution)))
        if gx < 0 or gx >= msg.info.width or gy < 0 or gy >= msg.info.height:
            return False
        v = msg.data[gy * msg.info.width + gx]
        return 0 <= v < 50

    def _voxel_quality(self, entry: dict[str, Any]) -> float:
        """Reconstruction quality score in [0, 1]. Combines density,
        yaw view diversity, and elevation diversity. 1 = saturated."""
        density_q = clamp01(entry["count"] / float(self.density_target))
        yaw_q = clamp01(len(entry["yaw_bins"]) / float(self.view_target))
        # Elev-q: at least 2 distinct elevations per voxel = full diversity.
        elev_q = clamp01(len(entry["elev_bins"]) / 2.0)
        # Combine yaw + elev as average (so a single yaw with 2 elevs ≠ 0).
        view_q = (yaw_q + elev_q) / 2.0
        return math.sqrt(density_q * view_q)

    def _annotate_voxel(self, key: tuple[int, int, int], entry: dict[str, Any]) -> dict[str, Any]:
        """Stamp reconstruction_score + low_quality + centroid on the
        voxel record (computed lazily to keep ingest cheap)."""
        score = self._voxel_quality(entry)
        cx = entry["sum_x"] / max(1, entry["count"])
        cy = entry["sum_y"] / max(1, entry["count"])
        cz = entry["sum_z"] / max(1, entry["count"])
        entry["reconstruction_score"] = score
        entry["low_quality"] = score < self.low_quality_threshold
        entry["centroid_xyz"] = (cx, cy, cz)
        return entry

    def _pick_target_robot(self, wx: float, wy: float) -> str:
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
        scored: list[tuple[float, tuple[int, int, int], dict[str, Any]]] = []
        for key, entry in self._voxels.items():
            ix, iy, iz = key
            if not self._voxel_xy_in_known_free(ix, iy):
                continue
            self._annotate_voxel(key, entry)
            q = entry["reconstruction_score"]
            if q >= 0.95:
                continue
            scored.append((q, key, entry))
        if not scored:
            return []
        scored.sort(key=lambda t: t[0])

        cands: list[dict[str, Any]] = []
        accepted_xy: list[tuple[float, float]] = []
        for q, (ix, iy, iz), entry in scored:
            wx = (ix + 0.5) * self.xy_voxel
            wy = (iy + 0.5) * self.xy_voxel
            wz = (iz + 0.5) * self.z_voxel
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
                "z": round(wz, 3),
                "target_robot": ns_target,
                "recon_gain": round(1.0 - q, 4),
                "view_diversity": len(entry["yaw_bins"]) + len(entry["elev_bins"]),
                "voxel_count": int(entry["count"]),
                "voxel_size_m": round(self.xy_voxel, 3),
                "z_resolution_m": round(self.z_voxel, 3),
                "source_robot_ids": sorted(entry["source_robot_ids"]),
                "kind": "reconstruct",
            })
            accepted_xy.append((wx, wy))
            if len(cands) >= self.max_candidates:
                break
        return cands

    # ── publish ──────────────────────────────────────────────────────

    def _build_voxel_records(self) -> tuple[list[dict[str, Any]], int, int]:
        """Materialise every voxel as a JSON-serialisable record. Returns
        (records, low_quality_count, total_count)."""
        records: list[dict[str, Any]] = []
        low_count = 0
        for key, entry in self._voxels.items():
            ix, iy, iz = key
            self._annotate_voxel(key, entry)
            cx, cy, cz = entry["centroid_xyz"]
            records.append({
                "voxel_key": [int(ix), int(iy), int(iz)],
                "voxel_corner_xyz": [
                    round(ix * self.xy_voxel, 3),
                    round(iy * self.xy_voxel, 3),
                    round(iz * self.z_voxel, 3),
                ],
                "centroid_xyz": [round(cx, 3), round(cy, 3), round(cz, 3)],
                "point_count": int(entry["count"]),
                "source_robot_ids": sorted(entry["source_robot_ids"]),
                "yaw_bin_count": len(entry["yaw_bins"]),
                "elev_bin_count": len(entry["elev_bins"]),
                "first_seen_time": round(entry["first_t"], 3),
                "last_seen_time": round(entry["last_t"], 3),
                "reconstruction_score": round(entry["reconstruction_score"], 4),
                "low_quality": bool(entry["low_quality"]),
            })
            if entry["low_quality"]:
                low_count += 1
        return records, low_count, len(records)

    def _build_summary(self, low_count: int, total: int) -> dict[str, Any]:
        z_min = self._z_seen_min if self._z_seen_min != float("inf") else 0.0
        z_max = self._z_seen_max if self._z_seen_max != float("-inf") else 0.0
        return {
            "stamp_sec": now_sec_from_node(self),
            "schema": "reconstruction_quality_summary/v1",
            "xy_resolution_m": round(self.xy_voxel, 3),
            "z_resolution_m": round(self.z_voxel, 3),
            "z_range_observed_m": [round(z_min, 3), round(z_max, 3)],
            "z_filter_range_m": [round(self.z_min, 3), round(self.z_max, 3)],
            "occupied_3d_voxels": int(total),
            "low_quality_3d_voxels": int(low_count),
            "point_count": int(self._point_count),
            "cloud_count": int(self._cloud_count),
            "density_target_pts": self.density_target,
            "view_diversity_target": self.view_target,
            "low_quality_score_threshold": round(self.low_quality_threshold, 3),
            "per_robot_point_count": dict(self._per_robot_points),
        }

    def _tick(self) -> None:
        cands = self._build_candidates()
        records, low_count, total = self._build_voxel_records()
        summary = self._build_summary(low_count, total)
        # Wire-format payload for /cfpa2/reconstruction_candidates is
        # the candidate stream — same key fields as before so CFPA2 can
        # ingest unchanged. Aggregate metrics piggyback on the same
        # payload for downstream tools.
        payload: dict[str, Any] = dict(summary)
        payload["voxel_count"] = int(total)  # legacy alias for older readers
        payload["voxel_size_m"] = round(self.xy_voxel, 3)  # legacy alias
        payload["candidates"] = cands
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._pub.publish(msg)
        self._publish_markers(cands, records)

        # Per §10.1 deliverables: persist three files (one stays as
        # legacy reconstruction_quality.json for backward compat).
        if self.output_path:
            atomic_write_json(self.output_path, payload)
        if self.summary_output_path:
            atomic_write_json(self.summary_output_path, summary)
        if self.voxels_output_path:
            # Cap the voxel list so the JSON file stays parseable.
            slice_records = records
            if self.max_voxels_serialised and len(records) > self.max_voxels_serialised:
                # Prefer low-quality voxels, then sort by reconstruction
                # score so the cap retains the most actionable ones.
                slice_records = sorted(
                    records,
                    key=lambda r: (not r["low_quality"], r["reconstruction_score"]),
                )[: self.max_voxels_serialised]
            atomic_write_json(
                self.voxels_output_path,
                {
                    "stamp_sec": summary["stamp_sec"],
                    "schema": "reconstruction_voxels/v1",
                    "xy_resolution_m": summary["xy_resolution_m"],
                    "z_resolution_m": summary["z_resolution_m"],
                    "voxel_count": int(total),
                    "low_quality_voxels": int(low_count),
                    "voxels_serialised": len(slice_records),
                    "voxels": slice_records,
                },
            )

    def _publish_markers(
        self,
        cands: list[dict[str, Any]],
        records: list[dict[str, Any]] | None = None,
    ) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        # 3D low-quality voxel cubes (slimmer, transparent, drawn first).
        if records is not None:
            voxel_idx = 0
            for r in records:
                if not r["low_quality"]:
                    continue
                voxel_idx += 1
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "reconstruction_low_quality_voxel"
                m.id = voxel_idx
                m.type = Marker.CUBE
                m.action = Marker.ADD
                cx, cy, cz = r["centroid_xyz"]
                m.pose.position.x = float(cx)
                m.pose.position.y = float(cy)
                m.pose.position.z = float(cz)
                m.pose.orientation.w = 1.0
                m.scale.x = self.xy_voxel
                m.scale.y = self.xy_voxel
                m.scale.z = self.z_voxel
                # Magenta-ish, opaque proportional to (1 - score).
                gain = 1.0 - float(r["reconstruction_score"])
                m.color.r = 0.4 + 0.6 * gain
                m.color.g = 0.1
                m.color.b = 0.7
                m.color.a = 0.35 + 0.45 * gain
                ma.markers.append(m)
        # Candidate markers (large flat discs at goal level).
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
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = max(0.20, self.xy_voxel * 1.2)
            m.scale.y = max(0.20, self.xy_voxel * 1.2)
            m.scale.z = 0.05
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
