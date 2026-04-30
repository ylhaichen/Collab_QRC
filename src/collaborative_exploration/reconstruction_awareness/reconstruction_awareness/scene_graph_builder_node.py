#!/usr/bin/env python3
"""Geometry-only 3D scene graph builder.

Implements proposal §11 (Scene Graph Layer) Level-1 scaffold from §22
step 10 — but **without VLM/GPU** so it runs on the same laptop the
SLAM stack runs on. Semantic labels (room "kitchen", obstacle
"chair") are deliberately not produced here; that is left to a future
VLM-based enrichment node. What this node does produce is the
graph topology that a VLM enrichment can later annotate:

- room nodes (free-space connected components on /merged_map)
- corridor nodes (rooms with high aspect ratio or small inscribed
  circle radius)
- doorway nodes (narrow passages between adjacent rooms)
- obstacle nodes (occupied connected components inside the explored
  region, with area filter to exclude bounding walls)
- frontier nodes (delegated from CFPA2 loop_candidates)
- low_quality_reconstruction_region nodes (delegated from
  reconstruction_quality candidates)
- robot_failure_event nodes (placeholder; populated by the consumer
  when collision logs are merged into the graph offline)

Edges (proposal §11.1):
- connected_to (room ↔ room via doorway)
- inside (obstacle inside room, low-quality region inside room)
- near (rooms within `near_distance_m`)
- needs_revisit (low_quality region or loop candidate near a room)

Subscribes
----------
/merged_map                              (OccupancyGrid)
/cfpa2/loop_candidates                   (String JSON)
/cfpa2/reconstruction_candidates         (String JSON)

Publishes
---------
/cfpa2/scene_graph                       (std_msgs/String JSON, ~0.5 Hz)
/cfpa2/scene_graph_markers               (visualization_msgs/MarkerArray)

The output JSON shape matches proposal §11.1 (node fields + edge
fields) so a downstream LLM prompt template can address nodes by
node_id and edges by relation.
"""
from __future__ import annotations

import json
import math
import os
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .common import atomic_write_json, now_sec_from_node


# OccupancyGrid convention: 0=free, 100=occupied, -1=unknown.
_FREE = 0
_OCC = 1
_UNK = 2


def _grid_to_classes(msg: OccupancyGrid) -> np.ndarray:
    """Return an HxW uint8 array with 0=free, 1=occupied, 2=unknown."""
    w, h = int(msg.info.width), int(msg.info.height)
    arr = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
    out = np.full((h, w), _UNK, dtype=np.uint8)
    out[arr == 0] = _FREE
    # Anything ≥ 50 in nav2 occupancy convention = occupied.
    out[arr >= 50] = _OCC
    return out


def _grid_xy(ix: int, iy: int, msg: OccupancyGrid) -> tuple[float, float]:
    """Convert (col, row) cell indices to world (x, y) at cell center."""
    res = float(msg.info.resolution)
    ox = float(msg.info.origin.position.x)
    oy = float(msg.info.origin.position.y)
    return ox + (ix + 0.5) * res, oy + (iy + 0.5) * res


class SceneGraphBuilderNode(Node):
    def __init__(self) -> None:
        super().__init__("scene_graph_builder_node")
        # Topology params.
        self.declare_parameter("min_room_area_m2", 1.5)
        self.declare_parameter("corridor_aspect_ratio", 3.0)
        self.declare_parameter("corridor_inscribed_radius_m", 0.55)
        self.declare_parameter("doorway_min_width_m", 0.30)
        self.declare_parameter("doorway_max_width_m", 1.20)
        self.declare_parameter("obstacle_min_area_m2", 0.05)
        self.declare_parameter("obstacle_max_area_m2", 4.0)
        self.declare_parameter("near_distance_m", 1.5)
        # Topics + outputs.
        self.declare_parameter("map_topic", "/merged_map")
        self.declare_parameter("loop_candidates_topic", "/cfpa2/loop_candidates")
        self.declare_parameter(
            "reconstruction_candidates_topic", "/cfpa2/reconstruction_candidates"
        )
        self.declare_parameter("publish_rate_hz", 0.5)
        self.declare_parameter("output_path", "")

        self.min_room_area_m2 = float(self.get_parameter("min_room_area_m2").value)
        self.corridor_aspect_ratio = float(self.get_parameter("corridor_aspect_ratio").value)
        self.corridor_inscribed_radius_m = float(
            self.get_parameter("corridor_inscribed_radius_m").value
        )
        self.doorway_min_width_m = float(self.get_parameter("doorway_min_width_m").value)
        self.doorway_max_width_m = float(self.get_parameter("doorway_max_width_m").value)
        self.obstacle_min_area_m2 = float(self.get_parameter("obstacle_min_area_m2").value)
        self.obstacle_max_area_m2 = float(self.get_parameter("obstacle_max_area_m2").value)
        self.near_distance_m = float(self.get_parameter("near_distance_m").value)
        self.map_topic = str(self.get_parameter("map_topic").value).strip()
        self.loop_topic = str(self.get_parameter("loop_candidates_topic").value).strip()
        self.recon_topic = str(self.get_parameter("reconstruction_candidates_topic").value).strip()
        rate = max(0.05, float(self.get_parameter("publish_rate_hz").value))
        self.output_path = str(self.get_parameter("output_path").value).strip()

        self._map_msg: OccupancyGrid | None = None
        self._loop_payload: dict[str, Any] = {}
        self._recon_payload: dict[str, Any] = {}
        self._first_seen: dict[str, float] = {}

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, self.map_topic, self._on_map, map_qos)
        self.create_subscription(String, self.loop_topic, self._on_loop, 10)
        self.create_subscription(String, self.recon_topic, self._on_recon, 10)

        self._pub = self.create_publisher(String, "/cfpa2/scene_graph", 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/cfpa2/scene_graph_markers", 10
        )
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"scene_graph_builder_node up: map={self.map_topic} "
            f"corridor_ar={self.corridor_aspect_ratio} doorway=[{self.doorway_min_width_m}, "
            f"{self.doorway_max_width_m}]m obstacle=[{self.obstacle_min_area_m2}, "
            f"{self.obstacle_max_area_m2}]m² output={self.output_path or 'none'}"
        )

    # ── subscriber callbacks ──────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_msg = msg

    def _on_loop(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self._loop_payload = payload

    def _on_recon(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self._recon_payload = payload

    # ── geometry analysis ────────────────────────────────────────────

    @staticmethod
    def _label_4connected(mask: np.ndarray) -> tuple[np.ndarray, int]:
        """Return (labels HxW int32, n_components). 4-connectivity flood fill."""
        from scipy.ndimage import label  # local import keeps node import light
        structure = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)
        return label(mask.astype(bool), structure=structure)  # type: ignore[no-any-return]

    @staticmethod
    def _distance_to_wall(free_mask: np.ndarray) -> np.ndarray:
        """Per-cell Euclidean distance (in cells) to nearest non-free cell."""
        from scipy.ndimage import distance_transform_edt
        return distance_transform_edt(free_mask)

    def _build_room_nodes(self, classes: np.ndarray, msg: OccupancyGrid, now_t: float) -> tuple[
        list[dict[str, Any]], np.ndarray
    ]:
        """Return (room nodes, room_label_grid). Label 0 = not a room cell."""
        free = (classes == _FREE)
        labels, n_comp = self._label_4connected(free)
        res = float(msg.info.resolution)
        cell_area = res * res
        kept_labels = np.zeros_like(labels)
        rooms: list[dict[str, Any]] = []
        if n_comp == 0:
            return rooms, kept_labels
        dist = self._distance_to_wall(free)  # in cell units
        new_label = 0
        for old_lbl in range(1, n_comp + 1):
            cells = labels == old_lbl
            area_cells = int(cells.sum())
            area_m2 = area_cells * cell_area
            if area_m2 < self.min_room_area_m2:
                continue
            new_label += 1
            kept_labels[cells] = new_label
            ys, xs = np.where(cells)
            ix_min, ix_max = int(xs.min()), int(xs.max())
            iy_min, iy_max = int(ys.min()), int(ys.max())
            wx_min, wy_min = _grid_xy(ix_min, iy_min, msg)
            wx_max, wy_max = _grid_xy(ix_max, iy_max, msg)
            cx_w, cy_w = _grid_xy(float(xs.mean()), float(ys.mean()), msg)
            cx_w -= 0.5 * res  # mean indices already used 0.5 offset twice
            cy_w -= 0.5 * res
            width_m = (ix_max - ix_min + 1) * res
            height_m = (iy_max - iy_min + 1) * res
            aspect = max(width_m, height_m) / max(0.05, min(width_m, height_m))
            inscribed_cells = float(dist[cells].max())
            inscribed_m = inscribed_cells * res
            is_corridor = (
                aspect >= self.corridor_aspect_ratio
                or inscribed_m < self.corridor_inscribed_radius_m
            )
            node_type = "corridor" if is_corridor else "room"
            node_id = f"{node_type}_{new_label}"
            self._first_seen.setdefault(node_id, now_t)
            rooms.append({
                "node_id": node_id,
                "type": node_type,
                "label": node_id,
                "centroid": {"x": round(cx_w, 3), "y": round(cy_w, 3)},
                "bbox": {
                    "x_min": round(wx_min, 3), "y_min": round(wy_min, 3),
                    "x_max": round(wx_max, 3), "y_max": round(wy_max, 3),
                },
                "area_m2": round(area_m2, 3),
                "aspect_ratio": round(aspect, 2),
                "inscribed_radius_m": round(inscribed_m, 3),
                "confidence": round(min(1.0, area_m2 / 4.0), 3),
                "first_seen_time": self._first_seen[node_id],
                "last_seen_time": now_t,
                "_grid_label": new_label,  # internal use; stripped before publish
            })
        return rooms, kept_labels

    def _detect_doorways(
        self,
        rooms: list[dict[str, Any]],
        room_labels: np.ndarray,
        classes: np.ndarray,
        msg: OccupancyGrid,
        now_t: float,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """Find narrow passages between rooms. Each pair of distinct room
        labels separated only by occupied cells with a narrow gap is a
        doorway. Implementation: dilate each room's free mask by 1 cell;
        where two dilated rooms overlap on occupied cells, that's a
        candidate boundary; cluster overlap pixels and keep clusters
        whose width is in [doorway_min, doorway_max] m.
        """
        if not rooms:
            return [], []
        from scipy.ndimage import binary_dilation, label as nd_label
        res = float(msg.info.resolution)
        occ = (classes == _OCC)
        # For each room, an axis-aligned 3×3 dilation of its free mask.
        struct = np.ones((3, 3), dtype=np.uint8)
        dilated_by_room: dict[int, np.ndarray] = {}
        for r in rooms:
            lbl = r["_grid_label"]
            mask = (room_labels == lbl)
            dilated_by_room[lbl] = binary_dilation(mask, structure=struct)
        doorway_nodes: list[dict[str, Any]] = []
        edges: list[dict[str, Any]] = []
        seen_pairs: set[tuple[int, int]] = set()
        for i, ri in enumerate(rooms):
            for rj in rooms[i + 1:]:
                a, b = ri["_grid_label"], rj["_grid_label"]
                key = tuple(sorted((a, b)))
                if key in seen_pairs:
                    continue
                # Find cells where both dilated masks overlap, AND that
                # are NOT inside either room (i.e., the partition between
                # them — in /merged_map this should be occupied or
                # unknown; we accept occupied OR free if the gap is
                # narrow).
                overlap = dilated_by_room[a] & dilated_by_room[b]
                room_a = (room_labels == a)
                room_b = (room_labels == b)
                overlap = overlap & ~room_a & ~room_b
                if not np.any(overlap):
                    seen_pairs.add(key)
                    continue
                comp_lbl, n_comp = nd_label(overlap, structure=struct)
                # Each connected overlap component is a candidate
                # doorway between rooms a and b.
                accepted = 0
                for c in range(1, n_comp + 1):
                    cells = (comp_lbl == c)
                    if not np.any(cells):
                        continue
                    ys, xs = np.where(cells)
                    if len(xs) < 1:
                        continue
                    width_cells = max(
                        xs.max() - xs.min() + 1,
                        ys.max() - ys.min() + 1,
                    )
                    width_m = width_cells * res
                    if width_m < self.doorway_min_width_m or width_m > self.doorway_max_width_m:
                        continue
                    # Reject the doorway if its overlap cells are mostly
                    # free (suggests rooms shouldn't have been split).
                    free_share = float(((classes == _FREE) & cells).sum()) / max(1, cells.sum())
                    if free_share > 0.7:
                        continue
                    cx_w, cy_w = _grid_xy(float(xs.mean()), float(ys.mean()), msg)
                    cx_w -= 0.5 * res
                    cy_w -= 0.5 * res
                    accepted += 1
                    door_id = f"doorway_{ri['node_id']}_{rj['node_id']}_{accepted}"
                    self._first_seen.setdefault(door_id, now_t)
                    doorway_nodes.append({
                        "node_id": door_id,
                        "type": "doorway",
                        "label": door_id,
                        "centroid": {"x": round(cx_w, 3), "y": round(cy_w, 3)},
                        "width_m": round(width_m, 3),
                        "between": [ri["node_id"], rj["node_id"]],
                        "confidence": 0.7,
                        "first_seen_time": self._first_seen[door_id],
                        "last_seen_time": now_t,
                    })
                    edges.append({
                        "source": ri["node_id"],
                        "target": rj["node_id"],
                        "relation": "connected_to",
                        "via": door_id,
                        "confidence": 0.7,
                    })
                seen_pairs.add(key)
        return doorway_nodes, edges

    def _build_obstacle_nodes(
        self,
        classes: np.ndarray,
        room_labels: np.ndarray,
        rooms: list[dict[str, Any]],
        msg: OccupancyGrid,
        now_t: float,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        occ = (classes == _OCC)
        labels, n_comp = self._label_4connected(occ)
        res = float(msg.info.resolution)
        cell_area = res * res
        nodes: list[dict[str, Any]] = []
        edges: list[dict[str, Any]] = []
        next_id = 0
        for lbl in range(1, n_comp + 1):
            cells = labels == lbl
            area_cells = int(cells.sum())
            area_m2 = area_cells * cell_area
            if area_m2 < self.obstacle_min_area_m2 or area_m2 > self.obstacle_max_area_m2:
                continue
            ys, xs = np.where(cells)
            ix_min, ix_max = int(xs.min()), int(xs.max())
            iy_min, iy_max = int(ys.min()), int(ys.max())
            cx_ix = float(xs.mean())
            cy_iy = float(ys.mean())
            cx_w, cy_w = _grid_xy(cx_ix, cy_iy, msg)
            cx_w -= 0.5 * res
            cy_w -= 0.5 * res
            wx_min, wy_min = _grid_xy(ix_min, iy_min, msg)
            wx_max, wy_max = _grid_xy(ix_max, iy_max, msg)
            # Find the nearest room: look at the 8-cell neighbourhood
            # around the obstacle's bbox margin and check room_labels.
            margin = 2
            sub_y0, sub_y1 = max(0, iy_min - margin), min(room_labels.shape[0], iy_max + margin + 1)
            sub_x0, sub_x1 = max(0, ix_min - margin), min(room_labels.shape[1], ix_max + margin + 1)
            sub = room_labels[sub_y0:sub_y1, sub_x0:sub_x1]
            counts: dict[int, int] = {}
            for v in np.unique(sub):
                if v == 0:
                    continue
                counts[int(v)] = int((sub == v).sum())
                # Track which rooms touch this obstacle.
            inside_room_id: str | None = None
            if counts:
                top_lbl = max(counts.items(), key=lambda kv: kv[1])[0]
                for r in rooms:
                    if r["_grid_label"] == top_lbl:
                        inside_room_id = r["node_id"]
                        break
            next_id += 1
            obs_id = f"obstacle_{next_id}"
            self._first_seen.setdefault(obs_id, now_t)
            node = {
                "node_id": obs_id,
                "type": "obstacle",
                "label": obs_id,
                "centroid": {"x": round(cx_w, 3), "y": round(cy_w, 3)},
                "bbox": {
                    "x_min": round(wx_min, 3), "y_min": round(wy_min, 3),
                    "x_max": round(wx_max, 3), "y_max": round(wy_max, 3),
                },
                "area_m2": round(area_m2, 3),
                "inside_room": inside_room_id,
                "confidence": 0.6,
                "first_seen_time": self._first_seen[obs_id],
                "last_seen_time": now_t,
            }
            nodes.append(node)
            if inside_room_id is not None:
                edges.append({
                    "source": obs_id,
                    "target": inside_room_id,
                    "relation": "inside",
                    "confidence": 0.7,
                })
        return nodes, edges

    def _attach_external_candidates(
        self,
        rooms: list[dict[str, Any]],
        room_labels: np.ndarray,
        msg: OccupancyGrid,
        now_t: float,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """Pull loop_close + reconstruct candidates into the graph as
        their own node types and link `near` / `needs_revisit` edges.
        """
        nodes: list[dict[str, Any]] = []
        edges: list[dict[str, Any]] = []
        res = float(msg.info.resolution)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)

        def _world_to_room_id(x: float, y: float) -> str | None:
            ix = int(math.floor((x - ox) / max(1e-6, res)))
            iy = int(math.floor((y - oy) / max(1e-6, res)))
            if iy < 0 or iy >= room_labels.shape[0] or ix < 0 or ix >= room_labels.shape[1]:
                return None
            lbl = int(room_labels[iy, ix])
            if lbl <= 0:
                return None
            for r in rooms:
                if r["_grid_label"] == lbl:
                    return r["node_id"]
            return None

        for i, c in enumerate(self._loop_payload.get("candidates", []) or []):
            if not isinstance(c, dict):
                continue
            x = float(c.get("x", 0.0))
            y = float(c.get("y", 0.0))
            nid = f"loop_candidate_{i+1}"
            self._first_seen.setdefault(nid, now_t)
            host = _world_to_room_id(x, y)
            nodes.append({
                "node_id": nid,
                "type": "loop_candidate",
                "label": nid,
                "centroid": {"x": round(x, 3), "y": round(y, 3)},
                "loop_gain": float(c.get("loop_gain", 0.0) or 0.0),
                "target_robot": str(c.get("target_robot", "")).strip("/") or None,
                "host_room": host,
                "confidence": 0.5,
                "first_seen_time": self._first_seen[nid],
                "last_seen_time": now_t,
            })
            if host is not None:
                edges.append({
                    "source": nid,
                    "target": host,
                    "relation": "needs_revisit",
                    "confidence": 0.5,
                })

        for i, c in enumerate(self._recon_payload.get("candidates", []) or []):
            if not isinstance(c, dict):
                continue
            x = float(c.get("x", 0.0))
            y = float(c.get("y", 0.0))
            nid = f"low_quality_reconstruction_region_{i+1}"
            self._first_seen.setdefault(nid, now_t)
            host = _world_to_room_id(x, y)
            nodes.append({
                "node_id": nid,
                "type": "low_quality_reconstruction_region",
                "label": nid,
                "centroid": {"x": round(x, 3), "y": round(y, 3)},
                "recon_gain": float(c.get("recon_gain", 0.0) or 0.0),
                "view_diversity": int(c.get("view_diversity", 0) or 0),
                "voxel_count": int(c.get("voxel_count", 0) or 0),
                "target_robot": str(c.get("target_robot", "")).strip("/") or None,
                "host_room": host,
                "confidence": 0.7,
                "first_seen_time": self._first_seen[nid],
                "last_seen_time": now_t,
            })
            if host is not None:
                edges.append({
                    "source": nid,
                    "target": host,
                    "relation": "inside",
                    "confidence": 0.7,
                })
                edges.append({
                    "source": nid,
                    "target": host,
                    "relation": "needs_revisit",
                    "confidence": 0.7,
                })
        return nodes, edges

    @staticmethod
    def _near_edges(rooms: list[dict[str, Any]], near_m: float) -> list[dict[str, Any]]:
        edges: list[dict[str, Any]] = []
        for i, ri in enumerate(rooms):
            for rj in rooms[i + 1:]:
                dx = ri["centroid"]["x"] - rj["centroid"]["x"]
                dy = ri["centroid"]["y"] - rj["centroid"]["y"]
                d = math.hypot(dx, dy)
                if d <= near_m:
                    edges.append({
                        "source": ri["node_id"],
                        "target": rj["node_id"],
                        "relation": "near",
                        "distance_m": round(d, 3),
                        "confidence": 0.6,
                    })
        return edges

    # ── publish ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._map_msg is None:
            return
        msg = self._map_msg
        now_t = now_sec_from_node(self)
        classes = _grid_to_classes(msg)
        rooms, room_labels = self._build_room_nodes(classes, msg, now_t)
        doorway_nodes, doorway_edges = self._detect_doorways(rooms, room_labels, classes, msg, now_t)
        obstacle_nodes, obstacle_edges = self._build_obstacle_nodes(classes, room_labels, rooms, msg, now_t)
        ext_nodes, ext_edges = self._attach_external_candidates(rooms, room_labels, msg, now_t)
        near_edges = self._near_edges(rooms, self.near_distance_m)

        # Strip internal helpers before publishing.
        for r in rooms:
            r.pop("_grid_label", None)

        nodes = rooms + doorway_nodes + obstacle_nodes + ext_nodes
        edges = doorway_edges + obstacle_edges + ext_edges + near_edges

        node_type_counts: dict[str, int] = {}
        for n in nodes:
            node_type_counts[n["type"]] = node_type_counts.get(n["type"], 0) + 1
        edge_relation_counts: dict[str, int] = {}
        for e in edges:
            edge_relation_counts[e["relation"]] = edge_relation_counts.get(e["relation"], 0) + 1

        payload: dict[str, Any] = {
            "stamp_sec": now_t,
            "schema": "scene_graph/v1",
            "frame_id": msg.header.frame_id or "map",
            "map_resolution_m": float(msg.info.resolution),
            "node_count": len(nodes),
            "edge_count": len(edges),
            "node_type_counts": node_type_counts,
            "edge_relation_counts": edge_relation_counts,
            "nodes": nodes,
            "edges": edges,
        }
        m = String()
        m.data = json.dumps(payload, separators=(",", ":"))
        self._pub.publish(m)
        self._publish_markers(nodes, edges)
        if self.output_path:
            atomic_write_json(self.output_path, payload)

    def _publish_markers(self, nodes: list[dict[str, Any]], edges: list[dict[str, Any]]) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        idx = 0
        type_color: dict[str, tuple[float, float, float]] = {
            "room": (0.2, 0.7, 0.2),
            "corridor": (0.2, 0.5, 0.8),
            "doorway": (0.95, 0.85, 0.1),
            "obstacle": (0.85, 0.25, 0.25),
            "loop_candidate": (0.7, 0.2, 0.7),
            "low_quality_reconstruction_region": (0.95, 0.4, 0.85),
        }
        for n in nodes:
            idx += 1
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = f"scene_graph/{n['type']}"
            m.id = idx
            m.action = Marker.ADD
            m.pose.position.x = float(n["centroid"]["x"])
            m.pose.position.y = float(n["centroid"]["y"])
            m.pose.position.z = 0.20
            m.pose.orientation.w = 1.0
            r, g, b = type_color.get(n["type"], (0.6, 0.6, 0.6))
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.55
            if n["type"] in ("room", "corridor"):
                m.type = Marker.CUBE
                bbox = n["bbox"]
                m.scale.x = max(0.3, bbox["x_max"] - bbox["x_min"])
                m.scale.y = max(0.3, bbox["y_max"] - bbox["y_min"])
                m.scale.z = 0.05
            else:
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 0.40
            ma.markers.append(m)
            # Text label.
            idx += 1
            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp = self.get_clock().now().to_msg()
            t.ns = f"scene_graph/{n['type']}_label"
            t.id = idx
            t.action = Marker.ADD
            t.pose.position.x = float(n["centroid"]["x"])
            t.pose.position.y = float(n["centroid"]["y"])
            t.pose.position.z = 0.55
            t.pose.orientation.w = 1.0
            t.type = Marker.TEXT_VIEW_FACING
            t.scale.z = 0.30
            t.color.r = 1.0
            t.color.g = 1.0
            t.color.b = 1.0
            t.color.a = 0.9
            t.text = n["node_id"]
            ma.markers.append(t)
        # Edges as line strips.
        node_xy = {n["node_id"]: (n["centroid"]["x"], n["centroid"]["y"]) for n in nodes}
        edge_color: dict[str, tuple[float, float, float]] = {
            "connected_to": (0.95, 0.85, 0.1),
            "inside": (0.4, 0.4, 0.4),
            "near": (0.5, 0.5, 0.9),
            "needs_revisit": (0.95, 0.4, 0.85),
        }
        from geometry_msgs.msg import Point as _PointMsg
        for e in edges:
            a_xy = node_xy.get(e["source"])
            b_xy = node_xy.get(e["target"])
            if a_xy is None or b_xy is None:
                continue
            idx += 1
            ln = Marker()
            ln.header.frame_id = "map"
            ln.header.stamp = self.get_clock().now().to_msg()
            ln.ns = f"scene_graph/edge_{e['relation']}"
            ln.id = idx
            ln.action = Marker.ADD
            ln.type = Marker.LINE_STRIP
            ln.scale.x = 0.04
            cr, cg, cb = edge_color.get(e["relation"], (0.7, 0.7, 0.7))
            ln.color.r = cr
            ln.color.g = cg
            ln.color.b = cb
            ln.color.a = 0.6
            p1 = _PointMsg(); p1.x = float(a_xy[0]); p1.y = float(a_xy[1]); p1.z = 0.18
            p2 = _PointMsg(); p2.x = float(b_xy[0]); p2.y = float(b_xy[1]); p2.z = 0.18
            ln.points = [p1, p2]
            ma.markers.append(ln)
        self._marker_pub.publish(ma)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneGraphBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
