#!/usr/bin/env python3
"""Keyframe logger for §10.2 offline 3DGS pipeline.

Captures (image, pose, intrinsics) tuples from a robot's front camera
periodically. Output is the directory layout that scripts/offline/
train_3dgs.py + standard 3DGS trainers (Inria gaussian-splatting,
gsplat) understand:

    <output_dir>/
        cameras.json          (intrinsics + per-frame extrinsics)
        keyframes/
            <ns>_NNNN.png     (RGB image, NNNN = frame index)
            <ns>_NNNN.pose.json
            ...

Camera topic auto-detection
---------------------------
At startup, the node enumerates topic types and binds to the first
sensor_msgs/Image whose name contains the configured `camera_match`
substring (default "front_camera"). If no camera is found within
`camera_wait_sec`, the node logs a warning and stays idle (no
keyframes saved). This keeps the logger optional — if MuJoCo is
launched without the RGBD camera plugin, the rest of the stack still
runs.

Triggering
----------
A keyframe is saved when EITHER condition fires:
  - robot pose has translated > `min_translation_m`
  - robot pose has rotated > `min_rotation_deg` (yaw)
  - elapsed since last keyframe > `period_sec`

This avoids dumping 20 Hz × 180 s = 3600 frames per trial; we end up
with 30-60 well-separated keyframes — the typical input size for
offline 3DGS training to converge in <30 min on RTX 4070.
"""
from __future__ import annotations

import json
import math
import os
import struct
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .common import now_sec_from_node, yaw_from_quat


def _decode_image(msg: Image) -> bytes | None:
    """Return PNG bytes for the image, or None on failure.

    Avoids the optional cv_bridge dep by handling the 3-channel 8-bit
    layouts we actually see on this stack:
      rgb8 / bgr8     — standard ROS encodings
      8UC3            — generic OpenCV encoding emitted by the MuJoCo
                        RGBD plugin; treated as BGR by convention
                        (cv::Mat default order)
    Anything else is logged and skipped.
    """
    enc = msg.encoding.lower()
    if enc not in ("rgb8", "bgr8", "8uc3"):
        return None
    try:
        from PIL import Image as PILImage
    except ImportError:
        return None
    n = msg.height * msg.width
    raw = bytes(msg.data)
    if len(raw) < n * 3:
        return None
    pil = PILImage.frombytes("RGB", (msg.width, msg.height), raw[: n * 3])
    if enc in ("bgr8", "8uc3"):
        b, g, r = pil.split()
        pil = PILImage.merge("RGB", (r, g, b))
    import io
    buf = io.BytesIO()
    pil.save(buf, format="PNG", compress_level=3)
    return buf.getvalue()


class KeyframeLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("keyframe_logger_node")
        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("camera_match", "front_camera")
        # Explicit topic overrides for non-namespaced cameras (e.g.
        # MuJoCo RGBD plugin which publishes to /<camera_name>/color/
        # image_raw with no robot ns prefix). Format:
        #   ["robot_a=/front_camera/color/image_raw",
        #    "robot_b=/b_front_camera/color/image_raw"]
        self.declare_parameter("camera_topic_overrides", [""])
        self.declare_parameter("camera_wait_sec", 8.0)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("min_translation_m", 0.50)
        self.declare_parameter("min_rotation_deg", 25.0)
        self.declare_parameter("period_sec", 4.0)
        self.declare_parameter("max_keyframes", 240)
        self.declare_parameter("default_focal_px", 320.0)

        nss = self.get_parameter("namespaces").value or []
        self.namespaces = [str(n).strip("/") for n in nss if str(n).strip("/")]
        self.match = str(self.get_parameter("camera_match").value).strip()
        # Parse per-ns camera topic overrides ("ns=topic" pairs).
        self._topic_override: dict[str, str] = {}
        for raw in self.get_parameter("camera_topic_overrides").value or []:
            s = str(raw).strip()
            if not s or "=" not in s:
                continue
            k, v = s.split("=", 1)
            self._topic_override[k.strip("/").strip()] = v.strip()
        self.camera_wait_sec = float(self.get_parameter("camera_wait_sec").value)
        self.output_dir = str(self.get_parameter("output_dir").value).strip()
        self.min_t = max(0.0, float(self.get_parameter("min_translation_m").value))
        self.min_r = max(0.0, float(self.get_parameter("min_rotation_deg").value))
        self.period_sec = max(0.5, float(self.get_parameter("period_sec").value))
        self.max_keyframes = max(1, int(self.get_parameter("max_keyframes").value))
        self.default_focal_px = float(self.get_parameter("default_focal_px").value)

        self._poses: dict[str, tuple[float, float, float, float]] = {}  # x,y,z,yaw
        self._last_kf: dict[str, dict[str, Any]] = {}
        self._kf_count: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self._intrinsics: dict[str, dict[str, Any]] = {}

        if self.output_dir:
            Path(self.output_dir).mkdir(parents=True, exist_ok=True)
            (Path(self.output_dir) / "keyframes").mkdir(parents=True, exist_ok=True)

        for ns in self.namespaces:
            self.create_subscription(
                Odometry, f"/{ns}/odom/nav", self._make_odom_cb(ns), 10
            )

        # Bind cameras lazily — try every 1 s for camera_wait_sec.
        self._bind_attempts = 0
        self._max_bind_attempts = max(1, int(self.camera_wait_sec))
        self._bound: set[str] = set()
        self.create_timer(1.0, self._try_bind_cameras)

        self.get_logger().info(
            f"keyframe_logger_node up: ns={self.namespaces} match={self.match!r} "
            f"period={self.period_sec}s min_t={self.min_t}m min_r={self.min_r}° "
            f"output_dir={self.output_dir or 'none'}"
        )

    # ── camera binding ────────────────────────────────────────────────

    def _try_bind_cameras(self) -> None:
        if self._bind_attempts >= self._max_bind_attempts and len(self._bound) == len(self.namespaces):
            return
        self._bind_attempts += 1
        try:
            topics = self.get_topic_names_and_types()
        except Exception:  # pragma: no cover
            topics = []
        topic_dict = dict(topics)
        for ns in self.namespaces:
            if ns in self._bound:
                continue
            target_topic: str | None = None
            # 1) Explicit override has top priority.
            if ns in self._topic_override:
                target_topic = self._topic_override[ns]
                if target_topic not in topic_dict:
                    # Bind anyway — topic may appear shortly after; the
                    # subscription queues until publisher arrives.
                    pass
            # 2) Otherwise scan for ns-prefixed Image topics matching `match`.
            if target_topic is None:
                for name, types in topics:
                    if "sensor_msgs/msg/Image" not in types:
                        continue
                    if f"/{ns}/" not in name and not name.startswith(f"/{ns}/"):
                        continue
                    if self.match and self.match not in name:
                        continue
                    target_topic = name
                    break
            # 3) Final fallback — non-namespaced MuJoCo RGBD layout
            #    (/front_camera/color/image_raw or /b_front_camera/...).
            #    Heuristic: if ns starts with "robot_b", prefer a "b_"-
            #    prefixed camera; otherwise a non-"b_"-prefixed one.
            if target_topic is None:
                want_b = ns.endswith("_b") or ns.endswith("b")
                for name, types in topics:
                    if "sensor_msgs/msg/Image" not in types:
                        continue
                    if self.match and self.match not in name:
                        continue
                    has_b_prefix = "/b_" in name or name.startswith("b_")
                    if want_b and has_b_prefix:
                        target_topic = name
                        break
                    if (not want_b) and (not has_b_prefix):
                        target_topic = name
                        break
            if target_topic is None:
                continue
            self._bind_image(ns, target_topic)
            info_topic = target_topic.replace("image_raw", "camera_info")
            self._bind_info(ns, info_topic)
            self._bound.add(ns)
        if self._bind_attempts >= self._max_bind_attempts:
            for ns in self.namespaces:
                if ns not in self._bound:
                    self.get_logger().warn(
                        f"keyframe_logger: no Image topic matched ns={ns} "
                        f"match={self.match!r} after {self.camera_wait_sec}s. "
                        f"Trial will run without keyframes for {ns} (offline "
                        f"3DGS will fall back to point-cloud-only init)."
                    )

    def _bind_image(self, ns: str, topic: str) -> None:
        self.get_logger().info(f"keyframe_logger: binding {ns} → {topic}")
        self.create_subscription(Image, topic, self._make_image_cb(ns), 5)

    def _bind_info(self, ns: str, topic: str) -> None:
        self.create_subscription(
            CameraInfo, topic, self._make_info_cb(ns), 5
        )

    # ── callbacks ────────────────────────────────────────────────────

    def _make_odom_cb(self, ns: str):
        def _cb(msg: Odometry) -> None:
            try:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                self._poses[ns] = (float(p.x), float(p.y), float(p.z), float(yaw_from_quat(q)))
            except Exception:  # pragma: no cover
                return
        return _cb

    def _make_info_cb(self, ns: str):
        def _cb(msg: CameraInfo) -> None:
            self._intrinsics[ns] = {
                "width": int(msg.width),
                "height": int(msg.height),
                "fx": float(msg.k[0]) if len(msg.k) >= 5 else self.default_focal_px,
                "fy": float(msg.k[4]) if len(msg.k) >= 5 else self.default_focal_px,
                "cx": float(msg.k[2]) if len(msg.k) >= 5 else 0.5 * float(msg.width),
                "cy": float(msg.k[5]) if len(msg.k) >= 5 else 0.5 * float(msg.height),
                "distortion": list(map(float, msg.d)) if msg.d else [],
            }
        return _cb

    def _make_image_cb(self, ns: str):
        def _cb(msg: Image) -> None:
            self._maybe_save_keyframe(ns, msg)
        return _cb

    # ── trigger logic ────────────────────────────────────────────────

    def _maybe_save_keyframe(self, ns: str, img: Image) -> None:
        if not self.output_dir:
            return
        if self._kf_count[ns] >= self.max_keyframes:
            return
        pose = self._poses.get(ns)
        if pose is None:
            return
        last = self._last_kf.get(ns)
        now = now_sec_from_node(self)
        ok = False
        if last is None:
            ok = True
        else:
            dt = now - float(last["t_sec"])
            dx = pose[0] - float(last["x"])
            dy = pose[1] - float(last["y"])
            dyaw = abs(((pose[3] - float(last["yaw"]) + math.pi) % (2 * math.pi)) - math.pi)
            translated = math.hypot(dx, dy) >= self.min_t
            rotated = math.degrees(dyaw) >= self.min_r
            timed_out = dt >= self.period_sec * 4.0
            if translated or rotated or timed_out or dt >= self.period_sec:
                ok = True
        if not ok:
            return
        self._save_keyframe(ns, img, pose, now)

    def _save_keyframe(
        self,
        ns: str,
        img: Image,
        pose: tuple[float, float, float, float],
        now_sec: float,
    ) -> None:
        idx = self._kf_count[ns]
        png = _decode_image(img)
        if png is None:
            self.get_logger().warn(
                f"{ns}: image encoding {img.encoding!r} unsupported "
                f"(need rgb8/bgr8 + Pillow); skipping keyframe."
            )
            return
        out_dir = Path(self.output_dir)
        kf_dir = out_dir / "keyframes"
        png_path = kf_dir / f"{ns}_{idx:04d}.png"
        meta_path = kf_dir / f"{ns}_{idx:04d}.pose.json"
        try:
            png_path.write_bytes(png)
        except OSError as exc:
            self.get_logger().warn(f"failed to write {png_path}: {exc}")
            return
        intr = self._intrinsics.get(ns) or {
            "width": int(img.width),
            "height": int(img.height),
            "fx": self.default_focal_px,
            "fy": self.default_focal_px,
            "cx": 0.5 * float(img.width),
            "cy": 0.5 * float(img.height),
            "distortion": [],
        }
        meta = {
            "namespace": ns,
            "frame_index": idx,
            "stamp_sec": round(now_sec, 4),
            "pose_world": {
                "x": round(pose[0], 4),
                "y": round(pose[1], 4),
                "z": round(pose[2], 4),
                "yaw_rad": round(pose[3], 4),
            },
            "intrinsics": intr,
            "image_path": png_path.name,
        }
        try:
            meta_path.write_text(json.dumps(meta, indent=2), encoding="utf-8")
        except OSError as exc:
            self.get_logger().warn(f"failed to write {meta_path}: {exc}")
            return
        self._last_kf[ns] = {
            "t_sec": now_sec,
            "x": pose[0],
            "y": pose[1],
            "yaw": pose[3],
        }
        self._kf_count[ns] += 1
        if self._kf_count[ns] in (1, 5, 25, 100, self.max_keyframes):
            self.get_logger().info(
                f"{ns}: saved keyframe {idx} → {png_path.name} "
                f"(total={self._kf_count[ns]})"
            )
        # Update summary at every save so the offline trainer can
        # discover the keyframe set without enumerating files.
        self._write_cameras_json()

    def _write_cameras_json(self) -> None:
        if not self.output_dir:
            return
        cams = {
            "schema": "keyframes/v1",
            "namespaces": self.namespaces,
            "default_focal_px": self.default_focal_px,
            "keyframe_counts": dict(self._kf_count),
            "intrinsics": dict(self._intrinsics),
        }
        try:
            (Path(self.output_dir) / "cameras.json").write_text(
                json.dumps(cams, indent=2), encoding="utf-8"
            )
        except OSError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyframeLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._write_cameras_json()
        except Exception:  # pragma: no cover
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
