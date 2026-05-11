from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
except ModuleNotFoundError:  # Allows pure export/backend unit tests without sourcing ROS 2.
    rclpy = None  # type: ignore[assignment]
    TransformStamped = Any  # type: ignore[misc,assignment]
    Odometry = Any  # type: ignore[misc,assignment]
    Node = object  # type: ignore[misc,assignment]
    String = Any  # type: ignore[misc,assignment]

from .common import (
    dumps_compact,
    invert_se2,
    loads_dict,
    pose_dict_from_msg,
    quat_from_yaw,
    se2_from_xyyaw,
    stamp_to_sec,
    wrap_pi,
    xyyaw_from_se2,
)


GraphKey = tuple[str, int]


@dataclass
class PoseRecord:
    robot_id: str
    keyframe_id: int
    x: float
    y: float
    yaw: float
    stamp_sec: float


@dataclass
class FactorRecord:
    factor_type: str
    key1: GraphKey
    key2: GraphKey | None
    measurement: tuple[float, float, float]
    noise_scale: float
    source: str
    match_id: str = ""


@dataclass
class PoseGraphSnapshot:
    poses: dict[GraphKey, PoseRecord]
    factors: list[FactorRecord]
    raw_inter_robot_matches: int = 0
    rejected_inter_robot_matches: int = 0
    alignment_confidence: float = 0.0
    gt_used_runtime: bool = False


def snapshot_to_factor_payload(snapshot: PoseGraphSnapshot) -> dict[str, Any]:
    """Return the stable JSON factor schema used by runtime and C++/export tools."""

    return {
        "schema": "team_pose_graph_factors/v1",
        "gt_used_runtime": bool(snapshot.gt_used_runtime),
        "raw_inter_robot_matches": int(snapshot.raw_inter_robot_matches),
        "rejected_inter_robot_matches": int(snapshot.rejected_inter_robot_matches),
        "alignment_confidence": round(float(snapshot.alignment_confidence), 5),
        "poses": [
            {
                "robot_id": pose.robot_id,
                "keyframe_id": pose.keyframe_id,
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "stamp_sec": pose.stamp_sec,
            }
            for _key, pose in sorted(snapshot.poses.items())
        ],
        "factors": [
            {
                "factor_type": factor.factor_type,
                "key1": list(factor.key1),
                "key2": list(factor.key2) if factor.key2 is not None else None,
                "measurement": list(factor.measurement),
                "noise_scale": factor.noise_scale,
                "source": factor.source,
                "match_id": factor.match_id,
            }
            for factor in snapshot.factors
        ],
    }


@dataclass
class TeamPoseGraphBackend:
    backend: str
    export_dir: Path
    metrics_path: Path
    last_optimized_poses: dict[GraphKey, tuple[float, float, float]] = field(default_factory=dict)

    def optimize_or_export(self, snapshot: PoseGraphSnapshot) -> dict[str, Any]:
        self.export_dir.mkdir(parents=True, exist_ok=True)
        requested = self.backend.strip().lower() or "auto"
        if requested == "auto":
            if self._python_gtsam_available():
                requested = "gtsam_python"
            elif self._cpp_gtsam_available():
                requested = "gtsam_cpp"
            else:
                requested = "g2o_export_only"
        if requested in {"gtsam", "gtsam_python"}:
            try:
                metrics = self._optimize_gtsam(snapshot)
                metrics["optimization_backend"] = "gtsam_python"
            except Exception as exc:
                metrics = self._export_only(snapshot, backend="g2o_export_only")
                metrics["optimization_backend"] = "g2o_export_only"
                metrics["optimization_success"] = False
                metrics["latest_optimization_error"] = f"gtsam_python_failed:{exc}"
                metrics["dependency_blocker"] = "python_gtsam_failed"
        elif requested == "gtsam_cpp":
            metrics = self._export_only(snapshot, backend="g2o_export_only")
            metrics["dependency_blocker"] = (
                "gtsam_cpp_external_optimizer_required"
                if self._cpp_gtsam_available()
                else "gtsam_cpp_not_found"
            )
        else:
            metrics = self._export_only(snapshot, backend="g2o_export_only")
            if self.backend.strip().lower() == "auto":
                metrics["dependency_blocker"] = self._dependency_blocker()
        self.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return metrics

    @staticmethod
    def _python_gtsam_available() -> bool:
        try:
            import gtsam  # noqa: F401
        except Exception:
            return False
        return True

    @staticmethod
    def _cpp_gtsam_available() -> bool:
        import os
        import shutil
        import subprocess
        from pathlib import Path

        cmake = shutil.which("cmake")
        if cmake:
            candidates: list[str] = []
            local_prefix = (
                Path(__file__).resolve().parents[4]
                / ".local_deps"
                / "gtsam_humble"
                / "extract"
                / "opt"
                / "ros"
                / "humble"
            )
            if (
                (local_prefix / "include" / "gtsam" / "slam" / "BetweenFactor.h").exists()
                and (local_prefix / "include" / "gtsam" / "nonlinear" / "LevenbergMarquardtOptimizer.h").exists()
                and any(local_prefix.glob("lib*/**/libgtsam.so*"))
            ):
                return True
            if (local_prefix / "lib" / "cmake" / "GTSAM" / "GTSAMConfig.cmake").exists():
                candidates.append(str(local_prefix))
            if os.environ.get("CMAKE_PREFIX_PATH"):
                candidates.append(os.environ["CMAKE_PREFIX_PATH"])
            cmd = [
                cmake,
                "--find-package",
                "-DNAME=GTSAM",
                "-DCOMPILER_ID=GNU",
                "-DLANGUAGE=CXX",
                "-DMODE=EXIST",
            ]
            try:
                env = dict(os.environ)
                if candidates:
                    env["CMAKE_PREFIX_PATH"] = os.pathsep.join(candidates)
                proc = subprocess.run(
                    cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env
                )
                if proc.returncode == 0 and "not found" not in (proc.stdout + proc.stderr).lower():
                    return True
            except Exception:
                pass
        ldconfig = shutil.which("ldconfig")
        if ldconfig:
            try:
                proc = subprocess.run([ldconfig, "-p"], check=False, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                return proc.returncode == 0 and "gtsam" in proc.stdout.lower()
            except Exception:
                return False
        return False

    def _dependency_blocker(self) -> str:
        blockers: list[str] = []
        if not self._python_gtsam_available():
            blockers.append("python_gtsam_not_found")
        if not self._cpp_gtsam_available():
            blockers.append("gtsam_cpp_not_found")
        return ";".join(blockers)

    @staticmethod
    def _factor_counts(snapshot: PoseGraphSnapshot) -> dict[str, int]:
        return {
            "num_odom_factors": sum(1 for f in snapshot.factors if f.factor_type == "odom"),
            "num_inter_robot_factors_inlier": sum(
                1 for f in snapshot.factors if f.factor_type == "inter_robot"
            ),
            "num_prior_factors": sum(1 for f in snapshot.factors if f.factor_type == "prior"),
        }

    def _base_metrics(self, snapshot: PoseGraphSnapshot) -> dict[str, Any]:
        counts = self._factor_counts(snapshot)
        return {
            "schema": "team_pose_graph_metrics/v1",
            "num_keyframes_robot_a": sum(1 for k in snapshot.poses if k[0] == "robot_a"),
            "num_keyframes_robot_b": sum(1 for k in snapshot.poses if k[0] == "robot_b"),
            "num_odom_factors": counts["num_odom_factors"],
            "num_prior_factors": counts["num_prior_factors"],
            "num_inter_robot_factors_raw": int(snapshot.raw_inter_robot_matches),
            "num_inter_robot_factors_inlier": counts["num_inter_robot_factors_inlier"],
            "num_inter_robot_factors_rejected": int(snapshot.rejected_inter_robot_matches),
            "pose_graph_num_factors": len(snapshot.factors),
            "alignment_confidence": round(float(snapshot.alignment_confidence), 5),
            "self_loop_events_available": False,
            "gt_used_runtime": bool(snapshot.gt_used_runtime),
        }

    def _export_only(self, snapshot: PoseGraphSnapshot, *, backend: str) -> dict[str, Any]:
        self.last_optimized_poses = {}
        self._write_json_export(snapshot)
        self._write_g2o_export(snapshot)
        metrics = self._base_metrics(snapshot)
        metrics.update({
            "optimization_backend": backend,
            "optimization_success": False,
            "latest_optimization_error": None,
            "pose_graph_error_before": None,
            "pose_graph_error_after": None,
            "export_paths": {
                "g2o": str(self.export_dir / "team_pose_graph.g2o"),
                "factors_json": str(self.export_dir / "team_pose_graph_factors.json"),
            },
        })
        return metrics

    def _write_json_export(self, snapshot: PoseGraphSnapshot) -> None:
        data = snapshot_to_factor_payload(snapshot)
        (self.export_dir / "team_pose_graph_factors.json").write_text(
            json.dumps(data, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def _write_g2o_export(self, snapshot: PoseGraphSnapshot) -> None:
        key_to_idx = {key: idx for idx, key in enumerate(sorted(snapshot.poses))}
        lines: list[str] = []
        for key, pose in sorted(snapshot.poses.items()):
            idx = key_to_idx[key]
            lines.append(f"VERTEX_SE2 {idx} {pose.x:.9f} {pose.y:.9f} {pose.yaw:.9f}")
        for factor in snapshot.factors:
            if factor.factor_type == "prior" and factor.key1 in key_to_idx:
                lines.append(f"FIX {key_to_idx[factor.key1]}")
                continue
            if factor.key2 is None or factor.key1 not in key_to_idx or factor.key2 not in key_to_idx:
                continue
            dx, dy, dyaw = factor.measurement
            info = max(1e-6, 1.0 / max(1e-6, factor.noise_scale * factor.noise_scale))
            lines.append(
                "EDGE_SE2 "
                f"{key_to_idx[factor.key1]} {key_to_idx[factor.key2]} "
                f"{dx:.9f} {dy:.9f} {wrap_pi(dyaw):.9f} "
                f"{info:.9f} 0 0 {info:.9f} 0 {info:.9f}"
            )
        (self.export_dir / "team_pose_graph.g2o").write_text("\n".join(lines) + "\n", encoding="utf-8")

    @staticmethod
    def _symbol(robot: str, keyframe_id: int) -> int:
        import gtsam

        prefix = ord("a") if robot == "robot_a" else ord("b")
        return int(gtsam.symbol(chr(prefix), int(keyframe_id)))

    def _optimize_gtsam(self, snapshot: PoseGraphSnapshot) -> dict[str, Any]:
        import gtsam

        if not any(f.factor_type == "inter_robot" for f in snapshot.factors):
            self._write_json_export(snapshot)
            self._write_g2o_export(snapshot)
            metrics = self._base_metrics(snapshot)
            metrics.update({
                "optimization_backend": "gtsam",
                "optimization_success": False,
                "latest_optimization_error": "no_inter_robot_factors",
                "pose_graph_error_before": None,
                "pose_graph_error_after": None,
                "export_paths": {
                    "g2o": str(self.export_dir / "team_pose_graph.g2o"),
                    "factors_json": str(self.export_dir / "team_pose_graph_factors.json"),
                },
            })
            return metrics

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas([0.05, 0.05, 0.03])
        odom_noise = gtsam.noiseModel.Diagonal.Sigmas([0.25, 0.25, 0.15])
        loop_noise = gtsam.noiseModel.Diagonal.Sigmas([0.50, 0.50, 0.25])
        for key, pose in sorted(snapshot.poses.items()):
            values.insert(self._symbol(*key), gtsam.Pose2(pose.x, pose.y, pose.yaw))
        for factor in snapshot.factors:
            k1 = self._symbol(*factor.key1)
            dx, dy, dyaw = factor.measurement
            meas = gtsam.Pose2(dx, dy, wrap_pi(dyaw))
            if factor.factor_type == "prior":
                graph.add(gtsam.PriorFactorPose2(k1, meas, prior_noise))
            elif factor.key2 is not None:
                k2 = self._symbol(*factor.key2)
                noise = loop_noise if factor.factor_type == "inter_robot" else odom_noise
                graph.add(gtsam.BetweenFactorPose2(k1, k2, meas, noise))
        error_before = float(graph.error(values))
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(30)
        result = gtsam.LevenbergMarquardtOptimizer(graph, values, params).optimize()
        error_after = float(graph.error(result))
        self.last_optimized_poses = {}
        for key in snapshot.poses:
            pose = result.atPose2(self._symbol(*key))
            self.last_optimized_poses[key] = (float(pose.x()), float(pose.y()), float(pose.theta()))
        self._write_json_export(snapshot)
        self._write_g2o_export(snapshot)
        metrics = self._base_metrics(snapshot)
        metrics.update({
            "optimization_backend": "gtsam",
            "optimization_success": True,
            "latest_optimization_error": round(error_after, 6),
            "pose_graph_error_before": round(error_before, 6),
            "pose_graph_error_after": round(error_after, 6),
            "export_paths": {
                "g2o": str(self.export_dir / "team_pose_graph.g2o"),
                "factors_json": str(self.export_dir / "team_pose_graph_factors.json"),
            },
        })
        return metrics


class TeamPoseGraphNode(Node):
    """Centralized team pose graph with GTSAM-or-export backend."""

    def __init__(self) -> None:
        super().__init__("team_pose_graph_node")
        self.declare_parameter("robots", ["robot_a", "robot_b"])
        self.declare_parameter("keyframe_topic", "/team_slam/keyframes")
        self.declare_parameter("additional_keyframe_topics", [])
        self.declare_parameter("match_topic", "/team_slam/cross_robot_matches")
        self.declare_parameter("robust_inliers_topic", "/team_slam/robust_loop_inliers")
        self.declare_parameter("metrics_topic", "/team_slam/pose_graph_metrics")
        self.declare_parameter("local_metrics_topic", "/team_slam/local/pose_graph_metrics")
        self.declare_parameter("factors_topic", "/team_slam/team_pose_graph_factors")
        self.declare_parameter("team_pose_graph_backend", "auto")
        self.declare_parameter("export_dir", "logs")
        self.declare_parameter("metrics_path", "logs/team_pose_graph_metrics.json")
        self.declare_parameter("optimize_period_sec", 2.0)
        self.declare_parameter("publish_global_odom", True)
        self.declare_parameter("publish_backend_metrics", True)
        self.declare_parameter("allow_export_only_outputs", False)
        self.declare_parameter("no_overlap_rejection_passed", False)

        raw_robots = self.get_parameter("robots").value
        self.robots = [str(r).strip().strip("/") for r in raw_robots if str(r).strip()]
        self.keyframe_topic = str(self.get_parameter("keyframe_topic").value)
        self.additional_keyframe_topics = [
            str(t) for t in self.get_parameter("additional_keyframe_topics").value if str(t).strip()
        ]
        self.match_topic = str(self.get_parameter("match_topic").value)
        self.robust_topic = str(self.get_parameter("robust_inliers_topic").value)
        self.metrics_topic = str(self.get_parameter("metrics_topic").value)
        self.local_metrics_topic = str(self.get_parameter("local_metrics_topic").value)
        self.factors_topic = str(self.get_parameter("factors_topic").value)
        self.publish_global_odom = bool(self.get_parameter("publish_global_odom").value)
        self.publish_backend_metrics = bool(self.get_parameter("publish_backend_metrics").value)
        self.allow_export_only_outputs = bool(self.get_parameter("allow_export_only_outputs").value)
        self.no_overlap_rejection_passed = bool(
            self.get_parameter("no_overlap_rejection_passed").value
        )
        export_dir = Path(str(self.get_parameter("export_dir").value))
        metrics_path = Path(str(self.get_parameter("metrics_path").value))
        self.backend = TeamPoseGraphBackend(
            backend=str(self.get_parameter("team_pose_graph_backend").value),
            export_dir=export_dir,
            metrics_path=metrics_path,
        )

        self.poses: dict[GraphKey, PoseRecord] = {}
        self.key_id_by_string: dict[str, GraphKey] = {}
        self.raw_matches = 0
        self.raw_rejected_matches = 0
        self.robust_payload: dict[str, Any] | None = None
        self.inter_factor_ids: set[str] = set()
        self.latest_odom: dict[str, Odometry] = {}
        self.latest_corrected: dict[str, Odometry] = {}
        self.latest_metrics: dict[str, Any] = {}

        self.create_subscription(String, self.keyframe_topic, self._on_keyframe, 50)
        for topic in self.additional_keyframe_topics:
            self.create_subscription(String, topic, self._on_keyframe, 50)
        self.create_subscription(String, self.match_topic, self._on_match, 50)
        self.create_subscription(String, self.robust_topic, self._on_robust_inliers, 10)
        for robot in self.robots:
            self.create_subscription(Odometry, f"/{robot}/Odometry", lambda msg, r=robot: self._on_odom(r, msg, corrected=False), 20)
            self.create_subscription(Odometry, f"/{robot}/corrected_odom", lambda msg, r=robot: self._on_odom(r, msg, corrected=True), 20)

        self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)
        self.local_metrics_pub = self.create_publisher(String, self.local_metrics_topic, 10)
        self.factors_pub = self.create_publisher(String, self.factors_topic, 10)
        self.global_odom_pubs = {
            robot: self.create_publisher(Odometry, f"/team_slam/{robot}/corrected_odom_global", 10)
            for robot in self.robots
        }
        self.map_tf_pubs = {
            robot: self.create_publisher(TransformStamped, f"/team_slam/team_map_to_{robot}_map", 10)
            for robot in self.robots
        }
        period = max(0.5, float(self.get_parameter("optimize_period_sec").value))
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"team_pose_graph_node up: backend={self.backend.backend} export_dir={export_dir}"
        )

    def _on_odom(self, robot: str, msg: Odometry, *, corrected: bool) -> None:
        if corrected:
            self.latest_corrected[robot] = msg
        else:
            self.latest_odom[robot] = msg

    def _on_keyframe(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if not payload or payload.get("schema") != "team_loop_keyframe/v1":
            return
        robot = str(payload.get("robot", payload.get("robot_id", ""))).strip().strip("/")
        if robot not in self.robots:
            return
        try:
            keyframe_id = int(payload.get("keyframe_id"))
        except Exception:
            keyframe_id = self._parse_keyframe_id(str(payload.get("id", "")))
        pose = payload.get("pose", {})
        record = PoseRecord(
            robot_id=robot,
            keyframe_id=keyframe_id,
            x=float(pose.get("x", 0.0)),
            y=float(pose.get("y", 0.0)),
            yaw=float(pose.get("yaw", 0.0)),
            stamp_sec=float(payload.get("stamp_sec", 0.0)),
        )
        key = (robot, keyframe_id)
        self.poses[key] = record
        self.key_id_by_string[str(payload.get("id", ""))] = key

    def _on_match(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if not payload or payload.get("schema") != "team_cross_robot_match/v1":
            return
        self.raw_matches += 1
        if not bool(payload.get("accepted", False)):
            self.raw_rejected_matches += 1

    def _on_robust_inliers(self, msg: String) -> None:
        payload = loads_dict(msg.data)
        if payload and payload.get("schema") == "team_robust_loop_inliers/v1":
            self.robust_payload = payload

    @staticmethod
    def _parse_keyframe_id(raw: str) -> int:
        try:
            return int(raw.rsplit("_", 1)[-1])
        except Exception:
            return -1

    @staticmethod
    def _relative_measurement(a: PoseRecord, b: PoseRecord) -> tuple[float, float, float]:
        t_a = se2_from_xyyaw(a.x, a.y, a.yaw)
        t_b = se2_from_xyyaw(b.x, b.y, b.yaw)
        return xyyaw_from_se2(invert_se2(t_a) @ t_b)

    def _build_snapshot(self) -> PoseGraphSnapshot:
        factors: list[FactorRecord] = []
        if ("robot_a", 0) in self.poses:
            factors.append(FactorRecord("prior", ("robot_a", 0), None, (0.0, 0.0, 0.0), 0.1, "anchor"))
        elif self.poses:
            first_key = sorted(self.poses)[0]
            first_pose = self.poses[first_key]
            factors.append(
                FactorRecord(
                    "prior",
                    first_key,
                    None,
                    (first_pose.x, first_pose.y, first_pose.yaw),
                    0.1,
                    "anchor",
                )
            )
        for robot in self.robots:
            keys = sorted([key for key in self.poses if key[0] == robot], key=lambda k: k[1])
            for k1, k2 in zip(keys, keys[1:]):
                factors.append(
                    FactorRecord(
                        "odom",
                        k1,
                        k2,
                        self._relative_measurement(self.poses[k1], self.poses[k2]),
                        1.0,
                        "keyframe_odom",
                    )
                )
        robust = self.robust_payload or {}
        if bool(robust.get("accepted", False)):
            for item in robust.get("inliers", []):
                payload = item.get("payload", item) if isinstance(item, dict) else {}
                factor = self._factor_from_match_payload(payload, item)
                if factor is not None and factor.match_id not in self.inter_factor_ids:
                    self.inter_factor_ids.add(factor.match_id)
                    factors.append(factor)
                elif factor is not None:
                    factors.append(factor)
        return PoseGraphSnapshot(
            poses=dict(self.poses),
            factors=factors,
            raw_inter_robot_matches=self.raw_matches,
            rejected_inter_robot_matches=self.raw_rejected_matches
            + int(robust.get("robust_rejected_matches", 0) or 0),
            alignment_confidence=float(robust.get("alignment_confidence", 0.0) or 0.0),
            gt_used_runtime=False,
        )

    def _key_from_payload(self, payload: dict[str, Any], item: dict[str, Any], prefix: str) -> GraphKey | None:
        robot = str(payload.get(f"{prefix}_robot", item.get(f"{prefix}_robot", ""))).strip().strip("/")
        key_raw = str(payload.get(f"{prefix}_keyframe", item.get(f"{prefix}_keyframe", "")))
        if key_raw in self.key_id_by_string:
            return self.key_id_by_string[key_raw]
        key_id = self._parse_keyframe_id(key_raw)
        key = (robot, key_id)
        return key if key in self.poses else None

    def _factor_from_match_payload(
        self,
        payload: dict[str, Any],
        item: dict[str, Any],
    ) -> FactorRecord | None:
        key1 = self._key_from_payload(payload, item, "source")
        key2 = self._key_from_payload(payload, item, "target")
        if key1 is None:
            key1 = self._key_from_payload(payload, item, "query")
        if key2 is None:
            key2 = self._key_from_payload(payload, item, "match")
        if key1 is None or key2 is None or key1[0] == key2[0]:
            return None
        measurement_raw = payload.get("T_query_to_match", item.get("T_query_to_match", []))
        try:
            measurement = (
                float(measurement_raw[0]),
                float(measurement_raw[1]),
                float(measurement_raw[2]),
            )
        except Exception:
            if key1 in self.poses and key2 in self.poses:
                measurement = self._relative_measurement(self.poses[key1], self.poses[key2])
            else:
                return None
        match_id = str(item.get("match_id") or payload.get("match_id") or f"{key1}-{key2}")
        noise = max(0.2, float(payload.get("rmse", item.get("rmse", 0.5)) or 0.5))
        return FactorRecord("inter_robot", key1, key2, measurement, noise, "robust", match_id)

    def _tick(self) -> None:
        snapshot = self._build_snapshot()
        self.factors_pub.publish(String(data=dumps_compact(snapshot_to_factor_payload(snapshot))))
        metrics = self.backend.optimize_or_export(snapshot)
        now = self.get_clock().now().to_msg()
        metrics["stamp_sec"] = round(float(now.sec) + float(now.nanosec) * 1e-9, 6)
        metrics["no_overlap_rejection_passed"] = self.no_overlap_rejection_passed
        self.latest_metrics = metrics
        if self.publish_backend_metrics:
            metrics_msg = String(data=dumps_compact(metrics))
            self.metrics_pub.publish(metrics_msg)
            self.local_metrics_pub.publish(metrics_msg)
        if self._outputs_allowed(metrics):
            self._publish_map_transforms()
            self._publish_global_odom()

    def _outputs_allowed(self, metrics: dict[str, Any]) -> bool:
        if bool(metrics.get("optimization_success", False)):
            return True
        return self.allow_export_only_outputs and metrics.get("optimization_backend") == "g2o_export_only"

    def _robust_transform(self) -> tuple[float, float, float] | None:
        robust = self.robust_payload or {}
        transform = robust.get("transform", {})
        try:
            return (float(transform["x"]), float(transform["y"]), float(transform.get("yaw", 0.0)))
        except Exception:
            return None

    def _publish_map_transforms(self) -> None:
        robust_tf = self._robust_transform()
        for robot in self.robots:
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "team_map"
            msg.child_frame_id = f"{robot}/map"
            if robot == "robot_a" or robust_tf is None:
                x, y, yaw = 0.0, 0.0, 0.0
            else:
                x, y, yaw = robust_tf
            qx, qy, qz, qw = quat_from_yaw(yaw)
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = 0.0
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            self.map_tf_pubs[robot].publish(msg)

    def _publish_global_odom(self) -> None:
        if not self.publish_global_odom:
            return
        robust_tf = self._robust_transform()
        for robot in self.robots:
            src = self.latest_corrected.get(robot) or self.latest_odom.get(robot)
            if src is None:
                continue
            pose = pose_dict_from_msg(src)
            t = se2_from_xyyaw(float(pose["x"]), float(pose["y"]), float(pose["yaw"]))
            if robot != "robot_a" and robust_tf is not None:
                t = se2_from_xyyaw(*robust_tf) @ t
            x, y, yaw = xyyaw_from_se2(t)
            qx, qy, qz, qw = quat_from_yaw(yaw)
            out = Odometry()
            out.header.stamp = src.header.stamp
            out.header.frame_id = "team_map"
            out.child_frame_id = src.child_frame_id
            out.pose = src.pose
            out.twist = src.twist
            out.pose.pose.position.x = x
            out.pose.pose.position.y = y
            out.pose.pose.orientation.x = qx
            out.pose.pose.orientation.y = qy
            out.pose.pose.orientation.z = qz
            out.pose.pose.orientation.w = qw
            self.global_odom_pubs[robot].publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeamPoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
