from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class LocalSlamBackend:
    name: str
    default_safe_mode: bool
    point_lio_primary_candidate: bool


@dataclass(frozen=True)
class PointLioTopicContract:
    robot_namespace: str
    native_odom_topic: str
    native_cloud_topic: str
    native_static_topic: str
    native_dynamic_topic: str
    outputs: dict[str, str]
    gt_used_runtime: bool = False


@dataclass(frozen=True)
class PointLioAdapterStatus:
    robot_namespace: str
    mode: str
    native_odom_rate_hz: float
    adapter_odom_rate_hz: float
    cloud_rate_hz: float
    livox_input_seen: bool
    imu_input_seen: bool
    dependency_blocker: str = ""
    primary_enabled: bool = False

    def to_payload(self) -> dict[str, Any]:
        native_ok = self.native_odom_rate_hz > 0.0 and self.adapter_odom_rate_hz > 0.0
        input_ok = bool(self.livox_input_seen and self.imu_input_seen)
        shadow_ready = native_ok and input_ok and not self.dependency_blocker
        primary_ready = shadow_ready and self.mode == "primary" and bool(self.primary_enabled)
        return {
            "schema": "point_lio_adapter_status/v1",
            "robot_id": self.robot_namespace,
            "mode": self.mode,
            "native_odom_rate_hz": round(float(self.native_odom_rate_hz), 5),
            "adapter_odom_rate_hz": round(float(self.adapter_odom_rate_hz), 5),
            "cloud_rate_hz": round(float(self.cloud_rate_hz), 5),
            "livox_input_seen": bool(self.livox_input_seen),
            "imu_input_seen": bool(self.imu_input_seen),
            "shadow_ready": bool(shadow_ready),
            "primary_ready": bool(primary_ready),
            "dependency_blocker": self.dependency_blocker,
            "gt_used_runtime": False,
        }


def _ns(robot_namespace: str) -> str:
    cleaned = str(robot_namespace or "robot_a").strip().strip("/")
    return cleaned or "robot_a"


def normalize_local_slam_backend(value: str | None) -> LocalSlamBackend:
    name = str(value or "").strip().lower()
    if not name:
        name = "fast_lio_scpgo"
    aliases = {
        "fast_lio": "fast_lio_scpgo",
        "scpgo": "fast_lio_scpgo",
        "point-lio": "point_lio",
        "pointlio": "point_lio",
        "swarm_lio2": "swarm_lio2_experimental",
    }
    name = aliases.get(name, name)
    if name not in {"fast_lio_scpgo", "point_lio", "swarm_lio2_experimental"}:
        name = "fast_lio_scpgo"
    return LocalSlamBackend(
        name=name,
        default_safe_mode=name == "fast_lio_scpgo",
        point_lio_primary_candidate=name == "point_lio",
    )


def build_point_lio_contract(robot_namespace: str) -> PointLioTopicContract:
    ns = _ns(robot_namespace)
    return PointLioTopicContract(
        robot_namespace=ns,
        native_odom_topic=f"/{ns}/point_lio/Odometry",
        native_cloud_topic=f"/{ns}/point_lio/cloud_registered_body",
        native_static_topic=f"/{ns}/point_lio/cloud_static",
        native_dynamic_topic=f"/{ns}/point_lio/cloud_dynamic",
        outputs={
            "odometry": f"/{ns}/Odometry",
            "corrected_odom": f"/{ns}/corrected_odom",
            "nav_odom": f"/{ns}/odom/nav",
            "registered_body": f"/{ns}/cloud_registered_body",
            "static_cloud": f"/{ns}/cloud_static",
            "dynamic_cloud": f"/{ns}/cloud_dynamic",
            "tf": "/tf",
        },
    )
