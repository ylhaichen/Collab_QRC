from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from .temporal_voxel_filter import Point3


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


def transform_body_points_to_odom(points: Iterable[Point3], pose: Pose2D) -> list[Point3]:
    c = math.cos(float(pose.yaw))
    s = math.sin(float(pose.yaw))
    out: list[Point3] = []
    for raw in points:
        x = float(raw[0])
        y = float(raw[1])
        z = float(raw[2])
        out.append((
            float(pose.x) + c * x - s * y,
            float(pose.y) + s * x + c * y,
            float(pose.z) + z,
        ))
    return out


def split_original_points_by_labels(
    original_points: Iterable[Point3],
    labels: Iterable[str],
) -> tuple[list[Point3], list[Point3]]:
    static_points: list[Point3] = []
    dynamic_points: list[Point3] = []
    for point, label in zip(original_points, labels):
        if str(label) == "dynamic":
            dynamic_points.append(point)
        else:
            static_points.append(point)
    return static_points, dynamic_points
