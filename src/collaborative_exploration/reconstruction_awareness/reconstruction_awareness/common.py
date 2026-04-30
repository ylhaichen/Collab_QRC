from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


def yaw_from_quat(q: Any) -> float:
    return math.atan2(
        2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
        1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z)),
    )


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, float(v)))


def now_sec_from_node(node: Any) -> float:
    return node.get_clock().now().nanoseconds / 1e9


def atomic_write_json(path: str, payload: dict[str, Any]) -> None:
    if not path:
        return
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(out.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(out)


def occupancy_at(msg: Any, x: float, y: float) -> int | None:
    res = float(msg.info.resolution)
    if res <= 0.0:
        return None
    gx = int(math.floor((x - float(msg.info.origin.position.x)) / res))
    gy = int(math.floor((y - float(msg.info.origin.position.y)) / res))
    w = int(msg.info.width)
    h = int(msg.info.height)
    if gx < 0 or gy < 0 or gx >= w or gy >= h:
        return None
    return int(msg.data[gy * w + gx])


def nearest_occupied_distance(msg: Any, x: float, y: float, radius_m: float) -> float:
    res = float(msg.info.resolution)
    if res <= 0.0:
        return float("inf")
    gx = int(math.floor((x - float(msg.info.origin.position.x)) / res))
    gy = int(math.floor((y - float(msg.info.origin.position.y)) / res))
    w = int(msg.info.width)
    h = int(msg.info.height)
    r = max(1, int(math.ceil(radius_m / res)))
    best = float("inf")
    for yy in range(max(0, gy - r), min(h, gy + r + 1)):
        for xx in range(max(0, gx - r), min(w, gx + r + 1)):
            v = int(msg.data[yy * w + xx])
            if v < 50:
                continue
            wx = float(msg.info.origin.position.x) + (xx + 0.5) * res
            wy = float(msg.info.origin.position.y) + (yy + 0.5) * res
            best = min(best, math.hypot(wx - x, wy - y))
    return best


def point_reachable_enough(msg: Any, x: float, y: float, clearance_m: float) -> bool:
    v = occupancy_at(msg, x, y)
    if v is None or v < 0 or v >= 50:
        return False
    return nearest_occupied_distance(msg, x, y, clearance_m) >= clearance_m
