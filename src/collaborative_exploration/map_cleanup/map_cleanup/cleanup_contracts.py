from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RequiredCleanupExport:
    root: Path
    pcd_dir: Path
    dense_global_map: Path
    poses_lidar2body: Path
    initial_naive_map: Path

    @classmethod
    def from_root(cls, root: str | Path) -> "RequiredCleanupExport":
        base = Path(root)
        return cls(
            root=base,
            pcd_dir=base / "pcds",
            dense_global_map=base / "dense_global_map.pcd",
            poses_lidar2body=base / "poses_lidar2body.csv",
            initial_naive_map=base / "initial_naive_map.pcd",
        )

    def missing_paths(self) -> list[str]:
        paths = [
            self.pcd_dir,
            self.dense_global_map,
            self.poses_lidar2body,
            self.initial_naive_map,
        ]
        return [str(path) for path in paths if not path.exists()]


@dataclass(frozen=True)
class CleanupCommand:
    backend: str
    argv: list[str]
    realtime_odometry_loop: bool = False


@dataclass(frozen=True)
class CleanupBackendStatus:
    backend: str
    backend_available: bool
    fallback_backend: str
    dependency_blocker: str = ""

    def selected_runtime_backend(self) -> str:
        if self.backend_available and not self.dependency_blocker:
            return self.backend
        return self.fallback_backend

    def to_payload(self) -> dict[str, Any]:
        return {
            "schema": "map_cleanup_backend_status/v1",
            "backend": self.backend,
            "backend_available": bool(self.backend_available),
            "runtime_ready": bool(self.backend_available and not self.dependency_blocker),
            "selected_runtime_backend": self.selected_runtime_backend(),
            "fallback_backend": self.fallback_backend,
            "dependency_blocker": self.dependency_blocker,
            "gt_used_runtime": False,
        }


def backend_available(backend: str) -> tuple[bool, str]:
    name = str(backend or "none").strip().lower()
    if name in {"none", "temporal_voxel_fallback"}:
        return True, ""
    exe = shutil.which(name)
    if exe:
        return True, ""
    source_dir = Path("src/vendor") / name
    if source_dir.exists():
        return True, ""
    return False, f"{name}_source_not_found"


def build_cleanup_command(
    backend: str,
    export: RequiredCleanupExport,
    *,
    output_dir: str | Path,
) -> CleanupCommand:
    name = str(backend or "none").strip().lower()
    out = Path(output_dir)
    if name == "erasor":
        argv = [
            "erasor",
            "--pcd_dir",
            str(export.pcd_dir),
            "--dense_global_map",
            str(export.dense_global_map),
            "--poses_lidar2body",
            str(export.poses_lidar2body),
            "--initial_naive_map",
            str(export.initial_naive_map),
            "--output_dir",
            str(out),
        ]
    elif name == "removert":
        argv = [
            "removert",
            "--input_map",
            str(export.dense_global_map),
            "--poses",
            str(export.poses_lidar2body),
            "--pcd_dir",
            str(export.pcd_dir),
            "--output_dir",
            str(out),
        ]
    else:
        argv = [
            "temporal_voxel_cleanup",
            "--input_map",
            str(export.initial_naive_map),
            "--output_dir",
            str(out),
        ]
    return CleanupCommand(backend=name, argv=argv, realtime_odometry_loop=False)
