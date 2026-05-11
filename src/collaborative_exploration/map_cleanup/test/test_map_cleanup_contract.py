from pathlib import Path

from map_cleanup.cleanup_contracts import (
    CleanupBackendStatus,
    RequiredCleanupExport,
    build_cleanup_command,
)


def test_required_cleanup_export_paths_match_erasor_removert_contract(tmp_path: Path) -> None:
    export = RequiredCleanupExport.from_root(tmp_path)

    assert export.pcd_dir == tmp_path / "pcds"
    assert export.dense_global_map == tmp_path / "dense_global_map.pcd"
    assert export.poses_lidar2body == tmp_path / "poses_lidar2body.csv"
    assert export.initial_naive_map == tmp_path / "initial_naive_map.pcd"
    assert export.missing_paths() == [
        str(tmp_path / "pcds"),
        str(tmp_path / "dense_global_map.pcd"),
        str(tmp_path / "poses_lidar2body.csv"),
        str(tmp_path / "initial_naive_map.pcd"),
    ]


def test_cleanup_backend_status_never_claims_external_backend_when_blocked() -> None:
    status = CleanupBackendStatus(
        backend="erasor",
        backend_available=False,
        fallback_backend="temporal_voxel_fallback",
        dependency_blocker="erasor_source_not_found",
    )

    payload = status.to_payload()

    assert payload["schema"] == "map_cleanup_backend_status/v1"
    assert payload["backend"] == "erasor"
    assert payload["runtime_ready"] is False
    assert payload["selected_runtime_backend"] == "temporal_voxel_fallback"
    assert payload["dependency_blocker"] == "erasor_source_not_found"
    assert payload["gt_used_runtime"] is False


def test_build_cleanup_command_uses_async_backend_not_realtime_odometry_loop(tmp_path: Path) -> None:
    export = RequiredCleanupExport.from_root(tmp_path)
    command = build_cleanup_command("removert", export, output_dir=tmp_path / "cleaned")

    assert command.backend == "removert"
    assert command.realtime_odometry_loop is False
    assert "dense_global_map.pcd" in " ".join(command.argv)
    assert "poses_lidar2body.csv" in " ".join(command.argv)
