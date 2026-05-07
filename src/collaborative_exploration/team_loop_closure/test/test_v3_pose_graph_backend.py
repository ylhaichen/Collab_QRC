import json
import math
from pathlib import Path

from team_loop_closure.team_pose_graph_node import (
    FactorRecord,
    PoseGraphSnapshot,
    PoseRecord,
    TeamPoseGraphBackend,
    snapshot_to_factor_payload,
)


def _snapshot() -> PoseGraphSnapshot:
    return PoseGraphSnapshot(
        poses={
            ("robot_a", 0): PoseRecord("robot_a", 0, 0.0, 0.0, 0.0, 1.0),
            ("robot_a", 1): PoseRecord("robot_a", 1, 1.0, 0.0, 0.0, 2.0),
            ("robot_b", 0): PoseRecord("robot_b", 0, 2.0, 1.0, math.pi / 2.0, 1.5),
        },
        factors=[
            FactorRecord("prior", ("robot_a", 0), None, (0.0, 0.0, 0.0), 1.0, "anchor"),
            FactorRecord("odom", ("robot_a", 0), ("robot_a", 1), (1.0, 0.0, 0.0), 1.0, "odom"),
            FactorRecord(
                "inter_robot",
                ("robot_a", 1),
                ("robot_b", 0),
                (1.0, 1.0, math.pi / 2.0),
                0.5,
                "robust",
                "m0",
            ),
        ],
        raw_inter_robot_matches=3,
        rejected_inter_robot_matches=2,
    )


def test_auto_backend_records_dependency_blocker_when_gtsam_missing(tmp_path: Path) -> None:
    backend = TeamPoseGraphBackend(
        backend="auto",
        export_dir=tmp_path,
        metrics_path=tmp_path / "team_pose_graph_metrics.json",
    )
    backend._python_gtsam_available = lambda: False
    backend._cpp_gtsam_available = lambda: False

    metrics = backend.optimize_or_export(_snapshot())

    assert metrics["optimization_backend"] == "g2o_export_only"
    assert metrics["optimization_success"] is False
    assert metrics["dependency_blocker"] == "python_gtsam_not_found;gtsam_cpp_not_found"
    assert (tmp_path / "team_pose_graph.g2o").exists()
    assert (tmp_path / "team_pose_graph_factors.json").exists()


def test_factor_payload_matches_export_schema() -> None:
    payload = snapshot_to_factor_payload(_snapshot())

    assert payload["schema"] == "team_pose_graph_factors/v1"
    assert payload["gt_used_runtime"] is False
    assert len(payload["poses"]) == 3
    assert any(f["factor_type"] == "inter_robot" and f["match_id"] == "m0" for f in payload["factors"])
    json.dumps(payload)
