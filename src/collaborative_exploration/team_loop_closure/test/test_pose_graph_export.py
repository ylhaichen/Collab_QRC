import json
import math
from pathlib import Path

from team_loop_closure.team_pose_graph_node import (
    FactorRecord,
    PoseGraphSnapshot,
    PoseRecord,
    TeamPoseGraphBackend,
)


def test_export_only_backend_writes_g2o_and_json(tmp_path: Path) -> None:
    snapshot = PoseGraphSnapshot(
        poses={
            ("robot_a", 0): PoseRecord("robot_a", 0, 0.0, 0.0, 0.0, 1.0),
            ("robot_a", 1): PoseRecord("robot_a", 1, 1.0, 0.0, 0.0, 2.0),
            ("robot_b", 0): PoseRecord("robot_b", 0, 0.0, 1.0, math.pi / 2.0, 1.5),
        },
        factors=[
            FactorRecord("prior", ("robot_a", 0), None, (0.0, 0.0, 0.0), 1.0, "anchor"),
            FactorRecord("odom", ("robot_a", 0), ("robot_a", 1), (1.0, 0.0, 0.0), 1.0, "odom"),
            FactorRecord("inter_robot", ("robot_a", 1), ("robot_b", 0), (0.0, 1.0, math.pi / 2.0), 0.5, "robust"),
        ],
    )
    backend = TeamPoseGraphBackend(
        backend="g2o_export_only",
        export_dir=tmp_path,
        metrics_path=tmp_path / "team_pose_graph_metrics.json",
    )

    metrics = backend.optimize_or_export(snapshot)

    assert metrics["optimization_backend"] == "g2o_export_only"
    assert metrics["optimization_success"] is False
    assert metrics["num_inter_robot_factors_inlier"] == 1
    assert (tmp_path / "team_pose_graph.g2o").exists()
    assert (tmp_path / "team_pose_graph_factors.json").exists()
    exported = json.loads((tmp_path / "team_pose_graph_factors.json").read_text())
    assert exported["schema"] == "team_pose_graph_factors/v1"
    assert any(f["factor_type"] == "inter_robot" for f in exported["factors"])
