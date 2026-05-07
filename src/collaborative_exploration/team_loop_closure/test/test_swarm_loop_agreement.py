import math

from team_loop_closure.common import se2_from_xyyaw
from team_loop_closure.relative_transform_manager_node import RelativeTransformManager
from team_loop_closure.swarm_loop_agreement import evaluate_swarm_loop_agreement


def test_swarm_loop_agreement_accepts_close_transforms() -> None:
    swarm = se2_from_xyyaw(1.0, -0.5, math.radians(10.0))
    loop = se2_from_xyyaw(1.18, -0.42, math.radians(12.0))

    result = evaluate_swarm_loop_agreement(
        swarm,
        loop,
        max_translation_m=0.5,
        max_yaw_rad=math.radians(5.0),
    )

    assert result.accepted
    assert result.translation_error_m < 0.5
    assert result.yaw_error_rad < math.radians(5.0)
    assert result.reason == "swarm_loop_agreement_accepted"


def test_swarm_loop_agreement_rejects_large_yaw_delta() -> None:
    swarm = se2_from_xyyaw(1.0, -0.5, math.radians(10.0))
    loop = se2_from_xyyaw(1.1, -0.45, math.radians(21.0))

    result = evaluate_swarm_loop_agreement(
        swarm,
        loop,
        max_translation_m=0.5,
        max_yaw_rad=math.radians(5.0),
    )

    assert not result.accepted
    assert result.translation_error_m < 0.5
    assert result.yaw_error_rad > math.radians(5.0)
    assert result.reason == "swarm_loop_yaw_disagreement"


def test_swarm_loop_agreement_rejects_large_translation_delta() -> None:
    swarm = se2_from_xyyaw(0.0, 0.0, 0.0)
    loop = se2_from_xyyaw(0.75, 0.0, math.radians(1.0))

    result = evaluate_swarm_loop_agreement(
        swarm,
        loop,
        max_translation_m=0.5,
        max_yaw_rad=math.radians(5.0),
    )

    assert not result.accepted
    assert result.translation_error_m > 0.5
    assert result.reason == "swarm_loop_translation_disagreement"


def test_relative_transform_manager_does_not_align_from_swarm_state_alone() -> None:
    manager = RelativeTransformManager.__new__(RelativeTransformManager)
    manager.robust_payload = None
    manager.metrics_payload = {
        "schema": "team_pose_graph_metrics/v1",
        "optimization_success": True,
        "optimization_backend": "gtsam_cpp",
        "gt_used_runtime": False,
    }
    manager.swarm_transform = se2_from_xyyaw(1.0, 0.0, 0.0)
    manager.swarm_agreement = None
    manager.current_transform = None
    manager.first_robust_stamp_sec = None
    manager.allow_export_only = False
    manager.require_swarm_agreement = True

    RelativeTransformManager._update_status(manager)

    assert manager.status == "unaligned"
    assert manager.last_reason == "waiting_for_robust_loop_inliers"
    assert manager.swarm_agreement is None
