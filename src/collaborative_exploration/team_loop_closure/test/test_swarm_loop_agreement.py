import math

from team_loop_closure.common import se2_from_xyyaw
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
