from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from .common import invert_se2, se2_from_xyyaw, wrap_pi, xyyaw_from_se2, yaw_from_quat

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped


@dataclass(frozen=True)
class SwarmLoopAgreementResult:
    accepted: bool
    translation_error_m: float
    yaw_error_rad: float
    reason: str

    @property
    def yaw_error_deg(self) -> float:
        return math.degrees(self.yaw_error_rad)


def se2_from_transform_msg(msg: "TransformStamped") -> np.ndarray:
    t = msg.transform.translation
    q = msg.transform.rotation
    return se2_from_xyyaw(float(t.x), float(t.y), yaw_from_quat(q))


def evaluate_swarm_loop_agreement(
    swarm_transform: np.ndarray,
    loop_transform: np.ndarray,
    *,
    max_translation_m: float,
    max_yaw_rad: float,
) -> SwarmLoopAgreementResult:
    delta = invert_se2(swarm_transform) @ loop_transform
    x, y, yaw = xyyaw_from_se2(delta)
    translation_error = math.hypot(x, y)
    yaw_error = abs(wrap_pi(yaw))
    if translation_error > max_translation_m:
        return SwarmLoopAgreementResult(
            accepted=False,
            translation_error_m=translation_error,
            yaw_error_rad=yaw_error,
            reason="swarm_loop_translation_disagreement",
        )
    if yaw_error > max_yaw_rad:
        return SwarmLoopAgreementResult(
            accepted=False,
            translation_error_m=translation_error,
            yaw_error_rad=yaw_error,
            reason="swarm_loop_yaw_disagreement",
        )
    return SwarmLoopAgreementResult(
        accepted=True,
        translation_error_m=translation_error,
        yaw_error_rad=yaw_error,
        reason="swarm_loop_agreement_accepted",
    )
