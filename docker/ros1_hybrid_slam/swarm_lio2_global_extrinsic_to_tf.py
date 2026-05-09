#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Iterable

import rospy
from geometry_msgs.msg import TransformStamped
from swarm_msgs.msg import GlobalExtrinsicStatus


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(v)) for v in values)


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 0.0 or not math.isfinite(norm):
        return 0.0, 0.0, 0.0, 1.0
    return tuple(v / norm for v in q)


def _quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    return -x, -y, -z, w


def _quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(
    q: tuple[float, float, float, float],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = _quat_multiply(_quat_multiply(q, (v[0], v[1], v[2], 0.0)), _quat_conjugate(q))
    return rotated[0], rotated[1], rotated[2]


def _norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in values))


class GlobalExtrinsicToTf:
    def __init__(self) -> None:
        source_topics = str(
            rospy.get_param(
                "~source_topics",
                "/global_extrinsic_to_teammate,/global_extrinsic_from_teammate",
            )
        )
        self.source_topics = [topic.strip() for topic in source_topics.split(",") if topic.strip()]
        self.output_topic = rospy.get_param(
            "~output_topic", "/robot_a/swarm_lio2_raw/relative_transform"
        )
        self.parent_drone_id = int(rospy.get_param("~parent_drone_id", 1))
        self.child_drone_id = int(rospy.get_param("~child_drone_id", 2))
        self.parent_frame = rospy.get_param("~parent_frame", "robot_a/map")
        self.child_frame = rospy.get_param("~child_frame", "robot_b/map")
        self.min_translation_norm = float(rospy.get_param("~min_translation_norm", 0.001))

        self.pub = rospy.Publisher(self.output_topic, TransformStamped, queue_size=10)
        self.subs = [
            rospy.Subscriber(topic, GlobalExtrinsicStatus, self._on_extrinsic, queue_size=20)
            for topic in self.source_topics
        ]
        rospy.loginfo(
            "swarm_lio2_global_extrinsic_to_tf: %s parent_drone_id=%d child_drone_id=%d -> %s",
            ",".join(self.source_topics),
            self.parent_drone_id,
            self.child_drone_id,
            self.output_topic,
        )

    def _on_extrinsic(self, msg: GlobalExtrinsicStatus) -> None:
        source_drone_id = int(msg.drone_id)
        if not msg.extrinsic:
            rospy.logwarn_throttle(
                5.0,
                "swarm_lio2_global_extrinsic_to_tf: empty extrinsic[] from drone_id=%d",
                source_drone_id,
            )
            return
        matched_pair = False
        for extrinsic in msg.extrinsic:
            teammate_id = int(extrinsic.teammate_id)
            if not self._matches_requested_pair(source_drone_id, teammate_id):
                continue
            matched_pair = True
            translation = tuple(float(v) for v in extrinsic.trans)
            rotation_deg = tuple(float(v) for v in extrinsic.rot_deg)
            if not _finite((*translation, *rotation_deg)):
                continue
            if _norm(translation) < self.min_translation_norm:
                continue
            rotation = _quat_normalize(
                _quat_from_rpy(*(math.radians(v) for v in rotation_deg))
            )
            if source_drone_id == self.child_drone_id:
                rotation, translation = self._invert_transform(rotation, translation)
            out = TransformStamped()
            out.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
            out.header.frame_id = self.parent_frame
            out.child_frame_id = self.child_frame
            out.transform.translation.x = translation[0]
            out.transform.translation.y = translation[1]
            out.transform.translation.z = translation[2]
            qx, qy, qz, qw = rotation
            out.transform.rotation.x = qx
            out.transform.rotation.y = qy
            out.transform.rotation.z = qz
            out.transform.rotation.w = qw
            self.pub.publish(out)
            return
        if not matched_pair:
            rospy.logwarn_throttle(
                5.0,
                "swarm_lio2_global_extrinsic_to_tf: no requested drone pair in extrinsic[] from drone_id=%d",
                source_drone_id,
            )

    def _matches_requested_pair(self, source_drone_id: int, teammate_id: int) -> bool:
        direct = (
            source_drone_id == self.parent_drone_id
            and teammate_id == self.child_drone_id
        )
        inverse = (
            source_drone_id == self.child_drone_id
            and teammate_id == self.parent_drone_id
        )
        return direct or inverse

    def _invert_transform(
        self,
        rotation: tuple[float, float, float, float],
        translation: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float, float], tuple[float, float, float]]:
        inverse_rotation = _quat_normalize(_quat_conjugate(rotation))
        rotated = _rotate_vector(inverse_rotation, translation)
        return inverse_rotation, (-rotated[0], -rotated[1], -rotated[2])


def main() -> None:
    rospy.init_node("swarm_lio2_global_extrinsic_to_tf")
    GlobalExtrinsicToTf()
    rospy.spin()


if __name__ == "__main__":
    main()
