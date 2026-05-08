#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Iterable

import rospy
from geometry_msgs.msg import TransformStamped
from swarm_msgs.msg import GlobalExtrinsicStatus


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


def _norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(v) * float(v) for v in values))


class GlobalExtrinsicToTf:
    def __init__(self) -> None:
        self.source_topic = rospy.get_param("~source_topic", "/global_extrinsic_to_teammate")
        self.output_topic = rospy.get_param(
            "~output_topic", "/robot_a/swarm_lio2_raw/relative_transform"
        )
        self.source_drone_id = int(rospy.get_param("~source_drone_id", 1))
        self.target_drone_id = int(rospy.get_param("~target_drone_id", 2))
        self.parent_frame = rospy.get_param("~parent_frame", "robot_a/map")
        self.child_frame = rospy.get_param("~child_frame", "robot_b/map")
        self.min_translation_norm = float(rospy.get_param("~min_translation_norm", 0.001))

        self.pub = rospy.Publisher(self.output_topic, TransformStamped, queue_size=10)
        self.sub = rospy.Subscriber(
            self.source_topic, GlobalExtrinsicStatus, self._on_extrinsic, queue_size=20
        )
        rospy.loginfo(
            "swarm_lio2_global_extrinsic_to_tf: %s drone_id=%d teammate_id=%d -> %s",
            self.source_topic,
            self.source_drone_id,
            self.target_drone_id,
            self.output_topic,
        )

    def _on_extrinsic(self, msg: GlobalExtrinsicStatus) -> None:
        if int(msg.drone_id) != self.source_drone_id:
            return
        for extrinsic in msg.extrinsic:
            if int(extrinsic.teammate_id) != self.target_drone_id:
                continue
            if _norm(extrinsic.trans) < self.min_translation_norm:
                return
            out = TransformStamped()
            out.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
            out.header.frame_id = self.parent_frame
            out.child_frame_id = self.child_frame
            out.transform.translation.x = float(extrinsic.trans[0])
            out.transform.translation.y = float(extrinsic.trans[1])
            out.transform.translation.z = float(extrinsic.trans[2])
            roll, pitch, yaw = (math.radians(float(v)) for v in extrinsic.rot_deg)
            qx, qy, qz, qw = _quat_from_rpy(roll, pitch, yaw)
            out.transform.rotation.x = qx
            out.transform.rotation.y = qy
            out.transform.rotation.z = qz
            out.transform.rotation.w = qw
            self.pub.publish(out)
            return


def main() -> None:
    rospy.init_node("swarm_lio2_global_extrinsic_to_tf")
    GlobalExtrinsicToTf()
    rospy.spin()


if __name__ == "__main__":
    main()
