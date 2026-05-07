#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


def _field_value(msg: Any, field: str) -> Any:
    cur = msg
    for part in field.split("."):
        if not part:
            continue
        cur = getattr(cur, part)
    return cur


class TopicFieldRecorder(Node):
    def __init__(self, topic: str, field: str, wait_sec: float) -> None:
        super().__init__("cross_loop_topic_field_recorder")
        self.topic = topic
        self.field = field
        self.wait_sec = wait_sec
        self.subscribed = False
        self.deadline = time.monotonic() + wait_sec
        self.timer = self.create_timer(0.2, self._try_subscribe)

    def _try_subscribe(self) -> None:
        if self.subscribed:
            return
        for name, types in self.get_topic_names_and_types():
            if name != self.topic or not types:
                continue
            msg_type = get_message(types[0])
            self.create_subscription(msg_type, self.topic, self._on_msg, 10)
            self.subscribed = True
            self.timer.cancel()
            return
        if time.monotonic() >= self.deadline:
            print(f"Timed out waiting for topic {self.topic}", file=sys.stderr, flush=True)
            rclpy.shutdown()

    def _on_msg(self, msg: Any) -> None:
        try:
            value = _field_value(msg, self.field)
        except AttributeError as exc:
            print(
                f"Failed to read field {self.field!r} from {self.topic}: {exc}",
                file=sys.stderr,
                flush=True,
            )
            return
        print(str(value), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--field", required=True)
    parser.add_argument("--wait-sec", type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = TopicFieldRecorder(args.topic, args.field, args.wait_sec)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
