#!/usr/bin/env python3
"""Set the iiwa PyBullet control mode after an optional startup delay."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


MODE_BY_NAME = {
    "free": 0,
    "position": 1,
    "torque": 2,
}


class ControlModeSetter(Node):
    def __init__(self, mode: int, repeats: int, interval_sec: float):
        super().__init__("iiwa_control_mode_setter")
        self.mode = int(mode)
        self.repeats = max(1, int(repeats))
        self.interval_sec = max(0.05, float(interval_sec))
        self.pub = self.create_publisher(Int8, "/iiwa7/control_mode", 10)

    def run(self):
        msg = Int8()
        msg.data = self.mode
        for index in range(self.repeats):
            self.pub.publish(msg)
            self.get_logger().info(
                f"Published /iiwa7/control_mode={self.mode} ({index + 1}/{self.repeats})"
            )
            rclpy.spin_once(self, timeout_sec=self.interval_sec)


def parse_mode(value: str) -> int:
    key = value.strip().lower()
    if key in MODE_BY_NAME:
        return MODE_BY_NAME[key]
    return int(value)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="position")
    parser.add_argument("--delay-sec", type=float, default=3.0)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--interval-sec", type=float, default=0.2)
    ns = parser.parse_args(args=args)

    if ns.delay_sec > 0.0:
        time.sleep(ns.delay_sec)

    rclpy.init()
    node = ControlModeSetter(parse_mode(ns.mode), ns.repeats, ns.interval_sec)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
