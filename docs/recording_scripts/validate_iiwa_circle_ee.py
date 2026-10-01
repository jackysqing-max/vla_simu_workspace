#!/usr/bin/env python3
"""Validate the actual iiwa end-effector path by fitting FK samples to a circle."""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import pybullet as p
import pybullet_data
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


AXES_BY_PLANE = {
    "xy": (0, 1, 2),
    "xz": (0, 2, 1),
    "yz": (1, 2, 0),
}


class CircleEEValidator(Node):
    def __init__(self, plane: str):
        super().__init__("iiwa_circle_ee_validator")
        self.n = 7
        self.plane = plane
        self.samples = []
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
        self.ee_link = self.n - 1
        self.create_subscription(JointState, "/iiwa7/joint_states", self.on_joint_state, 10)

    def on_joint_state(self, msg: JointState):
        if not msg.position or len(msg.position) < self.n:
            return
        for joint_index, joint_value in enumerate(msg.position[: self.n]):
            p.resetJointState(self.robot_id, joint_index, float(joint_value))
        link_state = p.getLinkState(
            self.robot_id,
            self.ee_link,
            computeForwardKinematics=True,
        )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((stamp, tuple(float(value) for value in link_state[4])))

    def close(self):
        try:
            p.disconnect(self.client)
        except Exception:
            pass


def fit_circle(points_2d):
    uv = np.asarray(points_2d, dtype=float)
    u = uv[:, 0]
    v = uv[:, 1]
    a = np.column_stack((2.0 * u, 2.0 * v, np.ones_like(u)))
    b = u * u + v * v
    cu, cv, c = np.linalg.lstsq(a, b, rcond=None)[0]
    radii = np.sqrt((u - cu) ** 2 + (v - cv) ** 2)
    radius = float(np.mean(radii))
    residual = radii - radius
    return float(cu), float(cv), radius, residual


def summarize(samples, plane: str, expected_radius: float):
    axis_u, axis_v, axis_w = AXES_BY_PLANE[plane]
    points = np.asarray([sample[1] for sample in samples], dtype=float)
    uv = points[:, [axis_u, axis_v]]
    w = points[:, axis_w]
    cu, cv, radius, residual = fit_circle(uv)
    traveled = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    return {
        "samples": len(samples),
        "duration": float(samples[-1][0] - samples[0][0]),
        "center_u": cu,
        "center_v": cv,
        "radius": radius,
        "expected_radius": expected_radius,
        "radius_error": radius - expected_radius,
        "radial_rms": float(math.sqrt(np.mean(residual * residual))),
        "radial_max_abs": float(np.max(np.abs(residual))),
        "plane_span": float(np.max(w) - np.min(w)),
        "traveled": traveled,
    }


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--plane", default="xy", choices=sorted(AXES_BY_PLANE))
    parser.add_argument("--expected-radius", type=float, default=0.035)
    parser.add_argument("--duration-sec", type=float, default=14.0)
    parser.add_argument("--warmup-sec", type=float, default=1.0)
    ns = parser.parse_args(args=args)

    rclpy.init()
    node = CircleEEValidator(ns.plane)
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < ns.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
        cutoff = node.samples[0][0] + ns.warmup_sec if node.samples else 0.0
        samples = [sample for sample in node.samples if sample[0] >= cutoff]
        if len(samples) < 20:
            raise RuntimeError(f"Not enough FK samples: {len(samples)}")
        stats = summarize(samples, ns.plane, ns.expected_radius)
        print(
            "[EE_CIRCLE] "
            f"samples={stats['samples']} duration={stats['duration']:.2f}s "
            f"plane={ns.plane} radius={stats['radius']:.4f}m "
            f"expected={stats['expected_radius']:.4f}m "
            f"radius_error={stats['radius_error'] * 1000.0:.1f}mm "
            f"radial_rms={stats['radial_rms'] * 1000.0:.1f}mm "
            f"radial_max={stats['radial_max_abs'] * 1000.0:.1f}mm "
            f"plane_span={stats['plane_span'] * 1000.0:.1f}mm "
            f"path_len={stats['traveled']:.3f}m"
        )
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
