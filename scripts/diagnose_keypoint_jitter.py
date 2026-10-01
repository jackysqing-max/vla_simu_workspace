#!/usr/bin/env python3
"""Measure mask/keypoint/axis jitter for the GMS keypoint demo."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


def finite_norm(values):
    array = np.asarray(values, dtype=np.float64)
    if array.size == 0:
        return float("nan")
    return float(np.linalg.norm(array))


def angle_deg(a, b, *, unsigned_axis=False):
    va = np.asarray(a, dtype=np.float64)
    vb = np.asarray(b, dtype=np.float64)
    na = np.linalg.norm(va)
    nb = np.linalg.norm(vb)
    if na <= 1e-9 or nb <= 1e-9:
        return float("nan")
    dot = float(np.dot(va / na, vb / nb))
    if unsigned_axis:
        dot = abs(dot)
    dot = max(-1.0, min(1.0, dot))
    return float(math.degrees(math.acos(dot)))


def summarize(values):
    finite = [float(v) for v in values if math.isfinite(float(v))]
    if not finite:
        return {"n": 0}
    finite_sorted = sorted(finite)
    return {
        "n": len(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "p95": finite_sorted[min(len(finite_sorted) - 1, int(0.95 * (len(finite_sorted) - 1)))],
        "max": max(finite),
    }


class KeypointJitterProbe(Node):
    def __init__(self, args):
        super().__init__("keypoint_jitter_probe")
        self.args = args
        self.last_mask = None
        self.last_mask_centroid = None
        self.last_selected = None
        self.last_raw_selected = None
        self.last_normal = None
        self.last_tangent = None
        self.last_metadata_stamp = None

        self.mask_areas = []
        self.mask_centroid_jumps = []
        self.mask_ious = []
        self.scores = []
        self.selected_steps_m = []
        self.raw_selected_steps_m = []
        self.normal_signed_angles = []
        self.normal_unsigned_angles = []
        self.normal_sign_flips = 0
        self.tangent_signed_angles = []
        self.tangent_unsigned_angles = []
        self.tangent_sign_flips = 0
        self.selected_index_values = []
        self.selected_index_changes = 0
        self.plane_counts = []
        self.plane_rms = []
        self.candidate_counts = []
        self.raw_to_stable_distance_m = []

        self.create_subscription(Image, args.mask_topic, self.on_mask, 10)
        self.create_subscription(Float32, args.score_topic, self.on_score, 10)
        self.create_subscription(PointStamped, args.keypoint_topic, self.on_keypoint, 10)
        self.create_subscription(Vector3Stamped, args.normal_topic, self.on_normal, 10)
        self.create_subscription(Vector3Stamped, args.tangent_topic, self.on_tangent, 10)
        self.create_subscription(String, args.metadata_topic, self.on_metadata, 10)
        self.pub_prompt = self.create_publisher(String, args.prompt_topic, 10)

    def publish_prompt(self):
        if not self.args.prompt:
            return
        msg = String()
        msg.data = self.args.prompt
        self.pub_prompt.publish(msg)

    def _mask_array(self, msg: Image):
        if msg.height <= 0 or msg.width <= 0:
            return None
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if data.size < msg.height * msg.step:
            return None
        rows = data[: msg.height * msg.step].reshape(msg.height, msg.step)
        return rows[:, : msg.width] > 0

    def on_mask(self, msg: Image):
        mask = self._mask_array(msg)
        if mask is None:
            return
        area = int(mask.sum())
        self.mask_areas.append(area)
        if area > 0:
            ys, xs = np.nonzero(mask)
            centroid = np.array([float(xs.mean()), float(ys.mean())], dtype=np.float64)
            if self.last_mask_centroid is not None:
                self.mask_centroid_jumps.append(finite_norm(centroid - self.last_mask_centroid))
            self.last_mask_centroid = centroid
        if self.last_mask is not None:
            intersection = int(np.logical_and(mask, self.last_mask).sum())
            union = int(np.logical_or(mask, self.last_mask).sum())
            if union > 0:
                self.mask_ious.append(float(intersection) / float(union))
        self.last_mask = mask

    def on_score(self, msg: Float32):
        self.scores.append(float(msg.data))

    def on_keypoint(self, msg: PointStamped):
        point = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=np.float64)
        if not np.all(np.isfinite(point)):
            return
        if self.last_selected is not None:
            self.selected_steps_m.append(finite_norm(point - self.last_selected))
        if self.last_raw_selected is not None:
            self.raw_to_stable_distance_m.append(finite_norm(point - self.last_raw_selected))
        self.last_selected = point

    def on_normal(self, msg: Vector3Stamped):
        vector = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64)
        if not np.all(np.isfinite(vector)):
            return
        if self.last_normal is not None:
            dot = float(np.dot(vector, self.last_normal))
            if dot < 0.0:
                self.normal_sign_flips += 1
            self.normal_signed_angles.append(angle_deg(vector, self.last_normal))
            self.normal_unsigned_angles.append(angle_deg(vector, self.last_normal, unsigned_axis=True))
        self.last_normal = vector

    def on_tangent(self, msg: Vector3Stamped):
        vector = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64)
        if not np.all(np.isfinite(vector)):
            return
        if self.last_tangent is not None:
            dot = float(np.dot(vector, self.last_tangent))
            if dot < 0.0:
                self.tangent_sign_flips += 1
            self.tangent_signed_angles.append(angle_deg(vector, self.last_tangent))
            self.tangent_unsigned_angles.append(angle_deg(vector, self.last_tangent, unsigned_axis=True))
        self.last_tangent = vector

    def on_metadata(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        selected_index = int(payload.get("selected_index", -1))
        if self.selected_index_values and selected_index != self.selected_index_values[-1]:
            self.selected_index_changes += 1
        self.selected_index_values.append(selected_index)
        candidates = payload.get("candidates", [])
        self.candidate_counts.append(len(candidates))
        if 0 <= selected_index < len(candidates):
            raw_xyz = np.asarray(candidates[selected_index].get("xyz", []), dtype=np.float64)
            if raw_xyz.shape == (3,) and np.all(np.isfinite(raw_xyz)):
                if self.last_raw_selected is not None:
                    self.raw_selected_steps_m.append(finite_norm(raw_xyz - self.last_raw_selected))
                if self.last_selected is not None:
                    self.raw_to_stable_distance_m.append(finite_norm(self.last_selected - raw_xyz))
                self.last_raw_selected = raw_xyz
        plane = payload.get("object_plane")
        if isinstance(plane, dict):
            try:
                self.plane_counts.append(int(plane.get("count", 0)))
                self.plane_rms.append(float(plane.get("rms", float("nan"))))
            except Exception:
                pass

    def report(self):
        area_stats = summarize(self.mask_areas)
        iou_stats = summarize(self.mask_ious)
        centroid_stats = summarize(self.mask_centroid_jumps)
        selected_stats = summarize([1000.0 * value for value in self.selected_steps_m])
        raw_selected_stats = summarize([1000.0 * value for value in self.raw_selected_steps_m])
        raw_to_stable_stats = summarize([1000.0 * value for value in self.raw_to_stable_distance_m])
        normal_signed = summarize(self.normal_signed_angles)
        normal_unsigned = summarize(self.normal_unsigned_angles)
        tangent_signed = summarize(self.tangent_signed_angles)
        tangent_unsigned = summarize(self.tangent_unsigned_angles)
        print("=== Keypoint Jitter Diagnostic ===")
        print(f"duration_sec: {self.args.duration_sec:.1f}")
        print(f"prompt: {self.args.prompt!r}")
        print(f"mask_samples: {len(self.mask_areas)} score_samples: {len(self.scores)}")
        if self.scores:
            print(f"score: {summarize(self.scores)}")
        print(f"mask_area_px: {area_stats}")
        print(f"mask_iou_frame_to_frame: {iou_stats}")
        print(f"mask_centroid_step_px: {centroid_stats}")
        print(f"candidate_count: {summarize(self.candidate_counts)}")
        print(
            "selected_index: "
            f"n={len(self.selected_index_values)} changes={self.selected_index_changes} "
            f"unique={sorted(set(self.selected_index_values)) if self.selected_index_values else []}"
        )
        print(f"raw_selected_step_mm: {raw_selected_stats}")
        print(f"stable_selected_step_mm: {selected_stats}")
        print(f"raw_to_stable_distance_mm: {raw_to_stable_stats}")
        print(f"plane_count: {summarize(self.plane_counts)}")
        print(f"plane_rms_m: {summarize(self.plane_rms)}")
        print(
            "normal_angle_deg: "
            f"signed={normal_signed} unsigned_axis={normal_unsigned} sign_flips={self.normal_sign_flips}"
        )
        print(
            "tangent_angle_deg: "
            f"signed={tangent_signed} unsigned_axis={tangent_unsigned} sign_flips={self.tangent_sign_flips}"
        )

        reasons = []
        if iou_stats.get("n", 0) and iou_stats.get("mean", 1.0) < 0.90:
            reasons.append("mask is unstable: low frame-to-frame IoU")
        if centroid_stats.get("n", 0) and centroid_stats.get("p95", 0.0) > 10.0:
            reasons.append("mask centroid is moving noticeably")
        if raw_selected_stats.get("n", 0) and raw_selected_stats.get("p95", 0.0) > 20.0:
            reasons.append("raw selected candidate jumps before smoothing")
        if selected_stats.get("n", 0) and selected_stats.get("p95", 0.0) > 10.0:
            reasons.append("published selected keypoint still jumps after smoothing")
        if self.selected_index_changes > max(2, 0.2 * max(1, len(self.selected_index_values))):
            reasons.append("selected candidate index changes frequently")
        if (
            tangent_signed.get("n", 0)
            and tangent_signed.get("p95", 0.0) > 90.0
            and tangent_unsigned.get("p95", 180.0) < 20.0
        ):
            reasons.append("tangent has PCA sign flips; align axis with previous frame")
        if (
            normal_signed.get("n", 0)
            and normal_signed.get("p95", 0.0) > 90.0
            and normal_unsigned.get("p95", 180.0) < 20.0
        ):
            reasons.append("normal has sign flips; align normal with previous/reference frame")
        if not reasons:
            reasons.append("no large jitter detected in sampled topics; inspect tracker/overlay timing next")
        print("likely_causes:")
        for reason in reasons:
            print(f"- {reason}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--prompt", default="left silver surgical instrument")
    parser.add_argument("--prompt-topic", default="/sam3/prompt")
    parser.add_argument("--mask-topic", default="/sam3/mask")
    parser.add_argument("--score-topic", default="/sam3/score")
    parser.add_argument("--keypoint-topic", default="/gms_keypoints/selected_keypoint_3d")
    parser.add_argument("--normal-topic", default="/gms_keypoints/object_plane_normal")
    parser.add_argument("--tangent-topic", default="/gms_keypoints/object_plane_tangent")
    parser.add_argument("--metadata-topic", default="/gms_keypoints/metadata_json")
    parser.add_argument("--prompt-period-sec", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = KeypointJitterProbe(args)
    start = time.monotonic()
    next_prompt = start
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration_sec:
            now = time.monotonic()
            if args.prompt and now >= next_prompt:
                node.publish_prompt()
                next_prompt = now + max(args.prompt_period_sec, 0.1)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
