#!/usr/bin/env python3
"""Send one VLM-RCM language command and print the selected hole only.

This is intentionally perception-only: it publishes the same language command
that the surgical task executor would hand to the port detector, but it never
publishes robot start or pivot commands.
"""

from __future__ import annotations

import argparse
import json
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped, Vector3Stamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

def fmt_float(value, digits=4, default="n/a"):
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return default


class LocateCapture(Node):
    def __init__(
        self,
        *,
        language_topic: str,
        language_control_topic: str,
        expected_instruction: str,
        ready_topic: str,
        status_topic: str,
        candidates_topic: str,
        verification_topic: str,
        locked_point_topic: str,
        locked_surface_axis_topic: str,
    ):
        super().__init__("vlm_rcm_locate_capture")
        self.language_topic = language_topic
        self.candidates_topic = candidates_topic
        self.expected_instruction = expected_instruction
        self.control_event = threading.Event()
        self.control_payload = {}
        self.ready_event = threading.Event()
        self.target_not_found_event = threading.Event()
        self.candidate_event = threading.Event()
        self.locked_point_event = threading.Event()
        self.locked_surface_axis_event = threading.Event()
        self.status_text = ""
        self.candidates_text = ""
        self.verification_text = ""
        self.locked_point = None
        self.locked_surface_axis = None

        self.pub_language = self.create_publisher(String, language_topic, 10)

        self.create_subscription(
            String,
            language_control_topic,
            self.on_language_control,
            10,
        )
        self.create_subscription(Bool, ready_topic, self.on_ready, 10)
        self.create_subscription(String, status_topic, self.on_status, 10)
        self.create_subscription(String, candidates_topic, self.on_candidates, 10)
        self.create_subscription(String, verification_topic, self.on_verification, 10)
        self.create_subscription(
            PointStamped,
            locked_point_topic,
            self.on_locked_point,
            10,
        )
        self.create_subscription(
            Vector3Stamped,
            locked_surface_axis_topic,
            self.on_locked_surface_axis,
            10,
        )

    def on_ready(self, msg: Bool):
        if msg.data:
            self.ready_event.set()
        else:
            self.ready_event.clear()

    def on_language_control(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("instruction") != self.expected_instruction:
            return
        if str(payload.get("action", "")).lower() == "locate":
            self.ready_event.clear()
            self.candidate_event.clear()
            self.locked_point_event.clear()
            self.locked_surface_axis_event.clear()
            self.target_not_found_event.clear()
            self.candidates_text = ""
            self.verification_text = ""
            self.locked_point = None
            self.locked_surface_axis = None
        self.control_payload = payload
        self.control_event.set()
        print(
            "[LLM CONTROL] "
            f"action={payload.get('action')} "
            f"target={payload.get('target_description')} "
            f"sam_prompt={payload.get('sam_prompt')}",
            flush=True,
        )

    def on_status(self, msg: String):
        self.status_text = msg.data
        expected_marker = f"instruction={self.expected_instruction!r};"
        if (
            msg.data.startswith("target_not_found:")
            and expected_marker in msg.data
        ):
            self.target_not_found_event.set()
            print(f"[TARGET NOT FOUND] {msg.data}", flush=True)
        if not self.control_event.is_set():
            return
        if "locked=true" in msg.data and not self.candidate_event.is_set():
            return
        if (
            "candidate=" in msg.data
            or "locked=true" in msg.data
            or "estimate rejected" in msg.data
        ):
            print(f"[STATUS] {msg.data}", flush=True)

    def on_candidates(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("language_instruction") != self.expected_instruction:
            return
        self.candidates_text = msg.data
        self.candidate_event.set()

    def on_verification(self, msg: String):
        try:
            payload = json.loads(msg.data)
            if payload.get("instruction") != self.expected_instruction:
                return
            self.verification_text = msg.data
            print(
                "[VERIFY] "
                f"{payload.get('decision')} "
                f"selected={payload.get('selected_candidate_id')} "
                f"reason={payload.get('reason')}",
                flush=True,
            )
        except Exception:
            print(f"[VERIFY] {msg.data}", flush=True)

    def on_locked_point(self, msg: PointStamped):
        self.locked_point = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z),
        )
        self.locked_point_event.set()

    def on_locked_surface_axis(self, msg: Vector3Stamped):
        self.locked_surface_axis = (
            float(msg.vector.x),
            float(msg.vector.y),
            float(msg.vector.z),
        )
        self.locked_surface_axis_event.set()

    def wait_for_language_subscriber(self, timeout_sec: float) -> bool:
        deadline = time.time() + max(timeout_sec, 0.0)
        while time.time() < deadline:
            if self.pub_language.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return self.pub_language.get_subscription_count() > 0

    def wait_for_candidates_publisher(self, timeout_sec: float) -> bool:
        deadline = time.time() + max(timeout_sec, 0.0)
        while time.time() < deadline:
            if self.count_publishers(self.candidates_topic) > 0:
                return True
            time.sleep(0.05)
        return self.count_publishers(self.candidates_topic) > 0

    def publish_command(self, command: str, repeat_sec: float):
        language_msg = String()
        language_msg.data = command

        deadline = time.time() + max(repeat_sec, 0.0)
        self.pub_language.publish(language_msg)
        while time.time() < deadline:
            time.sleep(0.2)
            self.pub_language.publish(language_msg)


def pretty_print_result(
    command: str,
    candidates_text: str,
    verification_text: str,
    locked_point,
    locked_surface_axis,
):
    print("\n================ LANGUAGE COMMAND ================", flush=True)
    print(command, flush=True)

    print("\n================ SELECTED HOLE ===================", flush=True)
    if not candidates_text:
        print("No /vlm_rcm/hole_candidates message was received.", flush=True)
        return

    try:
        payload = json.loads(candidates_text)
    except json.JSONDecodeError:
        print(candidates_text, flush=True)
        return

    selected_id = payload.get("selected_id")
    holes = payload.get("holes", [])
    selected = None
    for hole in holes:
        if hole.get("id") == selected_id:
            selected = hole
            break

    if verification_text:
        try:
            verification = json.loads(verification_text)
            print(
                "\n================ VERIFICATION ===================",
                flush=True,
            )
            print(
                "decision="
                f"{verification.get('decision')} "
                "selected="
                f"{verification.get('selected_candidate_id')} "
                "reason="
                f"{verification.get('reason')}",
                flush=True,
            )
        except json.JSONDecodeError:
            print("\n================ VERIFICATION ===================", flush=True)
            print(verification_text, flush=True)

    print(
        "verified="
        f"{payload.get('verified_selection', {}).get('decision', 'NONE')} "
        "selected="
        f"H{selected_id if selected_id is not None else '?'} "
        "candidates="
        f"{payload.get('count', '?')}/{payload.get('raw_count', '?')}",
        flush=True,
    )
    if payload.get("filter"):
        print(f"filter={payload.get('filter')}", flush=True)

    if selected is not None:
        center_px = selected.get("center_px", [])
        aperture_px = selected.get("aperture_center_px", center_px)
        center_world = selected.get("center_world", [])
        final_axis = selected.get("axis", [])
        support_normal = selected.get("support_normal", [])
        surface_axis = selected.get("surface_equivalent_axis", [])
        section_axis = selected.get("section_center_axis")
        selection_score = float(selected.get("selection_score", 0.0))
        selection_raw = selected.get("selection_score_raw")
        selection_rank = int(selected.get("selection_rank", 0))
        print(
            "selected_center_px="
            f"({center_px[0]:.1f}, {center_px[1]:.1f}) "
            "aperture_center_px="
            f"({aperture_px[0]:.1f}, {aperture_px[1]:.1f}) "
            "selected_center_world="
            f"({center_world[0]:.4f}, {center_world[1]:.4f}, {center_world[2]:.4f})",
            flush=True,
        )
        raw_text = "none" if selection_raw is None else f"{float(selection_raw):.4f}"
        print(
            "selected_score="
            f"{selection_score:.3f} raw={raw_text} rank={selection_rank} "
            f"source={selected.get('selection_score_source', 'none')}",
            flush=True,
        )
        print(
            "depth_geometry="
            f"offset_px={float(selected.get('depth_hole_center_offset_px', 0.0)):.1f} "
            f"overlap={float(selected.get('depth_hole_overlap_fraction', 0.0)):.2f} "
            f"area_frac={float(selected.get('depth_hole_area_fraction', 0.0)):.2f} "
            f"circularity={float(selected.get('depth_hole_circularity', 0.0)):.2f} "
            f"median_depth_m={float(selected.get('depth_hole_median_depth_m', 0.0)):.4f} "
            f"p90_depth_m={float(selected.get('depth_hole_p90_depth_m', 0.0)):.4f} "
            f"depth_to_aperture_offset_px="
            f"{float(selected.get('depth_centroid_to_aperture_offset_px', 0.0)):.1f} "
            f"aperture_source={selected.get('aperture_center_source', 'unknown')}",
            flush=True,
        )
        if len(final_axis) == 3:
            print(
                "final_axis="
                f"({final_axis[0]:.4f}, {final_axis[1]:.4f}, {final_axis[2]:.4f}) "
                f"source={selected.get('axis_source', 'unknown')}",
                flush=True,
            )
        if isinstance(section_axis, list) and len(section_axis) == 3:
            print(
                "section_center_axis="
                f"({section_axis[0]:.4f}, {section_axis[1]:.4f}, {section_axis[2]:.4f}) "
                f"sections={int(selected.get('section_center_count', 0))} "
                f"rms_m={fmt_float(selected.get('section_center_rms_m'), 4)} "
                f"depth_span_m={fmt_float(selected.get('section_center_depth_span_m'), 4)} "
                f"surface_angle_deg={fmt_float(selected.get('section_surface_angle_deg'), 2)}",
                flush=True,
            )
        if len(support_normal) == 3:
            print(
                "surface_plane_normal="
                f"({support_normal[0]:.4f}, {support_normal[1]:.4f}, {support_normal[2]:.4f})",
                flush=True,
            )
        if len(surface_axis) == 3:
            print(
                "surface_equivalent_axis="
                f"({surface_axis[0]:.4f}, {surface_axis[1]:.4f}, {surface_axis[2]:.4f}) "
                f"source={selected.get('surface_axis_source', 'unknown')} "
                "axis_surface_angle_deg="
                f"{float(selected.get('axis_surface_angle_deg', 0.0)):.2f}",
                flush=True,
            )
    if locked_point is not None:
        print(
            "locked_point_world="
            f"({locked_point[0]:.4f}, {locked_point[1]:.4f}, {locked_point[2]:.4f})",
            flush=True,
        )
    if locked_surface_axis is not None:
        print(
            "locked_surface_equivalent_axis="
            f"({locked_surface_axis[0]:.4f}, {locked_surface_axis[1]:.4f}, "
            f"{locked_surface_axis[2]:.4f})",
            flush=True,
        )

    print("\n================ ALL HOLES =======================", flush=True)
    print(
        "id | score | rank | circ | overlap | depth_m | sections | center_px(u,v) | center_world(x,y,z)",
        flush=True,
    )
    for hole in holes:
        center_px = hole.get("center_px", [0.0, 0.0])
        center_world = hole.get("center_world", [0.0, 0.0, 0.0])
        score = float(hole.get("selection_score", 0.0))
        rank = int(hole.get("selection_rank", 0))
        circularity = float(hole.get("depth_hole_circularity", 0.0))
        overlap = float(hole.get("depth_hole_overlap_fraction", 0.0))
        median_depth = float(hole.get("depth_hole_median_depth_m", 0.0))
        section_count = int(hole.get("section_center_count", 0))
        mark = "<-- selected" if hole.get("id") == selected_id else ""
        print(
            f"H{hole.get('id')} | "
            f"{score:.3f} | "
            f"{rank} | "
            f"{circularity:.2f} | "
            f"{overlap:.2f} | "
            f"{median_depth:.3f} | "
            f"{section_count} | "
            f"({center_px[0]:.1f}, {center_px[1]:.1f}) | "
            f"({center_world[0]:.4f}, {center_world[1]:.4f}, {center_world[2]:.4f}) "
            f"{mark}",
            flush=True,
        )


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="+", help="Language command for selecting a hole.")
    parser.add_argument("--language-topic", default="/vlm_rcm/language_command")
    parser.add_argument("--language-control-topic", default="/vlm_rcm/language_control")
    parser.add_argument("--ready-topic", default="/vlm_rcm/port_ready")
    parser.add_argument("--status-topic", default="/vlm_rcm/status")
    parser.add_argument("--candidates-topic", default="/vlm_rcm/hole_candidates")
    parser.add_argument("--verification-topic", default="/vlm_rcm/verification_result")
    parser.add_argument("--locked-point-topic", default="/vlm_rcm/locked_port_point")
    parser.add_argument("--locked-surface-axis-topic", default="/vlm_rcm/locked_surface_axis")
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--publish-warmup-sec", type=float, default=4.0)
    parser.add_argument("--publish-repeat-sec", type=float, default=1.0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    command = " ".join(args.command).strip()

    rclpy.init()
    node = LocateCapture(
        language_topic=args.language_topic,
        language_control_topic=args.language_control_topic,
        expected_instruction=command,
        ready_topic=args.ready_topic,
        status_topic=args.status_topic,
        candidates_topic=args.candidates_topic,
        verification_topic=args.verification_topic,
        locked_point_topic=args.locked_point_topic,
        locked_surface_axis_topic=args.locked_surface_axis_topic,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        repeat_sec = 0.0
        if not node.wait_for_language_subscriber(args.publish_warmup_sec):
            print("[WARN] no /vlm_rcm/language_command subscriber discovered yet", flush=True)
            repeat_sec = args.publish_repeat_sec
        if not node.wait_for_candidates_publisher(args.publish_warmup_sec):
            print("[WARN] no /vlm_rcm/hole_candidates publisher discovered yet", flush=True)
        print(f"[SEND] {command}", flush=True)
        print("[INFO] perception-only locate; no robot start command is published", flush=True)
        node.publish_command(command, repeat_sec)

        deadline = time.time() + max(args.timeout_sec, 0.0)
        while time.time() < deadline and not node.control_event.is_set():
            time.sleep(0.05)
        if not node.control_event.is_set():
            raise SystemExit("[ERROR] strict LLM produced no language control result")

        control_action = str(node.control_payload.get("action", "")).lower()
        if control_action == "unlock":
            print("\n================ LOCK CONTROL ====================", flush=True)
            print("RCM port lock cleared; waiting for a new target instruction.", flush=True)
            if node.status_text:
                print(f"status={node.status_text}", flush=True)
            return
        if control_action != "locate":
            raise SystemExit(
                f"[ERROR] strict LLM rejected instruction: action={control_action}"
            )

        while time.time() < deadline:
            if node.target_not_found_event.is_set():
                raise SystemExit(f"[ERROR] {node.status_text}")
            if node.ready_event.is_set() and node.candidate_event.is_set():
                break
            time.sleep(0.1)

        pretty_print_result(
            command,
            node.candidates_text,
            node.verification_text,
            node.locked_point,
            node.locked_surface_axis,
        )
        if not node.ready_event.is_set():
            raise SystemExit(
                f"[WARN] timed out after {args.timeout_sec:.1f}s before port_ready=true"
            )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
