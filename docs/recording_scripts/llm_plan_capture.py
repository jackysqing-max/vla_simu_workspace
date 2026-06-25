#!/usr/bin/env python3
"""Send one LLM task and print the resulting plan_json for OBS recording."""

from __future__ import annotations

import argparse
import json
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class PlanCapture(Node):
    def __init__(
        self,
        instruction_topic: str,
        status_topic: str,
        plan_topic: str,
    ):
        super().__init__("llm_plan_capture")
        self.plan_topic = plan_topic
        self.plan_event = threading.Event()
        self.status_event = threading.Event()
        self.plan_text = ""
        self.status_text = ""
        self.pub_instruction = self.create_publisher(String, instruction_topic, 10)
        self.sub_status = self.create_subscription(String, status_topic, self.on_status, 10)
        self.sub_plan = self.create_subscription(String, plan_topic, self.on_plan, 10)

    def on_status(self, msg: String):
        self.status_text = msg.data
        self.status_event.set()
        print(f"[STATUS] {msg.data}", flush=True)

    def on_plan(self, msg: String):
        self.plan_text = msg.data
        self.plan_event.set()

    def wait_for_instruction_subscriber(self, timeout_sec: float) -> bool:
        deadline = time.time() + max(timeout_sec, 0.0)
        while time.time() < deadline:
            if self.pub_instruction.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return self.pub_instruction.get_subscription_count() > 0

    def wait_for_plan_publisher(self, timeout_sec: float) -> bool:
        deadline = time.time() + max(timeout_sec, 0.0)
        while time.time() < deadline:
            if self.count_publishers(self.plan_topic) > 0:
                return True
            time.sleep(0.05)
        return self.count_publishers(self.plan_topic) > 0

    def publish_instruction(self, task: str, repeat_sec: float):
        msg = String()
        msg.data = task
        deadline = time.time() + max(repeat_sec, 0.0)
        self.pub_instruction.publish(msg)
        while time.time() < deadline:
            time.sleep(0.1)
            self.pub_instruction.publish(msg)


def pretty_print_plan(task: str, plan_text: str):
    print("\n================ USER TASK ================", flush=True)
    print(task, flush=True)

    print("\n================ PLAN JSON ================", flush=True)
    try:
        plan = json.loads(plan_text)
    except json.JSONDecodeError:
        print(plan_text, flush=True)
        return

    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)

    print("\n========== TARGET_PROMPT HANDOFF ==========", flush=True)
    steps = plan.get("steps", [])
    for step in steps:
        index = step.get("step_index", "?")
        action = step.get("action", "")
        prompt = step.get("target_prompt", "")
        if action in {"hover_target", "grasp_target"} and prompt:
            print(
                f"step {index}: {action} -> publish prompt to SAM3/fusion: \"{prompt}\"",
                flush=True,
            )
        elif action == "release_gripper":
            print(
                f"step {index}: release_gripper -> no new visual prompt; open gripper at current placement target",
                flush=True,
            )
        elif action == "wait":
            print(f"step {index}: wait -> no visual prompt", flush=True)
        else:
            print(f"step {index}: {action} -> prompt: \"{prompt}\"", flush=True)

    print("\n============ EXECUTION MEANING ============", flush=True)
    print(
        "The LLM does not directly move the robot. It emits a constrained JSON plan. "
        "The executor reads each step, sends target_prompt text to perception when needed, "
        "locks the grounded keypoint once stable, then runs the corresponding motion primitive.",
        flush=True,
    )


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task", nargs="+", help="Natural-language task to send.")
    parser.add_argument("--instruction-topic", default="/llm_task/instruction")
    parser.add_argument("--status-topic", default="/llm_task/status")
    parser.add_argument("--plan-topic", default="/llm_task/plan_json")
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--publish-warmup-sec", type=float, default=4.0)
    parser.add_argument("--publish-repeat-sec", type=float, default=1.0)
    parser.add_argument(
        "--discovery-settle-sec",
        type=float,
        default=1.0,
        help="Extra delay after ROS discovery before publishing the instruction.",
    )
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    task = " ".join(args.task).strip()

    rclpy.init()
    node = PlanCapture(
        instruction_topic=args.instruction_topic,
        status_topic=args.status_topic,
        plan_topic=args.plan_topic,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        repeat_sec = 0.0
        if not node.wait_for_instruction_subscriber(args.publish_warmup_sec):
            print("[WARN] no /llm_task/instruction subscriber discovered yet", flush=True)
            repeat_sec = args.publish_repeat_sec
        if not node.wait_for_plan_publisher(args.publish_warmup_sec):
            print("[WARN] no /llm_task/plan_json publisher discovered yet", flush=True)
        if args.discovery_settle_sec > 0.0:
            time.sleep(args.discovery_settle_sec)
        print(f"[SEND] {task}", flush=True)
        node.publish_instruction(task, repeat_sec)
        if not node.plan_event.wait(timeout=max(args.timeout_sec, 0.0)):
            raise SystemExit(
                f"[ERROR] timed out after {args.timeout_sec:.1f}s waiting for /llm_task/plan_json"
            )
        pretty_print_plan(task, node.plan_text)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
