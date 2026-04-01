#!/usr/bin/env python3
"""Interactive CLI for sending natural-language robot tasks."""

from __future__ import annotations

import argparse
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class LlmTaskCli(Node):
    """Publish free-form task text and print planner/executor status updates."""

    def __init__(self, instruction_topic: str, status_topic: str, echo_status: bool):
        super().__init__("llm_task_cli")
        self.echo_status = echo_status
        self.pub_instruction = self.create_publisher(String, instruction_topic, 10)
        self.sub_status = self.create_subscription(
            String,
            status_topic,
            self.on_status,
            10,
        )

    def on_status(self, msg: String):
        if self.echo_status:
            print(f"[status] {msg.data}", flush=True)

    def send_instruction(self, text: str):
        msg = String()
        msg.data = text
        self.pub_instruction.publish(msg)
        print(f"[sent] {text}", flush=True)


def build_parser():
    parser = argparse.ArgumentParser(description="Send natural-language tabletop tasks")
    parser.add_argument(
        "--instruction-topic",
        default="/llm_task/instruction",
        help="Topic that accepts natural-language instructions",
    )
    parser.add_argument(
        "--status-topic",
        default="/llm_task/status",
        help="Topic used by the planner/executor to report progress",
    )
    parser.add_argument(
        "--once",
        default="",
        help="Send one instruction and exit",
    )
    parser.add_argument(
        "--no-status",
        action="store_true",
        help="Do not print planner/executor status updates",
    )
    return parser


def run_interactive(node: LlmTaskCli):
    print("LLM task shell is ready.", flush=True)
    print("Type a natural-language instruction and press Enter.", flush=True)
    print("Commands: :quit, :exit, :help", flush=True)

    while True:
        try:
            line = input("task> ").strip()
        except EOFError:
            print("", flush=True)
            break
        except KeyboardInterrupt:
            print("", flush=True)
            break

        if not line:
            continue
        if line in {":quit", ":exit"}:
            break
        if line == ":help":
            print("Example: 依次移动到红色方块、蓝色方块和黄色方块上方", flush=True)
            continue
        node.send_instruction(line)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    rclpy.init()
    node = LlmTaskCli(
        instruction_topic=args.instruction_topic,
        status_topic=args.status_topic,
        echo_status=not args.no_status,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Give ROS discovery a brief moment so the first publish is not dropped.
    time.sleep(0.2)

    try:
        if args.once:
            node.send_instruction(args.once)
            time.sleep(0.5)
        else:
            run_interactive(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
