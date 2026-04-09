#!/usr/bin/env python3
"""Interactive CLI for switching the semantic target prompt."""

from __future__ import annotations

import argparse
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class SemanticPromptCli(Node):
    """Publish semantic prompts to the SAM3 prompt topic."""

    def __init__(self, prompt_topic: str, enable_topic: str, enable_on_send: bool):
        super().__init__("semantic_prompt_cli")
        self.pub_prompt = self.create_publisher(String, prompt_topic, 10)
        self.pub_enable = self.create_publisher(Bool, enable_topic, 10)
        self.enable_on_send = enable_on_send

    def send_prompt(self, text: str):
        msg = String()
        msg.data = text
        self.pub_prompt.publish(msg)
        if self.enable_on_send:
            enable_msg = Bool()
            enable_msg.data = True
            self.pub_enable.publish(enable_msg)
        print(f"[prompt] {text}", flush=True)


def build_parser():
    parser = argparse.ArgumentParser(description="Send semantic tracking prompts")
    parser.add_argument(
        "--prompt-topic",
        default="/sam3/prompt",
        help="Topic used to switch the SAM3 target prompt",
    )
    parser.add_argument(
        "--once",
        default="",
        help="Send one prompt and exit",
    )
    parser.add_argument(
        "--enable-topic",
        default="/llm_task/tracking_enabled",
        help="Bool topic used to enable tracking when a prompt is sent",
    )
    parser.add_argument(
        "--no-enable-on-send",
        action="store_true",
        help="Only publish the semantic prompt without enabling tracking",
    )
    return parser


def run_interactive(node: SemanticPromptCli):
    print("Semantic prompt shell is ready.", flush=True)
    print("Type a target prompt such as 'red cube', 'blue cube', or 'yellow cube'.", flush=True)
    print("Commands: :quit, :exit, :help", flush=True)

    while True:
        try:
            line = input("prompt> ").strip()
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
            print("Examples: red cube | blue cube | yellow cube | green cube", flush=True)
            continue
        node.send_prompt(line)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)

    rclpy.init()
    node = SemanticPromptCli(
        prompt_topic=args.prompt_topic,
        enable_topic=args.enable_topic,
        enable_on_send=not args.no_enable_on_send,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(0.2)

    try:
        if args.once:
            node.send_prompt(args.once)
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
