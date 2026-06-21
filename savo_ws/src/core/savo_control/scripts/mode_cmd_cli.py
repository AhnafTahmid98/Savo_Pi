#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish a control mode command for Robot Savo."""

from __future__ import annotations

import argparse
import json
import sys
import time

from savo_control.models import ControlMode, VALID_CONTROL_MODES, normalize_mode
from savo_control.ros import CONTROL_MODE_CMD


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a mode command to the savo_control mode manager."
    )
    parser.add_argument(
        "mode",
        nargs="?",
        help="Mode to publish: STOP, MANUAL, AUTO, NAV, or RECOVERY.",
    )
    parser.add_argument(
        "--topic",
        default=CONTROL_MODE_CMD,
        help=f"Mode command topic. Default: {CONTROL_MODE_CMD}",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=3,
        help="Number of messages to publish.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="Publish rate while repeating.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print without starting ROS.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print machine-readable output.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List valid modes and exit.",
    )
    return parser.parse_args(argv)


def build_result(mode: str, topic: str, repeat: int, dry_run: bool) -> dict:
    return {
        "package": "savo_control",
        "topic": topic,
        "mode": mode,
        "repeat": repeat,
        "dry_run": dry_run,
    }


def print_result(result: dict, *, as_json: bool) -> None:
    if as_json:
        print(json.dumps(result, indent=2, sort_keys=True))
        return

    action = "validated" if result["dry_run"] else "published"
    print(
        f"savo_control mode command {action}: "
        f"mode={result['mode']} topic={result['topic']} repeat={result['repeat']}"
    )


def publish_mode(mode: str, topic: str, repeat: int, hz: float) -> None:
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("mode_cmd_cli")

    try:
        publisher = node.create_publisher(String, topic, 10)

        msg = String()
        msg.data = mode

        count = max(1, repeat)
        period_s = 1.0 / hz if hz > 0.0 else 0.2

        # Give discovery a short moment on real ROS networks.
        time.sleep(0.2)

        for _ in range(count):
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(period_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if args.list:
        for mode in VALID_CONTROL_MODES:
            print(mode)
        return 0

    if not args.mode:
        print("ERROR: mode is required unless --list is used", file=sys.stderr)
        return 2

    try:
        mode = normalize_mode(args.mode)
    except ValueError as exc:
        valid = ", ".join(ControlMode.values())
        print(f"ERROR: {exc}. Valid modes: {valid}", file=sys.stderr)
        return 2

    repeat = max(1, args.repeat)
    result = build_result(
        mode=mode,
        topic=args.topic,
        repeat=repeat,
        dry_run=args.dry_run,
    )

    if not args.dry_run:
        publish_mode(mode, args.topic, repeat, args.hz)

    print_result(result, as_json=args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
