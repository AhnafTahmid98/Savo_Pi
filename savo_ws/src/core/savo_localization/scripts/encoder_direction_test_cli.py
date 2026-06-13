#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Check Robot Savo encoder direction signs from wheel odometry state JSON."""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


WHEEL_NAMES = ("FL", "FR", "RL", "RR")

DIRECTION_PATTERNS: dict[str, dict[str, int]] = {
    "forward": {"FL": +1, "FR": +1, "RL": +1, "RR": +1},
    "reverse": {"FL": -1, "FR": -1, "RL": -1, "RR": -1},
    "strafe_left": {"FL": -1, "FR": +1, "RL": +1, "RR": -1},
    "strafe_right": {"FL": +1, "FR": -1, "RL": -1, "RR": +1},
    "rotate_ccw": {"FL": -1, "FR": +1, "RL": -1, "RR": +1},
    "rotate_cw": {"FL": +1, "FR": -1, "RL": +1, "RR": -1},
    "stop": {"FL": 0, "FR": 0, "RL": 0, "RR": 0},
}


@dataclass(frozen=True)
class WheelDirectionResult:
    wheel: str
    expected: int
    observed: int
    count_start: int
    count_end: int
    count_delta: int
    ok: bool
    message: str


class EncoderDirectionNode(Node):
    def __init__(self, *, state_topic: str) -> None:
        super().__init__("encoder_direction_test_cli")

        self.samples: list[dict[str, Any]] = []
        self.parse_errors = 0

        self.create_subscription(
            String,
            state_topic,
            self._on_state,
            QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE),
        )

    def _on_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.parse_errors += 1
            return

        if isinstance(data, dict):
            data["_received_monotonic_s"] = time.monotonic()
            self.samples.append(data)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check four-wheel encoder direction signs from /savo_localization/wheel_odom_state."
    )
    parser.add_argument(
        "command",
        choices=sorted(DIRECTION_PATTERNS.keys()),
        help="Expected robot motion pattern.",
    )
    parser.add_argument(
        "--state-topic",
        default="/savo_localization/wheel_odom_state",
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument("--min-delta-count", type=int, default=3)
    parser.add_argument("--max-stop-delta-count", type=int, default=2)
    parser.add_argument("--no-prompt", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.duration <= 0.0:
        raise SystemExit("--duration must be > 0.0")

    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be > 0.0")

    if args.min_delta_count < 0:
        raise SystemExit("--min-delta-count must be >= 0")

    if args.max_stop_delta_count < 0:
        raise SystemExit("--max-stop-delta-count must be >= 0")

    command = normalize_command(args.command)

    rclpy.init()
    node = EncoderDirectionNode(state_topic=args.state_topic)

    try:
        wait_for_first_sample(node, timeout_s=args.timeout)

        if not args.no_prompt:
            print("")
            print("Robot Savo Encoder Direction Test")
            print("=================================")
            print(f"Command: {command}")
            print(f"Expected: {format_pattern(DIRECTION_PATTERNS[command])}")
            print("")
            if command == "stop":
                print(f"Keep the robot still for {args.duration:.1f} seconds.")
            else:
                print(f"Move the robot/command the robot as: {command}")
                print(f"Recording will run for {args.duration:.1f} seconds.")
            print("Press ENTER to start recording.")
            input()

        start_index = len(node.samples)
        deadline_s = time.monotonic() + args.duration

        while rclpy.ok() and time.monotonic() < deadline_s:
            rclpy.spin_once(node, timeout_sec=0.05)

        recorded = node.samples[start_index:]

        report = build_report(
            command=command,
            samples=recorded,
            state_topic=args.state_topic,
            parse_errors=node.parse_errors,
            min_delta_count=args.min_delta_count,
            max_stop_delta_count=args.max_stop_delta_count,
            duration_s=args.duration,
        )

        if args.json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print_text_report(report)

        return 0 if report["ok"] else 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_for_first_sample(node: EncoderDirectionNode, *, timeout_s: float) -> None:
    deadline_s = time.monotonic() + timeout_s

    while rclpy.ok() and not node.samples and time.monotonic() < deadline_s:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.samples:
        raise SystemExit("No encoder state messages received before timeout.")


def build_report(
    *,
    command: str,
    samples: list[dict[str, Any]],
    state_topic: str,
    parse_errors: int,
    min_delta_count: int,
    max_stop_delta_count: int,
    duration_s: float,
) -> dict[str, Any]:
    reasons: list[str] = []

    if len(samples) < 2:
        return {
            "ok": False,
            "status": "ERROR",
            "message": "Not enough encoder state samples received.",
            "reasons": ["need at least 2 state samples to compare counts"],
            "command": command,
            "state_topic": state_topic,
            "sample_count": len(samples),
            "parse_errors": parse_errors,
            "duration_s": duration_s,
        }

    first_counts = extract_counts(samples[0])
    last_counts = extract_counts(samples[-1])

    missing = [
        wheel for wheel in WHEEL_NAMES
        if wheel not in first_counts or wheel not in last_counts
    ]

    if missing:
        return {
            "ok": False,
            "status": "ERROR",
            "message": "Encoder state JSON is missing wheel counts.",
            "reasons": [f"missing wheel count data: {', '.join(missing)}"],
            "command": command,
            "state_topic": state_topic,
            "sample_count": len(samples),
            "parse_errors": parse_errors,
            "duration_s": duration_s,
        }

    expected_pattern = DIRECTION_PATTERNS[command]
    wheel_results: dict[str, WheelDirectionResult] = {}

    for wheel in WHEEL_NAMES:
        start = int(first_counts[wheel])
        end = int(last_counts[wheel])
        delta = end - start
        expected = int(expected_pattern[wheel])

        if command == "stop":
            observed = 0 if abs(delta) <= max_stop_delta_count else sign(delta)
            ok = observed == 0
        else:
            observed = 0 if abs(delta) < min_delta_count else sign(delta)
            ok = observed == expected

        wheel_results[wheel] = WheelDirectionResult(
            wheel=wheel,
            expected=expected,
            observed=observed,
            count_start=start,
            count_end=end,
            count_delta=delta,
            ok=ok,
            message="direction matched" if ok else "direction mismatch",
        )

    failed = [result for result in wheel_results.values() if not result.ok]

    if parse_errors > 0:
        reasons.append(f"{parse_errors} state messages failed JSON parsing")

    for result in failed:
        reasons.append(
            f"{result.wheel}: expected {direction_symbol(result.expected)}, "
            f"observed {direction_symbol(result.observed)}, "
            f"delta={result.count_delta}"
        )

    ok = not failed and parse_errors == 0

    return {
        "ok": ok,
        "status": "OK" if ok else "ERROR",
        "message": (
            "Encoder directions match expected pattern"
            if ok
            else "Encoder direction check failed"
        ),
        "reasons": reasons,
        "command": command,
        "expected_pattern": expected_pattern,
        "expected_pattern_text": format_pattern(expected_pattern),
        "state_topic": state_topic,
        "sample_count": len(samples),
        "parse_errors": parse_errors,
        "duration_s": duration_s,
        "min_delta_count": min_delta_count,
        "max_stop_delta_count": max_stop_delta_count,
        "wheel_results": {
            wheel: asdict(result)
            for wheel, result in wheel_results.items()
        },
    }


def extract_counts(state: dict[str, Any]) -> dict[str, int]:
    encoders = state.get("encoders", {})
    if not isinstance(encoders, dict):
        return {}

    counts: dict[str, int] = {}

    for wheel in WHEEL_NAMES:
        wheel_data = encoders.get(wheel, {})
        if not isinstance(wheel_data, dict):
            continue

        if "count" in wheel_data:
            counts[wheel] = int(wheel_data["count"])

    return counts


def print_text_report(report: dict[str, Any]) -> None:
    print("")
    print("Robot Savo Encoder Direction Test")
    print("=================================")
    print(f"Status: {report['status']}")
    print(f"Message: {report['message']}")
    print(f"Command: {report['command']}")
    print(f"Expected: {report.get('expected_pattern_text', '')}")
    print(f"State topic: {report['state_topic']}")
    print(f"Samples: {report['sample_count']}")
    print(f"Duration: {report['duration_s']:.2f}s")

    wheel_results = report.get("wheel_results", {})
    if isinstance(wheel_results, dict) and wheel_results:
        print("")
        print("Wheel results:")
        for wheel in WHEEL_NAMES:
            result = wheel_results.get(wheel, {})
            if not isinstance(result, dict):
                continue

            print(
                "  "
                f"{wheel}: "
                f"start={result.get('count_start')} "
                f"end={result.get('count_end')} "
                f"delta={result.get('count_delta')} "
                f"expected={direction_symbol(int(result.get('expected', 0)))} "
                f"observed={direction_symbol(int(result.get('observed', 0)))} "
                f"{'OK' if result.get('ok') else 'FAIL'}"
            )

    if report["reasons"]:
        print("")
        print("Reasons:")
        for reason in report["reasons"]:
            print(f"  - {reason}")

    print("")
    print(f"Result: {'PASS' if report['ok'] else 'FAIL'}")


def normalize_command(command: str) -> str:
    return command.strip().lower().replace("-", "_").replace(" ", "_")


def sign(value: int | float) -> int:
    if value > 0:
        return +1

    if value < 0:
        return -1

    return 0


def direction_symbol(value: int) -> str:
    if value > 0:
        return "+"

    if value < 0:
        return "-"

    return "0"


def format_pattern(pattern: dict[str, int]) -> str:
    return " ".join(
        f"{wheel}={direction_symbol(pattern[wheel])}"
        for wheel in WHEEL_NAMES
    )


if __name__ == "__main__":
    sys.exit(main())
