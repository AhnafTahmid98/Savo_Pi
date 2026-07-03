#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Smoke test for Robot Savo perception safety topics."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float32, String

    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    Bool = None
    Float32 = None
    String = None
    ROS_AVAILABLE = False

from savo_perception.ros.qos_profiles import (
    qos_depth_sensor,
    qos_range_sensor,
    qos_safety_bool,
    qos_slowdown_factor,
    qos_state_string,
)


@dataclass(frozen=True)
class SmokeScenario:
    name: str
    depth_front_m: float
    tof_left_m: float
    tof_right_m: float
    ultrasonic_front_m: float
    expected_stop: bool
    expected_slowdown_max: Optional[float] = None


SCENARIOS = [
    SmokeScenario(
        name="clear",
        depth_front_m=1.20,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=1.00,
        expected_stop=False,
        expected_slowdown_max=None,
    ),
    SmokeScenario(
        name="front_slow",
        depth_front_m=0.60,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=0.50,
        expected_stop=False,
        expected_slowdown_max=0.95,
    ),
    SmokeScenario(
        name="front_stop",
        depth_front_m=0.30,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=0.30,
        expected_stop=True,
        expected_slowdown_max=0.05,
    ),
    SmokeScenario(
        name="side_stop_left",
        depth_front_m=1.00,
        tof_left_m=0.10,
        tof_right_m=0.90,
        ultrasonic_front_m=0.90,
        expected_stop=True,
        expected_slowdown_max=0.05,
    ),
    SmokeScenario(
        name="missing_required",
        depth_front_m=1.00,
        tof_left_m=math.nan,
        tof_right_m=0.90,
        ultrasonic_front_m=0.90,
        expected_stop=True,
        expected_slowdown_max=0.05,
    ),
    SmokeScenario(
        name="missing_optional_depth",
        depth_front_m=math.nan,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=0.90,
        expected_stop=False,
        expected_slowdown_max=None,
    ),
]


@dataclass
class SmokeResult:
    name: str
    passed: bool
    expected_stop: bool
    actual_stop: Optional[bool]
    actual_slowdown: Optional[float]
    reason: str


class PerceptionSmokeNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("perception_smoke_test_cli")

        self.args = args
        self.latest_stop: Optional[bool] = None
        self.latest_stop_stamp_s: Optional[float] = None
        self.latest_slowdown: Optional[float] = None
        self.latest_slowdown_stamp_s: Optional[float] = None
        self.latest_safety_state = ""
        self.latest_range_health = ""

        self.depth_pub = self.create_publisher(
            Float32,
            args.depth_front_topic,
            qos_depth_sensor(),
        )
        self.tof_left_pub = self.create_publisher(
            Float32,
            args.tof_left_topic,
            qos_range_sensor(),
        )
        self.tof_right_pub = self.create_publisher(
            Float32,
            args.tof_right_topic,
            qos_range_sensor(),
        )
        self.ultrasonic_pub = self.create_publisher(
            Float32,
            args.ultrasonic_front_topic,
            qos_range_sensor(),
        )

        self.create_subscription(
            Bool,
            args.safety_stop_topic,
            self._on_stop,
            qos_safety_bool(),
        )
        self.create_subscription(
            Float32,
            args.slowdown_topic,
            self._on_slowdown,
            qos_slowdown_factor(),
        )
        self.create_subscription(
            String,
            args.safety_state_topic,
            self._on_safety_state,
            qos_state_string(),
        )
        self.create_subscription(
            String,
            args.range_health_topic,
            self._on_range_health,
            qos_state_string(),
        )

    def publish_scenario(self, scenario: SmokeScenario) -> None:
        self._publish_float(self.depth_pub, scenario.depth_front_m)
        self._publish_float(self.tof_left_pub, scenario.tof_left_m)
        self._publish_float(self.tof_right_pub, scenario.tof_right_m)
        self._publish_float(self.ultrasonic_pub, scenario.ultrasonic_front_m)

    def check_scenario(self, scenario: SmokeScenario) -> SmokeResult:
        if self.latest_stop is None:
            return SmokeResult(
                name=scenario.name,
                passed=False,
                expected_stop=scenario.expected_stop,
                actual_stop=None,
                actual_slowdown=self.latest_slowdown,
                reason="no /safety/stop received",
            )

        if self.latest_slowdown is None:
            return SmokeResult(
                name=scenario.name,
                passed=False,
                expected_stop=scenario.expected_stop,
                actual_stop=self.latest_stop,
                actual_slowdown=None,
                reason="no /safety/slowdown_factor received",
            )

        if bool(self.latest_stop) != bool(scenario.expected_stop):
            return SmokeResult(
                name=scenario.name,
                passed=False,
                expected_stop=scenario.expected_stop,
                actual_stop=self.latest_stop,
                actual_slowdown=self.latest_slowdown,
                reason="stop state mismatch",
            )

        if scenario.expected_slowdown_max is not None:
            if float(self.latest_slowdown) > float(scenario.expected_slowdown_max):
                return SmokeResult(
                    name=scenario.name,
                    passed=False,
                    expected_stop=scenario.expected_stop,
                    actual_stop=self.latest_stop,
                    actual_slowdown=self.latest_slowdown,
                    reason="slowdown too high",
                )

        return SmokeResult(
            name=scenario.name,
            passed=True,
            expected_stop=scenario.expected_stop,
            actual_stop=self.latest_stop,
            actual_slowdown=self.latest_slowdown,
            reason="ok",
        )

    def _on_stop(self, msg) -> None:
        self.latest_stop = bool(getattr(msg, "data", False))
        self.latest_stop_stamp_s = time.monotonic()

    def _on_slowdown(self, msg) -> None:
        value = float(getattr(msg, "data", 1.0))
        self.latest_slowdown = value if math.isfinite(value) else None
        self.latest_slowdown_stamp_s = time.monotonic()

    def _on_safety_state(self, msg) -> None:
        self.latest_safety_state = str(getattr(msg, "data", ""))

    def _on_range_health(self, msg) -> None:
        self.latest_range_health = str(getattr(msg, "data", ""))

    @staticmethod
    def _publish_float(pub, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)


def wait_for_output(node: PerceptionSmokeNode, timeout_s: float) -> bool:
    start = time.monotonic()

    while time.monotonic() - start < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_stop is not None and node.latest_slowdown is not None:
            return True

    return False


def run_scenario(node: PerceptionSmokeNode, scenario: SmokeScenario, args: argparse.Namespace) -> SmokeResult:
    node.latest_stop = None
    node.latest_slowdown = None

    start = time.monotonic()
    period_s = 1.0 / max(float(args.publish_rate_hz), 0.1)

    while time.monotonic() - start < float(args.settle_s):
        loop_start = time.monotonic()
        node.publish_scenario(scenario)
        rclpy.spin_once(node, timeout_sec=0.02)

        sleep_s = period_s - (time.monotonic() - loop_start)
        if sleep_s > 0.0:
            time.sleep(sleep_s)

    wait_for_output(node, args.timeout_s)
    return node.check_scenario(scenario)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Smoke test Robot Savo perception safety topics")

    parser.add_argument("--depth-front-topic", default="/depth/min_front_m")
    parser.add_argument("--tof-left-topic", default="/savo_perception/range/left_m")
    parser.add_argument("--tof-right-topic", default="/savo_perception/range/right_m")
    parser.add_argument(
        "--ultrasonic-front-topic",
        default="/savo_perception/range/front_ultrasonic_m",
    )

    parser.add_argument("--safety-stop-topic", default="/safety/stop")
    parser.add_argument("--slowdown-topic", default="/safety/slowdown_factor")
    parser.add_argument("--safety-state-topic", default="/savo_perception/safety_state")
    parser.add_argument("--range-health-topic", default="/savo_perception/range_health")

    parser.add_argument("--publish-rate-hz", type=float, default=10.0)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--timeout-s", type=float, default=2.0)

    parser.add_argument(
        "--scenario",
        choices=["all"] + [scenario.name for scenario in SCENARIOS],
        default="all",
    )

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this CLI.")
        return 1

    selected = SCENARIOS
    if args.scenario != "all":
        selected = [scenario for scenario in SCENARIOS if scenario.name == args.scenario]

    rclpy.init()
    node = PerceptionSmokeNode(args)

    results: list[SmokeResult] = []

    try:
        print("[PerceptionSmoke] Starting perception safety smoke test")
        print(f"  scenarios = {[s.name for s in selected]}")
        print(f"  settle_s  = {args.settle_s:.2f}")
        print(f"  timeout_s = {args.timeout_s:.2f}")
        print("-" * 96)

        for scenario in selected:
            result = run_scenario(node, scenario, args)
            results.append(result)

            mark = "PASS" if result.passed else "FAIL"
            slowdown = "---" if result.actual_slowdown is None else f"{result.actual_slowdown:.3f}"

            print(
                f"{mark:4s} | {result.name:<24s} | "
                f"expected_stop={str(result.expected_stop):<5s} "
                f"actual_stop={str(result.actual_stop):<5s} "
                f"slowdown={slowdown:<7s} "
                f"reason={result.reason}"
            )

        print("-" * 96)

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    passed = all(result.passed for result in results)
    print("Result:", "PASS" if passed else "FAIL")

    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
