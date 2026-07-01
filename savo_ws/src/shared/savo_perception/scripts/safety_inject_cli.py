#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Inject fake perception range values for safety testing."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32

    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    Float32 = None
    ROS_AVAILABLE = False

from savo_perception.ros.qos_profiles import qos_depth_sensor, qos_range_sensor


@dataclass(frozen=True)
class InjectScenario:
    name: str
    depth_front_m: float
    tof_left_m: float
    tof_right_m: float
    ultrasonic_front_m: float


SCENARIOS = {
    "clear": InjectScenario(
        name="clear",
        depth_front_m=1.20,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=1.00,
    ),
    "front_slow": InjectScenario(
        name="front_slow",
        depth_front_m=0.060,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=0.50,
    ),
    "front_stop": InjectScenario(
        name="front_stop",
        depth_front_m=0.030,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=0.030,
    ),
    "side_stop_left": InjectScenario(
        name="side_stop_left",
        depth_front_m=1.00,
        tof_left_m=0.030,
        tof_right_m=0.90,
        ultrasonic_front_m=0.90,
    ),
    "side_stop_right": InjectScenario(
        name="side_stop_right",
        depth_front_m=1.00,
        tof_left_m=0.90,
        tof_right_m=0.030,
        ultrasonic_front_m=0.90,
    ),
    "missing_required": InjectScenario(
        name="missing_required",
        depth_front_m=1.00,
        tof_left_m=math.nan,
        tof_right_m=0.90,
        ultrasonic_front_m=0.90,
    ),
    "missing_depth": InjectScenario(
        name="missing_depth",
        depth_front_m=math.nan,
        tof_left_m=0.90,
        tof_right_m=0.90,
        ultrasonic_front_m=0.90,
    ),
}


class SafetyInjectNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("safety_inject_cli")

        self.args = args
        self.scenario = self._resolve_scenario(args)

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

        self.start_wall_s = time.time()
        self.sent_count = 0

        period_s = 1.0 / max(float(args.rate_hz), 0.1)
        self.timer = self.create_timer(period_s, self._on_timer)

        print("[SafetyInject] Publishing fake range values")
        print(f"  scenario          = {self.scenario.name}")
        print(f"  duration_s        = {args.duration_s:.2f}")
        print(f"  rate_hz           = {args.rate_hz:.2f}")
        print(f"  depth_front       = {args.depth_front_topic}")
        print(f"  tof_left          = {args.tof_left_topic}")
        print(f"  tof_right         = {args.tof_right_topic}")
        print(f"  ultrasonic_front  = {args.ultrasonic_front_topic}")
        print()

    def _resolve_scenario(self, args: argparse.Namespace) -> InjectScenario:
        if args.scenario != "custom":
            return SCENARIOS[args.scenario]

        return InjectScenario(
            name="custom",
            depth_front_m=args.depth_front_m,
            tof_left_m=args.tof_left_m,
            tof_right_m=args.tof_right_m,
            ultrasonic_front_m=args.ultrasonic_front_m,
        )

    def _on_timer(self) -> None:
        elapsed = time.time() - self.start_wall_s

        self._publish(self.depth_pub, self.scenario.depth_front_m)
        self._publish(self.tof_left_pub, self.scenario.tof_left_m)
        self._publish(self.tof_right_pub, self.scenario.tof_right_m)
        self._publish(self.ultrasonic_pub, self.scenario.ultrasonic_front_m)

        self.sent_count += 1

        print(
            f"\r[SafetyInject] t={elapsed:5.2f}s sent={self.sent_count} "
            f"depth={self._fmt(self.scenario.depth_front_m)} "
            f"left={self._fmt(self.scenario.tof_left_m)} "
            f"right={self._fmt(self.scenario.tof_right_m)} "
            f"ultra={self._fmt(self.scenario.ultrasonic_front_m)}",
            end="",
            flush=True,
        )

        if self.args.duration_s > 0.0 and elapsed >= self.args.duration_s:
            print()
            raise KeyboardInterrupt

    @staticmethod
    def _publish(pub, value: float) -> None:
        msg = Float32()
        msg.data = float(value)
        pub.publish(msg)

    @staticmethod
    def _fmt(value: float) -> str:
        if not math.isfinite(float(value)):
            return "NaN"
        return f"{float(value):.3f}m"


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inject fake Robot Savo perception range values"
    )

    parser.add_argument(
        "--scenario",
        choices=sorted(list(SCENARIOS.keys()) + ["custom"]),
        default="clear",
    )

    parser.add_argument("--depth-front-topic", default="/depth/min_front_m")
    parser.add_argument("--tof-left-topic", default="/savo_perception/range/left_m")
    parser.add_argument("--tof-right-topic", default="/savo_perception/range/right_m")
    parser.add_argument(
        "--ultrasonic-front-topic",
        default="/savo_perception/range/front_ultrasonic_m",
    )

    parser.add_argument("--depth-front-m", type=float, default=1.0)
    parser.add_argument("--tof-left-m", type=float, default=0.9)
    parser.add_argument("--tof-right-m", type=float, default=0.9)
    parser.add_argument("--ultrasonic-front-m", type=float, default=0.9)

    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration-s", type=float, default=5.0, help="0 = run until Ctrl+C")

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this CLI.")
        return 1

    rclpy.init()
    node = SafetyInjectNode(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
