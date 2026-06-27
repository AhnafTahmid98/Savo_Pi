#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CLI helper to echo perception range topics."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

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


@dataclass
class RangeValue:
    name: str
    distance_m: Optional[float] = None
    stamp_mono_s: Optional[float] = None

    def update(self, value: float) -> None:
        if math.isfinite(value) and value > 0.0:
            self.distance_m = float(value)
        else:
            self.distance_m = None
        self.stamp_mono_s = time.monotonic()

    def age_s(self) -> Optional[float]:
        if self.stamp_mono_s is None:
            return None
        return max(0.0, time.monotonic() - self.stamp_mono_s)

    def stale(self, timeout_s: float) -> bool:
        age = self.age_s()
        return age is None or age > timeout_s


class RangeEchoNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("range_echo_cli")

        self.stale_timeout_s = float(args.stale_timeout)
        self.print_hz = float(args.print_hz)

        self.values: Dict[str, RangeValue] = {
            "depth_front": RangeValue("depth_front"),
            "tof_left": RangeValue("tof_left"),
            "tof_right": RangeValue("tof_right"),
            "ultrasonic_front": RangeValue("ultrasonic_front"),
        }

        self.create_subscription(
            Float32,
            args.depth_front_topic,
            lambda msg: self._on_range("depth_front", msg),
            qos_depth_sensor(),
        )
        self.create_subscription(
            Float32,
            args.tof_left_topic,
            lambda msg: self._on_range("tof_left", msg),
            qos_range_sensor(),
        )
        self.create_subscription(
            Float32,
            args.tof_right_topic,
            lambda msg: self._on_range("tof_right", msg),
            qos_range_sensor(),
        )
        self.create_subscription(
            Float32,
            args.ultrasonic_front_topic,
            lambda msg: self._on_range("ultrasonic_front", msg),
            qos_range_sensor(),
        )

        period_s = 1.0 / max(self.print_hz, 0.1)
        self.timer = self.create_timer(period_s, self._print_table)

        print("[RangeEcho] Listening to perception range topics")
        print(f"  depth_front       = {args.depth_front_topic}")
        print(f"  tof_left          = {args.tof_left_topic}")
        print(f"  tof_right         = {args.tof_right_topic}")
        print(f"  ultrasonic_front  = {args.ultrasonic_front_topic}")
        print(f"  stale_timeout_s   = {self.stale_timeout_s:.2f}")
        print()

    def _on_range(self, name: str, msg) -> None:
        self.values[name].update(float(getattr(msg, "data", math.nan)))

    def _print_table(self) -> None:
        print("\033[2J\033[H", end="")
        print("Robot Savo perception range echo")
        print("-" * 64)
        print(f"{'sensor':<20s} {'distance_m':>12s} {'age_s':>10s} {'state':>10s}")
        print("-" * 64)

        for name in ("depth_front", "tof_left", "tof_right", "ultrasonic_front"):
            item = self.values[name]
            distance = "---" if item.distance_m is None else f"{item.distance_m:.3f}"
            age = item.age_s()
            age_text = "---" if age is None else f"{age:.2f}"
            state = "STALE" if item.stale(self.stale_timeout_s) else "OK"

            print(f"{name:<20s} {distance:>12s} {age_text:>10s} {state:>10s}")

        print("-" * 64)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Echo Robot Savo perception range topics")

    parser.add_argument("--depth-front-topic", default="/depth/min_front_m")
    parser.add_argument("--tof-left-topic", default="/savo_perception/range/left_m")
    parser.add_argument("--tof-right-topic", default="/savo_perception/range/right_m")
    parser.add_argument(
        "--ultrasonic-front-topic",
        default="/savo_perception/range/front_ultrasonic_m",
    )

    parser.add_argument("--stale-timeout", type=float, default=0.50)
    parser.add_argument("--print-hz", type=float, default=2.0)

    return parser


def main() -> int:
    args = build_arg_parser().parse_args()

    if not ROS_AVAILABLE:
        print("ERROR: rclpy/std_msgs are not available. Source ROS 2 Jazzy before running this CLI.")
        return 1

    rclpy.init()
    node = RangeEchoNode(args)

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
