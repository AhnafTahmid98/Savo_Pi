#!/usr/bin/env python3
"""Print a compact summary of LaserScan messages."""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from savo_lidar.filters.range_filter import range_stats
from savo_lidar.utils.qos import scan_qos


class ScanEchoNode(Node):
    def __init__(
        self,
        *,
        scan_topic: str,
        samples: int,
        min_range_m: float,
        max_range_m: float,
    ) -> None:
        super().__init__("savo_lidar_scan_echo_cli")

        self._scan_topic = scan_topic
        self._samples = samples
        self._min_range_m = min_range_m
        self._max_range_m = max_range_m
        self._received = 0

        self._sub = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            scan_qos(),
        )

    @property
    def done(self) -> bool:
        return self._received >= self._samples

    def _on_scan(self, scan: LaserScan) -> None:
        self._received += 1

        (
            total_points,
            valid_points,
            valid_ratio,
            min_seen_m,
            max_seen_m,
            mean_seen_m,
        ) = range_stats(
            scan.ranges,
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
        )

        print(
            "scan "
            f"{self._received}/{self._samples} | "
            f"frame={scan.header.frame_id} | "
            f"points={total_points} | "
            f"valid={valid_points} | "
            f"valid_ratio={valid_ratio:.3f} | "
            f"min={_fmt_float(min_seen_m)} m | "
            f"max={_fmt_float(max_seen_m)} m | "
            f"mean={_fmt_float(mean_seen_m)} m"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Echo compact LaserScan summaries from a topic.",
    )
    parser.add_argument(
        "--topic",
        default="/scan",
        help="LaserScan topic to read.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of scan messages to print.",
    )
    parser.add_argument(
        "--min-range",
        type=float,
        default=0.15,
        help="Minimum valid range in metres.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=12.0,
        help="Maximum valid range in metres.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Maximum wait time in seconds.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.samples <= 0:
        print("samples must be > 0", file=sys.stderr)
        return 2

    if args.max_range <= args.min_range:
        print("max-range must be greater than min-range", file=sys.stderr)
        return 2

    rclpy.init()
    node = ScanEchoNode(
        scan_topic=args.topic,
        samples=args.samples,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
    )

    deadline = node.get_clock().now().nanoseconds / 1e9 + float(args.timeout)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

            now_s = node.get_clock().now().nanoseconds / 1e9
            if now_s > deadline:
                print(f"timeout waiting for {args.topic}", file=sys.stderr)
                return 1

        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _fmt_float(value: float | None) -> str:
    if value is None or not math.isfinite(float(value)):
        return "n/a"

    return f"{float(value):.3f}"


if __name__ == "__main__":
    sys.exit(main())