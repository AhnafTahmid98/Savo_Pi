#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Echo LaserScan summaries from a ROS topic."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from savo_lidar.constants import DEFAULT_SCAN_TOPIC
from savo_lidar.diagnostics import (
    compact_status_line,
    format_json_report,
    format_key_value_report,
)
from savo_lidar.filters import range_stats
from savo_lidar.ros import sensor_qos


class ScanEchoNode(Node):
    def __init__(
        self,
        *,
        topic: str,
        count: int,
        show_ranges: bool,
        max_ranges: int,
    ) -> None:
        super().__init__("savo_lidar_scan_echo_cli")

        self.topic = str(topic)
        self.target_count = max(1, int(count))
        self.show_ranges = bool(show_ranges)
        self.max_ranges = max(0, int(max_ranges))

        self.received = 0
        self.samples: list[dict[str, Any]] = []

        self._sub = self.create_subscription(
            LaserScan,
            self.topic,
            self._on_scan,
            sensor_qos(),
        )

    @property
    def done(self) -> bool:
        return self.received >= self.target_count

    def _on_scan(self, scan: LaserScan) -> None:
        self.received += 1

        (
            total_points,
            valid_points,
            valid_ratio,
            min_seen_m,
            max_seen_m,
            mean_seen_m,
        ) = range_stats(
            scan.ranges,
            min_range_m=scan.range_min,
            max_range_m=scan.range_max,
        )

        scan_rate_hz = 0.0
        if scan.scan_time > 0.0:
            scan_rate_hz = 1.0 / float(scan.scan_time)

        sample: dict[str, Any] = {
            "index": self.received,
            "topic": self.topic,
            "frame_id": scan.header.frame_id,
            "stamp_sec": int(scan.header.stamp.sec),
            "stamp_nanosec": int(scan.header.stamp.nanosec),
            "angle_min": float(scan.angle_min),
            "angle_max": float(scan.angle_max),
            "angle_increment": float(scan.angle_increment),
            "scan_time": float(scan.scan_time),
            "scan_rate_hz": scan_rate_hz,
            "range_min": float(scan.range_min),
            "range_max": float(scan.range_max),
            "total_points": total_points,
            "valid_points": valid_points,
            "valid_ratio": valid_ratio,
            "min_seen_m": min_seen_m,
            "max_seen_m": max_seen_m,
            "mean_seen_m": mean_seen_m,
        }

        if self.show_ranges:
            sample["ranges"] = list(scan.ranges[: self.max_ranges])

        self.samples.append(sample)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="scan_echo_cli.py",
        description="Echo LaserScan summaries from a LiDAR topic.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_SCAN_TOPIC,
        help="LaserScan topic to echo.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of scan messages to read.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Timeout in seconds.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON output.",
    )
    parser.add_argument(
        "--details",
        action="store_true",
        help="Print detailed text output.",
    )
    parser.add_argument(
        "--show-ranges",
        action="store_true",
        help="Include the first range values in output.",
    )
    parser.add_argument(
        "--max-ranges",
        type=int,
        default=20,
        help="Maximum number of ranges to print with --show-ranges.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.count <= 0:
        print("count must be > 0", file=sys.stderr)
        return 2

    if args.timeout <= 0.0:
        print("timeout must be > 0.0", file=sys.stderr)
        return 2

    rclpy.init()
    node = ScanEchoNode(
        topic=args.topic,
        count=args.count,
        show_ranges=args.show_ranges,
        max_ranges=args.max_ranges,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        deadline_s = time.monotonic() + float(args.timeout)

        while rclpy.ok() and not node.done:
            if time.monotonic() >= deadline_s:
                break

            executor.spin_once(timeout_sec=0.05)

        result = {
            "ok": node.done,
            "topic": args.topic,
            "requested_count": args.count,
            "received_count": node.received,
            "timeout_s": args.timeout,
            "samples": node.samples,
        }

        _print_result(result, json_enabled=args.json, details=args.details)
        return 0 if node.done else 1

    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def _print_result(
    result: dict[str, Any],
    *,
    json_enabled: bool,
    details: bool,
) -> None:
    if json_enabled:
        print(format_json_report(result))
        return

    if details:
        print(format_key_value_report("LiDAR scan echo", result))
        return

    latest = result["samples"][-1] if result["samples"] else {}

    print(
        compact_status_line(
            name="scan_echo",
            ok=bool(result["ok"]),
            message="received scan" if result["ok"] else "scan timeout",
            topic=result["topic"],
            received=result["received_count"],
            frame_id=latest.get("frame_id"),
            points=latest.get("total_points"),
            valid_ratio=latest.get("valid_ratio"),
            min_seen_m=latest.get("min_seen_m"),
            scan_rate_hz=latest.get("scan_rate_hz"),
        )
    )


if __name__ == "__main__":
    sys.exit(main())
