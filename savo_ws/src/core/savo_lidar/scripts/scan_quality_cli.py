#!/usr/bin/env python3
"""Read LaserScan messages and print scan-quality diagnostics."""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from savo_lidar.diagnostics import check_range_quality, compact_status_line, format_json_report
from savo_lidar.utils.qos import scan_qos
from savo_lidar.utils.timing import RateTracker


class ScanQualityCliNode(Node):
    def __init__(
        self,
        *,
        scan_topic: str,
        samples: int,
        min_range_m: float,
        max_range_m: float,
        warn_ratio: float,
        error_ratio: float,
        json_output: bool,
    ) -> None:
        super().__init__("savo_lidar_scan_quality_cli")

        self._scan_topic = scan_topic
        self._samples = samples
        self._min_range_m = min_range_m
        self._max_range_m = max_range_m
        self._warn_ratio = warn_ratio
        self._error_ratio = error_ratio
        self._json_output = json_output

        self._received = 0
        self._rate = RateTracker()
        self._last_result = None

        self._sub = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            scan_qos(),
        )

    @property
    def done(self) -> bool:
        return self._received >= self._samples

    @property
    def ok(self) -> bool:
        return bool(self._last_result and self._last_result.ok)

    def _on_scan(self, scan: LaserScan) -> None:
        self._received += 1
        rate_hz = self._rate.tick()

        self._last_result = check_range_quality(
            ranges=list(scan.ranges),
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
            scan_rate_hz=rate_hz,
            warn_ratio=self._warn_ratio,
            error_ratio=self._error_ratio,
        )

        if self._json_output:
            print(format_json_report(self._last_result.to_dict()))
            return

        print(
            compact_status_line(
                name=f"scan_quality {self._received}/{self._samples}",
                ok=self._last_result.ok,
                message=self._last_result.message,
                valid_ratio=f"{self._last_result.valid_ratio:.3f}",
                valid_points=self._last_result.valid_points,
                total_points=self._last_result.total_points,
                scan_rate_hz=f"{self._last_result.scan_rate_hz:.2f}",
                min_range_m=self._last_result.min_range_m,
                max_range_m=self._last_result.max_range_m,
                mean_range_m=self._last_result.mean_range_m,
            )
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check LaserScan quality from a Robot Savo LiDAR topic.",
    )
    parser.add_argument(
        "--topic",
        default="/scan",
        help="LaserScan topic to inspect.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=5,
        help="Number of scan messages to inspect.",
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
        "--warn-ratio",
        type=float,
        default=0.60,
        help="Warn if valid range ratio is below this value.",
    )
    parser.add_argument(
        "--error-ratio",
        type=float,
        default=0.30,
        help="Fail if valid range ratio is below this value.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Maximum wait time in seconds.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print JSON output.",
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

    if not 0.0 <= args.error_ratio <= args.warn_ratio <= 1.0:
        print("expected 0.0 <= error-ratio <= warn-ratio <= 1.0", file=sys.stderr)
        return 2

    rclpy.init()
    node = ScanQualityCliNode(
        scan_topic=args.topic,
        samples=args.samples,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        warn_ratio=args.warn_ratio,
        error_ratio=args.error_ratio,
        json_output=args.json,
    )

    deadline = node.get_clock().now().nanoseconds / 1e9 + float(args.timeout)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

            now_s = node.get_clock().now().nanoseconds / 1e9
            if now_s > deadline:
                print(f"timeout waiting for {args.topic}", file=sys.stderr)
                return 1

        return 0 if node.ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())