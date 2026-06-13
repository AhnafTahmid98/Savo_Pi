#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Check LaserScan quality from a ROS topic."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from savo_lidar.constants import (
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_QUALITY_ERROR_VALID_RATIO,
    DEFAULT_QUALITY_WARN_VALID_RATIO,
    DEFAULT_SCAN_RATE_HZ,
    DEFAULT_SCAN_TOPIC,
)
from savo_lidar.diagnostics import (
    check_range_quality,
    compact_status_line,
    format_json_report,
    format_key_value_report,
    run_scan_rate_check,
)
from savo_lidar.ros import sensor_qos
from savo_lidar.utils.timing import RateTracker


class ScanQualityNode(Node):
    def __init__(
        self,
        *,
        topic: str,
        count: int,
        min_range_m: float,
        max_range_m: float,
        warn_ratio: float,
        error_ratio: float,
        expected_rate_hz: float,
    ) -> None:
        super().__init__("savo_lidar_scan_quality_cli")

        self.topic = str(topic)
        self.target_count = max(1, int(count))

        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        self.warn_ratio = float(warn_ratio)
        self.error_ratio = float(error_ratio)
        self.expected_rate_hz = float(expected_rate_hz)

        self.received = 0
        self.rate_tracker = RateTracker()
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

        measured_rate_hz = self.rate_tracker.tick()

        range_quality = check_range_quality(
            scan.ranges,
            min_range_m=self.min_range_m,
            max_range_m=self.max_range_m,
            warn_ratio=self.warn_ratio,
            error_ratio=self.error_ratio,
        )

        scan_time_rate_hz = 0.0
        if scan.scan_time > 0.0:
            scan_time_rate_hz = 1.0 / float(scan.scan_time)

        rate_for_check = measured_rate_hz if measured_rate_hz > 0.0 else scan_time_rate_hz

        rate_quality = run_scan_rate_check(
            measured_rate_hz=rate_for_check,
            expected_rate_hz=self.expected_rate_hz,
        )

        sample = {
            "index": self.received,
            "topic": self.topic,
            "frame_id": scan.header.frame_id,
            "total_points": range_quality.total_points,
            "valid_points": range_quality.valid_points,
            "invalid_points": range_quality.invalid_points,
            "valid_ratio": range_quality.valid_ratio,
            "range_status": range_quality.status,
            "range_message": range_quality.message,
            "min_seen_m": range_quality.min_range_m,
            "max_seen_m": range_quality.max_range_m,
            "mean_seen_m": range_quality.mean_range_m,
            "measured_rate_hz": measured_rate_hz,
            "scan_time_rate_hz": scan_time_rate_hz,
            "rate_status_ok": rate_quality.ok,
            "rate_message": rate_quality.message,
            "expected_rate_hz": rate_quality.expected_hz,
            "min_allowed_rate_hz": rate_quality.min_allowed_hz,
            "ok": range_quality.ok and rate_quality.ok,
        }

        self.samples.append(sample)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="scan_quality_cli.py",
        description="Check LaserScan range quality and scan rate.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_SCAN_TOPIC,
        help="LaserScan topic to inspect.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=5,
        help="Number of scan messages to inspect.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Timeout in seconds.",
    )
    parser.add_argument(
        "--min-range",
        type=float,
        default=DEFAULT_MIN_RANGE_M,
        help="Minimum valid range in metres.",
    )
    parser.add_argument(
        "--max-range",
        type=float,
        default=DEFAULT_MAX_RANGE_M,
        help="Maximum valid range in metres.",
    )
    parser.add_argument(
        "--warn-ratio",
        type=float,
        default=DEFAULT_QUALITY_WARN_VALID_RATIO,
        help="Warn when valid ratio is below this value.",
    )
    parser.add_argument(
        "--error-ratio",
        type=float,
        default=DEFAULT_QUALITY_ERROR_VALID_RATIO,
        help="Fail when valid ratio is below this value.",
    )
    parser.add_argument(
        "--expected-rate",
        type=float,
        default=DEFAULT_SCAN_RATE_HZ,
        help="Expected scan rate in Hz.",
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
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        _validate_args(args)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    rclpy.init()
    node = ScanQualityNode(
        topic=args.topic,
        count=args.count,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        warn_ratio=args.warn_ratio,
        error_ratio=args.error_ratio,
        expected_rate_hz=args.expected_rate,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        deadline_s = time.monotonic() + float(args.timeout)

        while rclpy.ok() and not node.done:
            if time.monotonic() >= deadline_s:
                break

            executor.spin_once(timeout_sec=0.05)

        latest = node.samples[-1] if node.samples else {}
        ok = node.done and all(bool(sample.get("ok", False)) for sample in node.samples)

        result = {
            "ok": ok,
            "topic": args.topic,
            "requested_count": args.count,
            "received_count": node.received,
            "timeout_s": args.timeout,
            "min_range_m": args.min_range,
            "max_range_m": args.max_range,
            "warn_ratio": args.warn_ratio,
            "error_ratio": args.error_ratio,
            "expected_rate_hz": args.expected_rate,
            "latest": latest,
            "samples": node.samples,
        }

        _print_result(result, json_enabled=args.json, details=args.details)
        return 0 if ok else 1

    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def _validate_args(args: argparse.Namespace) -> None:
    if args.count <= 0:
        raise ValueError("count must be > 0")

    if args.timeout <= 0.0:
        raise ValueError("timeout must be > 0.0")

    if args.min_range <= 0.0:
        raise ValueError("min-range must be > 0.0")

    if args.max_range <= args.min_range:
        raise ValueError("max-range must be greater than min-range")

    if not 0.0 <= args.error_ratio <= 1.0:
        raise ValueError("error-ratio must be between 0.0 and 1.0")

    if not 0.0 <= args.warn_ratio <= 1.0:
        raise ValueError("warn-ratio must be between 0.0 and 1.0")

    if args.error_ratio > args.warn_ratio:
        raise ValueError("error-ratio cannot be greater than warn-ratio")

    if args.expected_rate <= 0.0:
        raise ValueError("expected-rate must be > 0.0")


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
        print(format_key_value_report("LiDAR scan quality", result))
        return

    latest = result.get("latest", {})

    print(
        compact_status_line(
            name="scan_quality",
            ok=bool(result.get("ok", False)),
            message="scan quality healthy" if result.get("ok") else "scan quality check failed",
            topic=result.get("topic"),
            received=result.get("received_count"),
            frame_id=latest.get("frame_id"),
            valid_ratio=latest.get("valid_ratio"),
            valid_points=latest.get("valid_points"),
            total_points=latest.get("total_points"),
            measured_rate_hz=latest.get("measured_rate_hz"),
            range_status=latest.get("range_status"),
        )
    )


if __name__ == "__main__":
    sys.exit(main())
