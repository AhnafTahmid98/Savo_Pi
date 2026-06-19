#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""PointCloud2 echo/check CLI for Robot Savo mapping."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional, Sequence


# =============================================================================
# Local source-tree import support
# =============================================================================
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]

if str(_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PACKAGE_ROOT))


from savo_mapping.constants import FRAME_CAMERA_DEPTH, TOPIC_REALSENSE_POINTS  # noqa: E402
from savo_mapping.diagnostics.pointcloud_ready_check import (  # noqa: E402
    evaluate_pointcloud_ready,
    evaluate_pointcloud_summary,
    pointcloud_result_to_diagnostic,
)
from savo_mapping.diagnostics.report_formatter import format_key_value_block  # noqa: E402
from savo_mapping.ros.adapters import pointcloud_summary  # noqa: E402
from savo_mapping.utils.timing import RateTracker, StaleMonitor, age_s, now_s  # noqa: E402


# =============================================================================
# Arguments
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Echo/check RealSense PointCloud2 input for Robot Savo mapping.",
    )

    parser.add_argument(
        "--topic",
        default=TOPIC_REALSENSE_POINTS,
        help="PointCloud2 topic to check.",
    )
    parser.add_argument(
        "--expected-frame",
        default=FRAME_CAMERA_DEPTH,
        help="Expected PointCloud2 frame id.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="ROS echo duration in seconds when --ros is used.",
    )
    parser.add_argument(
        "--min-rate",
        type=float,
        default=3.0,
        help="Minimum expected pointcloud rate.",
    )
    parser.add_argument(
        "--min-points",
        type=int,
        default=100,
        help="Minimum expected points per cloud.",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=1.0,
        help="Maximum allowed message age.",
    )
    parser.add_argument(
        "--ros",
        action="store_true",
        help="Subscribe to a real ROS PointCloud2 topic.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print full JSON result.",
    )

    return parser


# =============================================================================
# Dry check
# =============================================================================
def _run_dry_check(args: argparse.Namespace) -> int:
    result = evaluate_pointcloud_ready(
        enabled=True,
        msg_count=20,
        rate_hz=8.0,
        age_s=0.05,
        point_count=12000,
        frame_id=args.expected_frame,
        topic=args.topic,
        min_rate_hz=args.min_rate,
        min_points=args.min_points,
        stale_timeout_s=args.stale_timeout,
        width=160,
        height=75,
    )

    diagnostic = pointcloud_result_to_diagnostic(result, required=False)

    if args.json:
        print(result.to_json(indent=2))
    else:
        print(
            format_key_value_block(
                "Robot Savo pointcloud dry check",
                {
                    "ok": result.ok,
                    "level": diagnostic.level,
                    "message": result.message,
                    "topic": result.topic,
                    "frame_id": result.frame_id,
                    "rate_hz": result.rate_hz,
                    "age_s": result.age_s,
                    "point_count": result.point_count,
                    "width": result.width,
                    "height": result.height,
                },
            )
        )
        print()
        print("Real ROS check later:")
        print(f"python3 scripts/pointcloud_echo_cli.py --ros --topic {args.topic}")

    return 0 if result.ok else 2


# =============================================================================
# ROS check
# =============================================================================
class _PointcloudEchoNode:
    def __init__(self, args: argparse.Namespace) -> None:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2

        from savo_mapping.ros.qos_profiles import get_topic_qos_profile

        self.rclpy = rclpy
        self._owns_rclpy = not rclpy.ok()

        if self._owns_rclpy:
            rclpy.init(args=None)

        class PointcloudEchoNode(Node):
            def __init__(self) -> None:
                super().__init__("pointcloud_echo_cli")

                self.latest_msg = None
                self.latest_summary = None
                self.msg_count = 0
                self.first_wall_s: Optional[float] = None
                self.last_wall_s: Optional[float] = None
                self.rate_tracker = RateTracker()
                self.stale_monitor = StaleMonitor(timeout_s=args.stale_timeout)

                self.subscription = self.create_subscription(
                    PointCloud2,
                    args.topic,
                    self._on_msg,
                    get_topic_qos_profile(args.topic),
                )

            def _on_msg(self, msg) -> None:
                stamp = now_s()

                if self.first_wall_s is None:
                    self.first_wall_s = stamp

                self.latest_msg = msg
                self.latest_summary = pointcloud_summary(msg)
                self.msg_count += 1
                self.last_wall_s = stamp
                self.rate_tracker.tick()
                self.stale_monitor.update(stamp)

        self.node = PointcloudEchoNode()

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + max(0.1, float(duration_s))

        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def result(self, args: argparse.Namespace):
        node = self.node

        if node.latest_summary is None:
            return evaluate_pointcloud_ready(
                enabled=True,
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                point_count=0,
                frame_id=args.expected_frame,
                topic=args.topic,
                min_rate_hz=args.min_rate,
                min_points=args.min_points,
                stale_timeout_s=args.stale_timeout,
            )

        sample_age = age_s(node.last_wall_s)

        return evaluate_pointcloud_summary(
            summary=node.latest_summary,
            enabled=True,
            msg_count=node.msg_count,
            rate_hz=node.rate_tracker.rate_hz,
            age_s=sample_age,
            topic=args.topic,
            min_rate_hz=args.min_rate,
            min_points=args.min_points,
            stale_timeout_s=args.stale_timeout,
        )

    def shutdown(self) -> None:
        self.node.destroy_node()

        if self._owns_rclpy:
            self.rclpy.shutdown()


def _run_ros_check(args: argparse.Namespace) -> int:
    echo = _PointcloudEchoNode(args)

    try:
        echo.spin_for(args.duration)
        result = echo.result(args)
    finally:
        echo.shutdown()

    diagnostic = pointcloud_result_to_diagnostic(result, required=False)

    if args.json:
        print(result.to_json(indent=2))
    else:
        print(
            format_key_value_block(
                "Robot Savo pointcloud ROS check",
                {
                    "ok": result.ok,
                    "level": diagnostic.level,
                    "message": result.message,
                    "topic": result.topic,
                    "frame_id": result.frame_id,
                    "msg_count": result.msg_count,
                    "rate_hz": result.rate_hz,
                    "age_s": result.age_s,
                    "point_count": result.point_count,
                    "width": result.width,
                    "height": result.height,
                    "stale": result.stale,
                },
            )
        )

    return 0 if result.ok else 2


# =============================================================================
# Main
# =============================================================================
def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.ros:
        return _run_ros_check(args)

    return _run_dry_check(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
