#!/usr/bin/env python3
"""Filter raw LaserScan data into a mapping/navigation-ready scan topic."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from savo_lidar.constants import (
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_SCAN_TOPIC,
    STATUS_ERROR,
    STATUS_OK,
)
from savo_lidar.filters import filter_ranges, range_stats
from savo_lidar.models import make_scan_quality
from savo_lidar.ros import copy_laser_scan, get_float_param, get_string_param
from savo_lidar.utils import node_start_message, scan_qos, status_qos


class LidarFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_filter_node")

        self._input_scan_topic = get_string_param(self, "input_scan_topic", DEFAULT_SCAN_TOPIC)
        self._output_scan_topic = get_string_param(self, "output_scan_topic", "/scan/filtered")
        self._scan_quality_topic = get_string_param(
            self,
            "scan_quality_topic",
            "/savo_lidar/scan_quality",
        )

        self._min_range_m = get_float_param(self, "min_range_m", DEFAULT_MIN_RANGE_M)
        self._max_range_m = get_float_param(self, "max_range_m", DEFAULT_MAX_RANGE_M)
        self._warn_valid_ratio = get_float_param(self, "warn_valid_ratio", 0.60)
        self._error_valid_ratio = get_float_param(self, "error_valid_ratio", 0.30)

        if self._max_range_m <= self._min_range_m:
            raise ValueError(
                f"Invalid LiDAR range filter: min={self._min_range_m}, "
                f"max={self._max_range_m}"
            )

        self._scan_pub = self.create_publisher(
            LaserScan,
            self._output_scan_topic,
            scan_qos(),
        )
        self._quality_pub = self.create_publisher(
            String,
            self._scan_quality_topic,
            status_qos(),
        )

        self._scan_sub = self.create_subscription(
            LaserScan,
            self._input_scan_topic,
            self._on_scan,
            scan_qos(),
        )

        self._last_scan_rate_hz = 0.0

        self.get_logger().info(
            node_start_message(
                "lidar_filter_node",
                input=self._input_scan_topic,
                output=self._output_scan_topic,
                min_range_m=self._min_range_m,
                max_range_m=self._max_range_m,
            )
        )

    def _on_scan(self, scan: LaserScan) -> None:
        filtered = copy_laser_scan(scan)
        filtered.range_min = float(self._min_range_m)
        filtered.range_max = float(self._max_range_m)
        filtered.ranges = filter_ranges(
            scan.ranges,
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
        )

        self._scan_pub.publish(filtered)
        self._publish_quality(filtered)

    def _publish_quality(self, scan: LaserScan) -> None:
        (
            total_points,
            valid_points,
            _valid_ratio,
            min_seen_m,
            max_seen_m,
            mean_seen_m,
        ) = range_stats(
            scan.ranges,
            min_range_m=self._min_range_m,
            max_range_m=self._max_range_m,
        )

        scan_rate_hz = 0.0
        if scan.scan_time > 0.0:
            scan_rate_hz = 1.0 / float(scan.scan_time)

        quality = make_scan_quality(
            total_points=total_points,
            valid_points=valid_points,
            min_range_m=min_seen_m,
            max_range_m=max_seen_m,
            mean_range_m=mean_seen_m,
            scan_rate_hz=scan_rate_hz,
            warn_ratio=self._warn_valid_ratio,
            error_ratio=self._error_valid_ratio,
        )

        msg = String()
        msg.data = json.dumps(
            {
                "node": "lidar_filter_node",
                "status": quality.status,
                "message": quality.message,
                "total_points": quality.total_points,
                "valid_points": quality.valid_points,
                "valid_ratio": quality.valid_ratio,
                "min_range_m": quality.min_range_m,
                "max_range_m": quality.max_range_m,
                "mean_range_m": quality.mean_range_m,
                "scan_rate_hz": quality.scan_rate_hz,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self._quality_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()