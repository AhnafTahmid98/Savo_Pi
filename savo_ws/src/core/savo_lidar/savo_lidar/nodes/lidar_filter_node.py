#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR filter node — filters LaserScan ranges and publishes scan quality."""

from __future__ import annotations

from typing import Any

from savo_lidar.nodes._ros_compat import LaserScan, Node, String, rclpy

from savo_lidar.constants import (
    DEFAULT_FILTERED_SCAN_TOPIC,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_QUALITY_ERROR_VALID_RATIO,
    DEFAULT_QUALITY_WARN_VALID_RATIO,
    DEFAULT_SCAN_QUALITY_TOPIC,
    DEFAULT_SCAN_TOPIC,
)
from savo_lidar.filters import filter_ranges, range_stats
from savo_lidar.models import make_scan_quality
from savo_lidar.ros import copy_laser_scan, get_float_param, get_string_param
from savo_lidar.utils import node_start_message, node_stop_message, scan_qos, status_qos
from savo_lidar.utils.diagnostics import make_status_payload, payload_to_json


class LidarFilterNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_filter_node")

        self._input_scan_topic = get_string_param(
            self,
            "input_scan_topic",
            DEFAULT_SCAN_TOPIC,
        )
        self._output_scan_topic = get_string_param(
            self,
            "output_scan_topic",
            DEFAULT_FILTERED_SCAN_TOPIC,
        )
        self._scan_quality_topic = get_string_param(
            self,
            "scan_quality_topic",
            DEFAULT_SCAN_QUALITY_TOPIC,
        )

        self._min_range_m = get_float_param(self, "min_range_m", DEFAULT_MIN_RANGE_M)
        self._max_range_m = get_float_param(self, "max_range_m", DEFAULT_MAX_RANGE_M)
        self._warn_valid_ratio = get_float_param(
            self,
            "warn_valid_ratio",
            DEFAULT_QUALITY_WARN_VALID_RATIO,
        )
        self._error_valid_ratio = get_float_param(
            self,
            "error_valid_ratio",
            DEFAULT_QUALITY_ERROR_VALID_RATIO,
        )

        self._validate_config()

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

        self.get_logger().info(
            node_start_message(
                "lidar_filter_node",
                input=self._input_scan_topic,
                output=self._output_scan_topic,
                min_range_m=self._min_range_m,
                max_range_m=self._max_range_m,
            )
        )

    def destroy_node(self) -> bool:
        self.get_logger().info(node_stop_message("lidar_filter_node"))
        return super().destroy_node()

    def _validate_config(self) -> None:
        if self._max_range_m <= self._min_range_m:
            raise ValueError(
                f"Invalid LiDAR range filter: min={self._min_range_m}, "
                f"max={self._max_range_m}"
            )

        if not 0.0 <= self._error_valid_ratio <= 1.0:
            raise ValueError(
                f"error_valid_ratio must be between 0.0 and 1.0, "
                f"got {self._error_valid_ratio}"
            )

        if not 0.0 <= self._warn_valid_ratio <= 1.0:
            raise ValueError(
                f"warn_valid_ratio must be between 0.0 and 1.0, "
                f"got {self._warn_valid_ratio}"
            )

        if self._error_valid_ratio > self._warn_valid_ratio:
            raise ValueError("error_valid_ratio cannot be greater than warn_valid_ratio")

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
        msg.data = payload_to_json(
            make_status_payload(
                component="lidar_filter_node",
                status=quality.status,
                message=quality.message,
                total_points=quality.total_points,
                valid_points=quality.valid_points,
                valid_ratio=quality.valid_ratio,
                min_range_m=quality.min_range_m,
                max_range_m=quality.max_range_m,
                mean_range_m=quality.mean_range_m,
                scan_rate_hz=quality.scan_rate_hz,
                input_scan_topic=self._input_scan_topic,
                output_scan_topic=self._output_scan_topic,
            )
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
