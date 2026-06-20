#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pointcloud monitor node for Robot Savo mapping."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from savo_mapping.constants import (
    DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
    DEFAULT_PUBLISH_RATE_HZ,
    FRAME_CAMERA_DEPTH,
    TOPIC_REALSENSE_POINTS,
)
from savo_mapping.diagnostics.pointcloud_ready_check import (
    evaluate_pointcloud_msg,
    pointcloud_result_to_diagnostic,
)
from savo_mapping.ros.adapters import json_msg
from savo_mapping.ros.qos_profiles import get_topic_qos_profile, status_qos
from savo_mapping.utils.timing import RateTracker, age_s, now_s


@dataclass
class PointcloudRuntime:
    latest_msg: Optional[PointCloud2] = None
    msg_count: int = 0
    last_wall_s: Optional[float] = None

    def __post_init__(self) -> None:
        self.rate_tracker = RateTracker()

    @property
    def rate_hz(self) -> float:
        return self.rate_tracker.rate_hz

    @property
    def sample_age_s(self) -> Optional[float]:
        return age_s(self.last_wall_s)

    def update(self, msg: PointCloud2) -> None:
        stamp = now_s()

        self.latest_msg = msg
        self.msg_count += 1
        self.last_wall_s = stamp
        self.rate_tracker.tick(stamp)


class PointcloudMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_monitor_node")

        self._declare_parameters()
        self._load_parameters()

        self._runtime = PointcloudRuntime()
        self._last_log_state: Optional[tuple[bool, bool, int]] = None

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            status_qos(),
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_pointcloud,
            get_topic_qos_profile(self.pointcloud_topic),
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._on_timer,
        )

        self.get_logger().info(
            "Pointcloud monitor started: "
            f"enabled={self.enabled} topic={self.pointcloud_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", False)
        self.declare_parameter("pointcloud_topic", TOPIC_REALSENSE_POINTS)
        self.declare_parameter("status_topic", "/savo_mapping/pointcloud_status")
        self.declare_parameter("expected_frame", FRAME_CAMERA_DEPTH)

        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("min_rate_hz", 3.0)
        self.declare_parameter("min_points", 100)
        self.declare_parameter("stale_timeout_s", DEFAULT_POINTCLOUD_STALE_TIMEOUT_S)

        self.declare_parameter("verbose_status_log", False)

    def _load_parameters(self) -> None:
        self.enabled = bool(self.get_parameter("enabled").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.expected_frame = str(self.get_parameter("expected_frame").value)

        self.publish_rate_hz = self._positive_float_parameter(
            "publish_rate_hz",
            DEFAULT_PUBLISH_RATE_HZ,
        )
        self.min_rate_hz = self._non_negative_float_parameter("min_rate_hz", 3.0)
        self.min_points = max(0, int(self.get_parameter("min_points").value))
        self.stale_timeout_s = self._positive_float_parameter(
            "stale_timeout_s",
            DEFAULT_POINTCLOUD_STALE_TIMEOUT_S,
        )

        self.verbose_status_log = bool(
            self.get_parameter("verbose_status_log").value
        )

    def _positive_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value <= 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be positive. Using {default}."
            )
            return float(default)

        return value

    def _non_negative_float_parameter(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)

        if value < 0.0:
            self.get_logger().warning(
                f"Parameter {name} must be non-negative. Using {default}."
            )
            return float(default)

        return value

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        self._runtime.update(msg)

    def _on_timer(self) -> None:
        result = self._evaluate()
        diagnostic = pointcloud_result_to_diagnostic(result, required=self.enabled)

        payload = result.to_dict()
        payload["diagnostic"] = diagnostic.to_dict()

        self.status_pub.publish(json_msg(payload))

        log_state = (
            bool(result.ok),
            bool(result.stale),
            int(result.msg_count),
        )

        if self.verbose_status_log or log_state != self._last_log_state:
            self.get_logger().info(
                "Pointcloud status: "
                f"enabled={result.enabled} ok={result.ok} "
                f"stale={result.stale} count={result.msg_count} "
                f"points={result.point_count}"
            )

        self._last_log_state = log_state

    def _evaluate(self):
        if self._runtime.latest_msg is None:
            return evaluate_pointcloud_msg(
                _empty_pointcloud_msg(self.expected_frame),
                enabled=self.enabled,
                msg_count=0,
                rate_hz=0.0,
                age_s=None,
                topic=self.pointcloud_topic,
                min_rate_hz=self.min_rate_hz,
                min_points=self.min_points,
                stale_timeout_s=self.stale_timeout_s,
            )

        return evaluate_pointcloud_msg(
            self._runtime.latest_msg,
            enabled=self.enabled,
            msg_count=self._runtime.msg_count,
            rate_hz=self._runtime.rate_hz,
            age_s=self._runtime.sample_age_s,
            topic=self.pointcloud_topic,
            min_rate_hz=self.min_rate_hz,
            min_points=self.min_points,
            stale_timeout_s=self.stale_timeout_s,
        )


def _empty_pointcloud_msg(frame_id: str) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = str(frame_id)
    return msg


def main(args: Optional[list[str]] = None) -> int:
    rclpy.init(args=args)

    node = PointcloudMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("Pointcloud monitor stopped.")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())