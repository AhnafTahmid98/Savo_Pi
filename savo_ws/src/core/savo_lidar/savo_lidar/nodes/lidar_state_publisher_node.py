#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR state summary node - publishes one compact JSON summary."""

from __future__ import annotations

import json
from typing import Any

from ._ros_compat import Node, String, rclpy

from savo_lidar.constants import (
    DEFAULT_HEALTH_TOPIC,
    DEFAULT_STATE_TOPIC,
    DEFAULT_WATCHDOG_STATE_TOPIC,
    STATUS_ERROR,
    STATUS_OK,
    STATUS_WARN,
)
from savo_lidar.ros import get_float_param, get_string_param
from savo_lidar.utils import RateTracker, node_start_message, node_stop_message, status_qos
from savo_lidar.utils.diagnostics import make_status_payload, payload_to_json


class LidarStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_state_publisher_node")

        self._driver_state_topic = get_string_param(
            self,
            "driver_state_topic",
            DEFAULT_STATE_TOPIC,
        )
        self._health_topic = get_string_param(
            self,
            "health_topic",
            DEFAULT_HEALTH_TOPIC,
        )
        self._watchdog_state_topic = get_string_param(
            self,
            "watchdog_state_topic",
            DEFAULT_WATCHDOG_STATE_TOPIC,
        )
        self._summary_topic = get_string_param(
            self,
            "summary_topic",
            "/savo_lidar/state_summary",
        )
        self._publish_hz = get_float_param(self, "publish_hz", 1.0)

        self._validate_config()

        self._rate_tracker = RateTracker()

        self._driver_state: dict[str, Any] = {}
        self._health_state: dict[str, Any] = {}
        self._watchdog_state: dict[str, Any] = {}

        self._summary_pub = self.create_publisher(
            String,
            self._summary_topic,
            status_qos(),
        )

        self._driver_state_sub = self.create_subscription(
            String,
            self._driver_state_topic,
            self._on_driver_state,
            status_qos(),
        )
        self._health_sub = self.create_subscription(
            String,
            self._health_topic,
            self._on_health_state,
            status_qos(),
        )
        self._watchdog_sub = self.create_subscription(
            String,
            self._watchdog_state_topic,
            self._on_watchdog_state,
            status_qos(),
        )

        self._timer = self.create_timer(
            1.0 / self._publish_hz,
            self._on_timer,
        )

        self.get_logger().info(
            node_start_message(
                "lidar_state_publisher_node",
                driver_state_topic=self._driver_state_topic,
                health_topic=self._health_topic,
                watchdog_state_topic=self._watchdog_state_topic,
                summary_topic=self._summary_topic,
                publish_hz=self._publish_hz,
            )
        )

    def destroy_node(self) -> bool:
        self.get_logger().info(node_stop_message("lidar_state_publisher_node"))
        return super().destroy_node()

    def _validate_config(self) -> None:
        if self._publish_hz <= 0.0:
            raise ValueError(f"publish_hz must be > 0.0, got {self._publish_hz}")

    def _on_driver_state(self, msg: String) -> None:
        self._driver_state = self._safe_json_loads(msg.data)

    def _on_health_state(self, msg: String) -> None:
        self._health_state = self._safe_json_loads(msg.data)

    def _on_watchdog_state(self, msg: String) -> None:
        self._watchdog_state = self._safe_json_loads(msg.data)

    def _on_timer(self) -> None:
        summary_publish_rate_hz = self._rate_tracker.tick()
        status = self._combined_status()

        msg = String()
        msg.data = payload_to_json(
            make_status_payload(
                component="lidar_state_publisher_node",
                status=status,
                message=self._summary_message(status),
                driver_status=self._driver_state.get("status"),
                health_status=self._health_state.get("status"),
                watchdog_status=self._watchdog_state.get("status"),
                driver_running=self._driver_state.get("driver_running"),
                hardware_ok=self._health_state.get("hardware_ok"),
                scan_ok=self._health_state.get("scan_ok"),
                stale=self._watchdog_state.get("stale"),
                scan_count=self._driver_state.get("scan_count"),
                scan_rate_hz=self._health_state.get("scan_rate_hz")
                or self._driver_state.get("scan_rate_hz"),
                valid_ratio=self._health_state.get("valid_ratio")
                or self._driver_state.get("valid_ratio"),
                last_scan_age_s=self._watchdog_state.get("last_scan_age_s"),
                summary_publish_rate_hz=summary_publish_rate_hz,
                driver_state_received=bool(self._driver_state),
                health_state_received=bool(self._health_state),
                watchdog_state_received=bool(self._watchdog_state),
            )
        )
        self._summary_pub.publish(msg)

    def _combined_status(self) -> str:
        statuses = [
            str(self._driver_state.get("status", "")).upper(),
            str(self._health_state.get("status", "")).upper(),
            str(self._watchdog_state.get("status", "")).upper(),
        ]

        if STATUS_ERROR in statuses:
            return STATUS_ERROR

        if "STALE" in statuses:
            return "STALE"

        if STATUS_WARN in statuses:
            return STATUS_WARN

        if all(status == STATUS_OK for status in statuses if status):
            return STATUS_OK

        return STATUS_WARN

    def _summary_message(self, status: str) -> str:
        if status == STATUS_OK:
            return "LiDAR stack healthy"

        if status == STATUS_ERROR:
            return "LiDAR stack error"

        if status == "STALE":
            return "LiDAR scan stream stale"

        return "LiDAR stack degraded"

    def _safe_json_loads(self, data: str) -> dict[str, Any]:
        try:
            loaded = json.loads(data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid LiDAR status JSON: {exc}")
            return {}

        if not isinstance(loaded, dict):
            self.get_logger().warning("LiDAR status JSON is not an object")
            return {}

        return loaded


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarStatePublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
