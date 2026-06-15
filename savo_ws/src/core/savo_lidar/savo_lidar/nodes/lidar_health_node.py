#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR health node - combines driver, watchdog, and scan-quality state."""

from __future__ import annotations

import json
from typing import Any

from ._ros_compat import Node, String, rclpy

from savo_lidar.constants import (
    DEFAULT_HEALTH_TOPIC,
    DEFAULT_SCAN_QUALITY_TOPIC,
    DEFAULT_STATE_TOPIC,
    DEFAULT_WATCHDOG_STATE_TOPIC,
    SCAN_RATE_MIN_HZ,
    STATUS_ERROR,
    STATUS_OFFLINE,
)
from savo_lidar.ros import get_float_param, get_string_param
from savo_lidar.safety import LidarHealthPolicy
from savo_lidar.utils import RateTracker, node_start_message, node_stop_message, status_qos
from savo_lidar.utils.diagnostics import make_health_payload, payload_to_json


class LidarHealthNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_health_node")

        self._driver_state_topic = get_string_param(
            self,
            "state_topic",
            DEFAULT_STATE_TOPIC,
        )
        self._watchdog_state_topic = get_string_param(
            self,
            "watchdog_state_topic",
            DEFAULT_WATCHDOG_STATE_TOPIC,
        )
        self._scan_quality_topic = get_string_param(
            self,
            "scan_quality_topic",
            DEFAULT_SCAN_QUALITY_TOPIC,
        )
        self._health_topic = get_string_param(
            self,
            "health_topic",
            DEFAULT_HEALTH_TOPIC,
        )

        self._publish_hz = get_float_param(self, "publish_hz", 1.0)
        self._min_valid_ratio_warn = get_float_param(self, "min_valid_ratio_warn", 0.60)
        self._min_valid_ratio_error = get_float_param(self, "min_valid_ratio_error", 0.30)
        self._min_scan_rate_hz = get_float_param(
            self,
            "min_scan_rate_hz",
            SCAN_RATE_MIN_HZ,
        )

        self._validate_config()

        self._policy = LidarHealthPolicy(
            min_valid_ratio_warn=self._min_valid_ratio_warn,
            min_valid_ratio_error=self._min_valid_ratio_error,
            min_scan_rate_hz=self._min_scan_rate_hz,
        )
        self._rate_tracker = RateTracker()

        self._driver_state: dict[str, Any] = {}
        self._watchdog_state: dict[str, Any] = {}
        self._scan_quality: dict[str, Any] = {}

        self._health_pub = self.create_publisher(
            String,
            self._health_topic,
            status_qos(),
        )

        self._driver_state_sub = self.create_subscription(
            String,
            self._driver_state_topic,
            self._on_driver_state,
            status_qos(),
        )
        self._watchdog_sub = self.create_subscription(
            String,
            self._watchdog_state_topic,
            self._on_watchdog_state,
            status_qos(),
        )
        self._scan_quality_sub = self.create_subscription(
            String,
            self._scan_quality_topic,
            self._on_scan_quality,
            status_qos(),
        )

        self._timer = self.create_timer(
            1.0 / self._publish_hz,
            self._on_timer,
        )

        self.get_logger().info(
            node_start_message(
                "lidar_health_node",
                health_topic=self._health_topic,
                driver_state_topic=self._driver_state_topic,
                watchdog_state_topic=self._watchdog_state_topic,
                scan_quality_topic=self._scan_quality_topic,
                publish_hz=self._publish_hz,
            )
        )

    def destroy_node(self) -> bool:
        self.get_logger().info(node_stop_message("lidar_health_node"))
        return super().destroy_node()

    def _validate_config(self) -> None:
        if self._publish_hz <= 0.0:
            raise ValueError(f"publish_hz must be > 0.0, got {self._publish_hz}")

        if not 0.0 <= self._min_valid_ratio_error <= self._min_valid_ratio_warn:
            raise ValueError("min_valid_ratio_error cannot be greater than min_valid_ratio_warn")

        if self._min_scan_rate_hz < 0.0:
            raise ValueError(f"min_scan_rate_hz cannot be negative, got {self._min_scan_rate_hz}")

    def _on_driver_state(self, msg: String) -> None:
        self._driver_state = self._safe_json_loads(msg.data)

    def _on_watchdog_state(self, msg: String) -> None:
        self._watchdog_state = self._safe_json_loads(msg.data)

    def _on_scan_quality(self, msg: String) -> None:
        self._scan_quality = self._safe_json_loads(msg.data)

    def _on_timer(self) -> None:
        health_publish_rate_hz = self._rate_tracker.tick()

        hardware_ok = self._driver_hardware_ok()
        driver_running = bool(self._driver_state.get("driver_running", False))
        stale = bool(self._watchdog_state.get("stale", True))

        scan_rate_hz = self._scan_rate_hz()
        valid_ratio = self._valid_ratio()

        decision = self._policy.evaluate(
            hardware_ok=hardware_ok,
            driver_running=driver_running,
            stale=stale,
            scan_rate_hz=scan_rate_hz,
            valid_ratio=valid_ratio,
        )

        msg = String()
        msg.data = payload_to_json(
            make_health_payload(
                component="lidar_health_node",
                ok=decision.status not in (STATUS_ERROR, STATUS_OFFLINE),
                status=decision.status,
                message=decision.message,
                hardware_ok=decision.hardware_ok,
                scan_ok=decision.scan_ok,
                driver_running=driver_running,
                stale=stale,
                scan_rate_hz=scan_rate_hz,
                valid_ratio=valid_ratio,
                health_publish_rate_hz=health_publish_rate_hz,
                driver_state_received=bool(self._driver_state),
                watchdog_state_received=bool(self._watchdog_state),
                scan_quality_received=bool(self._scan_quality),
                driver_status=self._driver_state.get("status"),
                watchdog_status=self._watchdog_state.get("status"),
                scan_quality_status=self._scan_quality.get("status"),
            )
        )
        self._health_pub.publish(msg)

    def _driver_hardware_ok(self) -> bool:
        if not self._driver_state:
            return False

        status = str(self._driver_state.get("status", "")).upper()
        if status in (STATUS_ERROR, STATUS_OFFLINE):
            return False

        if "hardware_ok" in self._driver_state:
            return bool(self._driver_state.get("hardware_ok"))

        return True

    def _scan_rate_hz(self) -> float:
        for source in (self._driver_state, self._scan_quality):
            value = source.get("scan_rate_hz")
            if value is not None:
                try:
                    return max(0.0, float(value))
                except (TypeError, ValueError):
                    continue

        return 0.0

    def _valid_ratio(self) -> float:
        for source in (self._scan_quality, self._driver_state):
            value = source.get("valid_ratio")
            if value is not None:
                try:
                    return max(0.0, min(1.0, float(value)))
                except (TypeError, ValueError):
                    continue

        return 0.0

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
    node = LidarHealthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
