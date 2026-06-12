#!/usr/bin/env python3
"""Combine LiDAR state, watchdog, and scan-quality data into one health topic."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from savo_lidar.constants import STATUS_OFFLINE
from savo_lidar.ros import get_float_param, get_string_param
from savo_lidar.safety import LidarFaultLatch, LidarHealthPolicy
from savo_lidar.utils import node_start_message, status_qos


class LidarHealthNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_health_node")

        self._state_topic = get_string_param(self, "state_topic", "/savo_lidar/state")
        self._watchdog_topic = get_string_param(
            self,
            "watchdog_state_topic",
            "/savo_lidar/watchdog_state",
        )
        self._scan_quality_topic = get_string_param(
            self,
            "scan_quality_topic",
            "/savo_lidar/scan_quality",
        )
        self._health_topic = get_string_param(self, "health_topic", "/savo_lidar/health")

        self._publish_hz = get_float_param(self, "publish_hz", 1.0)
        self._min_valid_ratio_warn = get_float_param(self, "min_valid_ratio_warn", 0.60)
        self._min_valid_ratio_error = get_float_param(self, "min_valid_ratio_error", 0.30)
        self._min_scan_rate_hz = get_float_param(self, "min_scan_rate_hz", 2.0)

        if self._publish_hz <= 0.0:
            raise ValueError(f"publish_hz must be > 0.0, got {self._publish_hz}")

        self._policy = LidarHealthPolicy(
            min_valid_ratio_warn=self._min_valid_ratio_warn,
            min_valid_ratio_error=self._min_valid_ratio_error,
            min_scan_rate_hz=self._min_scan_rate_hz,
        )
        self._fault_latch = LidarFaultLatch()

        self._driver_state: dict[str, Any] = {}
        self._watchdog_state: dict[str, Any] = {}
        self._scan_quality: dict[str, Any] = {}

        self._health_pub = self.create_publisher(
            String,
            self._health_topic,
            status_qos(),
        )

        self._state_sub = self.create_subscription(
            String,
            self._state_topic,
            self._on_driver_state,
            status_qos(),
        )
        self._watchdog_sub = self.create_subscription(
            String,
            self._watchdog_topic,
            self._on_watchdog_state,
            status_qos(),
        )
        self._quality_sub = self.create_subscription(
            String,
            self._scan_quality_topic,
            self._on_scan_quality,
            status_qos(),
        )

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            node_start_message(
                "lidar_health_node",
                state_topic=self._state_topic,
                watchdog_topic=self._watchdog_topic,
                scan_quality_topic=self._scan_quality_topic,
                health_topic=self._health_topic,
                publish_hz=self._publish_hz,
            )
        )

    def _on_driver_state(self, msg: String) -> None:
        self._driver_state = _parse_json_msg(msg.data)

    def _on_watchdog_state(self, msg: String) -> None:
        self._watchdog_state = _parse_json_msg(msg.data)

    def _on_scan_quality(self, msg: String) -> None:
        self._scan_quality = _parse_json_msg(msg.data)

    def _on_timer(self) -> None:
        hardware_ok = bool(self._driver_state.get("hardware_ok", False))
        driver_running = bool(self._driver_state.get("driver_running", False))
        stale = bool(self._watchdog_state.get("stale", True))

        scan_rate_hz = _safe_float(
            self._scan_quality.get(
                "scan_rate_hz",
                self._driver_state.get("scan_rate_hz", 0.0),
            )
        )
        valid_ratio = _safe_float(
            self._scan_quality.get(
                "valid_ratio",
                self._driver_state.get("valid_ratio", 0.0),
            )
        )

        driver_status = str(self._driver_state.get("status", STATUS_OFFLINE))
        driver_message = str(self._driver_state.get("message", ""))

        if driver_status == "ERROR" and driver_message:
            self._fault_latch.latch(driver_message)

        if driver_status == "OK" and not stale:
            self._fault_latch.clear()

        decision = self._policy.evaluate(
            hardware_ok=hardware_ok,
            driver_running=driver_running,
            stale=stale,
            scan_rate_hz=scan_rate_hz,
            valid_ratio=valid_ratio,
            fault_latched=self._fault_latch.latched,
            fault_reason=self._fault_latch.reason,
        )

        payload = {
            "node": "lidar_health_node",
            "status": decision.status,
            "message": decision.message,
            "hardware_ok": decision.hardware_ok,
            "scan_ok": decision.scan_ok,
            "driver_running": driver_running,
            "stale": stale,
            "scan_rate_hz": scan_rate_hz,
            "valid_ratio": valid_ratio,
            "fault_latched": self._fault_latch.latched,
            "fault_reason": self._fault_latch.reason,
            "driver_status": driver_status,
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._health_pub.publish(msg)


def _parse_json_msg(data: str) -> dict[str, Any]:
    try:
        parsed = json.loads(data)
    except json.JSONDecodeError:
        return {}

    if not isinstance(parsed, dict):
        return {}

    return parsed


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


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