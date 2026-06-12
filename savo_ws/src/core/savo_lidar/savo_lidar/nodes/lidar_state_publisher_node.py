#!/usr/bin/env python3
"""Publish a compact LiDAR state summary for dashboards and logs."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from savo_lidar.constants import STATUS_OFFLINE
from savo_lidar.ros import get_float_param, get_string_param
from savo_lidar.utils import node_start_message, status_qos


class LidarStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_state_publisher_node")

        self._driver_state_topic = get_string_param(
            self,
            "driver_state_topic",
            "/savo_lidar/state",
        )
        self._health_topic = get_string_param(
            self,
            "health_topic",
            "/savo_lidar/health",
        )
        self._watchdog_topic = get_string_param(
            self,
            "watchdog_state_topic",
            "/savo_lidar/watchdog_state",
        )
        self._summary_topic = get_string_param(
            self,
            "summary_topic",
            "/savo_lidar/state_summary",
        )
        self._publish_hz = get_float_param(self, "publish_hz", 1.0)

        if self._publish_hz <= 0.0:
            raise ValueError(f"publish_hz must be > 0.0, got {self._publish_hz}")

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
            self._on_health,
            status_qos(),
        )
        self._watchdog_sub = self.create_subscription(
            String,
            self._watchdog_topic,
            self._on_watchdog,
            status_qos(),
        )

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            node_start_message(
                "lidar_state_publisher_node",
                driver_state_topic=self._driver_state_topic,
                health_topic=self._health_topic,
                watchdog_topic=self._watchdog_topic,
                summary_topic=self._summary_topic,
                publish_hz=self._publish_hz,
            )
        )

    def _on_driver_state(self, msg: String) -> None:
        self._driver_state = _parse_json_msg(msg.data)

    def _on_health(self, msg: String) -> None:
        self._health_state = _parse_json_msg(msg.data)

    def _on_watchdog(self, msg: String) -> None:
        self._watchdog_state = _parse_json_msg(msg.data)

    def _on_timer(self) -> None:
        status = str(self._health_state.get("status", STATUS_OFFLINE))
        message = str(self._health_state.get("message", "waiting for LiDAR health"))

        payload = {
            "node": "lidar_state_publisher_node",
            "status": status,
            "message": message,
            "backend": self._driver_state.get("backend"),
            "model": self._driver_state.get("model"),
            "frame_id": self._driver_state.get("frame_id"),
            "scan_topic": self._driver_state.get("scan_topic"),
            "driver_running": bool(self._driver_state.get("driver_running", False)),
            "hardware_ok": bool(self._health_state.get("hardware_ok", False)),
            "scan_ok": bool(self._health_state.get("scan_ok", False)),
            "stale": bool(self._watchdog_state.get("stale", True)),
            "scan_count": int(self._watchdog_state.get("scan_count", 0) or 0),
            "last_scan_age_s": self._watchdog_state.get("last_scan_age_s"),
            "scan_rate_hz": _safe_float(self._health_state.get("scan_rate_hz", 0.0)),
            "valid_ratio": _safe_float(self._health_state.get("valid_ratio", 0.0)),
            "fault_latched": bool(self._health_state.get("fault_latched", False)),
            "fault_reason": str(self._health_state.get("fault_reason", "")),
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._summary_pub.publish(msg)


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