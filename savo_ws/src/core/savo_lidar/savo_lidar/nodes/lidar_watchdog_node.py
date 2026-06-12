#!/usr/bin/env python3
"""Monitor LiDAR scan freshness and publish watchdog state."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from savo_lidar.constants import (
    DEFAULT_SCAN_TOPIC,
    DEFAULT_STALE_TIMEOUT_S,
)
from savo_lidar.ros import get_float_param, get_string_param
from savo_lidar.safety import ScanWatchdog, StaleScanPolicy
from savo_lidar.utils import node_start_message, scan_qos, status_qos


class LidarWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_watchdog_node")

        self._scan_topic = get_string_param(self, "scan_topic", DEFAULT_SCAN_TOPIC)
        self._watchdog_topic = get_string_param(
            self,
            "watchdog_state_topic",
            "/savo_lidar/watchdog_state",
        )
        self._timeout_s = get_float_param(self, "stale_timeout_s", DEFAULT_STALE_TIMEOUT_S)
        self._publish_hz = get_float_param(self, "publish_hz", 2.0)
        self._warn_before_stale_ratio = get_float_param(
            self,
            "warn_before_stale_ratio",
            0.75,
        )

        if self._timeout_s <= 0.0:
            raise ValueError(f"stale_timeout_s must be > 0.0, got {self._timeout_s}")

        if self._publish_hz <= 0.0:
            raise ValueError(f"publish_hz must be > 0.0, got {self._publish_hz}")

        self._watchdog = ScanWatchdog(timeout_s=self._timeout_s)
        self._policy = StaleScanPolicy(
            warn_before_stale_ratio=self._warn_before_stale_ratio
        )

        self._state_pub = self.create_publisher(
            String,
            self._watchdog_topic,
            status_qos(),
        )
        self._scan_sub = self.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            scan_qos(),
        )

        self._timer = self.create_timer(1.0 / self._publish_hz, self._on_timer)

        self.get_logger().info(
            node_start_message(
                "lidar_watchdog_node",
                scan_topic=self._scan_topic,
                watchdog_topic=self._watchdog_topic,
                stale_timeout_s=self._timeout_s,
                publish_hz=self._publish_hz,
            )
        )

    def _on_scan(self, _scan: LaserScan) -> None:
        self._watchdog.mark_scan()

    def _on_timer(self) -> None:
        age_s = self._watchdog.age_s()
        decision = self._policy.evaluate(
            last_scan_age_s=age_s,
            timeout_s=self._timeout_s,
        )

        msg = String()
        msg.data = json.dumps(
            {
                "node": "lidar_watchdog_node",
                "status": decision.status,
                "message": decision.message,
                "scan_topic": self._scan_topic,
                "stale": decision.stale,
                "scan_ok": decision.scan_ok,
                "scan_count": self._watchdog.scan_count,
                "last_scan_age_s": age_s,
                "timeout_s": self._timeout_s,
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self._state_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarWatchdogNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()