#!/usr/bin/env python3
"""Publish LaserScan data from the selected LiDAR backend."""

from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    DEFAULT_BAUDRATE,
    DEFAULT_FRAME_ID,
    DEFAULT_LIDAR_MODEL,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_SCAN_MODE,
    DEFAULT_SCAN_TOPIC,
    DEFAULT_SERIAL_PORT,
    DEFAULT_STALE_TIMEOUT_S,
    STATUS_ERROR,
    STATUS_OK,
)
from savo_lidar.drivers import LidarDriverError, create_lidar_driver
from savo_lidar.models import LidarDriverConfig
from savo_lidar.ros import get_bool_param, get_float_param, get_int_param, get_string_param
from savo_lidar.utils import node_start_message, node_stop_message, scan_qos, status_qos
from savo_lidar.utils.diagnostics import make_status_payload, payload_to_json


class LidarDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_driver_node")

        self._config = self._load_config()
        self._driver = create_lidar_driver(self._config)

        self._scan_pub = self.create_publisher(
            LaserScan,
            self._config.scan_topic,
            scan_qos(),
        )
        self._state_pub = self.create_publisher(
            String,
            "/savo_lidar/state",
            status_qos(),
        )

        self._scan_count = 0
        self._last_error = ""
        self._started_ok = False

        self._start_driver()

        timer_period_s = 1.0 / self._config.publish_rate_hz
        self._timer = self.create_timer(timer_period_s, self._on_timer)

        self.get_logger().info(
            node_start_message(
                "lidar_driver_node",
                backend=self._config.backend,
                model=self._config.model,
                port=self._config.serial_port if self._config.is_real_backend else None,
                baudrate=self._config.baudrate if self._config.is_real_backend else None,
                frame_id=self._config.frame_id,
                scan_topic=self._config.scan_topic,
                publish_rate_hz=self._config.publish_rate_hz,
            )
        )

    def destroy_node(self) -> bool:
        try:
            self._driver.stop()
            self.get_logger().info(node_stop_message("lidar_driver_node"))
        except Exception as exc:
            self.get_logger().warning(f"LiDAR driver stop failed: {exc}")

        return super().destroy_node()

    def _load_config(self) -> LidarDriverConfig:
        config = LidarDriverConfig(
            model=get_string_param(self, "model", DEFAULT_LIDAR_MODEL),
            backend=get_string_param(self, "backend", BACKEND_DRYRUN),
            serial_port=get_string_param(self, "serial_port", DEFAULT_SERIAL_PORT),
            baudrate=get_int_param(self, "baudrate", DEFAULT_BAUDRATE),
            frame_id=get_string_param(self, "frame_id", DEFAULT_FRAME_ID),
            scan_topic=get_string_param(self, "scan_topic", DEFAULT_SCAN_TOPIC),
            scan_mode=get_string_param(self, "scan_mode", DEFAULT_SCAN_MODE),
            min_range_m=get_float_param(self, "min_range_m", DEFAULT_MIN_RANGE_M),
            max_range_m=get_float_param(self, "max_range_m", DEFAULT_MAX_RANGE_M),
            expected_scan_rate_hz=get_float_param(self, "expected_scan_rate_hz", 5.5),
            publish_rate_hz=get_float_param(self, "publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ),
            stale_timeout_s=get_float_param(self, "stale_timeout_s", DEFAULT_STALE_TIMEOUT_S),
            inverted=get_bool_param(self, "inverted", False),
            angle_compensate=get_bool_param(self, "angle_compensate", True),
        )
        config.validate()
        return config

    def _start_driver(self) -> None:
        try:
            self._driver.start()
            self._started_ok = True
            self._last_error = ""
        except Exception as exc:
            self._started_ok = False
            self._last_error = str(exc)
            self.get_logger().error(f"Failed to start LiDAR driver: {exc}")

    def _on_timer(self) -> None:
        if not self._started_ok:
            self._publish_state(
                status=STATUS_ERROR,
                message=self._last_error or "LiDAR driver did not start",
                hardware_ok=False,
                scan_ok=False,
            )
            return

        try:
            scan_data = self._driver.read_scan()
            scan_msg = self._to_laser_scan(scan_data)
            self._scan_pub.publish(scan_msg)

            self._scan_count += 1
            self._publish_state(
                status=STATUS_OK,
                message="scan published",
                hardware_ok=True,
                scan_ok=True,
                scan_count=self._scan_count,
            )
        except LidarDriverError as exc:
            self._last_error = str(exc)
            self.get_logger().warning(f"LiDAR read failed: {exc}")
            self._publish_state(
                status=STATUS_ERROR,
                message=self._last_error,
                hardware_ok=True,
                scan_ok=False,
                scan_count=self._scan_count,
            )
        except Exception as exc:
            self._last_error = str(exc)
            self.get_logger().error(f"Unexpected LiDAR driver error: {exc}")
            self._publish_state(
                status=STATUS_ERROR,
                message=self._last_error,
                hardware_ok=False,
                scan_ok=False,
                scan_count=self._scan_count,
            )

    def _to_laser_scan(self, scan_data: Any) -> LaserScan:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._config.frame_id

        msg.angle_min = float(scan_data.angle_min_rad)
        msg.angle_max = float(scan_data.angle_max_rad)
        msg.angle_increment = float(scan_data.angle_increment_rad)

        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self._config.expected_scan_rate_hz

        msg.range_min = float(scan_data.range_min_m)
        msg.range_max = float(scan_data.range_max_m)
        msg.ranges = list(scan_data.ranges)
        msg.intensities = list(scan_data.intensities)

        if self._config.inverted:
            msg.ranges.reverse()
            msg.intensities.reverse()

        return msg

    def _publish_state(self, *, status: str, message: str, **items: Any) -> None:
        payload = make_status_payload(
            node="lidar_driver_node",
            status=status,
            message=message,
            backend=self._config.backend,
            model=self._config.model,
            frame_id=self._config.frame_id,
            scan_topic=self._config.scan_topic,
            driver_running=self._driver.running,
            **items,
        )

        msg = String()
        msg.data = payload_to_json(payload)
        self._state_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LidarDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()