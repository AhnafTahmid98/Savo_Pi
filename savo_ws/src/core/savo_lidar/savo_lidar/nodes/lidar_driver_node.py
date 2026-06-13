#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR driver node - publishes LaserScan from the selected backend."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from savo_lidar.constants import (
    BACKEND_DRYRUN,
    DEFAULT_BAUDRATE,
    DEFAULT_FRAME_ID,
    DEFAULT_HEARTBEAT_HZ,
    DEFAULT_HEARTBEAT_TOPIC,
    DEFAULT_LIDAR_MODEL,
    DEFAULT_MAX_RANGE_M,
    DEFAULT_MIN_RANGE_M,
    DEFAULT_PUBLISH_RATE_HZ,
    DEFAULT_SCAN_MODE,
    DEFAULT_SCAN_RATE_HZ,
    DEFAULT_SCAN_TOPIC,
    DEFAULT_SERIAL_PORT,
    DEFAULT_STALE_TIMEOUT_S,
    DEFAULT_STATE_TOPIC,
    MAX_ALLOWED_INF_RATIO_DEFAULT,
    MAX_ALLOWED_NAN_RATIO_DEFAULT,
    MIN_VALID_POINTS_DEFAULT,
    MOTOR_START_SETTLE_S_DEFAULT,
    MOTOR_STOP_TIMEOUT_S_DEFAULT,
    SCAN_RATE_MIN_HZ,
    SCAN_RATE_WARN_HZ,
    SERIAL_MAX_RECONNECT_DELAY_S_DEFAULT,
    SERIAL_RECONNECT_DELAY_S_DEFAULT,
    SERIAL_TIMEOUT_S_DEFAULT,
    STATUS_ERROR,
    STATUS_OK,
)
from savo_lidar.drivers import LidarDriverError, create_lidar_driver
from savo_lidar.filters import range_stats
from savo_lidar.models import LidarDriverConfig
from savo_lidar.ros import (
    get_bool_param,
    get_float_param,
    get_int_param,
    get_string_param,
)
from savo_lidar.utils import (
    RateTracker,
    node_start_message,
    node_stop_message,
    scan_qos,
    status_qos,
)
from savo_lidar.utils.diagnostics import make_status_payload, payload_to_json


class LidarDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_driver_node")

        self._config = self._load_config()
        self._driver = create_lidar_driver(self._config)
        self._rate_tracker = RateTracker()

        self._scan_count = 0
        self._last_error = ""
        self._started_ok = False

        self._scan_pub = self.create_publisher(
            LaserScan,
            self._config.scan_topic,
            scan_qos(),
        )

        self._state_pub = None
        if self._config.publish_driver_state:
            self._state_pub = self.create_publisher(
                String,
                self._config.driver_state_topic,
                status_qos(),
            )

        self._heartbeat_pub = self.create_publisher(
            String,
            self._config.heartbeat_topic,
            status_qos(depth=1),
        )

        self._start_driver()

        self._scan_timer = self.create_timer(
            1.0 / self._config.publish_rate_hz,
            self._on_scan_timer,
        )
        self._heartbeat_timer = self.create_timer(
            1.0 / self._config.heartbeat_hz,
            self._on_heartbeat_timer,
        )

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
            scan_topic=get_string_param(self, "scan_topic", DEFAULT_SCAN_TOPIC),
            frame_id=get_string_param(self, "frame_id", DEFAULT_FRAME_ID),
            serial_port=get_string_param(self, "serial_port", DEFAULT_SERIAL_PORT),
            baudrate=get_int_param(self, "baudrate", DEFAULT_BAUDRATE),
            scan_mode=get_string_param(self, "scan_mode", DEFAULT_SCAN_MODE),
            serial_timeout_s=get_float_param(
                self,
                "serial_timeout_s",
                SERIAL_TIMEOUT_S_DEFAULT,
            ),
            reconnect_on_error=get_bool_param(self, "reconnect_on_error", True),
            reconnect_delay_s=get_float_param(
                self,
                "reconnect_delay_s",
                SERIAL_RECONNECT_DELAY_S_DEFAULT,
            ),
            max_reconnect_delay_s=get_float_param(
                self,
                "max_reconnect_delay_s",
                SERIAL_MAX_RECONNECT_DELAY_S_DEFAULT,
            ),
            motor_start_settle_s=get_float_param(
                self,
                "motor_start_settle_s",
                MOTOR_START_SETTLE_S_DEFAULT,
            ),
            motor_stop_timeout_s=get_float_param(
                self,
                "motor_stop_timeout_s",
                MOTOR_STOP_TIMEOUT_S_DEFAULT,
            ),
            min_range_m=get_float_param(self, "min_range_m", DEFAULT_MIN_RANGE_M),
            max_range_m=get_float_param(self, "max_range_m", DEFAULT_MAX_RANGE_M),
            expected_scan_rate_hz=get_float_param(
                self,
                "expected_scan_rate_hz",
                DEFAULT_SCAN_RATE_HZ,
            ),
            min_scan_rate_hz=get_float_param(self, "min_scan_rate_hz", SCAN_RATE_MIN_HZ),
            scan_rate_warn_hz=get_float_param(self, "scan_rate_warn_hz", SCAN_RATE_WARN_HZ),
            publish_rate_hz=get_float_param(
                self,
                "publish_rate_hz",
                DEFAULT_PUBLISH_RATE_HZ,
            ),
            stale_timeout_s=get_float_param(self, "stale_timeout_s", DEFAULT_STALE_TIMEOUT_S),
            inverted=get_bool_param(self, "inverted", False),
            angle_compensate=get_bool_param(self, "angle_compensate", True),
            angle_offset_deg=get_float_param(self, "angle_offset_deg", 0.0),
            require_valid_scan=get_bool_param(self, "require_valid_scan", True),
            min_valid_points=get_int_param(self, "min_valid_points", MIN_VALID_POINTS_DEFAULT),
            max_allowed_inf_ratio=get_float_param(
                self,
                "max_allowed_inf_ratio",
                MAX_ALLOWED_INF_RATIO_DEFAULT,
            ),
            max_allowed_nan_ratio=get_float_param(
                self,
                "max_allowed_nan_ratio",
                MAX_ALLOWED_NAN_RATIO_DEFAULT,
            ),
            publish_driver_state=get_bool_param(self, "publish_driver_state", True),
            driver_state_topic=get_string_param(
                self,
                "driver_state_topic",
                DEFAULT_STATE_TOPIC,
            ),
            heartbeat_topic=get_string_param(
                self,
                "heartbeat_topic",
                DEFAULT_HEARTBEAT_TOPIC,
            ),
            heartbeat_hz=get_float_param(self, "heartbeat_hz", DEFAULT_HEARTBEAT_HZ),
            log_scan_summary=get_bool_param(self, "log_scan_summary", False),
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

    def _on_scan_timer(self) -> None:
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

            measured_rate_hz = self._rate_tracker.tick()
            stats = range_stats(
                scan_msg.ranges,
                min_range_m=scan_msg.range_min,
                max_range_m=scan_msg.range_max,
            )
            total_points, valid_points, valid_ratio, min_seen, max_seen, mean_seen = stats

            self._publish_state(
                status=STATUS_OK,
                message="scan published",
                hardware_ok=True,
                scan_ok=True,
                scan_count=self._scan_count,
                scan_rate_hz=measured_rate_hz,
                total_points=total_points,
                valid_points=valid_points,
                valid_ratio=valid_ratio,
                min_range_m=min_seen,
                max_range_m=max_seen,
                mean_range_m=mean_seen,
            )

            if self._config.log_scan_summary:
                self.get_logger().info(
                    "scan published | "
                    f"count={self._scan_count} "
                    f"rate_hz={measured_rate_hz:.2f} "
                    f"valid_ratio={valid_ratio:.2f}"
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

    def _on_heartbeat_timer(self) -> None:
        status = STATUS_OK if self._started_ok else STATUS_ERROR
        message = "driver running" if self._started_ok else "driver not running"

        msg = String()
        msg.data = payload_to_json(
            make_status_payload(
                component="lidar_driver_node",
                status=status,
                message=message,
                backend=self._config.backend,
                model=self._config.model,
                scan_count=self._scan_count,
                driver_running=self._driver.running,
                last_error=self._last_error or None,
            )
        )
        self._heartbeat_pub.publish(msg)

    def _to_laser_scan(self, scan_data: Any) -> LaserScan:
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._config.frame_id

        msg.angle_min = float(scan_data.angle_min_rad)
        msg.angle_max = float(scan_data.angle_max_rad)
        msg.angle_increment = float(scan_data.angle_increment_rad)

        scan_time_s = 1.0 / self._config.expected_scan_rate_hz
        point_count = len(scan_data.ranges)

        msg.time_increment = scan_time_s / float(point_count) if point_count > 0 else 0.0
        msg.scan_time = scan_time_s

        msg.range_min = float(scan_data.range_min_m)
        msg.range_max = float(scan_data.range_max_m)
        msg.ranges = list(scan_data.ranges)
        msg.intensities = list(scan_data.intensities)

        if self._config.inverted:
            msg.ranges.reverse()
            msg.intensities.reverse()

        return msg

    def _publish_state(self, *, status: str, message: str, **items: Any) -> None:
        if self._state_pub is None:
            return

        payload = make_status_payload(
            component="lidar_driver_node",
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
