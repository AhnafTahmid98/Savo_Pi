"""Convenience wrappers for LiDAR QoS profiles."""

from __future__ import annotations

from rclpy.qos import QoSProfile

from savo_lidar.ros.qos_profiles import (
    command_qos,
    latched_state_qos,
    sensor_qos,
    state_qos,
)


def scan_qos(depth: int = 10) -> QoSProfile:
    return sensor_qos(depth=depth)


def status_qos(depth: int = 10) -> QoSProfile:
    return state_qos(depth=depth)


def health_qos(depth: int = 10) -> QoSProfile:
    return state_qos(depth=depth)


def heartbeat_qos(depth: int = 10) -> QoSProfile:
    return state_qos(depth=depth)


def watchdog_qos(depth: int = 1) -> QoSProfile:
    return latched_state_qos(depth=depth)


def service_command_qos(depth: int = 10) -> QoSProfile:
    return command_qos(depth=depth)