# -*- coding: utf-8 -*-
"""QoS helpers for LiDAR utility code."""

from __future__ import annotations

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from savo_lidar.constants import (
    QOS_DEPTH_DEFAULT,
    QOS_SENSOR_DEPTH_DEFAULT,
    QOS_STATE_DEPTH_DEFAULT,
)


def scan_qos(depth: int = QOS_SENSOR_DEPTH_DEFAULT) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos(depth: int = QOS_STATE_DEPTH_DEFAULT) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def health_qos(depth: int = QOS_STATE_DEPTH_DEFAULT) -> QoSProfile:
    return status_qos(depth)


def watchdog_qos(depth: int = QOS_STATE_DEPTH_DEFAULT) -> QoSProfile:
    return status_qos(depth)


def heartbeat_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def service_command_qos(depth: int = QOS_DEPTH_DEFAULT) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _positive_depth(depth: int) -> int:
    return max(1, int(depth))


__all__ = [
    "health_qos",
    "heartbeat_qos",
    "scan_qos",
    "service_command_qos",
    "status_qos",
    "watchdog_qos",
]
