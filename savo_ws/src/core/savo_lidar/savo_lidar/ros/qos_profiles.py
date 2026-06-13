# -*- coding: utf-8 -*-
"""QoS profiles for LiDAR ROS nodes."""

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


def sensor_qos(depth: int = QOS_SENSOR_DEPTH_DEFAULT) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def state_qos(depth: int = QOS_STATE_DEPTH_DEFAULT) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def latched_state_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def command_qos(depth: int = QOS_DEPTH_DEFAULT) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=_positive_depth(depth),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _positive_depth(depth: int) -> int:
    return max(1, int(depth))


__all__ = [
    "command_qos",
    "latched_state_qos",
    "sensor_qos",
    "state_qos",
]
