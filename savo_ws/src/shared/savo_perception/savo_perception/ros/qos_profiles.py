#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Named QoS profiles for savo_perception Python fallback and diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


try:
    from rclpy.qos import (  # type: ignore
        DurabilityPolicy,
        HistoryPolicy,
        LivelinessPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    ROS_QOS_AVAILABLE = True

except Exception:
    ROS_QOS_AVAILABLE = False

    @dataclass(frozen=True)
    class _PolicyValue:
        name: str

        def __str__(self) -> str:
            return self.name

    class ReliabilityPolicy:
        RELIABLE = _PolicyValue("RELIABLE")
        BEST_EFFORT = _PolicyValue("BEST_EFFORT")

    class DurabilityPolicy:
        VOLATILE = _PolicyValue("VOLATILE")
        TRANSIENT_LOCAL = _PolicyValue("TRANSIENT_LOCAL")

    class HistoryPolicy:
        KEEP_LAST = _PolicyValue("KEEP_LAST")

    class LivelinessPolicy:
        AUTOMATIC = _PolicyValue("AUTOMATIC")

    @dataclass(frozen=True)
    class QoSProfile:
        history: object
        depth: int
        reliability: object
        durability: object
        liveliness: object


def is_ros_qos_available() -> bool:
    return ROS_QOS_AVAILABLE


def make_qos(
    *,
    depth: int,
    reliability,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    liveliness=LivelinessPolicy.AUTOMATIC,
) -> QoSProfile:
    depth_i = max(1, int(depth))

    return QoSProfile(
        history=history,
        depth=depth_i,
        reliability=reliability,
        durability=durability,
        liveliness=liveliness,
    )


def clone_with_depth(base: QoSProfile, depth: int) -> QoSProfile:
    depth_i = max(1, int(depth))

    return QoSProfile(
        history=base.history,
        depth=depth_i,
        reliability=base.reliability,
        durability=base.durability,
        liveliness=base.liveliness,
    )


QOS_CMD_VEL = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_RANGE_SENSOR = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

QOS_DEPTH_SENSOR = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

QOS_SAFETY_BOOL = make_qos(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_SLOWDOWN_FACTOR = make_qos(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_STATE_STRING = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_HEARTBEAT = make_qos(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_DIAG_EVENT = make_qos(
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_DIAG_STATE = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
)

QOS_LATCHED_STATUS = make_qos(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

QOS_DEBUG_BEST_EFFORT = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


def qos_cmd_vel(depth: int = 10) -> QoSProfile:
    return clone_with_depth(QOS_CMD_VEL, depth)


def qos_range_sensor(depth: int = 10) -> QoSProfile:
    return clone_with_depth(QOS_RANGE_SENSOR, depth)


def qos_depth_sensor(depth: int = 10) -> QoSProfile:
    return clone_with_depth(QOS_DEPTH_SENSOR, depth)


def qos_safety_bool(depth: int = 5) -> QoSProfile:
    return clone_with_depth(QOS_SAFETY_BOOL, depth)


def qos_slowdown_factor(depth: int = 5) -> QoSProfile:
    return clone_with_depth(QOS_SLOWDOWN_FACTOR, depth)


def qos_state_string(depth: int = 10) -> QoSProfile:
    return clone_with_depth(QOS_STATE_STRING, depth)


def qos_heartbeat(depth: int = 5) -> QoSProfile:
    return clone_with_depth(QOS_HEARTBEAT, depth)


def qos_diag_event(depth: int = 50) -> QoSProfile:
    return clone_with_depth(QOS_DIAG_EVENT, depth)


def qos_diag_state(depth: int = 10) -> QoSProfile:
    return clone_with_depth(QOS_DIAG_STATE, depth)


def qos_latched_status() -> QoSProfile:
    return clone_with_depth(QOS_LATCHED_STATUS, 1)


def qos_debug_best_effort(depth: int = 10) -> QoSProfile:
    return clone_with_depth(QOS_DEBUG_BEST_EFFORT, depth)


@dataclass(frozen=True)
class QoSDescriptor:
    name: str
    reliability: str
    durability: str
    history: str
    depth: int
    ros_qos_available: bool


def _policy_name(value) -> str:
    return getattr(value, "name", str(value))


def describe_qos(profile: QoSProfile, name: str = "custom") -> QoSDescriptor:
    return QoSDescriptor(
        name=name,
        reliability=_policy_name(profile.reliability),
        durability=_policy_name(profile.durability),
        history=_policy_name(profile.history),
        depth=int(profile.depth),
        ros_qos_available=ROS_QOS_AVAILABLE,
    )


def default_qos_registry() -> Dict[str, QoSProfile]:
    return {
        "cmd_vel": QOS_CMD_VEL,
        "range_sensor": QOS_RANGE_SENSOR,
        "depth_sensor": QOS_DEPTH_SENSOR,
        "safety_bool": QOS_SAFETY_BOOL,
        "slowdown_factor": QOS_SLOWDOWN_FACTOR,
        "state_string": QOS_STATE_STRING,
        "heartbeat": QOS_HEARTBEAT,
        "diag_event": QOS_DIAG_EVENT,
        "diag_state": QOS_DIAG_STATE,
        "latched_status": QOS_LATCHED_STATUS,
        "debug_best_effort": QOS_DEBUG_BEST_EFFORT,
    }


__all__ = [
    "ROS_QOS_AVAILABLE",
    "is_ros_qos_available",
    "make_qos",
    "clone_with_depth",
    "QOS_CMD_VEL",
    "QOS_RANGE_SENSOR",
    "QOS_DEPTH_SENSOR",
    "QOS_SAFETY_BOOL",
    "QOS_SLOWDOWN_FACTOR",
    "QOS_STATE_STRING",
    "QOS_HEARTBEAT",
    "QOS_DIAG_EVENT",
    "QOS_DIAG_STATE",
    "QOS_LATCHED_STATUS",
    "QOS_DEBUG_BEST_EFFORT",
    "qos_cmd_vel",
    "qos_range_sensor",
    "qos_depth_sensor",
    "qos_safety_bool",
    "qos_slowdown_factor",
    "qos_state_string",
    "qos_heartbeat",
    "qos_diag_event",
    "qos_diag_state",
    "qos_latched_status",
    "qos_debug_best_effort",
    "QoSDescriptor",
    "describe_qos",
    "default_qos_registry",
]