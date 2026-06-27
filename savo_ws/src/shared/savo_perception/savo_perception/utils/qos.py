#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""QoS compatibility helpers for Python fallback nodes and tools."""

from __future__ import annotations

from savo_perception.ros.qos_profiles import (
    QOS_CMD_VEL,
    QOS_DEBUG_BEST_EFFORT,
    QOS_DEPTH_SENSOR,
    QOS_DIAG_EVENT,
    QOS_DIAG_STATE,
    QOS_HEARTBEAT,
    QOS_LATCHED_STATUS,
    QOS_RANGE_SENSOR,
    QOS_SAFETY_BOOL,
    QOS_SLOWDOWN_FACTOR,
    QOS_STATE_STRING,
    QoSDescriptor,
    clone_with_depth,
    default_qos_registry,
    describe_qos,
    is_ros_qos_available,
    make_qos,
    qos_cmd_vel,
    qos_debug_best_effort,
    qos_depth_sensor,
    qos_diag_event,
    qos_diag_state,
    qos_heartbeat,
    qos_latched_status,
    qos_range_sensor,
    qos_safety_bool,
    qos_slowdown_factor,
    qos_state_string,
)


def qos_for_key(key: str):
    registry = default_qos_registry()
    try:
        return registry[key]
    except KeyError as exc:
        known = ", ".join(sorted(registry.keys()))
        raise KeyError(f"Unknown QoS key '{key}'. Known keys: {known}") from exc


def describe_qos_key(key: str) -> QoSDescriptor:
    return describe_qos(qos_for_key(key), key)


__all__ = [
    "QOS_CMD_VEL",
    "QOS_DEBUG_BEST_EFFORT",
    "QOS_DEPTH_SENSOR",
    "QOS_DIAG_EVENT",
    "QOS_DIAG_STATE",
    "QOS_HEARTBEAT",
    "QOS_LATCHED_STATUS",
    "QOS_RANGE_SENSOR",
    "QOS_SAFETY_BOOL",
    "QOS_SLOWDOWN_FACTOR",
    "QOS_STATE_STRING",
    "QoSDescriptor",
    "clone_with_depth",
    "default_qos_registry",
    "describe_qos",
    "describe_qos_key",
    "is_ros_qos_available",
    "make_qos",
    "qos_cmd_vel",
    "qos_debug_best_effort",
    "qos_depth_sensor",
    "qos_diag_event",
    "qos_diag_state",
    "qos_for_key",
    "qos_heartbeat",
    "qos_latched_status",
    "qos_range_sensor",
    "qos_safety_bool",
    "qos_slowdown_factor",
    "qos_state_string",
]