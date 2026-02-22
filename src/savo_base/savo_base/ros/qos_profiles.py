#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/ros/qos_profiles.py
------------------------------------------
Professional ROS2 Jazzy QoS profiles for `savo_base`.

Purpose
-------
Centralize QoS policy choices used by `savo_base` nodes so all publishers/
subscribers use consistent, production-safe defaults during real robot testing.

Why this module exists
----------------------
QoS mismatches are a common source of silent ROS2 failures (especially with
mixed Python/C++ nodes and sensor/control pipelines). This file provides:
- named QoS profiles for common patterns
- helper builders for custom depths
- lightweight documentation in one place

Design principles
-----------------
- Conservative defaults for control/safety paths
- Explicit names (avoid "magic QoS" in nodes)
- Easy to override per-node if needed
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
)


# =============================================================================
# Low-level builders
# =============================================================================
def make_qos(
    *,
    depth: int,
    reliability: ReliabilityPolicy,
    durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
    history: HistoryPolicy = HistoryPolicy.KEEP_LAST,
    liveliness: LivelinessPolicy = LivelinessPolicy.AUTOMATIC,
) -> QoSProfile:
    """
    Build a QoSProfile with consistent explicit fields.

    Parameters
    ----------
    depth : int
        Queue depth for KEEP_LAST history.
    reliability : ReliabilityPolicy
        RELIABLE or BEST_EFFORT.
    durability : DurabilityPolicy
        VOLATILE or TRANSIENT_LOCAL.
    history : HistoryPolicy
        Usually KEEP_LAST.
    liveliness : LivelinessPolicy
        Usually AUTOMATIC for `savo_base` nodes.
    """
    depth_i = int(depth)
    if depth_i < 1:
        depth_i = 1

    return QoSProfile(
        history=history,
        depth=depth_i,
        reliability=reliability,
        durability=durability,
        liveliness=liveliness,
    )


def clone_with_depth(base: QoSProfile, depth: int) -> QoSProfile:
    """
    Clone an existing QoS profile with only depth changed.
    """
    depth_i = int(depth)
    if depth_i < 1:
        depth_i = 1

    return QoSProfile(
        history=base.history,
        depth=depth_i,
        reliability=base.reliability,
        durability=base.durability,
        liveliness=base.liveliness,
    )


# =============================================================================
# Named QoS profiles (Robot Savo base/control conventions)
# =============================================================================
# Control commands (/cmd_vel_safe -> base driver)
# Use RELIABLE to avoid dropped commands in local robot graph.
QOS_CMD_VEL = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Safety / estop / watchdog trip / enable flags
# Keep queue small and reliable; we care about the latest valid state.
QOS_SAFETY_BOOL = make_qos(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Internal base state JSON/String topics
# RELIABLE helps dashboards and orchestration nodes see consistent updates.
QOS_STATE_STRING = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Heartbeat/status pulses
# RELIABLE is preferred (small bandwidth, important supervisory signal).
QOS_HEARTBEAT = make_qos(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Diagnostics event stream (String JSON logs/events)
# Slightly larger queue to avoid losing burst output during diagnostics.
QOS_DIAG_EVENT = make_qos(
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Diagnostics state/busy topics
QOS_DIAG_STATE = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# Optional latched-style configuration/status snapshot topics
# New subscribers receive the last message immediately.
QOS_LATCHED_STATUS = make_qos(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# For debug/high-rate non-critical streams where drops are acceptable
QOS_DEBUG_BEST_EFFORT = make_qos(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


# =============================================================================
# Convenience factory helpers (node-side usage)
# =============================================================================
def qos_cmd_vel(depth: int = 10) -> QoSProfile:
    """
    QoS for command velocity topics (/cmd_vel, /cmd_vel_safe).
    """
    return clone_with_depth(QOS_CMD_VEL, depth)


def qos_safety_bool(depth: int = 5) -> QoSProfile:
    """
    QoS for boolean safety/control flags:
    - /savo_base/enable
    - /savo_base/estop
    - /savo_base/watchdog_trip
    """
    return clone_with_depth(QOS_SAFETY_BOOL, depth)


def qos_state_string(depth: int = 10) -> QoSProfile:
    """
    QoS for state/status JSON published on std_msgs/String.
    """
    return clone_with_depth(QOS_STATE_STRING, depth)


def qos_heartbeat(depth: int = 5) -> QoSProfile:
    """
    QoS for heartbeat topics.
    """
    return clone_with_depth(QOS_HEARTBEAT, depth)


def qos_diag_event(depth: int = 50) -> QoSProfile:
    """
    QoS for diagnostic event/log stream topics.
    """
    return clone_with_depth(QOS_DIAG_EVENT, depth)


def qos_diag_state(depth: int = 10) -> QoSProfile:
    """
    QoS for diagnostic state/busy topics.
    """
    return clone_with_depth(QOS_DIAG_STATE, depth)


def qos_latched_status() -> QoSProfile:
    """
    QoS for latched status snapshot topics (TRANSIENT_LOCAL).
    """
    return clone_with_depth(QOS_LATCHED_STATUS, 1)


def qos_debug_best_effort(depth: int = 10) -> QoSProfile:
    """
    QoS for non-critical debug streams where low latency matters more than delivery.
    """
    return clone_with_depth(QOS_DEBUG_BEST_EFFORT, depth)


# =============================================================================
# Optional named registry (useful for logs/debug UIs)
# =============================================================================
@dataclass(frozen=True)
class QoSDescriptor:
    name: str
    reliability: str
    durability: str
    history: str
    depth: int


def describe_qos(profile: QoSProfile, name: str = "custom") -> QoSDescriptor:
    """
    Convert a QoSProfile to a simple descriptor for logging/debugging.
    """
    return QoSDescriptor(
        name=name,
        reliability=getattr(profile.reliability, "name", str(profile.reliability)),
        durability=getattr(profile.durability, "name", str(profile.durability)),
        history=getattr(profile.history, "name", str(profile.history)),
        depth=int(profile.depth),
    )


def default_qos_registry() -> Dict[str, QoSProfile]:
    """
    Returns the canonical `savo_base` QoS registry.
    """
    return {
        "cmd_vel": QOS_CMD_VEL,
        "safety_bool": QOS_SAFETY_BOOL,
        "state_string": QOS_STATE_STRING,
        "heartbeat": QOS_HEARTBEAT,
        "diag_event": QOS_DIAG_EVENT,
        "diag_state": QOS_DIAG_STATE,
        "latched_status": QOS_LATCHED_STATUS,
        "debug_best_effort": QOS_DEBUG_BEST_EFFORT,
    }


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # builders
    "make_qos",
    "clone_with_depth",
    # named profiles
    "QOS_CMD_VEL",
    "QOS_SAFETY_BOOL",
    "QOS_STATE_STRING",
    "QOS_HEARTBEAT",
    "QOS_DIAG_EVENT",
    "QOS_DIAG_STATE",
    "QOS_LATCHED_STATUS",
    "QOS_DEBUG_BEST_EFFORT",
    # factories
    "qos_cmd_vel",
    "qos_safety_bool",
    "qos_state_string",
    "qos_heartbeat",
    "qos_diag_event",
    "qos_diag_state",
    "qos_latched_status",
    "qos_debug_best_effort",
    # descriptors/registry
    "QoSDescriptor",
    "describe_qos",
    "default_qos_registry",
]