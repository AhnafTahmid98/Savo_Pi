#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/safety/__init__.py
-----------------------------------------
Public exports for the `savo_base.safety` package.

Design goals
------------
- Stable import surface for ROS2 Jazzy nodes (e.g. `base_driver_node.py`)
- Keep safety policies modular and testable
- Allow graceful package evolution without changing caller imports

Typical usage
-------------
from savo_base.safety import (
    CommandGuard,
    EstopLatch,
    TimeoutWatchdog,
    TimeoutWatchdogConfig,
    StaleCommandPolicy,
    StaleCommandPolicyConfig,
    WatchdogPolicy,
    WatchdogPolicyConfig,
    MotionPermission,
)
"""

# NOTE:
# We intentionally export all core safety primitives from a single place so
# nodes can use short imports (`from savo_base.safety import ...`) and we can
# refactor internal file organization later without breaking callers.

from .command_guard import CommandGuard
from .estop_latch import EstopLatch
from .timeout_watchdog import TimeoutWatchdog, TimeoutWatchdogConfig
from .stale_command_policy import StaleCommandPolicy, StaleCommandPolicyConfig
from .watchdog_policy import WatchdogPolicy, WatchdogPolicyConfig, MotionPermission

__all__ = [
    # command validation / clamping
    "CommandGuard",

    # stop / e-stop state handling
    "EstopLatch",

    # timeout watchdog core
    "TimeoutWatchdog",
    "TimeoutWatchdogConfig",

    # stale command policy
    "StaleCommandPolicy",
    "StaleCommandPolicyConfig",

    # combined decision policy
    "WatchdogPolicy",
    "WatchdogPolicyConfig",
    "MotionPermission",
]