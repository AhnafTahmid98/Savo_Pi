#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Public exports for savo_base.safety: guard, estop latch, watchdog, and policy."""

from .command_guard import CommandGuard, CommandLimits, Velocity3
from .estop_latch import EStopLatch
from .timeout_watchdog import TimeoutWatchdog, TimeoutWatchdogConfig
from .stale_command_policy import StaleCommandPolicy, StaleCommandPolicyConfig
from .watchdog_policy import WatchdogPolicy, WatchdogPolicyConfig, MotionPermission

# Compatibility alias
EstopLatch = EStopLatch

__all__ = [
    "CommandGuard",
    "CommandLimits",
    "Velocity3",

    "EStopLatch",
    "EstopLatch",

    "TimeoutWatchdog",
    "TimeoutWatchdogConfig",

    "StaleCommandPolicy",
    "StaleCommandPolicyConfig",

    "WatchdogPolicy",
    "WatchdogPolicyConfig",
    "MotionPermission",
]
