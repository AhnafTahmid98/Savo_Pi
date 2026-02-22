#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_control.controllers

Python-side reusable controller helpers for testing, tuning, and diagnostics.

Notes
-----
- C++ controllers remain the authoritative production/runtime path.
- These Python modules mirror core behavior for prototyping and tooling.
"""

# Generic scalar PID core (Python mirror of C++ pid.hpp)
from .pid_py import PidConfig, PidResult, Pid

# Heading/yaw controller wrapper built on top of Pid
from .heading_pid_py import (
    HeadingControllerConfig,
    HeadingControllerResult,
    HeadingController,
    HeadingPid,
)

# Distance/approach controller wrapper built on top of Pid
from .distance_pid_py import (
    DistanceControllerConfig,
    DistanceControllerResult,
    DistancePid,
    DistanceController,
)

__all__ = [
    # Generic PID core
    "PidConfig",
    "PidResult",
    "Pid",
    # Heading controller
    "HeadingControllerConfig",
    "HeadingControllerResult",
    "HeadingController",
    "HeadingPid",
    # Distance controller
    "DistanceControllerConfig",
    "DistanceControllerResult",
    "DistancePid",
    "DistanceController",
]