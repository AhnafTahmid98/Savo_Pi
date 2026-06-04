#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python controller helpers for testing and diagnostics. C++ remains the runtime path."""

# Generic PID core
from .pid_py import PidConfig, PidResult, Pid

# Heading/yaw controller
from .heading_pid_py import (
    HeadingControllerConfig,
    HeadingControllerResult,
    HeadingController,
    HeadingPid,
)

# Distance/approach controller
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