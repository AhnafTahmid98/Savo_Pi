#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Python fallback and diagnostic ROS nodes for savo_perception."""

from __future__ import annotations

from typing import Final


VL53_MUX_NODE_PY: Final[str] = "vl53_mux_node_py"
ULTRASONIC_NODE_PY: Final[str] = "ultrasonic_node_py"
SAFETY_STOP_NODE_PY: Final[str] = "safety_stop_node_py"
RANGE_HEALTH_NODE_PY: Final[str] = "range_health_node_py"
SENSOR_DASHBOARD_NODE: Final[str] = "sensor_dashboard_node"


PYTHON_FALLBACK_NODES: Final[tuple[str, ...]] = (
    VL53_MUX_NODE_PY,
    ULTRASONIC_NODE_PY,
    SAFETY_STOP_NODE_PY,
    RANGE_HEALTH_NODE_PY,
)

DEBUG_NODES: Final[tuple[str, ...]] = (
    SENSOR_DASHBOARD_NODE,
)


def list_python_fallback_nodes() -> tuple[str, ...]:
    return PYTHON_FALLBACK_NODES


def list_debug_nodes() -> tuple[str, ...]:
    return DEBUG_NODES


def list_all_python_nodes() -> tuple[str, ...]:
    return PYTHON_FALLBACK_NODES + DEBUG_NODES


__all__ = [
    "VL53_MUX_NODE_PY",
    "ULTRASONIC_NODE_PY",
    "SAFETY_STOP_NODE_PY",
    "RANGE_HEALTH_NODE_PY",
    "SENSOR_DASHBOARD_NODE",
    "PYTHON_FALLBACK_NODES",
    "DEBUG_NODES",
    "list_python_fallback_nodes",
    "list_debug_nodes",
    "list_all_python_nodes",
]