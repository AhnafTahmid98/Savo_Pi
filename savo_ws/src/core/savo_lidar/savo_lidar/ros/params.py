# -*- coding: utf-8 -*-
"""Parameter helpers shared by LiDAR ROS nodes."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node


def declare_if_missing(node: Node, name: str, default_value: Any) -> Any:
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)

    return node.get_parameter(name).value


def get_string_param(node: Node, name: str, default_value: str) -> str:
    value = declare_if_missing(node, name, default_value)
    return str(value).strip()


def get_bool_param(node: Node, name: str, default_value: bool) -> bool:
    value = declare_if_missing(node, name, default_value)

    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in ("1", "true", "yes", "on")


def get_int_param(node: Node, name: str, default_value: int) -> int:
    value = declare_if_missing(node, name, int(default_value))
    return int(value)


def get_float_param(node: Node, name: str, default_value: float) -> float:
    value = declare_if_missing(node, name, float(default_value))
    return float(value)


def get_positive_float_param(node: Node, name: str, default_value: float) -> float:
    value = get_float_param(node, name, default_value)

    if value <= 0.0:
        node.get_logger().warning(
            f"Parameter '{name}' must be > 0.0. Using default {default_value}."
        )
        return float(default_value)

    return value


def get_range_param(
    node: Node,
    min_name: str,
    max_name: str,
    default_min: float,
    default_max: float,
) -> tuple[float, float]:
    min_value = get_float_param(node, min_name, default_min)
    max_value = get_float_param(node, max_name, default_max)

    if min_value >= max_value:
        node.get_logger().warning(
            f"Invalid range params: {min_name}={min_value}, {max_name}={max_value}. "
            f"Using defaults {default_min}..{default_max}."
        )
        return float(default_min), float(default_max)

    return min_value, max_value


def init_rclpy_if_needed(args: list[str] | None = None) -> bool:
    if rclpy.ok():
        return False

    rclpy.init(args=args)
    return True


__all__ = [
    "declare_if_missing",
    "get_bool_param",
    "get_float_param",
    "get_int_param",
    "get_positive_float_param",
    "get_range_param",
    "get_string_param",
    "init_rclpy_if_needed",
]
