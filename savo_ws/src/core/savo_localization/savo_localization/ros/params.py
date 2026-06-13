#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Parameter helpers for Robot Savo localization Python nodes."""

from __future__ import annotations

from typing import Any

import rclpy
from rclpy.node import Node


def init_rclpy_if_needed(args: list[str] | None = None) -> bool:
    if rclpy.ok():
        return False

    rclpy.init(args=args)
    return True


def declare_if_missing(node: Node, name: str, default_value: Any) -> None:
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


def get_string_param(node: Node, name: str, default_value: str) -> str:
    declare_if_missing(node, name, default_value)
    value = node.get_parameter(name).value

    if value is None:
        return str(default_value)

    return str(value)


def get_int_param(node: Node, name: str, default_value: int) -> int:
    declare_if_missing(node, name, int(default_value))
    value = node.get_parameter(name).value

    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default_value)


def get_float_param(node: Node, name: str, default_value: float) -> float:
    declare_if_missing(node, name, float(default_value))
    value = node.get_parameter(name).value

    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default_value)


def get_bool_param(node: Node, name: str, default_value: bool) -> bool:
    declare_if_missing(node, name, bool(default_value))
    value = node.get_parameter(name).value

    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")

    return bool(value)


def get_string_list_param(
    node: Node,
    name: str,
    default_value: list[str] | tuple[str, ...],
) -> list[str]:
    declare_if_missing(node, name, list(default_value))
    value = node.get_parameter(name).value

    if value is None:
        return list(default_value)

    if isinstance(value, str):
        return [value]

    try:
        return [str(item) for item in value]
    except TypeError:
        return list(default_value)


def get_int_list_param(
    node: Node,
    name: str,
    default_value: list[int] | tuple[int, ...],
) -> list[int]:
    declare_if_missing(node, name, list(default_value))
    value = node.get_parameter(name).value

    if value is None:
        return list(default_value)

    try:
        return [int(item) for item in value]
    except (TypeError, ValueError):
        return list(default_value)


def get_float_list_param(
    node: Node,
    name: str,
    default_value: list[float] | tuple[float, ...],
) -> list[float]:
    declare_if_missing(node, name, list(default_value))
    value = node.get_parameter(name).value

    if value is None:
        return list(default_value)

    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return list(default_value)


def get_positive_float_param(node: Node, name: str, default_value: float) -> float:
    value = get_float_param(node, name, default_value)

    if value <= 0.0:
        raise ValueError(f"{name} must be > 0.0, got {value}")

    return value


def get_non_negative_float_param(node: Node, name: str, default_value: float) -> float:
    value = get_float_param(node, name, default_value)

    if value < 0.0:
        raise ValueError(f"{name} must be >= 0.0, got {value}")

    return value


def get_positive_int_param(node: Node, name: str, default_value: int) -> int:
    value = get_int_param(node, name, default_value)

    if value <= 0:
        raise ValueError(f"{name} must be > 0, got {value}")

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

    if max_value <= min_value:
        raise ValueError(
            f"{max_name} must be greater than {min_name}: "
            f"{min_name}={min_value}, {max_name}={max_value}"
        )

    return min_value, max_value
