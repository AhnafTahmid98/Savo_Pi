#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Savo — savo_localization/utils/params.py
----------------------------------------------
Professional ROS2 parameter utilities for localization nodes.

Why this file exists:
- Keep parameter declaration + reading consistent across imu_node / wheel_odom / EKF helpers
- Reduce boilerplate and prevent silent type mistakes (YAML int vs float, list vs scalar)
- Provide light validation (range checks) and clear error messages
- Safe to import outside ROS2: ROS interfaces are imported lazily only when needed

Usage:
    from savo_localization.utils.params import (
        ParamSpec, declare_params,
        get_param_float, get_param_int, get_param_bool, get_param_str
    )

    declare_params(self, [
        ParamSpec("rate_hz", 50.0, "Loop rate (Hz)"),
        ParamSpec("track_width_m", 0.165, "Track width (m)"),
    ])

    rate_hz = get_param_float(self, "rate_hz", min_value=1.0, max_value=200.0)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence, Tuple, Type, Union


# ---------------------------
# Types
# ---------------------------

ParamScalar = Union[bool, int, float, str]
ParamValue = Union[ParamScalar, Sequence[ParamScalar]]


@dataclass(frozen=True)
class ParamSpec:
    """
    Parameter declaration spec.

    name: parameter name
    default: default value (scalar, list, or tuple)
    description: shown via `ros2 param describe`
    """
    name: str
    default: ParamValue
    description: str = ""


# ---------------------------
# Public API
# ---------------------------

def declare_params(
    node: Any,
    specs: Iterable[Union[ParamSpec, Tuple[str, ParamValue], Tuple[str, ParamValue, str]]],
) -> None:
    """
    Declare a set of parameters on a ROS2 node.

    Specs can be:
      - ParamSpec(...)
      - ("name", default)
      - ("name", default, "description")
    """
    for item in specs:
        spec = _to_spec(item)

        if spec.description:
            descriptor = _make_descriptor(spec.description)
            if descriptor is not None:
                node.declare_parameter(spec.name, spec.default, descriptor)
            else:
                node.declare_parameter(spec.name, spec.default)
        else:
            node.declare_parameter(spec.name, spec.default)


def get_param(
    node: Any,
    name: str,
    expected_type: Type,
    *,
    default: Optional[Any] = None,
    min_value: Optional[float] = None,
    max_value: Optional[float] = None,
    allow_none: bool = False,
) -> Any:
    """
    Read a parameter with type enforcement + optional numeric range checks.

    Notes:
    - YAML often provides ints where floats are expected (e.g., 30 instead of 30.0).
      We allow int->float coercion when expected_type is float.

    Raises:
        ValueError on type mismatch or range violation.
    """
    p = node.get_parameter(name)
    value = getattr(p, "value", None)

    if value is None:
        if allow_none:
            return None
        if default is not None:
            return default
        raise ValueError(f"Parameter '{name}' is unset (None) and no default was provided")

    # List/tuple parameters
    if expected_type in (list, tuple):
        if not isinstance(value, (list, tuple)):
            raise ValueError(f"Parameter '{name}' expected {expected_type.__name__}, got {type(value).__name__}")
        return list(value) if expected_type is list else tuple(value)

    # Scalar: allow int->float if expected float
    if expected_type is float and isinstance(value, int):
        value = float(value)

    if not isinstance(value, expected_type):
        raise ValueError(f"Parameter '{name}' expected {expected_type.__name__}, got {type(value).__name__}")

    if isinstance(value, (int, float)):
        _validate_range(name, float(value), min_value, max_value)

    return value


def get_param_bool(node: Any, name: str, default: Optional[bool] = None) -> bool:
    return bool(get_param(node, name, bool, default=default))


def get_param_int(
    node: Any,
    name: str,
    default: Optional[int] = None,
    *,
    min_value: Optional[int] = None,
    max_value: Optional[int] = None,
) -> int:
    v = int(get_param(node, name, int, default=default))
    _validate_range(
        name,
        float(v),
        None if min_value is None else float(min_value),
        None if max_value is None else float(max_value),
    )
    return v


def get_param_float(
    node: Any,
    name: str,
    default: Optional[float] = None,
    *,
    min_value: Optional[float] = None,
    max_value: Optional[float] = None,
) -> float:
    v = float(get_param(node, name, float, default=default))
    _validate_range(name, v, min_value, max_value)
    return v


def get_param_str(node: Any, name: str, default: Optional[str] = None) -> str:
    return str(get_param(node, name, str, default=default))


def get_param_list(node: Any, name: str, default: Optional[List[Any]] = None) -> List[Any]:
    v = get_param(node, name, list, default=default if default is not None else [])
    return list(v)


def get_param_tuple(node: Any, name: str, default: Optional[Tuple[Any, ...]] = None) -> Tuple[Any, ...]:
    v = get_param(node, name, tuple, default=default if default is not None else tuple())
    return tuple(v)


# ---------------------------
# Internal helpers
# ---------------------------

def _to_spec(item: Union[ParamSpec, Tuple[str, ParamValue], Tuple[str, ParamValue, str]]) -> ParamSpec:
    if isinstance(item, ParamSpec):
        return item
    if isinstance(item, tuple) and len(item) == 2:
        return ParamSpec(name=str(item[0]), default=item[1], description="")
    if isinstance(item, tuple) and len(item) == 3:
        return ParamSpec(name=str(item[0]), default=item[1], description=str(item[2]))
    raise ValueError(f"Invalid parameter spec: {item!r}")


def _validate_range(name: str, value: float, min_value: Optional[float], max_value: Optional[float]) -> None:
    if min_value is not None and value < min_value:
        raise ValueError(f"Parameter '{name}' out of range: {value} < min {min_value}")
    if max_value is not None and value > max_value:
        raise ValueError(f"Parameter '{name}' out of range: {value} > max {max_value}")


def _make_descriptor(description: str) -> Optional[Any]:
    """
    Lazily create rcl_interfaces.msg.ParameterDescriptor.
    Returns None if ROS interfaces aren't available (unit test environment).
    """
    try:
        from rcl_interfaces.msg import ParameterDescriptor  # type: ignore
        return ParameterDescriptor(description=description)
    except Exception:
        return None