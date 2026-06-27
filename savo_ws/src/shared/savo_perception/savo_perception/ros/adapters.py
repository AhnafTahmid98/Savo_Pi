#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small ROS message adapter helpers for Python fallback nodes and diagnostics."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional


try:
    from geometry_msgs.msg import Twist  # type: ignore
    from std_msgs.msg import Bool, Float32, String  # type: ignore

    ROS_MSGS_AVAILABLE = True

except Exception:
    ROS_MSGS_AVAILABLE = False

    @dataclass
    class Float32:
        data: float = 0.0

    @dataclass
    class Bool:
        data: bool = False

    @dataclass
    class String:
        data: str = ""

    @dataclass
    class _Vector3:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0

    class Twist:
        def __init__(self) -> None:
            self.linear = _Vector3()
            self.angular = _Vector3()


def is_ros_msgs_available() -> bool:
    return ROS_MSGS_AVAILABLE


def make_float32(value: Any, *, default: float = 0.0) -> Float32:
    msg = Float32()
    try:
        value_f = float(value)
    except Exception:
        value_f = float(default)

    msg.data = value_f
    return msg


def make_bool(value: Any) -> Bool:
    msg = Bool()

    if isinstance(value, bool):
        msg.data = value
        return msg

    if isinstance(value, (int, float)):
        msg.data = bool(value)
        return msg

    text = str(value).strip().lower()
    msg.data = text in {"1", "true", "yes", "on"}
    return msg


def make_string(value: Any) -> String:
    msg = String()
    msg.data = str(value)
    return msg


def make_json_string(payload: Mapping[str, Any], *, sort_keys: bool = True) -> String:
    return make_string(json.dumps(dict(payload), sort_keys=sort_keys, separators=(",", ":")))


def float_from_msg(msg: Any, *, default: Optional[float] = None) -> Optional[float]:
    try:
        return float(getattr(msg, "data", msg))
    except Exception:
        return default


def bool_from_msg(msg: Any, *, default: bool = False) -> bool:
    value = getattr(msg, "data", msg)

    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)):
        return bool(value)

    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on"}:
        return True
    if text in {"0", "false", "no", "off"}:
        return False

    return default


def string_from_msg(msg: Any, *, default: str = "") -> str:
    try:
        return str(getattr(msg, "data", msg))
    except Exception:
        return default


def json_from_string_msg(msg: Any, *, default: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    text = string_from_msg(msg, default="")
    if not text:
        return dict(default or {})

    try:
        parsed = json.loads(text)
    except Exception:
        return dict(default or {})

    return parsed if isinstance(parsed, dict) else dict(default or {})


def make_twist(
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    wz: float = 0.0,
    vz: float = 0.0,
    wx: float = 0.0,
    wy: float = 0.0,
) -> Twist:
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.linear.z = float(vz)
    msg.angular.x = float(wx)
    msg.angular.y = float(wy)
    msg.angular.z = float(wz)
    return msg


def zero_twist() -> Twist:
    return make_twist()


def twist_to_dict(msg: Any) -> Dict[str, Dict[str, float]]:
    return {
        "linear": {
            "x": float(getattr(msg.linear, "x", 0.0)),
            "y": float(getattr(msg.linear, "y", 0.0)),
            "z": float(getattr(msg.linear, "z", 0.0)),
        },
        "angular": {
            "x": float(getattr(msg.angular, "x", 0.0)),
            "y": float(getattr(msg.angular, "y", 0.0)),
            "z": float(getattr(msg.angular, "z", 0.0)),
        },
    }


def scale_twist(msg: Any, factor: float) -> Twist:
    f = float(factor)
    return make_twist(
        vx=float(getattr(msg.linear, "x", 0.0)) * f,
        vy=float(getattr(msg.linear, "y", 0.0)) * f,
        vz=float(getattr(msg.linear, "z", 0.0)) * f,
        wx=float(getattr(msg.angular, "x", 0.0)) * f,
        wy=float(getattr(msg.angular, "y", 0.0)) * f,
        wz=float(getattr(msg.angular, "z", 0.0)) * f,
    )


def clamp_value(value: float, limit: float) -> float:
    v = float(value)
    lim = abs(float(limit))

    if lim <= 0.0:
        return 0.0

    if v > lim:
        return lim
    if v < -lim:
        return -lim
    return v


def clamp_twist(
    msg: Any,
    *,
    max_vx: float,
    max_vy: float,
    max_wz: float,
) -> Twist:
    return make_twist(
        vx=clamp_value(float(getattr(msg.linear, "x", 0.0)), max_vx),
        vy=clamp_value(float(getattr(msg.linear, "y", 0.0)), max_vy),
        wz=clamp_value(float(getattr(msg.angular, "z", 0.0)), max_wz),
    )


def valid_distance_m(
    value: Any,
    *,
    min_m: float = 0.0,
    max_m: float = 10.0,
) -> Optional[float]:
    try:
        d = float(value)
    except Exception:
        return None

    if not math.isfinite(d):
        return None

    if d < float(min_m) or d > float(max_m):
        return None

    return d


def range_msg(value_m: Any, *, min_m: float = 0.0, max_m: float = 10.0) -> Optional[Float32]:
    d = valid_distance_m(value_m, min_m=min_m, max_m=max_m)
    if d is None:
        return None
    return make_float32(d)


def safety_stop_msg(stop: Any) -> Bool:
    return make_bool(stop)


def slowdown_msg(value: Any, *, min_value: float = 0.0, max_value: float = 1.0) -> Float32:
    try:
        v = float(value)
    except Exception:
        v = 1.0

    if v < min_value:
        v = min_value
    if v > max_value:
        v = max_value

    return make_float32(v)


__all__ = [
    "ROS_MSGS_AVAILABLE",
    "is_ros_msgs_available",
    "Float32",
    "Bool",
    "String",
    "Twist",
    "make_float32",
    "make_bool",
    "make_string",
    "make_json_string",
    "float_from_msg",
    "bool_from_msg",
    "string_from_msg",
    "json_from_string_msg",
    "make_twist",
    "zero_twist",
    "twist_to_dict",
    "scale_twist",
    "clamp_value",
    "clamp_twist",
    "valid_distance_m",
    "range_msg",
    "safety_stop_msg",
    "slowdown_msg",
]