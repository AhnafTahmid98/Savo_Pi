# -*- coding: utf-8 -*-

"""Small ROS-message helpers for Python fallback nodes and tests."""

from __future__ import annotations

from typing import Any


def bool_msg_value(msg: Any, *, default: bool = False) -> bool:
    if msg is None:
        return default

    if hasattr(msg, "data"):
        return bool(msg.data)

    return bool(msg)


def float_msg_value(msg: Any, *, default: float = 0.0) -> float:
    if msg is None:
        return default

    value = msg.data if hasattr(msg, "data") else msg

    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def string_msg_value(msg: Any, *, default: str = "") -> str:
    if msg is None:
        return default

    value = msg.data if hasattr(msg, "data") else msg

    if value is None:
        return default

    return str(value)


def make_bool_msg(value: bool, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from std_msgs.msg import Bool as msg_type
        except Exception as exc:
            raise RuntimeError("std_msgs is required to create Bool messages") from exc

    msg = msg_type()
    msg.data = bool(value)
    return msg


def make_float32_msg(value: float, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from std_msgs.msg import Float32 as msg_type
        except Exception as exc:
            raise RuntimeError("std_msgs is required to create Float32 messages") from exc

    msg = msg_type()
    msg.data = float(value)
    return msg


def make_float64_msg(value: float, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from std_msgs.msg import Float64 as msg_type
        except Exception as exc:
            raise RuntimeError("std_msgs is required to create Float64 messages") from exc

    msg = msg_type()
    msg.data = float(value)
    return msg


def make_string_msg(value: object, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from std_msgs.msg import String as msg_type
        except Exception as exc:
            raise RuntimeError("std_msgs is required to create String messages") from exc

    msg = msg_type()
    msg.data = str(value)
    return msg


def stamp_to_sec(stamp: Any, *, default: float = 0.0) -> float:
    if stamp is None:
        return default

    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)

    if sec is None or nanosec is None:
        return default

    return float(sec) + float(nanosec) * 1.0e-9


def header_stamp_to_sec(msg: Any, *, default: float = 0.0) -> float:
    header = getattr(msg, "header", None)
    if header is None:
        return default

    return stamp_to_sec(getattr(header, "stamp", None), default=default)


def has_header(msg: Any) -> bool:
    return hasattr(msg, "header") and getattr(msg, "header") is not None


def safe_frame_id(msg: Any, *, default: str = "") -> str:
    header = getattr(msg, "header", None)
    if header is None:
        return default

    frame_id = getattr(header, "frame_id", default)
    return str(frame_id or default)


def twist_tuple(msg: Any) -> tuple[float, float, float]:
    return (
        float(msg.linear.x),
        float(msg.linear.y),
        float(msg.angular.z),
    )


def odom_twist_tuple(msg: Any) -> tuple[float, float, float]:
    twist = msg.twist.twist
    return twist_tuple(twist)


__all__ = [
    "bool_msg_value",
    "float_msg_value",
    "has_header",
    "header_stamp_to_sec",
    "make_bool_msg",
    "make_float32_msg",
    "make_float64_msg",
    "make_string_msg",
    "odom_twist_tuple",
    "safe_frame_id",
    "stamp_to_sec",
    "string_msg_value",
    "twist_tuple",
]
