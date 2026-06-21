# -*- coding: utf-8 -*-

"""Adapters for TwistCommand and ROS-style Twist messages."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from savo_control.models import TwistCommand


def twist_from_mapping(
    data: Mapping[str, Any],
    *,
    source: str = "",
    stamp_sec: float = 0.0,
) -> TwistCommand:
    return TwistCommand(
        vx=float(data.get("vx", data.get("linear_x", 0.0))),
        vy=float(data.get("vy", data.get("linear_y", 0.0))),
        wz=float(data.get("wz", data.get("angular_z", 0.0))),
        source=source or str(data.get("source", "")),
        stamp_sec=float(data.get("stamp_sec", stamp_sec)),
    ).sanitized()


def twist_from_sequence(
    values: Sequence[float],
    *,
    source: str = "",
    stamp_sec: float = 0.0,
) -> TwistCommand:
    if len(values) != 3:
        raise ValueError("Twist sequence must contain exactly 3 values: vx, vy, wz")

    return TwistCommand(
        vx=float(values[0]),
        vy=float(values[1]),
        wz=float(values[2]),
        source=source,
        stamp_sec=stamp_sec,
    ).sanitized()


def twist_from_ros_msg(
    msg: Any,
    *,
    source: str = "",
    stamp_sec: float = 0.0,
) -> TwistCommand:
    return TwistCommand(
        vx=float(msg.linear.x),
        vy=float(msg.linear.y),
        wz=float(msg.angular.z),
        source=source,
        stamp_sec=stamp_sec,
    ).sanitized()


def twist_to_dict(cmd: TwistCommand) -> dict:
    return cmd.sanitized().to_dict()


def copy_to_ros_msg(cmd: TwistCommand, msg: Any) -> Any:
    safe = cmd.sanitized()

    msg.linear.x = safe.vx
    msg.linear.y = safe.vy
    msg.linear.z = 0.0

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = safe.wz

    return msg


def twist_to_ros_msg(cmd: TwistCommand, msg_type: Any | None = None) -> Any:
    if msg_type is None:
        try:
            from geometry_msgs.msg import Twist as msg_type
        except Exception as exc:
            raise RuntimeError(
                "geometry_msgs is required to create a ROS Twist message"
            ) from exc

    return copy_to_ros_msg(cmd, msg_type())


def zero_ros_twist(msg_type: Any | None = None) -> Any:
    return twist_to_ros_msg(TwistCommand.zero(), msg_type=msg_type)


__all__ = [
    "copy_to_ros_msg",
    "twist_from_mapping",
    "twist_from_ros_msg",
    "twist_from_sequence",
    "twist_to_dict",
    "twist_to_ros_msg",
    "zero_ros_twist",
]
