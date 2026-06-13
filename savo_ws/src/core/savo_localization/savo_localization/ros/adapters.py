#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Small ROS message helpers for localization diagnostics."""

from __future__ import annotations

import json
from collections.abc import Mapping
from typing import Any

from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from savo_localization.constants import (
    FRAME_BASE_LINK,
    FRAME_IMU,
    FRAME_ODOM,
    STATUS_UNKNOWN,
)


def make_string_msg(data: str) -> String:
    msg = String()
    msg.data = str(data)
    return msg


def make_json_msg(
    payload: Mapping[str, Any],
    *,
    compact: bool = True,
    sort_keys: bool = True,
) -> String:
    separators = (",", ":") if compact else None

    msg = String()
    msg.data = json.dumps(
        dict(payload),
        separators=separators,
        sort_keys=sort_keys,
    )
    return msg


def parse_json_msg(msg: String | str) -> dict[str, Any]:
    data = msg.data if isinstance(msg, String) else str(msg)

    try:
        parsed = json.loads(data)
    except json.JSONDecodeError:
        return {}

    if not isinstance(parsed, dict):
        return {}

    return parsed


def make_status_payload(
    *,
    node: str,
    status: str = STATUS_UNKNOWN,
    message: str = "",
    **items: Any,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "node": str(node),
        "status": str(status),
        "message": str(message),
    }

    for key, value in items.items():
        if value is not None:
            payload[str(key)] = value

    return payload


def vector3_to_dict(vector: Vector3) -> dict[str, float]:
    return {
        "x": float(vector.x),
        "y": float(vector.y),
        "z": float(vector.z),
    }


def quaternion_to_dict(quaternion: Quaternion) -> dict[str, float]:
    return {
        "x": float(quaternion.x),
        "y": float(quaternion.y),
        "z": float(quaternion.z),
        "w": float(quaternion.w),
    }


def imu_to_dict(msg: Imu) -> dict[str, Any]:
    return {
        "frame_id": msg.header.frame_id,
        "orientation": quaternion_to_dict(msg.orientation),
        "angular_velocity": vector3_to_dict(msg.angular_velocity),
        "linear_acceleration": vector3_to_dict(msg.linear_acceleration),
        "orientation_covariance": list(msg.orientation_covariance),
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance),
    }


def odom_to_dict(msg: Odometry) -> dict[str, Any]:
    return {
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "position": {
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "z": float(msg.pose.pose.position.z),
        },
        "orientation": quaternion_to_dict(msg.pose.pose.orientation),
        "linear_velocity": vector3_to_dict(msg.twist.twist.linear),
        "angular_velocity": vector3_to_dict(msg.twist.twist.angular),
    }


def transform_to_dict(msg: TransformStamped) -> dict[str, Any]:
    return {
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "translation": vector3_to_dict(msg.transform.translation),
        "rotation": quaternion_to_dict(msg.transform.rotation),
    }


def validate_imu_frame(msg: Imu, *, expected_frame_id: str = FRAME_IMU) -> bool:
    return str(msg.header.frame_id).strip() == str(expected_frame_id).strip()


def validate_odom_frames(
    msg: Odometry,
    *,
    expected_frame_id: str = FRAME_ODOM,
    expected_child_frame_id: str = FRAME_BASE_LINK,
) -> bool:
    return (
        str(msg.header.frame_id).strip() == str(expected_frame_id).strip()
        and str(msg.child_frame_id).strip() == str(expected_child_frame_id).strip()
    )


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def safe_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        text = value.strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off"):
            return False

    if value is None:
        return bool(default)

    return bool(value)