# -*- coding: utf-8 -*-

"""Adapters from YAML/parameter dictionaries to pure Python config models."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from savo_control.models import ControlMode, DistanceApproachConfig
from savo_control.ros import (
    as_bool,
    as_float,
    deep_get,
    get_ros_parameters,
    load_yaml_file,
    merge_dicts,
    normalized_topic,
)


def load_node_params(path: str | Path, node_name: str) -> dict[str, Any]:
    data = load_yaml_file(path)
    return get_ros_parameters(data, node_name)


def load_merged_node_params(
    paths: list[str | Path],
    node_name: str,
    *,
    common_node_name: str = "/**",
) -> dict[str, Any]:
    merged: dict[str, Any] = {}

    for path in paths:
        data = load_yaml_file(path)

        if common_node_name in data:
            merged = merge_dicts(merged, get_ros_parameters(data, common_node_name))

        if node_name in data:
            merged = merge_dicts(merged, get_ros_parameters(data, node_name))

    return merged


def control_mode_from_params(
    params: Mapping[str, Any],
    *,
    key: str = "startup_mode",
    default: ControlMode = ControlMode.STOP,
) -> ControlMode:
    return ControlMode.from_text(params.get(key), default=default)


def topic_from_params(
    params: Mapping[str, Any],
    key: str,
    *,
    default: str = "",
) -> str:
    return normalized_topic(str(params.get(key, default)), default=default)


def command_limits_from_params(params: Mapping[str, Any]) -> dict[str, float]:
    return {
        "max_vx": as_float(deep_get(params, "limits.vx.max_abs"), default=0.20),
        "max_vy": as_float(deep_get(params, "limits.vy.max_abs"), default=0.20),
        "max_wz": as_float(deep_get(params, "limits.wz.max_abs"), default=0.60),
    }


def deadbands_from_params(params: Mapping[str, Any]) -> dict[str, float]:
    return {
        "vx_deadband": as_float(deep_get(params, "limits.vx.deadband"), default=0.0),
        "vy_deadband": as_float(deep_get(params, "limits.vy.deadband"), default=0.0),
        "wz_deadband": as_float(deep_get(params, "limits.wz.deadband"), default=0.0),
    }


def distance_approach_config_from_params(
    params: Mapping[str, Any],
) -> DistanceApproachConfig:
    return DistanceApproachConfig(
        target_distance_m=as_float(params.get("target_distance_m"), default=0.60),
        tolerance_m=as_float(params.get("tolerance_m"), default=0.04),
        hard_min_distance_m=as_float(params.get("hard_min_distance_m"), default=0.35),
        min_valid_distance_m=as_float(params.get("min_valid_distance_m"), default=0.05),
        max_valid_distance_m=as_float(params.get("max_valid_distance_m"), default=3.00),
        distance_timeout_s=as_float(params.get("distance_timeout_s"), default=0.40),
        kp=as_float(params.get("kp", deep_get(params, "pid.kp")), default=0.45),
        ki=as_float(params.get("ki", deep_get(params, "pid.ki")), default=0.0),
        kd=as_float(params.get("kd", deep_get(params, "pid.kd")), default=0.03),
        max_forward_vx=as_float(params.get("max_forward_vx"), default=0.10),
        allow_reverse=as_bool(params.get("allow_reverse"), default=False),
        max_reverse_vx=as_float(params.get("max_reverse_vx"), default=0.05),
        min_vx_when_active=as_float(params.get("min_vx_when_active"), default=0.04),
        disable_min_vx_below_error_m=as_float(
            params.get("disable_min_vx_below_error_m"),
            default=0.08,
        ),
    ).sanitized()


def distance_approach_config_from_yaml(
    path: str | Path,
    *,
    node_name: str = "distance_approach_node",
) -> DistanceApproachConfig:
    return distance_approach_config_from_params(load_node_params(path, node_name))


def distance_topics_from_params(params: Mapping[str, Any]) -> dict[str, str]:
    return {
        "distance_topic": topic_from_params(
            params,
            "distance_topic",
            default="/depth/min_front_m",
        ),
        "cmd_vel_out_topic": topic_from_params(
            params,
            "cmd_vel_out_topic",
            default="/cmd_vel_auto",
        ),
        "mode_cmd_topic": topic_from_params(
            params,
            "mode_cmd_topic",
            default="/savo_control/mode_cmd",
        ),
        "safety_stop_topic": topic_from_params(
            params,
            "safety_stop_topic",
            default="/safety/stop",
        ),
        "enable_topic": topic_from_params(
            params,
            "enable_topic",
            default="/savo_control/distance_approach_enable",
        ),
        "target_topic": topic_from_params(
            params,
            "target_topic",
            default="/savo_control/distance_approach_target",
        ),
    }


def runtime_flags_from_params(params: Mapping[str, Any]) -> dict[str, bool]:
    return {
        "auto_start": as_bool(params.get("auto_start"), default=False),
        "respect_safety_stop": as_bool(
            params.get("respect_safety_stop"),
            default=True,
        ),
        "publish_status": as_bool(params.get("publish_status"), default=True),
    }


__all__ = [
    "command_limits_from_params",
    "control_mode_from_params",
    "deadbands_from_params",
    "distance_approach_config_from_params",
    "distance_approach_config_from_yaml",
    "distance_topics_from_params",
    "load_merged_node_params",
    "load_node_params",
    "runtime_flags_from_params",
    "topic_from_params",
]
