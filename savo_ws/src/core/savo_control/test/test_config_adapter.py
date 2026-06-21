#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for config adapter helpers."""

from __future__ import annotations

from pathlib import Path
import tempfile

from savo_control.adapters import (
    command_limits_from_params,
    control_mode_from_params,
    deadbands_from_params,
    distance_approach_config_from_params,
    distance_approach_config_from_yaml,
    distance_topics_from_params,
    load_merged_node_params,
    load_node_params,
    runtime_flags_from_params,
    topic_from_params,
)
from savo_control.models import ControlMode, DistanceApproachConfig


def test_load_node_params():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "params.yaml"
        path.write_text(
            "test_node:\n"
            "  ros__parameters:\n"
            "    value: 42\n"
            "    enabled: true\n",
            encoding="utf-8",
        )

        params = load_node_params(path, "test_node")

    assert params == {
        "value": 42,
        "enabled": True,
    }


def test_load_merged_node_params_merges_common_and_node_values():
    with tempfile.TemporaryDirectory() as tmp:
        common = Path(tmp) / "common.yaml"
        node = Path(tmp) / "node.yaml"

        common.write_text(
            "/**:\n"
            "  ros__parameters:\n"
            "    publish_rate_hz: 30.0\n"
            "    limits:\n"
            "      vx:\n"
            "        max_abs: 0.20\n"
            "        deadband: 0.005\n"
            "      vy:\n"
            "        max_abs: 0.20\n",
            encoding="utf-8",
        )
        node.write_text(
            "cmd_vel_shaper_node:\n"
            "  ros__parameters:\n"
            "    limits:\n"
            "      vx:\n"
            "        max_abs: 0.10\n"
            "    input_timeout_sec: 0.30\n",
            encoding="utf-8",
        )

        params = load_merged_node_params([common, node], "cmd_vel_shaper_node")

    assert params["publish_rate_hz"] == 30.0
    assert params["input_timeout_sec"] == 0.30
    assert params["limits"]["vx"]["max_abs"] == 0.10
    assert params["limits"]["vx"]["deadband"] == 0.005
    assert params["limits"]["vy"]["max_abs"] == 0.20


def test_control_mode_from_params():
    assert control_mode_from_params({"startup_mode": "AUTO"}) == ControlMode.AUTO
    assert control_mode_from_params({"startup_mode": "manual"}) == ControlMode.MANUAL
    assert control_mode_from_params({"startup_mode": "bad"}) == ControlMode.STOP
    assert control_mode_from_params({}, default=ControlMode.NAV) == ControlMode.NAV


def test_control_mode_from_params_custom_key():
    assert (
        control_mode_from_params(
            {"default_mode": "RECOVERY"},
            key="default_mode",
        )
        == ControlMode.RECOVERY
    )


def test_topic_from_params():
    params = {
        "cmd_topic": "cmd_vel_auto",
        "safe_topic": "/cmd_vel_safe",
    }

    assert topic_from_params(params, "cmd_topic") == "/cmd_vel_auto"
    assert topic_from_params(params, "safe_topic") == "/cmd_vel_safe"
    assert topic_from_params(params, "missing", default="/fallback") == "/fallback"


def test_command_limits_from_params():
    params = {
        "limits": {
            "vx": {"max_abs": 0.25},
            "vy": {"max_abs": 0.20},
            "wz": {"max_abs": 0.60},
        }
    }

    assert command_limits_from_params(params) == {
        "max_vx": 0.25,
        "max_vy": 0.20,
        "max_wz": 0.60,
    }


def test_command_limits_from_params_uses_defaults():
    assert command_limits_from_params({}) == {
        "max_vx": 0.20,
        "max_vy": 0.20,
        "max_wz": 0.60,
    }


def test_deadbands_from_params():
    params = {
        "limits": {
            "vx": {"deadband": 0.005},
            "vy": {"deadband": 0.006},
            "wz": {"deadband": 0.010},
        }
    }

    assert deadbands_from_params(params) == {
        "vx_deadband": 0.005,
        "vy_deadband": 0.006,
        "wz_deadband": 0.010,
    }


def test_deadbands_from_params_uses_defaults():
    assert deadbands_from_params({}) == {
        "vx_deadband": 0.0,
        "vy_deadband": 0.0,
        "wz_deadband": 0.0,
    }


def test_distance_approach_config_from_params_flat_pid_values():
    params = {
        "target_distance_m": 0.70,
        "tolerance_m": 0.05,
        "hard_min_distance_m": 0.30,
        "min_valid_distance_m": 0.04,
        "max_valid_distance_m": 2.50,
        "distance_timeout_s": 0.35,
        "kp": 0.50,
        "ki": 0.01,
        "kd": 0.02,
        "max_forward_vx": 0.12,
        "allow_reverse": True,
        "max_reverse_vx": 0.04,
        "min_vx_when_active": 0.03,
        "disable_min_vx_below_error_m": 0.07,
    }

    cfg = distance_approach_config_from_params(params)

    assert isinstance(cfg, DistanceApproachConfig)
    assert cfg.target_distance_m == 0.70
    assert cfg.tolerance_m == 0.05
    assert cfg.hard_min_distance_m == 0.30
    assert cfg.min_valid_distance_m == 0.04
    assert cfg.max_valid_distance_m == 2.50
    assert cfg.distance_timeout_s == 0.35
    assert cfg.kp == 0.50
    assert cfg.ki == 0.01
    assert cfg.kd == 0.02
    assert cfg.max_forward_vx == 0.12
    assert cfg.allow_reverse is True
    assert cfg.max_reverse_vx == 0.04
    assert cfg.min_vx_when_active == 0.03
    assert cfg.disable_min_vx_below_error_m == 0.07


def test_distance_approach_config_from_params_nested_pid_values():
    params = {
        "pid": {
            "kp": 0.55,
            "ki": 0.02,
            "kd": 0.04,
        }
    }

    cfg = distance_approach_config_from_params(params)

    assert cfg.kp == 0.55
    assert cfg.ki == 0.02
    assert cfg.kd == 0.04


def test_distance_approach_config_from_params_uses_safe_defaults():
    cfg = distance_approach_config_from_params({})

    assert cfg.target_distance_m == 0.60
    assert cfg.tolerance_m == 0.04
    assert cfg.hard_min_distance_m == 0.35
    assert cfg.min_valid_distance_m == 0.05
    assert cfg.max_valid_distance_m == 3.00
    assert cfg.distance_timeout_s == 0.40
    assert cfg.kp == 0.45
    assert cfg.ki == 0.0
    assert cfg.kd == 0.03
    assert cfg.max_forward_vx == 0.10
    assert cfg.allow_reverse is False
    assert cfg.max_reverse_vx == 0.05


def test_distance_approach_config_from_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "distance.yaml"
        path.write_text(
            "distance_approach_node:\n"
            "  ros__parameters:\n"
            "    target_distance_m: 0.80\n"
            "    tolerance_m: 0.03\n"
            "    kp: 0.40\n",
            encoding="utf-8",
        )

        cfg = distance_approach_config_from_yaml(path)

    assert cfg.target_distance_m == 0.80
    assert cfg.tolerance_m == 0.03
    assert cfg.kp == 0.40


def test_distance_topics_from_params():
    params = {
        "distance_topic": "depth/min_front_m",
        "cmd_vel_out_topic": "cmd_vel_auto",
        "mode_cmd_topic": "savo_control/mode_cmd",
        "safety_stop_topic": "safety/stop",
        "enable_topic": "savo_control/distance_approach_enable",
        "target_topic": "savo_control/distance_approach_target",
    }

    assert distance_topics_from_params(params) == {
        "distance_topic": "/depth/min_front_m",
        "cmd_vel_out_topic": "/cmd_vel_auto",
        "mode_cmd_topic": "/savo_control/mode_cmd",
        "safety_stop_topic": "/safety/stop",
        "enable_topic": "/savo_control/distance_approach_enable",
        "target_topic": "/savo_control/distance_approach_target",
    }


def test_distance_topics_from_params_uses_defaults():
    assert distance_topics_from_params({}) == {
        "distance_topic": "/depth/min_front_m",
        "cmd_vel_out_topic": "/cmd_vel_auto",
        "mode_cmd_topic": "/savo_control/mode_cmd",
        "safety_stop_topic": "/safety/stop",
        "enable_topic": "/savo_control/distance_approach_enable",
        "target_topic": "/savo_control/distance_approach_target",
    }


def test_runtime_flags_from_params():
    params = {
        "auto_start": "true",
        "respect_safety_stop": "false",
        "publish_status": "true",
    }

    assert runtime_flags_from_params(params) == {
        "auto_start": True,
        "respect_safety_stop": False,
        "publish_status": True,
    }


def test_runtime_flags_from_params_uses_defaults():
    assert runtime_flags_from_params({}) == {
        "auto_start": False,
        "respect_safety_stop": True,
        "publish_status": True,
    }
