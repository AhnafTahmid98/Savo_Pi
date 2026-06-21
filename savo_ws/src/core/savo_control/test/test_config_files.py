#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for config file syntax and runtime parameter contracts."""

from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"


def load_yaml(name: str) -> dict:
    path = CONFIG_DIR / name
    assert path.is_file(), f"Missing config file: {name}"

    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(data, dict), f"{name} must contain a YAML dictionary"
    return data


def params(data: dict, node_name: str) -> dict:
    assert node_name in data, f"Missing node section: {node_name}"
    node = data[node_name]

    assert isinstance(node, dict)
    assert "ros__parameters" in node
    assert isinstance(node["ros__parameters"], dict)

    return node["ros__parameters"]


def test_all_yaml_files_parse():
    yaml_files = sorted(CONFIG_DIR.glob("*.yaml"))

    assert yaml_files, "No YAML config files found."

    for path in yaml_files:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert data is not None, f"{path.name} is empty"


def test_control_common_contract():
    data = load_yaml("control_common.yaml")
    common = params(data, "/**")

    required = [
        "publish_rate_hz",
        "input_timeout_sec",
        "cmd_timeout_sec",
        "default_mode",
        "startup_mode",
        "use_safety_stop_gate",
        "limits",
        "limiter",
        "timing",
    ]

    for key in required:
        assert key in common

    assert common["default_mode"] == "STOP"
    assert common["startup_mode"] == "STOP"


def test_cmd_vel_shaper_contract():
    data = load_yaml("cmd_vel_shaper.yaml")
    cfg = params(data, "cmd_vel_shaper_node")

    required = [
        "publish_rate_hz",
        "input_timeout_sec",
        "zero_on_stale_input",
        "use_safety_stop_gate",
        "safety_stop_hold_sec",
        "limits",
        "limiter",
        "log",
    ]

    for key in required:
        assert key in cfg

    for axis in ["vx", "vy", "wz"]:
        assert axis in cfg["limits"]
        assert cfg["limits"][axis]["max_abs"] > 0.0
        assert cfg["limits"][axis]["deadband"] >= 0.0
        assert cfg["limits"][axis]["max_rise_rate"] > 0.0
        assert cfg["limits"][axis]["max_fall_rate"] > 0.0


def test_twist_mux_contract():
    data = load_yaml("twist_mux.yaml")
    cfg = params(data, "twist_mux_node")

    required = [
        "publish_rate_hz",
        "cmd_timeout_sec",
        "recovery_cmd_timeout_sec",
        "default_mode",
        "zero_on_unknown_mode",
        "recovery_override_enabled",
        "latch_recovery_active",
        "recovery_active_timeout_sec",
        "use_safety_stop_gate",
    ]

    for key in required:
        assert key in cfg

    assert cfg["default_mode"] in ["STOP", "MANUAL", "AUTO", "NAV", "RECOVERY"]


def test_control_mode_manager_contract():
    data = load_yaml("control_mode_manager.yaml")
    cfg = params(data, "control_mode_manager_node")

    required = [
        "update_rate_hz",
        "startup_mode",
        "manual_override_preempts_all",
        "recovery_preempts_auto_nav",
        "recovery_preempts_manual",
        "latch_recovery_mode",
        "safety_stop_forces_stop_mode",
        "external_stop_forces_stop_mode",
        "fallback_to_stop_when_source_unavailable",
        "manual_source_available_default",
        "auto_source_available_default",
        "nav_source_available_default",
        "recovery_source_available_default",
    ]

    for key in required:
        assert key in cfg

    assert cfg["startup_mode"] == "STOP"


def test_heading_pid_contract():
    data = load_yaml("heading_pid.yaml")
    cfg = params(data, "heading_pid_node")

    required = [
        "odom_topic",
        "base_cmd_topic",
        "output_topic",
        "heading_target_topic",
        "heading_hold_enable_topic",
        "publish_rate_hz",
        "odom_timeout_sec",
        "base_cmd_timeout_sec",
        "output",
        "pid",
        "heading_tolerance_rad",
        "max_wz_rad_s",
        "ctrl",
    ]

    for key in required:
        assert key in cfg

    assert cfg["output_topic"] == "/cmd_vel_auto"
    assert cfg["pid"]["output_min"] < 0.0
    assert cfg["pid"]["output_max"] > 0.0


def test_rotate_to_heading_contract():
    data = load_yaml("rotate_to_heading.yaml")
    cfg = params(data, "rotate_to_heading_node")

    required = [
        "odom_topic",
        "rotate_target_topic",
        "enable_topic",
        "output_topic",
        "publish_rate_hz",
        "odom_timeout_sec",
        "settle_cycles_required",
        "max_rotate_time_sec",
        "output",
        "pid",
        "heading_tolerance_rad",
        "max_wz_rad_s",
        "ctrl",
    ]

    for key in required:
        assert key in cfg

    assert cfg["output_topic"] == "/cmd_vel_auto"
    assert cfg["settle_cycles_required"] >= 1


def test_recovery_contract():
    data = load_yaml("recovery.yaml")

    manager = params(data, "recovery_manager_node")
    backup = params(data, "backup_escape_node")
    status = params(data, "recovery_status_node")

    assert manager["topics"]["cmd_vel_recovery"] == "/cmd_vel_recovery"
    assert manager["command"]["backup_speed_m_s"] > 0.0
    assert manager["policy"]["max_attempts_per_streak"] >= 1

    assert backup["topics"]["cmd_out"] == "/cmd_vel_recovery"
    assert backup["command"]["backup_speed_m_s"] > 0.0
    assert backup["command"]["turn_sign"] in [-1, 1]

    assert status["cmd_vel_recovery_topic"] == "/cmd_vel_recovery"
    assert status["publish_hz"] > 0.0


def test_stuck_detector_contract():
    data = load_yaml("stuck_detector.yaml")
    cfg = params(data, "stuck_detector_node")

    required = [
        "enabled",
        "active_detection",
        "topics",
        "timing",
        "command_thresholds",
        "observed_motion_thresholds",
        "detection",
        "recovery",
        "output",
        "safety",
    ]

    for key in required:
        assert key in cfg

    assert cfg["topics"]["cmd_vel_safe"] == "/cmd_vel_safe"
    assert cfg["topics"]["odom"] == "/odometry/filtered"
    assert cfg["detection"]["ignore_when_safety_stop_true"] is True
    assert cfg["recovery"]["publish_recovery_request"] is False


def test_distance_approach_hybrid_contract():
    data = load_yaml("distance_approach.yaml")

    cpp = params(data, "distance_approach_node")
    py = params(data, "distance_pid_test_node")

    shared_keys = [
        "distance_topic",
        "cmd_vel_out_topic",
        "mode_cmd_topic",
        "safety_stop_topic",
        "target_distance_m",
        "tolerance_m",
        "hard_min_distance_m",
        "max_forward_vx",
        "loop_hz",
        "respect_safety_stop",
    ]

    for key in shared_keys:
        assert key in cpp
        assert key in py
        assert cpp[key] == py[key]

    assert cpp["cmd_vel_out_topic"] == "/cmd_vel_auto"
    assert py["cmd_vel_out_topic"] == "/cmd_vel_auto"
    assert cpp["enable_topic"] == "/savo_control/distance_approach_enable"
    assert cpp["target_topic"] == "/savo_control/distance_approach_target"


def test_auto_test_modes_contract():
    data = load_yaml("auto_test_modes.yaml")
    cfg = params(data, "auto_test_manager_node")

    assert cfg["output_topic"] == "/cmd_vel_auto"
    assert cfg["required_mode"] == "AUTO"
    assert cfg["auto_start"] is False
    assert "tests" in cfg
    assert cfg["default_test_name"] in cfg["tests"]


def test_topic_remaps_reference_contract():
    data = load_yaml("topic_remaps_example.yaml")

    assert data["topic_contract"]["command_chain"]["shaped_output"] == "/cmd_vel"
    assert data["topic_contract"]["command_chain"]["safety_output"] == "/cmd_vel_safe"
    assert data["savo_base"]["base_driver_input"] == "/cmd_vel_safe"

    forbidden = data["savo_base"]["forbidden_direct_inputs"]
    assert "/cmd_vel" in forbidden
    assert "/cmd_vel_mux" in forbidden
    assert "/cmd_vel_auto" in forbidden


def test_production_configs_do_not_output_directly_to_base():
    files_to_check = [
        "cmd_vel_shaper.yaml",
        "twist_mux.yaml",
        "heading_pid.yaml",
        "rotate_to_heading.yaml",
        "distance_approach.yaml",
        "recovery.yaml",
    ]

    output_keys = {
        "output_topic",
        "cmd_vel_out_topic",
        "cmd_out",
        "cmd_vel_recovery",
    }

    def walk(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key in output_keys:
                    assert child != "/cmd_vel_safe", f"{key} outputs directly to /cmd_vel_safe"
                walk(child)
        elif isinstance(value, list):
            for item in value:
                walk(item)

    for name in files_to_check:
        walk(load_yaml(name))
