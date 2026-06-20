#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping parameter loading helpers."""

from __future__ import annotations

from pathlib import Path

import pytest

from savo_mapping.ros.params import (
    FrontierExplorerParams,
    MappingModeManagerParams,
    MappingSupervisorParams,
    PointcloudMonitorParams,
    build_default_param_bundle,
    frontier_explorer_params_from_dict,
    mapping_mode_manager_params_from_dict,
    mapping_supervisor_params_from_dict,
    pointcloud_monitor_params_from_dict,
)
from savo_mapping.utils.param_loader import (
    deep_merge_dicts,
    get_nested,
    get_ros_parameters,
    load_profile,
    load_profile_stack,
    load_ros_parameters,
    load_yaml_file,
    load_yaml_files,
    merge_ros_parameter_files,
    require_file,
    require_mapping,
    require_nested,
    resolve_path,
    save_yaml_file,
    set_nested,
)


# =============================================================================
# Path helpers
# =============================================================================
def test_resolve_path_returns_absolute_path(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"

    resolved = resolve_path(path)

    assert resolved.is_absolute()
    assert resolved.name == "config.yaml"


def test_require_file_accepts_existing_file(tmp_path: Path) -> None:
    file_path = tmp_path / "test.yaml"
    file_path.write_text("a: 1\n", encoding="utf-8")

    resolved = require_file(file_path)

    assert resolved == file_path.resolve()


def test_require_file_rejects_missing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        require_file(tmp_path / "missing.yaml")


def test_require_file_rejects_directory(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        require_file(tmp_path)


# =============================================================================
# YAML loading / saving
# =============================================================================
def test_load_yaml_file(tmp_path: Path) -> None:
    file_path = tmp_path / "config.yaml"
    file_path.write_text(
        "mapping:\n"
        "  scan_topic: /scan\n"
        "  rate_hz: 5.0\n",
        encoding="utf-8",
    )

    data = load_yaml_file(file_path)

    assert data["mapping"]["scan_topic"] == "/scan"
    assert data["mapping"]["rate_hz"] == 5.0


def test_load_yaml_file_empty_file_returns_empty_dict(tmp_path: Path) -> None:
    file_path = tmp_path / "empty.yaml"
    file_path.write_text("", encoding="utf-8")

    assert load_yaml_file(file_path) == {}


def test_load_yaml_file_rejects_non_mapping_root(tmp_path: Path) -> None:
    file_path = tmp_path / "bad.yaml"
    file_path.write_text("- one\n- two\n", encoding="utf-8")

    with pytest.raises(ValueError):
        load_yaml_file(file_path)


def test_save_yaml_file_creates_parent_directory(tmp_path: Path) -> None:
    file_path = tmp_path / "nested" / "params.yaml"

    saved = save_yaml_file(
        file_path,
        {
            "mapping_supervisor": {
                "scan_topic": "/scan",
                "publish_rate_hz": 5.0,
            }
        },
    )

    assert saved.exists()

    data = load_yaml_file(saved)

    assert data["mapping_supervisor"]["scan_topic"] == "/scan"
    assert data["mapping_supervisor"]["publish_rate_hz"] == 5.0


def test_load_yaml_files_deep_merges_files(tmp_path: Path) -> None:
    base = tmp_path / "base.yaml"
    override = tmp_path / "override.yaml"

    base.write_text(
        "mapping:\n"
        "  scan_topic: /scan\n"
        "  odom_topic: /odom\n"
        "  publish_rate_hz: 5.0\n",
        encoding="utf-8",
    )
    override.write_text(
        "mapping:\n"
        "  odom_topic: /odometry/filtered\n"
        "  require_map: true\n",
        encoding="utf-8",
    )

    data = load_yaml_files((base, override))

    assert data["mapping"]["scan_topic"] == "/scan"
    assert data["mapping"]["odom_topic"] == "/odometry/filtered"
    assert data["mapping"]["publish_rate_hz"] == 5.0
    assert data["mapping"]["require_map"] is True


# =============================================================================
# Dictionary helpers
# =============================================================================
def test_deep_merge_dicts_preserves_nested_values() -> None:
    base = {
        "mapping": {
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "timeouts": {
                "scan": 0.5,
                "odom": 0.5,
            },
        }
    }

    override = {
        "mapping": {
            "odom_topic": "/odometry/filtered",
            "timeouts": {
                "odom": 1.0,
            },
        }
    }

    merged = deep_merge_dicts(base, override)

    assert merged["mapping"]["scan_topic"] == "/scan"
    assert merged["mapping"]["odom_topic"] == "/odometry/filtered"
    assert merged["mapping"]["timeouts"]["scan"] == 0.5
    assert merged["mapping"]["timeouts"]["odom"] == 1.0


def test_get_nested_returns_value() -> None:
    data = {
        "a": {
            "b": {
                "c": 10,
            }
        }
    }

    assert get_nested(data, "a.b.c") == 10


def test_get_nested_returns_default_for_missing_value() -> None:
    data = {"a": {"b": 1}}

    assert get_nested(data, "a.x", default="missing") == "missing"


def test_require_nested_returns_value() -> None:
    data = {"a": {"b": 1}}

    assert require_nested(data, "a.b") == 1


def test_require_nested_raises_for_missing_value() -> None:
    data = {"a": {"b": 1}}

    with pytest.raises(KeyError):
        require_nested(data, "a.x")


def test_set_nested_adds_new_nested_value() -> None:
    data = {"a": {"b": 1}}

    updated = set_nested(data, "a.c", 2)

    assert updated["a"]["b"] == 1
    assert updated["a"]["c"] == 2


def test_set_nested_creates_nested_path() -> None:
    data = {}

    updated = set_nested(data, "a.b.c", 3)

    assert updated["a"]["b"]["c"] == 3


def test_set_nested_rejects_empty_path() -> None:
    with pytest.raises(ValueError):
        set_nested({}, "", 1)


def test_require_mapping_accepts_mapping() -> None:
    assert require_mapping({"a": 1}, "test") == {"a": 1}


def test_require_mapping_rejects_non_mapping() -> None:
    with pytest.raises(ValueError):
        require_mapping(["not", "mapping"], "test")


# =============================================================================
# ROS-style parameter helpers
# =============================================================================
def test_get_ros_parameters() -> None:
    data = {
        "mapping_supervisor_node": {
            "ros__parameters": {
                "scan_topic": "/scan",
                "publish_rate_hz": 5.0,
            }
        }
    }

    params = get_ros_parameters(data, "mapping_supervisor_node")

    assert params["scan_topic"] == "/scan"
    assert params["publish_rate_hz"] == 5.0


def test_get_ros_parameters_missing_node_returns_empty_params() -> None:
    params = get_ros_parameters({}, "missing_node")

    assert params == {}


def test_get_ros_parameters_rejects_bad_ros_parameters() -> None:
    data = {
        "mapping_supervisor_node": {
            "ros__parameters": ["bad"]
        }
    }

    with pytest.raises(ValueError):
        get_ros_parameters(data, "mapping_supervisor_node")


def test_load_ros_parameters(tmp_path: Path) -> None:
    file_path = tmp_path / "params.yaml"
    file_path.write_text(
        "mapping_supervisor_node:\n"
        "  ros__parameters:\n"
        "    scan_topic: /scan\n"
        "    odom_topic: /odometry/filtered\n",
        encoding="utf-8",
    )

    params = load_ros_parameters(file_path, "mapping_supervisor_node")

    assert params["scan_topic"] == "/scan"
    assert params["odom_topic"] == "/odometry/filtered"


def test_merge_ros_parameter_files(tmp_path: Path) -> None:
    base = tmp_path / "base.yaml"
    override = tmp_path / "override.yaml"

    base.write_text(
        "mapping_supervisor_node:\n"
        "  ros__parameters:\n"
        "    scan_topic: /scan\n"
        "    odom_topic: /odom\n"
        "    publish_rate_hz: 5.0\n",
        encoding="utf-8",
    )
    override.write_text(
        "mapping_supervisor_node:\n"
        "  ros__parameters:\n"
        "    odom_topic: /odometry/filtered\n"
        "    require_map: true\n",
        encoding="utf-8",
    )

    params = merge_ros_parameter_files(
        (base, override),
        "mapping_supervisor_node",
    )

    assert params["scan_topic"] == "/scan"
    assert params["odom_topic"] == "/odometry/filtered"
    assert params["publish_rate_hz"] == 5.0
    assert params["require_map"] is True


# =============================================================================
# Profile helpers
# =============================================================================
def test_load_profile_with_overrides(tmp_path: Path) -> None:
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        "mapping_supervisor:\n"
        "  scan_topic: /scan\n"
        "  require_map: false\n",
        encoding="utf-8",
    )

    data = load_profile(
        profile,
        overrides={
            "mapping_supervisor": {
                "require_map": True,
            }
        },
    )

    assert data["mapping_supervisor"]["scan_topic"] == "/scan"
    assert data["mapping_supervisor"]["require_map"] is True


def test_load_profile_stack_with_overrides(tmp_path: Path) -> None:
    base = tmp_path / "base.yaml"
    robot = tmp_path / "robot.yaml"

    base.write_text(
        "mapping_supervisor:\n"
        "  scan_topic: /scan\n"
        "  publish_rate_hz: 5.0\n",
        encoding="utf-8",
    )
    robot.write_text(
        "mapping_supervisor:\n"
        "  odom_topic: /odometry/filtered\n",
        encoding="utf-8",
    )

    data = load_profile_stack(
        (base, robot),
        overrides={
            "mapping_supervisor": {
                "publish_rate_hz": 10.0,
            }
        },
    )

    assert data["mapping_supervisor"]["scan_topic"] == "/scan"
    assert data["mapping_supervisor"]["odom_topic"] == "/odometry/filtered"
    assert data["mapping_supervisor"]["publish_rate_hz"] == 10.0


# =============================================================================
# Typed ROS params
# =============================================================================
def test_default_param_bundle_contains_expected_sections() -> None:
    bundle = build_default_param_bundle()

    assert "mapping_supervisor" in bundle
    assert "mapping_mode_manager" in bundle
    assert "frontier_explorer" in bundle
    assert "pointcloud_monitor" in bundle


def test_mapping_supervisor_params_defaults() -> None:
    params = MappingSupervisorParams()

    assert params.scan_topic == "/scan"
    assert params.odom_topic == "/odometry/filtered"
    assert params.map_topic == "/map"
    assert params.require_scan is True
    assert params.require_odom is True
    assert params.require_tf is True
    assert params.require_map is False
    assert params.require_pointcloud is False


def test_mapping_supervisor_params_from_dict() -> None:
    params = mapping_supervisor_params_from_dict(
        {
            "require_map": True,
            "require_pointcloud": True,
            "publish_rate_hz": 10.0,
            "scan_stale_timeout_s": 1.5,
        }
    )

    assert params.require_map is True
    assert params.require_pointcloud is True
    assert params.publish_rate_hz == 10.0
    assert params.scan_stale_timeout_s == 1.5


def test_mapping_mode_manager_params_defaults() -> None:
    params = MappingModeManagerParams()

    assert params.default_mode == "idle"
    assert params.allow_manual_mapping is True
    assert params.allow_autonomous_mapping is False
    assert params.allow_map_save is True


def test_mapping_mode_manager_params_from_dict() -> None:
    params = mapping_mode_manager_params_from_dict(
        {
            "default_mode": "manual_mapping",
            "allow_autonomous_mapping": True,
            "publish_rate_hz": 2.0,
        }
    )

    assert params.default_mode == "manual_mapping"
    assert params.allow_autonomous_mapping is True
    assert params.publish_rate_hz == 2.0


def test_frontier_explorer_params_defaults() -> None:
    params = FrontierExplorerParams()

    assert params.enabled is False
    assert params.map_topic == "/map"
    assert params.goal_clearance_m == 0.35
    assert params.max_failed_goals == 20


def test_frontier_explorer_params_from_dict() -> None:
    params = frontier_explorer_params_from_dict(
        {
            "enabled": True,
            "min_frontier_size_cells": 10,
            "goal_clearance_m": 0.45,
            "max_failed_goals": 5,
        }
    )

    assert params.enabled is True
    assert params.min_frontier_size_cells == 10
    assert params.goal_clearance_m == 0.45
    assert params.max_failed_goals == 5


def test_pointcloud_monitor_params_defaults() -> None:
    params = PointcloudMonitorParams()

    assert params.enabled is False
    assert params.pointcloud_topic == "/savo_edge/realsense/points"
    assert params.expected_frame == "camera_depth_optical_frame"


def test_pointcloud_monitor_params_from_dict() -> None:
    params = pointcloud_monitor_params_from_dict(
        {
            "enabled": True,
            "min_rate_hz": 5.0,
            "stale_timeout_s": 2.0,
        }
    )

    assert params.enabled is True
    assert params.min_rate_hz == 5.0
    assert params.stale_timeout_s == 2.0