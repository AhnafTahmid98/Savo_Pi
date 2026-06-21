#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for ROS parameter helper utilities."""

from __future__ import annotations

from pathlib import Path
import tempfile

import pytest

from savo_control.ros import (
    as_bool,
    as_float,
    as_int,
    deep_get,
    flatten_params,
    get_ros_parameters,
    load_ros_parameters,
    load_yaml_file,
    merge_dicts,
    normalized_topic,
    require_keys,
)


def test_load_yaml_file_reads_dictionary():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "params.yaml"
        path.write_text(
            "node:\n"
            "  ros__parameters:\n"
            "    value: 42\n",
            encoding="utf-8",
        )

        data = load_yaml_file(path)

    assert data["node"]["ros__parameters"]["value"] == 42


def test_load_yaml_file_empty_returns_empty_dict():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "empty.yaml"
        path.write_text("", encoding="utf-8")

        data = load_yaml_file(path)

    assert data == {}


def test_load_yaml_file_rejects_non_dictionary_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "list.yaml"
        path.write_text("- one\n- two\n", encoding="utf-8")

        with pytest.raises(ValueError):
            load_yaml_file(path)


def test_get_ros_parameters():
    data = {
        "test_node": {
            "ros__parameters": {
                "rate": 30.0,
                "enabled": True,
            }
        }
    }

    params = get_ros_parameters(data, "test_node")

    assert params == {
        "rate": 30.0,
        "enabled": True,
    }


def test_get_ros_parameters_rejects_missing_node():
    with pytest.raises(KeyError):
        get_ros_parameters({}, "missing_node")


def test_get_ros_parameters_rejects_missing_ros_parameters():
    with pytest.raises(KeyError):
        get_ros_parameters({"node": {}}, "node")


def test_load_ros_parameters():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "params.yaml"
        path.write_text(
            "test_node:\n"
            "  ros__parameters:\n"
            "    rate: 20.0\n",
            encoding="utf-8",
        )

        params = load_ros_parameters(path, "test_node")

    assert params["rate"] == 20.0


def test_deep_get():
    data = {
        "limits": {
            "vx": {
                "max_abs": 0.25,
            }
        }
    }

    assert deep_get(data, "limits.vx.max_abs") == 0.25
    assert deep_get(data, "limits.vy.max_abs", 0.0) == 0.0
    assert deep_get({}, "missing.value", "fallback") == "fallback"


def test_flatten_params():
    data = {
        "publish_rate_hz": 30.0,
        "limits": {
            "vx": {
                "max_abs": 0.25,
                "deadband": 0.005,
            },
            "wz": {
                "max_abs": 0.60,
            },
        },
    }

    flat = flatten_params(data)

    assert flat == {
        "publish_rate_hz": 30.0,
        "limits.vx.max_abs": 0.25,
        "limits.vx.deadband": 0.005,
        "limits.wz.max_abs": 0.60,
    }


def test_flatten_params_with_prefix():
    flat = flatten_params({"vx": {"max_abs": 0.25}}, prefix="limits")

    assert flat == {
        "limits.vx.max_abs": 0.25,
    }


def test_merge_dicts_preserves_nested_values():
    base = {
        "limits": {
            "vx": 0.25,
            "vy": 0.20,
        },
        "rate": 30.0,
    }
    override = {
        "limits": {
            "vx": 0.10,
        }
    }

    merged = merge_dicts(base, override)

    assert merged == {
        "limits": {
            "vx": 0.10,
            "vy": 0.20,
        },
        "rate": 30.0,
    }


def test_merge_dicts_replaces_non_dict_values():
    merged = merge_dicts(
        {
            "limits": {
                "vx": 0.25,
            }
        },
        {
            "limits": 0.10,
        },
    )

    assert merged == {
        "limits": 0.10,
    }


def test_require_keys_accepts_present_keys():
    require_keys(
        {
            "rate": 30.0,
            "enabled": True,
        },
        ["rate", "enabled"],
    )


def test_require_keys_rejects_missing_keys():
    with pytest.raises(KeyError):
        require_keys({"rate": 30.0}, ["rate", "enabled"])


def test_as_bool():
    assert as_bool(True) is True
    assert as_bool(False) is False

    assert as_bool("true") is True
    assert as_bool("1") is True
    assert as_bool("yes") is True
    assert as_bool("on") is True

    assert as_bool("false") is False
    assert as_bool("0") is False
    assert as_bool("no") is False
    assert as_bool("off") is False

    assert as_bool(None, default=True) is True
    assert as_bool(None, default=False) is False
    assert as_bool(1) is True
    assert as_bool(0) is False


def test_as_float():
    assert as_float(0.25) == 0.25
    assert as_float("0.25") == 0.25
    assert as_float(None, default=1.0) == 1.0
    assert as_float("bad", default=2.0) == 2.0


def test_as_int():
    assert as_int(5) == 5
    assert as_int("5") == 5
    assert as_int(None, default=1) == 1
    assert as_int("bad", default=2) == 2


def test_normalized_topic():
    assert normalized_topic("/cmd_vel") == "/cmd_vel"
    assert normalized_topic("cmd_vel") == "/cmd_vel"
    assert normalized_topic("//savo_control//mode_cmd") == "/savo_control/mode_cmd"
    assert normalized_topic("", default="/fallback") == "/fallback"
    assert normalized_topic(None, default="/fallback") == "/fallback"
