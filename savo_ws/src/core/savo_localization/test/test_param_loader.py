#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for YAML parameter loading helpers used by Robot Savo localization."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from savo_localization.utils.param_loader import (
    deep_merge_dicts,
    get_node_parameters,
    load_yaml_file,
    merge_ros_parameter_files,
    normalize_ros_parameters,
    require_mapping,
)


def write_yaml(path: Path, payload: dict) -> Path:
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )
    return path


def test_load_yaml_file_reads_mapping(tmp_path: Path) -> None:
    path = write_yaml(
        tmp_path / "params.yaml",
        {
            "imu_node": {
                "ros__parameters": {
                    "frame_id": "imu_link",
                    "publish_rate_hz": 25.0,
                }
            }
        },
    )

    data = load_yaml_file(path)

    assert data["imu_node"]["ros__parameters"]["frame_id"] == "imu_link"
    assert data["imu_node"]["ros__parameters"]["publish_rate_hz"] == pytest.approx(25.0)


def test_load_yaml_file_returns_empty_dict_for_empty_file(tmp_path: Path) -> None:
    path = tmp_path / "empty.yaml"
    path.write_text("", encoding="utf-8")

    assert load_yaml_file(path) == {}


def test_load_yaml_file_rejects_missing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        load_yaml_file(tmp_path / "missing.yaml")


def test_load_yaml_file_rejects_non_mapping_root(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text("- one\n- two\n", encoding="utf-8")

    with pytest.raises(ValueError):
        load_yaml_file(path)


def test_require_mapping_accepts_dict() -> None:
    data = {"a": 1}

    assert require_mapping(data, name="test") == data


def test_require_mapping_rejects_list() -> None:
    with pytest.raises(ValueError):
        require_mapping([1, 2, 3], name="test")


def test_deep_merge_dicts_overlays_simple_values() -> None:
    base = {
        "imu_node": {
            "ros__parameters": {
                "publish_rate_hz": 25.0,
                "mode": "ndof",
            }
        }
    }

    overlay = {
        "imu_node": {
            "ros__parameters": {
                "publish_rate_hz": 50.0,
            }
        }
    }

    merged = deep_merge_dicts(base, overlay)

    assert merged["imu_node"]["ros__parameters"]["publish_rate_hz"] == pytest.approx(50.0)
    assert merged["imu_node"]["ros__parameters"]["mode"] == "ndof"


def test_deep_merge_dicts_preserves_nested_values() -> None:
    base = {
        "wheel_odom_node": {
            "ros__parameters": {
                "frames": {
                    "odom": "odom",
                    "base": "base_link",
                },
                "publish_rate_hz": 30.0,
            }
        }
    }

    overlay = {
        "wheel_odom_node": {
            "ros__parameters": {
                "frames": {
                    "base": "robot_base",
                }
            }
        }
    }

    merged = deep_merge_dicts(base, overlay)

    assert merged["wheel_odom_node"]["ros__parameters"]["frames"]["odom"] == "odom"
    assert merged["wheel_odom_node"]["ros__parameters"]["frames"]["base"] == "robot_base"
    assert merged["wheel_odom_node"]["ros__parameters"]["publish_rate_hz"] == pytest.approx(30.0)


def test_deep_merge_dicts_does_not_mutate_inputs() -> None:
    base = {
        "imu_node": {
            "ros__parameters": {
                "publish_rate_hz": 25.0,
            }
        }
    }

    overlay = {
        "imu_node": {
            "ros__parameters": {
                "publish_rate_hz": 50.0,
            }
        }
    }

    merged = deep_merge_dicts(base, overlay)

    assert merged["imu_node"]["ros__parameters"]["publish_rate_hz"] == pytest.approx(50.0)
    assert base["imu_node"]["ros__parameters"]["publish_rate_hz"] == pytest.approx(25.0)
    assert overlay["imu_node"]["ros__parameters"]["publish_rate_hz"] == pytest.approx(50.0)


def test_normalize_ros_parameters_wraps_plain_node_mapping() -> None:
    normalized = normalize_ros_parameters(
        {
            "imu_node": {
                "frame_id": "imu_link",
                "publish_rate_hz": 25.0,
            }
        }
    )

    assert normalized == {
        "imu_node": {
            "ros__parameters": {
                "frame_id": "imu_link",
                "publish_rate_hz": 25.0,
            }
        }
    }


def test_normalize_ros_parameters_keeps_existing_ros_parameters() -> None:
    payload = {
        "wheel_odom_node": {
            "ros__parameters": {
                "publish_rate_hz": 30.0,
                "publish_tf": False,
            }
        }
    }

    normalized = normalize_ros_parameters(payload)

    assert normalized == payload


def test_get_node_parameters_returns_ros_parameters() -> None:
    payload = {
        "imu_node": {
            "ros__parameters": {
                "frame_id": "imu_link",
                "mode": "ndof",
            }
        }
    }

    params = get_node_parameters(payload, "imu_node")

    assert params["frame_id"] == "imu_link"
    assert params["mode"] == "ndof"


def test_get_node_parameters_returns_empty_for_missing_node() -> None:
    payload = {
        "imu_node": {
            "ros__parameters": {
                "frame_id": "imu_link",
            }
        }
    }

    assert get_node_parameters(payload, "wheel_odom_node") == {}


def test_get_node_parameters_rejects_bad_node_payload() -> None:
    payload = {
        "imu_node": "bad",
    }

    with pytest.raises(ValueError):
        get_node_parameters(payload, "imu_node")


def test_merge_ros_parameter_files_applies_overlay_order(tmp_path: Path) -> None:
    base = write_yaml(
        tmp_path / "base.yaml",
        {
            "wheel_odom_node": {
                "ros__parameters": {
                    "publish_rate_hz": 30.0,
                    "publish_tf": False,
                    "wheel_diameter_m": 0.065,
                }
            }
        },
    )

    profile = write_yaml(
        tmp_path / "profile.yaml",
        {
            "wheel_odom_node": {
                "ros__parameters": {
                    "publish_rate_hz": 20.0,
                    "publish_tf": True,
                }
            }
        },
    )

    merged = merge_ros_parameter_files([base, profile])
    params = get_node_parameters(merged, "wheel_odom_node")

    assert params["publish_rate_hz"] == pytest.approx(20.0)
    assert params["publish_tf"] is True
    assert params["wheel_diameter_m"] == pytest.approx(0.065)


def test_merge_ros_parameter_files_supports_plain_node_mappings(tmp_path: Path) -> None:
    base = write_yaml(
        tmp_path / "base.yaml",
        {
            "imu_node": {
                "frame_id": "imu_link",
                "publish_rate_hz": 25.0,
            }
        },
    )

    overlay = write_yaml(
        tmp_path / "overlay.yaml",
        {
            "imu_node": {
                "publish_rate_hz": 50.0,
            }
        },
    )

    merged = merge_ros_parameter_files([base, overlay])
    params = get_node_parameters(merged, "imu_node")

    assert params["frame_id"] == "imu_link"
    assert params["publish_rate_hz"] == pytest.approx(50.0)


def test_merge_ros_parameter_files_empty_list_returns_empty_dict() -> None:
    assert merge_ros_parameter_files([]) == {}


def test_robot_savo_profile_overlay_example(tmp_path: Path) -> None:
    topics = write_yaml(
        tmp_path / "topics.yaml",
        {
            "imu_node": {
                "ros__parameters": {
                    "imu_topic": "/imu/data",
                    "imu_state_topic": "/savo_localization/imu_state",
                }
            },
            "wheel_odom_node": {
                "ros__parameters": {
                    "wheel_odom_topic": "/wheel/odom",
                    "wheel_odom_state_topic": "/savo_localization/wheel_odom_state",
                }
            },
        },
    )

    frames = write_yaml(
        tmp_path / "frames.yaml",
        {
            "imu_node": {
                "ros__parameters": {
                    "frame_id": "imu_link",
                }
            },
            "wheel_odom_node": {
                "ros__parameters": {
                    "odom_frame_id": "odom",
                    "base_frame_id": "base_link",
                }
            },
        },
    )

    profile = write_yaml(
        tmp_path / "profile.yaml",
        {
            "imu_node": {
                "ros__parameters": {
                    "publish_rate_hz": 25.0,
                }
            },
            "wheel_odom_node": {
                "ros__parameters": {
                    "publish_rate_hz": 30.0,
                    "publish_tf": False,
                }
            },
        },
    )

    merged = merge_ros_parameter_files([topics, frames, profile])

    imu_params = get_node_parameters(merged, "imu_node")
    wheel_params = get_node_parameters(merged, "wheel_odom_node")

    assert imu_params["imu_topic"] == "/imu/data"
    assert imu_params["imu_state_topic"] == "/savo_localization/imu_state"
    assert imu_params["frame_id"] == "imu_link"
    assert imu_params["publish_rate_hz"] == pytest.approx(25.0)

    assert wheel_params["wheel_odom_topic"] == "/wheel/odom"
    assert wheel_params["wheel_odom_state_topic"] == "/savo_localization/wheel_odom_state"
    assert wheel_params["odom_frame_id"] == "odom"
    assert wheel_params["base_frame_id"] == "base_link"
    assert wheel_params["publish_rate_hz"] == pytest.approx(30.0)
    assert wheel_params["publish_tf"] is False
