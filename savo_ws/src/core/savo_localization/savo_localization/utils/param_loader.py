#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YAML loading and merge helpers. No ROS imports."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    file_path = Path(path).expanduser()

    if not file_path.exists():
        raise FileNotFoundError(f"YAML file not found: {file_path}")

    if not file_path.is_file():
        raise ValueError(f"YAML path is not a file: {file_path}")

    with file_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a dictionary: {file_path}")

    return data


def deep_merge_dicts(
    base: dict[str, Any],
    override: dict[str, Any],
) -> dict[str, Any]:
    merged = deepcopy(base)

    for key, value in override.items():
        if (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        ):
            merged[key] = deep_merge_dicts(merged[key], value)
        else:
            merged[key] = deepcopy(value)

    return merged


def load_merged_yaml(*paths: str | Path) -> dict[str, Any]:
    merged: dict[str, Any] = {}

    for path in paths:
        data = load_yaml_file(path)
        merged = deep_merge_dicts(merged, data)

    return merged


def get_nested_value(
    data: dict[str, Any],
    dotted_key: str,
    default: Any = None,
) -> Any:
    current: Any = data

    for part in str(dotted_key).split("."):
        if not isinstance(current, dict) or part not in current:
            return default

        current = current[part]

    return current


def require_nested_value(data: dict[str, Any], dotted_key: str) -> Any:
    value = get_nested_value(data, dotted_key, default=None)

    if value is None:
        raise KeyError(f"Required config value missing: {dotted_key}")

    return value


def get_ros_parameters(
    data: dict[str, Any],
    node_name: str,
) -> dict[str, Any]:
    value = get_nested_value(
        data,
        f"{node_name}.ros__parameters",
        default={},
    )

    if not isinstance(value, dict):
        raise ValueError(f"{node_name}.ros__parameters must be a dictionary")

    return value


def require_ros_parameter(
    data: dict[str, Any],
    node_name: str,
    param_name: str,
) -> Any:
    return require_nested_value(
        data,
        f"{node_name}.ros__parameters.{param_name}",
    )


def update_nested_value(
    data: dict[str, Any],
    dotted_key: str,
    value: Any,
) -> dict[str, Any]:
    updated = deepcopy(data)
    current = updated

    parts = str(dotted_key).split(".")
    if not parts:
        raise ValueError("dotted_key cannot be empty")

    for part in parts[:-1]:
        if part not in current or not isinstance(current[part], dict):
            current[part] = {}

        current = current[part]

    current[parts[-1]] = value
    return updated


def flatten_dict(
    data: dict[str, Any],
    *,
    prefix: str = "",
) -> dict[str, Any]:
    flattened: dict[str, Any] = {}

    for key, value in data.items():
        full_key = f"{prefix}.{key}" if prefix else str(key)

        if isinstance(value, dict):
            flattened.update(flatten_dict(value, prefix=full_key))
        else:
            flattened[full_key] = value

    return flattened