#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YAML parameter loading helpers for Robot Savo localization."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Iterable

import yaml


def require_mapping(value: Any, *, name: str = "value") -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")

    return value


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    yaml_path = Path(path)

    if not yaml_path.exists():
        raise FileNotFoundError(str(yaml_path))

    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {yaml_path}")

    return data


def deep_merge_dicts(
    base: dict[str, Any],
    overlay: dict[str, Any],
) -> dict[str, Any]:
    result = deepcopy(base)

    for key, overlay_value in overlay.items():
        base_value = result.get(key)

        if isinstance(base_value, dict) and isinstance(overlay_value, dict):
            result[key] = deep_merge_dicts(base_value, overlay_value)
        else:
            result[key] = deepcopy(overlay_value)

    return result


def normalize_ros_parameters(payload: dict[str, Any]) -> dict[str, Any]:
    data = require_mapping(payload, name="payload")
    normalized: dict[str, Any] = {}

    for node_name, node_payload in data.items():
        if node_payload is None:
            normalized[node_name] = {"ros__parameters": {}}
            continue

        if not isinstance(node_payload, dict):
            raise ValueError(f"node payload for {node_name} must be a mapping")

        if "ros__parameters" in node_payload:
            params = node_payload["ros__parameters"]

            if params is None:
                params = {}

            if not isinstance(params, dict):
                raise ValueError(
                    f"ros__parameters for {node_name} must be a mapping"
                )

            normalized[node_name] = deepcopy(node_payload)
            normalized[node_name]["ros__parameters"] = deepcopy(params)
        else:
            normalized[node_name] = {
                "ros__parameters": deepcopy(node_payload),
            }

    return normalized


def get_node_parameters(
    payload: dict[str, Any],
    node_name: str,
) -> dict[str, Any]:
    data = require_mapping(payload, name="payload")

    if node_name not in data:
        return {}

    node_payload = data[node_name]

    if not isinstance(node_payload, dict):
        raise ValueError(f"node payload for {node_name} must be a mapping")

    if "ros__parameters" not in node_payload:
        return deepcopy(node_payload)

    params = node_payload["ros__parameters"]

    if params is None:
        return {}

    if not isinstance(params, dict):
        raise ValueError(f"ros__parameters for {node_name} must be a mapping")

    return deepcopy(params)


def get_ros_parameters(
    payload: dict[str, Any],
    node_name: str,
) -> dict[str, Any]:
    return get_node_parameters(payload, node_name)


def merge_ros_parameter_files(paths: Iterable[str | Path]) -> dict[str, Any]:
    merged: dict[str, Any] = {}

    for path in paths:
        loaded = normalize_ros_parameters(load_yaml_file(path))
        merged = deep_merge_dicts(merged, loaded)

    return merged


def load_merged_yaml(paths: Iterable[str | Path]) -> dict[str, Any]:
    merged: dict[str, Any] = {}

    for path in paths:
        merged = deep_merge_dicts(merged, load_yaml_file(path))

    return merged


def flatten_dict(
    payload: dict[str, Any],
    *,
    separator: str = ".",
    prefix: str = "",
) -> dict[str, Any]:
    data = require_mapping(payload, name="payload")
    flattened: dict[str, Any] = {}

    for key, value in data.items():
        full_key = f"{prefix}{separator}{key}" if prefix else str(key)

        if isinstance(value, dict):
            flattened.update(
                flatten_dict(
                    value,
                    separator=separator,
                    prefix=full_key,
                )
            )
        else:
            flattened[full_key] = value

    return flattened


def get_nested_value(
    payload: dict[str, Any],
    key_path: str,
    *,
    separator: str = ".",
    default: Any = None,
) -> Any:
    current: Any = payload

    for part in key_path.split(separator):
        if not isinstance(current, dict) or part not in current:
            return default

        current = current[part]

    return current


def require_nested_value(
    payload: dict[str, Any],
    key_path: str,
    *,
    separator: str = ".",
) -> Any:
    sentinel = object()
    value = get_nested_value(
        payload,
        key_path,
        separator=separator,
        default=sentinel,
    )

    if value is sentinel:
        raise KeyError(key_path)

    return value


def update_nested_value(
    payload: dict[str, Any],
    key_path: str,
    value: Any,
    *,
    separator: str = ".",
) -> dict[str, Any]:
    result = deepcopy(payload)
    current = result

    parts = key_path.split(separator)

    for part in parts[:-1]:
        next_value = current.get(part)

        if not isinstance(next_value, dict):
            next_value = {}
            current[part] = next_value

        current = next_value

    current[parts[-1]] = value
    return result


def require_ros_parameter(
    payload: dict[str, Any],
    node_name: str,
    parameter_name: str,
) -> Any:
    params = get_node_parameters(payload, node_name)

    if parameter_name not in params:
        raise KeyError(f"{node_name}.{parameter_name}")

    return params[parameter_name]


__all__ = [
    "require_mapping",
    "load_yaml_file",
    "deep_merge_dicts",
    "normalize_ros_parameters",
    "get_node_parameters",
    "get_ros_parameters",
    "merge_ros_parameter_files",
    "load_merged_yaml",
    "flatten_dict",
    "get_nested_value",
    "require_nested_value",
    "update_nested_value",
    "require_ros_parameter",
]
