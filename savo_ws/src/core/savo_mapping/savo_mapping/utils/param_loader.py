#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YAML parameter loading helpers. No ROS imports."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Optional

import yaml


# =============================================================================
# Path helpers
# =============================================================================
def resolve_path(path: str | Path) -> Path:
    value = Path(path).expanduser()

    if not value.is_absolute():
        value = Path.cwd() / value

    return value.resolve()


def require_file(path: str | Path) -> Path:
    resolved = resolve_path(path)

    if not resolved.exists():
        raise FileNotFoundError(f"File not found: {resolved}")

    if not resolved.is_file():
        raise ValueError(f"Path is not a file: {resolved}")

    return resolved


# =============================================================================
# YAML loading
# =============================================================================
def load_yaml_file(path: str | Path) -> Dict[str, Any]:
    file_path = require_file(path)

    with file_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {file_path}")

    return dict(data)


def load_yaml_files(paths: Iterable[str | Path]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}

    for path in paths:
        merged = deep_merge_dicts(merged, load_yaml_file(path))

    return merged


def save_yaml_file(path: str | Path, data: Mapping[str, Any]) -> Path:
    file_path = resolve_path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)

    with file_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(
            dict(data),
            stream,
            default_flow_style=False,
            sort_keys=False,
        )

    return file_path


# =============================================================================
# Dictionary helpers
# =============================================================================
def deep_merge_dicts(
    base: Mapping[str, Any],
    override: Mapping[str, Any],
) -> Dict[str, Any]:
    result: Dict[str, Any] = dict(base)

    for key, value in override.items():
        if (
            key in result
            and isinstance(result[key], MutableMapping)
            and isinstance(value, Mapping)
        ):
            result[key] = deep_merge_dicts(result[key], value)
        else:
            result[key] = value

    return result


def get_nested(
    data: Mapping[str, Any],
    path: str,
    default: Any = None,
    separator: str = ".",
) -> Any:
    if not path:
        return default

    current: Any = data

    for part in path.split(separator):
        key = part.strip()

        if not key:
            return default

        if not isinstance(current, Mapping) or key not in current:
            return default

        current = current[key]

    return current


def require_nested(
    data: Mapping[str, Any],
    path: str,
    separator: str = ".",
) -> Any:
    value = get_nested(data, path, default=None, separator=separator)

    if value is None:
        raise KeyError(f"Missing required parameter: {path}")

    return value


def set_nested(
    data: Mapping[str, Any],
    path: str,
    value: Any,
    separator: str = ".",
) -> Dict[str, Any]:
    result: Dict[str, Any] = dict(data)
    current: Dict[str, Any] = result

    parts = [part.strip() for part in path.split(separator) if part.strip()]

    if not parts:
        raise ValueError("Nested path cannot be empty.")

    for part in parts[:-1]:
        existing = current.get(part)

        if not isinstance(existing, dict):
            existing = {}

        current[part] = existing
        current = existing

    current[parts[-1]] = value
    return result


def require_mapping(value: Any, name: str = "value") -> Dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping.")

    return dict(value)


# =============================================================================
# ROS-style parameter helpers
# =============================================================================
def get_ros_parameters(
    data: Mapping[str, Any],
    node_name: str,
) -> Dict[str, Any]:
    node_data = get_nested(data, node_name, default={})
    node_map = require_mapping(node_data, node_name)

    ros_params = node_map.get("ros__parameters", {})

    return require_mapping(ros_params, f"{node_name}.ros__parameters")


def load_ros_parameters(
    path: str | Path,
    node_name: str,
) -> Dict[str, Any]:
    return get_ros_parameters(load_yaml_file(path), node_name)


def merge_ros_parameter_files(
    paths: Iterable[str | Path],
    node_name: str,
) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}

    for path in paths:
        merged = deep_merge_dicts(
            merged,
            load_ros_parameters(path, node_name),
        )

    return merged


# =============================================================================
# Profile helpers
# =============================================================================
def load_profile(
    profile_path: str | Path,
    overrides: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    profile = load_yaml_file(profile_path)

    if overrides:
        profile = deep_merge_dicts(profile, overrides)

    return profile


def load_profile_stack(
    profile_paths: Iterable[str | Path],
    overrides: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    profile = load_yaml_files(profile_paths)

    if overrides:
        profile = deep_merge_dicts(profile, overrides)

    return profile


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    sample = {
        "mapping_supervisor_node": {
            "ros__parameters": {
                "scan_topic": "/scan",
                "odom_topic": "/odometry/filtered",
                "publish_rate_hz": 5.0,
            }
        }
    }

    print(get_ros_parameters(sample, "mapping_supervisor_node"))
    print(get_nested(sample, "mapping_supervisor_node.ros__parameters.scan_topic"))


if __name__ == "__main__":
    main()


__all__ = [
    "resolve_path",
    "require_file",
    "load_yaml_file",
    "load_yaml_files",
    "save_yaml_file",
    "deep_merge_dicts",
    "get_nested",
    "require_nested",
    "set_nested",
    "require_mapping",
    "get_ros_parameters",
    "load_ros_parameters",
    "merge_ros_parameter_files",
    "load_profile",
    "load_profile_stack",
    "main",
]