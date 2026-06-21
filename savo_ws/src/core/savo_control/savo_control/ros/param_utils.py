# -*- coding: utf-8 -*-

"""Small parameter helpers for Python fallback tools and tests."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml


def load_yaml_file(path: str | Path) -> dict:
    file_path = Path(path)

    data = yaml.safe_load(file_path.read_text(encoding="utf-8"))
    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML file must contain a dictionary: {file_path}")

    return data


def get_ros_parameters(data: Mapping[str, Any], node_name: str) -> dict:
    node = data.get(node_name)

    if not isinstance(node, Mapping):
        raise KeyError(f"Missing node section: {node_name}")

    params = node.get("ros__parameters")
    if not isinstance(params, Mapping):
        raise KeyError(f"Missing ros__parameters for node: {node_name}")

    return dict(params)


def load_ros_parameters(path: str | Path, node_name: str) -> dict:
    return get_ros_parameters(load_yaml_file(path), node_name)


def deep_get(data: Mapping[str, Any], dotted_key: str, default: Any = None) -> Any:
    current: Any = data

    for part in dotted_key.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return default
        current = current[part]

    return current


def flatten_params(data: Mapping[str, Any], *, prefix: str = "") -> dict[str, Any]:
    flat: dict[str, Any] = {}

    for key, value in data.items():
        dotted = f"{prefix}.{key}" if prefix else str(key)

        if isinstance(value, Mapping):
            flat.update(flatten_params(value, prefix=dotted))
        else:
            flat[dotted] = value

    return flat


def merge_dicts(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict:
    merged = dict(base)

    for key, value in override.items():
        if (
            key in merged
            and isinstance(merged[key], Mapping)
            and isinstance(value, Mapping)
        ):
            merged[key] = merge_dicts(merged[key], value)
        else:
            merged[key] = value

    return merged


def require_keys(data: Mapping[str, Any], keys: list[str]) -> None:
    missing = [key for key in keys if key not in data]

    if missing:
        raise KeyError(f"Missing required parameter keys: {missing}")


def as_bool(value: Any, *, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        normalized = value.strip().lower()

        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off"}:
            return False

    if value is None:
        return default

    return bool(value)


def as_float(value: Any, *, default: float = 0.0) -> float:
    if value is None:
        return default

    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_int(value: Any, *, default: int = 0) -> int:
    if value is None:
        return default

    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def normalized_topic(value: str, *, default: str = "") -> str:
    topic = str(value or default).strip()

    if not topic:
        return default

    if not topic.startswith("/"):
        topic = f"/{topic}"

    while "//" in topic:
        topic = topic.replace("//", "/")

    return topic


__all__ = [
    "as_bool",
    "as_float",
    "as_int",
    "deep_get",
    "flatten_params",
    "get_ros_parameters",
    "load_ros_parameters",
    "load_yaml_file",
    "merge_dicts",
    "normalized_topic",
    "require_keys",
]
