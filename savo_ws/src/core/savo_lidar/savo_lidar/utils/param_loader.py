"""YAML parameter loading helpers used by launch profiles and CLI tools."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_yaml_file(path: str | Path) -> dict[str, Any]:
    yaml_path = Path(path).expanduser().resolve()

    if not yaml_path.exists():
        raise FileNotFoundError(f"YAML file does not exist: {yaml_path}")

    if not yaml_path.is_file():
        raise FileNotFoundError(f"YAML path is not a file: {yaml_path}")

    with yaml_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if data is None:
        return {}

    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a dictionary: {yaml_path}")

    return data


def deep_merge_dicts(
    base: dict[str, Any],
    override: dict[str, Any],
) -> dict[str, Any]:
    merged = dict(base)

    for key, value in override.items():
        if (
            key in merged
            and isinstance(merged[key], dict)
            and isinstance(value, dict)
        ):
            merged[key] = deep_merge_dicts(merged[key], value)
        else:
            merged[key] = value

    return merged


def load_merged_yaml(*paths: str | Path) -> dict[str, Any]:
    merged: dict[str, Any] = {}

    for path in paths:
        loaded = load_yaml_file(path)
        merged = deep_merge_dicts(merged, loaded)

    return merged


def get_nested_value(
    data: dict[str, Any],
    dotted_key: str,
    default: Any = None,
) -> Any:
    current: Any = data

    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]

    return current


def require_nested_value(data: dict[str, Any], dotted_key: str) -> Any:
    value = get_nested_value(data, dotted_key, default=None)

    if value is None:
        raise KeyError(f"Missing required config value: {dotted_key}")

    return value