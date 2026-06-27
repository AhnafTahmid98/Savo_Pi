#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Config and parameter loading helpers for savo_perception."""

from __future__ import annotations

import json
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional


try:
    import yaml  # type: ignore

    YAML_AVAILABLE = True
except Exception:
    yaml = None
    YAML_AVAILABLE = False


@dataclass(frozen=True)
class ParamLoadResult:
    path: str
    data: Dict[str, Any]
    params: Dict[str, Any]
    node_name: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "path": self.path,
            "node_name": self.node_name,
            "data": self.data,
            "params": self.params,
        }


def is_yaml_available() -> bool:
    return YAML_AVAILABLE


def _ensure_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def normalize_node_name(name: str) -> str:
    text = str(name or "").strip()
    if not text:
        return ""
    return text[1:] if text.startswith("/") else text


def load_json_file(path: str | Path) -> Dict[str, Any]:
    p = Path(path).expanduser()
    with p.open("r", encoding="utf-8") as f:
        data = json.load(f)
    return _ensure_dict(data)


def load_yaml_file(path: str | Path) -> Dict[str, Any]:
    if not YAML_AVAILABLE:
        raise RuntimeError("PyYAML is not available. Install python3-yaml.")

    p = Path(path).expanduser()
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)  # type: ignore[union-attr]

    return _ensure_dict(data)


def load_config_file(path: str | Path) -> Dict[str, Any]:
    p = Path(path).expanduser()
    suffix = p.suffix.lower()

    if suffix == ".json":
        return load_json_file(p)

    if suffix in {".yaml", ".yml"}:
        return load_yaml_file(p)

    raise ValueError(f"Unsupported config file type: {p}")


def deep_merge(base: Mapping[str, Any], override: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    result = deepcopy(dict(base))

    if not override:
        return result

    for key, value in override.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, Mapping)
        ):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)

    return result


def merge_many(*maps: Optional[Mapping[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}

    for item in maps:
        if item:
            result = deep_merge(result, item)

    return result


def get_nested(
    data: Mapping[str, Any],
    keys: Iterable[str],
    default: Any = None,
) -> Any:
    current: Any = data

    for key in keys:
        if not isinstance(current, Mapping):
            return default
        if key not in current:
            return default
        current = current[key]

    return current


def flatten_nested(
    data: Mapping[str, Any],
    *,
    prefix: str = "",
    separator: str = ".",
) -> Dict[str, Any]:
    out: Dict[str, Any] = {}

    for key, value in data.items():
        full_key = f"{prefix}{separator}{key}" if prefix else str(key)

        if isinstance(value, Mapping):
            out.update(flatten_nested(value, prefix=full_key, separator=separator))
        else:
            out[full_key] = value

    return out


def extract_ros_parameters(
    config: Mapping[str, Any],
    *,
    node_name: str = "",
) -> Dict[str, Any]:
    data = _ensure_dict(config)

    if "ros__parameters" in data and isinstance(data["ros__parameters"], Mapping):
        return dict(data["ros__parameters"])

    node = normalize_node_name(node_name)
    merged: Dict[str, Any] = {}

    wildcard_params = get_nested(data, ["/**", "ros__parameters"], default={})
    if isinstance(wildcard_params, Mapping):
        merged = deep_merge(merged, wildcard_params)

    if node:
        candidates = [
            node,
            f"/{node}",
        ]

        for candidate in candidates:
            params = get_nested(data, [candidate, "ros__parameters"], default=None)
            if isinstance(params, Mapping):
                merged = deep_merge(merged, params)

    return merged


def load_ros_params_file(path: str | Path, *, node_name: str = "") -> ParamLoadResult:
    data = load_config_file(path)
    params = extract_ros_parameters(data, node_name=node_name)

    return ParamLoadResult(
        path=str(Path(path).expanduser()),
        data=data,
        params=params,
        node_name=normalize_node_name(node_name),
    )


def overlay_params(
    defaults: Mapping[str, Any],
    file_params: Optional[Mapping[str, Any]] = None,
    cli_params: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    return merge_many(defaults, file_params, cli_params)


def require_keys(params: Mapping[str, Any], required_keys: Iterable[str]) -> list[str]:
    return [key for key in required_keys if key not in params]


def pick_keys(params: Mapping[str, Any], keys: Iterable[str]) -> Dict[str, Any]:
    return {key: params[key] for key in keys if key in params}


def drop_none_values(params: Mapping[str, Any]) -> Dict[str, Any]:
    return {key: value for key, value in params.items() if value is not None}


def coerce_hex_int(value: Any) -> int:
    if isinstance(value, int):
        return value

    text = str(value).strip()
    return int(text, 0)


def config_path_exists(path: str | Path) -> bool:
    return Path(path).expanduser().exists()


__all__ = [
    "YAML_AVAILABLE",
    "ParamLoadResult",
    "is_yaml_available",
    "normalize_node_name",
    "load_json_file",
    "load_yaml_file",
    "load_config_file",
    "deep_merge",
    "merge_many",
    "get_nested",
    "flatten_nested",
    "extract_ros_parameters",
    "load_ros_params_file",
    "overlay_params",
    "require_keys",
    "pick_keys",
    "drop_none_values",
    "coerce_hex_int",
    "config_path_exists",
]