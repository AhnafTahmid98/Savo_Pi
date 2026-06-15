# Copyright 2026 Ahnaf Tahmid
from __future__ import annotations

from typing import Any


def get_param_value(node, name: str, default: Any) -> Any:
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def get_float_param(node, name: str, default: float, min_value: float | None = None) -> float:
    value = float(get_param_value(node, name, default))

    if min_value is not None and value < min_value:
        return default

    return value


def get_int_param(node, name: str, default: int, min_value: int | None = None) -> int:
    value = int(get_param_value(node, name, default))

    if min_value is not None and value < min_value:
        return default

    return value


def get_bool_param(node, name: str, default: bool) -> bool:
    return bool(get_param_value(node, name, default))


def get_str_param(node, name: str, default: str, allow_empty: bool = False) -> str:
    value = str(get_param_value(node, name, default)).strip()

    if not allow_empty and not value:
        return default

    return value
