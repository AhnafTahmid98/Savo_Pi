# -*- coding: utf-8 -*-

"""Validation helpers for configs, commands, and CLI input."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .math_utils import finite_or_zero


def require_non_empty_string(value: object, *, name: str) -> str:
    text = str(value).strip()

    if not text:
        raise ValueError(f"{name} must be a non-empty string")

    return text


def require_positive_float(value: object, *, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a positive float") from exc

    if result <= 0.0:
        raise ValueError(f"{name} must be > 0.0")

    return result


def require_non_negative_float(value: object, *, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a non-negative float") from exc

    if result < 0.0:
        raise ValueError(f"{name} must be >= 0.0")

    return result


def require_bool(value: object, *, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a bool")

    return value


def require_choice(value: object, choices: list[str] | tuple[str, ...], *, name: str) -> str:
    text = require_non_empty_string(value, name=name)

    if text not in choices:
        raise ValueError(f"{name} must be one of: {', '.join(choices)}")

    return text


def require_mapping(value: object, *, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")

    return value


def require_keys(data: Mapping[str, Any], keys: list[str], *, name: str = "data") -> None:
    missing = [key for key in keys if key not in data]

    if missing:
        raise ValueError(f"{name} missing required keys: {missing}")


def validate_topic_name(topic: object, *, name: str = "topic") -> str:
    text = require_non_empty_string(topic, name=name)

    if not text.startswith("/"):
        raise ValueError(f"{name} must start with '/'")

    if "//" in text:
        raise ValueError(f"{name} must not contain '//'")

    if " " in text:
        raise ValueError(f"{name} must not contain spaces")

    return text


def sanitize_float(value: object, *, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default

    safe = finite_or_zero(result)
    return safe if safe != 0.0 else default if result != 0.0 else 0.0


def validate_limits(
    *,
    max_vx: float,
    max_vy: float,
    max_wz: float,
) -> tuple[float, float, float]:
    return (
        require_positive_float(max_vx, name="max_vx"),
        require_positive_float(max_vy, name="max_vy"),
        require_positive_float(max_wz, name="max_wz"),
    )


def validate_rate(rate_hz: object, *, name: str = "rate_hz") -> float:
    return require_positive_float(rate_hz, name=name)


def validate_timeout(timeout_s: object, *, name: str = "timeout_s") -> float:
    return require_non_negative_float(timeout_s, name=name)


__all__ = [
    "require_bool",
    "require_choice",
    "require_keys",
    "require_mapping",
    "require_non_empty_string",
    "require_non_negative_float",
    "require_positive_float",
    "sanitize_float",
    "validate_limits",
    "validate_rate",
    "validate_timeout",
    "validate_topic_name",
]
