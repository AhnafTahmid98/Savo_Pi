"""Validation helpers for package configuration and runtime values."""


def require_non_empty_string(value: str, name: str) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a string")

    stripped = value.strip()
    if not stripped:
        raise ValueError(f"{name} must not be empty")

    return stripped


def require_positive_float(value: float, name: str) -> float:
    numeric_value = float(value)
    if numeric_value <= 0.0:
        raise ValueError(f"{name} must be positive")

    return numeric_value


def require_non_negative_float(value: float, name: str) -> float:
    numeric_value = float(value)
    if numeric_value < 0.0:
        raise ValueError(f"{name} must be non-negative")

    return numeric_value


def require_positive_int(value: int, name: str) -> int:
    numeric_value = int(value)
    if numeric_value <= 0:
        raise ValueError(f"{name} must be positive")

    return numeric_value


def require_non_negative_int(value: int, name: str) -> int:
    numeric_value = int(value)
    if numeric_value < 0:
        raise ValueError(f"{name} must be non-negative")

    return numeric_value


def require_probability(value: float, name: str) -> float:
    numeric_value = float(value)
    if numeric_value < 0.0 or numeric_value > 1.0:
        raise ValueError(f"{name} must be between 0.0 and 1.0")

    return numeric_value