"""Logging helpers for Robot Savo power Python tools."""

from __future__ import annotations

import logging as py_logging
import sys
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

from savo_power import constants as c


class LogLevel(str, Enum):
    """Stable log level names."""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


@dataclass(frozen=True)
class LoggingConfig:
    """Configuration for Python logging."""

    name: str = c.PACKAGE_NAME
    level: LogLevel = LogLevel.INFO

    include_time: bool = True
    include_level: bool = True
    include_name: bool = True

    stream: str = "stderr"


class ThrottleState:
    """Small helper for throttled log messages."""

    def __init__(self) -> None:
        self._last_log_time_by_key: dict[str, float] = {}

    def should_log(self, key: str, period_s: float) -> bool:
        """Return True when a throttled message should be emitted."""

        now = time.monotonic()
        safe_period = max(0.0, float(period_s))
        previous = self._last_log_time_by_key.get(key)

        if previous is None or now - previous >= safe_period:
            self._last_log_time_by_key[key] = now
            return True

        return False

    def clear(self) -> None:
        """Clear throttle state."""

        self._last_log_time_by_key.clear()


_GLOBAL_THROTTLE_STATE = ThrottleState()


def normalize_log_level(value: str | int | LogLevel | None) -> LogLevel:
    """Normalize common log level values."""

    if isinstance(value, LogLevel):
        return value

    if isinstance(value, int):
        if value <= py_logging.DEBUG:
            return LogLevel.DEBUG

        if value <= py_logging.INFO:
            return LogLevel.INFO

        if value <= py_logging.WARNING:
            return LogLevel.WARNING

        if value <= py_logging.ERROR:
            return LogLevel.ERROR

        return LogLevel.CRITICAL

    if value is None:
        return LogLevel.INFO

    normalized = str(value).strip().upper()

    if normalized in {"WARN"}:
        return LogLevel.WARNING

    for level in LogLevel:
        if normalized == level.value:
            return level

    return LogLevel.INFO


def log_level_to_python(level: str | int | LogLevel | None) -> int:
    """Convert stable log level to Python logging level."""

    normalized = normalize_log_level(level)

    if normalized == LogLevel.DEBUG:
        return py_logging.DEBUG

    if normalized == LogLevel.INFO:
        return py_logging.INFO

    if normalized == LogLevel.WARNING:
        return py_logging.WARNING

    if normalized == LogLevel.ERROR:
        return py_logging.ERROR

    return py_logging.CRITICAL


def python_level_to_log_level(level: int) -> LogLevel:
    """Convert Python logging level to stable log level."""

    return normalize_log_level(int(level))


def make_log_format(config: LoggingConfig | None = None) -> str:
    """Create a Python logging format string."""

    config = config or LoggingConfig()
    parts: list[str] = []

    if config.include_time:
        parts.append("%(asctime)s")

    if config.include_level:
        parts.append("%(levelname)s")

    if config.include_name:
        parts.append("[%(name)s]")

    parts.append("%(message)s")

    return " ".join(parts)


def _stream_from_name(name: str) -> Any:
    normalized = str(name).strip().lower()

    if normalized == "stdout":
        return sys.stdout

    return sys.stderr


def configure_python_logging(
    config: LoggingConfig | None = None,
    *,
    force: bool = False,
) -> py_logging.Logger:
    """Configure Python logging and return package logger."""

    config = config or LoggingConfig()

    py_logging.basicConfig(
        level=log_level_to_python(config.level),
        format=make_log_format(config),
        stream=_stream_from_name(config.stream),
        force=force,
    )

    logger = py_logging.getLogger(config.name)
    logger.setLevel(log_level_to_python(config.level))

    return logger


def get_python_logger(name: str | None = None) -> py_logging.Logger:
    """Return Python logger for this package or child name."""

    if not name:
        return py_logging.getLogger(c.PACKAGE_NAME)

    if name.startswith(c.PACKAGE_NAME):
        return py_logging.getLogger(name)

    return py_logging.getLogger(f"{c.PACKAGE_NAME}.{name}")


def set_python_logger_level(
    logger: py_logging.Logger,
    level: str | int | LogLevel | None,
) -> None:
    """Set Python logger level."""

    logger.setLevel(log_level_to_python(level))


def get_node_logger(
    node: object | None,
    *,
    fallback_name: str = c.PACKAGE_NAME,
) -> object:
    """Return ROS node logger if possible, otherwise Python logger."""

    if node is not None and hasattr(node, "get_logger"):
        try:
            logger = node.get_logger()

            if logger is not None:
                return logger
        except Exception:
            pass

    return get_python_logger(fallback_name)


def _call_logger_method(
    logger: object,
    method_names: tuple[str, ...],
    message: str,
) -> bool:
    for method_name in method_names:
        method = getattr(logger, method_name, None)

        if callable(method):
            method(message)
            return True

    return False


def log_debug(logger: object, message: object) -> None:
    """Log debug message using ROS-style or Python-style logger."""

    text = str(message)

    if _call_logger_method(logger, ("debug",), text):
        return

    get_python_logger().debug(text)


def log_info(logger: object, message: object) -> None:
    """Log info message using ROS-style or Python-style logger."""

    text = str(message)

    if _call_logger_method(logger, ("info",), text):
        return

    get_python_logger().info(text)


def log_warning(logger: object, message: object) -> None:
    """Log warning message using ROS-style or Python-style logger."""

    text = str(message)

    if _call_logger_method(logger, ("warning", "warn"), text):
        return

    get_python_logger().warning(text)


def log_error(logger: object, message: object) -> None:
    """Log error message using ROS-style or Python-style logger."""

    text = str(message)

    if _call_logger_method(logger, ("error",), text):
        return

    get_python_logger().error(text)


def log_critical(logger: object, message: object) -> None:
    """Log critical/fatal message using ROS-style or Python-style logger."""

    text = str(message)

    if _call_logger_method(logger, ("fatal", "critical"), text):
        return

    get_python_logger().critical(text)


def log_by_level(
    logger: object,
    level: str | int | LogLevel | None,
    message: object,
) -> None:
    """Log message at normalized level."""

    normalized = normalize_log_level(level)

    if normalized == LogLevel.DEBUG:
        log_debug(logger, message)
        return

    if normalized == LogLevel.INFO:
        log_info(logger, message)
        return

    if normalized == LogLevel.WARNING:
        log_warning(logger, message)
        return

    if normalized == LogLevel.ERROR:
        log_error(logger, message)
        return

    log_critical(logger, message)


def log_once(
    logger: object,
    key: str,
    level: str | int | LogLevel | None,
    message: object,
    *,
    state: ThrottleState | None = None,
) -> bool:
    """Log a message once per key."""

    throttle = state or _GLOBAL_THROTTLE_STATE

    if not throttle.should_log(f"once:{key}", float("inf")):
        return False

    log_by_level(logger, level, message)
    return True


def log_throttled(
    logger: object,
    key: str,
    level: str | int | LogLevel | None,
    message: object,
    *,
    period_s: float = 5.0,
    state: ThrottleState | None = None,
) -> bool:
    """Log a message at most once per period."""

    throttle = state or _GLOBAL_THROTTLE_STATE

    if not throttle.should_log(f"throttle:{key}", period_s):
        return False

    log_by_level(logger, level, message)
    return True


def log_exception(
    logger: object,
    message: object,
    exc: BaseException,
    *,
    level: str | int | LogLevel | None = LogLevel.ERROR,
) -> None:
    """Log exception as compact text."""

    log_by_level(
        logger,
        level,
        f"{message}: {type(exc).__name__}: {exc}",
    )


def log_power_reading(logger: object, reading: object) -> None:
    """Log a power reading using its format_line() method if available."""

    if hasattr(reading, "format_line"):
        log_info(logger, reading.format_line())
        return

    log_info(logger, reading)


def log_startup(logger: object, node_name: str, *, backend: str = "python") -> None:
    """Log a stable node startup message."""

    log_info(
        logger,
        f"{node_name} started backend={backend}",
    )


def log_shutdown(logger: object, node_name: str) -> None:
    """Log a stable node shutdown message."""

    log_info(
        logger,
        f"{node_name} stopped",
    )


def reset_global_throttle_state() -> None:
    """Reset global throttle state. Mostly useful for tests."""

    _GLOBAL_THROTTLE_STATE.clear()


__all__ = [
    "LogLevel",
    "LoggingConfig",
    "ThrottleState",
    "configure_python_logging",
    "get_node_logger",
    "get_python_logger",
    "log_by_level",
    "log_critical",
    "log_debug",
    "log_error",
    "log_exception",
    "log_info",
    "log_once",
    "log_power_reading",
    "log_shutdown",
    "log_startup",
    "log_throttled",
    "log_warning",
    "log_level_to_python",
    "make_log_format",
    "normalize_log_level",
    "python_level_to_log_level",
    "reset_global_throttle_state",
    "set_python_logger_level",
]
