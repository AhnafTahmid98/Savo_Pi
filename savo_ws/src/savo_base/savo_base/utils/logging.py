#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/utils/logging.py
---------------------------------------
Lightweight logging helpers for `savo_base` (ROS2 Jazzy friendly).

Purpose
-------
Provide a small abstraction layer so modules can log consistently whether they
run:
- inside ROS2 nodes (`rclpy` logger available), or
- in plain Python scripts / diagnostics (no ROS logger)

Design goals
------------
- Dependency-light (standard library only in this file)
- Safe in real robot runtime loops
- Consistent message style for hardware/base bringup
- Easy drop-in usage from drivers, safety policies, and utils

Typical usage
-------------
from savo_base.utils.logging import (
    get_logger_adapter,
    log_info,
    log_warn,
    log_error,
    log_debug,
)

logger = get_logger_adapter(self)   # self can be a ROS2 node
log_info(logger, "Motor board initialized")
"""

from __future__ import annotations

import json
import logging
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional


# =============================================================================
# Internal constants
# =============================================================================

_LEVEL_DEBUG = "DEBUG"
_LEVEL_INFO = "INFO"
_LEVEL_WARN = "WARN"
_LEVEL_ERROR = "ERROR"


# =============================================================================
# Time / formatting helpers
# =============================================================================

def _ts_wall_str() -> str:
    """
    Human-readable local timestamp for console logs.
    Example: 2026-02-22 21:14:08
    """
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())


def _safe_json(payload: Dict[str, Any]) -> str:
    """
    Compact JSON string for structured details.
    """
    try:
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), default=str)
    except Exception:
        return str(payload)


def _stringify_message(msg: Any) -> str:
    try:
        return str(msg)
    except Exception:
        return "<unprintable message>"


# =============================================================================
# Stdlib fallback logger setup
# =============================================================================

def _ensure_std_logger(name: str = "savo_base") -> logging.Logger:
    """
    Return a configured stdlib logger (idempotent).
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        fmt = logging.Formatter(
            fmt="%(asctime)s | %(name)s | %(levelname)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        handler.setFormatter(fmt)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger


# =============================================================================
# Logger adapter (ROS2 logger or stdlib logger)
# =============================================================================

@dataclass
class LoggerAdapter:
    """
    Small adapter that hides whether the underlying logger is:
    - a ROS2 logger (rclpy node logger), or
    - a stdlib logging.Logger instance

    Methods match common ROS logger style:
      debug(), info(), warn(), error()
    """
    target: Any
    name: str = "savo_base"

    def __post_init__(self) -> None:
        if self.target is None:
            self.target = _ensure_std_logger(self.name)

    # ---- Introspection -----------------------------------------------------

    @property
    def is_ros_logger(self) -> bool:
        """
        Best-effort detection of ROS2 logger-like object.
        """
        t = self.target
        return all(hasattr(t, m) for m in ("info", "warn", "error"))

    @property
    def is_std_logger(self) -> bool:
        return isinstance(self.target, logging.Logger)

    # ---- Core emit ---------------------------------------------------------

    def _emit(self, level: str, msg: Any) -> None:
        text = _stringify_message(msg)

        # ROS2 logger path (rclpy logger or node.get_logger())
        if self.is_ros_logger:
            try:
                if level == _LEVEL_DEBUG and hasattr(self.target, "debug"):
                    self.target.debug(text)
                    return
                if level == _LEVEL_INFO and hasattr(self.target, "info"):
                    self.target.info(text)
                    return
                if level == _LEVEL_WARN and hasattr(self.target, "warn"):
                    self.target.warn(text)
                    return
                if level == _LEVEL_ERROR and hasattr(self.target, "error"):
                    self.target.error(text)
                    return
            except Exception:
                # Fall through to std logger
                pass

        # stdlib fallback
        std_logger = self.target if self.is_std_logger else _ensure_std_logger(self.name)
        if level == _LEVEL_DEBUG:
            std_logger.debug(text)
        elif level == _LEVEL_INFO:
            std_logger.info(text)
        elif level == _LEVEL_WARN:
            std_logger.warning(text)
        else:
            std_logger.error(text)

    # ---- Public methods ----------------------------------------------------

    def debug(self, msg: Any) -> None:
        self._emit(_LEVEL_DEBUG, msg)

    def info(self, msg: Any) -> None:
        self._emit(_LEVEL_INFO, msg)

    def warn(self, msg: Any) -> None:
        self._emit(_LEVEL_WARN, msg)

    def error(self, msg: Any) -> None:
        self._emit(_LEVEL_ERROR, msg)


# =============================================================================
# Public helper constructors
# =============================================================================

def get_logger_adapter(source: Any = None, *, name: str = "savo_base") -> LoggerAdapter:
    """
    Create a LoggerAdapter from a source object.

    Supported sources
    -----------------
    - ROS2 Node (`source.get_logger()`)
    - ROS2 logger directly
    - stdlib logging.Logger
    - None (creates stdlib fallback logger)

    Examples
    --------
    logger = get_logger_adapter(self)               # from ROS2 node
    logger = get_logger_adapter(self.get_logger())  # from ROS2 logger
    logger = get_logger_adapter()                   # plain python fallback
    """
    if source is None:
        return LoggerAdapter(target=_ensure_std_logger(name), name=name)

    # ROS2 Node-like object
    if hasattr(source, "get_logger") and callable(source.get_logger):
        try:
            return LoggerAdapter(target=source.get_logger(), name=name)
        except Exception:
            return LoggerAdapter(target=_ensure_std_logger(name), name=name)

    # Already a logger-like object or stdlib logger
    return LoggerAdapter(target=source, name=name)


# =============================================================================
# Structured logging helpers
# =============================================================================

def format_kv(**kwargs: Any) -> str:
    """
    Format key=value pairs into a compact stable string.

    Example:
      format_kv(backend="freenove", addr="0x40", pwm_hz=50)
      -> "backend=freenove addr=0x40 pwm_hz=50"
    """
    parts = []
    for k, v in kwargs.items():
        parts.append(f"{k}={v}")
    return " ".join(parts)


def format_event(
    event: str,
    *,
    level: str = _LEVEL_INFO,
    component: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Format a standardized event log line.

    Example output:
      [INFO] [base_driver_node] motor_board_init backend=freenove addr=0x40
    """
    lvl = str(level).upper()
    comp = f"[{component}] " if component else ""
    base = f"[{lvl}] {comp}{event}"
    if details:
        # Prefer human-friendly kv if flat; else compact JSON
        try:
            if all(isinstance(k, str) for k in details.keys()):
                return f"{base} {format_kv(**details)}"
        except Exception:
            pass
        return f"{base} details={_safe_json(details)}"
    return base


def log_event(
    logger: LoggerAdapter,
    event: str,
    *,
    level: str = _LEVEL_INFO,
    component: Optional[str] = None,
    details: Optional[Dict[str, Any]] = None,
) -> None:
    """
    Emit a standardized event message through the adapter.
    """
    msg = format_event(event, level=level, component=component, details=details)
    lvl = str(level).upper()
    if lvl == _LEVEL_DEBUG:
        logger.debug(msg)
    elif lvl == _LEVEL_INFO:
        logger.info(msg)
    elif lvl in ("WARN", "WARNING"):
        logger.warn(msg)
    elif lvl == _LEVEL_ERROR:
        logger.error(msg)
    else:
        logger.info(msg)


# =============================================================================
# Convenience wrappers (common usage in nodes)
# =============================================================================

def log_debug(logger: LoggerAdapter, msg: Any, *, component: Optional[str] = None) -> None:
    if component:
        logger.debug(f"[{component}] {_stringify_message(msg)}")
    else:
        logger.debug(msg)


def log_info(logger: LoggerAdapter, msg: Any, *, component: Optional[str] = None) -> None:
    if component:
        logger.info(f"[{component}] {_stringify_message(msg)}")
    else:
        logger.info(msg)


def log_warn(logger: LoggerAdapter, msg: Any, *, component: Optional[str] = None) -> None:
    if component:
        logger.warn(f"[{component}] {_stringify_message(msg)}")
    else:
        logger.warn(msg)


def log_error(logger: LoggerAdapter, msg: Any, *, component: Optional[str] = None) -> None:
    if component:
        logger.error(f"[{component}] {_stringify_message(msg)}")
    else:
        logger.error(msg)


def log_exception(
    logger: LoggerAdapter,
    exc: BaseException,
    *,
    message: str = "Unhandled exception",
    component: Optional[str] = None,
    include_type: bool = True,
) -> None:
    """
    Emit a compact exception log message (without full traceback).

    Full traceback should be logged selectively in outer layers to avoid noisy
    high-rate loops.
    """
    exc_text = f"{exc.__class__.__name__}: {exc}" if include_type else str(exc)
    if component:
        logger.error(f"[{component}] {message} | {exc_text}")
    else:
        logger.error(f"{message} | {exc_text}")


# =============================================================================
# Rate-limited logging helper
# =============================================================================

@dataclass
class RateLimitedLogger:
    """
    Simple per-key rate limiter for repeated warnings/errors.

    Useful in real robot loops to avoid flooding logs when:
    - sensor/topic is stale
    - board writes fail repeatedly
    - optional dependency import keeps failing

    Example
    -------
    rl = RateLimitedLogger(get_logger_adapter(self), period_s=1.0)
    rl.warn("board_write_fail", "Motor board write failed")
    """
    logger: LoggerAdapter
    period_s: float = 1.0
    _last_emit_mono: Dict[str, float] = None  # initialized in __post_init__

    def __post_init__(self) -> None:
        if self._last_emit_mono is None:
            self._last_emit_mono = {}

    def _can_emit(self, key: str) -> bool:
        now = time.monotonic()
        last = self._last_emit_mono.get(str(key), 0.0)
        if (now - last) >= max(0.0, float(self.period_s)):
            self._last_emit_mono[str(key)] = now
            return True
        return False

    def debug(self, key: str, msg: Any) -> bool:
        if self._can_emit(key):
            self.logger.debug(msg)
            return True
        return False

    def info(self, key: str, msg: Any) -> bool:
        if self._can_emit(key):
            self.logger.info(msg)
            return True
        return False

    def warn(self, key: str, msg: Any) -> bool:
        if self._can_emit(key):
            self.logger.warn(msg)
            return True
        return False

    def error(self, key: str, msg: Any) -> bool:
        if self._can_emit(key):
            self.logger.error(msg)
            return True
        return False


# =============================================================================
# Public exports
# =============================================================================

__all__ = [
    "LoggerAdapter",
    "RateLimitedLogger",
    "get_logger_adapter",
    "format_kv",
    "format_event",
    "log_event",
    "log_debug",
    "log_info",
    "log_warn",
    "log_error",
    "log_exception",
]