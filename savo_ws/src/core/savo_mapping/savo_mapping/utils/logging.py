#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Logging helpers for Robot Savo mapping. No ROS imports."""

from __future__ import annotations

import logging
import sys
from dataclasses import dataclass
from typing import Optional


# =============================================================================
# Defaults
# =============================================================================
DEFAULT_LOG_FORMAT = "[%(levelname)s] [%(name)s] %(message)s"
DEFAULT_DATE_FORMAT = "%H:%M:%S"


# =============================================================================
# Logging config
# =============================================================================
@dataclass(frozen=True)
class LoggerConfig:
    name: str = "savo_mapping"
    level: str = "INFO"
    use_timestamp: bool = False

    def numeric_level(self) -> int:
        value = str(self.level).strip().upper()

        if not value:
            return logging.INFO

        return int(getattr(logging, value, logging.INFO))


def normalize_log_level(level: str | int) -> int:
    if isinstance(level, int):
        return level

    value = str(level).strip().upper()

    if not value:
        return logging.INFO

    return int(getattr(logging, value, logging.INFO))


def make_log_format(use_timestamp: bool = False) -> str:
    if use_timestamp:
        return "[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s"

    return DEFAULT_LOG_FORMAT


def configure_logger(
    name: str = "savo_mapping",
    level: str | int = "INFO",
    use_timestamp: bool = False,
    stream: Optional[object] = None,
) -> logging.Logger:
    logger = logging.getLogger(str(name))
    logger.setLevel(normalize_log_level(level))
    logger.propagate = False

    if logger.handlers:
        for handler in logger.handlers:
            handler.setLevel(normalize_log_level(level))
        return logger

    handler = logging.StreamHandler(stream or sys.stdout)
    handler.setLevel(normalize_log_level(level))

    formatter = logging.Formatter(
        fmt=make_log_format(use_timestamp=use_timestamp),
        datefmt=DEFAULT_DATE_FORMAT,
    )
    handler.setFormatter(formatter)

    logger.addHandler(handler)
    return logger


def get_logger(
    name: str = "savo_mapping",
    level: str | int = "INFO",
) -> logging.Logger:
    return configure_logger(name=name, level=level)


def set_logger_level(
    logger: logging.Logger,
    level: str | int,
) -> None:
    numeric = normalize_log_level(level)
    logger.setLevel(numeric)

    for handler in logger.handlers:
        handler.setLevel(numeric)


# =============================================================================
# Message helpers
# =============================================================================
def log_key_value(
    logger: logging.Logger,
    key: str,
    value: object,
    level: str | int = "INFO",
) -> None:
    logger.log(
        normalize_log_level(level),
        "%s=%s",
        str(key),
        value,
    )


def log_section(
    logger: logging.Logger,
    title: str,
    level: str | int = "INFO",
) -> None:
    logger.log(
        normalize_log_level(level),
        "---- %s ----",
        str(title),
    )


def log_status_dict(
    logger: logging.Logger,
    status: dict,
    level: str | int = "INFO",
) -> None:
    numeric = normalize_log_level(level)

    for key in sorted(status.keys()):
        logger.log(numeric, "%s=%s", key, status[key])


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    logger = configure_logger("savo_mapping.demo", level="DEBUG")

    logger.info("Logger ready.")
    logger.debug("Debug output enabled.")
    log_key_value(logger, "mode", "manual_mapping")
    log_section(logger, "status")
    log_status_dict(logger, {"ready": True, "degraded": False})


if __name__ == "__main__":
    main()


__all__ = [
    "DEFAULT_LOG_FORMAT",
    "DEFAULT_DATE_FORMAT",
    "LoggerConfig",
    "normalize_log_level",
    "make_log_format",
    "configure_logger",
    "get_logger",
    "set_logger_level",
    "log_key_value",
    "log_section",
    "log_status_dict",
    "main",
]