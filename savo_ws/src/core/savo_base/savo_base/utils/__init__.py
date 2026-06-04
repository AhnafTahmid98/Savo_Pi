# -*- coding: utf-8 -*-
"""Shared utility helpers for savo_base: clamp, timing, logging, params, QoS, topic names."""

from __future__ import annotations

from .clamp import (
    clamp,
    clamp_int,
    clamp_float,
    clamp01,
    clamp_symmetric,
    clamp_tuple4,
    clamp_iterable,
    clamp_signed_duty,
    clamp_unsigned_duty,
)

from .diagnostics import (
    make_diag_payload,
    make_error_diag,
    dumps_compact,
    dumps_pretty,
    now_mono,
    now_wall,
    age_s,
    normalize_status_level,
    worst_status,
    exception_message,
    exception_trace,
)

from .logging import (
    get_logger_adapter,
    log_info,
    log_warn,
    log_error,
    log_debug,
    log_exception,
)

__all__ = [
    # clamp
    "clamp",
    "clamp_int",
    "clamp_float",
    "clamp01",
    "clamp_symmetric",
    "clamp_tuple4",
    "clamp_iterable",
    "clamp_signed_duty",
    "clamp_unsigned_duty",

    # diagnostics
    "make_diag_payload",
    "make_error_diag",
    "dumps_compact",
    "dumps_pretty",
    "now_mono",
    "now_wall",
    "age_s",
    "normalize_status_level",
    "worst_status",
    "exception_message",
    "exception_trace",

    # logging
    "get_logger_adapter",
    "log_info",
    "log_warn",
    "log_error",
    "log_debug",
    "log_exception",
]