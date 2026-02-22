# -*- coding: utf-8 -*-
"""
Robot SAVO — savo_base/utils/__init__.py
----------------------------------------
Utility package exports for `savo_base`.

Purpose
-------
Provides stable import paths for shared helpers used across nodes, drivers,
and safety modules.

Example
-------
    from savo_base.utils import clamp, now_mono_s, build_json_diag, DEFAULT_QOS_DEPTH
    from savo_base.utils import topic_names
"""

from __future__ import annotations

# -----------------------------------------------------------------------------
# Clamp helpers
# -----------------------------------------------------------------------------
try:
    from .clamp import (
        clamp,
        clamp_int,
        clamp_float,
        clamp_abs,
        saturate,
        clip_tuple,
    )
except Exception:
    # Keep package import robust during staged development
    clamp = None
    clamp_int = None
    clamp_float = None
    clamp_abs = None
    saturate = None
    clip_tuple = None


# -----------------------------------------------------------------------------
# Diagnostics helpers
# -----------------------------------------------------------------------------
try:
    from .diagnostics import (
        build_json_diag,
        build_status_summary,
        safe_json_dumps,
        utc_timestamp_iso,
    )
except Exception:
    build_json_diag = None
    build_status_summary = None
    safe_json_dumps = None
    utc_timestamp_iso = None


# -----------------------------------------------------------------------------
# Logging helpers
# -----------------------------------------------------------------------------
try:
    from .logging import (
        get_logger_name,
        throttle_warn,
        throttle_info,
        exc_to_str,
    )
except Exception:
    get_logger_name = None
    throttle_warn = None
    throttle_info = None
    exc_to_str = None


# -----------------------------------------------------------------------------
# Parameter helpers
# -----------------------------------------------------------------------------
try:
    from .param_loader import (
        get_param,
        get_param_str,
        get_param_bool,
        get_param_int,
        get_param_float,
        get_param_list,
        parse_int_auto_base,
    )
except Exception:
    get_param = None
    get_param_str = None
    get_param_bool = None
    get_param_int = None
    get_param_float = None
    get_param_list = None
    parse_int_auto_base = None


# -----------------------------------------------------------------------------
# QoS helpers/constants
# -----------------------------------------------------------------------------
try:
    from .qos import (
        make_qos_reliable,
        make_qos_best_effort,
        make_qos_sensor,
        make_qos_command,
        DEFAULT_QOS_DEPTH,
    )
except Exception:
    make_qos_reliable = None
    make_qos_best_effort = None
    make_qos_sensor = None
    make_qos_command = None
    DEFAULT_QOS_DEPTH = None


# -----------------------------------------------------------------------------
# Rate / timing helpers
# -----------------------------------------------------------------------------
try:
    from .ratekeeper import Ratekeeper
except Exception:
    Ratekeeper = None

try:
    from .timing import (
        now_mono_s,
        now_wall_s,
        elapsed_s,
        hz_to_period_s,
    )
except Exception:
    now_mono_s = None
    now_wall_s = None
    elapsed_s = None
    hz_to_period_s = None


# -----------------------------------------------------------------------------
# Topic names module (export as module + selected constants)
# -----------------------------------------------------------------------------
try:
    from . import topic_names
except Exception:
    topic_names = None

try:
    from .topic_names import (
        CMD_VEL,
        CMD_VEL_SAFE,
        SAFETY_STOP,
        SAFETY_SLOWDOWN_FACTOR,
        SAVO_BASE_WATCHDOG_STATE,
        SAVO_BASE_BASE_STATE,
        SAVO_BASE_HEARTBEAT,
        SAVO_BASE_DIAG,
        SAVO_BASE_MOTOR_STATUS,
        BASE_DRIVER_TOPICS,
        BASE_STATUS_TOPICS,
        SAFETY_TOPICS,
        DEFAULT_BASE_DRIVER_TOPIC_MAP,
        DEFAULT_SAFETY_GATE_TOPIC_MAP,
        normalize_topic_name,
        join_topic,
        topic_in_namespace,
    )
except Exception:
    CMD_VEL = None
    CMD_VEL_SAFE = None
    SAFETY_STOP = None
    SAFETY_SLOWDOWN_FACTOR = None
    SAVO_BASE_WATCHDOG_STATE = None
    SAVO_BASE_BASE_STATE = None
    SAVO_BASE_HEARTBEAT = None
    SAVO_BASE_DIAG = None
    SAVO_BASE_MOTOR_STATUS = None
    BASE_DRIVER_TOPICS = None
    BASE_STATUS_TOPICS = None
    SAFETY_TOPICS = None
    DEFAULT_BASE_DRIVER_TOPIC_MAP = None
    DEFAULT_SAFETY_GATE_TOPIC_MAP = None
    normalize_topic_name = None
    join_topic = None
    topic_in_namespace = None


__all__ = [
    # clamp
    "clamp",
    "clamp_int",
    "clamp_float",
    "clamp_abs",
    "saturate",
    "clip_tuple",
    # diagnostics
    "build_json_diag",
    "build_status_summary",
    "safe_json_dumps",
    "utc_timestamp_iso",
    # logging
    "get_logger_name",
    "throttle_warn",
    "throttle_info",
    "exc_to_str",
    # params
    "get_param",
    "get_param_str",
    "get_param_bool",
    "get_param_int",
    "get_param_float",
    "get_param_list",
    "parse_int_auto_base",
    # qos
    "make_qos_reliable",
    "make_qos_best_effort",
    "make_qos_sensor",
    "make_qos_command",
    "DEFAULT_QOS_DEPTH",
    # rate/timing
    "Ratekeeper",
    "now_mono_s",
    "now_wall_s",
    "elapsed_s",
    "hz_to_period_s",
    # topic names (module + selected exports)
    "topic_names",
    "CMD_VEL",
    "CMD_VEL_SAFE",
    "SAFETY_STOP",
    "SAFETY_SLOWDOWN_FACTOR",
    "SAVO_BASE_WATCHDOG_STATE",
    "SAVO_BASE_BASE_STATE",
    "SAVO_BASE_HEARTBEAT",
    "SAVO_BASE_DIAG",
    "SAVO_BASE_MOTOR_STATUS",
    "BASE_DRIVER_TOPICS",
    "BASE_STATUS_TOPICS",
    "SAFETY_TOPICS",
    "DEFAULT_BASE_DRIVER_TOPIC_MAP",
    "DEFAULT_SAFETY_GATE_TOPIC_MAP",
    "normalize_topic_name",
    "join_topic",
    "topic_in_namespace",
]