#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/ros/params.py
------------------------------------
Professional ROS2 Jazzy parameter helpers for `savo_base`.

Purpose
-------
Centralize parameter declaration + reading logic used by `savo_base` nodes.

Why this module exists
----------------------
Without a shared params helper, each node repeats:
- declare_parameter(...)
- type conversions
- range clamping
- common topic names/defaults

This module keeps parameter handling:
- consistent
- readable
- safer for real robot testing
- easier to maintain as your `savo_base` grows

Scope
-----
This module does NOT:
- access hardware directly
- create publishers/subscribers
- run kinematics or motor output

It ONLY helps nodes declare/read/validate parameters.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from rclpy.node import Node


# =============================================================================
# Generic low-level helpers
# =============================================================================
def declare_if_missing(node: Node, name: str, default_value: Any) -> None:
    """
    Declare a parameter only if it has not already been declared.

    This helps avoid exceptions when multiple helper functions are used
    in the same node and some params overlap.
    """
    if not node.has_parameter(name):
        node.declare_parameter(name, default_value)


def get_param(node: Node, name: str, default: Any = None) -> Any:
    """
    Read a parameter value safely.
    Returns `default` if reading fails.
    """
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def get_str(node: Node, name: str, default: str = "") -> str:
    v = get_param(node, name, default)
    return str(v) if v is not None else str(default)


def get_bool(node: Node, name: str, default: bool = False) -> bool:
    v = get_param(node, name, default)
    return bool(v)


def get_int(node: Node, name: str, default: int = 0, *, min_value: Optional[int] = None, max_value: Optional[int] = None) -> int:
    try:
        out = int(get_param(node, name, default))
    except Exception:
        out = int(default)

    if min_value is not None and out < min_value:
        out = min_value
    if max_value is not None and out > max_value:
        out = max_value
    return out


def get_float(
    node: Node,
    name: str,
    default: float = 0.0,
    *,
    min_value: Optional[float] = None,
    max_value: Optional[float] = None,
) -> float:
    try:
        out = float(get_param(node, name, default))
    except Exception:
        out = float(default)

    if min_value is not None and out < min_value:
        out = min_value
    if max_value is not None and out > max_value:
        out = max_value
    return out


def get_str_list(node: Node, name: str, default: Optional[Sequence[str]] = None) -> List[str]:
    """
    Read a string-array parameter safely (or coerce a scalar to one-item list).
    """
    if default is None:
        default = []

    v = get_param(node, name, list(default))
    if v is None:
        return list(default)

    if isinstance(v, (list, tuple)):
        return [str(x) for x in v]
    return [str(v)]


# =============================================================================
# Common parameter declaration groups (shared across savo_base nodes)
# =============================================================================
def declare_common_node_params(node: Node, *, default_robot_name: str = "Robot Savo") -> None:
    """
    Declare common parameters used by most `savo_base` nodes.
    """
    declare_if_missing(node, "robot_name", default_robot_name)
    declare_if_missing(node, "pretty_json", False)
    declare_if_missing(node, "log_debug", False)


def declare_common_topic_params(node: Node) -> None:
    """
    Declare common `savo_base` topic parameters used across nodes.

    These defaults are intentionally centralized to reduce drift between nodes.
    """
    # Command/control path
    declare_if_missing(node, "cmd_vel_in_topic", "/cmd_vel_safe")
    declare_if_missing(node, "cmd_vel_raw_topic", "/cmd_vel")

    # Optional text-command / LLM integration hooks (base consumes motion commands, not voice)
    declare_if_missing(node, "base_command_text_topic", "/savo_base/command_text")

    # State / status
    declare_if_missing(node, "base_state_topic", "/savo_base/state")
    declare_if_missing(node, "motor_board_status_topic", "/savo_base/motor_board_status")
    declare_if_missing(node, "watchdog_state_topic", "/savo_base/watchdog_state")
    declare_if_missing(node, "heartbeat_topic", "/savo_base/heartbeat")

    # Watchdog/control flags
    declare_if_missing(node, "enable_topic", "/savo_base/enable")
    declare_if_missing(node, "estop_topic", "/savo_base/estop")
    declare_if_missing(node, "watchdog_trip_topic", "/savo_base/watchdog_trip")

    # Diagnostics orchestration topics
    declare_if_missing(node, "diag_run_request_topic", "/savo_base/diag/run_request")
    declare_if_missing(node, "diag_cancel_request_topic", "/savo_base/diag/cancel_request")
    declare_if_missing(node, "diag_state_topic", "/savo_base/diag/state")
    declare_if_missing(node, "diag_event_topic", "/savo_base/diag/event")
    declare_if_missing(node, "diag_busy_topic", "/savo_base/diag/busy")


def declare_base_limits_params(node: Node) -> None:
    """
    Declare base motion limit parameters (robot-command level, not wheel PWM level).
    """
    declare_if_missing(node, "limits.max_vx_mps", 0.35)
    declare_if_missing(node, "limits.max_vy_mps", 0.35)
    declare_if_missing(node, "limits.max_wz_radps", 1.20)

    declare_if_missing(node, "limits.max_ax_mps2", 0.80)
    declare_if_missing(node, "limits.max_ay_mps2", 0.80)
    declare_if_missing(node, "limits.max_awz_radps2", 2.00)

    # Deadbands at command level (useful for joystick / noisy command streams)
    declare_if_missing(node, "limits.deadband_vx", 0.01)
    declare_if_missing(node, "limits.deadband_vy", 0.01)
    declare_if_missing(node, "limits.deadband_wz", 0.02)


def declare_driver_hardware_params(node: Node) -> None:
    """
    Declare motor board / hardware driver parameters for `base_driver_node`.
    """
    # Driver selection
    declare_if_missing(node, "driver.board_type", "freenove_mecanum")  # or "dryrun"
    declare_if_missing(node, "driver.dry_run", False)

    # PCA9685 / I2C
    declare_if_missing(node, "driver.i2c_bus", 1)
    declare_if_missing(node, "driver.i2c_addr", 0x40)
    declare_if_missing(node, "driver.pwm_freq_hz", 50.0)
    declare_if_missing(node, "driver.max_duty", 3000)
    declare_if_missing(node, "driver.quench_ms", 18)

    # Wheel channel mapping (locked to your Robot Savo setup)
    declare_if_missing(node, "driver.channels.fl", [0, 1])
    declare_if_missing(node, "driver.channels.rl", [3, 2])
    declare_if_missing(node, "driver.channels.fr", [6, 7])
    declare_if_missing(node, "driver.channels.rr", [4, 5])

    # Wheel inversion flags
    declare_if_missing(node, "driver.invert.fl", False)
    declare_if_missing(node, "driver.invert.rl", False)
    declare_if_missing(node, "driver.invert.fr", False)
    declare_if_missing(node, "driver.invert.rr", False)

    # Kinematic sign conventions (robot command -> wheel mixing)
    declare_if_missing(node, "driver.forward_sign", -1)
    declare_if_missing(node, "driver.strafe_sign", 1)
    declare_if_missing(node, "driver.rotate_sign", 1)

    # Optional rotation scaling (Q/E feel in legacy teleop behavior equivalent)
    declare_if_missing(node, "driver.turn_gain", 1.0)

    # Driver loop
    declare_if_missing(node, "driver.loop_hz", 30.0)
    declare_if_missing(node, "driver.command_timeout_s", 0.50)

    # Safety on startup/shutdown
    declare_if_missing(node, "driver.stop_on_startup", True)
    declare_if_missing(node, "driver.stop_on_shutdown", True)


def declare_watchdog_params(node: Node) -> None:
    """
    Declare watchdog timing/behavior parameters for `base_watchdog_node`.
    """
    declare_if_missing(node, "watchdog.timeout_s", 0.50)
    declare_if_missing(node, "watchdog.publish_hz", 10.0)
    declare_if_missing(node, "watchdog.require_cmd_before_enable", False)
    declare_if_missing(node, "watchdog.auto_clear_on_new_cmd", True)


def declare_heartbeat_params(node: Node) -> None:
    """
    Declare heartbeat publisher parameters for `base_heartbeat_node`.
    """
    declare_if_missing(node, "heartbeat.publish_hz", 2.0)
    declare_if_missing(node, "heartbeat.frame_id", "base_link")
    declare_if_missing(node, "heartbeat.include_system_time", True)


def declare_state_publisher_params(node: Node) -> None:
    """
    Declare state-publisher timing parameters for `base_state_publisher_node`.
    """
    declare_if_missing(node, "state.publish_hz", 10.0)
    declare_if_missing(node, "state.include_debug_fields", True)


def declare_diag_runner_params(node: Node) -> None:
    """
    Declare diag runner parameters (shared defaults used by base_diag_runner_node).
    """
    declare_if_missing(node, "diag.state_publish_hz", 2.0)
    declare_if_missing(node, "diag.default_timeout_s", 20.0)
    declare_if_missing(node, "diag.max_timeout_s", 120.0)
    declare_if_missing(node, "diag.terminate_grace_s", 2.0)
    declare_if_missing(node, "diag.allow_parallel_runs", False)
    declare_if_missing(node, "diag.capture_output", True)
    declare_if_missing(node, "diag.max_output_chars", 4000)


def declare_all_base_node_common_params(node: Node) -> None:
    """
    Convenience helper to declare the most common shared parameters in one call.

    Use this at the top of `savo_base` nodes before reading params.
    """
    declare_common_node_params(node)
    declare_common_topic_params(node)


# =============================================================================
# Dataclass parameter bundles (clean node-side usage)
# =============================================================================
@dataclass
class CommonNodeParams:
    robot_name: str = "Robot Savo"
    pretty_json: bool = False
    log_debug: bool = False


@dataclass
class CommonTopicParams:
    cmd_vel_in_topic: str = "/cmd_vel_safe"
    cmd_vel_raw_topic: str = "/cmd_vel"
    base_command_text_topic: str = "/savo_base/command_text"

    base_state_topic: str = "/savo_base/state"
    motor_board_status_topic: str = "/savo_base/motor_board_status"
    watchdog_state_topic: str = "/savo_base/watchdog_state"
    heartbeat_topic: str = "/savo_base/heartbeat"

    enable_topic: str = "/savo_base/enable"
    estop_topic: str = "/savo_base/estop"
    watchdog_trip_topic: str = "/savo_base/watchdog_trip"

    diag_run_request_topic: str = "/savo_base/diag/run_request"
    diag_cancel_request_topic: str = "/savo_base/diag/cancel_request"
    diag_state_topic: str = "/savo_base/diag/state"
    diag_event_topic: str = "/savo_base/diag/event"
    diag_busy_topic: str = "/savo_base/diag/busy"


@dataclass
class BaseLimitsParams:
    max_vx_mps: float = 0.35
    max_vy_mps: float = 0.35
    max_wz_radps: float = 1.20

    max_ax_mps2: float = 0.80
    max_ay_mps2: float = 0.80
    max_awz_radps2: float = 2.00

    deadband_vx: float = 0.01
    deadband_vy: float = 0.01
    deadband_wz: float = 0.02


@dataclass
class DriverHardwareParams:
    board_type: str = "freenove_mecanum"
    dry_run: bool = False

    i2c_bus: int = 1
    i2c_addr: int = 0x40
    pwm_freq_hz: float = 50.0
    max_duty: int = 3000
    quench_ms: int = 18

    channels_fl: Tuple[int, int] = (0, 1)
    channels_rl: Tuple[int, int] = (3, 2)
    channels_fr: Tuple[int, int] = (6, 7)
    channels_rr: Tuple[int, int] = (4, 5)

    invert_fl: bool = False
    invert_rl: bool = False
    invert_fr: bool = False
    invert_rr: bool = False

    forward_sign: int = -1
    strafe_sign: int = 1
    rotate_sign: int = 1
    turn_gain: float = 1.0

    loop_hz: float = 30.0
    command_timeout_s: float = 0.50

    stop_on_startup: bool = True
    stop_on_shutdown: bool = True


@dataclass
class WatchdogParams:
    timeout_s: float = 0.50
    publish_hz: float = 10.0
    require_cmd_before_enable: bool = False
    auto_clear_on_new_cmd: bool = True


@dataclass
class HeartbeatParams:
    publish_hz: float = 2.0
    frame_id: str = "base_link"
    include_system_time: bool = True


@dataclass
class StatePublisherParams:
    publish_hz: float = 10.0
    include_debug_fields: bool = True


@dataclass
class DiagRunnerParams:
    state_publish_hz: float = 2.0
    default_timeout_s: float = 20.0
    max_timeout_s: float = 120.0
    terminate_grace_s: float = 2.0
    allow_parallel_runs: bool = False
    capture_output: bool = True
    max_output_chars: int = 4000


# =============================================================================
# Readers for dataclass bundles
# =============================================================================
def read_common_node_params(node: Node) -> CommonNodeParams:
    declare_common_node_params(node)
    return CommonNodeParams(
        robot_name=get_str(node, "robot_name", "Robot Savo"),
        pretty_json=get_bool(node, "pretty_json", False),
        log_debug=get_bool(node, "log_debug", False),
    )


def read_common_topic_params(node: Node) -> CommonTopicParams:
    declare_common_topic_params(node)
    return CommonTopicParams(
        cmd_vel_in_topic=get_str(node, "cmd_vel_in_topic", "/cmd_vel_safe"),
        cmd_vel_raw_topic=get_str(node, "cmd_vel_raw_topic", "/cmd_vel"),
        base_command_text_topic=get_str(node, "base_command_text_topic", "/savo_base/command_text"),

        base_state_topic=get_str(node, "base_state_topic", "/savo_base/state"),
        motor_board_status_topic=get_str(node, "motor_board_status_topic", "/savo_base/motor_board_status"),
        watchdog_state_topic=get_str(node, "watchdog_state_topic", "/savo_base/watchdog_state"),
        heartbeat_topic=get_str(node, "heartbeat_topic", "/savo_base/heartbeat"),

        enable_topic=get_str(node, "enable_topic", "/savo_base/enable"),
        estop_topic=get_str(node, "estop_topic", "/savo_base/estop"),
        watchdog_trip_topic=get_str(node, "watchdog_trip_topic", "/savo_base/watchdog_trip"),

        diag_run_request_topic=get_str(node, "diag_run_request_topic", "/savo_base/diag/run_request"),
        diag_cancel_request_topic=get_str(node, "diag_cancel_request_topic", "/savo_base/diag/cancel_request"),
        diag_state_topic=get_str(node, "diag_state_topic", "/savo_base/diag/state"),
        diag_event_topic=get_str(node, "diag_event_topic", "/savo_base/diag/event"),
        diag_busy_topic=get_str(node, "diag_busy_topic", "/savo_base/diag/busy"),
    )


def read_base_limits_params(node: Node) -> BaseLimitsParams:
    declare_base_limits_params(node)
    return BaseLimitsParams(
        max_vx_mps=get_float(node, "limits.max_vx_mps", 0.35, min_value=0.01),
        max_vy_mps=get_float(node, "limits.max_vy_mps", 0.35, min_value=0.01),
        max_wz_radps=get_float(node, "limits.max_wz_radps", 1.20, min_value=0.05),

        max_ax_mps2=get_float(node, "limits.max_ax_mps2", 0.80, min_value=0.01),
        max_ay_mps2=get_float(node, "limits.max_ay_mps2", 0.80, min_value=0.01),
        max_awz_radps2=get_float(node, "limits.max_awz_radps2", 2.00, min_value=0.01),

        deadband_vx=get_float(node, "limits.deadband_vx", 0.01, min_value=0.0),
        deadband_vy=get_float(node, "limits.deadband_vy", 0.01, min_value=0.0),
        deadband_wz=get_float(node, "limits.deadband_wz", 0.02, min_value=0.0),
    )


def _read_channel_pair(node: Node, param_name: str, default_pair: Tuple[int, int]) -> Tuple[int, int]:
    vals = get_param(node, param_name, list(default_pair))
    try:
        if isinstance(vals, (list, tuple)) and len(vals) == 2:
            a = int(vals[0])
            b = int(vals[1])
            return (a, b)
    except Exception:
        pass
    return default_pair


def read_driver_hardware_params(node: Node) -> DriverHardwareParams:
    declare_driver_hardware_params(node)

    forward_sign = get_int(node, "driver.forward_sign", -1)
    strafe_sign = get_int(node, "driver.strafe_sign", 1)
    rotate_sign = get_int(node, "driver.rotate_sign", 1)

    # Enforce sign-only values
    forward_sign = -1 if forward_sign < 0 else 1
    strafe_sign = -1 if strafe_sign < 0 else 1
    rotate_sign = -1 if rotate_sign < 0 else 1

    return DriverHardwareParams(
        board_type=get_str(node, "driver.board_type", "freenove_mecanum"),
        dry_run=get_bool(node, "driver.dry_run", False),

        i2c_bus=get_int(node, "driver.i2c_bus", 1, min_value=0),
        i2c_addr=get_int(node, "driver.i2c_addr", 0x40, min_value=0x03, max_value=0x77),
        pwm_freq_hz=get_float(node, "driver.pwm_freq_hz", 50.0, min_value=10.0, max_value=2000.0),
        max_duty=get_int(node, "driver.max_duty", 3000, min_value=0, max_value=4095),
        quench_ms=get_int(node, "driver.quench_ms", 18, min_value=0, max_value=1000),

        channels_fl=_read_channel_pair(node, "driver.channels.fl", (0, 1)),
        channels_rl=_read_channel_pair(node, "driver.channels.rl", (3, 2)),
        channels_fr=_read_channel_pair(node, "driver.channels.fr", (6, 7)),
        channels_rr=_read_channel_pair(node, "driver.channels.rr", (4, 5)),

        invert_fl=get_bool(node, "driver.invert.fl", False),
        invert_rl=get_bool(node, "driver.invert.rl", False),
        invert_fr=get_bool(node, "driver.invert.fr", False),
        invert_rr=get_bool(node, "driver.invert.rr", False),

        forward_sign=forward_sign,
        strafe_sign=strafe_sign,
        rotate_sign=rotate_sign,
        turn_gain=get_float(node, "driver.turn_gain", 1.0, min_value=0.01, max_value=5.0),

        loop_hz=get_float(node, "driver.loop_hz", 30.0, min_value=5.0, max_value=500.0),
        command_timeout_s=get_float(node, "driver.command_timeout_s", 0.50, min_value=0.05, max_value=10.0),

        stop_on_startup=get_bool(node, "driver.stop_on_startup", True),
        stop_on_shutdown=get_bool(node, "driver.stop_on_shutdown", True),
    )


def read_watchdog_params(node: Node) -> WatchdogParams:
    declare_watchdog_params(node)
    return WatchdogParams(
        timeout_s=get_float(node, "watchdog.timeout_s", 0.50, min_value=0.05, max_value=10.0),
        publish_hz=get_float(node, "watchdog.publish_hz", 10.0, min_value=0.5, max_value=200.0),
        require_cmd_before_enable=get_bool(node, "watchdog.require_cmd_before_enable", False),
        auto_clear_on_new_cmd=get_bool(node, "watchdog.auto_clear_on_new_cmd", True),
    )


def read_heartbeat_params(node: Node) -> HeartbeatParams:
    declare_heartbeat_params(node)
    return HeartbeatParams(
        publish_hz=get_float(node, "heartbeat.publish_hz", 2.0, min_value=0.2, max_value=50.0),
        frame_id=get_str(node, "heartbeat.frame_id", "base_link"),
        include_system_time=get_bool(node, "heartbeat.include_system_time", True),
    )


def read_state_publisher_params(node: Node) -> StatePublisherParams:
    declare_state_publisher_params(node)
    return StatePublisherParams(
        publish_hz=get_float(node, "state.publish_hz", 10.0, min_value=0.5, max_value=200.0),
        include_debug_fields=get_bool(node, "state.include_debug_fields", True),
    )


def read_diag_runner_params(node: Node) -> DiagRunnerParams:
    declare_diag_runner_params(node)
    default_timeout_s = get_float(node, "diag.default_timeout_s", 20.0, min_value=1.0, max_value=600.0)
    max_timeout_s = get_float(node, "diag.max_timeout_s", 120.0, min_value=default_timeout_s, max_value=3600.0)
    return DiagRunnerParams(
        state_publish_hz=get_float(node, "diag.state_publish_hz", 2.0, min_value=0.5, max_value=50.0),
        default_timeout_s=default_timeout_s,
        max_timeout_s=max_timeout_s,
        terminate_grace_s=get_float(node, "diag.terminate_grace_s", 2.0, min_value=0.1, max_value=30.0),
        allow_parallel_runs=get_bool(node, "diag.allow_parallel_runs", False),
        capture_output=get_bool(node, "diag.capture_output", True),
        max_output_chars=get_int(node, "diag.max_output_chars", 4000, min_value=128, max_value=200000),
    )


# =============================================================================
# Debug helpers (optional, useful in logs)
# =============================================================================
def dataclass_params_to_dict(params_obj: Any) -> Dict[str, Any]:
    """
    Convert any params dataclass bundle to dict for logging/state publishing.
    """
    if hasattr(params_obj, "__dataclass_fields__"):
        return asdict(params_obj)
    raise TypeError(f"Expected dataclass params object, got {type(params_obj).__name__}")


def summarize_common_params_for_log(
    common: CommonNodeParams,
    topics: CommonTopicParams,
) -> str:
    """
    Compact one-line summary for startup logs.
    """
    return (
        f"robot_name={common.robot_name}, "
        f"pretty_json={common.pretty_json}, "
        f"cmd_vel_in={topics.cmd_vel_in_topic}, "
        f"base_state={topics.base_state_topic}"
    )


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # low-level helpers
    "declare_if_missing",
    "get_param",
    "get_str",
    "get_bool",
    "get_int",
    "get_float",
    "get_str_list",
    # declarations
    "declare_common_node_params",
    "declare_common_topic_params",
    "declare_base_limits_params",
    "declare_driver_hardware_params",
    "declare_watchdog_params",
    "declare_heartbeat_params",
    "declare_state_publisher_params",
    "declare_diag_runner_params",
    "declare_all_base_node_common_params",
    # dataclasses
    "CommonNodeParams",
    "CommonTopicParams",
    "BaseLimitsParams",
    "DriverHardwareParams",
    "WatchdogParams",
    "HeartbeatParams",
    "StatePublisherParams",
    "DiagRunnerParams",
    # readers
    "read_common_node_params",
    "read_common_topic_params",
    "read_base_limits_params",
    "read_driver_hardware_params",
    "read_watchdog_params",
    "read_heartbeat_params",
    "read_state_publisher_params",
    "read_diag_runner_params",
    # debug
    "dataclass_params_to_dict",
    "summarize_common_params_for_log",
]