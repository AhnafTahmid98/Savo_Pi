#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot SAVO — savo_base/ros/topic_contract.py
--------------------------------------------
Professional ROS2 Jazzy topic contract definitions for `savo_base`.

Purpose
-------
Single source of truth for:
- canonical topic names
- expected ROS message types
- direction (pub/sub)
- role/meaning in the base stack

Why this module exists
----------------------
As the robot grows (base, perception, localization, control, nav, speech, LLM),
topic-name drift becomes a real integration risk. This file helps keep the
`savo_base` package consistent and integration-friendly.

Design principles
-----------------
- Stable canonical defaults (can still be remapped in launch)
- Human-readable contract metadata
- Zero hardware access
- Zero ROS node startup side effects
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, Iterable, List, Optional, Tuple, Type

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


# =============================================================================
# Topic spec datamodel
# =============================================================================
@dataclass(frozen=True)
class TopicSpec:
    """
    Contract entry for one ROS topic in `savo_base`.

    Fields
    ------
    name : str
        Canonical topic name.
    msg_type : type
        ROS message class (e.g., std_msgs.msg.Bool).
    direction : str
        Expected usage from the perspective of a typical `savo_base` node group.
        One of: 'sub', 'pub', 'pubsub'
    description : str
        Human-readable purpose.
    required : bool
        Whether the topic is considered required for normal base bringup.
    qos_key : str
        Symbolic key matching `savo_base.ros.qos_profiles` registry/factory intent.
    """
    name: str
    msg_type: Type
    direction: str
    description: str
    required: bool = True
    qos_key: str = "state_string"

    @property
    def msg_type_name(self) -> str:
        return _ros_msg_type_name(self.msg_type)


def _ros_msg_type_name(msg_type: Type) -> str:
    """
    Convert ROS message class to a readable 'pkg/msg/Type' string when possible.
    """
    module = getattr(msg_type, "__module__", "")
    name = getattr(msg_type, "__name__", str(msg_type))

    # Example module: "std_msgs.msg._bool"
    if ".msg" in module:
        pkg = module.split(".")[0]
        return f"{pkg}/msg/{name}"
    return name


def topic_spec_to_dict(spec: TopicSpec) -> Dict[str, object]:
    """
    JSON/log-friendly dict representation of TopicSpec.
    """
    d = asdict(spec)
    d["msg_type"] = spec.msg_type_name
    return d


# =============================================================================
# Canonical topic names (single source of truth)
# =============================================================================
# Command input path
CMD_VEL_RAW_TOPIC = "/cmd_vel"
CMD_VEL_SAFE_TOPIC = "/cmd_vel_safe"

# Optional text command hook (LLM / speech / server integration can publish text here)
BASE_COMMAND_TEXT_TOPIC = "/savo_base/command_text"

# Base state/status outputs
BASE_STATE_TOPIC = "/savo_base/state"
MOTOR_BOARD_STATUS_TOPIC = "/savo_base/motor_board_status"
WATCHDOG_STATE_TOPIC = "/savo_base/watchdog_state"
HEARTBEAT_TOPIC = "/savo_base/heartbeat"

# Enable / E-stop / watchdog control flags
BASE_ENABLE_TOPIC = "/savo_base/enable"
BASE_ESTOP_TOPIC = "/savo_base/estop"
WATCHDOG_TRIP_TOPIC = "/savo_base/watchdog_trip"

# Diagnostics orchestration
DIAG_RUN_REQUEST_TOPIC = "/savo_base/diag/run_request"
DIAG_CANCEL_REQUEST_TOPIC = "/savo_base/diag/cancel_request"
DIAG_STATE_TOPIC = "/savo_base/diag/state"
DIAG_EVENT_TOPIC = "/savo_base/diag/event"
DIAG_BUSY_TOPIC = "/savo_base/diag/busy"


# =============================================================================
# Topic contract registry (canonical defaults)
# =============================================================================
TOPIC_CONTRACT: Dict[str, TopicSpec] = {
    # -------------------------------------------------------------------------
    # Command/control path
    # -------------------------------------------------------------------------
    "cmd_vel_raw": TopicSpec(
        name=CMD_VEL_RAW_TOPIC,
        msg_type=Twist,
        direction="sub",
        description="Raw robot velocity command (pre-safety). Normally not consumed directly by base driver in production.",
        required=False,
        qos_key="cmd_vel",
    ),
    "cmd_vel_safe": TopicSpec(
        name=CMD_VEL_SAFE_TOPIC,
        msg_type=Twist,
        direction="sub",
        description="Safety-gated robot velocity command consumed by base driver.",
        required=True,
        qos_key="cmd_vel",
    ),
    "base_command_text": TopicSpec(
        name=BASE_COMMAND_TEXT_TOPIC,
        msg_type=String,
        direction="sub",
        description="Optional text command input for orchestration/debug (e.g., LLM/speech adapter). Base motion still executes via cmd_vel topics.",
        required=False,
        qos_key="state_string",
    ),

    # -------------------------------------------------------------------------
    # Base state / status outputs
    # -------------------------------------------------------------------------
    "base_state": TopicSpec(
        name=BASE_STATE_TOPIC,
        msg_type=String,
        direction="pub",
        description="Base runtime state JSON (mode, enable, estop, watchdog, command summary).",
        required=True,
        qos_key="state_string",
    ),
    "motor_board_status": TopicSpec(
        name=MOTOR_BOARD_STATUS_TOPIC,
        msg_type=String,
        direction="pub",
        description="Motor board/driver status JSON (connected, dry-run, bus/address, errors).",
        required=True,
        qos_key="state_string",
    ),
    "watchdog_state": TopicSpec(
        name=WATCHDOG_STATE_TOPIC,
        msg_type=String,
        direction="pub",
        description="Watchdog runtime state JSON (tripped, reason, command age, timeout).",
        required=True,
        qos_key="state_string",
    ),
    "heartbeat": TopicSpec(
        name=HEARTBEAT_TOPIC,
        msg_type=String,
        direction="pub",
        description="Periodic heartbeat JSON for supervision and bringup diagnostics.",
        required=True,
        qos_key="heartbeat",
    ),

    # -------------------------------------------------------------------------
    # Control flags (pub/sub depending on node role)
    # -------------------------------------------------------------------------
    "base_enable": TopicSpec(
        name=BASE_ENABLE_TOPIC,
        msg_type=Bool,
        direction="pubsub",
        description="Enable/disable base motor output path.",
        required=False,
        qos_key="safety_bool",
    ),
    "base_estop": TopicSpec(
        name=BASE_ESTOP_TOPIC,
        msg_type=Bool,
        direction="pubsub",
        description="Emergency stop flag. When true, base output must be forced to stop.",
        required=False,
        qos_key="safety_bool",
    ),
    "watchdog_trip": TopicSpec(
        name=WATCHDOG_TRIP_TOPIC,
        msg_type=Bool,
        direction="pubsub",
        description="Watchdog trip flag indicating command timeout or supervision failure.",
        required=True,
        qos_key="safety_bool",
    ),

    # -------------------------------------------------------------------------
    # Diagnostics orchestration
    # -------------------------------------------------------------------------
    "diag_run_request": TopicSpec(
        name=DIAG_RUN_REQUEST_TOPIC,
        msg_type=String,
        direction="sub",
        description="Diagnostic run request JSON (tool, args, timeout, tag).",
        required=False,
        qos_key="diag_state",
    ),
    "diag_cancel_request": TopicSpec(
        name=DIAG_CANCEL_REQUEST_TOPIC,
        msg_type=String,
        direction="sub",
        description="Diagnostic cancel request JSON.",
        required=False,
        qos_key="diag_state",
    ),
    "diag_state": TopicSpec(
        name=DIAG_STATE_TOPIC,
        msg_type=String,
        direction="pub",
        description="Diagnostic runner state JSON (idle/running/finished/error).",
        required=False,
        qos_key="diag_state",
    ),
    "diag_event": TopicSpec(
        name=DIAG_EVENT_TOPIC,
        msg_type=String,
        direction="pub",
        description="Diagnostic event/log stream JSON (stdout/stderr/progress/events).",
        required=False,
        qos_key="diag_event",
    ),
    "diag_busy": TopicSpec(
        name=DIAG_BUSY_TOPIC,
        msg_type=Bool,
        direction="pub",
        description="Boolean status indicating whether diag runner is currently busy.",
        required=False,
        qos_key="diag_state",
    ),
}


# =============================================================================
# Contract query helpers
# =============================================================================
def get_topic_spec(key: str) -> TopicSpec:
    """
    Get a TopicSpec by symbolic contract key.

    Raises
    ------
    KeyError if the key is unknown.
    """
    return TOPIC_CONTRACT[key]


def has_topic_spec(key: str) -> bool:
    return key in TOPIC_CONTRACT


def get_topic_name(key: str) -> str:
    return get_topic_spec(key).name


def get_topic_msg_type(key: str) -> Type:
    return get_topic_spec(key).msg_type


def get_topic_msg_type_name(key: str) -> str:
    return get_topic_spec(key).msg_type_name


def get_topic_qos_key(key: str) -> str:
    return get_topic_spec(key).qos_key


def list_topic_keys(*, required_only: bool = False) -> List[str]:
    keys = list(TOPIC_CONTRACT.keys())
    if required_only:
        keys = [k for k in keys if TOPIC_CONTRACT[k].required]
    return keys


def list_topic_specs(*, required_only: bool = False) -> List[TopicSpec]:
    specs = list(TOPIC_CONTRACT.values())
    if required_only:
        specs = [s for s in specs if s.required]
    return specs


def topic_contract_to_dict(*, required_only: bool = False) -> Dict[str, Dict[str, object]]:
    """
    Export contract registry as a JSON/log-friendly dict.
    """
    out: Dict[str, Dict[str, object]] = {}
    for key, spec in TOPIC_CONTRACT.items():
        if required_only and not spec.required:
            continue
        out[key] = topic_spec_to_dict(spec)
    return out


# =============================================================================
# Validation helpers for startup checks
# =============================================================================
@dataclass(frozen=True)
class TopicBindingIssue:
    key: str
    expected_name: str
    actual_name: str
    reason: str


def validate_topic_bindings(bindings: Dict[str, str]) -> List[TopicBindingIssue]:
    """
    Validate a mapping of symbolic topic keys -> topic names.

    Useful at node startup if you want to compare remapped/parameterized names
    against the canonical contract and print informative warnings.

    This does NOT reject remapping (remapping is valid in ROS2). It just reports
    differences so teams can see what changed.
    """
    issues: List[TopicBindingIssue] = []

    for key, spec in TOPIC_CONTRACT.items():
        if key not in bindings:
            continue
        actual = str(bindings[key])
        if actual != spec.name:
            issues.append(
                TopicBindingIssue(
                    key=key,
                    expected_name=spec.name,
                    actual_name=actual,
                    reason="non-canonical remap/name override",
                )
            )

    return issues


def summarize_binding_issues(issues: List[TopicBindingIssue]) -> str:
    """
    Build a compact human-readable summary for logs.
    """
    if not issues:
        return "All topic bindings match canonical savo_base contract."

    parts = []
    for it in issues:
        parts.append(f"{it.key}: '{it.actual_name}' (canon: '{it.expected_name}')")
    return "Topic binding overrides detected -> " + "; ".join(parts)


# =============================================================================
# Group helpers (useful in docs/bringup checks)
# =============================================================================
def command_path_keys() -> Tuple[str, ...]:
    return ("cmd_vel_raw", "cmd_vel_safe", "base_command_text")


def state_output_keys() -> Tuple[str, ...]:
    return ("base_state", "motor_board_status", "watchdog_state", "heartbeat")


def control_flag_keys() -> Tuple[str, ...]:
    return ("base_enable", "base_estop", "watchdog_trip")


def diag_keys() -> Tuple[str, ...]:
    return (
        "diag_run_request",
        "diag_cancel_request",
        "diag_state",
        "diag_event",
        "diag_busy",
    )


# =============================================================================
# Public exports
# =============================================================================
__all__ = [
    # dataclasses
    "TopicSpec",
    "TopicBindingIssue",
    # constants (topic names)
    "CMD_VEL_RAW_TOPIC",
    "CMD_VEL_SAFE_TOPIC",
    "BASE_COMMAND_TEXT_TOPIC",
    "BASE_STATE_TOPIC",
    "MOTOR_BOARD_STATUS_TOPIC",
    "WATCHDOG_STATE_TOPIC",
    "HEARTBEAT_TOPIC",
    "BASE_ENABLE_TOPIC",
    "BASE_ESTOP_TOPIC",
    "WATCHDOG_TRIP_TOPIC",
    "DIAG_RUN_REQUEST_TOPIC",
    "DIAG_CANCEL_REQUEST_TOPIC",
    "DIAG_STATE_TOPIC",
    "DIAG_EVENT_TOPIC",
    "DIAG_BUSY_TOPIC",
    # registry
    "TOPIC_CONTRACT",
    # helpers
    "topic_spec_to_dict",
    "get_topic_spec",
    "has_topic_spec",
    "get_topic_name",
    "get_topic_msg_type",
    "get_topic_msg_type_name",
    "get_topic_qos_key",
    "list_topic_keys",
    "list_topic_specs",
    "topic_contract_to_dict",
    "validate_topic_bindings",
    "summarize_binding_issues",
    "command_path_keys",
    "state_output_keys",
    "control_flag_keys",
    "diag_keys",
]