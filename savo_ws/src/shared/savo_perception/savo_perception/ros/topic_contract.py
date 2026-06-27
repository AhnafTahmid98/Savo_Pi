#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Topic name constants, expected message types, and role annotations for savo_perception."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, List, Tuple, Type


try:
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, Float32, String
    ROS_MSGS_AVAILABLE = True
except Exception:
    ROS_MSGS_AVAILABLE = False

    class Twist:
        pass

    class Bool:
        pass

    class Float32:
        pass

    class String:
        pass


# =============================================================================
# Topic spec datamodel
# =============================================================================
@dataclass(frozen=True)
class TopicSpec:
    """
    Contract entry for one ROS topic in `savo_perception`.

    Direction is from the perspective of the whole perception package:
    'sub', 'pub', or 'pubsub'.
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
    module = getattr(msg_type, "__module__", "")
    name = getattr(msg_type, "__name__", str(msg_type))

    if ".msg" in module:
        pkg = module.split(".")[0]
        return f"{pkg}/msg/{name}"

    fallback_names = {
        "Twist": "geometry_msgs/msg/Twist",
        "Bool": "std_msgs/msg/Bool",
        "Float32": "std_msgs/msg/Float32",
        "String": "std_msgs/msg/String",
    }
    return fallback_names.get(name, name)


def topic_spec_to_dict(spec: TopicSpec) -> Dict[str, object]:
    d = asdict(spec)
    d["msg_type"] = spec.msg_type_name
    return d


# =============================================================================
# Canonical topic names
# =============================================================================
CMD_VEL_RAW_TOPIC = "/cmd_vel"
CMD_VEL_SAFE_TOPIC = "/cmd_vel_safe"

DEPTH_FRONT_M_TOPIC = "/depth/min_front_m"

TOF_LEFT_M_TOPIC = "/savo_perception/range/left_m"
TOF_RIGHT_M_TOPIC = "/savo_perception/range/right_m"
ULTRASONIC_FRONT_M_TOPIC = "/savo_perception/range/front_ultrasonic_m"

SAFETY_STOP_TOPIC = "/safety/stop"
SAFETY_SLOWDOWN_FACTOR_TOPIC = "/safety/slowdown_factor"

RANGE_HEALTH_TOPIC = "/savo_perception/range_health"
SAFETY_STATE_TOPIC = "/savo_perception/safety_state"
SENSOR_STATUS_TOPIC = "/savo_perception/sensor_status"
HEARTBEAT_TOPIC = "/savo_perception/heartbeat"

DASHBOARD_TOPIC = "/savo_perception/dashboard"
DASHBOARD_TEXT_TOPIC = "/savo_perception/dashboard_text"

DIAG_RUN_REQUEST_TOPIC = "/savo_perception/diag/run_request"
DIAG_CANCEL_REQUEST_TOPIC = "/savo_perception/diag/cancel_request"
DIAG_STATE_TOPIC = "/savo_perception/diag/state"
DIAG_EVENT_TOPIC = "/savo_perception/diag/event"
DIAG_BUSY_TOPIC = "/savo_perception/diag/busy"


# =============================================================================
# Topic contract registry
# =============================================================================
TOPIC_CONTRACT: Dict[str, TopicSpec] = {
    # -------------------------------------------------------------------------
    # Command path
    # -------------------------------------------------------------------------
    "cmd_vel_raw": TopicSpec(
        name=CMD_VEL_RAW_TOPIC,
        msg_type=Twist,
        direction="sub",
        description="Raw velocity command before perception safety gating.",
        required=True,
        qos_key="cmd_vel",
    ),
    "cmd_vel_safe": TopicSpec(
        name=CMD_VEL_SAFE_TOPIC,
        msg_type=Twist,
        direction="pub",
        description="Safety-gated velocity command published by cmd_vel_safety_gate.",
        required=True,
        qos_key="cmd_vel",
    ),

    # -------------------------------------------------------------------------
    # External perception input
    # -------------------------------------------------------------------------
    "depth_front_m": TopicSpec(
        name=DEPTH_FRONT_M_TOPIC,
        msg_type=Float32,
        direction="sub",
        description="Minimum front depth distance in meters from edge RealSense/savo_realsense.",
        required=False,
        qos_key="depth_sensor",
    ),

    # -------------------------------------------------------------------------
    # Range sensor topics
    # -------------------------------------------------------------------------
    "tof_left_m": TopicSpec(
        name=TOF_LEFT_M_TOPIC,
        msg_type=Float32,
        direction="pubsub",
        description="Left VL53L1X range in meters from TCA9548A mux channel 3.",
        required=True,
        qos_key="range_sensor",
    ),
    "tof_right_m": TopicSpec(
        name=TOF_RIGHT_M_TOPIC,
        msg_type=Float32,
        direction="pubsub",
        description="Right VL53L1X range in meters from TCA9548A mux channel 2.",
        required=True,
        qos_key="range_sensor",
    ),
    "ultrasonic_front_m": TopicSpec(
        name=ULTRASONIC_FRONT_M_TOPIC,
        msg_type=Float32,
        direction="pubsub",
        description="Front HC-SR04 ultrasonic range in meters.",
        required=True,
        qos_key="range_sensor",
    ),

    # -------------------------------------------------------------------------
    # Safety outputs
    # -------------------------------------------------------------------------
    "safety_stop": TopicSpec(
        name=SAFETY_STOP_TOPIC,
        msg_type=Bool,
        direction="pub",
        description="True when perception requires robot motion to stop.",
        required=True,
        qos_key="safety_bool",
    ),
    "safety_slowdown_factor": TopicSpec(
        name=SAFETY_SLOWDOWN_FACTOR_TOPIC,
        msg_type=Float32,
        direction="pub",
        description="Global slowdown factor from perception safety fusion.",
        required=True,
        qos_key="slowdown_factor",
    ),

    # -------------------------------------------------------------------------
    # State / health outputs
    # -------------------------------------------------------------------------
    "range_health": TopicSpec(
        name=RANGE_HEALTH_TOPIC,
        msg_type=String,
        direction="pub",
        description="Range sensor health JSON from range_health_node.",
        required=True,
        qos_key="state_string",
    ),
    "safety_state": TopicSpec(
        name=SAFETY_STATE_TOPIC,
        msg_type=String,
        direction="pub",
        description="Safety fusion state JSON from safety_stop_node.",
        required=True,
        qos_key="state_string",
    ),
    "sensor_status": TopicSpec(
        name=SENSOR_STATUS_TOPIC,
        msg_type=String,
        direction="pub",
        description="Compact perception sensor status JSON for supervision.",
        required=False,
        qos_key="state_string",
    ),
    "heartbeat": TopicSpec(
        name=HEARTBEAT_TOPIC,
        msg_type=String,
        direction="pub",
        description="Periodic perception heartbeat JSON.",
        required=True,
        qos_key="heartbeat",
    ),

    # -------------------------------------------------------------------------
    # Dashboard/debug outputs
    # -------------------------------------------------------------------------
    "dashboard": TopicSpec(
        name=DASHBOARD_TOPIC,
        msg_type=String,
        direction="pub",
        description="Perception dashboard JSON.",
        required=False,
        qos_key="state_string",
    ),
    "dashboard_text": TopicSpec(
        name=DASHBOARD_TEXT_TOPIC,
        msg_type=String,
        direction="pub",
        description="Human-readable perception dashboard summary.",
        required=False,
        qos_key="state_string",
    ),

    # -------------------------------------------------------------------------
    # Diagnostics orchestration
    # -------------------------------------------------------------------------
    "diag_run_request": TopicSpec(
        name=DIAG_RUN_REQUEST_TOPIC,
        msg_type=String,
        direction="sub",
        description="Diagnostic run request JSON.",
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
        description="Diagnostic runner state JSON.",
        required=False,
        qos_key="diag_state",
    ),
    "diag_event": TopicSpec(
        name=DIAG_EVENT_TOPIC,
        msg_type=String,
        direction="pub",
        description="Diagnostic event/log stream JSON.",
        required=False,
        qos_key="diag_event",
    ),
    "diag_busy": TopicSpec(
        name=DIAG_BUSY_TOPIC,
        msg_type=Bool,
        direction="pub",
        description="True while a perception diagnostic is running.",
        required=False,
        qos_key="diag_state",
    ),
}


# =============================================================================
# Contract query helpers
# =============================================================================
def is_ros_msgs_available() -> bool:
    return ROS_MSGS_AVAILABLE


def get_topic_spec(key: str) -> TopicSpec:
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
    out: Dict[str, Dict[str, object]] = {}
    for key, spec in TOPIC_CONTRACT.items():
        if required_only and not spec.required:
            continue
        out[key] = topic_spec_to_dict(spec)
    return out


# =============================================================================
# Validation helpers
# =============================================================================
@dataclass(frozen=True)
class TopicBindingIssue:
    key: str
    expected_name: str
    actual_name: str
    reason: str


def validate_topic_bindings(bindings: Dict[str, str]) -> List[TopicBindingIssue]:
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
    if not issues:
        return "All topic bindings match canonical savo_perception contract."

    parts = []
    for issue in issues:
        parts.append(f"{issue.key}: '{issue.actual_name}' (canon: '{issue.expected_name}')")

    return "Topic binding overrides detected -> " + "; ".join(parts)


# =============================================================================
# Group helpers
# =============================================================================
def command_path_keys() -> Tuple[str, ...]:
    return ("cmd_vel_raw", "cmd_vel_safe")


def external_input_keys() -> Tuple[str, ...]:
    return ("depth_front_m",)


def range_sensor_keys() -> Tuple[str, ...]:
    return ("tof_left_m", "tof_right_m", "ultrasonic_front_m")


def safety_output_keys() -> Tuple[str, ...]:
    return ("safety_stop", "safety_slowdown_factor")


def state_output_keys() -> Tuple[str, ...]:
    return ("range_health", "safety_state", "sensor_status", "heartbeat")


def dashboard_keys() -> Tuple[str, ...]:
    return ("dashboard", "dashboard_text")


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
    # availability
    "ROS_MSGS_AVAILABLE",
    "is_ros_msgs_available",
    # dataclasses
    "TopicSpec",
    "TopicBindingIssue",
    # constants
    "CMD_VEL_RAW_TOPIC",
    "CMD_VEL_SAFE_TOPIC",
    "DEPTH_FRONT_M_TOPIC",
    "TOF_LEFT_M_TOPIC",
    "TOF_RIGHT_M_TOPIC",
    "ULTRASONIC_FRONT_M_TOPIC",
    "SAFETY_STOP_TOPIC",
    "SAFETY_SLOWDOWN_FACTOR_TOPIC",
    "RANGE_HEALTH_TOPIC",
    "SAFETY_STATE_TOPIC",
    "SENSOR_STATUS_TOPIC",
    "HEARTBEAT_TOPIC",
    "DASHBOARD_TOPIC",
    "DASHBOARD_TEXT_TOPIC",
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
    "external_input_keys",
    "range_sensor_keys",
    "safety_output_keys",
    "state_output_keys",
    "dashboard_keys",
    "diag_keys",
]