"""Utility helpers shared by Robot Savo LiDAR modules."""

from .clamp import (
    clamp_float,
    clamp_int,
    finite_or_default,
    positive_or_default,
    ratio_or_default,
)
from .diagnostics import (
    json_health,
    json_status,
    make_health_payload,
    make_status_payload,
    payload_to_json,
)
from .logging import (
    fault_message,
    hardware_message,
    kv,
    node_start_message,
    node_stop_message,
    status_message,
)
from .qos import (
    health_qos,
    heartbeat_qos,
    scan_qos,
    service_command_qos,
    status_qos,
    watchdog_qos,
)
from .ratekeeper import RateKeeper
from .timing import RateTracker, elapsed_s, is_stale, monotonic_now_s, wall_time_s
from .topic_names import (
    is_absolute_topic,
    is_private_topic,
    join_topic,
    normalize_topic,
)

__all__ = [
    "RateKeeper",
    "RateTracker",
    "clamp_float",
    "clamp_int",
    "elapsed_s",
    "fault_message",
    "finite_or_default",
    "hardware_message",
    "health_qos",
    "heartbeat_qos",
    "is_absolute_topic",
    "is_private_topic",
    "is_stale",
    "join_topic",
    "json_health",
    "json_status",
    "kv",
    "make_health_payload",
    "make_status_payload",
    "monotonic_now_s",
    "node_start_message",
    "node_stop_message",
    "normalize_topic",
    "payload_to_json",
    "positive_or_default",
    "ratio_or_default",
    "scan_qos",
    "service_command_qos",
    "status_message",
    "status_qos",
    "wall_time_s",
    "watchdog_qos",
]