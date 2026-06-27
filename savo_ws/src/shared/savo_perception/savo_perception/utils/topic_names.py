# -*- coding: utf-8 -*-
"""Topic name constants and helpers for savo_perception. Zero ROS dependency."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional


try:
    from savo_perception.constants import (
        TOPIC_CMD_VEL as _TOPIC_CMD_VEL,
        TOPIC_CMD_VEL_SAFE as _TOPIC_CMD_VEL_SAFE,
        TOPIC_DEPTH_FRONT_M as _TOPIC_DEPTH_FRONT_M,
        TOPIC_TOF_LEFT_M as _TOPIC_TOF_LEFT_M,
        TOPIC_TOF_RIGHT_M as _TOPIC_TOF_RIGHT_M,
        TOPIC_ULTRASONIC_FRONT_M as _TOPIC_ULTRASONIC_FRONT_M,
        TOPIC_SAFETY_STOP as _TOPIC_SAFETY_STOP,
        TOPIC_SAFETY_SLOWDOWN_FACTOR as _TOPIC_SAFETY_SLOWDOWN_FACTOR,
        TOPIC_SAVO_PERCEPTION_RANGE_HEALTH as _TOPIC_RANGE_HEALTH,
        TOPIC_SAVO_PERCEPTION_SAFETY_STATE as _TOPIC_SAFETY_STATE,
        TOPIC_SAVO_PERCEPTION_DASHBOARD as _TOPIC_DASHBOARD,
        TOPIC_SAVO_PERCEPTION_DASHBOARD_TEXT as _TOPIC_DASHBOARD_TEXT,
    )
    _HAS_PACKAGE_CONSTANTS = True
except Exception:
    _HAS_PACKAGE_CONSTANTS = False
    _TOPIC_CMD_VEL = "/cmd_vel"
    _TOPIC_CMD_VEL_SAFE = "/cmd_vel_safe"
    _TOPIC_DEPTH_FRONT_M = "/depth/min_front_m"
    _TOPIC_TOF_LEFT_M = "/savo_perception/range/left_m"
    _TOPIC_TOF_RIGHT_M = "/savo_perception/range/right_m"
    _TOPIC_ULTRASONIC_FRONT_M = "/savo_perception/range/front_ultrasonic_m"
    _TOPIC_SAFETY_STOP = "/safety/stop"
    _TOPIC_SAFETY_SLOWDOWN_FACTOR = "/safety/slowdown_factor"
    _TOPIC_RANGE_HEALTH = "/savo_perception/range_health"
    _TOPIC_SAFETY_STATE = "/savo_perception/safety_state"
    _TOPIC_DASHBOARD = "/savo_perception/dashboard"
    _TOPIC_DASHBOARD_TEXT = "/savo_perception/dashboard_text"


def ensure_leading_slash(name: str) -> str:
    """Return topic with a leading '/'."""
    s = str(name or "").strip()
    if not s:
        return "/"
    return s if s.startswith("/") else f"/{s}"


def strip_trailing_slash(name: str) -> str:
    """Remove trailing slash unless the topic is root ('/')."""
    s = ensure_leading_slash(name)
    if s != "/" and s.endswith("/"):
        return s[:-1]
    return s


def normalize_topic_name(name: str) -> str:
    """Normalize a topic string."""
    s = ensure_leading_slash(name)
    while "//" in s:
        s = s.replace("//", "/")
    return strip_trailing_slash(s)


def join_topic(*parts: str) -> str:
    """Join topic path fragments safely."""
    cleaned = []
    for part in parts:
        if part is None:
            continue
        text = str(part).strip()
        if not text:
            continue
        cleaned.append(text.strip("/"))

    if not cleaned:
        return "/"

    return normalize_topic_name("/" + "/".join(cleaned))


CMD_VEL = _TOPIC_CMD_VEL
CMD_VEL_SAFE = _TOPIC_CMD_VEL_SAFE

DEPTH_FRONT_M = _TOPIC_DEPTH_FRONT_M
TOF_LEFT_M = _TOPIC_TOF_LEFT_M
TOF_RIGHT_M = _TOPIC_TOF_RIGHT_M
ULTRASONIC_FRONT_M = _TOPIC_ULTRASONIC_FRONT_M

SAFETY_STOP = _TOPIC_SAFETY_STOP
SAFETY_SLOWDOWN_FACTOR = _TOPIC_SAFETY_SLOWDOWN_FACTOR

SAVO_PERCEPTION_RANGE_HEALTH = _TOPIC_RANGE_HEALTH
SAVO_PERCEPTION_SAFETY_STATE = _TOPIC_SAFETY_STATE
SAVO_PERCEPTION_DASHBOARD = _TOPIC_DASHBOARD
SAVO_PERCEPTION_DASHBOARD_TEXT = _TOPIC_DASHBOARD_TEXT

SAVO_PERCEPTION_HEARTBEAT = "/savo_perception/heartbeat"
SAVO_PERCEPTION_DIAG = "/savo_perception/diag"
SAVO_PERCEPTION_SENSOR_STATUS = "/savo_perception/sensor_status"

ODOM_FILTERED = "/odometry/filtered"
VO_ODOM = "/vo/odom"


@dataclass(frozen=True)
class RangeSensorTopics:
    """Topic contract for range sensor publishers."""
    depth_front_m: str = DEPTH_FRONT_M
    tof_left_m: str = TOF_LEFT_M
    tof_right_m: str = TOF_RIGHT_M
    ultrasonic_front_m: str = ULTRASONIC_FRONT_M


@dataclass(frozen=True)
class PerceptionSafetyTopics:
    """Topic contract for perception safety nodes."""
    cmd_vel_in: str = CMD_VEL
    cmd_vel_out: str = CMD_VEL_SAFE
    safety_stop: str = SAFETY_STOP
    slowdown_factor: str = SAFETY_SLOWDOWN_FACTOR
    safety_state: str = SAVO_PERCEPTION_SAFETY_STATE


@dataclass(frozen=True)
class PerceptionStatusTopics:
    """Topic contract for perception status and diagnostics."""
    range_health: str = SAVO_PERCEPTION_RANGE_HEALTH
    safety_state: str = SAVO_PERCEPTION_SAFETY_STATE
    dashboard: str = SAVO_PERCEPTION_DASHBOARD
    dashboard_text: str = SAVO_PERCEPTION_DASHBOARD_TEXT
    heartbeat: str = SAVO_PERCEPTION_HEARTBEAT
    diag: str = SAVO_PERCEPTION_DIAG
    sensor_status: str = SAVO_PERCEPTION_SENSOR_STATUS


RANGE_SENSOR_TOPICS = RangeSensorTopics()
PERCEPTION_SAFETY_TOPICS = PerceptionSafetyTopics()
PERCEPTION_STATUS_TOPICS = PerceptionStatusTopics()


def topic_in_namespace(topic: str, namespace: Optional[str]) -> str:
    """
    Apply a ROS namespace to an absolute topic.

    Empty namespace keeps the original topic.
    """
    t = normalize_topic_name(topic)
    if not namespace:
        return t

    ns = normalize_topic_name(namespace)
    if ns == "/":
        return t

    return join_topic(ns, t.lstrip("/"))


def remap_topic_dict(
    topic_map: Mapping[str, str],
    namespace: Optional[str] = None,
    overrides: Optional[Mapping[str, str]] = None,
) -> Dict[str, str]:
    """Build a resolved topic dict from a logical-name->topic map."""
    ov = dict(overrides or {})
    out: Dict[str, str] = {}

    for key, topic in topic_map.items():
        if key in ov:
            out[key] = normalize_topic_name(ov[key])
        else:
            out[key] = topic_in_namespace(topic, namespace)

    return out


def prefixed_topic_bundle(prefix: str) -> Dict[str, str]:
    """Return common perception topics under a custom prefix."""
    p = normalize_topic_name(prefix)
    return {
        "heartbeat": join_topic(p, "heartbeat"),
        "diag": join_topic(p, "diag"),
        "sensor_status": join_topic(p, "sensor_status"),
        "range_health": join_topic(p, "range_health"),
        "safety_state": join_topic(p, "safety_state"),
        "dashboard": join_topic(p, "dashboard"),
        "dashboard_text": join_topic(p, "dashboard_text"),
    }


DEFAULT_RANGE_TOPIC_MAP: Dict[str, str] = {
    "depth_front_topic": DEPTH_FRONT_M,
    "tof_left_topic": TOF_LEFT_M,
    "tof_right_topic": TOF_RIGHT_M,
    "ultrasonic_front_topic": ULTRASONIC_FRONT_M,
}

DEFAULT_SAFETY_TOPIC_MAP: Dict[str, str] = {
    "cmd_in": CMD_VEL,
    "cmd_out": CMD_VEL_SAFE,
    "stop": SAFETY_STOP,
    "slowdown_factor": SAFETY_SLOWDOWN_FACTOR,
    "safety_state": SAVO_PERCEPTION_SAFETY_STATE,
}

DEFAULT_STATUS_TOPIC_MAP: Dict[str, str] = {
    "range_health": SAVO_PERCEPTION_RANGE_HEALTH,
    "safety_state": SAVO_PERCEPTION_SAFETY_STATE,
    "dashboard": SAVO_PERCEPTION_DASHBOARD,
    "dashboard_text": SAVO_PERCEPTION_DASHBOARD_TEXT,
    "heartbeat": SAVO_PERCEPTION_HEARTBEAT,
    "diag": SAVO_PERCEPTION_DIAG,
    "sensor_status": SAVO_PERCEPTION_SENSOR_STATUS,
}


def validate_topic_names(topics: Iterable[str]) -> bool:
    """Basic topic-name sanity check."""
    for topic in topics:
        name = normalize_topic_name(topic)
        if not name or not name.startswith("/"):
            return False
        if " " in name:
            return False
    return True


__all__ = [
    "ensure_leading_slash",
    "strip_trailing_slash",
    "normalize_topic_name",
    "join_topic",
    "topic_in_namespace",
    "remap_topic_dict",
    "prefixed_topic_bundle",
    "validate_topic_names",
    "CMD_VEL",
    "CMD_VEL_SAFE",
    "DEPTH_FRONT_M",
    "TOF_LEFT_M",
    "TOF_RIGHT_M",
    "ULTRASONIC_FRONT_M",
    "SAFETY_STOP",
    "SAFETY_SLOWDOWN_FACTOR",
    "SAVO_PERCEPTION_RANGE_HEALTH",
    "SAVO_PERCEPTION_SAFETY_STATE",
    "SAVO_PERCEPTION_DASHBOARD",
    "SAVO_PERCEPTION_DASHBOARD_TEXT",
    "SAVO_PERCEPTION_HEARTBEAT",
    "SAVO_PERCEPTION_DIAG",
    "SAVO_PERCEPTION_SENSOR_STATUS",
    "ODOM_FILTERED",
    "VO_ODOM",
    "RangeSensorTopics",
    "PerceptionSafetyTopics",
    "PerceptionStatusTopics",
    "RANGE_SENSOR_TOPICS",
    "PERCEPTION_SAFETY_TOPICS",
    "PERCEPTION_STATUS_TOPICS",
    "DEFAULT_RANGE_TOPIC_MAP",
    "DEFAULT_SAFETY_TOPIC_MAP",
    "DEFAULT_STATUS_TOPIC_MAP",
]