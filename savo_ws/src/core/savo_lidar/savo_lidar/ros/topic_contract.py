# -*- coding: utf-8 -*-
"""ROS topic contract for LiDAR nodes."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from savo_lidar.constants import (
    DEFAULT_FILTERED_SCAN_TOPIC,
    DEFAULT_HEALTH_TOPIC,
    DEFAULT_HEARTBEAT_TOPIC,
    DEFAULT_SCAN_QUALITY_TOPIC,
    DEFAULT_SCAN_TOPIC,
    DEFAULT_STATE_TOPIC,
    DEFAULT_WATCHDOG_STATE_TOPIC,
)


@dataclass(frozen=True)
class LidarTopics:
    scan: str = DEFAULT_SCAN_TOPIC
    filtered_scan: str = DEFAULT_FILTERED_SCAN_TOPIC

    state: str = DEFAULT_STATE_TOPIC
    health: str = DEFAULT_HEALTH_TOPIC
    scan_quality: str = DEFAULT_SCAN_QUALITY_TOPIC
    heartbeat: str = DEFAULT_HEARTBEAT_TOPIC
    watchdog_state: str = DEFAULT_WATCHDOG_STATE_TOPIC
    state_summary: str = "/savo_lidar/state_summary"

    front_sector: str = "/savo_lidar/sector_scan/front"
    left_sector: str = "/savo_lidar/sector_scan/left"
    right_sector: str = "/savo_lidar/sector_scan/right"
    back_sector: str = "/savo_lidar/sector_scan/back"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


DEFAULT_TOPICS = LidarTopics()


def get_default_topics() -> LidarTopics:
    return DEFAULT_TOPICS


def normalize_topic_name(topic: str) -> str:
    value = str(topic).strip()

    if not value:
        raise ValueError("Topic name cannot be empty")

    if value.startswith("~"):
        return value

    if not value.startswith("/"):
        value = f"/{value}"

    while "//" in value:
        value = value.replace("//", "/")

    if len(value) > 1:
        value = value.rstrip("/")

    return value


def validate_lidar_topics(topics: LidarTopics) -> None:
    seen: dict[str, str] = {}

    for field_name, topic in topics.to_dict().items():
        normalized = normalize_topic_name(topic)

        if normalized in seen:
            raise ValueError(
                f"Duplicate LiDAR topic '{normalized}' used by "
                f"'{seen[normalized]}' and '{field_name}'"
            )

        seen[normalized] = field_name


__all__ = [
    "DEFAULT_TOPICS",
    "LidarTopics",
    "get_default_topics",
    "normalize_topic_name",
    "validate_lidar_topics",
]
