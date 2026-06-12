"""ROS topic contract for the Robot Savo LiDAR package."""

from dataclasses import dataclass

from savo_lidar.constants import (
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
    state: str = DEFAULT_STATE_TOPIC
    health: str = DEFAULT_HEALTH_TOPIC
    scan_quality: str = DEFAULT_SCAN_QUALITY_TOPIC
    heartbeat: str = DEFAULT_HEARTBEAT_TOPIC
    watchdog_state: str = DEFAULT_WATCHDOG_STATE_TOPIC
    front_sector: str = "/savo_lidar/front_sector"


DEFAULT_TOPICS = LidarTopics()


def get_default_topics() -> LidarTopics:
    return DEFAULT_TOPICS


def normalize_topic_name(topic: str) -> str:
    topic = str(topic).strip()

    if not topic:
        raise ValueError("Topic name cannot be empty.")

    if not topic.startswith("/"):
        topic = f"/{topic}"

    return topic


def validate_lidar_topics(topics: LidarTopics) -> None:
    seen: dict[str, str] = {}

    for field_name, topic in topics.__dict__.items():
        normalized = normalize_topic_name(topic)

        if normalized in seen:
            raise ValueError(
                f"Duplicate LiDAR topic '{normalized}' used by "
                f"'{seen[normalized]}' and '{field_name}'."
            )

        seen[normalized] = field_name