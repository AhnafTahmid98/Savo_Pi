# Copyright 2026 Ahnaf Tahmid
from collections.abc import Iterable

from savo_realsense.ros.topic_contract import (
    DEFAULT_TOPICS,
    REQUIRED_IMAGE_TOPICS,
)


def missing_topics(
    available_topics: Iterable[str],
    required_topics: Iterable[str] = REQUIRED_IMAGE_TOPICS,
) -> list[str]:
    available = set(available_topics)

    return [
        topic
        for topic in required_topics
        if topic not in available
    ]


def optional_topics_present(
    available_topics: Iterable[str],
    optional_topics: Iterable[str] | None = None,
) -> list[str]:
    available = set(available_topics)
    topics = optional_topics or (DEFAULT_TOPICS.pointcloud,)

    return [
        topic
        for topic in topics
        if topic in available
    ]


def topics_available(
    available_topics: Iterable[str],
    required_topics: Iterable[str] = REQUIRED_IMAGE_TOPICS,
) -> bool:
    return not missing_topics(available_topics, required_topics)


def format_missing_topics_report(missing: list[str]) -> str:
    if not missing:
        return "All required RealSense topics are available"

    lines = ["Missing RealSense topics:"]
    lines.extend(f"- {topic}" for topic in missing)

    return "\n".join(lines)
