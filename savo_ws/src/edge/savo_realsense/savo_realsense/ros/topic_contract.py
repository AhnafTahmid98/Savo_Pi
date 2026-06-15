# Copyright 2026 Ahnaf Tahmid
from dataclasses import dataclass

from savo_realsense.constants import (
    DEFAULT_CAMERA_STATUS_TOPIC,
    DEFAULT_COLOR_IMAGE_TOPIC,
    DEFAULT_COLOR_INFO_TOPIC,
    DEFAULT_DEPTH_IMAGE_TOPIC,
    DEFAULT_DEPTH_INFO_TOPIC,
    DEFAULT_DIAGNOSTICS_TOPIC,
    DEFAULT_POINTCLOUD_TOPIC,
)


@dataclass(frozen=True)
class RealSenseTopics:
    color_image: str = DEFAULT_COLOR_IMAGE_TOPIC
    color_info: str = DEFAULT_COLOR_INFO_TOPIC
    depth_image: str = DEFAULT_DEPTH_IMAGE_TOPIC
    depth_info: str = DEFAULT_DEPTH_INFO_TOPIC
    pointcloud: str = DEFAULT_POINTCLOUD_TOPIC

    status: str = DEFAULT_CAMERA_STATUS_TOPIC
    diagnostics: str = DEFAULT_DIAGNOSTICS_TOPIC


DEFAULT_TOPICS = RealSenseTopics()


REQUIRED_IMAGE_TOPICS = (
    DEFAULT_TOPICS.color_image,
    DEFAULT_TOPICS.color_info,
    DEFAULT_TOPICS.depth_image,
    DEFAULT_TOPICS.depth_info,
)


OPTIONAL_NAV_TOPICS = (
    DEFAULT_TOPICS.pointcloud,
)


STATUS_TOPICS = (
    DEFAULT_TOPICS.status,
    DEFAULT_TOPICS.diagnostics,
)


def all_camera_topics(topics: RealSenseTopics = DEFAULT_TOPICS) -> tuple[str, ...]:
    return (
        topics.color_image,
        topics.color_info,
        topics.depth_image,
        topics.depth_info,
        topics.pointcloud,
    )


def required_camera_topics(topics: RealSenseTopics = DEFAULT_TOPICS) -> tuple[str, ...]:
    return (
        topics.color_image,
        topics.color_info,
        topics.depth_image,
        topics.depth_info,
    )
