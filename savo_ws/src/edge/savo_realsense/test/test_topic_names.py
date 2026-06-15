# Copyright 2026 Ahnaf Tahmid
import pytest

from savo_realsense.ros.topic_contract import (
    DEFAULT_TOPICS,
    REQUIRED_IMAGE_TOPICS,
    all_camera_topics,
    required_camera_topics,
)
from savo_realsense.utils.topic_names import join_topic, normalize_topic_name


def test_normalize_topic_name_adds_leading_slash() -> None:
    assert normalize_topic_name("camera/color/image_raw") == "/camera/color/image_raw"


def test_normalize_topic_name_removes_duplicate_slashes() -> None:
    assert normalize_topic_name("//camera//color/image_raw") == "/camera/color/image_raw"


def test_normalize_topic_name_rejects_empty_value() -> None:
    with pytest.raises(ValueError):
        normalize_topic_name("")


def test_join_topic_builds_clean_topic() -> None:
    assert join_topic("/camera/camera", "color", "image_raw") == "/camera/camera/color/image_raw"


def test_required_camera_topics_match_contract() -> None:
    assert required_camera_topics() == REQUIRED_IMAGE_TOPICS


def test_all_camera_topics_includes_pointcloud() -> None:
    topics = all_camera_topics()

    assert DEFAULT_TOPICS.color_image in topics
    assert DEFAULT_TOPICS.depth_image in topics
    assert DEFAULT_TOPICS.pointcloud in topics
