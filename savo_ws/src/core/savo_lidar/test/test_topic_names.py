import pytest

from savo_lidar.ros.topic_contract import (
    LidarTopics,
    normalize_topic_name,
    validate_lidar_topics,
)
from savo_lidar.utils.topic_names import (
    is_absolute_topic,
    is_private_topic,
    join_topic,
    normalize_topic,
)


def test_normalize_topic_adds_leading_slash():
    assert normalize_topic("scan") == "/scan"


def test_normalize_topic_removes_duplicate_slashes():
    assert normalize_topic("//savo_lidar//health") == "/savo_lidar/health"


def test_normalize_topic_removes_trailing_slash():
    assert normalize_topic("/savo_lidar/health/") == "/savo_lidar/health"


def test_normalize_topic_rejects_empty_topic():
    with pytest.raises(ValueError):
        normalize_topic("")


def test_join_topic_combines_namespace_and_name():
    assert join_topic("/savo_lidar", "health") == "/savo_lidar/health"


def test_join_topic_rejects_empty_suffix():
    with pytest.raises(ValueError):
        join_topic("/savo_lidar", "")


def test_is_absolute_topic_detects_absolute_topic():
    assert is_absolute_topic("/scan")
    assert not is_absolute_topic("scan")


def test_is_private_topic_detects_private_topic():
    assert is_private_topic("~/scan")
    assert not is_private_topic("/scan")


def test_normalize_topic_name_adds_leading_slash():
    assert normalize_topic_name("scan") == "/scan"


def test_normalize_topic_name_rejects_empty_topic():
    with pytest.raises(ValueError):
        normalize_topic_name("")


def test_validate_lidar_topics_accepts_default_topics():
    validate_lidar_topics(LidarTopics())


def test_validate_lidar_topics_rejects_duplicate_topics():
    topics = LidarTopics(
        scan="/scan",
        state="/scan",
    )

    with pytest.raises(ValueError):
        validate_lidar_topics(topics)