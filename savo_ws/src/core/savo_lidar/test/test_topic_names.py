# -*- coding: utf-8 -*-

import pytest

from savo_lidar.ros import (
    LidarTopics,
    get_default_topics,
    normalize_topic_name,
    validate_lidar_topics,
)
from savo_lidar.utils import (
    is_absolute_topic,
    is_private_topic,
    join_topic,
    normalize_topic,
)


def test_normalize_topic_adds_leading_slash():
    assert normalize_topic("scan") == "/scan"


def test_normalize_topic_keeps_absolute_topic():
    assert normalize_topic("/scan") == "/scan"


def test_normalize_topic_removes_trailing_slash():
    assert normalize_topic("/scan/") == "/scan"


def test_normalize_topic_collapses_duplicate_slashes():
    assert normalize_topic("//savo_lidar//health/") == "/savo_lidar/health"


def test_normalize_topic_keeps_private_topic():
    assert normalize_topic("~/state") == "~/state"


def test_normalize_topic_rejects_empty_topic():
    with pytest.raises(ValueError):
        normalize_topic("")

    with pytest.raises(ValueError):
        normalize_topic("   ")


def test_is_absolute_topic():
    assert is_absolute_topic("/scan")
    assert not is_absolute_topic("scan")
    assert not is_absolute_topic("~/scan")


def test_is_private_topic():
    assert is_private_topic("~/scan")
    assert not is_private_topic("/scan")
    assert not is_private_topic("scan")


def test_join_topic_joins_parts():
    assert join_topic("savo_lidar", "health") == "/savo_lidar/health"


def test_join_topic_ignores_empty_parts():
    assert join_topic("/savo_lidar/", "", "health/") == "/savo_lidar/health"


def test_join_topic_rejects_empty_parts():
    with pytest.raises(ValueError):
        join_topic("", "  ")


def test_normalize_topic_name_adds_leading_slash():
    assert normalize_topic_name("scan") == "/scan"


def test_normalize_topic_name_keeps_absolute_topic():
    assert normalize_topic_name("/scan") == "/scan"


def test_normalize_topic_name_cleans_duplicate_slashes():
    assert normalize_topic_name("//savo_lidar//health/") == "/savo_lidar/health"


def test_normalize_topic_name_keeps_private_topic():
    assert normalize_topic_name("~/state") == "~/state"


def test_normalize_topic_name_rejects_empty_topic():
    with pytest.raises(ValueError):
        normalize_topic_name("")


def test_default_lidar_topics_match_contract():
    topics = get_default_topics()

    assert topics.scan == "/scan"
    assert topics.filtered_scan == "/scan_filtered"
    assert topics.state == "/savo_lidar/state"
    assert topics.health == "/savo_lidar/health"
    assert topics.scan_quality == "/savo_lidar/scan_quality"
    assert topics.heartbeat == "/savo_lidar/heartbeat"
    assert topics.watchdog_state == "/savo_lidar/watchdog_state"
    assert topics.state_summary == "/savo_lidar/state_summary"
    assert topics.front_sector == "/savo_lidar/sector_scan/front"
    assert topics.left_sector == "/savo_lidar/sector_scan/left"
    assert topics.right_sector == "/savo_lidar/sector_scan/right"
    assert topics.back_sector == "/savo_lidar/sector_scan/back"


def test_lidar_topics_to_dict_contains_all_topics():
    topics = LidarTopics()
    data = topics.to_dict()

    assert data["scan"] == "/scan"
    assert data["filtered_scan"] == "/scan_filtered"
    assert data["health"] == "/savo_lidar/health"
    assert data["front_sector"] == "/savo_lidar/sector_scan/front"


def test_validate_lidar_topics_accepts_default_topics():
    validate_lidar_topics(LidarTopics())


def test_validate_lidar_topics_rejects_duplicate_topics():
    topics = LidarTopics(
        scan="/scan",
        filtered_scan="/scan",
    )

    with pytest.raises(ValueError):
        validate_lidar_topics(topics)


def test_validate_lidar_topics_detects_duplicates_after_normalization():
    topics = LidarTopics(
        health="/savo_lidar/health",
        scan_quality="savo_lidar/health",
    )

    with pytest.raises(ValueError):
        validate_lidar_topics(topics)


def test_validate_lidar_topics_accepts_private_topics():
    topics = LidarTopics(
        scan="~/scan",
        filtered_scan="~/scan_filtered",
        state="~/state",
        health="~/health",
        scan_quality="~/scan_quality",
        heartbeat="~/heartbeat",
        watchdog_state="~/watchdog_state",
        state_summary="~/state_summary",
        front_sector="~/front_sector",
        left_sector="~/left_sector",
        right_sector="~/right_sector",
        back_sector="~/back_sector",
    )

    validate_lidar_topics(topics)
