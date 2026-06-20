#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo mapping topic contracts."""

from __future__ import annotations

import pytest

from savo_mapping.ros.topic_contract import (
    DEFAULT_TOPIC_CONTRACT,
    TopicSpec,
    build_default_topic_contract,
    get_optional_topic_names,
    get_publication_topic_names,
    get_required_topic_names,
    get_subscription_topic_names,
    validate_contract_topics,
)
from savo_mapping.utils.topic_names import (
    DEFAULT_TOPIC_GROUPS,
    get_all_mapping_topics,
    get_topic,
    join_topic,
    normalize_topic,
    topic_basename,
    topic_namespace,
    validate_topics,
)


# =============================================================================
# Topic normalization
# =============================================================================
def test_normalize_topic_adds_leading_slash() -> None:
    assert normalize_topic("scan") == "/scan"
    assert normalize_topic("/scan") == "/scan"
    assert normalize_topic("savo_mapping/status") == "/savo_mapping/status"


def test_normalize_topic_rejects_empty_value() -> None:
    with pytest.raises(ValueError):
        normalize_topic("")


def test_join_topic() -> None:
    assert join_topic("savo_mapping", "ready") == "/savo_mapping/ready"
    assert join_topic("/savo_mapping/", "/status") == "/savo_mapping/status"


def test_topic_basename_and_namespace() -> None:
    assert topic_basename("/savo_mapping/status") == "status"
    assert topic_namespace("/savo_mapping/status") == "/savo_mapping"
    assert topic_namespace("/scan") == "/"


def test_validate_topics_accepts_valid_topics() -> None:
    validate_topics(
        {
            "scan": "/scan",
            "status": "/savo_mapping/status",
            "map": "/map",
        }
    )


def test_validate_topics_rejects_empty_topic() -> None:
    with pytest.raises(ValueError):
        validate_topics({"bad": ""})


# =============================================================================
# Topic groups
# =============================================================================
def test_default_topic_groups_include_core_topics() -> None:
    assert DEFAULT_TOPIC_GROUPS.core["scan"] == "/scan"
    assert DEFAULT_TOPIC_GROUPS.core["odom"] == "/odometry/filtered"
    assert DEFAULT_TOPIC_GROUPS.core["map"] == "/map"
    assert DEFAULT_TOPIC_GROUPS.transforms["tf"] == "/tf"
    assert DEFAULT_TOPIC_GROUPS.transforms["tf_static"] == "/tf_static"


def test_default_topic_groups_include_mapping_outputs() -> None:
    assert DEFAULT_TOPIC_GROUPS.outputs["ready"] == "/savo_mapping/ready"
    assert DEFAULT_TOPIC_GROUPS.outputs["status"] == "/savo_mapping/status"
    assert DEFAULT_TOPIC_GROUPS.outputs["map_quality"] == "/savo_mapping/map_quality"


def test_all_topics_preserves_duplicate_status_keys_with_group_prefixes() -> None:
    topics = get_all_mapping_topics()

    assert topics["outputs.status"] == "/savo_mapping/status"
    assert topics["edge_realsense.status"] == "/realsense/status"

    assert "status" not in topics


def test_all_topics_keeps_unique_short_keys() -> None:
    topics = get_all_mapping_topics()

    assert topics["scan"] == "/scan"
    assert topics["odom"] == "/odometry/filtered"
    assert topics["map_quality"] == "/savo_mapping/map_quality"


def test_all_topic_values_include_duplicate_topic_values() -> None:
    values = DEFAULT_TOPIC_GROUPS.all_topic_values()

    assert "/savo_mapping/status" in values
    assert "/realsense/status" in values
    assert "/scan" in values
    assert "/map" in values


def test_get_topic_returns_unique_topic() -> None:
    assert get_topic("scan") == "/scan"
    assert get_topic("map_quality") == "/savo_mapping/map_quality"


def test_get_topic_supports_group_qualified_keys() -> None:
    assert get_topic("outputs.status") == "/savo_mapping/status"
    assert get_topic("edge_realsense.status") == "/realsense/status"


def test_get_topic_rejects_ambiguous_short_key() -> None:
    with pytest.raises(KeyError):
        get_topic("status")


# =============================================================================
# Topic specs
# =============================================================================
def test_topic_spec_normalizes_topic() -> None:
    spec = TopicSpec(
        key="scan",
        topic="scan",
        msg_type="sensor_msgs/msg/LaserScan",
        required=True,
    )

    assert spec.topic == "/scan"
    assert spec.required is True


def test_topic_spec_to_dict() -> None:
    spec = TopicSpec(
        key="status",
        topic="/savo_mapping/status",
        msg_type="std_msgs/msg/String",
        required=True,
    )

    data = spec.to_dict()

    assert data["key"] == "status"
    assert data["topic"] == "/savo_mapping/status"
    assert data["msg_type"] == "std_msgs/msg/String"
    assert data["required"] is True


# =============================================================================
# Default topic contract
# =============================================================================
def test_build_default_topic_contract() -> None:
    contract = build_default_topic_contract()

    assert contract.subscriptions["scan"].topic == "/scan"
    assert contract.subscriptions["odom"].topic == "/odometry/filtered"
    assert contract.publications["ready"].topic == "/savo_mapping/ready"
    assert contract.publications["status"].topic == "/savo_mapping/status"


def test_default_topic_contract_matches_builder() -> None:
    built = build_default_topic_contract()

    assert DEFAULT_TOPIC_CONTRACT.to_dict() == built.to_dict()


def test_required_topic_names() -> None:
    topics = get_required_topic_names()

    assert topics["scan"] == "/scan"
    assert topics["odom"] == "/odometry/filtered"
    assert topics["tf"] == "/tf"
    assert topics["tf_static"] == "/tf_static"
    assert topics["ready"] == "/savo_mapping/ready"
    assert topics["status"] == "/savo_mapping/status"
    assert topics["mode"] == "/savo_mapping/mode"


def test_optional_topic_names() -> None:
    topics = get_optional_topic_names()

    assert topics["map"] == "/map"
    assert topics["map_metadata"] == "/map_metadata"
    assert topics["realsense_points"] == "/savo_edge/realsense/points"
    assert topics["apriltag_detections"] == "/apriltag/detections"


def test_publication_topic_names() -> None:
    topics = get_publication_topic_names()

    assert topics["ready"] == "/savo_mapping/ready"
    assert topics["status"] == "/savo_mapping/status"
    assert topics["mode"] == "/savo_mapping/mode"
    assert topics["map_quality"] == "/savo_mapping/map_quality"
    assert topics["exploration_status"] == "/savo_mapping/exploration_status"


def test_subscription_topic_names() -> None:
    topics = get_subscription_topic_names()

    assert topics["scan"] == "/scan"
    assert topics["odom"] == "/odometry/filtered"
    assert topics["tf"] == "/tf"
    assert topics["tf_static"] == "/tf_static"
    assert topics["realsense_status"] == "/realsense/status"


def test_contract_all_topics_contains_subscriptions_and_publications() -> None:
    all_topics = DEFAULT_TOPIC_CONTRACT.all_topics()

    assert "scan" in all_topics
    assert "odom" in all_topics
    assert "ready" in all_topics
    assert "status" in all_topics
    assert "map_quality" in all_topics


def test_contract_to_dict_contains_expected_groups() -> None:
    data = DEFAULT_TOPIC_CONTRACT.to_dict()

    assert "subscriptions" in data
    assert "optional_subscriptions" in data
    assert "publications" in data
    assert "action_clients" in data


def test_validate_contract_topics_passes() -> None:
    validate_contract_topics()