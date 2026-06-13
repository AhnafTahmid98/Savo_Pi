#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tests for Robot Savo localization topic contract."""

from __future__ import annotations

import pytest

from savo_localization.constants import (
    DEFAULT_DIAGNOSTICS_TOPIC,
    DEFAULT_EKF_HEALTH_TOPIC,
    DEFAULT_EKF_STATE_TOPIC,
    DEFAULT_FILTERED_ODOM_TOPIC,
    DEFAULT_IMU_STATE_TOPIC,
    DEFAULT_IMU_TOPIC,
    DEFAULT_STATE_SUMMARY_TOPIC,
    DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    DEFAULT_WHEEL_ODOM_TOPIC,
)
from savo_localization.ros.topic_contract import (
    LOCALIZATION_TOPIC_CONTRACT,
    TopicContract,
    TopicSpec,
    get_default_topic_contract,
    get_topic,
    get_topic_names,
    require_absolute_topic,
    topic_name_valid,
)


def test_topic_spec_defaults() -> None:
    spec = TopicSpec(
        name="imu",
        topic="/imu/data",
        message_type="sensor_msgs/msg/Imu",
        owner="imu_node",
    )

    assert spec.name == "imu"
    assert spec.topic == "/imu/data"
    assert spec.message_type == "sensor_msgs/msg/Imu"
    assert spec.owner == "imu_node"
    assert spec.required is True


def test_topic_spec_to_dict() -> None:
    spec = TopicSpec(
        name="wheel_odom",
        topic="/wheel/odom",
        message_type="nav_msgs/msg/Odometry",
        owner="wheel_odom_node",
        required=True,
    )

    assert spec.to_dict() == {
        "name": "wheel_odom",
        "topic": "/wheel/odom",
        "message_type": "nav_msgs/msg/Odometry",
        "owner": "wheel_odom_node",
        "required": True,
    }


def test_topic_name_valid_accepts_absolute_topics() -> None:
    assert topic_name_valid("/imu/data")
    assert topic_name_valid("/wheel/odom")
    assert topic_name_valid("/savo_localization/health")


@pytest.mark.parametrize(
    "topic",
    [
        "",
        " ",
        "imu/data",
        "wheel/odom",
        None,
    ],
)
def test_topic_name_valid_rejects_invalid_topics(topic: str | None) -> None:
    assert not topic_name_valid(topic)


def test_require_absolute_topic_returns_valid_topic() -> None:
    assert require_absolute_topic("/imu/data") == "/imu/data"
    assert require_absolute_topic(" /wheel/odom ") == "/wheel/odom"


@pytest.mark.parametrize(
    "topic",
    [
        "",
        " ",
        "imu/data",
        "wheel/odom",
    ],
)
def test_require_absolute_topic_rejects_invalid_topic(topic: str) -> None:
    with pytest.raises(ValueError):
        require_absolute_topic(topic)


def test_default_topic_contract_contains_core_topics() -> None:
    contract = get_default_topic_contract()

    names = contract.names()

    assert "imu" in names
    assert "imu_state" in names
    assert "wheel_odom" in names
    assert "wheel_odom_state" in names
    assert "filtered_odom" in names
    assert "ekf_state" in names
    assert "ekf_health" in names
    assert "state_summary" in names
    assert "diagnostics" in names


def test_default_topic_contract_matches_constants() -> None:
    contract = get_default_topic_contract()

    assert contract.topic("imu") == DEFAULT_IMU_TOPIC
    assert contract.topic("imu_state") == DEFAULT_IMU_STATE_TOPIC
    assert contract.topic("wheel_odom") == DEFAULT_WHEEL_ODOM_TOPIC
    assert contract.topic("wheel_odom_state") == DEFAULT_WHEEL_ODOM_STATE_TOPIC
    assert contract.topic("filtered_odom") == DEFAULT_FILTERED_ODOM_TOPIC
    assert contract.topic("ekf_state") == DEFAULT_EKF_STATE_TOPIC
    assert contract.topic("ekf_health") == DEFAULT_EKF_HEALTH_TOPIC
    assert contract.topic("state_summary") == DEFAULT_STATE_SUMMARY_TOPIC
    assert contract.topic("diagnostics") == DEFAULT_DIAGNOSTICS_TOPIC


def test_localization_topic_contract_is_default_contract() -> None:
    default_contract = get_default_topic_contract()

    assert LOCALIZATION_TOPIC_CONTRACT.topic("imu") == default_contract.topic("imu")
    assert LOCALIZATION_TOPIC_CONTRACT.topic("wheel_odom") == default_contract.topic(
        "wheel_odom"
    )
    assert LOCALIZATION_TOPIC_CONTRACT.topic("filtered_odom") == default_contract.topic(
        "filtered_odom"
    )


def test_topic_contract_get_spec() -> None:
    contract = get_default_topic_contract()

    spec = contract.spec("wheel_odom")

    assert spec.name == "wheel_odom"
    assert spec.topic == DEFAULT_WHEEL_ODOM_TOPIC
    assert spec.message_type == "nav_msgs/msg/Odometry"
    assert spec.owner == "wheel_odom_node"
    assert spec.required is True


def test_topic_contract_rejects_unknown_topic_name() -> None:
    contract = get_default_topic_contract()

    with pytest.raises(KeyError):
        contract.spec("not_a_topic")

    with pytest.raises(KeyError):
        contract.topic("not_a_topic")


def test_topic_contract_to_dict() -> None:
    contract = get_default_topic_contract()
    data = contract.to_dict()

    assert isinstance(data, dict)
    assert data["imu"]["topic"] == DEFAULT_IMU_TOPIC
    assert data["wheel_odom"]["topic"] == DEFAULT_WHEEL_ODOM_TOPIC
    assert data["filtered_odom"]["topic"] == DEFAULT_FILTERED_ODOM_TOPIC


def test_get_topic_helper() -> None:
    assert get_topic("imu") == DEFAULT_IMU_TOPIC
    assert get_topic("wheel_odom") == DEFAULT_WHEEL_ODOM_TOPIC
    assert get_topic("filtered_odom") == DEFAULT_FILTERED_ODOM_TOPIC


def test_get_topic_names_helper() -> None:
    names = get_topic_names()

    assert isinstance(names, tuple)
    assert "imu" in names
    assert "wheel_odom" in names
    assert "filtered_odom" in names


def test_custom_topic_contract() -> None:
    contract = TopicContract(
        specs={
            "custom": TopicSpec(
                name="custom",
                topic="/custom/topic",
                message_type="std_msgs/msg/String",
                owner="test_node",
                required=False,
            )
        }
    )

    assert contract.topic("custom") == "/custom/topic"
    assert contract.spec("custom").required is False
    assert contract.names() == ("custom",)


def test_custom_topic_contract_validates_topics() -> None:
    with pytest.raises(ValueError):
        TopicContract(
            specs={
                "bad": TopicSpec(
                    name="bad",
                    topic="relative/topic",
                    message_type="std_msgs/msg/String",
                    owner="test_node",
                )
            }
        )


def test_robot_savo_topic_ownership() -> None:
    contract = get_default_topic_contract()

    assert contract.spec("imu").owner == "imu_node"
    assert contract.spec("imu_state").owner == "imu_node"
    assert contract.spec("wheel_odom").owner == "wheel_odom_node"
    assert contract.spec("wheel_odom_state").owner == "wheel_odom_node"
    assert contract.spec("filtered_odom").owner == "ekf_filter_node"
    assert contract.spec("ekf_state").owner == "ekf_state_publisher_node"
    assert contract.spec("ekf_health").owner == "localization_health_node"
    assert contract.spec("state_summary").owner == "localization_health_node"


def test_robot_savo_message_types() -> None:
    contract = get_default_topic_contract()

    assert contract.spec("imu").message_type == "sensor_msgs/msg/Imu"
    assert contract.spec("wheel_odom").message_type == "nav_msgs/msg/Odometry"
    assert contract.spec("filtered_odom").message_type == "nav_msgs/msg/Odometry"
    assert contract.spec("imu_state").message_type == "std_msgs/msg/String"
    assert contract.spec("wheel_odom_state").message_type == "std_msgs/msg/String"
    assert contract.spec("ekf_state").message_type == "std_msgs/msg/String"
    assert contract.spec("state_summary").message_type == "std_msgs/msg/String"