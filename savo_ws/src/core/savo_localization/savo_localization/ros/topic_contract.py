#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS topic contract for Robot Savo localization."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

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


def normalize_topic_name(topic: str | None) -> str:
    if topic is None:
        return ""

    value = str(topic).strip()

    if not value:
        return ""

    if not value.startswith("/"):
        value = f"/{value}"

    while "//" in value:
        value = value.replace("//", "/")

    return value


def topic_name_valid(topic: str | None) -> bool:
    if topic is None:
        return False

    value = str(topic).strip()

    return bool(value) and value.startswith("/")


def require_absolute_topic(topic: str | None) -> str:
    if topic is None:
        raise ValueError("topic cannot be None")

    value = str(topic).strip()

    if not value:
        raise ValueError("topic cannot be empty")

    if not value.startswith("/"):
        raise ValueError(f"topic must be absolute and start with '/': {topic!r}")

    return value


@dataclass(frozen=True)
class TopicSpec:
    name: str
    topic: str
    message_type: str
    owner: str
    required: bool = True

    def __post_init__(self) -> None:
        require_absolute_topic(self.topic)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TopicContract:
    specs: dict[str, TopicSpec]

    def __post_init__(self) -> None:
        for key, spec in self.specs.items():
            if key != spec.name:
                raise ValueError(
                    f"topic spec key {key!r} does not match spec name {spec.name!r}"
                )

            require_absolute_topic(spec.topic)

    def spec(self, name: str) -> TopicSpec:
        if name not in self.specs:
            raise KeyError(name)

        return self.specs[name]

    def topic(self, name: str) -> str:
        return self.spec(name).topic

    def names(self) -> tuple[str, ...]:
        return tuple(self.specs.keys())

    def required_specs(self) -> dict[str, TopicSpec]:
        return {
            name: spec
            for name, spec in self.specs.items()
            if spec.required
        }

    def optional_specs(self) -> dict[str, TopicSpec]:
        return {
            name: spec
            for name, spec in self.specs.items()
            if not spec.required
        }

    def to_dict(self) -> dict[str, dict[str, Any]]:
        return {
            name: spec.to_dict()
            for name, spec in self.specs.items()
        }


@dataclass(frozen=True)
class LocalizationTopics:
    imu: str = DEFAULT_IMU_TOPIC
    imu_state: str = DEFAULT_IMU_STATE_TOPIC
    wheel_odom: str = DEFAULT_WHEEL_ODOM_TOPIC
    wheel_odom_state: str = DEFAULT_WHEEL_ODOM_STATE_TOPIC
    filtered_odom: str = DEFAULT_FILTERED_ODOM_TOPIC
    ekf_state: str = DEFAULT_EKF_STATE_TOPIC
    ekf_health: str = DEFAULT_EKF_HEALTH_TOPIC
    state_summary: str = DEFAULT_STATE_SUMMARY_TOPIC
    diagnostics: str = DEFAULT_DIAGNOSTICS_TOPIC

    def to_dict(self) -> dict[str, str]:
        return {
            "imu": self.imu,
            "imu_state": self.imu_state,
            "wheel_odom": self.wheel_odom,
            "wheel_odom_state": self.wheel_odom_state,
            "filtered_odom": self.filtered_odom,
            "ekf_state": self.ekf_state,
            "ekf_health": self.ekf_health,
            "state_summary": self.state_summary,
            "diagnostics": self.diagnostics,
        }


def make_localization_topics(
    *,
    imu: str = DEFAULT_IMU_TOPIC,
    imu_state: str = DEFAULT_IMU_STATE_TOPIC,
    wheel_odom: str = DEFAULT_WHEEL_ODOM_TOPIC,
    wheel_odom_state: str = DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    filtered_odom: str = DEFAULT_FILTERED_ODOM_TOPIC,
    ekf_state: str = DEFAULT_EKF_STATE_TOPIC,
    ekf_health: str = DEFAULT_EKF_HEALTH_TOPIC,
    state_summary: str = DEFAULT_STATE_SUMMARY_TOPIC,
    diagnostics: str = DEFAULT_DIAGNOSTICS_TOPIC,
) -> LocalizationTopics:
    topics = LocalizationTopics(
        imu=normalize_topic_name(imu),
        imu_state=normalize_topic_name(imu_state),
        wheel_odom=normalize_topic_name(wheel_odom),
        wheel_odom_state=normalize_topic_name(wheel_odom_state),
        filtered_odom=normalize_topic_name(filtered_odom),
        ekf_state=normalize_topic_name(ekf_state),
        ekf_health=normalize_topic_name(ekf_health),
        state_summary=normalize_topic_name(state_summary),
        diagnostics=normalize_topic_name(diagnostics),
    )

    validate_localization_topics(topics)
    return topics


def get_default_topics() -> LocalizationTopics:
    return make_localization_topics()


def validate_localization_topics(topics: LocalizationTopics) -> bool:
    for name, topic in topics.to_dict().items():
        try:
            require_absolute_topic(topic)
        except ValueError as exc:
            raise ValueError(f"{name}: {exc}") from exc

    return True


def get_default_topic_contract() -> TopicContract:
    topics = get_default_topics()

    return TopicContract(
        specs={
            "imu": TopicSpec(
                name="imu",
                topic=topics.imu,
                message_type="sensor_msgs/msg/Imu",
                owner="imu_node",
            ),
            "imu_state": TopicSpec(
                name="imu_state",
                topic=topics.imu_state,
                message_type="std_msgs/msg/String",
                owner="imu_node",
            ),
            "wheel_odom": TopicSpec(
                name="wheel_odom",
                topic=topics.wheel_odom,
                message_type="nav_msgs/msg/Odometry",
                owner="wheel_odom_node",
            ),
            "wheel_odom_state": TopicSpec(
                name="wheel_odom_state",
                topic=topics.wheel_odom_state,
                message_type="std_msgs/msg/String",
                owner="wheel_odom_node",
            ),
            "filtered_odom": TopicSpec(
                name="filtered_odom",
                topic=topics.filtered_odom,
                message_type="nav_msgs/msg/Odometry",
                owner="ekf_filter_node",
            ),
            "ekf_state": TopicSpec(
                name="ekf_state",
                topic=topics.ekf_state,
                message_type="std_msgs/msg/String",
                owner="ekf_state_publisher_node",
            ),
            "ekf_health": TopicSpec(
                name="ekf_health",
                topic=topics.ekf_health,
                message_type="diagnostic_msgs/msg/DiagnosticArray",
                owner="localization_health_node",
            ),
            "state_summary": TopicSpec(
                name="state_summary",
                topic=topics.state_summary,
                message_type="std_msgs/msg/String",
                owner="localization_health_node",
            ),
            "diagnostics": TopicSpec(
                name="diagnostics",
                topic=topics.diagnostics,
                message_type="diagnostic_msgs/msg/DiagnosticArray",
                owner="localization_health_node",
                required=False,
            ),
        }
    )


LOCALIZATION_TOPIC_CONTRACT = get_default_topic_contract()

DEFAULT_TOPICS = get_default_topics().to_dict()
DEFAULT_TOPIC_SPECS = LOCALIZATION_TOPIC_CONTRACT.specs

REQUIRED_TOPICS = {
    name: spec.topic
    for name, spec in LOCALIZATION_TOPIC_CONTRACT.required_specs().items()
}

OPTIONAL_TOPICS = {
    name: spec.topic
    for name, spec in LOCALIZATION_TOPIC_CONTRACT.optional_specs().items()
}


def get_topic(name: str) -> str:
    return LOCALIZATION_TOPIC_CONTRACT.topic(name)


def get_topic_spec(name: str) -> TopicSpec:
    return LOCALIZATION_TOPIC_CONTRACT.spec(name)


def get_topic_names() -> tuple[str, ...]:
    return LOCALIZATION_TOPIC_CONTRACT.names()


def topic_contract_to_dict() -> dict[str, dict[str, Any]]:
    return LOCALIZATION_TOPIC_CONTRACT.to_dict()


__all__ = [
    "TopicSpec",
    "TopicContract",
    "LocalizationTopics",
    "LOCALIZATION_TOPIC_CONTRACT",
    "DEFAULT_TOPICS",
    "DEFAULT_TOPIC_SPECS",
    "REQUIRED_TOPICS",
    "OPTIONAL_TOPICS",
    "normalize_topic_name",
    "topic_name_valid",
    "require_absolute_topic",
    "make_localization_topics",
    "get_default_topics",
    "validate_localization_topics",
    "get_default_topic_contract",
    "get_topic",
    "get_topic_spec",
    "get_topic_names",
    "topic_contract_to_dict",
]
