#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS topic contract for Robot Savo localization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from savo_localization.constants import (
    DEFAULT_ENCODER_STATE_TOPIC,
    DEFAULT_EKF_HEALTH_TOPIC,
    DEFAULT_FILTERED_ODOM_TOPIC,
    DEFAULT_IMU_STATE_TOPIC,
    DEFAULT_IMU_TOPIC,
    DEFAULT_STATE_SUMMARY_TOPIC,
    DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    DEFAULT_WHEEL_ODOM_TOPIC,
)


@dataclass(frozen=True)
class LocalizationTopics:
    imu: str = DEFAULT_IMU_TOPIC
    wheel_odom: str = DEFAULT_WHEEL_ODOM_TOPIC
    filtered_odom: str = DEFAULT_FILTERED_ODOM_TOPIC

    imu_state: str = DEFAULT_IMU_STATE_TOPIC
    encoder_state: str = DEFAULT_ENCODER_STATE_TOPIC
    wheel_odom_state: str = DEFAULT_WHEEL_ODOM_STATE_TOPIC
    ekf_health: str = DEFAULT_EKF_HEALTH_TOPIC
    state_summary: str = DEFAULT_STATE_SUMMARY_TOPIC


DEFAULT_TOPICS: Final[LocalizationTopics] = LocalizationTopics()


def normalize_topic_name(topic: str) -> str:
    topic = str(topic).strip()

    if not topic:
        raise ValueError("Topic name cannot be empty.")

    while "//" in topic:
        topic = topic.replace("//", "/")

    if topic.startswith("~/"):
        return topic.rstrip("/")

    if not topic.startswith("/"):
        topic = f"/{topic}"

    if len(topic) > 1:
        topic = topic.rstrip("/")

    return topic


def get_default_topics() -> LocalizationTopics:
    return DEFAULT_TOPICS


def validate_localization_topics(topics: LocalizationTopics) -> None:
    values = [
        topics.imu,
        topics.wheel_odom,
        topics.filtered_odom,
        topics.imu_state,
        topics.encoder_state,
        topics.wheel_odom_state,
        topics.ekf_health,
        topics.state_summary,
    ]

    normalized = [normalize_topic_name(value) for value in values]

    if len(set(normalized)) != len(normalized):
        raise ValueError("Localization topic contract contains duplicate topics.")


def make_localization_topics(
    *,
    imu: str = DEFAULT_IMU_TOPIC,
    wheel_odom: str = DEFAULT_WHEEL_ODOM_TOPIC,
    filtered_odom: str = DEFAULT_FILTERED_ODOM_TOPIC,
    imu_state: str = DEFAULT_IMU_STATE_TOPIC,
    encoder_state: str = DEFAULT_ENCODER_STATE_TOPIC,
    wheel_odom_state: str = DEFAULT_WHEEL_ODOM_STATE_TOPIC,
    ekf_health: str = DEFAULT_EKF_HEALTH_TOPIC,
    state_summary: str = DEFAULT_STATE_SUMMARY_TOPIC,
) -> LocalizationTopics:
    topics = LocalizationTopics(
        imu=normalize_topic_name(imu),
        wheel_odom=normalize_topic_name(wheel_odom),
        filtered_odom=normalize_topic_name(filtered_odom),
        imu_state=normalize_topic_name(imu_state),
        encoder_state=normalize_topic_name(encoder_state),
        wheel_odom_state=normalize_topic_name(wheel_odom_state),
        ekf_health=normalize_topic_name(ekf_health),
        state_summary=normalize_topic_name(state_summary),
    )

    validate_localization_topics(topics)
    return topics