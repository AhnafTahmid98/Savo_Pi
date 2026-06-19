#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS QoS adapters for Robot Savo mapping Python nodes."""

from __future__ import annotations

from typing import Any

from savo_mapping.utils.qos import (
    QOS_DEFAULT,
    QOS_RELIABLE,
    QOS_SENSOR_DATA,
    QOS_TRANSIENT_LOCAL,
    QosConfig,
    get_qos_config,
    qos_name_for_topic,
)


# =============================================================================
# ROS QoS conversion
# =============================================================================
def qos_config_to_profile(config: QosConfig) -> Any:
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    reliability = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }.get(config.reliability, ReliabilityPolicy.RELIABLE)

    durability = {
        "volatile": DurabilityPolicy.VOLATILE,
        "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
    }.get(config.durability, DurabilityPolicy.VOLATILE)

    history = {
        "keep_last": HistoryPolicy.KEEP_LAST,
        "keep_all": HistoryPolicy.KEEP_ALL,
    }.get(config.history, HistoryPolicy.KEEP_LAST)

    return QoSProfile(
        history=history,
        depth=max(1, int(config.depth)),
        reliability=reliability,
        durability=durability,
    )


def get_qos_profile(name: str) -> Any:
    return qos_config_to_profile(get_qos_config(name))


def get_topic_qos_profile(topic: str) -> Any:
    return get_qos_profile(qos_name_for_topic(topic))


# =============================================================================
# Standard mapping QoS profiles
# =============================================================================
def sensor_data_qos() -> Any:
    return get_qos_profile(QOS_SENSOR_DATA)


def reliable_qos() -> Any:
    return get_qos_profile(QOS_RELIABLE)


def transient_local_qos() -> Any:
    return get_qos_profile(QOS_TRANSIENT_LOCAL)


def default_qos() -> Any:
    return get_qos_profile(QOS_DEFAULT)


# =============================================================================
# Topic-specific helpers
# =============================================================================
def scan_qos() -> Any:
    return sensor_data_qos()


def pointcloud_qos() -> Any:
    return sensor_data_qos()


def map_qos() -> Any:
    return transient_local_qos()


def map_metadata_qos() -> Any:
    return transient_local_qos()


def status_qos() -> Any:
    return reliable_qos()


def tf_static_qos() -> Any:
    return transient_local_qos()


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    for topic in (
        "/scan",
        "/map",
        "/map_metadata",
        "/savo_edge/realsense/points",
        "/savo_mapping/status",
    ):
        print(f"{topic}: {qos_name_for_topic(topic)}")


if __name__ == "__main__":
    main()


__all__ = [
    "qos_config_to_profile",
    "get_qos_profile",
    "get_topic_qos_profile",
    "sensor_data_qos",
    "reliable_qos",
    "transient_local_qos",
    "default_qos",
    "scan_qos",
    "pointcloud_qos",
    "map_qos",
    "map_metadata_qos",
    "status_qos",
    "tf_static_qos",
    "main",
]