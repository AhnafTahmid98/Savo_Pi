#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""QoS profile helpers for Python ROS nodes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final


# =============================================================================
# QoS names
# =============================================================================
QOS_DEFAULT: Final[str] = "default"
QOS_SENSOR_DATA: Final[str] = "sensor_data"
QOS_RELIABLE: Final[str] = "reliable"
QOS_TRANSIENT_LOCAL: Final[str] = "transient_local"


# =============================================================================
# QoS config
# =============================================================================
@dataclass(frozen=True)
class QosConfig:
    name: str
    depth: int = 10
    reliability: str = "reliable"
    durability: str = "volatile"
    history: str = "keep_last"

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "depth": self.depth,
            "reliability": self.reliability,
            "durability": self.durability,
            "history": self.history,
        }


DEFAULT_QOS: Final[QosConfig] = QosConfig(
    name=QOS_DEFAULT,
    depth=10,
    reliability="reliable",
    durability="volatile",
)

SENSOR_DATA_QOS: Final[QosConfig] = QosConfig(
    name=QOS_SENSOR_DATA,
    depth=10,
    reliability="best_effort",
    durability="volatile",
)

RELIABLE_QOS: Final[QosConfig] = QosConfig(
    name=QOS_RELIABLE,
    depth=10,
    reliability="reliable",
    durability="volatile",
)

TRANSIENT_LOCAL_QOS: Final[QosConfig] = QosConfig(
    name=QOS_TRANSIENT_LOCAL,
    depth=1,
    reliability="reliable",
    durability="transient_local",
)


# =============================================================================
# Lookup helpers
# =============================================================================
def normalize_qos_name(name: str) -> str:
    value = str(name).strip().lower().replace("-", "_").replace(" ", "_")

    if not value:
        raise ValueError("QoS name cannot be empty.")

    return value


def get_qos_config(name: str) -> QosConfig:
    value = normalize_qos_name(name)

    if value == QOS_DEFAULT:
        return DEFAULT_QOS

    if value == QOS_SENSOR_DATA:
        return SENSOR_DATA_QOS

    if value == QOS_RELIABLE:
        return RELIABLE_QOS

    if value == QOS_TRANSIENT_LOCAL:
        return TRANSIENT_LOCAL_QOS

    raise KeyError(f"Unknown QoS profile: {name}")


def list_qos_profiles() -> tuple[str, ...]:
    return (
        QOS_DEFAULT,
        QOS_SENSOR_DATA,
        QOS_RELIABLE,
        QOS_TRANSIENT_LOCAL,
    )


# =============================================================================
# rclpy adapter
# =============================================================================
def to_rclpy_qos(config: QosConfig):
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


def get_rclpy_qos(name: str):
    return to_rclpy_qos(get_qos_config(name))


# =============================================================================
# Topic defaults
# =============================================================================
def qos_name_for_topic(topic: str) -> str:
    value = str(topic).strip()

    if value in ("/scan", "/savo_edge/realsense/points"):
        return QOS_SENSOR_DATA

    if value in ("/map", "/map_metadata", "/tf_static"):
        return QOS_TRANSIENT_LOCAL

    return QOS_RELIABLE


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    for name in list_qos_profiles():
        print(get_qos_config(name).to_dict())

    print(qos_name_for_topic("/scan"))
    print(qos_name_for_topic("/savo_mapping/status"))


if __name__ == "__main__":
    main()


__all__ = [
    "QOS_DEFAULT",
    "QOS_SENSOR_DATA",
    "QOS_RELIABLE",
    "QOS_TRANSIENT_LOCAL",
    "QosConfig",
    "DEFAULT_QOS",
    "SENSOR_DATA_QOS",
    "RELIABLE_QOS",
    "TRANSIENT_LOCAL_QOS",
    "normalize_qos_name",
    "get_qos_config",
    "list_qos_profiles",
    "to_rclpy_qos",
    "get_rclpy_qos",
    "qos_name_for_topic",
    "main",
]