# -*- coding: utf-8 -*-

"""QoS helper names and builders for Python fallback ROS nodes."""

from __future__ import annotations

from dataclasses import dataclass


QOS_COMMAND = "command"
QOS_STATUS = "status"
QOS_SENSOR = "sensor"
QOS_LATCHED = "latched"
QOS_DEFAULT = "default"


@dataclass(frozen=True)
class QosProfileSpec:
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


QOS_SPECS = {
    QOS_COMMAND: QosProfileSpec(
        name=QOS_COMMAND,
        depth=10,
        reliability="reliable",
        durability="volatile",
    ),
    QOS_STATUS: QosProfileSpec(
        name=QOS_STATUS,
        depth=10,
        reliability="reliable",
        durability="volatile",
    ),
    QOS_SENSOR: QosProfileSpec(
        name=QOS_SENSOR,
        depth=5,
        reliability="best_effort",
        durability="volatile",
    ),
    QOS_LATCHED: QosProfileSpec(
        name=QOS_LATCHED,
        depth=1,
        reliability="reliable",
        durability="transient_local",
    ),
    QOS_DEFAULT: QosProfileSpec(
        name=QOS_DEFAULT,
        depth=10,
        reliability="reliable",
        durability="volatile",
    ),
}


def get_qos_spec(name: str) -> QosProfileSpec:
    return QOS_SPECS.get(str(name).strip().lower(), QOS_SPECS[QOS_DEFAULT])


def get_qos_spec_dict(name: str) -> dict:
    return get_qos_spec(name).to_dict()


def create_ros_qos_profile(name: str):
    try:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
    except Exception as exc:
        raise RuntimeError("rclpy is required to create a ROS QoSProfile") from exc

    spec = get_qos_spec(name)

    reliability = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }[spec.reliability]

    durability = {
        "volatile": DurabilityPolicy.VOLATILE,
        "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
    }[spec.durability]

    history = {
        "keep_last": HistoryPolicy.KEEP_LAST,
    }[spec.history]

    return QoSProfile(
        history=history,
        depth=spec.depth,
        reliability=reliability,
        durability=durability,
    )


def qos_name_for_topic(topic: str) -> str:
    text = str(topic)

    if text in {
        "/cmd_vel_manual",
        "/cmd_vel_auto",
        "/cmd_vel_nav",
        "/cmd_vel_recovery",
        "/cmd_vel_mux",
        "/cmd_vel",
        "/cmd_vel_safe",
        "/savo_control/mode_cmd",
    }:
        return QOS_COMMAND

    if text in {
        "/scan",
        "/depth/min_front_m",
        "/savo_perception/range/front_ultrasonic_m",
        "/savo_perception/range/left_m",
        "/savo_perception/range/right_m",
        "/odometry/filtered",
        "/wheel/odom",
        "/imu/data",
    }:
        return QOS_SENSOR

    if (
        text.endswith("/state")
        or text.endswith("_state")
        or text.endswith("/status")
        or text.endswith("_status")
        or "status" in text
    ):
        return QOS_STATUS

    return QOS_DEFAULT


__all__ = [
    "QOS_COMMAND",
    "QOS_DEFAULT",
    "QOS_LATCHED",
    "QOS_SENSOR",
    "QOS_SPECS",
    "QOS_STATUS",
    "QosProfileSpec",
    "create_ros_qos_profile",
    "get_qos_spec",
    "get_qos_spec_dict",
    "qos_name_for_topic",
]
