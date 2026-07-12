# -*- coding: utf-8 -*-

"""QoS specifications and lazy ROS QoS builders for savo_speech."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Final

from savo_speech import constants as c


QOS_STATE: Final[str] = "state"
QOS_EVENT: Final[str] = "event"
QOS_STATUS: Final[str] = "status"
QOS_HEALTH: Final[str] = "health"
QOS_HEARTBEAT: Final[str] = "heartbeat"
QOS_DEFAULT: Final[str] = "default"


@dataclass(frozen=True)
class QosProfileSpec:
    name: str
    depth: int
    reliability: str
    durability: str
    history: str = "keep_last"

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "depth": self.depth,
            "reliability": self.reliability,
            "durability": self.durability,
            "history": self.history,
        }


QOS_SPECS: Final[dict[str, QosProfileSpec]] = {
    QOS_STATE: QosProfileSpec(
        name=QOS_STATE,
        depth=1,
        reliability="reliable",
        durability="transient_local",
    ),
    QOS_EVENT: QosProfileSpec(
        name=QOS_EVENT,
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
    QOS_HEALTH: QosProfileSpec(
        name=QOS_HEALTH,
        depth=10,
        reliability="reliable",
        durability="volatile",
    ),
    QOS_HEARTBEAT: QosProfileSpec(
        name=QOS_HEARTBEAT,
        depth=5,
        reliability="best_effort",
        durability="volatile",
    ),
    QOS_DEFAULT: QosProfileSpec(
        name=QOS_DEFAULT,
        depth=10,
        reliability="reliable",
        durability="volatile",
    ),
}


def get_qos_spec(name: str) -> QosProfileSpec:
    key = str(name).strip().lower()
    return QOS_SPECS.get(key, QOS_SPECS[QOS_DEFAULT])


def get_qos_spec_dict(name: str) -> dict:
    return get_qos_spec(name).to_dict()


def qos_name_for_topic(topic: str) -> str:
    name = str(topic)

    if name in {
        c.TOPIC_STATE,
        c.TOPIC_WAKE_STATE,
        c.TOPIC_LISTENING,
        c.TOPIC_INPUT_MUTED,
        c.TOPIC_OUTPUT_MUTED,
        c.TOPIC_TTS_GATE,
        c.TOPIC_FACE_STATE,
        c.TOPIC_LAST_ERROR,
    }:
        return QOS_STATE

    if name in {
        c.TOPIC_WAKE_WORD_DETECTED,
        c.TOPIC_TRANSCRIPT,
        c.TOPIC_TTS_STARTED,
        c.TOPIC_TTS_FINISHED,
        c.TOPIC_TTS_CANCELLED,
    }:
        return QOS_EVENT

    if name == c.TOPIC_HEALTH:
        return QOS_HEALTH

    if name == c.TOPIC_HEARTBEAT:
        return QOS_HEARTBEAT

    if name == c.TOPIC_STATUS:
        return QOS_STATUS

    return QOS_DEFAULT


def create_ros_qos_profile(name: str):
    try:
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
    except Exception as exc:
        raise RuntimeError(
            "rclpy is required to create a ROS QoSProfile"
        ) from exc

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
