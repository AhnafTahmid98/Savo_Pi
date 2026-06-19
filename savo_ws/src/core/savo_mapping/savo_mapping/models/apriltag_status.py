#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AprilTag status model for mapping-time semantic landmarks. No ROS imports."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from savo_mapping.constants import TOPIC_APRILTAG_DETECTIONS


# =============================================================================
# AprilTag observation
# =============================================================================
@dataclass(frozen=True)
class AprilTagObservation:
    tag_id: int
    family: str = "tag36h11"

    frame_id: str = ""
    map_frame: str = "map"

    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    yaw: Optional[float] = None

    confidence: float = 0.0
    label: Optional[str] = None
    timestamp_s: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "tag_id": self.tag_id,
            "family": self.family,
            "frame_id": self.frame_id,
            "map_frame": self.map_frame,
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": self.yaw,
            "confidence": self.confidence,
            "label": self.label,
            "timestamp_s": self.timestamp_s,
        }


# =============================================================================
# AprilTag detector status
# =============================================================================
@dataclass(frozen=True)
class AprilTagStatus:
    enabled: bool = False
    ok: bool = False
    stale: bool = True

    topic: str = TOPIC_APRILTAG_DETECTIONS
    msg_count: int = 0
    detection_count: int = 0
    unique_tag_count: int = 0

    last_tag_id: Optional[int] = None
    last_label: Optional[str] = None
    age_s: Optional[float] = None

    stale_timeout_s: float = 1.0
    message: str = "AprilTag detection disabled."
    timestamp_s: float = field(default_factory=time.time)

    observations: tuple[AprilTagObservation, ...] = field(default_factory=tuple)
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "ok": self.ok,
            "stale": self.stale,
            "topic": self.topic,
            "msg_count": self.msg_count,
            "detection_count": self.detection_count,
            "unique_tag_count": self.unique_tag_count,
            "last_tag_id": self.last_tag_id,
            "last_label": self.last_label,
            "age_s": self.age_s,
            "stale_timeout_s": self.stale_timeout_s,
            "message": self.message,
            "timestamp_s": self.timestamp_s,
            "observations": [
                observation.to_dict()
                for observation in self.observations
            ],
            "extra": dict(self.extra),
        }

    def to_json(self, *, indent: Optional[int] = None) -> str:
        return json.dumps(self.to_dict(), indent=indent, sort_keys=True)


# =============================================================================
# Builders
# =============================================================================
def make_apriltag_disabled_status(
    topic: str = TOPIC_APRILTAG_DETECTIONS,
) -> AprilTagStatus:
    return AprilTagStatus(
        enabled=False,
        ok=False,
        stale=True,
        topic=topic,
        message="AprilTag detection disabled.",
    )


def make_apriltag_status(
    enabled: bool,
    msg_count: int = 0,
    detection_count: int = 0,
    unique_tag_count: int = 0,
    last_tag_id: Optional[int] = None,
    last_label: Optional[str] = None,
    age_s: Optional[float] = None,
    stale_timeout_s: float = 1.0,
    topic: str = TOPIC_APRILTAG_DETECTIONS,
    observations: Optional[list[AprilTagObservation]] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> AprilTagStatus:
    if not enabled:
        return make_apriltag_disabled_status(topic=topic)

    messages = max(0, int(msg_count))
    detections = max(0, int(detection_count))
    unique_tags = max(0, int(unique_tag_count))
    timeout = max(0.0, float(stale_timeout_s))

    stale = True if age_s is None else float(age_s) > timeout
    ok = messages > 0 and detections > 0 and not stale

    if ok:
        message = "AprilTag detection OK."
    elif messages <= 0:
        message = "Waiting for AprilTag detections."
    elif detections <= 0:
        message = "AprilTag topic active, no tags detected."
    else:
        message = "AprilTag detections stale."

    return AprilTagStatus(
        enabled=True,
        ok=ok,
        stale=stale,
        topic=str(topic),
        msg_count=messages,
        detection_count=detections,
        unique_tag_count=unique_tags,
        last_tag_id=last_tag_id,
        last_label=last_label,
        age_s=age_s,
        stale_timeout_s=timeout,
        message=message,
        observations=tuple(observations or ()),
        extra=dict(extra or {}),
    )


def apriltag_status_from_dict(data: Dict[str, Any]) -> AprilTagStatus:
    observations: list[AprilTagObservation] = []

    for item in data.get("observations", []):
        if not isinstance(item, dict):
            continue

        observations.append(
            AprilTagObservation(
                tag_id=int(item.get("tag_id", -1)),
                family=str(item.get("family", "tag36h11")),
                frame_id=str(item.get("frame_id", "")),
                map_frame=str(item.get("map_frame", "map")),
                x=item.get("x"),
                y=item.get("y"),
                z=item.get("z"),
                yaw=item.get("yaw"),
                confidence=float(item.get("confidence", 0.0)),
                label=item.get("label"),
                timestamp_s=float(item.get("timestamp_s", time.time())),
            )
        )

    return AprilTagStatus(
        enabled=bool(data.get("enabled", False)),
        ok=bool(data.get("ok", False)),
        stale=bool(data.get("stale", True)),
        topic=str(data.get("topic", TOPIC_APRILTAG_DETECTIONS)),
        msg_count=max(0, int(data.get("msg_count", 0))),
        detection_count=max(0, int(data.get("detection_count", 0))),
        unique_tag_count=max(0, int(data.get("unique_tag_count", 0))),
        last_tag_id=data.get("last_tag_id"),
        last_label=data.get("last_label"),
        age_s=data.get("age_s"),
        stale_timeout_s=max(0.0, float(data.get("stale_timeout_s", 1.0))),
        message=str(data.get("message", "")),
        timestamp_s=float(data.get("timestamp_s", time.time())),
        observations=tuple(observations),
        extra=dict(data.get("extra", {})),
    )


# =============================================================================
# CLI entry
# =============================================================================
def main() -> None:
    observation = AprilTagObservation(
        tag_id=21,
        family="tag36h11",
        frame_id="camera_color_optical_frame",
        x=4.25,
        y=8.70,
        z=0.0,
        yaw=1.57,
        confidence=0.92,
        label="A201",
    )

    print(make_apriltag_disabled_status().to_json(indent=2))
    print(
        make_apriltag_status(
            enabled=True,
            msg_count=4,
            detection_count=1,
            unique_tag_count=1,
            last_tag_id=21,
            last_label="A201",
            age_s=0.05,
            observations=[observation],
        ).to_json(indent=2)
    )


if __name__ == "__main__":
    main()


__all__ = [
    "AprilTagObservation",
    "AprilTagStatus",
    "make_apriltag_disabled_status",
    "make_apriltag_status",
    "apriltag_status_from_dict",
    "main",
]